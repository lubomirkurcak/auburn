use super::*;

mod gjk3d;

#[derive(Clone, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// # Limitation
/// Must contain the origin.
pub struct Poly3d {
    pub points: Vec<Vec3>,
}

impl From<Box3d> for Poly3d {
    fn from(value: Box3d) -> Self {
        let value = value.halfsize;
        Self {
            points: vec![
                Vec3::new(-value.x, -value.y, -value.z),
                Vec3::new(value.x, -value.y, -value.z),
                Vec3::new(value.x, value.y, -value.z),
                Vec3::new(-value.x, value.y, -value.z),
                Vec3::new(-value.x, -value.y, value.z),
                Vec3::new(value.x, -value.y, value.z),
                Vec3::new(value.x, value.y, value.z),
                Vec3::new(-value.x, value.y, value.z),
            ],
        }
    }
}

pub struct Poly3dDiff<'a, T: Transformation3d> {
    a: &'a Poly3d,
    b: &'a Poly3d,
    rel: &'a T,
}

impl<'a, T: Transformation3d> Poly3dDiff<'a, T> {
    pub fn new(a: &'a Poly3d, b: &'a Poly3d, rel: &'a T) -> Poly3dDiff<'a, T> {
        Self { a, b, rel }
    }
}

impl Poly3d {
    pub fn new(points: &[Vec3]) -> Self {
        Self {
            points: points.to_vec(),
        }
    }
    pub fn regular2d(sides: usize, radius: f32) -> Self {
        let mut points = Vec::with_capacity(sides);
        for i in 0..sides {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (sides as f32);
            points.push(radius * Vec3::new(angle.cos(), angle.sin(), 0.0))
        }
        Self { points }
    }
    pub fn uv_sphere(sides: usize, stacks: usize, radius: f32) -> Self {
        let mut points = Vec::with_capacity(sides * (stacks - 2) + 2);
        points.push(Vec3::new(0.0, 0.0, radius));
        for i in 1..stacks - 1 {
            let stack_angle = std::f32::consts::PI * (i as f32) / (stacks as f32);
            let stack_radius = radius * stack_angle.sin();
            let stack_height = radius * stack_angle.cos();
            for j in 0..sides {
                let side_angle = 2.0 * std::f32::consts::PI * (j as f32) / (sides as f32);
                points.push(Vec3::new(
                    stack_radius * side_angle.cos(),
                    stack_radius * side_angle.sin(),
                    stack_height,
                ));
            }
        }
        points.push(Vec3::new(0.0, 0.0, -radius));
        Self { points }
    }
}

impl ExtremePoint3d for Poly3d {
    fn extreme_point(&self, direction: Vec3) -> Vec3 {
        self.points
            .iter()
            .cloned()
            .fold((f32::MIN, Vec3::ZERO), |(best_score, best_p), p| {
                let score = direction.dot(p);
                if score > best_score {
                    (score, p)
                } else {
                    (best_score, best_p)
                }
            })
            .1
    }
}

impl<T: Transformation3d> ExtremePoint3d for Poly3dDiff<'_, T> {
    fn extreme_point(&self, direction: Vec3) -> Vec3 {
        self.a.extreme_point(direction) - self.rel.apply(self.b.extreme_point(-direction))
    }
}

// impl<'a, T: Transformation3d> MinkowskiDifferenceLifetimed<'a, Poly3d> for Poly3d {
//     type Output = Poly3dDiff<'a, T>;
//     fn minkowski_difference_lt(&'a self, t: &'a Poly3d) -> Self::Output {
//         Poly3dDiff { a: self, b: t }
//     }
// }

impl<T: Transformation3d> CollidesRel3d<Point> for Poly3dDiff<'_, T> {
    fn collides_rel(&self, _t: &Point, rel: &impl Transformation3d) -> bool {
        let mut point_count = 1;
        let mut points = [Vec3::ZERO; 4];

        let delta = rel.apply_origin();
        let a = self.extreme_point(delta);
        if a.dot(delta) < 0.0 {
            return false;
        }

        let mut delta = -a;
        for _ in 0..16 {
            let p = self.extreme_point(delta);
            if p.dot(delta) <= 0.0 {
                return false;
            }

            points[point_count] = p;
            point_count += 1;

            match gjk3d::do_simplex(&mut points, &mut point_count) {
                Some(new_direction) => delta = new_direction,
                None => return true,
            }
        }

        false
    }
}

impl<T: Transformation3d> PenetratesRel3d<Point> for Poly3dDiff<'_, T> {
    fn penetrates_rel(&self, _t: &Point, rel: &impl Transformation3d) -> Option<Vec3> {
        let mut point_count = 1;
        let mut points = [Vec3::ZERO; 4];

        let delta = rel.apply_origin();
        let a = self.extreme_point(delta);
        if a.dot(delta) < 0.0 {
            return None;
        }

        let mut delta = -a;
        for _ in 0..16 {
            let p = self.extreme_point(delta);
            if p.dot(delta) <= 0.0 {
                return None;
            }

            points[point_count] = p;
            point_count += 1;

            match gjk3d::do_simplex(&mut points, &mut point_count) {
                Some(new_direction) => delta = new_direction,
                None => {
                    assert!(point_count == 4);
                    return Some(gjk3d::Epa3d::epa(self, &mut points));
                }
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_box() -> Poly3d {
        Poly3d::new(&[
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 1.0),
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
        ])
    }

    #[test]
    fn tetrahedron_extreme_points() {
        //     C
        //    ||\
        //    || \
        //    ||  \
        //   | |   \
        //   | |    \
        //   | |     \
        //  |  A______B
        //  | /    /
        //  |/ /
        //  D
        let a = Vec3::ZERO;
        let b = Vec3::X;
        let c = 2.0 * Vec3::Y;
        let d = Vec3::new(0.5, 0.5, 3.0);
        let tetrahedron = Poly3d::new(&[a, b, c, d]);
        assert_eq!(b, tetrahedron.extreme_point(Vec3::X));
        assert_eq!(c, tetrahedron.extreme_point(Vec3::new(1.0, 1.0, 0.0)));
        assert_eq!(c, tetrahedron.extreme_point(Vec3::Y));
        assert_eq!(c, tetrahedron.extreme_point(Vec3::new(-1.0, 1.0, 0.0)));
        assert_eq!(a, tetrahedron.extreme_point(Vec3::new(-1.0, -1.0, 0.0)));
        assert_eq!(b, tetrahedron.extreme_point(Vec3::new(1.0, -1.0, 0.0)));
        assert_eq!(d, tetrahedron.extreme_point(Vec3::Z));
    }

    #[test]
    fn box_box_gjk() {
        let a = unit_box();
        let rel = Translate3d::from(Vec3::new(0.5, 0.5, 0.5));
        let poly_diff = Poly3dDiff::new(&a, &a, &rel);
        assert!(poly_diff.collides_rel(&Point, &IdentityTransform));
    }

    #[test]
    fn box_box_gjk2() {
        let a = unit_box();
        let rel = Translate3d::from(Vec3::new(2.0, 2.0, 0.0));
        let poly_diff = Poly3dDiff::new(&a, &a, &rel);
        assert!(!poly_diff.collides_rel(&Point, &IdentityTransform));
    }

    fn box_box_gjk_sweep(start: Vec3, end: Vec3, steps: usize, expected_collides: bool) {
        for i in 0..steps {
            let p = start.lerp(end, i as f32 / steps as f32);
            let a = unit_box();
            let rel = Translate3d::from(p);
            let poly_diff = Poly3dDiff::new(&a, &a, &rel);
            assert_eq!(
                expected_collides,
                poly_diff.collides_rel(&Point, &IdentityTransform)
            );
        }
    }

    #[test]
    fn box_box_gjk_sweep_far_right() {
        box_box_gjk_sweep(
            Vec3::new(1.1, -1.1, 0.0),
            Vec3::new(1.1, 1.1, 0.0),
            10,
            false,
        );
    }

    #[test]
    fn box_box_gjk_sweep_near_right() {
        box_box_gjk_sweep(
            Vec3::new(0.9, -0.9, 0.0),
            Vec3::new(0.9, 0.9, 0.0),
            10,
            true,
        );
    }

    #[test]
    fn box_box_gjk_sweep_far_up() {
        box_box_gjk_sweep(
            Vec3::new(-1.1, 1.1, 0.0),
            Vec3::new(1.1, 1.1, 0.0),
            10,
            false,
        );
    }

    #[test]
    fn box_box_gjk_sweep_near_up() {
        box_box_gjk_sweep(
            Vec3::new(-0.9, 0.9, 0.0),
            Vec3::new(0.9, 0.9, 0.0),
            10,
            true,
        );
    }

    #[test]
    fn box_box_gjk_sweep_far_left() {
        box_box_gjk_sweep(
            Vec3::new(-1.1, -1.1, 0.0),
            Vec3::new(-1.1, 1.1, 0.0),
            10,
            false,
        );
    }

    #[test]
    fn box_box_gjk_sweep_near_left() {
        box_box_gjk_sweep(
            Vec3::new(-0.9, -0.9, 0.0),
            Vec3::new(-0.9, 0.9, 0.0),
            10,
            true,
        );
    }

    #[test]
    fn box_box_gjk_sweep_far_down() {
        box_box_gjk_sweep(
            Vec3::new(-1.1, -1.1, 0.0),
            Vec3::new(1.1, -1.1, 0.0),
            10,
            false,
        );
    }

    #[test]
    fn box_box_gjk_sweep_near_down() {
        box_box_gjk_sweep(
            Vec3::new(-0.9, -0.9, 0.0),
            Vec3::new(0.9, -0.9, 0.0),
            10,
            true,
        );
    }
}
