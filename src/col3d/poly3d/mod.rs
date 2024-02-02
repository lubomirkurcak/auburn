use super::*;

mod gjk3d;

#[derive(Clone, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// # Limitation
/// Must contain the origin.
pub struct Poly3d {
    pub points: Vec<Vec3>,
}

pub struct Poly3dDiff<'a> {
    a: &'a Poly3d,
    b: &'a Poly3d,
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

impl ExtremePoint3d for Poly3dDiff<'_> {
    fn extreme_point(&self, direction: Vec3) -> Vec3 {
        self.a.extreme_point(direction) - self.b.extreme_point(-direction)
    }
}

impl<'a> MinkowskiDifferenceLifetimed<'a, Poly3d> for Poly3d {
    type Output = Poly3dDiff<'a>;
    fn minkowski_difference_lt(&'a self, t: &'a Poly3d) -> Self::Output {
        Poly3dDiff { a: self, b: t }
    }
}

impl CollidesRel3d<Point> for Poly3dDiff<'_> {
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
