use super::*;

mod gjk2d;

#[derive(Clone, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
// pub struct Poly2d<const N: usize> {
//     pub points: [Vec2; N],
// }
/// # Limitation
/// Must contain the origin.
pub struct Poly2d {
    pub points: Vec<Vec2>,
}

pub struct Poly2dDiff<'a> {
    a: &'a Poly2d,
    b: &'a Poly2d,
}

//

impl Poly2d {
    pub fn new(points: &[Vec2]) -> Self {
        Self {
            points: points.to_vec(),
        }
    }
    pub fn regular(sides: usize, radius: f32) -> Self {
        let mut points = Vec::with_capacity(sides);
        for i in 0..sides {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (sides as f32);
            // points[i] = radius * Vec2::new(angle.cos(), angle.sin());
            points.push(radius * Vec2::new(angle.cos(), angle.sin()))
        }
        Self { points }
    }
}

impl SymmetricBoundingBox2d for Poly2d {
    fn symmetric_bounding_box(&self) -> Box2d {
        self.points
            .iter()
            .fold(Box2d::with_halfdims(0.0, 0.0), |mut b, point| {
                b.cover(point);
                b
            })
    }
}

impl ExtremePoint2d for Poly2d {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        self.points
            .iter()
            .cloned()
            .fold((f32::MIN, Vec2::ZERO), |(best_score, best_p), p| {
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

impl ExtremePoint2d for Poly2dDiff<'_> {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        self.a.extreme_point(direction) - self.b.extreme_point(-direction)
    }
}

impl<'a> MinkowskiDifferenceLifetimed<'a, Poly2d> for Poly2d {
    type Output = Poly2dDiff<'a>;
    fn minkowski_difference_lt(&'a self, t: &'a Poly2d) -> Self::Output {
        Poly2dDiff { a: self, b: t }
    }
}

impl DefaultCol2dImpls for Poly2d {}

// Collides

impl CollidesRel2d<Point> for Poly2dDiff<'_> {
    fn collides_rel(&self, _t: &Point, rel: &impl Transform2d) -> bool {
        let mut point_count = 1;
        let mut points = [Vec2::ZERO; 4];

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

            match gjk2d::do_simplex(&mut points, &mut point_count) {
                Some(new_direction) => delta = new_direction,
                None => return true,
            }
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_extreme_points() {
        // C
        // |\
        // | \
        // |  \
        // |   \
        // |    \
        // |     \
        // A______B
        let a = Vec2::ZERO;
        let b = Vec2::X;
        let c = 2.0 * Vec2::Y;
        let triangle = Poly2d::new(&[a, b, c]);
        assert_eq!(b, triangle.extreme_point(Vec2::X));
        assert_eq!(c, triangle.extreme_point(Vec2::new(1.0, 1.0)));
        assert_eq!(c, triangle.extreme_point(Vec2::Y));
        assert_eq!(c, triangle.extreme_point(Vec2::new(-1.0, 1.0)));
        assert_eq!(a, triangle.extreme_point(Vec2::new(-1.0, -1.0)));
        assert_eq!(b, triangle.extreme_point(Vec2::new(1.0, -1.0)));
    }
}
