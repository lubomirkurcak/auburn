use super::*;

#[derive(Clone, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
// pub struct Poly2d<const N: usize> {
//     pub points: [Vec2; N],
// }
pub struct Poly2d {
    pub points: Vec<Vec2>,
}

impl Poly2d {
    pub fn new(points: &[Vec2]) -> Self {
        Self {
            points: points.to_vec(),
        }
    }
    pub fn regular(sides: usize, radius: f32) -> Self {
        let mut points = vec![];
        for i in 0..sides {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (sides as f32);
            points[i] = radius * Vec2::new(angle.cos(), angle.sin());
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

fn cross_aba(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(
        a.y * a.y * b.x - a.x * a.y * b.y,
        a.x * a.x * b.y - a.x * a.y * b.x,
    )
}

impl CollidesRel2d<Poly2d> for Poly2d {
    fn collides_rel(&self, t: &Poly2d, rel: &impl Transform2d) -> bool {
        let delta = rel.apply_origin();
        let a = self.support(t, delta);
        if a.dot(delta) < 0.0 {
            return false;
        }

        let delta = -a;
        let b = self.support(t, delta);

        let delta = cross_aba(a - b, -a);
        let c = self.support(t, delta);
        todo!()
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
