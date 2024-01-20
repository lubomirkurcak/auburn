use super::*;

#[derive(Clone)]
pub struct Poly2d<const N: usize> {
    pub points: [Vec2; N],
}

// type PolyPoint2d = Poly2d<1>;
// type Line2d = Poly2d<2>;
pub type Triangle2d = Poly2d<3>;
// type Simplex2d = Triangle2d;

pub struct PolyDifference2d<'a, const N: usize> {
    pub a: &'a Poly2d<N>,
}

impl<const N: usize> Poly2d<N> {
    pub const fn new(points: [Vec2; N]) -> Self {
        Self { points }
    }
    pub fn regular(radius: f32) -> Self {
        let mut points = [Vec2::ZERO; N];
        for i in 0..N {
            let angle = 2.0 * std::f32::consts::PI * (i as f32) / (N as f32);
            points[i] = radius * Vec2::new(angle.cos(), angle.sin());
        }
        Self { points }
    }
}

impl<const N: usize> SymmetricBoundingBox2d for Poly2d<N> {
    fn symmetric_bounding_box(&self) -> Box2d {
        self.points
            .iter()
            .fold(Box2d::with_halfdims(0.0, 0.0), |mut b, point| {
                b.cover(point);
                b
            })
    }
}

impl<const N: usize> ExtremePoint2d for Poly2d<N> {
    fn extreme_point(&self, direction: &Vec2) -> Vec2 {
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

// NOTE(lubo): aka GJK
impl<const N: usize> CollidesRel2d<()> for Poly2d<N> {
    fn collides_rel(&self, t: &(), rel: &impl Transform2d) -> bool {
        let delta = rel.apply_origin();
        let a = self.extreme_point(&delta);
        todo!()
    }
}

impl<const N: usize, const M: usize> CollidesRel2d<Poly2d<M>> for Poly2d<N> {
    fn collides_rel(&self, t: &Poly2d<M>, rel: &impl Transform2d) -> bool {
        let delta = rel.apply_origin();
        let a = self.extreme_point(&delta);
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
        let triangle = Triangle2d::new([a, b, c]);
        assert_eq!(b, triangle.extreme_point(&Vec2::X));
        assert_eq!(c, triangle.extreme_point(&Vec2::new(1.0, 1.0)));
        assert_eq!(c, triangle.extreme_point(&Vec2::Y));
        assert_eq!(c, triangle.extreme_point(&Vec2::new(-1.0, 1.0)));
        assert_eq!(a, triangle.extreme_point(&Vec2::new(-1.0, -1.0)));
        assert_eq!(b, triangle.extreme_point(&Vec2::new(1.0, -1.0)));
    }
}
