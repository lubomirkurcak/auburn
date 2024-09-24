mod v_poly2d;

use super::*;

// mod gjk2d;

#[derive(Clone, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// # Limitation
/// Must contain the origin.
pub struct Poly2d {
    pub points: Vec<Vec2>,
}
// pub struct Poly2d<const N: usize> {
//     pub points: [Vec2; N],
// }

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

impl Poly2d {
    /// [Poly2d] represents a convex polygon encompassing the origin.
    /// This function checks that invariant by testing whether all points are the farthest point
    /// in their respective directions away from the origin.
    ///
    /// This is a strict check, meaning that the origin must be strictly contained within the
    /// polygon to pass.
    pub fn strict_check(&self) -> bool {
        self.points.iter().all(|&p| self.extreme_point(p) == p)
    }
}

impl From<Box2d> for Poly2d {
    fn from(b: Box2d) -> Self {
        let halfsize = b.halfsize;
        let min = -halfsize;
        let max = halfsize;
        Self {
            points: vec![
                Vec2::new(min.x, min.y),
                Vec2::new(min.x, max.y),
                Vec2::new(max.x, max.y),
                Vec2::new(max.x, min.y),
            ],
        }
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

impl DefaultCol2dImpls for Poly2d {}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_box() -> Poly2d {
        //     D______C
        //     |      |
        //     |      |
        //     |      |
        // O = A______B
        let a = Poly2d::new(&[
            Vec2::new(0.0, 0.0),
            Vec2::new(0.0, 1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(1.0, 0.0),
        ]);
        // NOTE: This unit box does not strictly contain the origin making this check possible to
        // fail.
        // assert!(a.strict_check());
        a
    }

    #[test]
    fn triangle_extreme_points() {
        // C
        // |\
        // | \
        // |  \
        // |   \
        // |  O \
        // |     \
        // A______B
        let a = Vec2::new(-1.0, -1.0);
        let b = Vec2::new(1.0, -1.0);
        let c = Vec2::new(-1.0, 2.0);
        let triangle = Poly2d::new(&[a, b, c]);

        assert!(triangle.strict_check());

        assert_eq!(b, triangle.extreme_point(Vec2::X));
        assert_eq!(c, triangle.extreme_point(Vec2::new(1.0, 1.0)));
        assert_eq!(c, triangle.extreme_point(Vec2::Y));
        assert_eq!(c, triangle.extreme_point(Vec2::new(-1.0, 1.0)));
        assert_eq!(a, triangle.extreme_point(Vec2::new(-1.0, -1.0)));
        assert_eq!(b, triangle.extreme_point(Vec2::new(1.0, -1.0)));
    }

    #[test]
    fn unit_box_extreme_points() {
        let a = Vec2::ZERO;
        let b = Vec2::X;
        let c = Vec2::ONE;
        let d = Vec2::Y;
        let unit_box = unit_box();

        assert_eq!(a, unit_box.extreme_point(Vec2::new(-1.0, -1.0)));
        assert_eq!(b, unit_box.extreme_point(Vec2::new(1.0, -1.0)));
        assert_eq!(c, unit_box.extreme_point(Vec2::new(1.0, 1.0)));
        assert_eq!(d, unit_box.extreme_point(Vec2::new(-1.0, 1.0)));
    }

    // TODO: Re-enable these tests
    /*
    #[test]
    fn box_box_gjk() {
        let a = unit_box();
        let rel = Translate2d::from(Vec2::new(0.5, 0.0));
        let poly_diff = Poly2dDiff::new(&a, &a, &rel);
        assert!(poly_diff.collides_rel(&Point, &IdentityTransform));
    }

    #[test]
    fn box_box_gjk2() {
        let a = unit_box();
        let rel = Translate2d::from(Vec2::new(2.0, 2.0));
        let poly_diff = Poly2dDiff::new(&a, &a, &rel);
        assert!(!poly_diff.collides_rel(&Point, &IdentityTransform));
    }

    fn box_box_gjk_sweep(start: Vec2, end: Vec2, steps: usize, expected_collides: bool) {
        for i in 0..steps {
            let p = start.lerp(end, i as f32 / steps as f32);
            let a = unit_box();
            let rel = Translate2d::from(p);
            let poly_diff = Poly2dDiff::new(&a, &a, &rel);
            assert_eq!(
                expected_collides,
                poly_diff.collides_rel(&Point, &IdentityTransform)
            );
        }
    }

    #[test]
    fn box_box_gjk_sweep_far_right() {
        box_box_gjk_sweep(Vec2::new(1.1, -1.1), Vec2::new(1.1, 1.1), 10, false);
    }

    #[test]
    fn box_box_gjk_sweep_near_right() {
        box_box_gjk_sweep(Vec2::new(0.9, -0.9), Vec2::new(0.9, 0.9), 10, true);
    }

    #[test]
    fn box_box_gjk_sweep_far_up() {
        box_box_gjk_sweep(Vec2::new(-1.1, 1.1), Vec2::new(1.1, 1.1), 10, false);
    }

    #[test]
    fn box_box_gjk_sweep_near_up() {
        box_box_gjk_sweep(Vec2::new(-0.9, 0.9), Vec2::new(0.9, 0.9), 10, true);
    }

    #[test]
    fn box_box_gjk_sweep_far_left() {
        box_box_gjk_sweep(Vec2::new(-1.1, -1.1), Vec2::new(-1.1, 1.1), 10, false);
    }

    #[test]
    fn box_box_gjk_sweep_near_left() {
        box_box_gjk_sweep(Vec2::new(-0.9, -0.9), Vec2::new(-0.9, 0.9), 10, true);
    }

    #[test]
    fn box_box_gjk_sweep_far_down() {
        box_box_gjk_sweep(Vec2::new(-1.1, -1.1), Vec2::new(1.1, -1.1), 10, false);
    }

    #[test]
    fn box_box_gjk_sweep_near_down() {
        box_box_gjk_sweep(Vec2::new(-0.9, -0.9), Vec2::new(0.9, -0.9), 10, true);
    }
    */
}
