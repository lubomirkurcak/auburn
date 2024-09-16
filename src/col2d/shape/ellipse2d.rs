use crate::Square;

use super::*;

/// 2D ellipse centered at the origin, with radii for the x and y axes.
///
/// Ellipses are really stupid shapes and we could care less about them.
#[derive(Default, Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
pub struct Ellipse2d {
    pub radii: Vec2,
}

//

impl SymmetricBoundingBox2d for Ellipse2d {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(self.radii.x, self.radii.y)
    }
}

impl Ellipse2d {
    /// The intercept of a ray starting at the ellipse's center.
    fn intercept_ray(&self, dir: Vec2) -> Vec2 {
        dir * (dir / self.radii).length_recip()
    }

    fn extreme_point_direction(&self, dir: Vec2) -> Vec2 {
        let a = self.radii.x;
        let b = self.radii.y;
        Vec2::new(a * a * dir.x, b * b * dir.y)
    }

    fn extreme_point_by_composing(&self, dir: Vec2) -> Vec2 {
        self.intercept_ray(self.extreme_point_direction(dir))
    }

    fn extreme_point_expanded(&self, dir: Vec2) -> Vec2 {
        let a = self.radii.x;
        let b = self.radii.y;
        let ap = a * dir.x;
        let bq = b * dir.y;
        let d = Vec2::new(ap, bq).length_recip();
        Vec2::new(a * ap * d, b * bq * d)
    }
}

impl ExtremePoint2d for Ellipse2d {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        self.extreme_point_expanded(direction)
    }
}

impl MinkowskiNegationIsIdentity for Ellipse2d {}

// @note(lubo): Ellipse + Ellipse != Ellipse
// impl MinkowskiSum<Ellipse2d> for Ellipse2d {}

// @note(lubo): This could potentially work but why bother
// impl MinkowskiSum<Box2d> for Ellipse2d {}

// Collides

impl CollidesRel2d<Point> for Ellipse2d {
    fn collides_rel(&self, t: &Point, rel: &impl Transformation2d) -> bool {
        let delta = rel.apply_origin();
        (delta.x / self.radii.x).square() + (delta.y / self.radii.y).square() <= 1.0
    }
}

// Penetrates

// Sdf

// Sdf Vector

#[cfg(test)]
mod tests {
    use super::*;

    const E1: Ellipse2d = Ellipse2d {
        radii: Vec2::new(2.0, 0.5),
    };

    #[test]
    fn test_ellipse_extreme_point() {
        assert_eq!(E1.extreme_point(Vec2::X), Vec2::new(2.0, 0.0));
        assert_eq!(E1.extreme_point(Vec2::Y), Vec2::new(0.0, 0.5));
        assert_eq!(E1.extreme_point(-Vec2::X), Vec2::new(-2.0, 0.0));
        assert_eq!(E1.extreme_point(-Vec2::Y), Vec2::new(0.0, -0.5));
    }

    #[test]
    fn test_extreme_point_compose() {
        assert_eq!(E1.extreme_point_by_composing(Vec2::X), Vec2::new(2.0, 0.0));
        assert_eq!(E1.extreme_point_by_composing(Vec2::Y), Vec2::new(0.0, 0.5));
        assert_eq!(
            E1.extreme_point_by_composing(-Vec2::X),
            Vec2::new(-2.0, 0.0)
        );
        assert_eq!(
            E1.extreme_point_by_composing(-Vec2::Y),
            Vec2::new(0.0, -0.5)
        );
    }

    #[test]
    fn test_extreme_point_expanded() {
        assert_eq!(E1.extreme_point_expanded(Vec2::X), Vec2::new(2.0, 0.0));
        assert_eq!(E1.extreme_point_expanded(Vec2::Y), Vec2::new(0.0, 0.5));
        assert_eq!(E1.extreme_point_expanded(-Vec2::X), Vec2::new(-2.0, 0.0));
        assert_eq!(E1.extreme_point_expanded(-Vec2::Y), Vec2::new(0.0, -0.5));
    }
}
