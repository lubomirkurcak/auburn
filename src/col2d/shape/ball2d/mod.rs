use super::*;

mod v_ball2d;
mod v_point;
// mod v_box2d;

impl DefaultMinkowski<Box2d> for Ball {}

impl DefaultCol2dImpls for Ball {}

impl SymmetricBoundingBox2d for Ball {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(self.radius, self.radius)
    }
}

impl ExtremePoint2d for Ball {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        self.radius * direction.normalize_or_zero()
    }
}
