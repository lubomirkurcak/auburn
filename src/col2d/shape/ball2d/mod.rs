use super::*;

mod v_ball2d;
mod v_point;
// mod v_box2d;

impl DefaultMinkowski<Box2d> for Ball {}

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

impl<T: Transformation2d> ExtremePointT2d<T> for Ball {
    fn extreme_point_t(&self, t: &T, direction: Vec2) -> Vec2 {
        t.apply_origin() + t.scaling_factor() * self.extreme_point(direction)
    }
}
