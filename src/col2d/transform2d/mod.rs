use super::*;

mod axis_transform2d;
mod isotropic2d;
mod translate2d;

pub use axis_transform2d::*;
pub use isotropic2d::*;
pub use translate2d::*;

/// Trait for transforming 2D points.
pub trait Transform2d {
    /// Transform the origin.
    fn apply_origin(&self) -> Vec2;

    /// Transform a point.
    fn apply(&self, point: Vec2) -> Vec2;

    /// Invert transformation.
    fn unapply(&self, point: Vec2) -> Vec2;
}

impl Transform2d for IdentityTransform {
    fn apply_origin(&self) -> Vec2 {
        Vec2::ZERO
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        point
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        point
    }
}
