mod ball;
#[cfg(feature = "bevy")]
mod bevy_transform;
mod point;
mod transform;

pub use ball::*;
use glam::Vec3;
pub use point::*;
pub use transform::*;

/// Trait for transforming 3D points.
pub trait Transformation3d {
    /// Transform the origin.
    fn apply_origin(&self) -> Vec3 {
        self.apply(Vec3::ZERO)
    }

    /// Transform a point.
    fn apply(&self, point: Vec3) -> Vec3;

    /// Invert transform the origin.
    fn unapply_origin(&self) -> Vec3 {
        self.unapply(Vec3::ZERO)
    }

    /// Invert transformation.
    fn unapply(&self, point: Vec3) -> Vec3;
}

impl Transformation3d for IdentityTransform {
    fn apply_origin(&self) -> Vec3 {
        Vec3::ZERO
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        point
    }
}
