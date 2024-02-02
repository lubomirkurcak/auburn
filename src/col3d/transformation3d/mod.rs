use super::*;

mod axis_transform3d;
mod isotropic3d;
mod transform3d;
mod translate3d;

pub use axis_transform3d::*;
pub use isotropic3d::*;
pub use transform3d::*;
pub use translate3d::*;

#[cfg(feature = "bevy")]
mod bevy_transform;
#[cfg(feature = "bevy")]
pub use bevy_transform::*;

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
