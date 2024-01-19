mod translate3d;
mod axis_transform3d;
mod isotropic3d;

use super::*;

/// Trait for transforming 3D points.
pub trait Transform3d {
    /// Transform the origin.
    fn apply_origin(&self) -> Vec3;

    /// Transform a point.
    fn apply(&self, point: Vec3) -> Vec3;

    /// Invert transformation.
    fn unapply(&self, point: Vec3) -> Vec3;
}
