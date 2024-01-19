//! 3D Collisions.
//!
//! # Generic
//!
//! * `()` - point (the origin)
//! * [Ball] - ball
//!
//! # 3D
//!
//! Collision and Resolution:
//! * [Collides3d::collides]
//! * [Penetrates3d::penetrates]
//! * [Sdf3d::sdf]
//! * [Sdf3dVector::sdfvector].
//!
//! Shapes:
//! * [Box3d] - 3D box

pub use crate::{Quat, Vec2, Vec3};

mod ball3d;
mod box3d;
mod cylinder3d;
mod transform3d;

pub use crate::col::*;
pub use ball3d::*;
pub use box3d::*;
pub use cylinder3d::*;
pub use transform3d::*;

#[doc(alias = "Support")]
#[doc(alias = "SupportPoint")]
#[doc(alias = "SupportPoint3d")]
/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePoint3d {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```rust
    /// let sphere = Sphere::with_radius(2.0);
    /// let point = sphere.extreme_point(&Vec3::X);
    /// assert_eq!(point, Vec2::new(2.0, 0.0, 0.0));
    /// ```
    fn extreme_point(&self, direction: &Vec3) -> Vec3;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [BoundingBox3d]
/// * [Penetrates3d]
pub trait Collides3d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate3d::new(t.pos - self.pos);
    /// if self.collides(&t, &rel) {
    ///     println!("hit!");
    /// }
    /// ```
    ///
    /// # See also
    /// * [Penetrates3d::penetrates].
    fn collides(&self, t: &T, rel: &impl Transform3dTrait) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [BoundingBox3d]
/// * [Penetrates3d]
pub trait CollidesT3d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `delta` - The vector from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let delta = t.pos - self.pos;
    /// if self.collides(&t, &delta) {
    ///     println!("hit!");
    /// }
    /// ```
    ///
    /// # See also
    /// * [Penetrates3d::penetrates].
    fn collides(
        &self,
        transform: &impl Transform3dTrait,
        t: &T,
        t_transform: &impl Transform3dTrait,
    ) -> bool;
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [`Collides3d`]
pub trait Penetrates3d<T> {
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate3d::new(t.pos - self.pos);
    /// if let Some(p) = self.penetration(&t, &rel) {
    ///     t.pos += p; // push `t`
    /// }
    /// ```
    ///
    /// # See also
    /// * [Collides3d::collides].
    fn penetrates(&self, t: &T, rel: &impl Transform3dTrait) -> Option<Vec3>;
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`Sdf3dVector`]
pub trait Sdf3d<T> {
    /// Computes *scalar* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate3d::new(t.pos - self.pos);
    /// let s = self.sdf(&t, &rel);
    /// println!("distance: {}", s);
    /// ```
    ///
    /// # See also
    /// * [Sdf3dVector::sdfvector].
    fn sdf(&self, t: &T, rel: &impl Transform3dTrait) -> f32;
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`Sdf3d`]
pub trait Sdf3dVector<T> {
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate3d::new(t.pos - self.pos);
    /// let p = self.sdfvector(&t, &rel);
    /// t.pos += p; // push or pull `t` such that it touches `self`
    /// ```
    ///
    /// # See also
    /// * [Sdf3d::sdf].
    fn sdfvector(&self, t: &T, rel: &impl Transform3dTrait) -> Vec3;
}