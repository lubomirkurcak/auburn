//! Physics.
//!
//! # Generic
//!
//! * `()` - point (the origin)
//! * [Ball] - ball (aka "sphere")
//!
//! # 2D
//!
//! Collision and Resolution:
//! * [Collides2d::collides]
//! * [Penetrates2d::penetrates]
//! * [Sdf2d::sdf]
//! * [Sdf2dVector::sdfvector].
//!
//! Shapes:
//! * [Box2d] - 2D box
//! * [RoundedBox2d] - 2D box
//! * [Poly2d] - 2D box
//! * [Tilemap] - 2D tilemap
//!
//! # 2D
//!
//! Collision and Resolution:
//! * [Collides3d::collides]
//! * [Penetrates3d::penetrates]
//! * [Sdf3d::sdf]
//! * [Sdf3dVector::sdfvector].
//!
//! Shapes:
//! * [Box3d] - 3D box

mod ball;
mod box2d;
mod point;
mod poly2d;
mod rounded_box2d;
mod tilemap;
mod transform2d;
mod box3d;
mod transform3d;
mod transform;
mod ball2d;
mod ball3d;
mod cylinder3d;

// use bevy::prelude::*;
pub use glam::Vec2;
pub use glam::Vec3;
pub use glam::Quat;

pub use point::*;
pub use ball::*;
pub use transform::*;

pub use ball2d::*;
pub use box2d::*;
pub use poly2d::*;
pub use rounded_box2d::*;
pub use tilemap::*;
pub use transform2d::*;

pub use ball3d::*;
pub use box3d::*;
pub use cylinder3d::*;
pub use transform3d::*;

/// Trait for computing Minkowski sum.
///
/// # Example
/// ```rust
/// impl MinkowskiSum<Sphere> for Box2d {
///     type Output = Box2d;
///
///     fn minkowski_sum(&self, t: &Box2d) -> Self::Output {
///         Self::Output {
///             halfsize: self.halfsize + t.halfsize,
///         }
///     }
/// }
/// ```
///
/// # See also
/// * [MinkowskiNegation]
/// * [MinkowskiDifference]
pub trait MinkowskiSum<T> {
    type Output;
    /// Computes Minkowski sum of `self` and `t`.
    ///
    /// # Example
    /// ```rust
    /// let a = Sphere::with_radius(1.0).at_origin();
    /// let b = Sphere::with_radius(1.0).at_position(Vec2::new(1.0, 0.0));
    /// assert_eq!(
    ///     a.minkowski_sum(&b),
    ///     Sphere::with_radius(2.0).at_position(Vec2::new(1.0, 0.0))
    /// );
    /// ```
    ///
    /// # See also
    /// * [MinkowskiNegation::minkowski_negation]
    /// * [MinkowskiDifference::minkowski_difference]
    fn minkowski_sum(&self, t: &T) -> Self::Output;
}

/// Marker trait for when the Minkowski negation is identity.
/// This would usually be true for shapes centered at the origin and symmetric around it.
///
/// # See also
/// * [MinkowskiNegation]
/// * [MinkowskiSum]
/// * [MinkowskiDifference]
pub trait MinkowskiNegationIsIdentity: Copy {}

/// Trait for computing Minkowski negation.
///
/// # See also
/// * [MinkowskiNegationIsIdentity]
/// * [MinkowskiSum]
/// * [MinkowskiDifference]
pub trait MinkowskiNegation {
    /// Computes Minkowski negation. (Reflection about the origin)
    ///
    /// # Example
    ///
    /// ```rust
    /// let a = Sphere::with_radius(1.0).at_position(Vec2::new(1.0, 0.0));
    /// assert_eq!(
    ///     a.minkowski_negation();
    ///     Sphere::with_radius(1.0).at_position(Vec2::new(-1.0, 0.0))
    /// );
    /// ```
    ///
    /// # See also
    /// * [MinkowskiSum::minkowski_sum]
    /// * [MinkowskiDifference::minkowski_difference]
    fn minkowski_negation(&self) -> Self;
}

/// Trait for computing Minkowski difference.
///
/// # Default implementations:
///
/// For [MinkowskiNegation]:
/// ```rust
/// impl<T, U, V> MinkowskiDifference<U> for T
/// where
///     T: MinkowskiSum<U, Output = V>,
///     U: MinkowskiNegation,
/// {
///     type Output = V;
///
///     fn minkowski_difference(&self, t: &U) -> Self::Output {
///         self.minkowski_sum(&t.minkowski_negation())
///     }
/// }
/// ```
///
/// For [MinkowskiNegationIsIdentity]:
/// ```rust
/// impl<T, U, V> MinkowskiDifference<U> for T
/// where
///     T: MinkowskiSum<U, Output = V>,
///     U: MinkowskiNegationIsIdentity,
/// {
///     type Output = V;
///     fn minkowski_difference(&self, t: &U) -> Self::Output {
///         self.minkowski_sum(&t)
///     }
/// }
/// ```
///
/// # See also
/// * [MinkowskiSum]
/// * [MinkowskiNegation]
pub trait MinkowskiDifference<T> {
    type Output;
    /// Computes Minkowski difference between `self` and `t`.
    ///
    /// # Example
    /// ```rust
    /// let a = Sphere::with_radius(1.0).at_origin();
    /// let b = Sphere::with_radius(1.0).at_position(Vec2::new(1.0, 0.0));
    /// assert_eq!(
    ///     a.minkowski_difference(&b),
    ///     Sphere::with_radius(2.0).at_position(Vec2::new(-1.0, 0.0))
    /// );
    /// ```
    ///
    /// # Notes
    /// Useful for collisions, since when the difference contains the origin, the shapes overlap.
    ///
    /// Has a default implementation when `Self : MinkowskiSum<T>` and `T : MinkowskiNegation`.
    ///
    /// # See also
    /// * [MinkowskiDifference]
    /// * [MinkowskiSum::minkowski_sum]
    /// * [MinkowskiNegation::minkowski_negation]
    fn minkowski_difference(&self, t: &T) -> Self::Output;
}

impl<T: MinkowskiNegationIsIdentity> MinkowskiNegation for T {
    fn minkowski_negation(&self) -> Self {
        *self
    }
}

impl<T, U, V> MinkowskiDifference<U> for T
where
    T: MinkowskiSum<U, Output = V>,
    U: MinkowskiNegation,
{
    type Output = V;
    fn minkowski_difference(&self, t: &U) -> Self::Output {
        self.minkowski_sum(&t.minkowski_negation())
    }
}

#[doc(alias = "Support")]
#[doc(alias = "SupportPoint")]
#[doc(alias = "SupportPoint2d")]
/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePoint2d {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```rust
    /// let sphere = Sphere::with_radius(2.0);
    /// let point = sphere.extreme_point(&Vec2::X);
    /// assert_eq!(point, Vec2::new(2.0, 0.0));
    /// ```
    fn extreme_point(&self, direction: &Vec2) -> Vec2;
}

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

/// Trait for computing bounding box of a shape.
///
/// # See also
/// * [Collides2d]
pub trait SymmetricBoundingBox2d {
    /// Computes the bounding box.
    ///
    /// # Example
    /// ```
    /// assert_eq!(
    ///     Ball::with_radius(1.0).bounding_box(),
    ///     Box2d::with_halfdims(1.0, 1.0)
    /// );
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn symmetric_bounding_box(&self) -> Box2d;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [BoundingBox2d]
/// * [Penetrates2d]
pub trait Collides2d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate2d::new(t.pos - self.pos);
    /// if self.collides(&t, &rel) {
    ///     println!("hit!");
    /// }
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides(&self, t: &T, rel: &impl Transform2dTrait) -> bool;
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
/// * [BoundingBox2d]
/// * [Penetrates2d]
pub trait CollidesT2d<T> {
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
    /// * [Penetrates2d::penetrates].
    fn collides(
        &self,
        transform: &impl Transform2dTrait,
        t: &T,
        t_transform: &impl Transform2dTrait,
    ) -> bool;
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
/// * [`Collides2d`]
pub trait Penetrates2d<T> {
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate2d::new(t.pos - self.pos);
    /// if let Some(p) = self.penetration(&t, &rel) {
    ///     t.pos += p; // push `t`
    /// }
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn penetrates(&self, t: &T, rel: &impl Transform2dTrait) -> Option<Vec2>;
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
/// * [`Sdf2dVector`]
pub trait Sdf2d<T> {
    /// Computes *scalar* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate2d::new(t.pos - self.pos);
    /// let s = self.sdf(&t, &rel);
    /// println!("distance: {}", s);
    /// ```
    ///
    /// # See also
    /// * [Sdf2dVector::sdfvector].
    fn sdf(&self, t: &T, rel: &impl Transform2dTrait) -> f32;
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
/// * [`Sdf2d`]
pub trait Sdf2dVector<T> {
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate2d::new(t.pos - self.pos);
    /// let p = self.sdfvector(&t, &rel);
    /// t.pos += p; // push or pull `t` such that it touches `self`
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfvector(&self, t: &T, rel: &impl Transform2dTrait) -> Vec2;
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
