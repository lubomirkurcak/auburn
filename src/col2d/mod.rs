//! 2D Collisions.
//!
//! # Generic
//!
//! * `()` - point (the origin)
//! * [Ball] - ball
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

pub use crate::Vec2;

mod ball2d;
mod box2d;
mod point2d;
mod poly2d;
mod rounded_box2d;
mod tilemap;
mod transform2d;

pub use crate::col::*;
pub use ball2d::*;
pub use box2d::*;
pub use point2d::*;
pub use poly2d::*;
pub use rounded_box2d::*;
pub use tilemap::*;
pub use transform2d::*;

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

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [BoundingBox2d]
/// * [Penetrates2d]
pub trait CollidesRel2d<T> {
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
    fn collides_rel(&self, t: &T, rel: &impl Transform2d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [BoundingBox2d]
/// * [Penetrates2d]
pub trait Collides2d<T, U: Transform2d> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `delta` - The vector from `self` to `t`
    ///
    /// # Example
    /// ```
    /// let transform = Translate2d::new(self.pos);
    /// let t_transform = Translate2d::new(t.pos);
    /// if self.collides(&transform, &t, &t_transform) {
    ///     println!("hit!");
    /// }
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides(&self, transform: &U, t: &T, t_transform: &U) -> bool;
}

impl<T, U> Collides2d<T, U> for T
where
    T: CollidesRel2d<T>,
    U: Transform2d + DeltaTransform,
{
    fn collides(&self, transform: &U, t: &T, t_transform: &U) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
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
    fn penetrates(&self, t: &T, rel: &impl Transform2d) -> Option<Vec2>;
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`Sdf2dVector`]
pub trait Sdf2d<T> {
    /// Computes *scalar* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `t` - The object to check collision against
    /// * `delta` - Transform of `t`
    ///
    /// # Example
    /// ```
    /// let rel = Translate2d::new(t.pos - self.pos);
    /// let s = self.sdf(&transform, &t, &t_transform);
    /// println!("distance: {}", s);
    /// ```
    ///
    /// # See also
    /// * [Sdf2dVector::sdfvector].
    fn sdf(&self, t: &T, rel: &impl Transform2d) -> f32;
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
    fn sdfvector(&self, t: &T, rel: &impl Transform2d) -> Vec2;
}
