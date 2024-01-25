//! 3D Collisions.
//!
//! # Generic
//!
//! * [Point] - point (the origin)
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
    /// ```
    /// # use auburn::col3d::*;
    /// let ball = Ball::with_radius(2.0);
    /// let point = ball.extreme_point(&Vec3::X);
    /// assert_eq!(point, Vec3::new(2.0, 0.0, 0.0));
    /// ```
    fn extreme_point(&self, direction: &Vec3) -> Vec3;
}

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [BoundingBox3d]
/// * [Penetrates3d]
pub trait CollidesRel3d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// assert!(a.collides_rel(&b, &rel))
    /// ```
    ///
    /// # See also
    /// * [Penetrates3d::penetrates].
    fn collides_rel(&self, t: &T, rel: &impl Transform3d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [BoundingBox3d]
/// * [Penetrates3d]
pub trait Collides3d<T, U: Transform3d> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `t` - The object to check collision against
    /// * `delta` - Transform of `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let a_t = Translate3d::from(Vec3::new(0.0, 0.0, 0.0));
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b_t = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert!(a.collides(&a_t, &b, &b_t))
    /// ```
    ///
    /// # See also
    /// * [Penetrates3d::penetrates].
    fn collides(&self, transform: &U, t: &T, t_transform: &U) -> bool;
}

impl<T, U> Collides3d<T, U> for T
where
    T: CollidesRel3d<T>,
    U: Transform3d + DeltaTransform,
{
    fn collides(&self, transform: &U, t: &T, t_transform: &U) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
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
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(a.penetrates(&b, &rel), Some(Vec3::new(-1.0, 0.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides3d::collides].
    fn penetrates(&self, t: &T, rel: &impl Transform3d) -> Option<Vec3>;
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
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(a.sdf(&b, &rel), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf3dVector::sdfvector].
    fn sdf(&self, t: &T, rel: &impl Transform3d) -> f32;
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
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(a.sdfvector(&b, &rel), Vec3::new(-1.0, 0.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf3d::sdf].
    fn sdfvector(&self, t: &T, rel: &impl Transform3d) -> Vec3;
}

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

trait DefaultCol3dImpls {}

impl<A> CollidesRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: CollidesRel3d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transform3d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

impl<A, B, C> CollidesRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel3d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transform3d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A, B, C> Penetrates3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Penetrates3d<Point>,
{
    fn penetrates(&self, t: &B, rel: &impl Transform3d) -> Option<Vec3> {
        self.minkowski_difference(t).penetrates(&Point, rel)
    }
}

impl<A, B, C> Sdf3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Sdf3d<Point>,
{
    fn sdf(&self, t: &B, rel: &impl Transform3d) -> f32 {
        self.minkowski_difference(t).sdf(&Point, rel)
    }
}

impl<A> Sdf3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: Sdf3d<Point>,
{
    fn sdf(&self, t: &A, rel: &impl Transform3d) -> f32 {
        t.sdf(&Point, rel)
    }
}

impl<A, B, C> Sdf3dVector<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Sdf3dVector<Point>,
{
    fn sdfvector(&self, t: &B, rel: &impl Transform3d) -> Vec3 {
        self.minkowski_difference(t).sdfvector(&Point, rel)
    }
}

impl<A> Sdf3dVector<A> for Point
where
    A: DefaultCol3dImpls,
    A: Sdf3dVector<Point>,
{
    fn sdfvector(&self, t: &A, rel: &impl Transform3d) -> Vec3 {
        t.sdfvector(&Point, rel)
    }
}
