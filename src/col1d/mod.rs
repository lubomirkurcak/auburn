//! 1D Collisions.
//!
//! # Generic
//!
//! * [Point] - point (the origin)
//! * [Ball] - ball
//!
//! # 1D
//!
//! Collision and Resolution:
//! * [Collides1d::collides]
//! * [Penetrates1d::penetrates]
//! * [Sdf1d::sdf]
//! * [Sdf1dVector::sdfvector].
//!
//! Shapes:
//! * [Box1d] - 1D box

pub use crate::Vec2;

pub type Vec1 = f32;

mod ball1d;
mod box1d;
mod point1d;
mod transform1d;

pub use crate::col::*;
pub use ball1d::*;
pub use box1d::*;
pub use point1d::*;
pub use transform1d::*;

/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePoint1d {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```
    /// # use auburn::col1d::*;
    /// let ball = Ball::with_radius(2.0);
    /// let point = ball.extreme_point(Vec1::X);
    /// assert_eq!(point, Vec1::new(2.0, 0.0));
    /// ```
    fn extreme_point(&self, direction: Vec1) -> Vec1;
}

// pub trait Support1d<T> {
//     fn support(&self, other: &T, direction: Vec1) -> Vec1;
// }
//
// impl<T, U> Support1d<U> for T
// where
//     T: ExtremePoint1d,
//     U: ExtremePoint1d,
// {
//     fn support(&self, other: &U, direction: Vec1) -> Vec1 {
//         self.extreme_point(direction) - other.extreme_point(-direction)
//     }
// }

/// Trait for computing bounding box of a shape.
///
/// # See also
/// * [Collides1d]
pub trait SymmetricBoundingBox1d {
    /// Computes the bounding box.
    ///
    /// # Example
    /// ```
    /// use auburn::col1d::*;
    /// assert_eq!(
    ///     Ball::with_radius(1.0).symmetric_bounding_box(),
    ///     Box1d::with_halfdims(1.0, 1.0),
    /// );
    /// ```
    ///
    /// # See also
    /// * [Collides1d::collides].
    fn symmetric_bounding_box(&self) -> Box1d;
}

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [BoundingBox1d]
/// * [Penetrates1d]
pub trait CollidesRel1d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col1d::*;
    /// let a = Box1d::with_halfdims(1.0, 1.0);
    /// let b = Box1d::with_halfdims(1.0, 1.0);
    /// let rel = Translate1d::from(Vec1::new(1.0, 0.0));
    /// assert!(a.collides_rel(&b, &rel))
    /// ```
    ///
    /// # See also
    /// * [Penetrates1d::penetrates].
    fn collides_rel(&self, t: &T, rel: &impl Transform1d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # Limitations
/// Currently, transformation types must be the same for both `Self` and `T`.
///
/// # See also
/// * [BoundingBox1d]
/// * [Penetrates1d]
pub trait Collides1d<T, U: Transform1d> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `delta` - The vector from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col1d::*;
    /// let a = Box1d::with_halfdims(1.0, 1.0);
    /// let a_t = Translate1d::from(Vec1::new(0.0, 0.0));
    /// let b = Box1d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate1d::from(Vec1::new(1.0, 0.0));
    /// assert!(a.collides(&a_t, &b, &b_t))
    /// ```
    ///
    /// # See also
    /// * [Penetrates1d::penetrates].
    fn collides(&self, transform: &U, t: &T, t_transform: &U) -> bool;
}

impl<A, B, T> Collides1d<B, T> for A
where
    A: CollidesRel1d<B>,
    T: Transform1d + DeltaTransform,
{
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [`Collides1d`]
pub trait Penetrates1d<T> {
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col1d::*;
    /// let a = Box1d::with_halfdims(1.0, 1.0);
    /// let b = Box1d::with_halfdims(1.0, 1.0);
    /// let rel = Translate1d::from(Vec1::new(1.0, 0.0));
    /// assert_eq!(a.penetrates(&b, &rel), Some(Vec1::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides1d::collides].
    fn penetrates(&self, t: &T, rel: &impl Transform1d) -> Option<Vec1>;
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`Sdf1dVector`]
pub trait Sdf1d<T> {
    /// Computes *scalar* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `t` - The object to check collision against
    /// * `delta` - Transform of `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col1d::*;
    /// let a = Box1d::with_halfdims(1.0, 1.0);
    /// let b = Box1d::with_halfdims(1.0, 1.0);
    /// let rel = Translate1d::from(Vec1::new(1.0, 0.0));
    /// assert_eq!(a.sdf(&b, &rel), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf1dVector::sdfvector].
    fn sdf(&self, t: &T, rel: &impl Transform1d) -> f32;
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`Sdf1d`]
pub trait Sdf1dVector<T> {
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col1d::*;
    /// let a = Box1d::with_halfdims(1.0, 1.0);
    /// let b = Box1d::with_halfdims(1.0, 1.0);
    /// let rel = Translate1d::from(Vec1::new(1.0, 0.0));
    /// assert_eq!(a.sdfvector(&b, &rel), Vec1::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf1d::sdf].
    fn sdfvector(&self, t: &T, rel: &impl Transform1d) -> Vec1;
}

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

trait DefaultCol1dImpls {}

impl<A> CollidesRel1d<A> for Point
where
    A: DefaultCol1dImpls,
    A: CollidesRel1d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transform1d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

impl<A, B, C> CollidesRel1d<B> for A
where
    A: DefaultCol1dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel1d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transform1d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A> Penetrates1d<A> for Point
where
    A: DefaultCol1dImpls,
    A: Penetrates1d<Point>,
{
    fn penetrates(&self, other: &A, rel: &impl Transform1d) -> Option<Vec1> {
        other.penetrates(&Point, rel) // .map(|v| -v)
    }
}

impl<A, B, C> Penetrates1d<B> for A
where
    A: DefaultCol1dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Penetrates1d<Point>,
{
    fn penetrates(&self, t: &B, rel: &impl Transform1d) -> Option<Vec1> {
        self.minkowski_difference(t).penetrates(&Point, rel)
    }
}

impl<A, B, C> Sdf1d<B> for A
where
    A: DefaultCol1dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Sdf1d<Point>,
{
    fn sdf(&self, t: &B, rel: &impl Transform1d) -> f32 {
        self.minkowski_difference(t).sdf(&Point, rel)
    }
}

impl<A> Sdf1d<A> for Point
where
    A: DefaultCol1dImpls,
    A: Sdf1d<Point>,
{
    fn sdf(&self, t: &A, rel: &impl Transform1d) -> f32 {
        t.sdf(&Point, rel)
    }
}

impl<A, B, C> Sdf1dVector<B> for A
where
    A: DefaultCol1dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Sdf1dVector<Point>,
{
    fn sdfvector(&self, t: &B, rel: &impl Transform1d) -> Vec1 {
        self.minkowski_difference(t).sdfvector(&Point, rel)
    }
}

impl<A> Sdf1dVector<A> for Point
where
    A: DefaultCol1dImpls,
    A: Sdf1dVector<Point>,
{
    fn sdfvector(&self, t: &A, rel: &impl Transform1d) -> Vec1 {
        t.sdfvector(&Point, rel)
    }
}

