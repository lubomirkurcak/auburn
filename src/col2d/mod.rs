//! 2D Collisions.
//!
//! # Generic
//!
//! * [Point] - point (the origin)
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
// mod tilemap;
mod transform2d;

pub use crate::col::*;
pub use ball2d::*;
pub use box2d::*;
pub use point2d::*;
pub use poly2d::*;
pub use rounded_box2d::*;
// pub use tilemap::*;
pub use transform2d::*;

/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePoint2d {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let ball = Ball::with_radius(2.0);
    /// let point = ball.extreme_point(Vec2::X);
    /// assert_eq!(point, Vec2::new(2.0, 0.0));
    /// ```
    fn extreme_point(&self, direction: Vec2) -> Vec2;
}

// pub trait Support2d<T> {
//     fn support(&self, other: &T, direction: Vec2) -> Vec2;
// }
//
// impl<T, U> Support2d<U> for T
// where
//     T: ExtremePoint2d,
//     U: ExtremePoint2d,
// {
//     fn support(&self, other: &U, direction: Vec2) -> Vec2 {
//         self.extreme_point(direction) - other.extreme_point(-direction)
//     }
// }

/// Trait for computing bounding box of a shape.
///
/// # See also
/// * [Collides2d]
pub trait SymmetricBoundingBox2d {
    /// Computes the bounding box.
    ///
    /// # Example
    /// ```
    /// use auburn::col2d::*;
    /// assert_eq!(
    ///     Ball::with_radius(1.0).symmetric_bounding_box(),
    ///     Box2d::with_halfdims(1.0, 1.0),
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
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert!(a.collides_rel(&b, &rel))
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides_rel(&self, t: &T, rel: &impl Transform2d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # Limitations
/// Currently, transformation types must be the same for both `Self` and `T`.
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
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert!(a.collides(&a_t, &b, &b_t))
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides(&self, transform: &U, t: &T, t_transform: &U) -> bool;
}

impl<A, B, T> Collides2d<B, T> for A
where
    A: CollidesRel2d<B>,
    T: Transform2d + DeltaTransform,
{
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool {
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
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.penetrates(&b, &rel), Some(Vec2::new(-1.0, 0.0)));
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
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdf(&b, &rel), -1.0);
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
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdfvector(&b, &rel), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfvector(&self, t: &T, rel: &impl Transform2d) -> Vec2;
}

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

trait DefaultCol2dImpls {}

impl<A> CollidesRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: CollidesRel2d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transform2d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

impl<A, B, C> CollidesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel2d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transform2d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A, B, C> Penetrates2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Penetrates2d<Point>,
{
    fn penetrates(&self, t: &B, rel: &impl Transform2d) -> Option<Vec2> {
        self.minkowski_difference(t).penetrates(&Point, rel)
    }
}

impl<A, B, C> Sdf2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Sdf2d<Point>,
{
    fn sdf(&self, t: &B, rel: &impl Transform2d) -> f32 {
        self.minkowski_difference(t).sdf(&Point, rel)
    }
}

impl<A> Sdf2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: Sdf2d<Point>,
{
    fn sdf(&self, t: &A, rel: &impl Transform2d) -> f32 {
        t.sdf(&Point, rel)
    }
}

impl<A, B, C> Sdf2dVector<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: Sdf2dVector<Point>,
{
    fn sdfvector(&self, t: &B, rel: &impl Transform2d) -> Vec2 {
        self.minkowski_difference(t).sdfvector(&Point, rel)
    }
}

impl<A> Sdf2dVector<A> for Point
where
    A: DefaultCol2dImpls,
    A: Sdf2dVector<Point>,
{
    fn sdfvector(&self, t: &A, rel: &impl Transform2d) -> Vec2 {
        t.sdfvector(&Point, rel)
    }
}
