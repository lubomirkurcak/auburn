//! 2D Collisions
//!
//! # Shapes:
//! * [Ball] - ball
//! * [Box2d] - 2D box
//! * [RoundedBox2d] - 2D rounded box
//! * [Poly2d] - 2D convex polygon
//! * [Tilemap] - 2D tilemap
//!
//! # Collision and Resolution:
//! * [Collides2d::collides]
//! * [Penetrates2d::penetrates]
//! * [Sdf2d::sdf]
//! * [Sdf2dVector::sdfvector].
//!
//! # Transformations:
//! * [Translate2d] - translation
//! * [Transform2d] - standard 2D transform
//! * [Isotropic2d] - scale-uniform transform
//! * [AxisTransform2d]
//! * [bevy::prelude::Transform] - bevy transform (requires feature `"bevy"`)

pub use crate::Vec2;

mod ball2d;
mod box2d;
mod ellipse2d;
mod point2d;
mod poly2d;
mod rounded_box2d;
mod tilemap;
mod transformation2d;

pub use crate::col::*;
pub use ball2d::*;
pub use box2d::*;
// pub use ellipse2d::*;
pub use point2d::*;
pub use poly2d::*;
pub use rounded_box2d::*;
pub use tilemap::*;
pub use transformation2d::*;

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
/// * [SymmetricBoundingBox2d]
/// * [PenetratesRel2d]
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
    fn collides_rel(&self, t: &T, rel: &impl Transformation2d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # Limitations
/// Currently, transformation types must be the same for both `Self` and `T`.
///
/// # See also
/// * [SymmetricBoundingBox2d]
/// * [Penetrates2d]
pub trait Collides2d<B, T: Transformation2d> {
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
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool;
}

impl<A, B, T> Collides2d<B, T> for A
where
    A: CollidesRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [Collides2d]
pub trait PenetratesRel2d<T> {
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
    /// assert_eq!(a.penetrates_rel(&b, &rel), Some(Vec2::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn penetrates_rel(&self, t: &T, rel: &impl Transformation2d) -> Option<Vec2>;
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [Collides2d]
pub trait Penetrates2d<B, T: Transformation2d> {
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
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.penetrates(&a_t, &b, &b_t), Some(Vec2::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn penetrates(&self, a_t: &T, b: &B, b_t: &T) -> Option<Vec2>;
}

impl<A, B, T> Penetrates2d<B, T> for A
where
    A: PenetratesRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn penetrates(&self, a_transform: &T, b: &B, b_transform: &T) -> Option<Vec2> {
        let rel = a_transform.delta_transform(b_transform);
        self.penetrates_rel(b, &rel)
    }
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [SdfRel2dVector]
pub trait SdfRel2d<T> {
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
    /// assert_eq!(a.sdf_rel(&b, &rel), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf2dVector::sdfvector].
    fn sdf_rel(&self, t: &T, rel: &impl Transformation2d) -> f32;
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [Sdf2dVector]
pub trait Sdf2d<B, T: Transformation2d> {
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
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdf(&a_t, &b, &b_t), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf2dVector::sdfvector].
    fn sdf(&self, a_t: &T, b: &B, b_t: &T) -> f32;
}

impl<A, B, T> Sdf2d<B, T> for A
where
    A: SdfRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn sdf(&self, a_transform: &T, b: &B, b_transform: &T) -> f32 {
        let rel = a_transform.delta_transform(b_transform);
        self.sdf_rel(b, &rel)
    }
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [SdfRel2d]
pub trait SdfRel2dVector<T> {
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
    /// assert_eq!(a.sdfvector_rel(&b, &rel), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfvector_rel(&self, t: &T, rel: &impl Transformation2d) -> Vec2;
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [Sdf2d]
pub trait Sdf2dVector<B, T: Transformation2d> {
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
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdfvector(&a_t, &b, &b_t), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfvector(&self, a_t: &T, b: &B, b_t: &T) -> Vec2;
}

impl<A, B, T> Sdf2dVector<B, T> for A
where
    A: SdfRel2dVector<B>,
    T: Transformation2d + DeltaTransform,
{
    fn sdfvector(&self, a_transform: &T, b: &B, b_transform: &T) -> Vec2 {
        let rel = a_transform.delta_transform(b_transform);
        self.sdfvector_rel(b, &rel)
    }
}

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

trait DefaultCol2dImpls {}

impl<A> CollidesRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: CollidesRel2d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transformation2d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

impl<A, B, C> CollidesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel2d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transformation2d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A> PenetratesRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: PenetratesRel2d<Point>,
{
    fn penetrates_rel(&self, other: &A, rel: &impl Transformation2d) -> Option<Vec2> {
        other.penetrates_rel(&Point, rel) // .map(|v| -v)
    }
}

impl<A, B, C> PenetratesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: PenetratesRel2d<Point>,
{
    fn penetrates_rel(&self, t: &B, rel: &impl Transformation2d) -> Option<Vec2> {
        self.minkowski_difference(t).penetrates_rel(&Point, rel)
    }
}

impl<A, B, C> SdfRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfRel2d<Point>,
{
    fn sdf_rel(&self, t: &B, rel: &impl Transformation2d) -> f32 {
        self.minkowski_difference(t).sdf_rel(&Point, rel)
    }
}

impl<A> SdfRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: SdfRel2d<Point>,
{
    fn sdf_rel(&self, t: &A, rel: &impl Transformation2d) -> f32 {
        t.sdf_rel(&Point, rel)
    }
}

impl<A, B, C> SdfRel2dVector<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfRel2dVector<Point>,
{
    fn sdfvector_rel(&self, t: &B, rel: &impl Transformation2d) -> Vec2 {
        self.minkowski_difference(t).sdfvector_rel(&Point, rel)
    }
}

impl<A> SdfRel2dVector<A> for Point
where
    A: DefaultCol2dImpls,
    A: SdfRel2dVector<Point>,
{
    fn sdfvector_rel(&self, t: &A, rel: &impl Transformation2d) -> Vec2 {
        t.sdfvector_rel(&Point, rel)
    }
}
