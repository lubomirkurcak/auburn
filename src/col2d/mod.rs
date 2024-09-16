//! 2D Collisions
//!
//! # Shapes:
//! * [Ball] - ball
//! * [Box2d] - 2D box
//! * [RoundedBox2d] - 2D rounded box
//! * [Tilemap] - 2D tilemap (requires feature `"tilemap"`)
//!
//! # Collision and Resolution:
//! * [Collides2d::collides]
//! * [Penetrates2d::penetrates]
//! * [Sdf2d::sdf]
//! * [Sdf2dVector::sdfv].
//!
//! # Transformations:
//! * [Translate2d] - translation
//! * [Transform2d] - standard 2D transform
//! * [Isotropic2d] - scale-uniform transform
//! * [AxisTransform2d]
//! * [bevy::prelude::Transform] - bevy transform (requires feature `"bevy"`)

pub use crate::Vec2;

mod detection;
mod shape;
mod transformation2d;

pub use crate::col::*;
pub use detection::*;
pub use shape::*;
pub use transformation2d::*;

#[doc(alias = "Support")]
#[doc(alias = "SupportPoint")]
#[doc(alias = "SupportPoint3d")]
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
    fn sdfv_rel(&self, t: &B, rel: &impl Transformation2d) -> Vec2 {
        self.minkowski_difference(t).sdfv_rel(&Point, rel)
    }
}

impl<A> SdfRel2dVector<A> for Point
where
    A: DefaultCol2dImpls,
    A: SdfRel2dVector<Point>,
{
    fn sdfv_rel(&self, t: &A, rel: &impl Transformation2d) -> Vec2 {
        t.sdfv_rel(&Point, rel)
    }
}
