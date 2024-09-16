//! 3D Collisions
//!
//! # Shapes
//! * [Ball] - ball
//! * [Box3d] - 3D box
//! * [Cylinder3d] - 3D cylinder
//!
//! # Collision and Resolution
//! * [Collides3d::collides]
//! * [Penetrates3d::penetrates]
//! * [Sdf3d::sdf]
//! * [Sdf3dVector::sdfv].
//!
//! # Transformations
//! * [Translate3d] - translation
//! * [Transform3d] - standard 3D transform
//! * [Isotropic3d] - scale-uniform transform
//! * [AxisTransform3d]
//! * [bevy::prelude::Transform] - bevy transform (requires feature `"bevy"`)

pub use crate::{Quat, Vec2, Vec3};

mod detection;
mod shape;
mod transformation3d;

pub use crate::col::*;
pub use detection::*;
pub use shape::*;
pub use transformation3d::*;

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
    /// assert_eq!(
    ///     ball.extreme_point(Vec3::X),
    ///     Vec3::new(2.0, 0.0, 0.0)
    /// );
    /// ```
    fn extreme_point(&self, direction: Vec3) -> Vec3;
}

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

trait DefaultCol3dImpls {}

impl<A> CollidesRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: CollidesRel3d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transformation3d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

impl<A, B, C> CollidesRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel3d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transformation3d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A> PenetratesRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: PenetratesRel3d<Point>,
{
    fn penetrates_rel(&self, other: &A, rel: &impl Transformation3d) -> Option<Vec3> {
        other.penetrates_rel(&Point, rel) // .map(|v| -v)
    }
}

impl<A, B, C> PenetratesRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: PenetratesRel3d<Point>,
{
    fn penetrates_rel(&self, t: &B, rel: &impl Transformation3d) -> Option<Vec3> {
        self.minkowski_difference(t).penetrates_rel(&Point, rel)
    }
}

impl<A, B, C> SdfRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfRel3d<Point>,
{
    fn sdf_rel(&self, t: &B, rel: &impl Transformation3d) -> f32 {
        self.minkowski_difference(t).sdf_rel(&Point, rel)
    }
}

impl<A> SdfRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: SdfRel3d<Point>,
{
    fn sdf_rel(&self, t: &A, rel: &impl Transformation3d) -> f32 {
        t.sdf_rel(&Point, rel)
    }
}

impl<A, B, C> SdfRel3dVector<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfRel3dVector<Point>,
{
    fn sdfv_rel(&self, t: &B, rel: &impl Transformation3d) -> Vec3 {
        self.minkowski_difference(t).sdfv_rel(&Point, rel)
    }
}

impl<A> SdfRel3dVector<A> for Point
where
    A: DefaultCol3dImpls,
    A: SdfRel3dVector<Point>,
{
    fn sdfv_rel(&self, t: &A, rel: &impl Transformation3d) -> Vec3 {
        t.sdfv_rel(&Point, rel)
    }
}
