//! 3D Collisions
//!
//! # Shapes
//! * [Ball] - ball
//! * [Box3d] - 3D box
//! * [Cylinder3d] - 3D cylinder
//! * [Poly3d] - 3D convex polyhedron
//!
//! # Collision and Resolution
//! * [Collides3d::collides]
//! * [Penetrates3d::penetrates]
//! * [Sdf3d::sdf]
//! * [Sdf3dVector::sdfvector].
//!
//! # Transformations
//! * [Translate3d] - translation
//! * [Transform3d] - standard 3D transform
//! * [Isotropic3d] - scale-uniform transform
//! * [AxisTransform3d]
//! * [bevy::prelude::Transform] - bevy transform (requires feature `"bevy"`)

pub use crate::{Quat, Vec2, Vec3};

mod ball3d;
mod box3d;
mod cylinder3d;
mod point3d;
mod poly3d;
mod transformation3d;

pub use crate::col::*;
pub use ball3d::*;
pub use box3d::*;
pub use cylinder3d::*;
pub use point3d::*;
pub use poly3d::*;
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

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [SymmetricBoundingBox3d]
/// * [PenetratesRel3d]
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
    fn collides_rel(&self, t: &T, rel: &impl Transformation3d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [SymmetricBoundingBox3d]
/// * [Penetrates3d]
pub trait Collides3d<B, T: Transformation3d> {
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
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool;
}

impl<A, B, T> Collides3d<B, T> for A
where
    A: CollidesRel3d<B>,
    T: Transformation3d + DeltaTransform,
{
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [`CollidesRel3d`]
pub trait PenetratesRel3d<B> {
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
    /// assert_eq!(a.penetrates_rel(&b, &rel), Some(Vec3::new(-1.0, 0.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides3d::collides].
    fn penetrates_rel(&self, t: &B, rel: &impl Transformation3d) -> Option<Vec3>;
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [`Collides3d`]
pub trait Penetrates3d<B, T: Transformation3d> {
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
    /// let a_t = Translate3d::from(Vec3::new(0.0, 0.0, 0.0));
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b_t = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(
    ///     a.penetrates(&a_t, &b, &b_t),
    ///     Some(Vec3::new(-1.0, 0.0, 0.0))
    /// );
    /// ```
    ///
    /// # See also
    /// * [Collides3d::collides].
    fn penetrates(&self, transform: &T, other: &B, other_transform: &T) -> Option<Vec3>;
}

impl<A, B, T> Penetrates3d<B, T> for A
where
    A: PenetratesRel3d<B>,
    T: Transformation3d + DeltaTransform,
{
    fn penetrates(&self, transform: &T, t: &B, t_transform: &T) -> Option<Vec3> {
        let rel = transform.delta_transform(t_transform);
        self.penetrates_rel(t, &rel)
    }
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`SdfRel3dVector`]
pub trait SdfRel3d<T> {
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
    /// assert_eq!(a.sdf_rel(&b, &rel), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf3dVector::sdfvector].
    fn sdf_rel(&self, t: &T, rel: &impl Transformation3d) -> f32;
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`SdfRel3d`]
pub trait SdfRel3dVector<T> {
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
    /// assert_eq!(a.sdfvector_rel(&b, &rel), Vec3::new(-1.0, 0.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf3d::sdf].
    fn sdfvector_rel(&self, t: &T, rel: &impl Transformation3d) -> Vec3;
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
    fn sdfvector_rel(&self, t: &B, rel: &impl Transformation3d) -> Vec3 {
        self.minkowski_difference(t).sdfvector_rel(&Point, rel)
    }
}

impl<A> SdfRel3dVector<A> for Point
where
    A: DefaultCol3dImpls,
    A: SdfRel3dVector<Point>,
{
    fn sdfvector_rel(&self, t: &A, rel: &impl Transformation3d) -> Vec3 {
        t.sdfvector_rel(&Point, rel)
    }
}
