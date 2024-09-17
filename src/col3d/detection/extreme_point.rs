use super::*;

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
