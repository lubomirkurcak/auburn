use super::*;

/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePointLocalSpace2d {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let ball = Ball::with_radius(2.0);
    /// let point = ball.extreme_point_local_space(Vec2::X);
    /// assert_eq!(point, Vec2::new(2.0, 0.0));
    /// ```
    fn extreme_point_local_space(&self, direction: Vec2) -> Vec2;
}

/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePoint2d<T: Transformation2d> {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let ball = Ball::with_radius(2.0);
    /// let t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// let point = ball.extreme_point(&t, Vec2::X);
    /// assert_eq!(point, Vec2::new(3.0, 0.0));
    /// ```
    fn extreme_point(&self, t: &T, direction: Vec2) -> Vec2;
}

#[cfg(disabled)]
impl<'a, S: ExtremePoint2d, T: Transformation2d + Invertible> ExtremePoint2d
    for Collider2d<'a, S, T>
{
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        let rel = self.transform.inverse().apply(direction);
        self.transform.apply(self.shape.extreme_point(rel))
    }
}
