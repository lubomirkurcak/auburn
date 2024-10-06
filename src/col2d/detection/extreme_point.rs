use crate::{debug, trace};

use super::*;

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

/// Trait for computing extreme points of a shape along a direction.
pub trait ExtremePointT2d<T: Transformation2d>: ExtremePoint2d {
    /// Computes the farthest point along a direction.
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let ball = Ball::with_radius(2.0);
    /// let t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// let point = ball.extreme_point_t(&t, Vec2::X);
    /// assert_eq!(point, Vec2::new(3.0, 0.0));
    /// ```
    fn extreme_point_t(&self, t: &T, direction: Vec2) -> Vec2 {
        debug!("DefaultCol2dImpls::extreme_point_t");
        let local_direction = t.unapply_normal(direction);
        trace!("local_direction: {:?}", local_direction);
        let local_extreme_point = self.extreme_point(local_direction);
        trace!("local_extreme_point: {:?}", local_extreme_point);
        let extreme_point = t.apply(local_extreme_point);
        trace!("extreme_point: {:?}", extreme_point);
        extreme_point
    }
}
