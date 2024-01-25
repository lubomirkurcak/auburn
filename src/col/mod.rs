mod ball;
mod point;
mod transform;

pub use ball::*;
pub use point::*;
pub use transform::*;

/// Trait for computing Minkowski sum.
///
/// # Example
/// ```
/// use auburn::col2d::*;
/// struct Ball {
///     radius: f32,
/// }
/// impl MinkowskiSum<Ball> for Ball {
///     type Output = Ball;
///
///     fn minkowski_sum(&self, b: &Ball) -> Self::Output {
///         Self::Output {
///             radius: self.radius + b.radius,
///         }
///     }
/// }
/// ```
///
/// # See also
/// * [MinkowskiNegation]
/// * [MinkowskiDifference]
pub trait MinkowskiSum<T> {
    type Output;
    /// Computes Minkowski sum of `self` and `t`.
    ///
    /// # Example
    /// ```
    /// use auburn::col2d::*;
    /// let a = Ball::with_radius(1.0);
    /// let b = Box2d::with_halfdims(0.5, 0.5);
    /// assert_eq!(
    ///     a.minkowski_sum(&b),
    ///     RoundedBox2d::new(Vec2::new(0.5, 0.5), 1.0)
    /// );
    /// ```
    ///
    /// # See also
    /// * [MinkowskiNegation::minkowski_negation]
    /// * [MinkowskiDifference::minkowski_difference]
    fn minkowski_sum(&self, t: &T) -> Self::Output;
}

/// Marker trait for when the Minkowski negation is identity.
/// This would usually be true for shapes centered at the origin and symmetric around it.
///
/// # See also
/// * [MinkowskiNegation]
/// * [MinkowskiSum]
/// * [MinkowskiDifference]
pub trait MinkowskiNegationIsIdentity: Copy {}

/// Trait for computing Minkowski negation.
///
/// # See also
/// * [MinkowskiNegationIsIdentity]
/// * [MinkowskiSum]
/// * [MinkowskiDifference]
pub trait MinkowskiNegation {
    /// Computes Minkowski negation. (Reflection about the origin)
    ///
    /// # Example
    ///
    /// ```
    /// use auburn::col2d::*;
    /// let a = Ball::with_radius(1.0);
    /// assert_eq!(
    ///     a.minkowski_negation(),
    ///     Ball::with_radius(1.0)
    /// );
    /// ```
    ///
    /// # See also
    /// * [MinkowskiSum::minkowski_sum]
    /// * [MinkowskiDifference::minkowski_difference]
    fn minkowski_negation(&self) -> Self;
}

/// Trait for computing Minkowski difference.
///
/// # See also
/// * [MinkowskiSum]
/// * [MinkowskiNegation]
pub trait MinkowskiDifference<T> {
    type Output;
    /// Computes Minkowski difference between `self` and `t`.
    ///
    /// # Example
    /// ```
    /// use auburn::col2d::*;
    /// let a = Ball::with_radius(1.0);
    /// let b = Ball::with_radius(1.0);
    /// assert_eq!(
    ///     a.minkowski_difference(&b),
    ///     Ball::with_radius(2.0)
    /// );
    /// ```
    ///
    /// # Notes
    /// Useful for collisions, since when the difference contains the origin, the shapes overlap.
    ///
    /// Has a default implementation when `Self : MinkowskiSum<T>` and `T : MinkowskiNegation`.
    ///
    /// # See also
    /// * [MinkowskiDifference]
    /// * [MinkowskiSum::minkowski_sum]
    /// * [MinkowskiNegation::minkowski_negation]
    fn minkowski_difference(&self, t: &T) -> Self::Output;
}

pub trait MinkowskiDifferenceLifetimed<'a, T: 'a> {
    type Output: 'a;
    fn minkowski_difference_lt(&'a self, t: &'a T) -> Self::Output;
}

impl<T: MinkowskiNegationIsIdentity> MinkowskiNegation for T {
    fn minkowski_negation(&self) -> Self {
        *self
    }
}

impl<'l, A, B: 'l, C: 'l> MinkowskiDifferenceLifetimed<'l, B> for A
where
    A: MinkowskiDifference<B, Output = C>,
{
    type Output = C;

    fn minkowski_difference_lt(&'_ self, t: &'l B) -> Self::Output {
        MinkowskiDifference::minkowski_difference(self, t)
    }
}

impl<T, U, V> MinkowskiDifference<U> for T
where
    T: MinkowskiSum<U, Output = V>,
    U: MinkowskiNegation,
{
    type Output = V;
    fn minkowski_difference(&self, t: &U) -> Self::Output {
        self.minkowski_sum(&t.minkowski_negation())
    }
}
