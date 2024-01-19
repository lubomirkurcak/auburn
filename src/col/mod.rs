use crate::{Quat, Vec2, Vec3};

mod ball;
mod point;
mod transform;

pub use ball::*;
pub use point::*;
pub use transform::*;

/// Trait for computing Minkowski sum.
///
/// # Example
/// ```rust
/// impl MinkowskiSum<Sphere> for Box2d {
///     type Output = Box2d;
///
///     fn minkowski_sum(&self, t: &Box2d) -> Self::Output {
///         Self::Output {
///             halfsize: self.halfsize + t.halfsize,
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
    /// ```rust
    /// let a = Sphere::with_radius(1.0).at_origin();
    /// let b = Sphere::with_radius(1.0).at_position(Vec2::new(1.0, 0.0));
    /// assert_eq!(
    ///     a.minkowski_sum(&b),
    ///     Sphere::with_radius(2.0).at_position(Vec2::new(1.0, 0.0))
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
    /// ```rust
    /// let a = Sphere::with_radius(1.0).at_position(Vec2::new(1.0, 0.0));
    /// assert_eq!(
    ///     a.minkowski_negation();
    ///     Sphere::with_radius(1.0).at_position(Vec2::new(-1.0, 0.0))
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
/// # Default implementations:
///
/// For [MinkowskiNegation]:
/// ```rust
/// impl<T, U, V> MinkowskiDifference<U> for T
/// where
///     T: MinkowskiSum<U, Output = V>,
///     U: MinkowskiNegation,
/// {
///     type Output = V;
///
///     fn minkowski_difference(&self, t: &U) -> Self::Output {
///         self.minkowski_sum(&t.minkowski_negation())
///     }
/// }
/// ```
///
/// For [MinkowskiNegationIsIdentity]:
/// ```rust
/// impl<T, U, V> MinkowskiDifference<U> for T
/// where
///     T: MinkowskiSum<U, Output = V>,
///     U: MinkowskiNegationIsIdentity,
/// {
///     type Output = V;
///     fn minkowski_difference(&self, t: &U) -> Self::Output {
///         self.minkowski_sum(&t)
///     }
/// }
/// ```
///
/// # See also
/// * [MinkowskiSum]
/// * [MinkowskiNegation]
pub trait MinkowskiDifference<T> {
    type Output;
    /// Computes Minkowski difference between `self` and `t`.
    ///
    /// # Example
    /// ```rust
    /// let a = Sphere::with_radius(1.0).at_origin();
    /// let b = Sphere::with_radius(1.0).at_position(Vec2::new(1.0, 0.0));
    /// assert_eq!(
    ///     a.minkowski_difference(&b),
    ///     Sphere::with_radius(2.0).at_position(Vec2::new(-1.0, 0.0))
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

impl<T: MinkowskiNegationIsIdentity> MinkowskiNegation for T {
    fn minkowski_negation(&self) -> Self {
        *self
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
