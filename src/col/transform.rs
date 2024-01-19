//! Imagine you have transformations `A` and `B`.
//! You can calculate the difference between them by `A.inverse().compose(&B)`.

pub trait Invertible {
    /// Inversion of the transformation.
    ///
    /// The operation holds this property:
    /// ```rust
    /// assert_eq!(p, a.inverse().apply(a.apply(p)))
    /// ```
    fn inverse(&self) -> Self;
}

pub trait Composable {
    /// Compose two transformations.
    ///
    /// The operation holds this property:
    /// ```rust
    /// assert_eq!(
    ///     a.apply(b.apply(p)),
    ///     a.compose(&b).apply(p),
    /// )
    /// ```
    fn compose(&self, other: &Self) -> Self;
}

pub trait DeltaTransform {
    fn delta_transform(&self, other: &Self) -> Self;
}

impl<T: Invertible + Composable> DeltaTransform for T {
    fn delta_transform(&self, other: &Self) -> Self {
        self.inverse().compose(other)
    }
}

