
pub trait Invertible {
    /// Inversion of the transformation.
    ///
    /// The operation holds this property:
    /// ```rust
    /// assert_eq!(p, a.inverse().apply(a.apply(p)))
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
    fn compose(&self, other: &Self) -> Self;
}

