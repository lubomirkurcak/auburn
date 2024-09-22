use super::*;

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [SdfRel2d]
pub trait SdfvRel2d<T> {
    /// Computes *vector* signed-distance between `self` and `t` in `self`-centric space.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdfv_rel(&b, &rel), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfv_rel(&self, t: &T, rel: &impl Transformation2d) -> (bool, Vec2);
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [Sdf2d]
pub trait Sdfv2d<'a, A: 'a, B: 'a, T, BB>
where
    T: Transformation2d + 'a,
    BB: Into<Collider2d<'a, B, T>>,
    A: SdfvRel2d<B>,
{
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Collider2d {
    ///     shape: &Box2d::with_halfdims(1.0, 1.0),
    ///     transform: &Vec2::new(0.0, 0.0),
    /// };
    /// let b = Collider2d {
    ///     shape: &Box2d::with_halfdims(1.0, 1.0),
    ///     transform: &Vec2::new(1.0, 0.0),
    /// };
    /// assert_eq!(a.sdfv(b), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfv(self, b: BB) -> (bool, Vec2);
}

impl<'a, A: 'a, B: 'a, T, AA, BB> Sdfv2d<'a, A, B, T, BB> for AA
where
    A: SdfvRel2d<B>,
    T: Transformation2d + DeltaTransform + 'a,
    Collider2d<'a, A, T>: From<AA>,
    Collider2d<'a, B, T>: From<BB>,
{
    fn sdfv(self, bb: BB) -> (bool, Vec2) {
        let a: Collider2d<'a, A, T> = self.into();
        let b: Collider2d<'a, B, T> = bb.into();
        let rel = a.transform.delta_transform(b.transform);
        let (collides, sdfv_local) = a.shape.sdfv_rel(b.shape, &rel);
        let sdfv = a.transform.apply_normal(sdfv_local);
        (collides, sdfv)
    }
}
