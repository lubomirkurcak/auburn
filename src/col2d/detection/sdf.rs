use super::*;

/// Trait for computing the *scalar* signed-distance between `Self` and `B`.
///
/// # See also
/// * [SdfvRel2d]
pub trait SdfRel2d<B> {
    /// Computes *scalar* signed-distance between `self` and `b` (in `self`-centric space, which
    /// does not matter since `sdf` is rotation-symmetric scalar value).
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `b` - The object to check collision against
    /// * `rel` - Relative transform of `b` in relation to `self`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Ball::with_radius(1.0);
    /// let b = Ball::with_radius(1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdf_rel(&b, &rel), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdfv2d::sdfv].
    fn sdf_rel(&self, t: &B, rel: &impl Transformation2d) -> f32;
}

/// Trait for computing the *scalar* signed-distance between `Self` and `B`.
///
/// # See also
/// * [Sdfv2d]
pub trait Sdf2d<'a, A: 'a, B: 'a, T, BB>
where
    T: Transformation2d + 'a,
    BB: Into<Collider2d<'a, B, T>>,
    A: SdfRel2d<B>,
{
    /// Computes *scalar* signed-distance between `self` and `b`.
    ///
    /// # Arguments
    /// * `b` - The collider to check collision against
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Collider2d {
    ///     shape: &Ball::with_radius(1.0),
    ///     transform: &Vec2::new(0.0, 0.0),
    /// };
    /// let b = Collider2d {
    ///     shape: &Ball::with_radius(1.0),
    ///     transform: &Vec2::new(1.0, 0.0),
    /// };
    /// assert_eq!(a.sdf(b), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdfv2d::sdfv].
    fn sdf(self, b: BB) -> f32;
}

impl<'a, A: 'a, B: 'a, T, AA, BB> Sdf2d<'a, A, B, T, BB> for AA
where
    A: SdfRel2d<B>,
    T: Transformation2d + DeltaTransform + 'a,
    Collider2d<'a, A, T>: From<AA>,
    Collider2d<'a, B, T>: From<BB>,
{
    fn sdf(self, bb: BB) -> f32 {
        let a: Collider2d<'a, A, T> = self.into();
        let b: Collider2d<'a, B, T> = bb.into();
        let rel = a.transform.delta_transform(b.transform);
        a.shape.sdf_rel(b.shape, &rel)
    }
}
