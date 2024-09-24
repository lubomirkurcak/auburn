use super::*;

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [SdfRel2d]
pub trait SdfvCommonRel2d<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool, T> {
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
    /// assert_eq!(a.sdfv_rel(&b, &rel), (true, Vec2::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfv_common_rel(&self, t: &T, rel: &impl Transformation2d) -> (bool, Option<Vec2>);
}

#[cfg(disabled)]
/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [Sdfv2d]
pub trait SdfvCommon2d<
    'a,
    const COMPUTE_PENETRATION: bool,
    const COMPUTE_DISTANCE: bool,
    A: 'a,
    B: 'a,
    T,
    BB,
> where
    T: Transformation2d + 'a,
    BB: Into<Collider2d<'a, B, T>>,
    A: SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, B>,
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
    /// * [Sdfv2d::sdfv].
    fn sdfv_common(self, b: BB) -> (bool, Option<Vec2>);
}

#[cfg(disabled)]
impl<
        'a,
        const COMPUTE_PENETRATION: bool,
        const COMPUTE_DISTANCE: bool,
        A: 'a,
        B: 'a,
        T,
        AA,
        BB,
    > SdfvCommon2d<'a, COMPUTE_PENETRATION, COMPUTE_DISTANCE, A, B, T, BB> for AA
where
    A: SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, B>,
    T: Transformation2d + DeltaTransform + 'a,
    Collider2d<'a, A, T>: From<AA>,
    Collider2d<'a, B, T>: From<BB>,
{
    fn sdfv_common(self, bb: BB) -> (bool, Option<Vec2>) {
        let a: Collider2d<'a, A, T> = self.into();
        let b: Collider2d<'a, B, T> = bb.into();
        let rel = a.transform.delta_transform(b.transform);
        let (collides, sdfv_local) = a.shape.sdfv_common_rel(b.shape, &rel);
        let sdfv = sdfv_local.map(|x| a.transform.apply_normal(x));
        (collides, sdfv)
    }
}

//

impl<A, B> CollidesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: SdfvCommonRel2d<false, false, B>,
{
    fn collides_rel(&self, b: &B, rel: &impl Transformation2d) -> bool {
        SdfvCommonRel2d::<false, false, B>::sdfv_common_rel(self, b, rel).0
    }
}

impl<A, B> PenetratesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: SdfvCommonRel2d<true, false, B>,
{
    fn penetrates_rel(&self, b: &B, rel: &impl Transformation2d) -> Option<Vec2> {
        SdfvCommonRel2d::<true, false, B>::sdfv_common_rel(self, b, rel).1
    }
}

impl<A, B> SdfvRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: SdfvCommonRel2d<true, true, B>,
{
    fn sdfv_rel(&self, b: &B, rel: &impl Transformation2d) -> (bool, Vec2) {
        let (collides, sdfv) = SdfvCommonRel2d::<true, true, B>::sdfv_common_rel(self, b, rel);
        (collides, sdfv.unwrap())
    }
}
