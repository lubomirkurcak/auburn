use super::*;

/// Trait for computing the *vector* signed-distance between `Self` and `B`.
///
/// # See also
/// * [SdfRel2d]
pub trait SdfvCommonRel2d<
    const COMPUTE_PENETRATION: bool,
    const COMPUTE_DISTANCE: bool,
    B,
    T: Transformation2d,
>
{
    /// Computes *vector* signed-distance between `self` and `t` in `self`-centric space.
    ///
    /// # Arguments
    /// * `b` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `b`
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
    fn sdfv_common_rel(&self, b: &B, rel: &T) -> (bool, Vec2);
}

//

impl<A, B, T> CollidesRel2d<B, T> for A
where
    A: SdfvCommonRel2d<false, false, B, T>,
    T: Transformation2d,
{
    fn collides_rel(&self, b: &B, rel: &T) -> bool {
        SdfvCommonRel2d::<false, false, B, T>::sdfv_common_rel(self, b, rel).0
    }
}

impl<A, B, T> PenetratesRel2d<B, T> for A
where
    A: SdfvCommonRel2d<true, false, B, T>,
    T: Transformation2d,
{
    fn penetrates_rel(&self, b: &B, rel: &T) -> Option<Vec2> {
        let (collides, sdfv) = SdfvCommonRel2d::<true, false, B, T>::sdfv_common_rel(self, b, rel);
        if collides {
            Some(sdfv)
        } else {
            None
        }
    }
}

impl<A, B, T> DistanceToRel2d<B, T> for A
where
    A: SdfvCommonRel2d<false, true, B, T>,
    T: Transformation2d,
{
    fn distance_to_rel(&self, b: &B, rel: &T) -> Option<Vec2> {
        let (collides, sdfv) = SdfvCommonRel2d::<false, true, B, T>::sdfv_common_rel(self, b, rel);
        if collides {
            None
        } else {
            Some(sdfv)
        }
    }
}

impl<A, B, T> SdfvRel2d<B, T> for A
where
    A: SdfvCommonRel2d<true, true, B, T>,
    T: Transformation2d,
{
    fn sdfv_rel(&self, b: &B, rel: &T) -> (bool, Vec2) {
        SdfvCommonRel2d::<true, true, B, T>::sdfv_common_rel(self, b, rel)
    }
}

//

// trait ReverseCollidesRelImpl<A, B> {}
// impl<A, B, T> CollidesRel2d<B, T> for A
// where
//     A: SdfvCommonRel2d<false, false, B, T>,
//     T: Transformation2d,
// {
//     fn collides_rel(&self, b: &B, rel: &T) -> bool {
//         SdfvCommonRel2d::<false, false, B, T>::sdfv_common_rel(self, b, rel).0
//     }
// }

