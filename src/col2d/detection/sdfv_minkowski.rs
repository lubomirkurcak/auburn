use super::*;
use crate::col2d::shape::local_minkowski_diff::LocalMinkowskiDiff2d;

pub trait DefaultMinkowski<T: ExtremePoint2d>: ExtremePoint2d {}

/// # Important
/// Calcuates positions in A's local space.
pub trait SdfvMinkowski2d<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool> {
    /// # Important
    /// Calcuates positions in A's local space.
    fn sdfv_minkowski(&self) -> (bool, Vec2);
}

impl<A, B> SdfvCommonRel2d<false, false, B> for A
where
    A: DefaultMinkowski<B>,
    B: ExtremePoint2d,
{
    fn sdfv_common_rel(&self, other: &B, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, false>::sdfv_minkowski(&diff)
    }
}

impl<A, B> SdfvCommonRel2d<false, true, B> for A
where
    A: DefaultMinkowski<B>,
    B: ExtremePoint2d,
{
    fn sdfv_common_rel(&self, other: &B, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, true>::sdfv_minkowski(&diff)
    }
}

impl<A, B> SdfvCommonRel2d<true, false, B> for A
where
    A: DefaultMinkowski<B>,
    B: ExtremePoint2d,
{
    fn sdfv_common_rel(&self, other: &B, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<true, false>::sdfv_minkowski(&diff)
    }
}

impl<A, B> SdfvCommonRel2d<true, true, B> for A
where
    A: DefaultMinkowski<B>,
    B: ExtremePoint2d,
{
    fn sdfv_common_rel(&self, other: &B, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<true, true>::sdfv_minkowski(&diff)
    }
}
