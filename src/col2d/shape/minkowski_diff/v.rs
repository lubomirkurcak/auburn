use super::*;
use crate::col2d::detection::SdfvMinkowski2d;

impl<A, B, T> SdfvMinkowski2d<false, false> for MinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        let local_diff = self.to_local();
        let (collides, _) = SdfvMinkowski2d::<false, false>::sdfv_minkowski(&local_diff);
        (collides, Vec2::NAN)
    }
}

impl<A, B, T> SdfvMinkowski2d<false, true> for MinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        let local_diff = self.to_local();
        let (collides, local) = SdfvMinkowski2d::<false, true>::sdfv_minkowski(&local_diff);
        let result = self.a_t.apply(local);
        (collides, result)
    }
}

impl<A, B, T> SdfvMinkowski2d<true, false> for MinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        let local_diff = self.to_local();
        let (collides, local) = SdfvMinkowski2d::<true, false>::sdfv_minkowski(&local_diff);
        let result = self.a_t.apply(local);
        (collides, result)
    }
}

impl<A, B, T> SdfvMinkowski2d<true, true> for MinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        let local_diff = self.to_local();
        let (collides, local) = SdfvMinkowski2d::<true, true>::sdfv_minkowski(&local_diff);
        let result = self.a_t.apply(local);
        (collides, result)
    }
}
