use local_minkowski_diff::LocalMinkowskiDiff2d;

use crate::{debug, error, info, trace, warn};

use super::*;

impl SdfvCommonRel2d<false, false, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, false>::sdfv_minkowski(&diff)
    }
}

impl SdfvCommonRel2d<false, true, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, true>::sdfv_minkowski(&diff)
    }
}

impl SdfvCommonRel2d<true, false, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, true>::sdfv_minkowski(&diff)
    }
}

impl SdfvCommonRel2d<true, true, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<true, true>::sdfv_minkowski(&diff)
    }
}

/*
impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Poly2d> for Poly2d
{
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = MinkowskiDiff2d::new(self, other, rel);
        SdfvMinkowski2d::<COMPUTE_PENETRATION, COMPUTE_DISTANCE>::sdfv(&diff)
    }
}
*/
