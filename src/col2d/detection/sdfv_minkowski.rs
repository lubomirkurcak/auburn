use super::*;

pub trait SdfvMinkowski2d<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool> {
    fn sdfv(&self) -> (bool, Vec2);
}

/*
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
        let (collides, sdfv) = SdfvCommonRel2d::<true, false, B>::sdfv_common_rel(self, b, rel);
        if collides {
            Some(sdfv)
        } else {
            None
        }
    }
}

impl<A, B> DistanceToRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: SdfvCommonRel2d<false, true, B>,
{
    fn distance_to_rel(&self, b: &B, rel: &impl Transformation2d) -> Option<Vec2> {
        let (collides, sdfv) = SdfvCommonRel2d::<false, true, B>::sdfv_common_rel(self, b, rel);
        if collides {
            None
        } else {
            Some(sdfv)
        }
    }
}

impl<A, B> SdfvRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: SdfvCommonRel2d<true, true, B>,
{
    fn sdfv_rel(&self, b: &B, rel: &impl Transformation2d) -> (bool, Vec2) {
        SdfvCommonRel2d::<true, true, B>::sdfv_common_rel(self, b, rel)
    }
}
*/
