use super::*;

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

pub trait DefaultCol3dImpls {}

impl<A> CollidesRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: CollidesRel3d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transformation3d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

impl<A, B, C> CollidesRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel3d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transformation3d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A> PenetratesRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: PenetratesRel3d<Point>,
{
    fn penetrates_rel(&self, other: &A, rel: &impl Transformation3d) -> Option<Vec3> {
        other.penetrates_rel(&Point, rel) // .map(|v| -v)
    }
}

impl<A, B, C> PenetratesRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: PenetratesRel3d<Point>,
{
    fn penetrates_rel(&self, t: &B, rel: &impl Transformation3d) -> Option<Vec3> {
        self.minkowski_difference(t).penetrates_rel(&Point, rel)
    }
}

impl<A, B, C> SdfRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfRel3d<Point>,
{
    fn sdf_rel(&self, t: &B, rel: &impl Transformation3d) -> f32 {
        self.minkowski_difference(t).sdf_rel(&Point, rel)
    }
}

impl<A> SdfRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: SdfRel3d<Point>,
{
    fn sdf_rel(&self, t: &A, rel: &impl Transformation3d) -> f32 {
        t.sdf_rel(&Point, rel)
    }
}

impl<A, B, C> SdfvRel3d<B> for A
where
    A: DefaultCol3dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfvRel3d<Point>,
{
    fn sdfv_rel(&self, t: &B, rel: &impl Transformation3d) -> Vec3 {
        self.minkowski_difference(t).sdfv_rel(&Point, rel)
    }
}

impl<A> SdfvRel3d<A> for Point
where
    A: DefaultCol3dImpls,
    A: SdfvRel3d<Point>,
{
    fn sdfv_rel(&self, t: &A, rel: &impl Transformation3d) -> Vec3 {
        t.sdfv_rel(&Point, rel)
    }
}
