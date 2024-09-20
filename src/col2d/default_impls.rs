use super::*;

// TODO(lubo): These could be simplified with specialization. (RFC 1210)

pub trait DefaultCol2dImpls {}

impl<A, T: Transformation2d> ExtremePoint2d<T> for A
where
    A: DefaultCol2dImpls,
    A: ExtremePointLocalSpace2d,
{
    fn extreme_point(&self, t: &T, direction: Vec2) -> Vec2 {
        dbg!(direction);
        let local_direction = t.unapply_normal(direction);
        dbg!(local_direction);
        let local_extreme_point = self.extreme_point_local_space(local_direction);
        dbg!(local_extreme_point);
        let extreme_point = t.apply(local_extreme_point);
        dbg!(extreme_point);
        extreme_point
    }
}

impl<A> CollidesRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: CollidesRel2d<Point>,
{
    fn collides_rel(&self, other: &A, rel: &impl Transformation2d) -> bool {
        other.collides_rel(&Point, rel)
    }
}

#[cfg(minkowski)]
impl<A, B, C> CollidesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: CollidesRel2d<Point>,
{
    fn collides_rel(&self, t: &B, rel: &impl Transformation2d) -> bool {
        self.minkowski_difference(t).collides_rel(&Point, rel)
    }
}

impl<A> PenetratesRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: PenetratesRel2d<Point>,
{
    fn penetrates_rel(&self, other: &A, rel: &impl Transformation2d) -> Option<Vec2> {
        other.penetrates_rel(&Point, rel) // .map(|v| -v)
    }
}

#[cfg(minkowski)]
impl<A, B, C> PenetratesRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: PenetratesRel2d<Point>,
{
    fn penetrates_rel(&self, t: &B, rel: &impl Transformation2d) -> Option<Vec2> {
        self.minkowski_difference(t).penetrates_rel(&Point, rel)
    }
}

#[cfg(minkowski)]
impl<A, B, C> SdfRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfRel2d<Point>,
{
    fn sdf_rel(&self, t: &B, rel: &impl Transformation2d) -> f32 {
        self.minkowski_difference(t).sdf_rel(&Point, rel)
    }
}

impl<A> SdfRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: SdfRel2d<Point>,
{
    fn sdf_rel(&self, t: &A, rel: &impl Transformation2d) -> f32 {
        t.sdf_rel(&Point, rel)
    }
}

#[cfg(minkowski)]
impl<A, B, C> SdfvRel2d<B> for A
where
    A: DefaultCol2dImpls,
    A: MinkowskiDifference<B, Output = C>,
    C: SdfvRel2d<Point>,
{
    fn sdfv_rel(&self, t: &B, rel: &impl Transformation2d) -> Vec2 {
        self.minkowski_difference(t).sdfv_rel(&Point, rel)
    }
}

impl<A> SdfvRel2d<A> for Point
where
    A: DefaultCol2dImpls,
    A: SdfvRel2d<Point>,
{
    fn sdfv_rel(&self, t: &A, rel: &impl Transformation2d) -> Vec2 {
        t.sdfv_rel(&Point, rel)
    }
}
