use super::*;

impl MinkowskiNegationIsIdentity for () {}

impl SymmetricBoundingBox2d for () {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(0.0, 0.0)
    }
}

impl<T: Copy> MinkowskiSum<T> for () {
    type Output = T;

    fn minkowski_sum(&self, t: &T) -> Self::Output {
        *t
    }
}

impl ExtremePoint2d for () {
    fn extreme_point(&self, direction: &Vec2) -> Vec2 {
        Vec2::ZERO
    }
}

impl Sdf2d<()> for () {
    fn sdf(&self, t: &(), rel: &impl Transform2dTrait) -> f32 {
        let delta = rel.apply_origin();
        delta.length()
    }
}

impl Sdf2dVector<()> for () {
    fn sdfvector(&self, t: &(), rel: &impl Transform2dTrait) -> Vec2 {
        rel.apply_origin()
    }
}
