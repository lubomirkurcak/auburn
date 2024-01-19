use super::{
    Ball, Box2d, CollidesRel2d, ExtremePoint2d, Penetrates2d, Sdf2d, Sdf2dVector,
    SymmetricBoundingBox2d, Transform2d, Vec2,
};

impl SymmetricBoundingBox2d for () {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(0.0, 0.0)
    }
}

impl ExtremePoint2d for () {
    fn extreme_point(&self, _direction: &Vec2) -> Vec2 {
        Vec2::ZERO
    }
}

impl Sdf2d<()> for () {
    fn sdf(&self, _t: &(), rel: &impl Transform2d) -> f32 {
        let delta = rel.apply_origin();
        delta.length()
    }
}

impl Sdf2dVector<()> for () {
    fn sdfvector(&self, _t: &(), rel: &impl Transform2d) -> Vec2 {
        rel.apply_origin()
    }
}
