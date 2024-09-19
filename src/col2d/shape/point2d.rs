use super::*;

impl SymmetricBoundingBox2d for Point {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(0.0, 0.0)
    }
}

impl ExtremePointLocalSpace2d for Point {
    fn extreme_point_local_space(&self, _direction: Vec2) -> Vec2 {
        Vec2::ZERO
    }
}

// impl Sdf2d<Point> for Point {
//     fn sdf(&self, _t: &Point, rel: &impl Transform2d) -> f32 {
//         let delta = rel.apply_origin();
//         delta.length()
//     }
// }
//
// impl Sdf2dVector<Point> for Point {
//     fn sdfv(&self, _t: &Point, rel: &impl Transform2d) -> Vec2 {
//         rel.apply_origin()
//     }
// }
