use super::*;

impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Point> for Ball
{
    fn sdfv_common_rel(&self, _: &Point, rel: &impl Transformation2d) -> (bool, Option<Vec2>) {
        let delta = rel.apply_origin();
        let length = delta.length();
        if length > 0.0 {
            let collides = length < self.radius;
            let new_length = length - self.radius;
            (collides, Some(delta * (new_length / length)))
        } else {
            (true, Some(self.radius * Vec2::X))
        }
    }
}

impl SdfRel2d<Point> for Ball {
    fn sdf_rel(&self, _: &Point, rel: &impl Transformation2d) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}
