use super::*;

impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Ball> for Ball
{
    fn sdfv_common_rel(&self, b: &Ball, rel: &impl Transformation2d) -> (bool, Option<Vec2>) {
        let radius = self.radius + b.radius;
        let delta = rel.apply_origin();
        let length = delta.length();
        if length > 0.0 {
            let collides = length < radius;
            let new_length = length - radius;
            (collides, Some(delta * (new_length / length)))
        } else {
            (true, Some(radius * Vec2::X))
        }
    }
}

impl SdfRel2d<Ball> for Ball {
    fn sdf_rel(&self, b: &Ball, rel: &impl Transformation2d) -> f32 {
        let radius = self.radius + b.radius;
        let delta = rel.apply_origin();
        delta.length() - radius
    }
}
