use super::*;

impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Point> for RoundedBox2d
{
    fn sdfv_common_rel(&self, _: &Point, rel: &impl Transformation2d) -> (bool, Option<Vec2>) {
        let delta = rel.apply_origin();
        let delta_x = delta.x.abs() - self.halfsize.x;
        let delta_y = delta.y.abs() - self.halfsize.y;

        if self.box_part().collides_rel(t, rel) {
            if delta_x > delta_y {
                return Vec2::new((delta_x - self.radius) * delta.x.signum(), 0.0);
            } else {
                return Vec2::new(0.0, (delta_y - self.radius) * delta.y.signum());
            }
        } else {
            if delta_x <= 0.0 {
                return Vec2::new(0.0, (delta_y - self.radius) * delta.y.signum());
            } else if delta_y <= 0.0 {
                return Vec2::new((delta_x - self.radius) * delta.x.signum(), 0.0);
            } else {
                let corner = Vec2::new(delta_x * delta.x.signum(), delta_y * delta.y.signum());
                let corner_length = corner.length();
                let corner = corner * (corner_length - self.radius) / corner_length;
                return corner;
            }
        }
    }
}

impl SdfRel2d<Point> for RoundedBox2d {
    fn sdf_rel(&self, p: &Point, rel: &impl Transformation2d) -> f32 {
        SdfRel2d::sdf_rel(&self.box_part(), p, rel) - self.radius
    }
}
