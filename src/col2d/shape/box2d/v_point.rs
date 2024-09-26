use super::*;

impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Point> for Box2d
{
    fn sdfv_common_rel(&self, _: &Point, rel: &impl Transformation2d) -> (bool, Vec2) {
        let p = rel.apply_origin();
        let px = p.x.abs() - self.halfsize.x;
        let py = p.y.abs() - self.halfsize.y;

        if px.is_sign_negative() && py.is_sign_negative() {
            let mut penetration = Vec2::NAN;

            if COMPUTE_PENETRATION {
                if px > py {
                    penetration = Vec2::new(px * p.x.signum(), 0.0);
                } else {
                    penetration = Vec2::new(0.0, py * p.y.signum());
                }
            }

            (true, penetration)
        } else {
            let mut distance = Vec2::NAN;

            if COMPUTE_DISTANCE {
                let dx = px.max(0.0);
                let dy = py.max(0.0);
                distance = Vec2::new(dx * p.x.signum(), dy * p.y.signum());
            }

            (false, distance)
        }
    }
}

impl SdfRel2d<Point> for Box2d {
    fn sdf_rel(&self, t: &Point, rel: &impl Transformation2d) -> f32 {
        let delta = rel.apply_origin();
        let delta_x = delta.x.abs() - self.halfsize.x;
        let delta_y = delta.y.abs() - self.halfsize.y;

        if self.collides_rel(t, rel) {
            delta_x.max(delta_y)
        } else {
            if delta_x < 0.0 {
                delta_y
            } else if delta_y < 0.0 {
                delta_x
            } else {
                Vec2::new(delta_x, delta_y).length()
            }
        }
    }
}
