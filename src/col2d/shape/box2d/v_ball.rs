use rounded_box2d::RoundedBox2d;

use super::*;

impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Ball> for Box2d
{
    fn sdfv_common_rel(&self, t: &Ball, rel: &impl Transformation2d) -> (bool, Vec2) {
        let rounded_box = RoundedBox2d::new(self.halfsize, t.radius);
        SdfvCommonRel2d::<COMPUTE_PENETRATION, COMPUTE_DISTANCE, _>::sdfv_common_rel(
            &rounded_box,
            &Point,
            rel,
        )
    }
}

impl SdfRel2d<Ball> for Box2d {
    fn sdf_rel(&self, b: &Ball, rel: &impl Transformation2d) -> f32 {
        let b_center = rel.apply_origin();
        let towards_self = -b_center;
        // TODO: Exploit ball symmetry
        let p = b.extreme_point_t(rel, towards_self);
        let px = p.x.abs() - self.halfsize.x;
        let py = p.y.abs() - self.halfsize.y;

        if self.collides_rel(b, rel) {
            px.min(py)
        } else {
            if px.is_sign_negative() {
                py
            } else if py.is_sign_negative() {
                px
            } else {
                Vec2::new(px, py).length()
            }
        }
    }
}
