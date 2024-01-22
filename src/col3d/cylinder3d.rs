use super::*;

/// Upright 3D cylinder given by [height] and [radius].
#[derive(Default, Clone, Copy)]
struct Cylinder3d {
    halfheight: f32,
    radius: f32,
}

impl Cylinder3d {
    pub fn new(height: f32, radius: f32) -> Self {
        Self {
            halfheight: height / 2.0,
            radius,
        }
    }

    pub fn to_ball(&self) -> Ball {
        Ball::new(self.radius)
    }
}

impl ExtremePoint3d for Cylinder3d {
    fn extreme_point(&self, direction: &Vec3) -> Vec3 {
        let d = crate::col2d::ExtremePoint2d::extreme_point(
            &self.to_ball(),
            &Vec2::new(direction.x, direction.y),
        );

        let z = if direction.z > 0.0 {
            self.halfheight
        } else {
            -self.halfheight
        };

        Vec3::new(d.x, d.y, z)
    }
}

impl MinkowskiSum<Cylinder3d> for Cylinder3d {
    type Output = Self;

    fn minkowski_sum(&self, other: &Cylinder3d) -> Self::Output {
        Self::new(
            self.halfheight + other.halfheight,
            self.radius + other.radius,
        )
    }
}

impl MinkowskiNegationIsIdentity for Cylinder3d {}

impl CollidesRel3d<Point> for Cylinder3d {
    fn collides_rel(&self, t: &Point, rel: &impl Transform3d) -> bool {
        let o = rel.apply_origin();
        if o.z < -self.halfheight || o.z > self.halfheight {
            return false;
        }

        let o2d = Vec2::new(o.x, o.y);

        crate::col2d::CollidesRel2d::collides_rel(&self.to_ball(), &Point, &o2d)
    }
}

impl Penetrates3d<Point> for Cylinder3d {
    fn penetrates(&self, t: &Point, rel: &impl Transform3d) -> Option<Vec3> {
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let delta2d = Vec2::new(delta.x, delta.y);
            let delta2d_sq = delta2d.length_squared();
            let z2 = delta.z * delta.z;
            if z2 < delta2d_sq {
                return Some(Vec3::new(0.0, 0.0, delta.z.signum() * self.halfheight));
            } else {
                return crate::col2d::Penetrates2d::penetrates(&self.to_ball(), t, &delta2d)
                    .map(|v| Vec3::new(v.x, v.y, 0.0));
            }
        }
        None
    }
}
