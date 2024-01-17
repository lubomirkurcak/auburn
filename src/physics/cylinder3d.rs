use crate::{
    Ball, Collides3d, ExtremePoint3d, MinkowskiNegationIsIdentity, MinkowskiSum, Penetrates3d,
    Vec2, Vec3,
};

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
        let d = crate::ExtremePoint2d::extreme_point(
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

impl Collides3d<()> for Cylinder3d {
    fn collides(&self, t: &(), rel: &impl crate::Transform3dTrait) -> bool {
        let o = rel.apply_origin();
        if o.z < -self.halfheight || o.z > self.halfheight {
            return false;
        }

        let o2d = Vec2::new(o.x, o.y);

        crate::Collides2d::collides(&self.to_ball(), (), &o2d);
    }
}

impl Collides3d<Cylinder3d> for () {
    fn collides(&self, t: &Cylinder3d, rel: &impl crate::Transform3dTrait) -> bool {
        t.collides(&(), rel)
    }
}

impl Penetrates3d<Cylinder3d> for () {
    fn penetrates(&self, t: &Cylinder3d, rel: &impl crate::Transform3dTrait) -> Option<Vec3> {
        if Collides3d::collides(&(), t, rel) {
            let delta = rel.apply_origin();
            let delta2d = Vec2::new(delta.x, delta.y);
            let delta2d_sq = delta2d.length_squared();
            let z2 = delta.z * delta.z;
            if z2 < delta2d_sq {
                return Some(Vec3::new(0.0, 0.0, delta.z.signum() * t.halfheight));
            } else {
                return crate::Penetrates2d::penetrates(&(), &t.to_ball(), &delta2d)
                    .map(|v| Vec3::new(v.x, v.y, 0.0));
            }
        }
        None
    }
}
impl Penetrates3d<()> for Cylinder3d {
    fn penetrates(&self, _t: &(), rel: &impl Transform3dTrait) -> Option<Vec3> {
        ().penetrates(self, rel)
    }
}
