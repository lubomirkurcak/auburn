use std::ops::Mul;

use crate::{
    Ball, Collides3d, ExtremePoint3d, Penetrates3d, Sdf3d, Sdf3dVector, Transform3dTrait, Vec3,
};

// impl SymmetricBoundingBox3d for Ball {
//     fn symmetric_bounding_box(&self) -> Box3d {
//         Box3d::with_halfdims(self.radius, self.radius)
//     }
// }

impl ExtremePoint3d for Ball {
    fn extreme_point(&self, direction: &Vec3) -> Vec3 {
        self.radius * direction.normalize_or_zero()
    }
}

impl Collides3d<()> for Ball {
    fn collides(&self, _t: &(), rel: &impl Transform3dTrait) -> bool {
        rel.apply_origin().length_squared() < self.radius * self.radius
    }
}
impl Collides3d<Ball> for () {
    fn collides(&self, t: &Ball, delta: &impl Transform3dTrait) -> bool {
        t.collides(&(), delta)
    }
}

impl Penetrates3d<Ball> for () {
    fn penetrates(&self, t: &Ball, rel: &impl Transform3dTrait) -> Option<Vec3> {
        if self.collides(t, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec3::new(t.radius, 0.0, 0.0))
            } else {
                let old_magn = distance_to_center;
                let new_magn = t.radius - distance_to_center;
                let penetration = delta.mul(new_magn / old_magn);
                Some(penetration)
            }
        } else {
            None
        }
    }
}
impl Penetrates3d<()> for Ball {
    fn penetrates(&self, _t: &(), rel: &impl Transform3dTrait) -> Option<Vec3> {
        ().penetrates(self, rel)
    }
}

impl Sdf3d<()> for Ball {
    fn sdf(&self, _t: &(), rel: &impl Transform3dTrait) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl Sdf3dVector<()> for Ball {
    fn sdfvector(&self, _t: &(), rel: &impl Transform3dTrait) -> Vec3 {
        let delta = rel.apply_origin();
        let length = delta.length();
        if length > 0.0 {
            let new_length = length - self.radius;
            delta.mul(new_length / length)
        } else {
            self.radius * Vec3::X
        }
    }
}
