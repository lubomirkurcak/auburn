use std::ops::Mul;

use super::{
    Ball, CollidesRel3d, ExtremePoint3d, PenetratesRel3d, SdfRel3d, SdfRel3dVector, Transformation3d, Vec3,
};

// impl SymmetricBoundingBox3d for Ball {
//     fn symmetric_bounding_box(&self) -> Box3d {
//         Box3d::with_halfdims(self.radius, self.radius)
//     }
// }

impl ExtremePoint3d for Ball {
    fn extreme_point(&self, direction: Vec3) -> Vec3 {
        self.radius * direction.normalize_or_zero()
    }
}

impl CollidesRel3d<()> for Ball {
    fn collides_rel(&self, _t: &(), rel: &impl Transformation3d) -> bool {
        rel.apply_origin().length_squared() < self.radius * self.radius
    }
}
impl CollidesRel3d<Ball> for () {
    fn collides_rel(&self, t: &Ball, delta: &impl Transformation3d) -> bool {
        t.collides_rel(&(), delta)
    }
}

impl PenetratesRel3d<Ball> for () {
    fn penetrates_rel(&self, t: &Ball, rel: &impl Transformation3d) -> Option<Vec3> {
        if self.collides_rel(t, rel) {
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
impl PenetratesRel3d<()> for Ball {
    fn penetrates_rel(&self, _t: &(), rel: &impl Transformation3d) -> Option<Vec3> {
        ().penetrates_rel(self, rel)
    }
}

impl SdfRel3d<()> for Ball {
    fn sdf_rel(&self, _t: &(), rel: &impl Transformation3d) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl SdfRel3dVector<()> for Ball {
    fn sdfvector_rel(&self, _t: &(), rel: &impl Transformation3d) -> Vec3 {
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
