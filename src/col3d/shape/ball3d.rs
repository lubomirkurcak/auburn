use super::*;

use core::ops::Mul;

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

// Collides

impl CollidesRel3d<Point> for Ball {
    fn collides_rel(&self, _t: &Point, rel: &impl Transformation3d) -> bool {
        rel.apply_origin().length_squared() < self.radius * self.radius
    }
}

impl DefaultCol3dImpls for Ball {}

// Penetrates

impl PenetratesRel3d<Point> for Ball {
    fn penetrates_rel(&self, t: &Point, rel: &impl Transformation3d) -> Option<Vec3> {
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec3::new(self.radius, 0.0, 0.0))
            } else {
                let old_magn = distance_to_center;
                let new_magn = self.radius - distance_to_center;
                let penetration = delta * (new_magn / old_magn);
                let penetration = -penetration;
                Some(penetration)
            }
        } else {
            None
        }
    }
}

impl SdfRel3d<Point> for Ball {
    fn sdf_rel(&self, _t: &Point, rel: &impl Transformation3d) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl SdfRel3dVector<Point> for Ball {
    fn sdfv_rel(&self, _t: &Point, rel: &impl Transformation3d) -> Vec3 {
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
