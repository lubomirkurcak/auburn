use super::*;

impl MinkowskiSum<Box2d> for Ball {
    type Output = RoundedBox2d;
    fn minkowski_sum(&self, t: &Box2d) -> Self::Output {
        Self::Output {
            halfsize: t.halfsize,
            radius: self.radius,
        }
    }
}

impl SymmetricBoundingBox2d for Ball {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(self.radius, self.radius)
    }
}

impl ExtremePoint2d for Ball {
    fn extreme_point(&self, direction: &Vec2) -> Vec2 {
        self.radius * direction.normalize_or_zero()
    }
}

impl CollidesRel2d<()> for Ball {
    fn collides_rel(&self, _t: &(), rel: &impl Transform2d) -> bool {
        rel.apply_origin().length_squared() < self.radius * self.radius
    }
}
// impl CollidesRel2d<Ball> for () {
//     fn collides_rel(&self, t: &Ball, delta: &impl Transform2d) -> bool {
//         t.collides_rel(&(), delta)
//     }
// }

impl Penetrates2d<Ball> for () {
    fn penetrates(&self, t: &Ball, rel: &impl Transform2d) -> Option<Vec2> {
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec2::new(t.radius, 0.0))
            } else {
                let old_magn = distance_to_center;
                let new_magn = t.radius - distance_to_center;
                let penetration = delta * (new_magn / old_magn);
                Some(penetration)
            }
        } else {
            None
        }
    }
}
impl Penetrates2d<()> for Ball {
    fn penetrates(&self, _t: &(), rel: &impl Transform2d) -> Option<Vec2> {
        ().penetrates(self, rel)
    }
}

impl Sdf2d<()> for Ball {
    fn sdf(&self, _t: &(), rel: &impl Transform2d) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl Sdf2dVector<()> for Ball {
    fn sdfvector(&self, _t: &(), rel: &impl Transform2d) -> Vec2 {
        let delta = rel.apply_origin();
        let length = delta.length();
        if length > 0.0 {
            let new_length = length - self.radius;
            delta * (new_length / length)
        } else {
            self.radius * Vec2::X
        }
    }
}
