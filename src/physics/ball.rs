use std::ops::Mul;

use super::*;

#[derive(Default, Clone, Copy)]
pub struct Ball {
    pub radius: f32,
}

impl Ball {
    pub const fn new(radius: f32) -> Self {
        Self { radius }
    }

    pub const fn with_radius(radius: f32) -> Self {
        Self::new(radius)
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

impl MinkowskiSum<Ball> for Ball {
    type Output = Self;

    fn minkowski_sum(&self, t: &Ball) -> Self::Output {
        Self::Output::with_radius(self.radius + t.radius)
    }
}

impl MinkowskiNegationIsIdentity for Ball {}

impl Collides2d<()> for Ball {
    fn collides(&self, t: &(), t_transform: &impl Transform2dTrait) -> bool {
        t_transform.apply_origin().length_squared() < self.radius * self.radius
    }
}
impl Collides2d<Ball> for () {
    fn collides(&self, t: &Ball, delta: &impl Transform2dTrait) -> bool {
        t.collides(&(), delta)
    }
}

impl Penetrates2d<Ball> for () {
    fn penetrates(&self, t: &Ball, rel: &impl Transform2dTrait) -> Option<Vec2> {
        if self.collides(t, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec2::new(t.radius, 0.0))
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
impl Penetrates2d<()> for Ball {
    fn penetrates(&self, t: &(), rel: &impl Transform2dTrait) -> Option<Vec2> {
        ().penetrates(self, rel)
    }
}

impl Sdf2d<()> for Ball {
    fn sdf(&self, t: &(), rel: &impl Transform2dTrait) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl Sdf2dVector<()> for Ball {
    fn sdfvector(&self, t: &(), rel: &impl Transform2dTrait) -> Vec2 {
        let delta = rel.apply_origin();
        let length = delta.length();
        if length > 0.0 {
            let new_length = length - self.radius;
            delta.mul(new_length / length)
        } else {
            self.radius * Vec2::X
        }
    }
}
