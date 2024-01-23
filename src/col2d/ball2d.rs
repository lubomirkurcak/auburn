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
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        self.radius * direction.normalize_or_zero()
    }
}

// Collides

impl CollidesRel2d<Point> for Ball {
    fn collides_rel(&self, _t: &Point, rel: &impl Transform2d) -> bool {
        rel.apply_origin().length_squared() < self.radius * self.radius
    }
}

impl DefaultCol2dImpls for Ball {}

// Penetrates

impl Penetrates2d<Point> for Ball {
    fn penetrates(&self, t: &Point, rel: &impl Transform2d) -> Option<Vec2> {
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec2::new(self.radius, 0.0))
            } else {
                let old_magn = distance_to_center;
                let new_magn = self.radius - distance_to_center;
                let penetration = delta * (new_magn / old_magn);
                Some(penetration)
            }
        } else {
            None
        }
    }
}

impl Sdf2d<Point> for Ball {
    fn sdf(&self, _t: &Point, rel: &impl Transform2d) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl Sdf2dVector<Point> for Ball {
    fn sdfvector(&self, _t: &Point, rel: &impl Transform2d) -> Vec2 {
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
