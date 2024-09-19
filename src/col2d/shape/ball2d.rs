use super::*;

#[cfg(minkoski)]
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

impl ExtremePointLocalSpace2d for Ball {
    fn extreme_point_local_space(&self, direction: Vec2) -> Vec2 {
        dbg!(direction);
        dbg!(self.radius * direction.normalize_or_zero());
        self.radius * direction.normalize_or_zero()
    }
}

// Collides

impl CollidesRel2d<Point> for Ball {
    fn collides_rel(&self, _: &Point, rel: &impl Transformation2d) -> bool {
        rel.apply_origin().length_squared() < self.radius * self.radius
    }
}

impl CollidesRel2d<Ball> for Ball {
    fn collides_rel(&self, b: &Ball, rel: &impl Transformation2d) -> bool {
        let radius = self.radius + b.radius;
        rel.apply_origin().length_squared() < radius * radius
    }
}

impl DefaultCol2dImpls for Ball {}

// Penetrates

impl PenetratesRel2d<Point> for Ball {
    fn penetrates_rel(&self, t: &Point, rel: &impl Transformation2d) -> Option<Vec2> {
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec2::new(self.radius, 0.0))
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

impl PenetratesRel2d<Ball> for Ball {
    fn penetrates_rel(&self, b: &Ball, rel: &impl Transformation2d) -> Option<Vec2> {
        let radius = self.radius + b.radius;
        if self.collides_rel(b, rel) {
            let delta = rel.apply_origin();
            let distance_to_center = delta.length();
            if distance_to_center < f32::EPSILON {
                Some(Vec2::new(radius, 0.0))
            } else {
                let old_magn = distance_to_center;
                let new_magn = radius - distance_to_center;
                let penetration = delta * (new_magn / old_magn);
                let penetration = -penetration;
                Some(penetration)
            }
        } else {
            None
        }
    }
}

// Sdf

impl SdfRel2d<Point> for Ball {
    fn sdf_rel(&self, _: &Point, rel: &impl Transformation2d) -> f32 {
        let delta = rel.apply_origin();
        delta.length() - self.radius
    }
}

impl SdfRel2d<Ball> for Ball {
    fn sdf_rel(&self, b: &Ball, rel: &impl Transformation2d) -> f32 {
        let radius = self.radius + b.radius;
        let delta = rel.apply_origin();
        delta.length() - radius
    }
}

// Sdfv

impl SdfvRel2d<Point> for Ball {
    fn sdfv_rel(&self, _: &Point, rel: &impl Transformation2d) -> Vec2 {
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

impl SdfvRel2d<Ball> for Ball {
    fn sdfv_rel(&self, b: &Ball, rel: &impl Transformation2d) -> Vec2 {
        let radius = self.radius + b.radius;
        let delta = rel.apply_origin();
        let length = delta.length();
        if length > 0.0 {
            let new_length = length - radius;
            delta * (new_length / length)
        } else {
            radius * Vec2::X
        }
    }
}
