use super::*;

// impl SymmetricBoundingBox1d for Ball {
//     fn symmetric_bounding_box(&self) -> Box1d {
//         Box1d::with_halfdims(self.radius, self.radius)
//     }
// }

// impl ExtremePoint1d for Ball {
//     fn extreme_point(&self, direction: Vec1) -> Vec1 {
//         self.radius * direction.normalize_or_zero()
//     }
// }

// Collides

impl CollidesRel1d<Point> for Ball {
    fn collides_rel(&self, _t: &Point, rel: &impl Transform1d) -> bool {
        rel.apply_origin().abs() < self.radius
    }
}

impl DefaultCol1dImpls for Ball {}

// Penetrates

impl Penetrates1d<Point> for Ball {
    fn penetrates(&self, t: &Point, rel: &impl Transform1d) -> Option<Vec1> {
        if self.collides_rel(t, rel) {
            Some(Sdf1dVector::sdfvector(self, t, rel))
        } else {
            None
        }
    }
}

impl Sdf1d<Point> for Ball {
    fn sdf(&self, _t: &Point, rel: &impl Transform1d) -> f32 {
        let delta = rel.apply_origin();
        delta.abs() - self.radius
    }
}

impl Sdf1dVector<Point> for Ball {
    fn sdfvector(&self, _t: &Point, rel: &impl Transform1d) -> Vec1 {
        let delta = rel.apply_origin();

        if delta > 0.0 {
            delta - self.radius
        } else {
            delta + self.radius
        }
    }
}
