use super::*;

#[derive(Default, Clone, Copy)]
pub struct RoundedBox2d {
    pub halfsize: Vec2,
    pub radius: f32,
}

impl RoundedBox2d {
    pub const fn new(halfsize: Vec2, radius: f32) -> Self {
        Self { halfsize, radius }
    }

    pub fn box_part(&self) -> Box2d {
        Box2d::new(self.halfsize)
    }
}

impl SymmetricBoundingBox2d for RoundedBox2d {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(self.halfsize.x + self.radius, self.halfsize.y + self.radius)
    }
}

impl ExtremePoint2d for RoundedBox2d {
    fn extreme_point(&self, direction: &Vec2) -> Vec2 {
        let a = Ball::new(self.radius);
        let b = Box2d::new(self.halfsize);
        a.extreme_point(direction) + b.extreme_point(direction)
    }
}

impl MinkowskiSum<RoundedBox2d> for RoundedBox2d {
    type Output = Self;
    fn minkowski_sum(&self, t: &RoundedBox2d) -> Self::Output {
        Self::Output {
            halfsize: self.halfsize + t.halfsize,
            radius: self.radius + t.radius,
        }
    }
}

impl MinkowskiNegationIsIdentity for RoundedBox2d {}

#[cfg(disable)]
impl CollidesRel2d<Point> for RoundedBox2d {
    fn collides(&self, t: &Point, delta: &Vec2) -> bool {
        // let b = Box2d::new(self.halfsize);
        // if b.collides(t, delta) {
        //     return true;
        // }
        // let c = Ball::new(self.radius);
        // if c.collides(t, delta) {
        //     return true;
        // }
        let p = self.extreme_point(delta);
        delta.length_squared() <= delta.dot(p)
    }
}

impl CollidesRel2d<Point> for RoundedBox2d {
    fn collides_rel(&self, t: &Point, rel: &impl Transform2d) -> bool {
        let bbox = self.symmetric_bounding_box();
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let x0 = -self.halfsize.x < delta.x;
            let x1 = delta.x < self.halfsize.x;
            if x0 && x1 {
                return true;
            }

            let y0 = -self.halfsize.y < delta.y;
            let y1 = delta.y < self.halfsize.y;
            if y0 && y1 {
                return true;
            }

            let c = Ball::with_radius(self.radius);

            if x0 {
                if y0 {
                    let cdelta = delta - self.halfsize;
                    c.collides_rel(t, &cdelta)
                } else {
                    let cdelta = delta + Vec2::new(-self.halfsize.x, self.halfsize.y);
                    c.collides_rel(t, &cdelta)
                }
            } else {
                if y0 {
                    let cdelta = delta + Vec2::new(self.halfsize.x, -self.halfsize.y);
                    c.collides_rel(t, &cdelta)
                } else {
                    let cdelta = delta + self.halfsize;
                    c.collides_rel(t, &cdelta)
                }
            }
        } else {
            false
        }
    }
}

// impl CollidesRel2d<RoundedBox2d> for Point {
//     fn collides_rel(&self, t: &RoundedBox2d, rel: &impl Transform2d) -> bool {
//         t.collides_rel(&Point, rel)
//     }
// }

impl Penetrates2d<Point> for RoundedBox2d {
    fn penetrates(&self, t: &Point, rel: &impl Transform2d) -> Option<Vec2> {
        let bbox = self.symmetric_bounding_box();
        if bbox.collides_rel(&Point, rel) {
            let delta = rel.apply_origin();

            let x0 = -self.halfsize.x < delta.x;
            let x1 = delta.x < self.halfsize.x;
            let middle_x = x0 && x1;

            let y0 = -self.halfsize.y < delta.y;
            let y1 = delta.y < self.halfsize.y;
            let middle_y = y0 && y1;

            if middle_x || middle_y {
                return bbox.penetrates(&Point, &delta);
            }

            let c = Ball::with_radius(self.radius);

            if x0 {
                if y0 {
                    let cdelta = delta - self.halfsize;
                    c.penetrates(t, &cdelta)
                } else {
                    let cdelta = delta + Vec2::new(-self.halfsize.x, self.halfsize.y);
                    c.penetrates(t, &cdelta)
                }
            } else {
                if y0 {
                    let cdelta = delta + Vec2::new(self.halfsize.x, -self.halfsize.y);
                    c.penetrates(t, &cdelta)
                } else {
                    let cdelta = delta + self.halfsize;
                    c.penetrates(t, &cdelta)
                }
            }
        } else {
            None
        }
    }
}

// impl Penetrates2d<RoundedBox2d> for Point {
//     fn penetrates(&self, t: &RoundedBox2d, delta: &Vec2) -> Option<Vec2> {
//         let neg = -*delta;
//         if let Some(p) = t.penetrates(&Point, &neg) {
//             Some(-p)
//         } else {
//             None
//         }
//     }
// }

// impl Penetrates2d<RoundedBox2d> for Point {
//     fn penetrates(&self, t: &RoundedBox2d, rel: &impl Transform2d) -> Option<Vec2> {
//         t.penetrates(&Point, rel)
//     }
// }

// impl CollidesRel2d<Box2d> for Ball {
//     fn collides_rel(&self, t: &Box2d, rel: &impl Transform2d) -> bool {
//         self.minkowski_difference(t).collides_rel(&Point, rel)
//     }
// }
// impl<T: Transform2d> Collides2d<Box2d, T> for Ball {
//     fn collides(&self, transform: &T, t: &Box2d, t_transform: &T) -> bool {
//         self.minkowski_difference(t).collides_rel(&Point, rel)
//     }
// }
// impl Penetrates2d<Box2d> for Ball {
//     fn penetrates(&self, t: &Box2d, rel: &impl Transform2d) -> Option<Vec2> {
//         Point.penetrates(&self.minkowski_difference(t), rel)
//     }
// }

#[cfg(disable)]
impl CollidesRel2d<RoundedBox2d> for RoundedBox2d {
    fn collides(&self, t: &RoundedBox2d, delta: &Vec2) -> bool {
        self.minkowski_difference(t).collides(&Point, &delta)
    }
}

#[cfg(disable)]
impl Penetrates2d<RoundedBox2d> for RoundedBox2d {
    fn penetrates(&self, t: &RoundedBox2d, delta: &Vec2) -> Option<Vec2> {
        Point.penetrates(&self.minkowski_difference(t), &delta)
    }
}

impl Sdf2d<Point> for RoundedBox2d {
    fn sdf(&self, t: &Point, rel: &impl Transform2d) -> f32 {
        Sdf2d::sdf(&self.box_part(), t, rel) - self.radius
    }
}

impl Sdf2dVector<Point> for RoundedBox2d {
    fn sdfvector(&self, t: &Point, rel: &impl Transform2d) -> Vec2 {
        // let d = Sdf2dVector::sdfvector(&self.box_part(), t, rel);
        // let l = d.length();
        // if l > 0.0 {
        //     d * (l - self.radius) / l
        // } else {
        //     d
        // }

        let delta = rel.apply_origin();
        let delta_x = delta.x.abs() - self.halfsize.x;
        let delta_y = delta.y.abs() - self.halfsize.y;

        if self.box_part().collides_rel(t, rel) {
            if delta_x > delta_y {
                return Vec2::new((delta_x - self.radius) * delta.x.signum(), 0.0);
            } else {
                return Vec2::new(0.0, (delta_y - self.radius) * delta.y.signum());
            }
        } else {
            if delta_x <= 0.0 {
                return Vec2::new(0.0, (delta_y - self.radius) * delta.y.signum());
            } else if delta_y <= 0.0 {
                return Vec2::new((delta_x - self.radius) * delta.x.signum(), 0.0);
            } else {
                let corner = Vec2::new(delta_x * delta.x.signum(), delta_y * delta.y.signum());
                let corner_length = corner.length();
                let corner = corner * (corner_length - self.radius) / corner_length;
                return corner;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
