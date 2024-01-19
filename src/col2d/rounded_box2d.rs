use super::{
    Ball, Box2d, CollidesRel2d, ExtremePoint2d, MinkowskiDifference, MinkowskiNegationIsIdentity,
    MinkowskiSum, Penetrates2d, SymmetricBoundingBox2d, Transform2d, Translate2d, Vec2,
};

#[derive(Default, Clone, Copy)]
pub struct RoundedBox2d {
    pub halfsize: Vec2,
    pub radius: f32,
}

impl RoundedBox2d {
    pub fn new(halfsize: Vec2, radius: f32) -> Self {
        Self { halfsize, radius }
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

impl MinkowskiSum<Box2d> for Ball {
    type Output = RoundedBox2d;
    fn minkowski_sum(&self, t: &Box2d) -> Self::Output {
        Self::Output {
            halfsize: t.halfsize,
            radius: self.radius,
        }
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
impl CollidesRel2d<()> for RoundedBox2d {
    fn collides(&self, t: &(), delta: &Vec2) -> bool {
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

impl CollidesRel2d<()> for RoundedBox2d {
    fn collides_rel(&self, _t: &(), rel: &impl Transform2d) -> bool {
        let bbox = self.symmetric_bounding_box();
        if ().collides_rel(&bbox, rel) {
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

            //return true;
            let c = Ball::with_radius(self.radius);

            if x0 {
                if y0 {
                    let ct = Translate2d::new(delta - self.halfsize);
                    return ().collides_rel(&c, &ct);
                } else {
                    println!("bottom right");
                }
            } else {
                if y0 {
                    println!("top left");
                } else {
                    println!("bottom left");
                }
            }
            return false;
        }
        false
    }
}

impl CollidesRel2d<RoundedBox2d> for () {
    fn collides_rel(&self, t: &RoundedBox2d, rel: &impl Transform2d) -> bool {
        t.collides_rel(&(), rel)
    }
}

impl Penetrates2d<()> for RoundedBox2d {
    fn penetrates(&self, _t: &(), rel: &impl Transform2d) -> Option<Vec2> {
        let bbox = self.symmetric_bounding_box();
        if bbox.collides_rel(&(), rel) {
            let delta = rel.apply_origin();

            let x0 = -self.halfsize.x < delta.x;
            let x1 = delta.x < self.halfsize.x;
            let middle_x = x0 && x1;

            let y0 = -self.halfsize.y < delta.y;
            let y1 = delta.y < self.halfsize.y;
            let middle_y = y0 && y1;

            if middle_x || middle_y {
                return bbox.penetrates(&(), &delta);
            }

            let c = Ball::with_radius(self.radius);

            if x0 {
                if y0 {
                    let cdelta = delta - self.halfsize;
                    return ().penetrates(&c, &cdelta);
                } else {
                    let cdelta = delta + Vec2::new(-self.halfsize.x, self.halfsize.y);
                    return ().penetrates(&c, &cdelta);
                }
            } else {
                if y0 {
                    let cdelta = delta + Vec2::new(self.halfsize.x, -self.halfsize.y);
                    return ().penetrates(&c, &cdelta);
                } else {
                    let cdelta = delta + self.halfsize;
                    return ().penetrates(&c, &cdelta);
                }
            }
            return None;
        }
        None
    }
}

// impl Penetrates2d<RoundedBox2d> for () {
//     fn penetrates(&self, t: &RoundedBox2d, delta: &Vec2) -> Option<Vec2> {
//         let neg = -*delta;
//         if let Some(p) = t.penetrates(&(), &neg) {
//             Some(-p)
//         } else {
//             None
//         }
//     }
// }

impl Penetrates2d<RoundedBox2d> for () {
    fn penetrates(&self, t: &RoundedBox2d, rel: &impl Transform2d) -> Option<Vec2> {
        t.penetrates(&(), rel)
    }
}

impl CollidesRel2d<Box2d> for Ball {
    fn collides_rel(&self, t: &Box2d, rel: &impl Transform2d) -> bool {
        self.minkowski_difference(t).collides_rel(&(), rel)
    }
}
impl Penetrates2d<Box2d> for Ball {
    fn penetrates(&self, t: &Box2d, rel: &impl Transform2d) -> Option<Vec2> {
        ().penetrates(&self.minkowski_difference(t), rel)
    }
}

#[cfg(disable)]
impl CollidesRel2d<RoundedBox2d> for RoundedBox2d {
    fn collides(&self, t: &RoundedBox2d, delta: &Vec2) -> bool {
        self.minkowski_difference(t).collides(&(), &delta)
    }
}

#[cfg(disable)]
impl Penetrates2d<RoundedBox2d> for RoundedBox2d {
    fn penetrates(&self, t: &RoundedBox2d, delta: &Vec2) -> Option<Vec2> {
        ().penetrates(&self.minkowski_difference(t), &delta)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
