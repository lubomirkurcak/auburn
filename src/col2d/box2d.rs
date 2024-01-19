use super::{
    Collides2d, ExtremePoint2d, MinkowskiDifference, MinkowskiNegationIsIdentity, MinkowskiSum,
    Penetrates2d, SymmetricBoundingBox2d, Transform2d, Vec2,
};

/// 2D rectangle *centered at the origin*.
#[derive(Default, Clone, Copy)]
pub struct Box2d {
    pub halfsize: Vec2,
}

impl Box2d {
    pub const fn with_halfdims(x: f32, y: f32) -> Self {
        Self::new(Vec2::new(x, y))
    }
    pub const fn new(halfsize: Vec2) -> Self {
        Self { halfsize }
    }
    pub fn cover(&mut self, point: &Vec2) {
        let x = point.x.abs();
        let y = point.y.abs();
        self.halfsize.x = self.halfsize.x.max(x);
        self.halfsize.y = self.halfsize.y.max(y);
    }
}

impl SymmetricBoundingBox2d for Box2d {
    fn symmetric_bounding_box(&self) -> Box2d {
        *self
    }
}

impl ExtremePoint2d for Box2d {
    fn extreme_point(&self, direction: &Vec2) -> Vec2 {
        Vec2::new(
            if direction.x > 0.0 {
                self.halfsize.x
            } else {
                -self.halfsize.x
            },
            if direction.y > 0.0 {
                self.halfsize.y
            } else {
                -self.halfsize.y
            },
        )
    }
}

impl MinkowskiSum<Box2d> for Box2d {
    type Output = Self;

    fn minkowski_sum(&self, t: &Box2d) -> Self::Output {
        Self::Output {
            halfsize: self.halfsize + t.halfsize,
        }
    }
}

impl MinkowskiNegationIsIdentity for Box2d {}

impl Collides2d<()> for Box2d {
    fn collides(&self, _t: &(), rel: &impl Transform2d) -> bool {
        let delta = rel.apply_origin();
        -self.halfsize.x < delta.x
            && delta.x < self.halfsize.x
            && -self.halfsize.y < delta.y
            && delta.y < self.halfsize.y
    }
}
impl Collides2d<Box2d> for () {
    fn collides(&self, t: &Box2d, rel: &impl Transform2d) -> bool {
        t.collides(&(), rel)
    }
}

impl Penetrates2d<Box2d> for () {
    fn penetrates(&self, t: &Box2d, rel: &impl Transform2d) -> Option<Vec2> {
        if self.collides(t, rel) {
            let delta = rel.apply_origin();
            let delta_x = t.halfsize.x - delta.x.abs();
            let delta_y = t.halfsize.y - delta.y.abs();
            let x_smaller = delta_x < delta_y;

            if x_smaller {
                return Some(Vec2::new(delta_x * delta.x.signum(), 0.0));
            } else {
                return Some(Vec2::new(0.0, delta_y * delta.y.signum()));
            }
        }
        None
    }
}

impl Penetrates2d<()> for Box2d {
    fn penetrates(&self, _t: &(), rel: &impl Transform2d) -> Option<Vec2> {
        ().penetrates(self, rel)
    }
}

#[cfg(penetrates_dir)]
impl Penetrates2dDir<()> for Box2d {
    fn penetrates_dir(&self, t: &(), delta: &Vec2, dir: &Vec2) -> Option<Vec2> {
        if self.collides(t, delta) {
            self.extreme_point(dir);

            todo!();

            let delta_x = self.halfsize.x - delta.x.abs();
            let delta_y = self.halfsize.y - delta.y.abs();
            let x_smaller = delta_x < delta_y;

            if x_smaller {
                return Some(Vec2::new(delta_x * delta.x.signum(), 0.0));
            } else {
                return Some(Vec2::new(0.0, delta_y * delta.y.signum()));
            }
        }
        None
    }
}

impl Collides2d<Box2d> for Box2d {
    fn collides(&self, t: &Box2d, rel: &impl Transform2d) -> bool {
        self.minkowski_difference(t).collides(&(), rel)
    }
}

impl Penetrates2d<Box2d> for Box2d {
    fn penetrates(&self, t: &Box2d, rel: &impl Transform2d) -> Option<Vec2> {
        // self.minkowski_difference(t).penetrates(&(), &delta)
        ().penetrates(&self.minkowski_difference(t), rel)
    }
}

#[cfg(penetrates_dir)]
impl Penetrates2dDir<Box2d> for Box2d {
    fn penetrates_dir(&self, t: &Box2d, delta: &Vec2, dir: &Vec2) -> Option<Vec2> {
        self.minkowski_difference(t)
            .penetrates_dir(&(), &delta, &dir)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_v_box() {
        let b = Box2d::with_halfdims(2.0, 1.0);
        let y1 = Vec2::new(0.0, 0.0);
        let y2 = Vec2::new(1.5, 0.0);
        let y3 = Vec2::new(0.0, -0.5);
        let n1 = Vec2::new(3.0, 0.0);
        let n2 = Vec2::new(0.0, 2.0);
        let n3 = Vec2::new(-3.0, 0.0);
        let n4 = Vec2::new(0.0, -2.0);
        let n5 = Vec2::new(-2.1, 1.1);

        assert!(b.collides(&(), &y1));
        assert!(().collides(&b, &y1));
        // TODO(lubo): Actualy check if this is reasonable
        assert!(b.penetrates(&(), &y1).is_some());
        assert!(().penetrates(&b, &y1).is_some());

        assert!(b.collides(&(), &y2));
        assert!(().collides(&b, &y2));
        assert_eq!(b.penetrates(&(), &y2), Some(Vec2::new(0.5, 0.0)));
        assert_eq!(().penetrates(&b, &y2), Some(Vec2::new(0.5, 0.0)));

        assert!(b.collides(&(), &y3));
        assert!(().collides(&b, &y3));
        assert_eq!(b.penetrates(&(), &y3), Some(Vec2::new(0.0, -0.5)));
        assert_eq!(().penetrates(&b, &y3), Some(Vec2::new(0.0, -0.5)));

        assert!(!b.collides(&(), &n1));
        assert!(!b.collides(&(), &n2));
        assert!(!b.collides(&(), &n3));
        assert!(!b.collides(&(), &n4));
        assert!(!b.collides(&(), &n5));
        assert_eq!(None, b.penetrates(&(), &n1));
        assert_eq!(None, b.penetrates(&(), &n2));
        assert_eq!(None, b.penetrates(&(), &n3));
        assert_eq!(None, b.penetrates(&(), &n4));
        assert_eq!(None, b.penetrates(&(), &n5));
        assert!(!().collides(&b, &n1));
        assert!(!().collides(&b, &n2));
        assert!(!().collides(&b, &n3));
        assert!(!().collides(&b, &n4));
        assert!(!().collides(&b, &n5));
        assert_eq!(None, ().penetrates(&b, &n1));
        assert_eq!(None, ().penetrates(&b, &n2));
        assert_eq!(None, ().penetrates(&b, &n3));
        assert_eq!(None, ().penetrates(&b, &n4));
        assert_eq!(None, ().penetrates(&b, &n5));
    }

    #[test]
    fn box_v_box_no_collision() {
        let b = Box2d::with_halfdims(0.5, 0.5);
        let delta = Vec2::new(2.0, 0.0);
        assert!(!b.collides(&b, &delta));
        assert_eq!(b.penetrates(&b, &delta), None);
    }

    #[test]
    fn box_v_box_perfect_overlap() {
        let b = Box2d::with_halfdims(0.5, 0.5);
        let delta = Vec2::new(0.0, 0.0);
        assert!(b.collides(&b, &delta));
        // TODO(lubo): Actualy check if this is reasonable
        assert!(b.penetrates(&b, &delta).is_some());
    }

    #[test]
    fn box_v_box_collision() {
        let b = Box2d::with_halfdims(0.5, 0.5);
        for i in 1..=9 {
            let delta = Vec2::new(i as f32 / 10.0, 0.0);
            assert!(b.collides(&b, &delta));
            assert_eq!(Some(Vec2::new(1.0, 0.0) - delta), b.penetrates(&b, &delta));
        }
    }
}
