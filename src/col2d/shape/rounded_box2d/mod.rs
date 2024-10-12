mod v_point;

use super::*;

#[derive(Default, Clone, Copy, Debug, PartialEq)]
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
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        let a = Ball::new(self.radius);
        let b = Box2d::new(self.halfsize);
        a.extreme_point(direction) + b.extreme_point(direction)
    }
}
