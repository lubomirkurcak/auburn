use crate::trace;

use super::*;

mod v_ball;
mod v_box2d;
mod v_point;

/// 2D rectangle *centered at the origin*.
#[derive(Default, Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
pub struct Box2d {
    pub halfsize: Vec2,
}

impl DefaultCol2dImpls for Box2d {}

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

//

impl SymmetricBoundingBox2d for Box2d {
    fn symmetric_bounding_box(&self) -> Box2d {
        *self
    }
}

impl ExtremePoint2d for Box2d {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        trace!("Box2d::extreme_point");
        trace!("direction: {:?}", direction);
        trace!(
            "result: {:?}",
            Vec2::new(
                direction.x.signum() * self.halfsize.x,
                direction.y.signum() * self.halfsize.y,
            )
        );
        Vec2::new(
            direction.x.signum() * self.halfsize.x,
            direction.y.signum() * self.halfsize.y,
        )
    }
}

#[cfg(minkoski)]
impl MinkowskiNegationIsIdentity for Box2d {}

#[cfg(minkoski)]
impl MinkowskiSum<Box2d> for Box2d {
    type Output = Self;

    fn minkowski_sum(&self, t: &Box2d) -> Self::Output {
        Self::Output {
            halfsize: self.halfsize + t.halfsize,
        }
    }
}

#[cfg(minkoski)]
impl MinkowskiSum<Ball> for Box2d {
    type Output = RoundedBox2d;
    fn minkowski_sum(&self, t: &Ball) -> Self::Output {
        Self::Output {
            halfsize: self.halfsize,
            radius: t.radius,
        }
    }
}
