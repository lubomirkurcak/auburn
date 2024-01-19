use super::*;

#[derive(Default)]
pub struct AxisTransform2d {
    pos: Vec2,
    scale: f32,
}

impl AxisTransform2d {
    pub const fn new(pos: Vec2, scale: f32) -> Self {
        Self { pos, scale }
    }
}

impl Transform2d for AxisTransform2d {
    fn apply_origin(&self) -> Vec2 {
        self.pos
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        self.pos + self.scale * point
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        (point - self.pos) / self.scale
    }
}

impl Invertible for AxisTransform2d {
    fn inverse(&self) -> Self {
        Self {
            pos: -self.pos,
            scale: 1.0 / self.scale,
        }
    }
}

impl Composable for AxisTransform2d {
    fn compose(&self, other: &Self) -> Self {
        Self {
            pos: self.apply(other.pos),
            scale: self.scale * other.scale,
        }
    }
}
