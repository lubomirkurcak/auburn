use crate::{Vec2, Invertible, Composable};

/// Trait for transforming 2D points.
pub trait Transform2dTrait {
    /// Transform the origin.
    fn apply_origin(&self) -> Vec2;

    /// Transform a point.
    fn apply(&self, point: Vec2) -> Vec2;

    /// Invert transformation.
    fn unapply(&self, point: Vec2) -> Vec2;
}

#[derive(Default)]
pub struct Translate2d {
    pos: Vec2,
}

impl Transform2dTrait for Vec2 {
    fn apply_origin(&self) -> Vec2 {
        *self
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        point + *self
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        point - *self
    }
}

impl Invertible for Vec2 {
    fn inverse(&self) -> Self {
        -*self
    }
}

impl Composable for Vec2 {
    fn compose(&self, other: &Self) -> Self {
        self.apply(*other)
    }
}

impl Translate2d {
    pub const fn new(pos: Vec2) -> Self {
        Self { pos }
    }
}

impl Transform2dTrait for Translate2d {
    fn apply_origin(&self) -> Vec2 {
        self.pos
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        point + self.pos
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        point - self.pos
    }
}

impl Invertible for Translate2d {
    fn inverse(&self) -> Self {
        Self::new(-self.pos)
    }
}

impl Composable for Translate2d {
    fn compose(&self, other: &Self) -> Self {
        Self::new(self.apply(other.pos))
    }
}

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

impl Transform2dTrait for AxisTransform2d {
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

#[derive(Default)]
pub struct Transform2d {
    pos: Vec2,
    rot: Rotor2d,
    scale: f32,
}

impl Transform2dTrait for Transform2d {
    fn apply_origin(&self) -> Vec2 {
        todo!()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        todo!()
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        todo!()
    }
}

impl Invertible for Transform2d {
    fn inverse(&self) -> Self {
        todo!()
    }
}

impl Composable for Transform2d {
    fn compose(&self, other: &Self) -> Self {
        todo!()
    }
}

pub struct Rotor2d {
    a: Vec2,
}

impl Default for Rotor2d {
    fn default() -> Self {
        Self {
            a: Vec2::new(0.0, 1.0),
        }
    }
}
