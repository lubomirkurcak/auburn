use crate::Composable;
use crate::Invertible;
use crate::Vec3;
use crate::Quat;

/// Trait for transforming 3D points.
pub trait Transform3dTrait {
    /// Transform the origin.
    fn apply_origin(&self) -> Vec3;

    /// Transform a point.
    fn apply(&self, point: Vec3) -> Vec3;

    /// Invert transformation.
    fn unapply(&self, point: Vec3) -> Vec3;
}

#[derive(Default)]
pub struct Translate3d {
    pos: Vec3,
}

impl Transform3dTrait for Vec3 {
    fn apply_origin(&self) -> Vec3 {
        *self
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point + *self
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        point - *self
    }
}

impl Invertible for Vec3 {
    fn inverse(&self) -> Self {
        -*self
    }
}

impl Composable for Vec3 {
    fn compose(&self, other: &Self) -> Self {
        self.apply(*other)
    }
}

impl Translate3d {
    pub const fn new(pos: Vec3) -> Self {
        Self { pos }
    }
}

impl Transform3dTrait for Translate3d {
    fn apply_origin(&self) -> Vec3 {
        self.pos
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point + self.pos
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        point - self.pos
    }
}

impl Invertible for Translate3d {
    fn inverse(&self) -> Self {
        Self::new(-self.pos)
    }
}

impl Composable for Translate3d {
    fn compose(&self, other: &Self) -> Self {
        Self::new(self.apply(other.pos))
    }
}

#[derive(Default)]
pub struct AxisTransform3d {
    pos: Vec3,
    scale: f32,
}

impl AxisTransform3d {
    pub const fn new(pos: Vec3, scale: f32) -> Self {
        Self { pos, scale }
    }
}

impl Transform3dTrait for AxisTransform3d {
    fn apply_origin(&self) -> Vec3 {
        self.pos
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.pos + self.scale * point
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        (point - self.pos) / self.scale
    }
}

impl Invertible for AxisTransform3d {
    fn inverse(&self) -> Self {
        Self {
            pos: -self.pos,
            scale: 1.0 / self.scale,
        }
    }
}

impl Composable for AxisTransform3d {
    fn compose(&self, other: &Self) -> Self {
        Self {
            pos: self.apply(other.pos),
            scale: self.scale * other.scale,
        }
    }
}

#[derive(Default)]
pub struct Transform3d {
    pos: Vec3,
    rot: Rotor3d,
    scale: f32,
}

impl Transform3dTrait for Transform3d {
    fn apply_origin(&self) -> Vec3 {
        todo!()
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        todo!()
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        todo!()
    }
}

impl Invertible for Transform3d {
    fn inverse(&self) -> Self {
        todo!()
    }
}

impl Composable for Transform3d {
    fn compose(&self, other: &Self) -> Self {
        todo!()
    }
}

pub struct Rotor3d {
    q: Quat,
}

impl Default for Rotor3d {
    fn default() -> Self {
        Self {
            q: Quat::default(),
        }
    }
}

