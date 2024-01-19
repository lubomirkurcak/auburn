use super::*;

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
