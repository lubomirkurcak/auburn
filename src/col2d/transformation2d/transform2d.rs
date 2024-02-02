use super::*;

#[derive(Default, Debug, Clone, PartialEq)]
/// Apply translation, rotation, and scale in this order:
/// 1. Scale
/// 2. Rotate
/// 3. Translate
pub struct Transform2d {
    pub pos: Vec2,
    pub rot: Rotor2d,
    pub scale: Vec2,
}

impl Transformation2d for Transform2d {
    fn apply_origin(&self) -> Vec2 {
        self.pos
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        self.rot * (self.scale * point) + self.pos
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        self.rot.inverse() * (point - self.pos) / self.scale
    }
}

impl Invertible for Transform2d {
    fn inverse(&self) -> Self {
        Self {
            pos: self.unapply(Vec2::ZERO),
            rot: self.rot.inverse(),
            scale: 1.0 / self.scale,
        }
    }
}

// @note(lubo): Important to note that transform with non-uniform scale composition
// is not algebraically closed, shear can be introduced, and is thus approximate.
impl Composable for Transform2d {
    fn compose(&self, other: &Self) -> Self {
        Self {
            pos: self.apply(other.pos),
            rot: self.rot * other.rot,
            scale: self.scale * other.scale,
        }
    }
}
