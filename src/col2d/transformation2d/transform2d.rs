use super::*;

#[derive(Debug, Clone, PartialEq)]
/// Apply translation, rotation, and scale in this order:
/// 1. Scale
/// 1. Rotate
/// 1. Translate
pub struct Transform2d {
    pub pos: Vec2,
    pub rot: Rotor2d,
    pub scale: Vec2,
}

impl Default for Transform2d {
    fn default() -> Self {
        Self {
            pos: Vec2::ZERO,
            rot: Rotor2d::IDENTITY,
            scale: Vec2::ONE,
        }
    }
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

    fn apply_normal(&self, normal: Vec2) -> Vec2 {
        self.rot * normal
    }

    fn unapply_normal(&self, normal: Vec2) -> Vec2 {
        self.rot.inverse() * normal
    }

    fn scaling_factor(&self) -> f32 {
        self.scale.x.max(self.scale.y)
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

impl Transform2d {
    pub const IDENTITY: Self = Self {
        pos: Vec2::ZERO,
        rot: Rotor2d::IDENTITY,
        scale: Vec2::ONE,
    };

    pub fn from_translation(translation: Vec2) -> Self {
        Self {
            pos: translation,
            ..Default::default()
        }
    }

    pub fn from_angle(angle: f32) -> Self {
        Self {
            rot: Rotor2d::from_angle(angle),
            ..Default::default()
        }
    }

    pub fn from_rotation(rotation: Rotor2d) -> Self {
        Self {
            rot: rotation,
            ..Default::default()
        }
    }

    pub fn from_scale(scale: Vec2) -> Self {
        Self {
            scale,
            ..Default::default()
        }
    }

    pub fn with_translation(mut self, translation: Vec2) -> Self {
        self.pos = translation;
        self
    }

    pub fn with_angle(mut self, angle: f32) -> Self {
        self.rot = Rotor2d::from_angle(angle);
        self
    }

    pub fn with_rotation(mut self, rotation: Rotor2d) -> Self {
        self.rot = rotation;
        self
    }

    pub fn with_scale(mut self, scale: Vec2) -> Self {
        self.scale = scale;
        self
    }
}
