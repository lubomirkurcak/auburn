use super::*;

impl Transformation3d for bevy::prelude::Transform {
    fn apply_origin(&self) -> Vec3 {
        self.translation
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.rotation * (self.scale * point) + self.translation
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        self.rotation.inverse() * (point - self.translation) / self.scale
    }
}

impl Invertible for bevy::prelude::Transform {
    fn inverse(&self) -> Self {
        Self {
            translation: self.unapply(Vec3::ZERO),
            rotation: self.rotation.inverse(),
            scale: 1.0 / self.scale,
        }
    }
}

// @note(lubo): Important to note that transform with non-uniform scale composition
// is not algebraically closed, shear can be introduced, and is thus approximate.
impl Composable for bevy::prelude::Transform {
    fn compose(&self, other: &Self) -> Self {
        Self {
            translation: self.apply(other.translation),
            rotation: self.rotation * other.rotation,
            scale: self.scale * other.scale,
        }
    }
}
