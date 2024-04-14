use super::*;

#[derive(Default, Debug, Clone, PartialEq)]
/// Standard 3D transform.
///
/// Applies translation, rotation, and scale in this order:
///  * Scale (each axis can be scaled independently)
///  * Rotate
///  * Translate
///
/// Is not algebraically closed under composition (shear will be ignored).
pub struct Transform3d {
    pos: Vec3,
    rot: Quat,
    scale: Vec3,
}

#[cfg(feature = "bevy")]
impl From<bevy::prelude::Transform> for Transform3d {
    fn from(transform: bevy::prelude::Transform) -> Self {
        Self {
            pos: transform.translation,
            rot: transform.rotation,
            scale: transform.scale,
        }
    }
}

#[cfg(feature = "bevy")]
impl From<Transform3d> for bevy::prelude::Transform {
    fn from(transform: Transform3d) -> Self {
        Self {
            translation: transform.pos,
            rotation: transform.rot,
            scale: transform.scale,
        }
    }
}

impl Transformation3d for Transform3d {
    fn apply_origin(&self) -> Vec3 {
        self.pos
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.rot * (self.scale * point) + self.pos
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        self.rot.inverse() * (point - self.pos) / self.scale
    }
}

impl Invertible for Transform3d {
    fn inverse(&self) -> Self {
        Self {
            pos: self.unapply(Vec3::ZERO),
            rot: self.rot.inverse(),
            scale: 1.0 / self.scale,
        }
    }
}

// @note(lubo): Important to note that transform with non-uniform scale composition
// is not algebraically closed, shear can be introduced, and is thus approximate.
impl Composable for Transform3d {
    fn compose(&self, other: &Self) -> Self {
        Self {
            pos: self.apply(other.pos),
            rot: self.rot * other.rot,
            scale: self.scale * other.scale,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::assert_approx_eq;
    use crate::utils::approx::Approx;

    use super::*;

    #[test]
    fn test_inverse() {
        let t = Transform3d {
            pos: Vec3::new(1.0, 2.0, 3.0),
            rot: Quat::from_rotation_x(0.5),
            scale: Vec3::splat(1.0),
        };
        let t_inv = t.inverse();
        let p = Vec3::new(1.0, 2.0, 3.0);
        assert_approx_eq!(t.apply(p), t_inv.unapply(p));
    }
}
