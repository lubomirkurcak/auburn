use core::ops::Mul;

use super::*;

mod axis_transform2d;
mod isotropic2d;
mod transform2d;
mod translate2d;

pub use axis_transform2d::*;
pub use isotropic2d::*;
pub use transform2d::*;
pub use translate2d::*;

#[cfg(feature = "bevy")]
mod bevy_transform;
#[cfg(feature = "bevy")]
pub use bevy_transform::*;

/// Trait for transforming 2D points.
pub trait Transformation2d {
    /// Transform the origin.
    fn apply_origin(&self) -> Vec2;

    /// Transform a point.
    fn apply(&self, point: Vec2) -> Vec2;

    /// Invert transformation.
    fn unapply(&self, point: Vec2) -> Vec2;

    /// Transform a normal.
    fn apply_normal(&self, normal: Vec2) -> Vec2;

    /// Invert transformation of a normal.
    fn unapply_normal(&self, normal: Vec2) -> Vec2;
}

impl Transformation2d for IdentityTransform {
    fn apply_origin(&self) -> Vec2 {
        Vec2::ZERO
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        point
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        point
    }

    fn apply_normal(&self, normal: Vec2) -> Vec2 {
        normal
    }

    fn unapply_normal(&self, normal: Vec2) -> Vec2 {
        normal
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
/// Represents 2D rotation.
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

impl Rotor2d {
    const IDENTITY: Self = Self {
        a: Vec2::new(0.0, 1.0),
    };

    /// Create a new rotor from an angle.
    pub fn from_angle(angle: f32) -> Self {
        Self {
            a: Vec2::new(angle.cos(), angle.sin()),
        }
    }

    /// Get the angle of the rotor.
    pub fn angle(&self) -> f32 {
        self.a.y.atan2(self.a.x)
    }

    /// Get the inverse of the rotor.
    pub fn inverse(&self) -> Self {
        Self {
            a: Vec2::new(self.a.x, -self.a.y),
        }
    }

    /// Ignore rotation outside of the XY plane.
    pub fn from_quaternion(quat: crate::Quat) -> Rotor2d {
        let a = Vec2::new(quat.w, quat.z);
        // @todo(lubo): attempt to save the signs of the rotation?
        let a = a.try_normalize().unwrap_or(Vec2::X);
        Rotor2d { a }
    }
}

impl Mul<Rotor2d> for Rotor2d {
    type Output = Rotor2d;

    /// Compose two rotors.
    fn mul(self, rhs: Rotor2d) -> Self::Output {
        Self::Output {
            a: Vec2::new(
                self.a.x * rhs.a.x - self.a.y * rhs.a.y,
                self.a.x * rhs.a.y + self.a.y * rhs.a.x,
            ),
        }
    }
}

impl Mul<Vec2> for Rotor2d {
    type Output = Vec2;

    /// Rotate a vector.
    fn mul(self, rhs: Vec2) -> Self::Output {
        Self::Output {
            x: self.a.x * rhs.x - self.a.y * rhs.y,
            y: self.a.x * rhs.y + self.a.y * rhs.x,
        }
    }
}
