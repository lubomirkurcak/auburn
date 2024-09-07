#![cfg_attr(not(feature = "std"), no_std)] // Use no_std when std feature is not enabled

pub use glam::{Quat, Vec2, Vec3};

mod col;
pub mod col2d;
pub mod col3d;
pub mod utils;

pub trait Square {
    fn square(self) -> Self;
}

impl Square for f32 {
    fn square(self) -> Self {
        self * self
    }
}

pub trait Lerp {
    fn lerp(self, other: Self, t: f32) -> Self;
}

impl Lerp for f32 {
    fn lerp(self, other: Self, t: f32) -> Self {
        self + t * (other - self)
    }
}

impl Lerp for Vec2 {
    fn lerp(self, other: Self, t: f32) -> Self {
        self + t * (other - self)
    }
}
