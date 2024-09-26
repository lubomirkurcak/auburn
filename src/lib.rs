#![cfg_attr(not(feature = "std"), no_std)] // Use no_std when std feature is not enabled

pub use glam::{Quat, Vec2, Vec3};
#[cfg(feature = "logging")]
use log;

mod col;
#[cfg(feature = "2d")]
pub mod col2d;
#[cfg(feature = "3d")]
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

#[cfg(feature = "logging")]
#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {
        log::trace!($($arg)*);
    };
}

#[cfg(feature = "logging")]
#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {
        log::debug!($($arg)*);
    };
}

#[cfg(feature = "logging")]
#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {
        log::info!($($arg)*);
    };
}

#[cfg(feature = "logging")]
#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {
        log::warn!($($arg)*);
    };
}

#[cfg(feature = "logging")]
#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {
        log::error!($($arg)*);
    };
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {};
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {};
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {};
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {};
}

#[cfg(not(feature = "logging"))]
#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {};
}
