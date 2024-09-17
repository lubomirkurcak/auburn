//! 3D Collisions
//!
//! # Shapes
//! * [Ball] - ball
//! * [Box3d] - 3D box
//! * [Cylinder3d] - 3D cylinder
//!
//! # Collision and Resolution
//! * [Collides3d::collides]
//! * [Penetrates3d::penetrates]
//! * [Sdf3d::sdf]
//! * [Sdf3dVector::sdfv].
//!
//! # Transformations
//! * [Translate3d] - translation
//! * [Transform3d] - standard 3D transform
//! * [Isotropic3d] - scale-uniform transform
//! * [AxisTransform3d]
//! * [bevy::prelude::Transform] - bevy transform (requires feature `"bevy"`)

pub use crate::{Quat, Vec2, Vec3};

mod detection;
mod shape;
mod transformation3d;
mod default_impls;

pub use crate::col::*;
pub use detection::*;
pub use shape::*;
pub use transformation3d::*;
pub use default_impls::*;

