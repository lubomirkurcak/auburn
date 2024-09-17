//! 2D Collisions
//!
//! # Shapes:
//! * [Ball] - ball
//! * [Box2d] - 2D box
//! * [RoundedBox2d] - 2D rounded box
//! * [Tilemap] - 2D tilemap (requires feature `"tilemap"`)
//!
//! # Collision and Resolution:
//! * [Collides2d::collides]
//! * [Penetrates2d::penetrates]
//! * [Sdf2d::sdf]
//! * [Sdf2dVector::sdfv].
//!
//! # Transformations:
//! * [Translate2d] - translation
//! * [Transform2d] - standard 2D transform
//! * [Isotropic2d] - scale-uniform transform
//! * [AxisTransform2d]
//! * [bevy::prelude::Transform] - bevy transform (requires feature `"bevy"`)

pub use crate::Vec2;

mod collider2d;
mod default_impls;
mod detection;
mod shape;
mod transformation2d;

pub use crate::col::*;
pub use collider2d::*;
pub use default_impls::*;
pub use detection::*;
pub use shape::*;
pub use transformation2d::*;

/// Trait for computing bounding box of a shape.
///
/// # See also
/// * [Collides2d]
pub trait SymmetricBoundingBox2d {
    /// Computes the bounding box.
    ///
    /// # Example
    /// ```
    /// use auburn::col2d::*;
    /// assert_eq!(
    ///     Ball::with_radius(1.0).symmetric_bounding_box(),
    ///     Box2d::with_halfdims(1.0, 1.0),
    /// );
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn symmetric_bounding_box(&self) -> Box2d;
}
