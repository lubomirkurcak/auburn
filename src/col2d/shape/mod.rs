use super::*;

mod ball2d;
mod box2d;
#[cfg(disabled)]
mod ellipse2d;
pub mod local_minkowski_diff;
mod minkowski_diff;
mod point2d;
#[cfg(all(feature = "poly", feature = "std"))]
mod poly2d;
mod rounded_box2d;
#[cfg(feature = "tilemap")]
mod tilemap;

pub use box2d::*;
#[cfg(all(feature = "poly", feature = "std"))]
pub use poly2d::*;
#[cfg(disabled)]
pub use rounded_box2d::*;
#[cfg(feature = "tilemap")]
pub use tilemap::*;

/// Macro for creating a shape enum.
#[macro_export]
macro_rules! shape_union {
    ($name: ident; $($variant:ident),+) => {
        #[derive(Debug, Clone)]
        #[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
        pub enum $name {
            $(
                $variant($variant),
            )+
        }

        impl ExtremePoint2d for $name {
            fn extreme_point(&self, dir: Vec2) -> Vec2 {
                match self {
                    $(
                        Self::$variant(shape) => shape.extreme_point(dir),
                    )+
                }
            }
        }

        impl<T: Transformation2d> ExtremePointT2d<T> for $name {}

        impl DefaultMinkowski<$name> for $name {}

    }
}

shape_union!(Shape2d; Ball, Box2d, Poly2d);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_shape_extreme_point() {
        let shape = Shape2d::Ball(Ball::with_radius(1.0));
        assert_eq!(
            shape.extreme_point(Vec2::new(1.0, 0.0)),
            Vec2::new(1.0, 0.0)
        );
    }

    #[test]
    fn test_collider_extreme_point() {
        let shape = Shape2d::Ball(Ball::with_radius(1.0));
        let collider = Collider2d::new(&shape, &Vec2::X);
        assert_eq!(
            collider.extreme_point(Vec2::new(1.0, 0.0)),
            Vec2::new(2.0, 0.0)
        );
    }

    #[test]
    fn test_collider_v_collider() {
        let shape = Shape2d::Ball(Ball::with_radius(1.0));
        let collider = Collider2d::new(&shape, &Vec2::X);
        assert!(collider.collides(collider));
    }
}
