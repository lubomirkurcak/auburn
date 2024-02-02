use super::*;

fn to_vec2(v: crate::Vec3) -> Vec2 {
    Vec2::new(v.x, v.y)
}

impl From<bevy::prelude::Transform> for Transform2d {
    fn from(transform: bevy::prelude::Transform) -> Self {
        Self {
            pos: to_vec2(transform.translation),
            rot: Rotor2d::from_quaternion(transform.rotation),
            scale: to_vec2(transform.scale),
        }
    }
}

impl Transformation2d for bevy::prelude::Transform {
    fn apply_origin(&self) -> Vec2 {
        Into::<Transform2d>::into(*self).apply_origin()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        Into::<Transform2d>::into(*self).apply(point)
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        Into::<Transform2d>::into(*self).unapply(point)
    }
}
