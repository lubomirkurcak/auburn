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

    fn apply_normal(&self, normal: Vec2) -> Vec2 {
        Into::<Transform2d>::into(*self).apply_normal(normal)
    }

    fn unapply_normal(&self, normal: Vec2) -> Vec2 {
        Into::<Transform2d>::into(*self).unapply_normal(normal)
    }
}

#[cfg(test)]
mod tests {
    use crate::col2d::*;
    use glam::Vec3;

    #[test_log::test]
    fn test_bevy_transform() {
        let transform = bevy::prelude::Transform::from_translation(Vec3::new(1.0, 2.0, 0.0));
        let transform2d: Transform2d = transform.into();
        assert_eq!(transform2d.pos, Vec2::new(1.0, 2.0));
    }
}
