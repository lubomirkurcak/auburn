use super::*;

#[derive(Default)]
pub struct Isotropic2d {
    pos: Vec2,
    rot: Rotor2d,
    scale: f32,
}

impl Transformation2d for Isotropic2d {
    fn apply_origin(&self) -> Vec2 {
        todo!()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        todo!()
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        todo!()
    }

    fn apply_normal(&self, normal: Vec2) -> Vec2 {
        todo!()
    }

    fn unapply_normal(&self, normal: Vec2) -> Vec2 {
        todo!()
    }

    fn scaling_factor(&self) -> f32  {
        self.scale
    }
}

impl Invertible for Isotropic2d {
    fn inverse(&self) -> Self {
        todo!()
    }
}

impl Composable for Isotropic2d {
    fn compose(&self, other: &Self) -> Self {
        todo!()
    }
}
