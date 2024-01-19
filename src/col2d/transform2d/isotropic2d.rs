use super::*;

#[derive(Default)]
pub struct Isotropic2d {
    pos: Vec2,
    rot: Rotor2d,
    scale: f32,
}

impl Transform2d for Isotropic2d {
    fn apply_origin(&self) -> Vec2 {
        todo!()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        todo!()
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        todo!()
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
