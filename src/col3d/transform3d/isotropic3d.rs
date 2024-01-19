use super::*;

#[derive(Default)]
pub struct Isotropic3d {
    pos: Vec3,
    rot: Rotor3d,
    scale: f32,
}

impl Transform3dTrait for Isotropic3d {
    fn apply_origin(&self) -> Vec3 {
        todo!()
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        todo!()
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        todo!()
    }
}

impl Invertible for Isotropic3d {
    fn inverse(&self) -> Self {
        todo!()
    }
}

impl Composable for Isotropic3d {
    fn compose(&self, other: &Self) -> Self {
        todo!()
    }
}

pub struct Rotor3d {
    q: Quat,
}

impl Default for Rotor3d {
    fn default() -> Self {
        Self { q: Quat::default() }
    }
}
