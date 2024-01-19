use super::*;

#[derive(Default)]
pub struct Translate3d {
    pos: Vec3,
}

impl Transform3dTrait for Vec3 {
    fn apply_origin(&self) -> Vec3 {
        *self
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point + *self
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        point - *self
    }
}

impl Invertible for Vec3 {
    fn inverse(&self) -> Self {
        -*self
    }
}

impl Composable for Vec3 {
    fn compose(&self, other: &Self) -> Self {
        self.apply(*other)
    }
}

impl Translate3d {
    pub const fn new(pos: Vec3) -> Self {
        Self { pos }
    }
}

impl Transform3dTrait for Translate3d {
    fn apply_origin(&self) -> Vec3 {
        self.pos
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point + self.pos
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        point - self.pos
    }
}

impl Invertible for Translate3d {
    fn inverse(&self) -> Self {
        Self::new(-self.pos)
    }
}

impl Composable for Translate3d {
    fn compose(&self, other: &Self) -> Self {
        Self::new(self.apply(other.pos))
    }
}
