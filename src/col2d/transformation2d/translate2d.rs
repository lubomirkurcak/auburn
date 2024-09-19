use super::*;

#[derive(Default, Clone, Copy, Debug, PartialEq)]
pub struct Translate2d {
    pos: Vec2,
}

impl From<Vec2> for Translate2d {
    fn from(pos: Vec2) -> Self {
        Self::new(pos)
    }
}

impl Transformation2d for Vec2 {
    fn apply_origin(&self) -> Vec2 {
        *self
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        point + *self
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        point - *self
    }

    fn apply_normal(&self, normal: Vec2) -> Vec2 {
        normal
    }

    fn unapply_normal(&self, normal: Vec2) -> Vec2 {
        normal
    }
}

impl Invertible for Vec2 {
    fn inverse(&self) -> Self {
        -*self
    }
}

impl Composable for Vec2 {
    fn compose(&self, other: &Self) -> Self {
        self.apply(*other)
    }
}

impl Translate2d {
    pub const fn new(pos: Vec2) -> Self {
        Self { pos }
    }
}

impl Transformation2d for Translate2d {
    fn apply_origin(&self) -> Vec2 {
        self.pos
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        point + self.pos
    }

    fn unapply(&self, point: Vec2) -> Vec2 {
        point - self.pos
    }

    fn apply_normal(&self, normal: Vec2) -> Vec2 {
        normal
    }

    fn unapply_normal(&self, normal: Vec2) -> Vec2 {
        normal
    }
}

impl Invertible for Translate2d {
    fn inverse(&self) -> Self {
        Self::new(-self.pos)
    }
}

impl Composable for Translate2d {
    fn compose(&self, other: &Self) -> Self {
        Self::new(self.apply(other.pos))
    }
}
