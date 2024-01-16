use super::*;

#[derive(Default, Clone, Copy)]
pub struct Ball {
    pub radius: f32,
}

impl Ball {
    pub const fn new(radius: f32) -> Self {
        Self { radius }
    }

    pub const fn with_radius(radius: f32) -> Self {
        Self::new(radius)
    }
}

impl MinkowskiSum<Ball> for Ball {
    type Output = Self;

    fn minkowski_sum(&self, t: &Ball) -> Self::Output {
        Self::Output::with_radius(self.radius + t.radius)
    }
}

impl MinkowskiNegationIsIdentity for Ball {}
