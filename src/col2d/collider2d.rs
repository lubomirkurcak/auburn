use super::*;

// #[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[derive(Debug, PartialEq, Eq)]
pub struct Collider2d<'a, S, T: Transformation2d> {
    pub shape: &'a S,
    pub transform: &'a T,
}

impl<'a, S, T: Transformation2d> Collider2d<'a, S, T> {
    pub fn new(shape: &'a S, transform: &'a T) -> Self {
        Self { shape, transform }
    }
}

impl<'a, S, T: Transformation2d> Clone for Collider2d<'a, S, T> {
    fn clone(&self) -> Self {
        Self {
            shape: self.shape,
            transform: self.transform,
        }
    }
}

impl<'a, S, T: Transformation2d> Copy for Collider2d<'a, S, T> {}

// impl<'a, S, T: Transformation2d> Collider2d<'a, S, T> {
//     pub fn new(shape: &'a S, transform: &'a T) -> Collider2d<'a, S, T> {
//         Collider2d { shape, transform }
//     }
// }

impl<'a, S, T: Transformation2d> From<(&'a S, &'a T)> for Collider2d<'a, S, T> {
    fn from((shape, transform): (&'a S, &'a T)) -> Self {
        Collider2d { shape, transform }
    }
}

impl<A, T> ExtremePoint2d for Collider2d<'_, A, T>
where
    A: ExtremePointT2d<T>,
    T: Transformation2d,
{
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        self.shape.extreme_point_t(&self.transform, direction)
    }
}
