use super::*;

#[derive(Default, Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
pub struct Point;

impl From<()> for Point {
    fn from(_: ()) -> Self {
        Point
    }
}

// impl MinkowskiNegationIsIdentity for Point {}

// impl<T: Copy> MinkowskiSum<T> for Point {
//     type Output = T;
//     fn minkowski_sum(&self, t: &T) -> Self::Output {
//         *t
//     }
// }
//
// impl<T: Copy> MinkowskiSum<Point> for T {
//     type Output = T;
//     fn minkowski_sum(&self, t: &T) -> Self::Output {
//         *t
//     }
// }
