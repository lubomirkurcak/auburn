use super::*;

impl MinkowskiNegationIsIdentity for () {}

impl<T: Copy> MinkowskiSum<T> for () {
    type Output = T;

    fn minkowski_sum(&self, t: &T) -> Self::Output {
        *t
    }
}

