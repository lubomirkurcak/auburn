//! Imagine you have transformations `A` and `B`.
//! You can calculate the difference between them by `A.inverse().compose(&B)`.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct IdentityTransform;

pub trait Invertible {
    /// Inversion of the transformation.
    #[cfg_attr(
        feature = "2d",
        doc = r##"
 The operation holds this property for any point `p` and transformation `a`:
 ```
 # use auburn::col2d::*;
 # let p = Vec2::new(1.0, 1.0);
 # let a = Translate2d::from(Vec2::new(1.0, 0.0));
 assert_eq!(p, a.inverse().apply(a.apply(p)))
 ```
"##
    )]
    fn inverse(&self) -> Self;
}

pub trait Composable {
    /// Compose two transformations.
    #[cfg_attr(
        feature = "2d",
        doc = r##"
The operation holds this property for any point `p` and transformations `a` and `b`:
```rust
# use auburn::col2d::*;
# let p = Vec2::new(1.0, 1.0);
# let a = Translate2d::from(Vec2::new(1.0, 0.0));
# let b = Translate2d::from(Vec2::new(0.0, 1.0));
assert_eq!(
    a.apply(b.apply(p)),
    a.compose(&b).apply(p),
)
```
"##
    )]
    fn compose(&self, other: &Self) -> Self;
}

pub trait DeltaTransform {
    /// Computes the difference between two transformations.
    #[cfg_attr(
        feature = "2d",
        doc = r##"
Useful for functions expecting a relative transform:

```
# use auburn::col2d::*;
let a = Translate2d::from(Vec2::new(1.0, 0.0));
let b = Translate2d::from(Vec2::new(0.0, 1.0));
assert_eq!(
    a.delta_transform(&b),
    Translate2d::from(Vec2::new(-1.0, 1.0)),
)
```
"##
    )]
    fn delta_transform(&self, other: &Self) -> Self;
}

impl<T: Invertible + Composable> DeltaTransform for T {
    fn delta_transform(&self, other: &Self) -> Self {
        self.inverse().compose(other)
    }
}
