use super::*;

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [Collides2d]
pub trait PenetratesRel2d<T> {
    /// Computes the smallest penetration vector between `self` and `t` in `self`-centric space.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.penetrates_rel(&b, &rel), Some(Vec2::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn penetrates_rel(&self, t: &T, rel: &impl Transformation2d) -> Option<Vec2>;
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [Collides2d]
pub trait Penetrates2d<'a, A: 'a, B: 'a, T, BB>
where
    T: Transformation2d + 'a,
    BB: Into<Collider2d<'a, B, T>>,
    A: PenetratesRel2d<B>,
{
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Collider2d {
    ///     shape: &Box2d::with_halfdims(1.0, 1.0),
    ///     transform: &Vec2::new(0.0, 0.0),
    /// };
    /// let b = Collider2d {
    ///     shape: &Box2d::with_halfdims(1.0, 1.0),
    ///     transform: &Vec2::new(1.0, 0.0),
    /// };
    /// assert_eq!(a.penetrates(b), Some(Vec2::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn penetrates(self, b: BB) -> Option<Vec2>;
}

impl<'a, A: 'a, B: 'a, T, AA, BB> Penetrates2d<'a, A, B, T, BB> for AA
where
    A: PenetratesRel2d<B>,
    T: Transformation2d + DeltaTransform + 'a,
    Collider2d<'a, A, T>: From<AA>,
    Collider2d<'a, B, T>: From<BB>,
{
    fn penetrates(self, bb: BB) -> Option<Vec2> {
        let a: Collider2d<'a, A, T> = self.into();
        let b: Collider2d<'a, B, T> = bb.into();
        let rel = a.transform.delta_transform(b.transform);
        a.shape.penetrates_rel(b.shape, &rel)
    }
}
