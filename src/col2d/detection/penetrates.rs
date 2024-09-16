use super::*;

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [Collides2d]
pub trait PenetratesRel2d<T> {
    /// Computes the smallest penetration vector between `self` and `t`.
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
pub trait Penetrates2d<B, T: Transformation2d> {
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.penetrates(&a_t, &b, &b_t), Some(Vec2::new(-1.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides2d::collides].
    fn penetrates(&self, a_t: &T, b: &B, b_t: &T) -> Option<Vec2>;
}

impl<A, B, T> Penetrates2d<B, T> for A
where
    A: PenetratesRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn penetrates(&self, a_transform: &T, b: &B, b_transform: &T) -> Option<Vec2> {
        let rel = a_transform.delta_transform(b_transform);
        self.penetrates_rel(b, &rel)
    }
}
