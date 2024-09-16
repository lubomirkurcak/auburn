use super::*;

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [SdfRel2d]
pub trait SdfRel2dVector<T> {
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdfvector_rel(&b, &rel), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfvector_rel(&self, t: &T, rel: &impl Transformation2d) -> Vec2;
}

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [Sdf2d]
pub trait Sdf2dVector<B, T: Transformation2d> {
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdfvector(&a_t, &b, &b_t), Vec2::new(-1.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf2d::sdf].
    fn sdfvector(&self, a_t: &T, b: &B, b_t: &T) -> Vec2;
}

impl<A, B, T> Sdf2dVector<B, T> for A
where
    A: SdfRel2dVector<B>,
    T: Transformation2d + DeltaTransform,
{
    fn sdfvector(&self, a_transform: &T, b: &B, b_transform: &T) -> Vec2 {
        let rel = a_transform.delta_transform(b_transform);
        self.sdfvector_rel(b, &rel)
    }
}


