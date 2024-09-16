use super::*;

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [SdfRel2dVector]
pub trait SdfRel2d<T> {
    /// Computes *scalar* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `t` - The object to check collision against
    /// * `delta` - Transform of `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdf_rel(&b, &rel), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf2dVector::sdfv].
    fn sdf_rel(&self, t: &T, rel: &impl Transformation2d) -> f32;
}

/// Trait for computing the *scalar* signed-distance between `Self` and `T`.
///
/// # See also
/// * [Sdf2dVector]
pub trait Sdf2d<B, T: Transformation2d> {
    /// Computes *scalar* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `t` - The object to check collision against
    /// * `delta` - Transform of `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert_eq!(a.sdf(&a_t, &b, &b_t), -1.0);
    /// ```
    ///
    /// # See also
    /// * [Sdf2dVector::sdfv].
    fn sdf(&self, a_t: &T, b: &B, b_t: &T) -> f32;
}

impl<A, B, T> Sdf2d<B, T> for A
where
    A: SdfRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn sdf(&self, a_transform: &T, b: &B, b_transform: &T) -> f32 {
        let rel = a_transform.delta_transform(b_transform);
        self.sdf_rel(b, &rel)
    }
}
