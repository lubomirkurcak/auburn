use super::*;

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [`CollidesRel3d`]
pub trait PenetratesRel3d<B> {
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(a.penetrates_rel(&b, &rel), Some(Vec3::new(-1.0, 0.0, 0.0)));
    /// ```
    ///
    /// # See also
    /// * [Collides3d::collides].
    fn penetrates_rel(&self, t: &B, rel: &impl Transformation3d) -> Option<Vec3>;
}

/// Trait for computing smallest penetration vector between `Self` and `T`.
///
/// # See also
/// * [`Collides3d`]
pub trait Penetrates3d<B, T: Transformation3d> {
    /// Computes the smallest penetration vector between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute penetration into
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let a_t = Translate3d::from(Vec3::new(0.0, 0.0, 0.0));
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b_t = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(
    ///     a.penetrates(&a_t, &b, &b_t),
    ///     Some(Vec3::new(-1.0, 0.0, 0.0))
    /// );
    /// ```
    ///
    /// # See also
    /// * [Collides3d::collides].
    fn penetrates(&self, transform: &T, other: &B, other_transform: &T) -> Option<Vec3>;
}

impl<A, B, T> Penetrates3d<B, T> for A
where
    A: PenetratesRel3d<B>,
    T: Transformation3d + DeltaTransform,
{
    fn penetrates(&self, transform: &T, t: &B, t_transform: &T) -> Option<Vec3> {
        let rel = transform.delta_transform(t_transform);
        self.penetrates_rel(t, &rel)
    }
}
