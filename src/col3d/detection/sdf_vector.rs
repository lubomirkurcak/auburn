use super::*;

/// Trait for computing the *vector* signed-distance between `Self` and `T`.
///
/// # See also
/// * [`SdfRel3d`]
pub trait SdfRel3dVector<T> {
    /// Computes *vector* signed-distance between `self` and `t`.
    ///
    /// # Arguments
    /// * `t` - The object to compute distance to
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert_eq!(a.sdfv_rel(&b, &rel), Vec3::new(-1.0, 0.0, 0.0));
    /// ```
    ///
    /// # See also
    /// * [Sdf3d::sdf].
    fn sdfv_rel(&self, t: &T, rel: &impl Transformation3d) -> Vec3;
}
