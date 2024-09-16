use super::*;

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [SymmetricBoundingBox3d]
/// * [PenetratesRel3d]
pub trait CollidesRel3d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let rel = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// assert!(a.collides_rel(&b, &rel))
    /// ```
    ///
    /// # See also
    /// * [Penetrates3d::penetrates].
    fn collides_rel(&self, t: &T, rel: &impl Transformation3d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # See also
/// * [SymmetricBoundingBox3d]
/// * [Penetrates3d]
pub trait Collides3d<B, T: Transformation3d> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `transform` - Transform of `Self`
    /// * `t` - The object to check collision against
    /// * `delta` - Transform of `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col3d::*;
    /// let a = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let a_t = Translate3d::from(Vec3::new(0.0, 0.0, 0.0));
    /// let b = Box3d::with_halfdims(1.0, 1.0, 1.0);
    /// let b_t = Translate3d::from(Vec3::new(1.0, 0.0, 0.0));
    /// assert!(a.collides(&a_t, &b, &b_t))
    /// ```
    ///
    /// # See also
    /// * [Penetrates3d::penetrates].
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool;
}

impl<A, B, T> Collides3d<B, T> for A
where
    A: CollidesRel3d<B>,
    T: Transformation3d + DeltaTransform,
{
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
}

/// Test
pub trait Collides3dV2<B, T: Transformation3d> {
    fn collides_v2(self, t: &B, t_transform: &T) -> bool;
}

impl<A, B, T> Collides3dV2<B, T> for (&A, &T)
where
    A: CollidesRel3d<B>,
    T: Transformation3d + DeltaTransform,
{
    fn collides_v2(self, t: &B, t_transform: &T) -> bool {
        let (a, a_t) = self;
        let rel = a_t.delta_transform(t_transform);
        a.collides_rel(t, &rel)
    }
}

/// Test
pub trait Collides3dV3<B, T: Transformation3d> {
    fn collides_v3(self, b: (&B, &T)) -> bool;
}

impl<A, B, T> Collides3dV3<B, T> for (&A, &T)
where
    A: CollidesRel3d<B>,
    T: Transformation3d + DeltaTransform,
{
    fn collides_v3(self, b: (&B, &T)) -> bool {
        let (a, a_t) = self;
        let (b, b_t) = b;
        let rel = a_t.delta_transform(b_t);
        a.collides_rel(b, &rel)
    }
}
