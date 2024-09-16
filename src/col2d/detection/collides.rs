use super::*;

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [SymmetricBoundingBox2d]
/// * [PenetratesRel2d]
pub trait CollidesRel2d<T> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let rel = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert!(a.collides_rel(&b, &rel))
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides_rel(&self, t: &T, rel: &impl Transformation2d) -> bool;
}

/// Trait for checking collision between `Self` and `T`.
///
/// # Limitations
/// Currently, transformation types must be the same for both `Self` and `T`.
///
/// # See also
/// * [SymmetricBoundingBox2d]
/// * [Penetrates2d]
pub trait Collides2d<B, T: Transformation2d> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `delta` - The vector from `self` to `t`
    ///
    /// # Example
    /// ```
    /// # use auburn::col2d::*;
    /// let a = Box2d::with_halfdims(1.0, 1.0);
    /// let a_t = Translate2d::from(Vec2::new(0.0, 0.0));
    /// let b = Box2d::with_halfdims(1.0, 1.0);
    /// let b_t = Translate2d::from(Vec2::new(1.0, 0.0));
    /// assert!(a.collides(&a_t, &b, &b_t))
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool;
}

impl<A, B, T> Collides2d<B, T> for A
where
    A: CollidesRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn collides(&self, transform: &T, t: &B, t_transform: &T) -> bool {
        let rel = transform.delta_transform(t_transform);
        self.collides_rel(t, &rel)
    }
}

/// Test
pub trait Collides2dV2<B, T: Transformation2d> {
    fn collides_v2(self, t: &B, t_transform: &T) -> bool;
}

impl<A, B, T> Collides2dV2<B, T> for (&A, &T)
where
    A: CollidesRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn collides_v2(self, t: &B, t_transform: &T) -> bool {
        let (a, a_t) = self;
        let rel = a_t.delta_transform(t_transform);
        a.collides_rel(t, &rel)
    }
}

/// Test
pub trait Collides2dV3<B, T: Transformation2d> {
    fn collides_v3(self, b: (&B, &T)) -> bool;
}

impl<A, B, T> Collides2dV3<B, T> for (&A, &T)
where
    A: CollidesRel2d<B>,
    T: Transformation2d + DeltaTransform,
{
    fn collides_v3(self, b: (&B, &T)) -> bool {
        let (a, a_t) = self;
        let (b, b_t) = b;
        let rel = a_t.delta_transform(b_t);
        a.collides_rel(b, &rel)
    }
}
