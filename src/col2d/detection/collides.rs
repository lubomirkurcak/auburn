use super::*;

/// Trait for checking collision between `Self` and `T` given relative transform between them.
///
/// # See also
/// * [SymmetricBoundingBox2d]
/// * [PenetratesRel2d]
pub trait CollidesRel2d<B, T: Transformation2d> {
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `b` - The object to check collision against
    /// * `rel` - The *relative* transform from `self` to `b`
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
    fn collides_rel(&self, b: &B, rel: &T) -> bool;
}

//

/// Trait for checking collision between `Self` and `T`.
///
/// # Limitations
/// Currently, transformation types must be the same for both `Self` and `T`.
///
/// # See also
/// * [SymmetricBoundingBox2d]
/// * [Penetrates2d]
pub trait Collides2d<'a, A: 'a, B: 'a, T, BB>
where
    T: Transformation2d + 'a,
    BB: Into<Collider2d<'a, B, T>>,
    A: CollidesRel2d<B, T>,
{
    /// Checks whether objects collide.
    ///
    /// # Arguments
    /// * `t` - The object to check collision against
    /// * `delta` - The vector from `self` to `t`
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
    /// assert!(a.collides(b))
    /// ```
    ///
    /// # See also
    /// * [Penetrates2d::penetrates].
    fn collides(self, b: BB) -> bool;
}

impl<'a, A: 'a, B: 'a, T, AA, BB> Collides2d<'a, A, B, T, BB> for AA
where
    A: CollidesRel2d<B, T>,
    T: Transformation2d + DeltaTransform + 'a,
    Collider2d<'a, A, T>: From<AA>,
    Collider2d<'a, B, T>: From<BB>,
    AA: Copy,
    BB: Copy,
{
    fn collides(self, bb: BB) -> bool {
        let a: Collider2d<'a, A, T> = self.into();
        let b: Collider2d<'a, B, T> = bb.into();
        let rel = a.transform.delta_transform(b.transform);
        a.shape.collides_rel(b.shape, &rel)
    }
}
