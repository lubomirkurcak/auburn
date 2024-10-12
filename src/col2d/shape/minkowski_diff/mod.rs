mod v;

use crate::trace;

use super::*;

pub struct MinkowskiDiff2d<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    a: &'a A,
    b: &'a B,
    a_t: &'a T,
    ab_t: &'a T,
}

impl<'a, A, B, T> MinkowskiDiff2d<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn to_local(&self) -> local_minkowski_diff::LocalMinkowskiDiff2d<'a, A, B, T> {
        local_minkowski_diff::LocalMinkowskiDiff2d::raw(self.a, self.b, self.ab_t)
    }
}

pub struct MinkowskiDiff2dOwned<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    a: &'a A,
    b: &'a B,
    a_t: &'a T,
    ab_t: T,
}

impl<'a, A, B, T> From<&'a MinkowskiDiff2dOwned<'a, A, B, T>> for MinkowskiDiff2d<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
    MinkowskiDiff2dOwned<'a, A, B, T>: 'a,
{
    fn from(value: &'a MinkowskiDiff2dOwned<'a, A, B, T>) -> Self {
        Self {
            a: value.a,
            b: value.b,
            a_t: value.a_t,
            ab_t: &value.ab_t,
        }
    }
}

impl<'a, A, B, T> MinkowskiDiff2dOwned<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    pub fn from_colliders(a: &'a Collider2d<A, T>, b: &'a Collider2d<B, T>) -> Self
    where
        T: Transformation2d + DeltaTransform,
    {
        Self {
            a: &a.shape,
            b: &b.shape,
            a_t: &a.transform,
            ab_t: a.transform.delta_transform(b.transform),
        }
    }
}

impl<'a, A, B, T> ExtremePoint2d for MinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        let local_dir = self.a_t.unapply_normal(direction);
        let local_diff = self.to_local();
        let local_result = local_diff.extreme_point(local_dir);
        trace!("local_result: {}", local_result);
        let result = self.a_t.apply(local_result);
        trace!("result: {}", result);
        result
    }
}

#[cfg(test)]
mod tests {
    use crate::utils::approx::Approx;
    use crate::{assert_approx_eq, debug, info};

    use super::*;

    #[test_log::test]
    fn case_1_box() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 0.0;
        let a_shape = Box2d::with_halfdims(1.0, 1.0);
        let b_shape = Box2d::with_halfdims(1.0, 1.0);
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();
        let diff = MinkowskiDiff2dOwned::from_colliders(&a, &b);
        let diff: MinkowskiDiff2d<_, _, _> = (&diff).into();
        assert_eq!(diff.extreme_point(Vec2::X).x, -1.0);
    }

    #[test_log::test]
    fn case_1_poly() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 0.0;
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();
        let diff = MinkowskiDiff2dOwned::from_colliders(&a, &b);
        let diff: MinkowskiDiff2d<_, _, _> = (&diff).into();
        assert_eq!(diff.extreme_point(Vec2::X).x, -1.0);
    }

    #[test_log::test]
    fn ground_truth_check_1() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 90.0;
        let a_shape = Box2d::with_halfdims(1.0, 1.0);
        let b_shape = Box2d::with_halfdims(1.0, 1.0);
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();

        let a_gt = a.extreme_point(Vec2::X);
        debug!("a_gt: {}", a_gt);
        let b_gt = b.extreme_point(-Vec2::X);
        debug!("b_gt: {}", b_gt);
        let m_gt = a_gt - b_gt;
        info!("m_gt: {}", m_gt);

        let diff = MinkowskiDiff2dOwned::from_colliders(&a, &b);
        let diff: MinkowskiDiff2d<_, _, _> = (&diff).into();
        let m_diff = diff.extreme_point(Vec2::X);

        assert_approx_eq!(m_gt.x, m_diff.x);
    }

    #[test_log::test]
    fn case_2_box() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 90.0;
        let a_shape = Box2d::with_halfdims(1.0, 1.0);
        let b_shape = Box2d::with_halfdims(1.0, 1.0);
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();
        let diff = MinkowskiDiff2dOwned::from_colliders(&a, &b);
        let diff: MinkowskiDiff2d<_, _, _> = (&diff).into();
        assert_eq!(diff.extreme_point(Vec2::X).x, -1.0);
    }

    #[test_log::test]
    fn case_2_poly() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 90.0;
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();
        let rel = a.transform.delta_transform(b.transform);
        debug!("rel: {:?}", rel);
        let diff = MinkowskiDiff2dOwned::from_colliders(&a, &b);
        let diff: MinkowskiDiff2d<_, _, _> = (&diff).into();
        assert_eq!(diff.extreme_point(Vec2::X).x, -1.0);
    }
}
