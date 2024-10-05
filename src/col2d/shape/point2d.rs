use super::*;

impl SymmetricBoundingBox2d for Point {
    fn symmetric_bounding_box(&self) -> Box2d {
        Box2d::with_halfdims(0.0, 0.0)
    }
}

impl ExtremePoint2d for Point {
    fn extreme_point(&self, _: Vec2) -> Vec2 {
        Vec2::ZERO
    }
}

impl<T: Transformation2d> ExtremePointT2d<T> for Point {
    fn extreme_point_t(&self, t: &T, direction: Vec2) -> Vec2 {
        t.apply_origin()
    }
}

impl SdfRel2d<Point> for Point {
    fn sdf_rel(&self, _t: &Point, rel: &impl Transformation2d) -> f32 {
        let delta = rel.apply_origin();
        delta.length()
    }
}

impl<const P: bool, const D: bool, T: Transformation2d> SdfvCommonRel2d<P, D, Point, T> for Point {
    fn sdfv_common_rel(&self, _t: &Point, rel: &T) -> (bool, Vec2) {
        let a = rel.apply_origin();
        let collides = a.x == 0.0 && a.y == 0.0;
        (collides, a)
    }
}

macro_rules! impl_reverse_sdfv_common_rel {
    ($a:ty, $b:ty) => {
        impl<const P: bool, const D: bool, T> SdfvCommonRel2d<P, D, $b, T> for $a
        where
            $b: SdfvCommonRel2d<P, D, $a, T>,
            T: Transformation2d + Invertible,
        {
            fn sdfv_common_rel(&self, b: &$b, rel: &T) -> (bool, Vec2) {
                let inv_rel = rel.inverse();
                let (collides, sdfv) =
                    SdfvCommonRel2d::<P, D, $a, T>::sdfv_common_rel(b, self, &inv_rel);
                let negated_sdfv = -sdfv;
                let transformed_sdfv = rel.apply_normal(negated_sdfv);
                (collides, transformed_sdfv)
            }
        }
    };
}

impl_reverse_sdfv_common_rel!(Point, Box2d);
impl_reverse_sdfv_common_rel!(Point, Ball);

#[cfg(test)]
mod tests {
    use super::*;

    fn default_minkowski_check<A, B>()
    where
        A: DefaultMinkowski<B>,
        B: ExtremePoint2d,
    {
    }

    #[test_log::test]
    fn point_point_default_minkowski_trait_check() {
        // default_minkowski_check::<Point, Point>();
    }
}
