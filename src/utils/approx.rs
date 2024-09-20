use crate::Vec2;
use crate::Vec3;

pub trait Approx {
    const DEFAULT_TOLERANCE: f32 = 1e-6;

    fn approx_eq(&self, other: &Self) -> bool {
        self.approx_eq_tolerance(other, Self::DEFAULT_TOLERANCE)
    }

    fn approx_eq_tolerance(&self, other: &Self, tolerance: f32) -> bool;
}

impl Approx for Vec2 {
    fn approx_eq_tolerance(&self, other: &Self, tolerance: f32) -> bool {
        let magn_sq = self.length_squared().max(other.length_squared());
        if magn_sq == 0.0 {
            return true;
        }

        let error = *self - *other;
        if error.x.is_nan() || error.y.is_nan() {
            return false;
        }
        if error.x.is_infinite() || error.y.is_infinite() {
            // @note(lubo): we can be more lenient here when the need arises
            return self == other;
        }

        let error_sq = error.length_squared();
        let relative_error_sq = error_sq / magn_sq;

        // @note(lubo): since we are dealing with squared values, this error is equivalent to 1e-6
        if relative_error_sq > tolerance * tolerance {
            return false;
        }

        if error.x * (error.x / magn_sq) > tolerance * tolerance {
            return false;
        }

        if error.y * (error.y / magn_sq) > tolerance * tolerance {
            return false;
        }

        true
    }
}

impl Approx for Vec3 {
    fn approx_eq_tolerance(&self, other: &Self, tolerance: f32) -> bool {
        let magn_sq = self.length_squared().max(other.length_squared());
        if magn_sq == 0.0 {
            return true;
        }

        let error = *self - *other;
        if error.x.is_nan() || error.y.is_nan() || error.z.is_nan() {
            return false;
        }
        if error.x.is_infinite() || error.y.is_infinite() || error.z.is_infinite() {
            // @note(lubo): we can be more lenient here when the need arises
            return self == other;
        }

        let error_sq = error.length_squared();
        let relative_error_sq = error_sq / magn_sq;

        // @note(lubo): since we are dealing with squared values, this error is equivalent to 1e-6
        if relative_error_sq > tolerance * tolerance {
            return false;
        }

        if error.x * (error.x / magn_sq) > tolerance * tolerance {
            return false;
        }

        if error.y * (error.y / magn_sq) > tolerance * tolerance {
            return false;
        }

        if error.z * (error.z / magn_sq) > tolerance * tolerance {
            return false;
        }

        true
    }
}

#[macro_export]
macro_rules! assert_approx_eq {
    ($a:expr, $b:expr) => {
        assert!($a.approx_eq(&$b), "{:?} != {:?}", $a, $b)
    };
}
