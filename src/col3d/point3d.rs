use super::*;

impl ExtremePoint3d for Point {
    fn extreme_point(&self, _direction: Vec3) -> Vec3 {
        Vec3::ZERO
    }
}
