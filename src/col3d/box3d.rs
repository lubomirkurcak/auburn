use super::{
    CollidesRel3d, ExtremePoint3d, MinkowskiDifference, MinkowskiNegationIsIdentity, MinkowskiSum,
    Penetrates3d, Transform3d, Vec3,
};

/// 3D rectangle *centered at the origin*.
#[derive(Default, Clone, Copy)]
pub struct Box3d {
    pub halfsize: Vec3,
}

impl Box3d {
    pub const fn with_halfdims(x: f32, y: f32, z: f32) -> Self {
        Self::new(Vec3::new(x, y, z))
    }
    pub const fn new(halfsize: Vec3) -> Self {
        Self { halfsize }
    }
    pub fn cover(&mut self, point: &Vec3) {
        let x = point.x.abs();
        let y = point.y.abs();
        self.halfsize.x = self.halfsize.x.max(x);
        self.halfsize.y = self.halfsize.y.max(y);
    }
}

// impl SymmetricBoundingBox3d for Box3d {
//     fn symmetric_bounding_box(&self) -> Box3d {
//         *self
//     }
// }

impl ExtremePoint3d for Box3d {
    fn extreme_point(&self, direction: &Vec3) -> Vec3 {
        Vec3::new(
            if direction.x > 0.0 {
                self.halfsize.x
            } else {
                -self.halfsize.x
            },
            if direction.y > 0.0 {
                self.halfsize.y
            } else {
                -self.halfsize.y
            },
            if direction.z > 0.0 {
                self.halfsize.z
            } else {
                -self.halfsize.z
            },
        )
    }
}

impl MinkowskiSum<Box3d> for Box3d {
    type Output = Self;

    fn minkowski_sum(&self, t: &Box3d) -> Self::Output {
        Self::Output {
            halfsize: self.halfsize + t.halfsize,
        }
    }
}

impl MinkowskiNegationIsIdentity for Box3d {}

impl CollidesRel3d<()> for Box3d {
    fn collides_rel(&self, _t: &(), rel: &impl Transform3d) -> bool {
        let delta = rel.apply_origin();
        -self.halfsize.x < delta.x
            && delta.x < self.halfsize.x
            && -self.halfsize.y < delta.y
            && delta.y < self.halfsize.y
            && -self.halfsize.z < delta.z
            && delta.z < self.halfsize.z
    }
}
impl CollidesRel3d<Box3d> for () {
    fn collides_rel(&self, t: &Box3d, rel: &impl Transform3d) -> bool {
        t.collides_rel(&(), rel)
    }
}

impl Penetrates3d<Box3d> for () {
    fn penetrates(&self, t: &Box3d, rel: &impl Transform3d) -> Option<Vec3> {
        if CollidesRel3d::collides_rel(&(), t, rel) {
            let delta = rel.apply_origin();
            let delta_x = t.halfsize.x - delta.x.abs();
            let delta_y = t.halfsize.y - delta.y.abs();
            let delta_z = t.halfsize.z - delta.z.abs();
            if delta_x < delta_y {
                if delta_z < delta_x {
                    return Some(Vec3::new(0.0, 0.0, delta_z * delta.z.signum()));
                } else {
                    return Some(Vec3::new(delta_x * delta.x.signum(), 0.0, 0.0));
                }
            } else if delta_z < delta_y {
                return Some(Vec3::new(0.0, 0.0, delta_z * delta.z.signum()));
            } else {
                return Some(Vec3::new(0.0, delta_y * delta.y.signum(), 0.0));
            }
        }
        None
    }
}

impl Penetrates3d<()> for Box3d {
    fn penetrates(&self, _t: &(), rel: &impl Transform3d) -> Option<Vec3> {
        ().penetrates(self, rel)
    }
}

#[cfg(penetrates_dir)]
impl Penetrates3dDir<()> for Box3d {
    fn penetrates_dir(&self, t: &(), delta: &Vec3, dir: &Vec3) -> Option<Vec3> {
        if self.collides(t, delta) {
            self.extreme_point(dir);

            todo!();

            let delta_x = self.halfsize.x - delta.x.abs();
            let delta_y = self.halfsize.y - delta.y.abs();
            let x_smaller = delta_x < delta_y;

            if x_smaller {
                return Some(Vec3::new(delta_x * delta.x.signum(), 0.0));
            } else {
                return Some(Vec3::new(0.0, delta_y * delta.y.signum()));
            }
        }
        None
    }
}

impl CollidesRel3d<Box3d> for Box3d {
    fn collides_rel(&self, t: &Box3d, rel: &impl Transform3d) -> bool {
        self.minkowski_difference(t).collides_rel(&(), rel)
    }
}

impl Penetrates3d<Box3d> for Box3d {
    fn penetrates(&self, t: &Box3d, rel: &impl Transform3d) -> Option<Vec3> {
        // self.minkowski_difference(t).penetrates(&(), &delta)
        Penetrates3d::penetrates(&(), &self.minkowski_difference(t), rel)
    }
}

#[cfg(penetrates_dir)]
impl Penetrates3dDir<Box3d> for Box3d {
    fn penetrates_dir(&self, t: &Box3d, delta: &Vec3, dir: &Vec3) -> Option<Vec3> {
        self.minkowski_difference(t)
            .penetrates_dir(&(), &delta, &dir)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_v_box() {
        let b = Box3d::with_halfdims(2.0, 1.0, 3.0);
        let y1 = Vec3::new(0.0, 0.0, 0.0);
        let y2 = Vec3::new(1.5, 0.0, 0.0);
        let y3 = Vec3::new(0.0, -0.5, 0.0);
        let n1 = Vec3::new(3.0, 0.0, 0.0);
        let n2 = Vec3::new(0.0, 2.0, 0.0);
        let n3 = Vec3::new(-3.0, 0.0, 0.0);
        let n4 = Vec3::new(0.0, -2.0, 0.0);
        let n5 = Vec3::new(-2.1, 1.1, 0.0);

        assert!(b.collides_rel(&(), &y1));
        assert!(().collides_rel(&b, &y1));
        // TODO(lubo): Actualy check if this is reasonable
        assert!(b.penetrates(&(), &y1).is_some());
        assert!(().penetrates(&b, &y1).is_some());

        assert!(b.collides_rel(&(), &y2));
        assert!(().collides_rel(&b, &y2));
        assert_eq!(b.penetrates(&(), &y2), Some(Vec3::new(0.5, 0.0, 0.0)));
        assert_eq!(().penetrates(&b, &y2), Some(Vec3::new(0.5, 0.0, 0.0)));

        assert!(b.collides_rel(&(), &y3));
        assert!(().collides_rel(&b, &y3));
        assert_eq!(b.penetrates(&(), &y3), Some(Vec3::new(0.0, -0.5, 0.0)));
        assert_eq!(().penetrates(&b, &y3), Some(Vec3::new(0.0, -0.5, 0.0)));

        assert!(!b.collides_rel(&(), &n1));
        assert!(!b.collides_rel(&(), &n2));
        assert!(!b.collides_rel(&(), &n3));
        assert!(!b.collides_rel(&(), &n4));
        assert!(!b.collides_rel(&(), &n5));
        assert_eq!(None, b.penetrates(&(), &n1));
        assert_eq!(None, b.penetrates(&(), &n2));
        assert_eq!(None, b.penetrates(&(), &n3));
        assert_eq!(None, b.penetrates(&(), &n4));
        assert_eq!(None, b.penetrates(&(), &n5));
        assert!(!().collides_rel(&b, &n1));
        assert!(!().collides_rel(&b, &n2));
        assert!(!().collides_rel(&b, &n3));
        assert!(!().collides_rel(&b, &n4));
        assert!(!().collides_rel(&b, &n5));
        assert_eq!(None, ().penetrates(&b, &n1));
        assert_eq!(None, ().penetrates(&b, &n2));
        assert_eq!(None, ().penetrates(&b, &n3));
        assert_eq!(None, ().penetrates(&b, &n4));
        assert_eq!(None, ().penetrates(&b, &n5));
    }

    #[test]
    fn box_v_box_no_collision() {
        let b = Box3d::with_halfdims(0.5, 0.5, 0.0);
        let delta = Vec3::new(2.0, 0.0, 0.0);
        assert!(!b.collides_rel(&b, &delta));
        assert_eq!(b.penetrates(&b, &delta), None);
    }

    #[test]
    fn box_v_box_perfect_overlap() {
        let b = Box3d::with_halfdims(0.5, 0.5, 0.5);
        let delta = Vec3::new(0.0, 0.0, 0.0);
        assert!(b.collides_rel(&b, &delta));
        // TODO(lubo): Actualy check if this is reasonable
        assert!(b.penetrates(&b, &delta).is_some());
    }

    #[test]
    fn box_v_box_collision() {
        let b = Box3d::with_halfdims(0.5, 0.5, 8.0);
        for i in 1..=9 {
            let delta = Vec3::new(i as f32 / 10.0, 0.0, 0.0);
            assert!(b.collides_rel(&b, &delta));
            assert_eq!(
                Some(Vec3::new(1.0, 0.0, 0.0) - delta),
                b.penetrates(&b, &delta)
            );
        }
    }
}
