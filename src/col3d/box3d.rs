use super::*;

/// 3D rectangle *centered at the origin*.
#[derive(Default, Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
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
    fn extreme_point(&self, direction: Vec3) -> Vec3 {
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

impl MinkowskiNegationIsIdentity for Box3d {}

impl MinkowskiSum<Box3d> for Box3d {
    type Output = Self;

    fn minkowski_sum(&self, t: &Box3d) -> Self::Output {
        Self::Output {
            halfsize: self.halfsize + t.halfsize,
        }
    }
}

impl DefaultCol3dImpls for Box3d {}

// Collides

impl CollidesRel3d<Point> for Box3d {
    fn collides_rel(&self, _t: &Point, rel: &impl Transform3d) -> bool {
        let delta = rel.apply_origin();
        -self.halfsize.x < delta.x
            && delta.x < self.halfsize.x
            && -self.halfsize.y < delta.y
            && delta.y < self.halfsize.y
            && -self.halfsize.z < delta.z
            && delta.z < self.halfsize.z
    }
}

// Penetrates

impl Penetrates3d<Point> for Box3d {
    fn penetrates(&self, t: &Point, rel: &impl Transform3d) -> Option<Vec3> {
        if self.collides_rel(t, rel) {
            let delta = rel.apply_origin();
            let delta_x = delta.x.abs() - self.halfsize.x;
            let delta_y = delta.y.abs() - self.halfsize.y;
            let delta_z = delta.z.abs() - self.halfsize.z;
            if delta_x > delta_y {
                if delta_z > delta_x {
                    return Some(Vec3::new(0.0, 0.0, delta_z * delta.z.signum()));
                } else {
                    return Some(Vec3::new(delta_x * delta.x.signum(), 0.0, 0.0));
                }
            } else if delta_z > delta_y {
                return Some(Vec3::new(0.0, 0.0, delta_z * delta.z.signum()));
            } else {
                return Some(Vec3::new(0.0, delta_y * delta.y.signum(), 0.0));
            }
        }
        None
    }
}

// Sdf

impl Sdf3d<Point> for Box3d {
    fn sdf(&self, t: &Point, rel: &impl Transform3d) -> f32 {
        let delta = rel.apply_origin();
        let delta_x = delta.x.abs() - self.halfsize.x;
        let delta_y = delta.y.abs() - self.halfsize.y;
        let delta_z = delta.z.abs() - self.halfsize.z;

        if self.collides_rel(t, rel) {
            delta_x.max(delta_y).max(delta_z)
        } else {
            if delta_z < 0.0 {
                if delta_x < 0.0 {
                    delta_y
                } else if delta_y < 0.0 {
                    delta_x
                } else {
                    Vec2::new(delta_x, delta_y).length()
                }
            } else {
                if delta_x < 0.0 {
                    Vec2::new(delta_z, delta_y).length()
                } else if delta_y < 0.0 {
                    Vec2::new(delta_z, delta_x).length()
                } else {
                    Vec3::new(delta_x, delta_y, delta_z).length()
                }
            }
        }
    }
}

// Sdf Vector

impl Sdf3dVector<Point> for Box3d {
    fn sdfvector(&self, t: &Point, rel: &impl Transform3d) -> Vec3 {
        let delta = rel.apply_origin();
        let delta_x = delta.x.abs() - self.halfsize.x;
        let delta_y = delta.y.abs() - self.halfsize.y;
        let delta_z = delta.z.abs() - self.halfsize.z;

        if self.collides_rel(t, rel) {
            if delta_x > delta_y {
                if delta_z > delta_x {
                    Vec3::new(0.0, 0.0, delta_z * delta.z.signum())
                } else {
                    Vec3::new(delta_x * delta.x.signum(), 0.0, 0.0)
                }
            } else {
                if delta_z > delta_y {
                    Vec3::new(0.0, 0.0, delta_z * delta.z.signum())
                } else {
                    Vec3::new(0.0, delta_y * delta.y.signum(), 0.0)
                }
            }
        } else {
            let delta_x = delta_x.max(0.0);
            let delta_y = delta_y.max(0.0);
            let delta_z = delta_z.max(0.0);
            Vec3::new(
                delta_x * delta.x.signum(),
                delta_y * delta.y.signum(),
                delta_z * delta.z.signum(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn point_v_box_collides() {
        let b = Box3d::with_halfdims(2.0, 1.0, 3.0);
        let y1 = Vec3::new(0.0, 0.0, 0.0);
        let y2 = Vec3::new(1.5, 0.0, 0.0);
        let y3 = Vec3::new(0.0, -0.5, 0.0);
        let n1 = Vec3::new(3.0, 0.0, 0.0);
        let n2 = Vec3::new(0.0, 2.0, 0.0);
        let n3 = Vec3::new(-3.0, 0.0, 0.0);
        let n4 = Vec3::new(0.0, -2.0, 0.0);
        let n5 = Vec3::new(-2.1, 1.1, 0.0);

        assert!(b.collides_rel(&Point, &y1));
        assert!(b.collides_rel(&Point, &y2));
        assert!(b.collides_rel(&Point, &y3));

        assert!(Point.collides_rel(&b, &y1));
        assert!(Point.collides_rel(&b, &y2));
        assert!(Point.collides_rel(&b, &y3));

        assert!(!b.collides_rel(&Point, &n1));
        assert!(!b.collides_rel(&Point, &n2));
        assert!(!b.collides_rel(&Point, &n3));
        assert!(!b.collides_rel(&Point, &n4));
        assert!(!b.collides_rel(&Point, &n5));

        assert!(!Point.collides_rel(&b, &n1));
        assert!(!Point.collides_rel(&b, &n2));
        assert!(!Point.collides_rel(&b, &n3));
        assert!(!Point.collides_rel(&b, &n4));
        assert!(!Point.collides_rel(&b, &n5));
    }
    #[test]
    fn point_v_box_penetrates() {
        let b = Box3d::with_halfdims(2.0, 1.0, 3.0);
        let y1 = Vec3::new(0.0, 0.0, 0.0);
        let y2 = Vec3::new(1.5, 0.0, 0.0);
        let y3 = Vec3::new(0.0, -0.5, 0.0);
        let n1 = Vec3::new(3.0, 0.0, 0.0);
        let n2 = Vec3::new(0.0, 2.0, 0.0);
        let n3 = Vec3::new(-3.0, 0.0, 0.0);
        let n4 = Vec3::new(0.0, -2.0, 0.0);
        let n5 = Vec3::new(-2.1, 1.1, 0.0);

        // TODO(lubo): Actualy check if this is reasonable
        assert!(b.penetrates(&Point, &y1).is_some());
        assert!(Point.penetrates(&b, &y1).is_some());

        // assert_eq!(b.penetrates(&Point, &y2), Some(Vec3::new(0.5, 0.0, 0.0)));
        // assert_eq!(Point.penetrates(&b, &y2), Some(Vec3::new(0.5, 0.0, 0.0)));
        assert_eq!(b.penetrates(&Point, &y2), Some(Vec3::new(-0.5, 0.0, 0.0)));
        assert_eq!(Point.penetrates(&b, &y2), Some(Vec3::new(-0.5, 0.0, 0.0)));

        // assert_eq!(b.penetrates(&Point, &y3), Some(Vec3::new(0.0, -0.5, 0.0)));
        // assert_eq!(Point.penetrates(&b, &y3), Some(Vec3::new(0.0, -0.5, 0.0)));
        assert_eq!(b.penetrates(&Point, &y3), Some(Vec3::new(0.0, 0.5, 0.0)));
        assert_eq!(Point.penetrates(&b, &y3), Some(Vec3::new(0.0, 0.5, 0.0)));

        assert_eq!(None, b.penetrates(&Point, &n1));
        assert_eq!(None, b.penetrates(&Point, &n2));
        assert_eq!(None, b.penetrates(&Point, &n3));
        assert_eq!(None, b.penetrates(&Point, &n4));
        assert_eq!(None, b.penetrates(&Point, &n5));
        assert_eq!(None, Point.penetrates(&b, &n1));
        assert_eq!(None, Point.penetrates(&b, &n2));
        assert_eq!(None, Point.penetrates(&b, &n3));
        assert_eq!(None, Point.penetrates(&b, &n4));
        assert_eq!(None, Point.penetrates(&b, &n5));
    }

    #[test]
    fn point_v_box_sdf() {
        let b = Box3d::with_halfdims(2.0, 1.0, 3.0);
        let y1 = Vec3::new(0.0, 0.0, 0.0);
        let y2 = Vec3::new(1.5, 0.0, 0.0);
        let y3 = Vec3::new(0.0, -0.5, 0.0);
        let n1 = Vec3::new(3.0, 0.0, 0.0);
        let n2 = Vec3::new(0.0, 2.0, 0.0);
        let n3 = Vec3::new(-3.0, 0.0, 0.0);
        let n4 = Vec3::new(0.0, -2.0, 0.0);
        let n5 = Vec3::new(-2.1, 1.1, 0.0);

        assert_eq!(b.sdf(&Point, &y1), -1.0);
        assert_eq!(b.sdf(&Point, &y2), -0.5);
        assert_eq!(b.sdf(&Point, &n1), 1.0);
        assert_eq!(b.sdf(&Point, &n2), 1.0);
        assert_eq!(b.sdf(&Point, &n3), 1.0);
        assert_eq!(b.sdf(&Point, &n4), 1.0);
        assert_relative_eq!(b.sdf(&Point, &n5), 0.1414213);
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
            // assert_eq!(
            //     Some(Vec3::new(1.0, 0.0, 0.0) - delta),
            //     b.penetrates(&b, &delta)
            // );
            assert_eq!(
                Some(delta - Vec3::new(1.0, 0.0, 0.0)),
                b.penetrates(&b, &delta)
            );
        }
    }
}
