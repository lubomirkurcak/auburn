use super::*;

#[derive(Default, Debug, Clone, PartialEq)]
/// Apply translation, rotation, and scale in this order:
/// 1. Scale
/// 2. Rotate
/// 3. Translate
pub struct Isotropic3d {
    pos: Vec3,
    rot: Quat,
    scale: f32,
}

// trait Approx {
//     fn approx_eq(&self, other: &Self) -> bool;
// }
//
// #[cfg(feature = "approx")]
// impl Approx for Isotropic3d {
//     fn approx_eq(&self, other: &Self) -> bool {
//         self.pos.approx_eq(&other.pos)
//             && self.rot.approx_eq(&other.rot)
//             && self.scale.approx_eq(&other.scale)
//     }
// }

impl Transform3d for Isotropic3d {
    fn apply_origin(&self) -> Vec3 {
        self.pos
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.rot * (self.scale * point) + self.pos
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        self.rot.inverse() * (point - self.pos) / self.scale
    }
}

impl Invertible for Isotropic3d {
    fn inverse(&self) -> Self {
        Self {
            pos: self.unapply(Vec3::ZERO),
            rot: self.rot.inverse(),
            scale: 1.0 / self.scale,
        }
    }
}

impl Composable for Isotropic3d {
    fn compose(&self, other: &Self) -> Self {
        Self {
            pos: self.apply(other.pos),
            rot: self.rot * other.rot,
            scale: self.scale * other.scale,
        }
    }
}

// unit tests
#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use crate::assert_approx_eq;
    use crate::utils::approx::Approx;

    use super::*;

    // const ORIGIN: Vec3 = Vec3::ZERO;
    const X: Vec3 = Vec3::X;
    const Y: Vec3 = Vec3::Y;
    const Z: Vec3 = Vec3::Z;
    const ONE: Vec3 = Vec3::new(1.0, 1.0, 1.0);

    const T1: Isotropic3d = Isotropic3d {
        pos: Vec3::new(1.0, 2.0, 3.0),
        rot: Quat::IDENTITY,
        scale: 2.0,
    };
    const T2: Isotropic3d = Isotropic3d {
        pos: Vec3::new(4.0, 5.0, 6.0),
        rot: Quat::IDENTITY,
        scale: 3.0,
    };
    const T2_T1: Isotropic3d = Isotropic3d {
        pos: Vec3::new(7.0, 11.0, 15.0),
        rot: Quat::IDENTITY,
        scale: 6.0,
    };

    const T1_O: Vec3 = Vec3::new(1.0, 2.0, 3.0);
    const T1_X: Vec3 = Vec3::new(3.0, 2.0, 3.0);
    const T1_Y: Vec3 = Vec3::new(1.0, 4.0, 3.0);
    const T1_Z: Vec3 = Vec3::new(1.0, 2.0, 5.0);
    const T1_ONE: Vec3 = Vec3::new(3.0, 4.0, 5.0);

    const T2_O: Vec3 = Vec3::new(4.0, 5.0, 6.0);
    const T2_X: Vec3 = Vec3::new(7.0, 5.0, 6.0);
    const T2_Y: Vec3 = Vec3::new(4.0, 8.0, 6.0);
    const T3_Z: Vec3 = Vec3::new(4.0, 5.0, 9.0);
    const T2_ONE: Vec3 = Vec3::new(7.0, 8.0, 9.0);

    const T2_T1_O: Vec3 = Vec3::new(7.0, 11.0, 15.0);
    const T2_T1_X: Vec3 = Vec3::new(13.0, 11.0, 15.0);
    const T2_T1_Y: Vec3 = Vec3::new(7.0, 17.0, 15.0);
    const T2_T1_Z: Vec3 = Vec3::new(7.0, 11.0, 21.0);
    const T2_T1_ONE: Vec3 = Vec3::new(13.0, 17.0, 21.0);

    #[test]
    fn t1_origin() {
        assert_eq!(T1.apply_origin(), T1_O);
    }
    #[test]
    fn t1_x() {
        assert_eq!(T1.apply(X), T1_X);
    }
    #[test]
    fn t1_y() {
        assert_eq!(T1.apply(Y), T1_Y);
    }
    #[test]
    fn t1_z() {
        assert_eq!(T1.apply(Z), T1_Z);
    }
    #[test]
    fn t1_one() {
        assert_eq!(T1.apply(ONE), T1_ONE);
    }

    #[test]
    fn t2_origin() {
        assert_eq!(T2.apply_origin(), T2_O);
    }
    #[test]
    fn t2_x() {
        assert_eq!(T2.apply(X), T2_X);
    }
    #[test]
    fn t2_y() {
        assert_eq!(T2.apply(Y), T2_Y);
    }
    #[test]
    fn t2_z() {
        assert_eq!(T2.apply(Z), T3_Z);
    }
    #[test]
    fn t2_one() {
        assert_eq!(T2.apply(ONE), T2_ONE);
    }

    #[test]
    fn t2_t1_apply_o() {
        assert_eq!(T2.apply(T1.apply_origin()), T2_T1_O);
    }
    #[test]
    fn t2_t1_compose_apply_o() {
        assert_eq!(T2.compose(&T1).apply_origin(), T2_T1_O);
    }
    #[test]
    fn t2_t1_expected_apply_o() {
        assert_eq!(T2_T1.apply_origin(), T2_T1_O);
    }
    #[test]
    fn test_compose_matches_o() {
        assert_eq!(T2.compose(&T1).apply_origin(), T2.apply(T1.apply_origin()));
    }

    #[test]
    fn t2_t1_apply_x() {
        assert_eq!(T2.apply(T1.apply(X)), T2_T1_X);
    }
    #[test]
    fn t2_t1_compose_apply_x() {
        assert_eq!(T2.compose(&T1).apply(X), T2_T1_X);
    }
    #[test]
    fn t2_t1_expected_apply_x() {
        assert_eq!(T2_T1.apply(X), T2_T1_X);
    }
    #[test]
    fn test_compose_matches_x() {
        assert_eq!(T2.compose(&T1).apply(X), T2.apply(T1.apply(X)));
    }

    #[test]
    fn t2_t1_apply_y() {
        assert_eq!(T2.apply(T1.apply(Y)), T2_T1_Y);
    }
    #[test]
    fn t2_t1_compose_apply_y() {
        assert_eq!(T2.compose(&T1).apply(Y), T2_T1_Y);
    }
    #[test]
    fn t2_t1_expected_apply_y() {
        assert_eq!(T2_T1.apply(Y), T2_T1_Y);
    }
    #[test]
    fn test_compose_matches_y() {
        assert_eq!(T2.compose(&T1).apply(Y), T2.apply(T1.apply(Y)));
    }

    #[test]
    fn t2_t1_apply_one() {
        assert_eq!(T2.apply(T1.apply(ONE)), T2_T1_ONE);
    }
    #[test]
    fn t2_t1_compose_apply_one() {
        assert_eq!(T2.compose(&T1).apply(ONE), T2_T1_ONE);
    }
    #[test]
    fn t2_t1_expected_apply_one() {
        assert_eq!(T2_T1.apply(ONE), T2_T1_ONE);
    }
    #[test]
    fn test_compose_matches_one() {
        assert_eq!(T2.compose(&T1).apply(ONE), T2.apply(T1.apply(ONE)));
    }

    #[test]
    fn test_compose() {
        assert_eq!(T2.compose(&T1), T2_T1);
    }

    #[test]
    fn t2_t1_apply_z() {
        assert_eq!(T2.apply(T1.apply(Z)), T2_T1_Z);
    }
    #[test]
    fn t2_t1_compose_apply_z() {
        assert_eq!(T2.compose(&T1).apply(Z), T2_T1_Z);
    }
    #[test]
    fn t2_t1_expected_apply_z() {
        assert_eq!(T2_T1.apply(Z), T2_T1_Z);
    }
    #[test]
    fn test_compose_matches_z() {
        assert_eq!(T2.compose(&T1).apply(Z), T2.apply(T1.apply(Z)));
    }

    #[test]
    fn test_compose_match() {
        let t = Isotropic3d {
            pos: Vec3::new(1.0, 2.0, 3.0),
            rot: Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0), 2.1),
            scale: 2.0,
        };
        let u = Isotropic3d {
            pos: Vec3::new(4.0, 5.0, 6.0),
            rot: Quat::from_axis_angle(Vec3::new(4.0, 5.0, 6.0), 3.2),
            scale: 3.0,
        };

        // assert_eq!(t.compose(&u).apply_origin(), t.apply(u.apply_origin()));
        assert_approx_eq!(t.compose(&u).apply_origin(), t.apply(u.apply_origin()));
        // assert_eq!(t.compose(&u).apply(X), t.apply(u.apply(X)));
        assert_approx_eq!(t.compose(&u).apply(X), t.apply(u.apply(X)));
    }

    #[test]
    fn test_inverse_match() {
        let t = Isotropic3d {
            pos: Vec3::new(1.0, 2.0, 3.0),
            rot: Quat::from_axis_angle(Vec3::new(1.0, 2.0, 3.0).normalize(), 2.1),
            scale: 2.0,
        };
        let t_inv = t.inverse();
        let t_t_inv_t = t.compose(&t_inv);
        let t_inv_t = t_inv.compose(&t);

        println!("t: {:?}", t);
        println!("t_inv: {:?}", t_inv);
        println!("t_t_inv_t: {:?}", t_t_inv_t);
        println!("t_inv_t: {:?}", t_inv_t);

        let p = Vec3::new(4.0, 5.0, 6.0);
        let t_p = t.apply(p);
        let t_inv_t_p = t_inv.apply(t_p);

        assert_approx_eq!(p, t_inv_t_p);
    }

    #[test]
    fn test_rot() {
        let T3: Isotropic3d = Isotropic3d {
            pos: Vec3::new(1.0, 2.0, 3.0),
            rot: Quat::from_rotation_x(0.1),
            scale: 2.0,
        };
        let T4: Isotropic3d = Isotropic3d {
            pos: Vec3::new(4.0, 5.0, 6.0),
            rot: Quat::from_rotation_y(0.2),
            scale: 3.0,
        };
        let T4_T3: Isotropic3d = Isotropic3d {
            pos: Vec3::new(7.0, 12.0, 15.0),
            rot: Quat::from_rotation_z(0.3),
            scale: 6.0,
        };
        let T3_ORIGIN: Vec3 = Vec3::new(1.0, 2.0, 3.0);
        let T4_ORIGIN: Vec3 = Vec3::new(4.0, 5.0, 6.0);
        let T4_T3_ORIGIN: Vec3 = Vec3::new(7.0, 11.0, 15.0);
    }
}

#[cfg(feature = "bevy")]
impl From<bevy::prelude::Transform> for Isotropic3d {
    fn from(transform: bevy::prelude::Transform) -> Self {
        Self {
            pos: transform.translation,
            rot: transform.rotation,
            scale: transform.scale.x,
        }
    }
}

#[cfg(feature = "bevy")]
impl Transform3d for bevy::prelude::Transform {
    fn apply_origin(&self) -> Vec3 {
        Into::<Isotropic3d>::into(*self).apply_origin()
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        Into::<Isotropic3d>::into(*self).apply(point)
    }

    fn unapply(&self, point: Vec3) -> Vec3 {
        Into::<Isotropic3d>::into(*self).unapply(point)
    }
}
