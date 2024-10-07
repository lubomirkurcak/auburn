mod collider2d;
pub mod common;
use common::box_box_transform2d_distance_bounds_check;
use common::F32Iterator;

use approx::assert_relative_eq;
use auburn::assert_approx_eq;
use auburn::col2d::*;
use auburn::trace;
use auburn::utils::approx::Approx;
use glam::Quat;

#[test_log::test]
fn local_extreme_points() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    assert_eq!(Vec2::new(2.0, 1.0), b.extreme_point(Vec2::new(1.0, 1.0)));
    assert_eq!(Vec2::new(2.0, -1.0), b.extreme_point(Vec2::new(1.0, -1.0)));
    assert_eq!(Vec2::new(-2.0, 1.0), b.extreme_point(Vec2::new(-1.0, 1.0)));
    assert_eq!(
        Vec2::new(-2.0, -1.0),
        b.extreme_point(Vec2::new(-1.0, -1.0))
    );
}

#[test_log::test]
fn extreme_points_translate2d() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let t = Vec2::new(0.5, 0.5);
    let c = Collider2d {
        shape: &b,
        transform: &t,
    };

    assert_eq!(Vec2::new(2.5, 1.5), c.extreme_point(Vec2::new(1.0, 1.0)));
    assert_eq!(Vec2::new(2.5, -0.5), c.extreme_point(Vec2::new(1.0, -1.0)));
    assert_eq!(Vec2::new(-1.5, 1.5), c.extreme_point(Vec2::new(-1.0, 1.0)));
    assert_eq!(
        Vec2::new(-1.5, -0.5),
        c.extreme_point(Vec2::new(-1.0, -1.0))
    );
}

#[test_log::test]
fn extreme_points_transform2d_rotate() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let t = Transform2d {
        pos: Vec2::ZERO,
        rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_2),
        scale: Vec2::ONE,
    };
    let c = Collider2d {
        shape: &b,
        transform: &t,
    };
    assert_approx_eq!(Vec2::new(1.0, 2.0), c.extreme_point(Vec2::new(1.0, 1.0)));
    assert_approx_eq!(Vec2::new(1.0, -2.0), c.extreme_point(Vec2::new(1.0, -1.0)));
    assert_approx_eq!(Vec2::new(-1.0, 2.0), c.extreme_point(Vec2::new(-1.0, 1.0)));
    assert_approx_eq!(
        Vec2::new(-1.0, -2.0),
        c.extreme_point(Vec2::new(-1.0, -1.0))
    );
}

#[test_log::test]
fn extreme_points_transform2d_rotate_from_quat() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let t = Transform2d {
        pos: Vec2::ZERO,
        rot: Rotor2d::from_quaternion(Quat::from_rotation_z(core::f32::consts::FRAC_PI_2)),
        scale: Vec2::ONE,
    };
    let c = Collider2d {
        shape: &b,
        transform: &t,
    };
    assert_approx_eq!(Vec2::new(1.0, 2.0), c.extreme_point(Vec2::new(1.0, 1.0)));
    assert_approx_eq!(Vec2::new(1.0, -2.0), c.extreme_point(Vec2::new(1.0, -1.0)));
    assert_approx_eq!(Vec2::new(-1.0, 2.0), c.extreme_point(Vec2::new(-1.0, 1.0)));
    assert_approx_eq!(
        Vec2::new(-1.0, -2.0),
        c.extreme_point(Vec2::new(-1.0, -1.0))
    );
}

#[test_log::test]
fn extreme_points_transform2d_rotate_translate() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let t = Transform2d {
        pos: Vec2::new(0.5, 0.5),
        rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_2),
        scale: Vec2::ONE,
    };
    let c = Collider2d {
        shape: &b,
        transform: &t,
    };
    assert_approx_eq!(Vec2::new(1.5, 2.5), c.extreme_point(Vec2::new(1.0, 1.0)));
    assert_approx_eq!(Vec2::new(1.5, -1.5), c.extreme_point(Vec2::new(1.0, -1.0)));
    assert_approx_eq!(Vec2::new(-0.5, 2.5), c.extreme_point(Vec2::new(-1.0, 1.0)));
    assert_approx_eq!(
        Vec2::new(-0.5, -1.5),
        c.extreme_point(Vec2::new(-1.0, -1.0))
    );
}

#[test_log::test]
fn point_v_box_collides() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let y1 = Vec2::new(0.0, 0.0);
    let y2 = Vec2::new(1.5, 0.0);
    let y3 = Vec2::new(0.0, -0.5);
    let n1 = Vec2::new(3.0, 0.0);
    let n2 = Vec2::new(0.0, 2.0);
    let n3 = Vec2::new(-3.0, 0.0);
    let n4 = Vec2::new(0.0, -2.0);
    let n5 = Vec2::new(-2.1, 1.1);

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

#[test_log::test]
fn point_v_box_penetrates() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let y1 = Vec2::new(0.0, 0.0);
    let y2 = Vec2::new(1.5, 0.0);
    let y3 = Vec2::new(0.0, -0.5);
    let n1 = Vec2::new(3.0, 0.0);
    let n2 = Vec2::new(0.0, 2.0);
    let n3 = Vec2::new(-3.0, 0.0);
    let n4 = Vec2::new(0.0, -2.0);
    let n5 = Vec2::new(-2.1, 1.1);

    // TODO(lubo): Actualy check if this is reasonable
    assert!(b.penetrates_rel(&Point, &y1).is_some());
    assert!(Point.penetrates_rel(&b, &y1).is_some());

    // assert_eq!(b.penetrates(&Point, &y2), Some(Vec2::new(0.5, 0.0)));
    // assert_eq!(Point.penetrates(&b, &y2), Some(Vec2::new(0.5, 0.0)));
    assert_eq!(b.penetrates_rel(&Point, &y2), Some(Vec2::new(-0.5, 0.0)));
    assert_eq!(Point.penetrates_rel(&b, &y2), Some(Vec2::new(-0.5, 0.0)));

    // assert_eq!(b.penetrates(&Point, &y3), Some(Vec2::new(0.0, -0.5)));
    // assert_eq!(Point.penetrates(&b, &y3), Some(Vec2::new(0.0, -0.5)));
    assert_eq!(b.penetrates_rel(&Point, &y3), Some(Vec2::new(0.0, 0.5)));
    assert_eq!(Point.penetrates_rel(&b, &y3), Some(Vec2::new(0.0, 0.5)));

    assert_eq!(None, b.penetrates_rel(&Point, &n1));
    assert_eq!(None, b.penetrates_rel(&Point, &n2));
    assert_eq!(None, b.penetrates_rel(&Point, &n3));
    assert_eq!(None, b.penetrates_rel(&Point, &n4));
    assert_eq!(None, b.penetrates_rel(&Point, &n5));

    assert_eq!(None, Point.penetrates_rel(&b, &n1));
    assert_eq!(None, Point.penetrates_rel(&b, &n2));
    assert_eq!(None, Point.penetrates_rel(&b, &n3));
    assert_eq!(None, Point.penetrates_rel(&b, &n4));
    assert_eq!(None, Point.penetrates_rel(&b, &n5));
}

#[test_log::test]
fn point_v_box_sdf() {
    let b = Box2d::with_halfdims(2.0, 1.0);
    let y1 = Vec2::new(0.0, 0.0);
    let y2 = Vec2::new(1.5, 0.0);
    let y3 = Vec2::new(0.0, -0.5);
    let n1 = Vec2::new(3.0, 0.0);
    let n2 = Vec2::new(0.0, 2.0);
    let n3 = Vec2::new(-3.0, 0.0);
    let n4 = Vec2::new(0.0, -2.0);
    let n5 = Vec2::new(-2.1, 1.1);

    assert_eq!(b.sdf_rel(&Point, &y1), -1.0);
    assert_eq!(b.sdf_rel(&Point, &y2), -0.5);
    assert_eq!(b.sdf_rel(&Point, &n1), 1.0);
    assert_eq!(b.sdf_rel(&Point, &n2), 1.0);
    assert_eq!(b.sdf_rel(&Point, &n3), 1.0);
    assert_eq!(b.sdf_rel(&Point, &n4), 1.0);
    assert_relative_eq!(b.sdf_rel(&Point, &n5), 0.1414213);
}

#[test_log::test]
fn box_v_box_no_collision() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let delta = Vec2::new(2.0, 0.0);
    assert!(!b.collides_rel(&b, &delta));
    assert_eq!(b.penetrates_rel(&b, &delta), None);
}

#[test_log::test]
fn box_v_box_perfect_overlap() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let delta = Vec2::new(0.0, 0.0);
    assert!(b.collides_rel(&b, &delta));
    // TODO(lubo): Actualy check if this is reasonable
    assert!(b.penetrates_rel(&b, &delta).is_some());
}

#[test_log::test]
fn box_v_box_collision() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    auburn::warn!("b: {:?}", b);
    for i in 1..=9 {
        let delta = Vec2::new(i as f32 / 10.0, 0.0);
        assert!(b.collides_rel(&b, &delta));
        // assert_eq!(Some(Vec2::new(1.0, 0.0) - delta), b.penetrates(&b, &delta));
        assert_eq!(
            Some(delta - Vec2::new(1.0, 0.0)),
            b.penetrates_rel(&b, &delta)
        );
    }
}

#[test_log::test]
fn box_v_box_collision_br() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    for i in 1..=9 {
        let delta = Vec2::new(i as f32 / 10.0, -0.0);
        assert!(b.collides_rel(&b, &delta));
        // assert_eq!(Some(Vec2::new(1.0, 0.0) - delta), b.penetrates(&b, &delta));
        assert_eq!(
            Some(delta - Vec2::new(1.0, 0.0)),
            b.penetrates_rel(&b, &delta)
        );
    }
}

#[test_log::test]
fn poly_from_box_v_poly_from_box_collision_random_case_1() {
    let b: Poly2d = Box2d::with_halfdims(0.5, 0.5).into();
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.49225292, 0.8704522)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(0.9166665, -0.73611116),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.36250287, 0.93198246)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    // let sdf = col1.sdf(col2);
    let (collides, sdfv) = col1.sdfv(col2);
    // assert!(sdf > 0.0);
    // assert!(sdf < 0.1);
    assert!(!collides);
    assert!(sdfv.length() < 0.1);
}

#[test_log::test]
fn box_v_box_collision_random_case_1() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.49225292, 0.8704522)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(0.9166665, -0.73611116),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.36250287, 0.93198246)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    let distance = sdfv.length();
    trace!("sdfv length: {distance}");
    box_box_transform2d_distance_bounds_check(col1, col2);
    assert!(sdfv.length() < 0.1);
}

#[test_log::test]
fn box_v_box_collision_random_case_2() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.97009665, -0.24271953)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(-0.2638891, -0.4861111),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.8197918, -0.5726604)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(collides);
    let distance = if collides { -1.0 } else { 1.0 } * sdfv.length();
    box_box_transform2d_distance_bounds_check(col1, col2);
}

#[test_log::test]
fn box_v_box_collision_random_case_3() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.27712706, 0.96083343)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(1.6249996, 0.80555564),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.25829336, 0.9660661)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    let distance = sdfv.length();
    trace!("sdfv length: {distance}");
    box_box_transform2d_distance_bounds_check(col1, col2);
}

#[test_log::test]
fn box_v_box_collision_random_case_4() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, 0.07703778, 0.9970285)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(8.097223, 0.43055537),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, 0.052785538, 0.9986058)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    let distance = sdfv.length();
    trace!("sdfv length: {distance}");
    box_box_transform2d_distance_bounds_check(col1, col2);
    assert!(distance > 7.0);
    assert!(distance < 7.1);
}

#[test_log::test]
fn box_v_box_collision_random_case_5() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, 0.53144556, 0.8470897)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(2.7638893, -0.097222336),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, 0.62631613, 0.77956754)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    box_box_transform2d_distance_bounds_check(col1, col2);
    assert!(sdfv.length() > 1.0);
}

#[test_log::test]
fn box_v_box_collision_random_case_6() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.29803792, 0.9545537)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(-1.1249999, -2.1527781),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.25837365, 0.96604484)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    box_box_transform2d_distance_bounds_check(col1, col2);
    assert!(sdfv.length() > 1.0);
}

#[test_log::test]
fn box_v_box_collision_random_case_7() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d {
        pos: Vec2::new(0.0, 0.0),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.045578003, 0.9989599)),
        scale: Vec2::new(1.0, 1.0),
    };
    let t2 = Transform2d {
        pos: Vec2::new(7.291667, 0.3611112),
        rot: Rotor2d::from_quaternion(Quat::from_xyzw(0.0, 0.0, -0.035485897, 0.99936825)),
        scale: Vec2::new(1.0, 1.0),
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    box_box_transform2d_distance_bounds_check(col1, col2);
    assert!(sdfv.length() > 1.0);
}

#[test_log::test]
fn box_v_box_collision_diamond_with_offset() {
    let mut errors = 0;
    for offset in F32Iterator::new(0.0, core::f32::consts::SQRT_2, 100) {
        let b = Box2d::with_halfdims(0.5, 0.5);
        let t1 = Transform2d {
            pos: Vec2::new(0.0, 0.0),
            rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_4),
            scale: Vec2::new(1.0, 1.0),
        };
        let padding = 0.0125;
        let t2 = Transform2d {
            pos: Vec2::new(
                offset + padding,
                core::f32::consts::SQRT_2 - offset + padding,
            ),
            rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_4),
            scale: Vec2::new(1.0, 1.0),
        };
        let col1 = Collider2d {
            shape: &b,
            transform: &t1,
        };
        let col2 = Collider2d {
            shape: &b,
            transform: &t2,
        };
        // let sdf = col1.sdf(col2);
        let (collides, sdfv) = col1.sdfv(col2);
        assert!(!collides);
        if !sdfv.approx_eq_tolerance(&Vec2::new(padding, padding), 10e-6) {
            auburn::error!("expected offset: {padding:?}, got sdfv: {sdfv:?}");
            errors += 1;
        }
        // assert_approx_eq!(sdfv, Vec2::new(padding, padding));
    }
    assert_eq!(errors, 0);
}

#[test_log::test]
fn box_v_box_collision_diamond_with_offset_mod1() {
    let mut errors = 0;
    for offset in F32Iterator::new(0.0, core::f32::consts::SQRT_2, 100) {
        let b = Box2d::with_halfdims(0.5, 0.5);
        let t1 = Transform2d {
            pos: Vec2::new(0.0, 0.0),
            rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_4 + 0.002),
            scale: Vec2::new(1.0, 1.0),
        };
        let padding = 0.0125;
        let t2 = Transform2d {
            pos: Vec2::new(
                offset + padding,
                core::f32::consts::SQRT_2 - offset + padding,
            ),
            rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_4),
            scale: Vec2::new(1.0, 1.0),
        };
        let col1 = Collider2d {
            shape: &b,
            transform: &t1,
        };
        let col2 = Collider2d {
            shape: &b,
            transform: &t2,
        };
        // let sdf = col1.sdf(col2);
        let (collides, sdfv) = col1.sdfv(col2);
        assert!(!collides);
        if !sdfv.approx_eq_tolerance(&Vec2::new(padding, padding), 10e-1) {
            auburn::error!(
                "expected offset: {:?}, got: {:?}",
                Vec2::new(padding, padding),
                sdfv
            );
            errors += 1;
        }
        // assert_approx_eq!(sdfv, Vec2::new(padding, padding));
    }
    assert_eq!(errors, 0);
}

#[test_log::test]
fn collision_x() {
    let tests = [
        (Vec2::new(5.0, 0.0), false),
        (Vec2::new(4.5, 0.0), false),
        (Vec2::new(4.0, 0.0), false),
        (Vec2::new(3.5, 0.0), false),
        (Vec2::new(3.0, 0.0), false),
        (Vec2::new(2.5, 0.0), false),
        (Vec2::new(2.0, 0.0), false),
        (Vec2::new(1.5, 0.0), true),
        (Vec2::new(1.0, 0.0), true),
        (Vec2::new(0.5, 0.0), true),
        (Vec2::new(0.0, 0.0), true),
        (Vec2::new(-0.5, 0.0), true),
        (Vec2::new(-1.0, 0.0), true),
        (Vec2::new(-1.5, 0.0), true),
        (Vec2::new(-2.0, 0.0), false),
        (Vec2::new(-2.5, 0.0), false),
        (Vec2::new(-3.0, 0.0), false),
        (Vec2::new(-3.5, 0.0), false),
        (Vec2::new(-4.0, 0.0), false),
        (Vec2::new(-4.5, 0.0), false),
        (Vec2::new(-5.0, 0.0), false),
    ];
    for &(offset, expect) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape = Box2d::with_halfdims(1.0, 1.0);
        let b_shape = Box2d::with_halfdims(1.0, 1.0);
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.collides(b));
    }
}

#[test_log::test]
fn box_v_point_case_1() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d::IDENTITY;
    let t2 = Transform2d {
        pos: Vec2::new(1.0, 0.0),
        rot: Rotor2d::IDENTITY,
        scale: Vec2::ONE,
    };
    let col1 = Collider2d {
        shape: &Point,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    trace!("sdfv: {sdfv}");
    assert_approx_eq!(sdfv, Vec2::new(0.5, 0.0));
}

#[test_log::test]
fn box_v_point_case_2() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t1 = Transform2d::IDENTITY;
    let t2 = Transform2d {
        pos: Vec2::new(1.0, 0.0),
        rot: Rotor2d::from_angle(core::f32::consts::FRAC_PI_4),
        scale: Vec2::ONE,
    };
    let col1 = Collider2d {
        shape: &Point,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &b,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    trace!("sdfv: {sdfv}");
    assert_approx_eq!(sdfv, Vec2::new(1.0 - core::f32::consts::FRAC_1_SQRT_2, 0.0));
}
