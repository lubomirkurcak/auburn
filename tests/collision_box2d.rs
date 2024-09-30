mod collider2d;
pub mod common;

use approx::assert_relative_eq;
use auburn::assert_approx_eq;
use auburn::col2d::*;
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

#[ignore]
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

#[ignore]
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
    let sdf = col1.sdf(col2);
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(sdf > 0.0);
    assert!(sdf < 0.1);
    assert!(!collides);
    assert!(sdfv.length() < 0.1);
}

#[ignore]
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
    let sdf = col1.sdf(col2);
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(sdf < 0.0);
    assert!(collides);
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
