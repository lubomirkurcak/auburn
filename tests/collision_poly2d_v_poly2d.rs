use auburn::{assert_approx_eq, col2d::*, trace};

use auburn::utils::approx::Approx;
use common::F32Iterator;

mod common;

const X_COLLISION_EXPECTATIONS: &[(Vec2, bool)] = &[
    (Vec2::new(5.0, 0.0), false),
    (Vec2::new(4.5, 0.0), false),
    (Vec2::new(4.0, 0.0), false),
    (Vec2::new(3.5, 0.0), false),
    (Vec2::new(3.0, 0.0), false),
    (Vec2::new(2.5, 0.0), false),
    (Vec2::new(2.001, 0.0), false),
    // (Vec2::new(2.0, 0.0), false),
    (Vec2::new(1.999, 0.0), true),
    (Vec2::new(1.5, 0.0), true),
    (Vec2::new(1.0, 0.0), true),
    (Vec2::new(0.5, 0.0), true),
    (Vec2::new(0.0, 0.0), true),
    (Vec2::new(-0.5, 0.0), true),
    (Vec2::new(-1.0, 0.0), true),
    (Vec2::new(-1.5, 0.0), true),
    (Vec2::new(-1.999, 0.0), true),
    // (Vec2::new(-2.0, 0.0), false),
    (Vec2::new(-2.001, 0.0), false),
    (Vec2::new(-2.5, 0.0), false),
    (Vec2::new(-3.0, 0.0), false),
    (Vec2::new(-3.5, 0.0), false),
    (Vec2::new(-4.0, 0.0), false),
    (Vec2::new(-4.5, 0.0), false),
    (Vec2::new(-5.0, 0.0), false),
];

const DIAG_COLLISION_EXPECTATIONS: &[(Vec2, bool)] = &[
    (Vec2::new(5.0, 5.0), false),
    (Vec2::new(4.5, 4.5), false),
    (Vec2::new(4.0, 4.0), false),
    (Vec2::new(3.5, 3.5), false),
    (Vec2::new(3.0, 3.0), false),
    (Vec2::new(2.5, 2.5), false),
    (Vec2::new(2.001, 2.001), false),
    // (Vec2::new(2.0, 2.0), false),
    (Vec2::new(1.999, 1.999), true),
    (Vec2::new(1.5, 1.5), true),
    (Vec2::new(1.0, 1.0), true),
    (Vec2::new(0.5, 0.5), true),
    (Vec2::new(0.0, 0.0), true),
    (Vec2::new(-0.5, -0.5), true),
    (Vec2::new(-1.0, -1.0), true),
    (Vec2::new(-1.5, -1.5), true),
    (Vec2::new(-1.999, -1.999), true),
    // (Vec2::new(-2.0, -2.0), false),
    (Vec2::new(-2.001, -2.001), false),
    (Vec2::new(-2.5, -2.5), false),
    (Vec2::new(-3.0, -3.0), false),
    (Vec2::new(-3.5, -3.5), false),
    (Vec2::new(-4.0, -4.0), false),
    (Vec2::new(-4.5, -4.5), false),
    (Vec2::new(-5.0, -5.0), false),
];

#[test_log::test]
fn collision_x() {
    for &(offset, expect) in X_COLLISION_EXPECTATIONS {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.collides(b));
    }
}

#[test_log::test]
fn collision_x_90_degree_rotation_variants() {
    let angle_cases: &[f32] = &[0.0, 90.0, 180.0, 270.0];
    for &a_angle in angle_cases {
        for &b_angle in angle_cases {
            for &(offset, expect) in X_COLLISION_EXPECTATIONS {
                println!("a_angle: {:?}", a_angle);
                println!("b_angle: {:?}", b_angle);
                println!("offset: {:?}", offset);
                println!("expect: {:?}", expect);
                let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
                let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
                let a_transform = Transform2d {
                    pos: Vec2::ZERO,
                    rot: Rotor2d::from_angle(a_angle.to_radians()),
                    scale: Vec2::ONE,
                };
                let b_transform = Transform2d {
                    pos: offset,
                    rot: Rotor2d::from_angle(b_angle.to_radians()),
                    scale: Vec2::ONE,
                };
                let a = (&a_shape, &a_transform);
                let b = (&b_shape, &b_transform);
                assert_eq!(expect, a.collides(b));
            }
        }
    }
}

#[test_log::test]
fn distance_to_x() {
    let tests = [
        (Vec2::new(5.0, 0.0), Some(Vec2::new(3.0, 0.0))),
        (Vec2::new(4.5, 0.0), Some(Vec2::new(2.5, 0.0))),
        (Vec2::new(4.0, 0.0), Some(Vec2::new(2.0, 0.0))),
        (Vec2::new(3.5, 0.0), Some(Vec2::new(1.5, 0.0))),
        (Vec2::new(3.0, 0.0), Some(Vec2::new(1.0, 0.0))),
        (Vec2::new(2.5, 0.0), Some(Vec2::new(0.5, 0.0))),
        (Vec2::new(2.0, 0.0), Some(Vec2::new(0.0, 0.0))),
        (Vec2::new(1.5, 0.0), None),
        (Vec2::new(1.0, 0.0), None),
        (Vec2::new(0.5, 0.0), None),
        (Vec2::new(0.0, 0.0), None),
        (Vec2::new(-0.5, 0.0), None),
        (Vec2::new(-1.0, 0.0), None),
        (Vec2::new(-1.5, 0.0), None),
        (Vec2::new(-2.0, 0.0), Some(Vec2::new(0.0, 0.0))),
        (Vec2::new(-2.5, 0.0), Some(Vec2::new(-0.5, 0.0))),
        (Vec2::new(-3.0, 0.0), Some(Vec2::new(-1.0, 0.0))),
        (Vec2::new(-3.5, 0.0), Some(Vec2::new(-1.5, 0.0))),
        (Vec2::new(-4.0, 0.0), Some(Vec2::new(-2.0, 0.0))),
        (Vec2::new(-4.5, 0.0), Some(Vec2::new(-2.5, 0.0))),
        (Vec2::new(-5.0, 0.0), Some(Vec2::new(-3.0, 0.0))),
    ];
    for &(offset, expect) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);

        let a: Collider2d<'_, _, Translate2d> = (&a_shape, &a_pos).into();
        let b: Collider2d<'_, _, Translate2d> = (&b_shape, &b_pos).into();
        assert_eq!(expect, a.distance_to(b));

        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.distance_to(b));
    }
}

#[test_log::test]
fn sdfv_x() {
    let tests = [
        (Vec2::new(5.0, 0.0), (false, Vec2::new(3.0, 0.0))),
        (Vec2::new(4.5, 0.0), (false, Vec2::new(2.5, 0.0))),
        (Vec2::new(4.0, 0.0), (false, Vec2::new(2.0, 0.0))),
        (Vec2::new(3.5, 0.0), (false, Vec2::new(1.5, 0.0))),
        (Vec2::new(3.0, 0.0), (false, Vec2::new(1.0, 0.0))),
        (Vec2::new(2.5, 0.0), (false, Vec2::new(0.5, 0.0))),
        (Vec2::new(2.0, 0.0), (false, Vec2::new(0.0, 0.0))),
        (Vec2::new(1.5, 0.0), (true, Vec2::new(-0.5, 0.0))),
        (Vec2::new(1.0, 0.0), (true, Vec2::new(-1.0, 0.0))),
        (Vec2::new(0.5, 0.0), (true, Vec2::new(-1.5, 0.0))),
        (Vec2::new(0.0, 0.0), (true, Vec2::new(-2.0, 0.0))),
        (Vec2::new(-0.5, 0.0), (true, Vec2::new(1.5, 0.0))),
        (Vec2::new(-1.0, 0.0), (true, Vec2::new(1.0, 0.0))),
        (Vec2::new(-1.5, 0.0), (true, Vec2::new(0.5, 0.0))),
        (Vec2::new(-2.0, 0.0), (false, Vec2::new(0.0, 0.0))),
        (Vec2::new(-2.5, 0.0), (false, Vec2::new(-0.5, 0.0))),
        (Vec2::new(-3.0, 0.0), (false, Vec2::new(-1.0, 0.0))),
        (Vec2::new(-3.5, 0.0), (false, Vec2::new(-1.5, 0.0))),
        (Vec2::new(-4.0, 0.0), (false, Vec2::new(-2.0, 0.0))),
        (Vec2::new(-4.5, 0.0), (false, Vec2::new(-2.5, 0.0))),
        (Vec2::new(-5.0, 0.0), (false, Vec2::new(-3.0, 0.0))),
    ];
    for &(offset, expect) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.sdfv(b));
    }
}

#[test_log::test]
fn collision_diag() {
    for &(offset, expect) in DIAG_COLLISION_EXPECTATIONS {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.collides(b));
    }
}

#[test_log::test]
fn collision_diag_90_degree_rotation_variants() {
    let angle_cases: &[f32] = &[0.0, 90.0, 180.0, 270.0];
    for &a_angle in angle_cases {
        for &b_angle in angle_cases {
            for &(offset, expect) in DIAG_COLLISION_EXPECTATIONS {
                println!("a_angle: {:?}", a_angle);
                println!("b_angle: {:?}", b_angle);
                println!("offset: {:?}", offset);
                println!("expect: {:?}", expect);
                let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
                let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
                let a_transform = Transform2d {
                    pos: Vec2::ZERO,
                    rot: Rotor2d::from_angle(a_angle.to_radians()),
                    scale: Vec2::ONE,
                };
                let b_transform = Transform2d {
                    pos: offset,
                    rot: Rotor2d::from_angle(b_angle.to_radians()),
                    scale: Vec2::ONE,
                };
                let a = (&a_shape, &a_transform);
                let b = (&b_shape, &b_transform);
                assert_eq!(expect, a.collides(b));
            }
        }
    }
}

#[test_log::test]
fn distance_to_diag() {
    let tests = [
        (Vec2::new(5.0, 5.0), Some(Vec2::new(3.0, 3.0))),
        (Vec2::new(4.5, 4.5), Some(Vec2::new(2.5, 2.5))),
        (Vec2::new(4.0, 4.0), Some(Vec2::new(2.0, 2.0))),
        (Vec2::new(3.5, 3.5), Some(Vec2::new(1.5, 1.5))),
        (Vec2::new(3.0, 3.0), Some(Vec2::new(1.0, 1.0))),
        (Vec2::new(2.5, 2.5), Some(Vec2::new(0.5, 0.5))),
        (Vec2::new(2.0, 2.0), Some(Vec2::new(0.0, 0.0))),
        (Vec2::new(1.5, 1.5), None),
        (Vec2::new(1.0, 1.0), None),
        (Vec2::new(0.5, 0.5), None),
        (Vec2::new(0.0, 0.0), None),
        (Vec2::new(-0.5, -0.5), None),
        (Vec2::new(-1.0, -1.0), None),
        (Vec2::new(-1.5, -1.5), None),
        (Vec2::new(-2.0, -2.0), Some(Vec2::new(0.0, 0.0))),
        (Vec2::new(-2.5, -2.5), Some(Vec2::new(-0.5, -0.5))),
        (Vec2::new(-3.0, -3.0), Some(Vec2::new(-1.0, -1.0))),
        (Vec2::new(-3.5, -3.5), Some(Vec2::new(-1.5, -1.5))),
        (Vec2::new(-4.0, -4.0), Some(Vec2::new(-2.0, -2.0))),
        (Vec2::new(-4.5, -4.5), Some(Vec2::new(-2.5, -2.5))),
        (Vec2::new(-5.0, -5.0), Some(Vec2::new(-3.0, -3.0))),
    ];
    for &(offset, expect) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.distance_to(b));
    }
}

#[test_log::test]
fn sdfv_diag() {
    let tests = [
        (Vec2::new(5.0, 5.0), (false, Vec2::new(3.0, 3.0))),
        (Vec2::new(4.5, 4.5), (false, Vec2::new(2.5, 2.5))),
        (Vec2::new(4.0, 4.0), (false, Vec2::new(2.0, 2.0))),
        (Vec2::new(3.5, 3.5), (false, Vec2::new(1.5, 1.5))),
        (Vec2::new(3.0, 3.0), (false, Vec2::new(1.0, 1.0))),
        (Vec2::new(2.5, 2.5), (false, Vec2::new(0.5, 0.5))),
        (Vec2::new(2.0, 2.0), (false, Vec2::new(0.0, 0.0))),
        (Vec2::new(1.5, 1.5), (true, Vec2::new(-0.5, 0.0))),
        (Vec2::new(1.0, 1.0), (true, Vec2::new(-1.0, 0.0))),
        (Vec2::new(0.5, 0.5), (true, Vec2::new(-1.5, 0.0))),
        (Vec2::new(0.0, 0.0), (true, Vec2::new(-2.0, 0.0))),
        (Vec2::new(-0.5, -0.5), (true, Vec2::new(1.5, 0.0))),
        (Vec2::new(-1.0, -1.0), (true, Vec2::new(1.0, 0.0))),
        (Vec2::new(-1.5, -1.5), (true, Vec2::new(0.5, 0.0))),
        (Vec2::new(-2.0, -2.0), (false, Vec2::new(0.0, 0.0))),
        (Vec2::new(-2.5, -2.5), (false, Vec2::new(-0.5, -0.5))),
        (Vec2::new(-3.0, -3.0), (false, Vec2::new(-1.0, -1.0))),
        (Vec2::new(-3.5, -3.5), (false, Vec2::new(-1.5, -1.5))),
        (Vec2::new(-4.0, -4.0), (false, Vec2::new(-2.0, -2.0))),
        (Vec2::new(-4.5, -4.5), (false, Vec2::new(-2.5, -2.5))),
        (Vec2::new(-5.0, -5.0), (false, Vec2::new(-3.0, -3.0))),
    ];
    for &(offset, expect) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.sdfv(b));
    }
}

#[test_log::test]
fn triangle_v_box_case_1() {
    let b = Box2d::with_halfdims(0.5, 0.5);
    let t2 = Transform2d {
        pos: Vec2::new(-0.88888913, 0.56944436),
        rot: Rotor2d::IDENTITY,
        scale: Vec2::ONE,
    };
    let col1 = Collider2d {
        shape: &b,
        transform: &Transform2d::IDENTITY,
    };
    let col2 = Collider2d {
        shape: &Poly2d::regular_upright(3, 1.0),
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(collides);
    trace!("sdfv: {sdfv}");
    // assert_approx_eq!(sdfv, Vec2::new());
}

#[test_log::test]
fn upright_triangle_v_box_descend() {
    let mut errors = 0;
    for offset in F32Iterator::new(2.0, 0.5, 100) {
        let col1 = Collider2d {
            shape: &Box2d::with_halfdims(0.5, 0.5),
            transform: &Transform2d::IDENTITY,
        };
        let col2 = Collider2d {
            shape: &Poly2d::regular_upright(3, 1.0),
            transform: &Transform2d {
                pos: Vec2::new(0.0, offset),
                rot: Rotor2d::IDENTITY,
                scale: Vec2::ONE,
            },
        };
        let (collides, sdfv) = col1.sdfv(col2);
        // assert!(collides);
        trace!("sdfv: {sdfv}");
        let expected = Vec2::new(0.0, offset - 1.0);
        if !sdfv.approx_eq_tolerance(&expected, 10e-5) {
            auburn::error!("expected offset: {expected:?}, got sdfv: {sdfv:?}");
            errors += 1;
        }
        // assert_approx_eq!(sdfv, Vec2::new(padding, padding));
    }
    assert_eq!(errors, 0);
}

#[test_log::test]
fn upright_triangle_v_box_descend_slight_offset() {
    let mut errors = 0;
    for offset in F32Iterator::new(2.0, 0.5, 100) {
        let col1 = Collider2d {
            shape: &Box2d::with_halfdims(0.5, 0.5),
            transform: &Transform2d::IDENTITY,
        };
        let col2 = Collider2d {
            shape: &Poly2d::regular_upright(3, 1.0),
            transform: &Transform2d {
                pos: Vec2::new(0.01, offset),
                rot: Rotor2d::IDENTITY,
                scale: Vec2::ONE,
            },
        };
        let (collides, sdfv) = col1.sdfv(col2);
        // assert!(collides);
        trace!("sdfv: {sdfv}");
        let expected = Vec2::new(0.0, offset - 1.0);
        if !sdfv.approx_eq_tolerance(&expected, 10e-5) {
            auburn::error!("expected offset: {expected:?}, got sdfv: {sdfv:?}");
            errors += 1;
        }
        // assert_approx_eq!(sdfv, Vec2::new(padding, padding));
    }
    assert_eq!(errors, 0);
}
