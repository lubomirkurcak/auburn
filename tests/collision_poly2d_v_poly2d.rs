use auburn::col2d::*;

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