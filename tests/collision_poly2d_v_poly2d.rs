use std::sync::LazyLock;

use auburn::col2d::*;

use env_logger::{fmt::Formatter, Builder};
use std::io::Write;

static LOGGER: LazyLock<()> = LazyLock::new(|| {
    env_logger::builder()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .is_test(true)
        .try_init()
        .unwrap();
});

#[test]
fn collision_x() {
    LazyLock::force(&LOGGER);
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
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_pos = Translate2d::from(Vec2::ZERO);
        let b_pos = Translate2d::from(offset);
        let a = (&a_shape, &a_pos);
        let b = (&b_shape, &b_pos);
        assert_eq!(expect, a.collides(b));
    }
}

#[test]
fn distance_to_x() {
    LazyLock::force(&LOGGER);
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

#[test]
fn sdfv_x() {
    LazyLock::force(&LOGGER);
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

#[test]
fn collision_diag() {
    LazyLock::force(&LOGGER);
    let tests = [
        (Vec2::new(5.0, 5.0), false),
        (Vec2::new(4.5, 4.5), false),
        (Vec2::new(4.0, 4.0), false),
        (Vec2::new(3.5, 3.5), false),
        (Vec2::new(3.0, 3.0), false),
        (Vec2::new(2.5, 2.5), false),
        (Vec2::new(2.0, 2.0), false),
        (Vec2::new(1.5, 1.5), true),
        (Vec2::new(1.0, 1.0), true),
        (Vec2::new(0.5, 0.5), true),
        (Vec2::new(0.0, 0.0), true),
        (Vec2::new(-0.5, -0.5), true),
        (Vec2::new(-1.0, -1.0), true),
        (Vec2::new(-1.5, -1.5), true),
        (Vec2::new(-2.0, -2.0), false),
        (Vec2::new(-2.5, -2.5), false),
        (Vec2::new(-3.0, -3.0), false),
        (Vec2::new(-3.5, -3.5), false),
        (Vec2::new(-4.0, -4.0), false),
        (Vec2::new(-4.5, -4.5), false),
        (Vec2::new(-5.0, -5.0), false),
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
        assert_eq!(expect, a.collides(b));
    }
}

#[test]
fn distance_to_diag() {
    LazyLock::force(&LOGGER);
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

#[test]
fn sdfv_diag() {
    LazyLock::force(&LOGGER);
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
