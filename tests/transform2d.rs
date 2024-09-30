use auburn::assert_approx_eq;
use auburn::col2d::*;
use auburn::utils::approx::Approx;

#[test_log::test]
fn identity_compose() {
    let a = Transform2d::IDENTITY;
    let b = Transform2d::IDENTITY;
    let c = a.compose(&b);
    assert_eq!(c, Transform2d::IDENTITY);
}

#[test_log::test]
fn identity_inverse() {
    let a = Transform2d::IDENTITY;
    assert_eq!(a.inverse(), Transform2d::IDENTITY);
}

#[test_log::test]
fn identity_apply() {
    let a = Transform2d::IDENTITY;
    for &p in POINTS {
        assert_eq!(a.apply_origin(), Vec2::ZERO);
        assert_eq!(a.apply(p), p);
        assert_eq!(a.unapply(p), p);
        assert_eq!(a.apply_normal(p), p);
        assert_eq!(a.unapply_normal(p), p);
    }
}

#[test_log::test]
fn translate_inverse() {
    for &p in POINTS {
        let a = Transform2d::from_translation(p);
        let b = a.inverse();
        assert_eq!(b.apply_origin(), -p);
    }
}

#[test_log::test]
fn translate_compose_1() {
    for &p in POINTS {
        let a = Transform2d::from_translation(p);
        let b = Transform2d::from_translation(-p);
        let c = a.compose(&b);
        assert_eq!(c, Transform2d::IDENTITY);
    }
}

#[test_log::test]
fn translate_compose_2() {
    for &p in POINTS {
        for &q in POINTS {
            let a = Transform2d::from_translation(p);
            let b = Transform2d::from_translation(q);
            let c = a.compose(&b);
            assert_eq!(c.apply_origin(), p + q);
        }
    }
}

#[test_log::test]
fn rotate_90_apply() {
    let a = Transform2d::from_angle(std::f32::consts::FRAC_PI_2);
    for &p in POINTS {
        let rotated = a.apply(p);
        assert_approx_eq!(rotated, Vec2::new(-p.y, p.x));
    }
}

#[test_log::test]
fn rotate_180_apply() {
    let a = Transform2d::from_angle(std::f32::consts::PI);
    for &p in POINTS {
        let rotated = a.apply(p);
        assert_approx_eq!(rotated, Vec2::new(-p.x, -p.y));
    }
}

#[test_log::test]
fn rotate_270_apply() {
    let a = Transform2d::from_angle(std::f32::consts::PI + std::f32::consts::FRAC_PI_2);
    for &p in POINTS {
        let rotated = a.apply(p);
        assert_approx_eq!(rotated, Vec2::new(p.y, -p.x));
    }
}

#[test_log::test]
fn rotate_90_unapply() {
    let a = Transform2d::from_angle(std::f32::consts::FRAC_PI_2);
    for &p in POINTS {
        let rotated = a.unapply(p);
        assert_approx_eq!(rotated, Vec2::new(p.y, -p.x));
    }
}

#[test_log::test]
fn rotate_180_unapply() {
    let a = Transform2d::from_angle(std::f32::consts::PI);
    for &p in POINTS {
        let rotated = a.unapply(p);
        assert_approx_eq!(rotated, Vec2::new(-p.x, -p.y));
    }
}

#[test_log::test]
fn rotate_270_unapply() {
    let a = Transform2d::from_angle(std::f32::consts::PI + std::f32::consts::FRAC_PI_2);
    for &p in POINTS {
        let rotated = a.unapply(p);
        assert_approx_eq!(rotated, Vec2::new(-p.y, p.x));
    }
}

#[test_log::test]
fn delta_transform_1() {
    let a = Transform2d::IDENTITY;
    let b =
        Transform2d::from_translation(Vec2::new(0.0, 3.0)).with_angle(std::f32::consts::FRAC_PI_2);
    let c = a.delta_transform(&b);
    assert_eq!(c, b);
}

const RIGHT_ANGLES: &[f32] = &[
    0.0,
    std::f32::consts::FRAC_PI_2,
    std::f32::consts::PI,
    std::f32::consts::PI + std::f32::consts::FRAC_PI_2,
];

const POINTS: &[Vec2] = &[
    Vec2::new(0.0, 0.0),
    Vec2::new(1.0, 0.0),
    Vec2::new(1.0, 1.0),
    Vec2::new(0.0, 1.0),
    Vec2::new(0.5, 0.5),
    Vec2::new(0.5, 0.0),
    Vec2::new(0.0, 0.5),
    Vec2::new(0.5, 1.0),
    Vec2::new(1.0, 0.5),
    Vec2::new(0.25, 0.25),
    Vec2::new(0.75, 0.25),
    Vec2::new(0.75, 0.75),
    Vec2::new(0.25, 0.75),
    Vec2::new(0.25, 0.5),
    Vec2::new(0.5, 0.25),
    Vec2::new(0.75, 0.5),
    Vec2::new(0.5, 0.75),
    Vec2::new(0.5, 0.25),
    Vec2::new(0.25, 0.75),
    Vec2::new(0.75, 0.25),
    Vec2::new(0.25, 0.25),
    Vec2::new(0.75, 0.75),
    Vec2::new(0.75, 0.5),
    Vec2::new(0.5, 0.75),
];
