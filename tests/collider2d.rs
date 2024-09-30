use auburn::utils::approx::Approx;
use auburn::{assert_approx_eq, col2d::*, debug, info};

#[test_log::test]
fn collider2d_box() {
    let a = Box2d::with_halfdims(1.0, 1.0);
    assert_eq!(a.extreme_point(Vec2::X), Vec2::new(1.0, 1.0));
    assert_eq!(a.extreme_point(Vec2::Y), Vec2::new(1.0, 1.0));
    assert_eq!(a.extreme_point(-Vec2::X), Vec2::new(-1.0, -1.0));
    assert_eq!(a.extreme_point(-Vec2::Y), Vec2::new(-1.0, -1.0));
    assert_eq!(
        a.extreme_point(Vec2::new(-1.0, -1.0)),
        Vec2::new(-1.0, -1.0)
    );
    assert_eq!(a.extreme_point(Vec2::new(-1.0, 1.0)), Vec2::new(-1.0, 1.0));
    assert_eq!(a.extreme_point(Vec2::new(1.0, -1.0)), Vec2::new(1.0, -1.0));
    assert_eq!(a.extreme_point(Vec2::new(1.0, 1.0)), Vec2::new(1.0, 1.0));
}

#[test_log::test]
fn collider2d_box_translate() {
    let shape = Box2d::with_halfdims(1.0, 1.0);
    let translations = &[
        Vec2::new(1.0, 0.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(-1.0, 0.0),
        Vec2::new(0.0, -1.0),
    ];
    for &translation in translations {
        let t = Translate2d::from(translation);
        let a: Collider2d<_, _> = (&shape, &t).into();
        assert_eq!(a.extreme_point(Vec2::X), translation + Vec2::new(1.0, 1.0));
        assert_eq!(a.extreme_point(Vec2::Y), translation + Vec2::new(1.0, 1.0));
        assert_eq!(
            a.extreme_point(-Vec2::X),
            translation + Vec2::new(-1.0, -1.0)
        );
        assert_eq!(
            a.extreme_point(-Vec2::Y),
            translation + Vec2::new(-1.0, -1.0)
        );
        assert_eq!(
            a.extreme_point(Vec2::new(-1.0, -1.0)),
            translation + Vec2::new(-1.0, -1.0)
        );
        assert_eq!(
            a.extreme_point(Vec2::new(-1.0, 1.0)),
            translation + Vec2::new(-1.0, 1.0)
        );
        assert_eq!(
            a.extreme_point(Vec2::new(1.0, -1.0)),
            translation + Vec2::new(1.0, -1.0)
        );
        assert_eq!(
            a.extreme_point(Vec2::new(1.0, 1.0)),
            translation + Vec2::new(1.0, 1.0)
        );
    }
}

#[test_log::test]
fn collider2d_box_rotate_45() {
    let s = core::f32::consts::SQRT_2;
    let shape = Box2d::with_halfdims(1.0, 1.0);
    let rotations = &[
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_4),
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_2 + core::f32::consts::FRAC_PI_4),
        Rotor2d::from_angle(core::f32::consts::PI + core::f32::consts::FRAC_PI_4),
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_2 * 3.0 + core::f32::consts::FRAC_PI_4),
    ];
    for &rot in rotations {
        info!("rot: {:?}", rot);
        let t = Transform2d::from_rotation(rot);
        let a: Collider2d<_, _> = (&shape, &t).into();
        assert_approx_eq!(a.extreme_point(Vec2::X), Vec2::new(s, 0.0));
        assert_approx_eq!(a.extreme_point(Vec2::Y), Vec2::new(0.0, s));
        assert_approx_eq!(a.extreme_point(-Vec2::X), Vec2::new(-s, 0.0));
        assert_approx_eq!(a.extreme_point(-Vec2::Y), Vec2::new(0.0, -s));
    }
}

#[test_log::test]
fn collider2d_box_rotate() {
    let shape = Box2d::with_halfdims(1.0, 1.0);
    let rotations = &[
        Rotor2d::from_angle(0.0),
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_2),
        Rotor2d::from_angle(core::f32::consts::PI),
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_2 * 3.0),
    ];
    for &rot in rotations {
        info!("rot: {:?}", rot);
        let t = Transform2d::from_rotation(rot);
        let a: Collider2d<_, _> = (&shape, &t).into();
        assert_approx_eq!(a.extreme_point(Vec2::X).x, 1.0);
        assert_approx_eq!(a.extreme_point(Vec2::Y).y, 1.0);
        assert_approx_eq!(a.extreme_point(-Vec2::X).x, -1.0);
        assert_approx_eq!(a.extreme_point(-Vec2::Y).y, -1.0);
        assert_approx_eq!(
            a.extreme_point(Vec2::new(-1.0, -1.0)),
            Vec2::new(-1.0, -1.0)
        );
        assert_approx_eq!(a.extreme_point(Vec2::new(-1.0, 1.0)), Vec2::new(-1.0, 1.0));
        assert_approx_eq!(a.extreme_point(Vec2::new(1.0, -1.0)), Vec2::new(1.0, -1.0));
        assert_approx_eq!(a.extreme_point(Vec2::new(1.0, 1.0)), Vec2::new(1.0, 1.0));
    }
}

#[test_log::test]
fn collider2d_box_rotate_translate() {
    let shape = Box2d::with_halfdims(1.0, 1.0);
    let rotations = &[
        Rotor2d::from_angle(0.0),
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_2),
        Rotor2d::from_angle(core::f32::consts::PI),
        Rotor2d::from_angle(core::f32::consts::FRAC_PI_2 * 3.0),
    ];
    let translations = &[
        Vec2::new(1.0, 0.0),
        Vec2::new(0.0, 1.0),
        Vec2::new(-1.0, 0.0),
        Vec2::new(0.0, -1.0),
    ];
    for &rot in rotations {
        for &translation in translations {
            info!("rot: {:?}, translation: {:?}", rot, translation);
            let t = Transform2d::from_rotation(rot).with_translation(translation);
            let a: Collider2d<_, _> = (&shape, &t).into();
            assert_approx_eq!(a.extreme_point(Vec2::X).x, translation.x + 1.0);
            assert_approx_eq!(a.extreme_point(Vec2::Y).y, translation.y + 1.0);
            assert_approx_eq!(a.extreme_point(-Vec2::X).x, translation.x + -1.0);
            assert_approx_eq!(a.extreme_point(-Vec2::Y).y, translation.y + -1.0);
            assert_approx_eq!(
                a.extreme_point(Vec2::new(-1.0, -1.0)),
                translation + Vec2::new(-1.0, -1.0)
            );
            assert_approx_eq!(
                a.extreme_point(Vec2::new(-1.0, 1.0)),
                translation + Vec2::new(-1.0, 1.0)
            );
            assert_approx_eq!(
                a.extreme_point(Vec2::new(1.0, -1.0)),
                translation + Vec2::new(1.0, -1.0)
            );
            assert_approx_eq!(
                a.extreme_point(Vec2::new(1.0, 1.0)),
                translation + Vec2::new(1.0, 1.0)
            );
        }
    }
}
