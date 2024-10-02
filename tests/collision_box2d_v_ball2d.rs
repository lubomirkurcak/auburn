mod collider2d;
pub mod common;

use approx::assert_relative_eq;
use auburn::assert_approx_eq;
use auburn::col2d::*;
use auburn::trace;
use auburn::utils::approx::Approx;
use glam::Quat;

#[test_log::test]
fn case_1() {
    let boxx = Box2d::with_halfdims(0.5, 0.5);
    let ball = Ball::with_radius(0.5);
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
        shape: &boxx,
        transform: &t1,
    };
    let col2 = Collider2d {
        shape: &ball,
        transform: &t2,
    };
    let (collides, sdfv) = col1.sdfv(col2);
    assert!(!collides);
    assert!(sdfv.length() < 0.1);
}
