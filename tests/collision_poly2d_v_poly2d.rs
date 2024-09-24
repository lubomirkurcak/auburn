use auburn::col2d::*;

#[test]
fn collision_poly2d_v_poly2d_x() {
    let tests = [
        (false, Vec2::new(5.0, 0.0)),
        (false, Vec2::new(4.5, 0.0)),
        (false, Vec2::new(4.0, 0.0)),
        (false, Vec2::new(3.5, 0.0)),
        (false, Vec2::new(3.0, 0.0)),
        (false, Vec2::new(2.5, 0.0)),
        (false, Vec2::new(2.0, 0.0)),
        (true, Vec2::new(1.5, 0.0)),
        (true, Vec2::new(1.0, 0.0)),
        (true, Vec2::new(0.5, 0.0)),
        (true, Vec2::new(0.0, 0.0)),
        (true, Vec2::new(-0.5, 0.0)),
        (true, Vec2::new(-1.0, 0.0)),
        (true, Vec2::new(-1.5, 0.0)),
        (false, Vec2::new(-2.0, 0.0)),
        (false, Vec2::new(-2.5, 0.0)),
        (false, Vec2::new(-3.0, 0.0)),
        (false, Vec2::new(-3.5, 0.0)),
        (false, Vec2::new(-4.0, 0.0)),
        (false, Vec2::new(-4.5, 0.0)),
        (false, Vec2::new(-5.0, 0.0)),
    ];
    for &(expect, offset) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let rel = Translate2d::from(offset);

        let (collides, _distance) = a.sdfv_common_rel(&b, &rel);
        println!("collides: {:?}", collides);
        assert_eq!(expect, collides);
    }
}

#[test]
fn collision_poly2d_v_poly2d_diag() {
    let tests = [
        (false, Vec2::new(5.0, 5.0)),
        (false, Vec2::new(4.5, 4.5)),
        (false, Vec2::new(4.0, 4.0)),
        (false, Vec2::new(3.5, 3.5)),
        (false, Vec2::new(3.0, 3.0)),
        (false, Vec2::new(2.5, 2.5)),
        (false, Vec2::new(2.0, 2.0)),
        (true, Vec2::new(1.5, 1.5)),
        (true, Vec2::new(1.0, 1.0)),
        (true, Vec2::new(0.5, 0.5)),
        (true, Vec2::new(0.0, 0.0)),
        (true, Vec2::new(-0.5, -0.5)),
        (true, Vec2::new(-1.0, -1.0)),
        (true, Vec2::new(-1.5, -1.5)),
        (false, Vec2::new(-2.0, -2.0)),
        (false, Vec2::new(-2.5, -2.5)),
        (false, Vec2::new(-3.0, -3.0)),
        (false, Vec2::new(-3.5, -3.5)),
        (false, Vec2::new(-4.0, -4.0)),
        (false, Vec2::new(-4.5, -4.5)),
        (false, Vec2::new(-5.0, -5.0)),
    ];
    for &(expect, offset) in &tests {
        println!("offset: {:?}", offset);
        println!("expect: {:?}", expect);
        let a: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let rel = Translate2d::from(offset);

        let (collides, _distance) = a.sdfv_common_rel(&b, &rel);
        println!("collides: {:?}", collides);
        assert_eq!(expect, collides);
    }
}
