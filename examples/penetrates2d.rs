use auburn::col2d::*;

fn main() {
    let a = Ball::with_radius(1.0);
    // let a_pos = Translate2d::new(Vec2::new(0.0, 0.0));
    let mut a_pos = Vec2::new(0.0, 0.0);
    dbg!(&(a, a_pos));

    let b = Ball::with_radius(1.0);
    // let b_pos = Translate2d::new(Vec2::new(1.0, 0.0));
    let b_pos = Vec2::new(1.0, 0.0);
    dbg!(&(b, b_pos));

    #[cfg(disabled)]
    // NOTE: Classic syntax
    if let Some(penetration) = a.penetrates(&a_pos, &b, &b_pos) {
        a_pos += penetration;
    } else {
        println!("No collision detected.");
    }

    #[cfg(disabled)]
    // TODO: Support this kind of syntax? (Variant 2)
    if let Some(penetration) = (&a, &a_pos).penetrates_v2(&b, &b_pos) {
        a_pos += penetration;
    } else {
        println!("No collision detected.");
    }

    // NOTE: For collision resolution this may not be as clean
    // TODO: Support this kind of syntax? (Variant 1)
    // if (&a, &a_pos).collides_v3((&b, &b_pos)) {
    let a = (&a, &a_pos);
    let b = (&b, &b_pos);

    if let Some(penetration) = a.penetrates_v3(b) {
        a_pos += penetration;
    } else {
        println!("No collision detected.");
    }

    #[cfg(disabled)]
    if let Some(penetration) = a.penetrates_v3(b) {
        a_pos += penetration;
    } else {
        println!("No collision detected.");
    }
}
