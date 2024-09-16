use auburn::col3d::*;

fn main() {
    let a = Ball::with_radius(1.0);
    let a_pos = Vec3::new(0.0, 0.0, 0.0);
    dbg!(&(a, a_pos));

    let b = Ball::with_radius(1.0);
    let b_pos = Vec3::new(1.0, 0.0, 0.0);
    dbg!(&(b, b_pos));

    // NOTE: Classic syntax
    if a.collides(&a_pos, &b, &b_pos) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }

    // TODO: Support this kind of syntax? (Variant 2)
    if (&a, &a_pos).collides_v2(&b, &b_pos) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }

    // NOTE: For collision resolution this may not be as clean
    // TODO: Support this kind of syntax? (Variant 1)
    // if (&a, &a_pos).collides_v3((&b, &b_pos)) {
    let a = (&a, &a_pos);
    let b = (&b, &b_pos);

    if a.collides_v3(b) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }
}
