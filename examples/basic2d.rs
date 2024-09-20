use auburn::col2d::*;

fn main() {
    let a_shape = Ball::with_radius(1.0);
    let a_pos = Vec2::new(0.0, 0.0);

    let b = Collider2d {
        shape: &Ball::with_radius(1.0),
        transform: &Vec2::new(1.0, 0.0),
    };

    if (&a_shape, &a_pos).collides((b.shape, b.transform)) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }

    let a = Collider2d {
        shape: &a_shape,
        transform: &a_pos,
    };

    if a.collides(b) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }

    if a.collides((b.shape, b.transform)) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }

    if (&a_shape, &a_pos).collides((b.shape, b.transform)) {
        println!("Collision detected!");
    } else {
        println!("No collision detected.");
    }

    // dbg!(a.extreme_point(Vec2::new(1.0, 0.0)));
}
