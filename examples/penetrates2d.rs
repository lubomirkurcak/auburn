use auburn::col2d::*;

fn main() {
    let a_shape = Ball::with_radius(1.0);
    let a_pos = Vec2::new(0.0, 0.0);

    let b = Collider2d {
        shape: &Ball::with_radius(1.0),
        transform: &Vec2::new(1.0, 0.0),
    };

    if let Some(p) = (&a_shape, &a_pos).penetrates((b.shape, b.transform)) {
        println!("Collision detected, penetration vector: {:?}", p);
    } else {
        println!("No collision detected.");
    }

    let a = Collider2d {
        shape: &a_shape,
        transform: &a_pos,
    };

    if let Some(p) = a.penetrates(b) {
        println!("Collision detected, penetration vector: {:?}", p);
    } else {
        println!("No collision detected.");
    }

    if let Some(p) = a.penetrates((b.shape, b.transform)) {
        println!("Collision detected, penetration vector: {:?}", p);
    } else {
        println!("No collision detected.");
    }

    if let Some(p) = (&a_shape, &a_pos).penetrates((b.shape, b.transform)) {
        println!("Collision detected, penetration vector: {:?}", p);
    } else {
        println!("No collision detected.");
    }
}
