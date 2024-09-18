use auburn::col2d::*;

fn main() {
    let a_shape = Ball::with_radius(1.0);
    let a_pos = Vec2::new(0.0, 0.0);

    let b = Collider2d {
        shape: &Ball::with_radius(1.0),
        transform: &Vec2::new(1.0, 0.0),
    };

    println!(
        "Distance: {}",
        (&a_shape, &a_pos).sdf((b.shape, b.transform))
    );

    let a = Collider2d {
        shape: &a_shape,
        transform: &a_pos,
    };

    println!("Distance: {}", a.sdf(b));
    println!("Distance: {}", a.sdf((b.shape, b.transform)));

    println!(
        "Distance: {}",
        (&a_shape, &a_pos).sdf((b.shape, b.transform))
    );
}
