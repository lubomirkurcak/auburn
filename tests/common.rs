use std::sync::LazyLock;

use auburn::{col2d::*, trace};

#[cfg(obsolete)]
pub static LOGGER: LazyLock<()> = LazyLock::new(|| {
    env_logger::builder()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .is_test(true)
        .try_init()
        .unwrap();
});

pub struct F32Iterator {
    min: f32,
    max: f32,
    steps: usize,
    current: usize,
}

impl F32Iterator {
    pub fn new(min: f32, max: f32, steps: usize) -> Self {
        Self {
            min,
            max,
            steps,
            current: 0,
        }
    }

    pub fn t(&self, t: f32) -> f32 {
        self.min + t * (self.max - self.min)
    }
}

impl Iterator for F32Iterator {
    type Item = f32;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current < self.steps {
            let t = self.current as f32 / self.steps as f32;
            self.current += 1;
            Some(self.t(t))
        } else {
            None
        }
    }
}

pub fn box_box_transform2d_distance_bounds_check<'a>(
    a: Collider2d<'a, Box2d, Transform2d>,
    b: Collider2d<'a, Box2d, Transform2d>,
)
// fn box_box_necessary_distance<'a, T>(a: T, b: T, sdfv: (bool, Vec2))
// where
//     T = Collider2d<'a, Box2d, Transform2d>,
{
    let (collides, sdfv) = a.sdfv(b);
    let distance = if collides { -1.0 } else { 1.0 } * sdfv.length();

    trace!("sdfv length: {distance}");

    let distance_between_centers =
        (a.transform.apply_origin() - b.transform.apply_origin()).length();

    let d = a.transform.scaling_factor();

    let a_min_radius = d * a.shape.halfsize.x.min(a.shape.halfsize.y);
    let a_max_radius = d * a.shape.halfsize.length();

    let b_min_radius = d * b.shape.halfsize.x.min(b.shape.halfsize.y);
    let b_max_radius = d * b.shape.halfsize.length();

    let min_radius = a_min_radius + b_min_radius;
    let max_radius = a_max_radius + b_max_radius;

    let min_distance = distance_between_centers - max_radius;
    let max_distance = distance_between_centers - min_radius;
    trace!("possible distance interval: [{min_distance}, {max_distance}]");
    assert!(distance >= min_distance);
    assert!(distance <= max_distance);
    // let sdf = a.sdf(b);
    // trace!("sdf: {sdf}");
    // assert_approx_eq!(distance, sdf);
}
