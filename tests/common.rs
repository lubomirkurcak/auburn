use std::sync::LazyLock;

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
