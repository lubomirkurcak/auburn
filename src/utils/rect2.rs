use lk_math::vector::{V2i32, Vector};

#[derive(Debug)]
pub struct Rect2i32 {
    pub min: V2i32,
    pub max: V2i32,
}

impl Rect2i32 {
    pub fn min_max(min: V2i32, max: V2i32) -> Self {
        Self { min, max }
    }

    pub fn pad(self, padding: V2i32) -> Self {
        Self {
            min: self.min - padding,
            max: self.max - padding,
        }
    }

    pub fn iterate_with_direction_f32(
        &self,
        dir_x: f32,
        dir_y: f32,
    ) -> Box<dyn Iterator<Item = V2i32>> {
        self.iterate_boxed(dir_x < 0.0, dir_y < 0.0)
    }

    pub fn iterate(&self) -> impl Iterator<Item = V2i32> {
        self.iterate_north_east()
    }

    pub fn iterate_north_east(&self) -> impl Iterator<Item = V2i32> {
        let x_range = self.min.x()..=self.max.x();
        let y_range = self.min.y()..=self.max.y();
        y_range.flat_map(move |y| x_range.clone().map(move |x| Vector::from_xy(x, y)))
    }

    pub fn iterate_south_east(&self) -> impl Iterator<Item = V2i32> {
        let x_range = self.min.x()..=self.max.x();
        let y_range = (self.min.y()..=self.max.y()).rev();
        y_range.flat_map(move |y| x_range.clone().map(move |x| Vector::from_xy(x, y)))
    }

    pub fn iterate_south_west(&self) -> impl Iterator<Item = V2i32> {
        let x_range = (self.min.x()..=self.max.x()).rev();
        let y_range = (self.min.y()..=self.max.y()).rev();
        y_range.flat_map(move |y| x_range.clone().map(move |x| Vector::from_xy(x, y)))
    }

    pub fn iterate_north_west(&self) -> impl Iterator<Item = V2i32> {
        let x_range = (self.min.x()..=self.max.x()).rev();
        let y_range = self.min.y()..=self.max.y();
        y_range.flat_map(move |y| x_range.clone().map(move |x| Vector::from_xy(x, y)))
    }

    pub fn iterate_boxed(&self, flip_x: bool, flip_y: bool) -> Box<dyn Iterator<Item = V2i32>> {
        if flip_y {
            if flip_x {
                Box::new(self.iterate_south_west())
            } else {
                Box::new(self.iterate_south_east())
            }
        } else {
            if flip_x {
                Box::new(self.iterate_north_west())
            } else {
                Box::new(self.iterate_north_east())
            }
        }
    }
}
