use super::*;
use crate::col2d::detection::SdfvMinkowski2d;

#[cfg(disabled)]
macro_rules! warn_simplex_not_converged {
    ($diff:expr) => {
        warn!(
            "Simplex algorithm did not converge in {} iterations",
            $diff.iteration_limit()
        );
    };
}

macro_rules! warn_simplex_not_converged {
    ($limit:ident) => {
        warn!(
            "Simplex algorithm did not converge in {} iterations",
            $limit
        );
    };
}

impl<A, B, T> SdfvMinkowski2d<false, false> for LocalMinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        println!("[ GJK (collision only)]");
        let mut direction = self.initial_direction();
        trace!("initial direction: {}", direction);
        let mut simplex = Simplex2d::new();
        let iteration_limit = 20;

        for _i in 0..iteration_limit {
            let human_counter = _i + 1;
            trace!("-- iteration {_i}");
            let a = self.extreme_point(direction);
            trace!("M{human_counter} point: {}", a);

            trace!("M{human_counter} fitness: {}", a.dot(direction));
            if a.dot(direction) <= 0.0 {
                trace!("Not colliding");
                return (false, Vec2::NAN);
            }

            let (collides, new_direction) = simplex.add_point(a);
            trace!("new direction: {}", new_direction);
            let new_direction_too_small = new_direction.length_squared() < f32::EPSILON;
            trace!("new direction too small: {}", new_direction_too_small);
            if collides || new_direction_too_small {
                trace!("Simplex collides");
                return (true, Vec2::NAN);
            }

            direction = new_direction;
        }

        // warn_simplex_not_converged!(self);
        warn_simplex_not_converged!(iteration_limit);

        (false, Vec2::NAN)
    }
}

impl<A, B, T> SdfvMinkowski2d<false, true> for LocalMinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        println!("[ GJK (distance only)]");
        let mut direction = self.initial_direction();
        let mut simplex = Simplex2d::new();
        let iteration_limit = 20;

        let mut last_a = self.extreme_point(direction);
        direction = simplex.add_point(last_a).1;

        for _ in 0..iteration_limit {
            let a = self.extreme_point(direction);
            let fitness = a.dot(direction);

            if fitness <= 0.0 {
                let last_fitness = last_a.dot(direction);
                if fitness <= last_fitness {
                    return (false, simplex.distance_to_origin());
                }
            }
            last_a = a;

            let (collides, new_direction) = simplex.add_point(a);
            let new_direction_too_small = new_direction.length_squared() < f32::EPSILON;

            let new_direction = if new_direction_too_small {
                direction.perp().normalize()
            } else {
                new_direction
            };

            if collides {
                assert_eq!(simplex.0.points.len(), 3);
                return (true, Vec2::NAN);
            }

            direction = new_direction;
        }

        // warn_simplex_not_converged!(diff);
        warn_simplex_not_converged!(iteration_limit);

        (false, simplex.distance_to_origin())
    }
}

impl<A, B, T> SdfvMinkowski2d<true, false> for LocalMinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        trace!("");
        trace!("[ GJK (penetration only)]");
        let iteration_limit = 20;
        let mut direction = self.initial_direction();
        trace!("initial direction: {}", direction);
        let mut simplex = Simplex2d::new();
        let mut last_a = self.extreme_point(direction);
        trace!("M1 point: {}", last_a);
        direction = simplex.add_point(last_a).1;

        for _i in 0..iteration_limit {
            let human_counter = _i + 2;
            trace!("-- iteration {_i}");
            trace!("direction: {}", direction);
            let a = self.extreme_point(direction);
            trace!("M{human_counter} point: {}", a);

            let fitness = a.dot(direction);
            trace!("M{human_counter} fitness: {}", fitness);

            if fitness <= 0.0 {
                trace!("Collision no longer possible");
                return (false, Vec2::NAN);
            }
            last_a = a;

            let (collides, new_direction) = simplex.add_point(a);
            trace!("new direction: {}", new_direction);
            // very close to zero. This is a sign that the origin lies on the boundary of the
            // incomplete simplex.
            // We mainly have two options:
            //   - Consider this a collision
            let new_direction_too_small = new_direction.length_squared() < f32::EPSILON;
            let new_direction = if new_direction_too_small {
                direction.perp().normalize()
            } else {
                new_direction
            };

            if collides {
                trace!("Simplex collides");
                simplex.enforce_counter_clockwise_winding();
                let mut epa = EpaPoly2d(simplex.0);
                let penetration = epa.calculate_penetration(&self, iteration_limit);
                return (true, penetration);
            }

            trace!("new direction: {}", new_direction);

            direction = new_direction;
        }

        // warn_simplex_not_converged!(diff);
        warn_simplex_not_converged!(iteration_limit);

        (false, Vec2::NAN)
    }
}

impl<A, B, T> SdfvMinkowski2d<true, true> for LocalMinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn sdfv_minkowski(&self) -> (bool, Vec2) {
        trace!("");
        trace!("[ GJK (full) ]");
        let iteration_limit = 20;
        let mut direction = self.initial_direction();
        trace!("initial direction: {}", direction);
        let mut simplex = Simplex2d::new();
        let mut last_a = self.extreme_point(direction);
        trace!("M1 point: {}", last_a);
        direction = simplex.add_point(last_a).1;

        for _i in 0..iteration_limit {
            let human_counter = _i + 2;
            trace!("-- iteration {_i}");
            trace!("direction: {}", direction);
            let a = self.extreme_point(direction);
            trace!("M{human_counter} point: {}", a);

            let fitness = a.dot(direction);
            trace!("M{human_counter} fitness: {}", fitness);

            if fitness <= 0.0 {
                trace!("Collision no longer possible");
                let last_fitness = last_a.dot(direction);
                trace!("last fitness: {}", last_fitness);
                if fitness <= last_fitness {
                    trace!("No intersection");
                    return (false, simplex.distance_to_origin());
                }
            }
            last_a = a;

            let (collides, new_direction) = simplex.add_point(a);
            trace!("new direction: {}", new_direction);
            // very close to zero. This is a sign that the origin lies on the boundary of the
            // incomplete simplex.
            // We mainly have two options:
            //   - Consider this a collision
            let new_direction_too_small = new_direction.length_squared() < f32::EPSILON;
            let new_direction = if new_direction_too_small {
                direction.perp().normalize()
            } else {
                new_direction
            };

            if collides {
                trace!("Simplex collides");
                simplex.enforce_counter_clockwise_winding();
                let mut epa = EpaPoly2d(simplex.0);
                let penetration = epa.calculate_penetration(&self, iteration_limit);
                return (true, penetration);
            }

            trace!("new direction: {}", new_direction);

            direction = new_direction;
        }

        // warn_simplex_not_converged!(diff);
        warn_simplex_not_converged!(iteration_limit);

        (false, simplex.distance_to_origin())
    }
}
