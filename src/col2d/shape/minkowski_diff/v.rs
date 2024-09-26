use super::*;

impl SdfvCommonRel2d<false, false, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = MinkowskiDiff2d::new(self, other, rel);
        let mut direction = diff.initial_direction();
        let mut simplex = Simplex2d::new();
        let iteration_limit = self.points.len() + other.points.len();

        for _i in 0..iteration_limit {
            let a = diff.extreme_point(direction);

            if a.dot(direction) <= 0.0 {
                return (false, Vec2::NAN);
            }

            let (collides, new_direction) = simplex.add_point(a);
            let new_direction_too_small = new_direction.length_squared() < f32::EPSILON;
            if collides || new_direction_too_small {
                return (true, Vec2::NAN);
            }

            direction = new_direction;
        }

        warn_simplex_not_converged!(diff);

        (false, Vec2::NAN)
    }
}

impl SdfvCommonRel2d<false, true, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = MinkowskiDiff2d::new(self, other, rel);
        let mut direction = diff.initial_direction();
        let mut simplex = Simplex2d::new();
        let iteration_limit = self.points.len() + other.points.len();

        let mut last_a = diff.extreme_point(direction);
        direction = simplex.add_point(last_a).1;

        for _ in 0..iteration_limit {
            let a = diff.extreme_point(direction);
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

impl SdfvCommonRel2d<true, true, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        trace!("");
        trace!("[ GJK ]");
        let diff = MinkowskiDiff2d::new(self, other, rel);
        let iteration_limit = self.points.len() + other.points.len();
        let mut direction = diff.initial_direction();
        trace!("initial direction: {}", direction);
        let mut simplex = Simplex2d::new();
        let mut last_a = diff.extreme_point(direction);
        trace!("M1 point: {}", last_a);
        direction = simplex.add_point(last_a).1;

        for _i in 0..iteration_limit {
            let human_counter = _i + 2;
            trace!("-- iteration {_i}");
            trace!("direction: {}", direction);
            let a = diff.extreme_point(direction);
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
                let penetration = epa.calculate_penetration(&diff, iteration_limit);
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
