use crate::{debug, error, info, trace, warn};

use super::*;

macro_rules! warn_simplex_not_converged {
    ($diff:expr) => {
        warn!(
            "Simplex algorithm did not converge in {} iterations",
            $diff.iteration_limit()
        );
    };
}

impl SdfvCommonRel2d<false, false, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = MinkowskiDiff2d::new(self, other, rel);
        let mut direction = diff.initial_direction();
        let mut simplex = Simplex2d::new();

        for _i in 0..diff.iteration_limit() {
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

        let mut last_a = diff.extreme_point(direction);
        direction = simplex.add_point(last_a).1;

        for _ in 0..diff.iteration_limit() {
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
                panic!("just wanna see if this ever happens");
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

        warn_simplex_not_converged!(diff);

        (false, simplex.distance_to_origin())
    }
}

impl SdfvCommonRel2d<true, true, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        trace!("");
        trace!("[ GJK ]");
        let diff = MinkowskiDiff2d::new(self, other, rel);
        let mut direction = diff.initial_direction();
        trace!("initial direction: {}", direction);
        let mut simplex = Simplex2d::new();
        let mut last_a = diff.extreme_point(direction);
        trace!("M1 point: {}", last_a);
        direction = simplex.add_point(last_a).1;

        for _i in 0..diff.iteration_limit() {
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
                assert_eq!(simplex.0.points.len(), 3);
                todo!();
            }

            trace!("new direction: {}", new_direction);

            direction = new_direction;
        }

        warn_simplex_not_converged!(diff);

        (false, simplex.distance_to_origin())
    }
}

struct MinkowskiDiff2d<'a, T: Transformation2d> {
    a: &'a Poly2d,
    b: &'a Poly2d,
    rel: &'a T,
}

impl<'a, T: Transformation2d> MinkowskiDiff2d<'a, T> {
    fn new(a: &'a Poly2d, b: &'a Poly2d, rel: &'a T) -> Self {
        Self { a, b, rel }
    }

    fn initial_direction(&self) -> Vec2 {
        let direction = self.rel.apply_origin();
        if direction.length_squared() < f32::EPSILON {
            Vec2::new(1.0, 0.0)
        } else {
            direction
        }
    }

    fn iteration_limit(&self) -> usize {
        self.a.points.len() + self.b.points.len()
    }
}

impl<T: Transformation2d> ExtremePoint2d for MinkowskiDiff2d<'_, T> {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        trace!("A point: {}", self.a.extreme_point(direction));
        trace!(
            "B point: {}",
            self.rel.apply(self.b.extreme_point(-direction))
        );
        self.a.extreme_point(direction) - self.rel.apply(self.b.extreme_point(-direction))
    }
}

struct Simplex2d(Poly2d);

impl Simplex2d {
    fn new() -> Self {
        Self(Poly2d {
            points: Vec::with_capacity(3),
        })
    }

    fn add_point(&mut self, point: Vec2) -> (bool, Vec2) {
        self.0.points.insert(0, point);
        trace!(
            "{} simplex points: {:?}",
            self.0.points.len(),
            self.0.points
        );
        match self.0.points.len() {
            1 => (false, -point),
            2 => {
                let a = self.0.points[0];
                let b = self.0.points[1];
                let ab = b - a;
                let ao = -a;
                if ab.dot(ao) >= 0.0 {
                    let ab_perp = ab.cross_aba(ao);
                    (false, ab_perp)
                } else {
                    (false, ao)
                }
            }
            3 => {
                let a = self.0.points[0];
                let b = self.0.points[1];
                let c = self.0.points[2];
                let ab = b - a;
                let ac = c - a;
                let ao = -a;

                trace!("a: {}", a);
                trace!("b: {}", b);
                trace!("c: {}", c);
                trace!("ab: {}", ab);
                trace!("ac: {}", ac);
                trace!("ao: {}", ao);

                let abc = ab.cross(ac);
                let ac_perp = abc.cross(ac);
                let ab_perp = ab.cross_f32(abc);

                trace!("abc: {}", abc);
                trace!("ab_perp: {}", ab_perp);
                trace!("ac_perp: {}", ac_perp);

                trace!("points before: {:?}", self.0.points);

                if same_direction(ab_perp, ao) {
                    trace!("same direction ab_perp ao");
                    self.0.points = vec![a, b];
                    trace!("points after: {:?}", self.0.points);
                    (false, ab_perp)
                } else if same_direction(ac_perp, ao) {
                    trace!("same direction ac_perp ao");
                    self.0.points = vec![a, c];
                    trace!("points after: {:?}", self.0.points);
                    (false, ac_perp)
                } else {
                    trace!("captured the origin");
                    (true, Vec2::NAN)
                }
            }
            _ => unreachable!(),
        }
    }

    fn distance_to_origin(&self) -> Vec2 {
        trace!(
            "distance_to_origin: {} simplex points: {:?}",
            self.0.points.len(),
            self.0.points
        );
        match self.0.points.len() {
            1 => -self.0.points[0],
            2 => {
                trace!("2 points case");
                let a = self.0.points[0];
                let b = self.0.points[1];
                let ab = b - a;
                let ao = -a;
                let t = ao.dot(ab) / ab.dot(ab);

                trace!("a: {}", a);
                trace!("b: {}", b);
                trace!("ab: {}", ab);
                trace!("ao: {}", ao);
                trace!("t: {}", t);

                if t <= 0.0 {
                    trace!("t <= 0.0");
                    trace!("ao: {}", ao);
                    ao
                } else if t >= 1.0 {
                    trace!("t >= 1.0");
                    trace!("b: {}", b);
                    -b
                } else {
                    trace!("0.0 < t < 1.0");
                    trace!("ab * t: {}", ab * t);
                    ao - ab * t
                }
            }
            3 => {
                unreachable!("3 points case - is it reachable? maybe in edge cases but maybe not in most common cases");
            }
            _ => unreachable!(),
        }
    }
}

fn same_direction(a: Vec2, b: Vec2) -> bool {
    a.dot(b) >= 0.0
}

trait Crossf32 {
    fn cross(self, other: Vec2) -> Vec2;
}

impl Crossf32 for f32 {
    fn cross(self, other: Vec2) -> Vec2 {
        Vec2::new(-self * other.y, self * other.x)
    }
}

trait Cross2d {
    fn cross(self, other: Self) -> f32;
    fn cross_f32(self, other: f32) -> Self;
    fn cross_aba(self, other: Self) -> Self;
}

impl Cross2d for Vec2 {
    fn cross(self, other: Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    fn cross_f32(self, other: f32) -> Self {
        Vec2::new(self.y * other, -self.x * other)
    }

    fn cross_aba(self, other: Self) -> Self {
        let cross = self.cross(other);
        Vec2::new(-self.y * cross, self.x * cross)
    }
}
