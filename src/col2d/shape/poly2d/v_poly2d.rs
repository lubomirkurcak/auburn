use super::*;

impl SdfvCommonRel2d<false, false, Poly2d> for Poly2d {
    fn sdfv_common_rel(&self, other: &Poly2d, rel: &impl Transformation2d) -> (bool, Option<Vec2>) {
        let diff = MinkowskiDiff2d::new(self, other, rel);
        let mut simplex = Simplex2d::new();
        let mut direction = diff.initial_direction();
        println!("");
        println!("[ GJK ]");
        println!("initial direction: {}", direction);

        let max_iter = self.points.len() + other.points.len();
        for _i in 0..max_iter {
            println!("human counter {}", _i + 1);
            let a = diff.extreme_point(direction);
            println!("M point: {}", a);

            let dot = a.dot(direction);
            println!("dot: {}", dot);
            if dot <= 0.0 {
                println!("No intersection");
                return (false, None);
            }

            let (collides, new_direction) = simplex.add_point(a);
            // NOTE: When the origin lies on the incomplete simplex, the calculated direction is
            // very close to zero. This is a sign that the origin lies on the boundary of the
            // incomplete simplex.
            // We mainly have two options:
            //   - Consider this a collision
            let new_direction_too_small = new_direction.length_squared() < f32::EPSILON;
            if collides || new_direction_too_small {
                println!("Simplex collides");
                return (true, None);
            }
            println!("new direction: {}", new_direction);

            direction = new_direction;
        }

        panic!(
            "Simplex algorithm did not converge in {} iterations",
            max_iter
        );
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
}

impl<T: Transformation2d> ExtremePoint2d for MinkowskiDiff2d<'_, T> {
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        println!("A point: {}", self.a.extreme_point(direction));
        println!(
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
            points: Vec::with_capacity(4),
        })
    }

    fn add_point(&mut self, point: Vec2) -> (bool, Vec2) {
        self.0.points.push(point);

        match self.0.points.len() {
            1 => (false, -point),
            2 => {
                let a = self.0.points[0];
                let b = self.0.points[1];
                let ab = b - a;
                let ao = -a;
                if ab.dot(ao) > 0.0 {
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
                let ab_perp = ab.cross_aba(ao);
                if ab_perp.dot(ac) > 0.0 {
                    let ac_perp = ac.cross_aba(ao);
                    if ac_perp.dot(ab) > 0.0 {
                        (true, Vec2::ZERO)
                    } else {
                        (false, ac_perp)
                    }
                } else {
                    if ac.cross_aba(ab).dot(ao) > 0.0 {
                        (false, ab_perp)
                    } else {
                        (false, ao)
                    }
                }
            }
            _ => unreachable!(),
        }
    }
}

trait CrossABA2d {
    fn cross_aba(self, other: Self) -> Self;
}

impl CrossABA2d for Vec2 {
    fn cross_aba(self, other: Self) -> Self {
        let cross_ab = self.x * other.y - self.y * other.x;
        Vec2::new(-self.y * cross_ab, self.x * cross_ab)
    }
}
