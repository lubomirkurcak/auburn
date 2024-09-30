mod v;

use crate::utils::approx::Approx;
use crate::{debug, error, info, trace, warn};

use super::*;

pub struct LocalMinkowskiDiff2d<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    a: &'a A,
    b: &'a B,
    rel: &'a T,
}

impl<'a, A, B, T> LocalMinkowskiDiff2d<'a, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    pub fn raw(a: &'a A, b: &'a B, rel: &'a T) -> Self {
        Self { a, b, rel }
    }

    pub fn initial_direction(&self) -> Vec2 {
        let direction = self.rel.apply_origin();
        if direction.length_squared() < f32::EPSILON {
            Vec2::new(1.0, 0.0)
        } else {
            direction
        }
    }

    // fn iteration_limit(&self) -> usize {
    //     self.a.points.len() + self.b.points.len()
    // }
}

impl<'a, A, B, T> ExtremePoint2d for LocalMinkowskiDiff2d<'_, A, B, T>
where
    A: ExtremePoint2d,
    B: ExtremePoint2d,
    T: Transformation2d,
{
    fn extreme_point(&self, direction: Vec2) -> Vec2 {
        trace!("direction: {}", direction);
        let a_point = self.a.extreme_point(direction);
        trace!("a_point: {}", a_point);
        let b_direction = self.rel.unapply_normal(-direction);
        trace!("b_direction: {}", b_direction);
        let b_point = self.rel.apply(self.b.extreme_point(b_direction));
        trace!("b_point: {}", b_point);
        let m_point = a_point - b_point;
        trace!("m_point: {}", m_point);
        m_point
    }
}

struct EpaPoly2d(Poly2d);

impl EpaPoly2d {
    fn calculate_penetration<'a, A, B, T>(
        &mut self,
        diff: &LocalMinkowskiDiff2d<'a, A, B, T>,
        iteration_limit: usize,
    ) -> Vec2
    where
        A: ExtremePoint2d,
        B: ExtremePoint2d,
        T: Transformation2d,
    {
        trace!("[ EPA ]");
        trace!(
            "starting with {} points: {:?}",
            self.0.points.len(),
            self.0.points
        );

        for i in 0..iteration_limit {
            trace!("[EPA] iteration {}", i);
            let (edge_index, a, b) = self.find_closest_edge();
            trace!("closest edge: {:?} {:?}", a, b);
            let normal = (b - a).perp_right();
            trace!("its normal (outwards direction): {}", normal);
            let point = diff.extreme_point(normal);
            trace!("Me{i} point: {}", point);
            let fitness = point.dot(normal);
            trace!("fitness: {}", fitness);
            let fitness_to_beat = a.dot(normal);
            trace!("fitness to beat: {}", fitness_to_beat);

            if fitness - fitness_to_beat < f32::EPSILON {
                trace!("EPA converged");
                let denom = normal.dot(normal);
                trace!("denom: {}", denom);
                return normal * (-fitness / denom);
            }

            trace!("splitting edge at index {}", edge_index);
            self.0.points.insert(edge_index + 1, point);
            trace!("points after: {:?}", self.0.points);
        }

        warn!("EPA did not converge in {} iterations", iteration_limit);

        Vec2::ZERO
    }

    fn find_closest_edge(&self) -> (usize, Vec2, Vec2) {
        let mut closest_edge = (Vec2::NAN, Vec2::NAN);
        let mut closest_distance = f32::INFINITY;
        let mut closest_edge_index = 0;

        for i in 0..self.0.points.len() {
            let a = self.0.points[i];
            let b = self.0.points[(i + 1) % self.0.points.len()];
            let ab = b - a;
            let ao = -a;
            let ab_perp = ab.cross_aba(ao);
            let distance = ab_perp.length_squared();

            if distance < closest_distance {
                closest_edge = (a, b);
                closest_distance = distance;
                closest_edge_index = i;
            }
        }

        (closest_edge_index, closest_edge.0, closest_edge.1)
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

    fn is_winding_counter_clockwise(&self) -> bool {
        assert_eq!(self.0.points.len(), 3);
        let a = self.0.points[0];
        let b = self.0.points[1];
        let c = self.0.points[2];
        let ab = b - a;
        let ac = c - a;
        ab.cross(ac) > 0.0
    }

    fn enforce_counter_clockwise_winding(&mut self) {
        assert_eq!(self.0.points.len(), 3);
        if !self.is_winding_counter_clockwise() {
            self.0.points.swap(1, 2);
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
    fn perp_right(self) -> Self;
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

    fn perp_right(self) -> Self {
        Vec2::new(self.y, -self.x)
    }
}

#[cfg(test)]
mod tests {
    use crate::assert_approx_eq;

    use super::*;

    #[test_log::test]
    fn case_1_box() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 0.0;
        let a_shape = Box2d::with_halfdims(1.0, 1.0);
        let b_shape = Box2d::with_halfdims(1.0, 1.0);
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();
        let dir = Vec2::X;
        let rel = a.transform.delta_transform(b.transform);
        let diff = LocalMinkowskiDiff2d::raw(a.shape, b.shape, &rel);
        let local_dir = a.transform.unapply_normal(dir);
        let local_extreme_point = diff.extreme_point(local_dir);
        let extreme_point = a.transform.apply_normal(local_extreme_point);
        assert_eq!(extreme_point.x, -1.0);
    }

    #[test_log::test]
    fn case_1_poly() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 0.0;
        let a_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let b_shape: Poly2d = Box2d::with_halfdims(1.0, 1.0).into();
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();
        let dir = Vec2::X;
        let rel = a.transform.delta_transform(b.transform);
        let diff = LocalMinkowskiDiff2d::raw(a.shape, b.shape, &rel);
        let local_dir = a.transform.unapply_normal(dir);
        let local_extreme_point = diff.extreme_point(local_dir);
        let extreme_point = a.transform.apply_normal(local_extreme_point);
        assert_eq!(extreme_point.x, -1.0);
    }

    #[test_log::test]
    fn ground_truth_check_1() {
        let a_angle: f32 = 0.0;
        let b_angle: f32 = 90.0;
        let a_shape = Box2d::with_halfdims(1.0, 1.0);
        let b_shape = Box2d::with_halfdims(1.0, 1.0);
        let a_transform = Transform2d {
            pos: Vec2::ZERO,
            rot: Rotor2d::from_angle(a_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let b_transform = Transform2d {
            pos: Vec2::new(3.0, 0.0),
            rot: Rotor2d::from_angle(b_angle.to_radians()),
            scale: Vec2::ONE,
        };
        let a: Collider2d<_, _> = (&a_shape, &a_transform).into();
        let b: Collider2d<_, _> = (&b_shape, &b_transform).into();

        let dir = Vec2::X;
        let m_gt;
        {
            let a_gt = a.extreme_point(dir);
            debug!("a_gt: {}", a_gt);
            let b_gt = b.extreme_point(-dir);
            debug!("b_gt: {}", b_gt);
            m_gt = a_gt - b_gt;
            info!("m_gt: {}", m_gt);
            let a_gt_a_space = a.transform.unapply(a_gt);
            debug!("a_gt_a_space: {}", a_gt_a_space);
            let b_gt_a_space = a.transform.unapply(b_gt);
            debug!("b_gt_a_space: {}", b_gt_a_space);
            let m_gt_a_space = a.transform.unapply(m_gt);
            debug!("m_gt_a_space: {}", m_gt_a_space);
        }
        let extreme_point;
        {
            let local_dir = a.transform.unapply_normal(dir);
            debug!("local_dir: {}", local_dir);
            let rel = a.transform.delta_transform(b.transform);
            debug!("rel: {:#?}", rel);
            let diff = LocalMinkowskiDiff2d::raw(a.shape, b.shape, &rel);
            let local_extreme_point = diff.extreme_point(local_dir);
            extreme_point = a.transform.apply(local_extreme_point);
        }

        assert_approx_eq!(m_gt, extreme_point);
    }
}
