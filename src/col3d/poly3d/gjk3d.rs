use super::Poly3d;
use crate::{col3d::ExtremePoint3d, Vec3};

/* ========================================================================
$File: $
$Date: $
$Revision: $
$Creator: Lubomir Kurcak $
$Notice: (C) Copyright 2019 All Rights Reserved. $
======================================================================== */

/*
  NOTE(lubo): Sources and inspiration

    It appears that we are using left-handed triangles. (#windings)

    Casey Muratori, Implementing GJK - 2006
    https://www.youtube.com/watch?v=Qupqu1xe7Io

    dyn4j, GJK - Distance & Closest Points
    http://www.dyn4j.org/2010/04/gjk-distance-closest-points/
*/

/*
  TODO(lubo):
    Parameter passing is kinda crazy
    We should refactor but this is highly performance oriented code, so only if it helps performance
*/

// TODO(lubo): Reorder ABCD normally instead of crazy casey style XD

// #define SAME_DIRECTION(V) V.dot(AO)>0
// #define CROSS_ABA(A,B) A.cross(B).cross(A);

fn same_direction(v: Vec3, ao: Vec3) -> bool {
    v.dot(ao) > 0.0
}

fn cross_aba(a: Vec3, b: Vec3) -> Vec3 {
    a.cross(b).cross(a)
}

fn is_zero(v: Vec3) -> bool {
    v.x == 0.0 && v.y == 0.0 && v.z == 0.0
}

/// Function assumes GJK started with proper initial direction, meaning that the new point `a` has to overshoot the origin along the direction `dir`, i.e. `a` cannot be strictly better than `b` when the direction generally faces the origin.
///
/// If `a` was strictly better than `b`, the alrogithm should have terminated in the previous step.
fn do_simplex_2(points: &mut [Vec3], _point_count: &mut usize) -> Option<Vec3> {
    let a = points[1];
    let b = points[0];
    let ao = -a;
    let ab = b - a;
    let dir = cross_aba(ab, ao);

    // let contains_origin = is_zero(dir);
    // return (contains_origin, dir);
    return Some(dir);
}

fn do_simplex_3(points: &mut [Vec3], point_count: &mut usize) -> Option<Vec3> {
    let a = points[2];
    let b = points[1];
    let c = points[0];
    let ao = -a;
    let ab = b - a;
    let ac = c - a;
    let abc = ab.cross(ac);

    let dir;

    if same_direction(abc.cross(ac), ao) {
        if same_direction(ac, ao) {
            //[A,C] ACxAOxAC
            dir = cross_aba(ac, ao);
            points[0] = c;
            points[1] = a;
            *point_count = 2;
        } else {
            if same_direction(ab, ao) {
                // [A,B] ABxAOxAB
                dir = cross_aba(ab, ao);
                points[0] = b;
                points[1] = a;
                *point_count = 2;
            } else {
                // [A] AO
                dir = ao;
                points[0] = a;
                *point_count = 1;
                // @todo(lubo): Added on 2024-01-28. Remove this branch if it's never hit.
                unreachable!("IS THIS EVEN REACHABLE?");
            }
        }
    } else {
        if same_direction(ab.cross(abc), ao) {
            if same_direction(ab, ao) {
                // [A,B] ABxAOxAB
                dir = cross_aba(ab, ao);
                points[0] = b;
                points[1] = a;
                *point_count = 2;
            } else {
                // [A] AO
                dir = ao;
                points[0] = a;
                *point_count = 1;
                // @todo(lubo): Added on 2024-01-28. Remove this branch if it's never hit.
                unreachable!("IS THIS EVEN REACHABLE?");
            }
        } else {
            if same_direction(abc, ao) {
                //[A,B,C] ABC
                dir = abc;
            } else {
                //[A,C,B] -ABC
                dir = -abc;
                points.swap(0, 1)
            }
        }
    }

    // let contains_origin = is_zero(dir);
    // return contains_origin;
    return Some(dir);
}

fn do_simplex_4(points: &mut [Vec3], point_count: &mut usize) -> Option<Vec3> {
    let a = points[3];
    let b = points[2];
    let c = points[1];
    let d = points[0];
    let ao = -a;
    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let abc = ab.cross(ac);
    let acd = ac.cross(ad);
    let adb = ad.cross(ab);

    let dir;

    // NOTE(lubo): Not really get what this comment is about. ABC ACD at the same time IS defined...
    // TODO(lubo): This is not complete - it can be ABC and ACD at the same time, for example.
    let mut contains_origin = false;
    if same_direction(abc, ao) {
        if same_direction(acd, ao) {
            if same_direction(adb, ao) {
                // TODO(lubo): Weird to think, how this could be possible.
                // Maybe it's not?
                // Also maybe this is not the only one.
                // [A] AO
                dir = ao;
                points[0] = a;
                *point_count = 1;
            } else {
                // [A,C] ACxAOxAC ?
                dir = cross_aba(ac, ao);
                points[0] = c;
                points[1] = a;
                *point_count = 2;
            }
        } else if same_direction(adb, ao) {
            // [A,B], ABxAOxAB
            dir = cross_aba(ab, ao);
            points[0] = b;
            points[1] = a;
            *point_count = 2;
        } else {
            // [A,B,C], ABC
            points[0] = c;
            points[1] = b;
            points[2] = a;
            *point_count = 3;
            dir = abc;
        }
    } else if same_direction(acd, ao) {
        if same_direction(adb, ao) {
            // [A,D] ADxAOxAD ?
            dir = cross_aba(ad, ao);
            points[0] = d;
            points[1] = a;
            *point_count = 2;
        } else {
            // [A,C,D], ACD
            points[0] = d;
            points[1] = c;
            points[2] = a;
            *point_count = 3;
            dir = acd;
        }
    } else if same_direction(adb, ao) {
        // [A,D,B], ADB
        points[1] = d;
        points[0] = b;
        points[2] = a;
        *point_count = 3;
        dir = adb;
    } else {
        // contains_origin = true;
        return None;
    }

    if !contains_origin {
        contains_origin = is_zero(dir);
        if contains_origin {
            return None;
        }
    }

    Some(dir)
}

/// Selects the closest feature of the simplex, checks whether it contains the origin, and calculates new direction to the origin if it does not.
///
/// Updates [points] and [point_count] accordingly. (Arguments are I/O)
///
/// # Returns
///
/// Returns `None` if the (possbily partial) simplex contains the origin. (it's possible that, for example just a line contained the origin, thus collision is detected early)
///
/// Returns the new direction to origin otherwise.
///
/// # Safety
///
/// This function assumes previous points were computed in a certain order and exploits that. Do
/// not use if you don't know what you are doing.
pub fn do_simplex(points: &mut [Vec3], point_count: &mut usize) -> Option<Vec3> {
    match *point_count {
        2 => do_simplex_2(points, point_count),
        3 => do_simplex_3(points, point_count),
        4 => do_simplex_4(points, point_count),
        _ => unreachable!(),
    }
}

/*
// NOTE(lubo): Distance to closest feature.
inline v3 distance_to_closest_feature_2(v3 b, v3 a)
{
    v3 result = {};

    v3 ab = b-a;
    v3 ao = -a;

    if(ab.dot(ao) > 0)
    {
        v3 origin = {0,0,0};
        v3 distance = distance_to_line(a, b, origin);
        result = distance;
    }
    else
    {
        result = a;
    }

    return result;
}

inline v3 distance_to_closest_feature_3(v3 c, v3 b, v3 a, v3 dir)
{
    v3 result = {};

    v3 ab = b-a;
    v3 ac = c-a;
    v3 ao = -a;

    if(ac.dot(ao) > 0)
    {
        if(ab.dot(ao) > 0)
        {
            v3 origin = {0,0,0};
            result = distance_to_plane(origin, dir, a);
        }
        else
        {
            v3 origin = {0,0,0};
            v3 distance = distance_to_line(a, c, origin);
            result = distance;
        }
    }
    else
    {
        if(ab.dot(ao) > 0)
        {
            v3 origin = {0,0,0};
            v3 distance = distance_to_line(a, b, origin);
            result = distance;
        }
        else
        {
            result = a;
        }
    }

    return result;
}

inline v3 distance_to_closest_feature(v3 *points, int point_count, v3 dir)
{
    v3 result = {};

    switch(point_count)
    {
        case 1:
        {
            // NOTE(lubo): Given dir = AO = Origin - A = -A => distance = --A = A
            result = -dir;
        } break;
        case 2:
        {
            // STUDY(lubo): At one point this seemed it
            // could work, then it look like this was
            // double the actual distance. Maybe worth
            // investigating.
            //result = dir;

#if 0
            v3 origin = {0,0,0};
            v3 distance = distance_to_line(points[0], points[1], origin);
            // v3 difference = distance - dir;
            result = distance;
#else
            result = distance_to_closest_feature_2(points[0], points[1]);
#endif
        } break;
        case 3:
        {
#if 0
            v3 origin = {0,0,0};
            result = distance_to_plane(origin, dir, points[0]);
#else
            result = distance_to_closest_feature_3(points[0], points[1], points[2], dir);
#endif
        } break;
        InvalidDefaultCase;
    }

    return result;
}

// NOTE(lubo): Distance to closest feature.
inline v3 distance_to_closest_feature_unordered_2(v3 b, v3 a)
{
    v3 result = {};

    v3 o = {};
    float t = inv_lerp(b, a, o);

    if(t < 0)
    {
        result = b;
    }
    else if(t > 1)
    {
        result = a;
    }
    else
    {
        result = distance_to_line(b, a, o);
    }

    return result;
}

inline v3 distance_to_closest_feature_unordered_3(v3 c, v3 b, v3 a)
{
    v3 o = {};
    float ab = inv_lerp(a, b, o);
    float ac = inv_lerp(a, c, o);
    float bc = inv_lerp(b, c, o);


};

// NOTE(lubo): Bro this problem is kinda hard XD
// inline v3 distance_to_closest_feature_unordered_3(v3 c, v3 b, v3 a)
// {
//     v3 result = {};

//     // NOTE(lubo): This doesn't work right? Cause AB and AC aren't orthogonal.
//     // v3 o = {};
//     // float tb = inv_lerp(b, a, o);
//     // float tc = inv_lerp(c, a, o);

//     v2 project_to_plane_reduce(a, b, c, {});

//     if(v2.x < 0)
//     {
//         if(v2.y < 0)
//         {
//             // A
//         }
//         else
//         {
//             if(v2.x + v2.y > 1)
//             {
//                 // B
//             }
//             else
//             {
//                 // AB
//             }
//         }
//     }
//     else if(t > 1)
//     {
//         result = a;
//     }
//     else
//     {
//         result = distance_to_line(b, a, origin);
//     }

//     return result;
// }

inline v3 distance_to_closest_feature_unordered(v3 *points, int point_count, v3 dir)
{
    v3 result = {};

    switch(point_count)
    {
        case 1:
            result = -dir;
            break;
        case 2:
            result = distance_to_closest_feature_unordered_2(points[0], points[1]);
            break;
        case 3:
            // TODO(lubo): Finish unordered 3-simplex case
            // NOTE(lubo): I'm not aware of us ever hitting this case.
            // NOTE(lubo): Will implement this is we encounter problems.
            // NOTE(lubo): Also we could maybe check if we can ensure
            // correct order as to not have unordered version problems.
            //result = distance_to_closest_feature_unordered_3(points[0], points[1], points[2], dir);
            //result = distance_to_closest_feature_3(points[0], points[1], points[2], dir);

            // NOTE(lubo): Why does this work so well? Should we even solve the complex problem?
            result = distance_to_plane({}, dir, points[0]);

            break;
        InvalidDefaultCase;
    }

    return result;
}

// NOTE(lubo): Get point in minkowski difference furthest in given direction
inline v3 support_function(float scale_a, float scale_b,
                           quaternion orientation_a, quaternion orientation_b,
                           Polyhedron poly_a, Polyhedron poly_b,
                           float radius_a, float radius_b, v3 delta, v3 dir)
{
    // v3 support_a = scale_a*inverse_normalized(orientation_a)*gjk_polytope_support(poly_a, orientation_a*(dir));
    // v3 support_b = scale_b*inverse_normalized(orientation_b)*gjk_polytope_support(poly_b, orientation_b*(-dir));

    // TODO(lubo): Maybe in cases like this always work with
    // normalized values?
    if(radius_a || radius_b) dir = noz(dir);

    v3 dir_a = orientation_a.conjugate() * dir;
    v3 dir_b = orientation_b.conjugate() * (-dir);

    v3 support_a = gjk_polyhedron_support(poly_a, dir_a);
    v3 support_b = gjk_polyhedron_support(poly_b, dir_b);

    if(radius_a) support_a += radius_a*dir_a;
    if(radius_b) support_b += radius_b*dir_b;

    support_a = orientation_a * support_a;
    support_b = orientation_b * support_b;

    support_a *= scale_a;
    support_b *= scale_b;

    v3 result = delta + support_a - support_b;
    return result;
}
*/

fn epa_unique_edge_filter(
    edges: &mut [usize],
    mut count: usize,
    max_count: usize,
    a: usize,
    b: usize,
) -> usize {
    let mut pair_already_exists = false;

    for i in 0..count {
        let _a = edges[2 * i];
        let _b = edges[2 * i + 1];

        if _a == b && _b == a {
            edges[2 * i] = edges[2 * (count - 1)];
            edges[2 * i + 1] = edges[2 * (count - 1) + 1];
            count -= 1;

            pair_already_exists = true;
            break;
        }
    }

    if !pair_already_exists {
        assert!(count < max_count);

        edges[2 * count] = a;
        edges[2 * count + 1] = b;

        count += 1;
    }

    return count;
}

fn plane_normal_from_points(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    (b - a).cross(c - a)
}

fn units_along_basis(a: Vec3, b: Vec3) -> f32 {
    a.dot(b) / b.dot(b)
}

fn project_to_plane_at_origin(a: Vec3, plane: Vec3) -> Vec3 {
    a - plane * units_along_basis(a, plane)
}

fn project_to_plane(a: Vec3, plane: Vec3, plane_origin: Vec3) -> Vec3 {
    project_to_plane_at_origin(a - plane_origin, plane) + plane_origin
}

fn distance_to_plane(a: Vec3, plane: Vec3, plane_origin: Vec3) -> Vec3 {
    project_to_plane(a, plane, plane_origin) - a
}

fn epa_do_triangle(
    points: &mut [Vec3],
    tris: &mut [usize],
    distances: &mut [Vec3],
    squares: &mut [f32],
    tri: usize,
) {
    let a = points[tris[3 * tri]];
    let b = points[tris[3 * tri + 1]];
    let c = points[tris[3 * tri + 2]];
    let plane_normal = plane_normal_from_points(a, b, c);
    distances[tri] = distance_to_plane(Vec3::ZERO, plane_normal, a);
    squares[tri] = distances[tri].dot(distances[tri]);
}

fn argmin(count: usize, squares: &[f32]) -> usize {
    let mut min = squares[0];
    let mut min_index = 0;
    for i in 1..count {
        if squares[i] < min {
            min = squares[i];
            min_index = i;
        }
    }
    min_index
}

trait Epa3d {
    fn epa(&self, simplex_points: &mut [Vec3]) -> Vec3;
}

impl<A: ExtremePoint3d> Epa3d for A {
    fn epa(&self, simplex_points: &mut [Vec3]) -> Vec3 {
        let mut result = Vec3::ZERO;

        // @todo(lubo): If we imagine two D20's these arrays are too small. Set two D20's case are our baseline.
        const EPA_MAX_POINTS: usize = 32;
        const EPA_MAX_TRIS: usize = 64;
        let mut points = [Vec3::ZERO; EPA_MAX_POINTS];
        let mut tris = [0usize; 3 * EPA_MAX_TRIS];
        for (i, v) in [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2].into_iter().enumerate() {
            tris[i] = v;
        }

        let mut distances = [Vec3::ZERO; EPA_MAX_TRIS];
        let mut squares = [0.0f32; EPA_MAX_TRIS];

        let mut point_count = 4;
        let mut tri_count = 4;

        points[0] = simplex_points[0];
        points[1] = simplex_points[1];
        points[2] = simplex_points[2];
        points[3] = simplex_points[3];

        // NOTE(lubo): For each triangle calculate distance and quandrance (|distance|^2)
        for tri in 0..tri_count {
            // EPA_DO_TRIANGLE(tri);
            epa_do_triangle(&mut points, &mut tris, &mut distances, &mut squares, tri);
        }

        loop {
            // NOTE(lubo): Choose closest triangle
            // let closest_tri = argmin(tri_count, &squares);
            let closest_tri = squares[0..tri_count]
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .unwrap()
                .0;

            let dir: Vec3 = distances[closest_tri];
            let a = self.extreme_point(dir);

            let old_point_score = squares[closest_tri];
            let new_point_score = a.dot(dir);

            if new_point_score - old_point_score <= 0.0001 {
                // NOTE(lubo): We don't see significant score increase. Break.

                // NOTE(lubo): Negative here so that we're consistent with distance (when not colliding).
                // The rule is that E0 + gjk_distance moves E0 so that it touches E1.
                result = -distances[closest_tri];
                break;
            } else {
                // STUDY(lubo): Hypothesis: when all points are on
                //  minkowski surface there cannot be more than 2
                //  triangles facing the new point.  Actually, now I think
                //  the points would have to be extreme and not just lie
                //  on the surface as there could be arbitrarily many point on a straight line on surface.
                let mut facing_triangles = 0;

                let mut unique_edge_count = 0;
                const EPA_MAX_EDGES: usize = 32;
                let mut unique_edges = [0usize; 2 * EPA_MAX_EDGES];

                // NOTE(lubo): For each face, if it is in the same direction as the new point
                let mut tri = 0;
                while tri < tri_count {
                    if distances[tri].dot(a) - squares[tri] > 0.0 {
                        facing_triangles += 1;

                        let ai = tris[3 * tri];
                        let bi = tris[3 * tri + 1];
                        let ci = tris[3 * tri + 2];

                        // NOTE(lubo): Collapse to one? Naah.
                        unique_edge_count = epa_unique_edge_filter(
                            &mut unique_edges,
                            unique_edge_count,
                            EPA_MAX_EDGES,
                            ai,
                            bi,
                        );
                        unique_edge_count = epa_unique_edge_filter(
                            &mut unique_edges,
                            unique_edge_count,
                            EPA_MAX_EDGES,
                            bi,
                            ci,
                        );
                        unique_edge_count = epa_unique_edge_filter(
                            &mut unique_edges,
                            unique_edge_count,
                            EPA_MAX_EDGES,
                            ci,
                            ai,
                        );

                        tris[3 * tri] = tris[3 * (tri_count - 1)];
                        tris[3 * tri + 1] = tris[3 * (tri_count - 1) + 1];
                        tris[3 * tri + 2] = tris[3 * (tri_count - 1) + 2];

                        distances[tri] = distances[tri_count - 1];
                        squares[tri] = squares[tri_count - 1];

                        tri_count -= 1;
                        tri -= 1;
                    }
                }

                if facing_triangles > 2 {
                    println!("[gjk] EPA new point facing {} triangles", facing_triangles);
                }

                let first_new_tri = tri_count;

                for edge in 0..unique_edge_count {
                    assert!(tri_count < EPA_MAX_TRIS);

                    tris[3 * tri_count] = unique_edges[2 * edge];
                    tris[3 * tri_count + 1] = unique_edges[2 * edge + 1];
                    tris[3 * tri_count + 2] = point_count;

                    tri_count += 1;
                }

                assert!(point_count < EPA_MAX_POINTS);
                points[point_count] = a;
                point_count += 1;

                for tri in first_new_tri..tri_count {
                    // EPA_DO_TRIANGLE(tri);
                    epa_do_triangle(&mut points, &mut tris, &mut distances, &mut squares, tri);
                }
            }
        }

        return result;
    }
}

/*

// STUDY(lubo): Just had a though... isn't EPA also convex hull solution? :O
//              At all times all points must lie on the boundary otherwise we would get ill-defined geometry in the add point and reconstruct faces step
internal v3 epa(float scale_a, float scale_b,
                  quaternion orientation_a, quaternion orientation_b,
                  Polyhedron poly_a, Polyhedron poly_b,
                  float radius_a, float radius_b, v3 delta,
                  v3 *simplex_points)
{
    v3 result = {};
    #define EPA_DO_TRIANGLE(I)                                  \
    {                                                           \
        v3 A = points[tris[3*I]];                               \
        v3 B = points[tris[3*I+1]];                             \
        v3 C = points[tris[3*I+2]];                             \
        v3 plane_normal = plane_normal_from_points(A, B, C);    \
        distances[I] = distance_to_plane({}, plane_normal, A);  \
        squares[I] = square(distances[I]);                \
    }

    // TODO/NOTE(lubo): If we imagine two D20 these numbers are super low XD. Set 2 D20 are our baseline.
    #define EPA_MAX_POINTS 32
    #define EPA_MAX_TRIS 64
    v3 points[EPA_MAX_POINTS];
    int tris[3 * EPA_MAX_TRIS] =
    {
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2,
    };
    v3 distances[EPA_MAX_TRIS];
    float squares[EPA_MAX_TRIS];

    int point_count = 4;
    int tri_count = 4;

    points[0] = simplex_points[0];
    points[1] = simplex_points[1];
    points[2] = simplex_points[2];
    points[3] = simplex_points[3];

    // NOTE(lubo): For each triangle calculate distance and quandrance (|distance|^2)
    for(int tri=0; tri<tri_count; ++tri)
    {
        EPA_DO_TRIANGLE(tri);
        // epa_do_triangle(points, tris, distances, squares, tri);
    }

    while(1)
    {
        // NOTE(lubo): Choose closest triangle
        int closest_tri = argmin(tri_count, squares);

        v3 dir = distances[closest_tri];
        v3 A = support_function(scale_a, scale_b, orientation_a, orientation_b, poly_a, poly_b, radius_a, radius_b, delta, dir);

        float old_point_score = squares[closest_tri];
        float new_point_score = A.dot(dir);

        if(new_point_score-old_point_score <= 0.0001f)
        {
            // NOTE(lubo): We don't see significant score increase. Break.

            // NOTE(lubo): Negative here so that we're consistent with distance (when not colliding).
            // The rule is that E0 + gjk_distance moves E0 so that it touches E1.
            result = -distances[closest_tri];
            break;
        }
        else
        {
            // STUDY(lubo): Hypothesis: when all points are on
            //  minkowski surface there cannot be more than 2
            //  triangles facing the new point.  Actually, now I think
            //  the points would have to be extreme and not just lie
            //  on the surface as there could be arbitrarily many point on a straight line on surface.
            int facing_triangles = 0;

            int unique_edge_count = 0;
            #define EPA_MAX_EDGES 32
            int unique_edges[2*EPA_MAX_EDGES];

            // NOTE(lubo): For each face, if it is in the same direction as the new point
            for(int tri=0; tri<tri_count; ++tri)
            {
                if(distances[tri].dot(A) - squares[tri] > 0)
                {
                    ++facing_triangles;

                    int Ai = tris[3*tri];
                    int Bi = tris[3*tri + 1];
                    int Ci = tris[3*tri + 2];

                    // NOTE(lubo): Collapse to one? Naah.
                    unique_edge_count = epa_unique_edge_filter(unique_edges, unique_edge_count, ArrayCount(unique_edges), Ai, Bi);
                    unique_edge_count = epa_unique_edge_filter(unique_edges, unique_edge_count, ArrayCount(unique_edges), Bi, Ci);
                    unique_edge_count = epa_unique_edge_filter(unique_edges, unique_edge_count, ArrayCount(unique_edges), Ci, Ai);

                    tris[3*tri] = tris[3*(tri_count-1)];
                    tris[3*tri + 1] = tris[3*(tri_count-1) + 1];
                    tris[3*tri + 2] = tris[3*(tri_count-1) + 2];

                    distances[tri] = distances[tri_count-1];
                    squares[tri] = squares[tri_count-1];

                    --tri_count;
                    --tri;
                }
            }

            // STUDY(lubo): Can we always guarantee this? We could optimize epa somewhat if so.
            if(facing_triangles > 2) loginfo("gjk", "EPA new point facing %d triangles", facing_triangles);

            int first_new_tri = tri_count;

            for(int edge=0; edge<unique_edge_count; ++edge)
            {
                assert(tri_count < EPA_MAX_TRIS);

                tris[3*tri_count] = unique_edges[2*edge];
                tris[3*tri_count + 1] = unique_edges[2*edge + 1];
                tris[3*tri_count + 2] = point_count;

                ++tri_count;
            }

            assert(point_count < EPA_MAX_POINTS);
            points[point_count] = A;
            ++point_count;

            for(int tri=first_new_tri; tri<tri_count; ++tri)
            {
                EPA_DO_TRIANGLE(tri);
                // epa_do_triangle(points, tris, distances, squares, tri);
            }
        }
    }

    return result;
}
*/

/*
struct Collision_Info
{
    b32x collides;
    v3 distance;
    //v3 closest_point;
    int point_count;
};
*/

/*
/// The initial `dir`
fn gjk(mut dir: Vec3) -> bool
{

    let mut point_count: usize = 0;

    let mut points: [Vec3; 4] = [Vec3::ZERO; 4];

    let start_point = support_function(scale_a, scale_b, orientation_a, orientation_b, poly_a, poly_b, radius_a, radius_b, delta, dir);
    point_count = 1;
    points[0] = start_point;
    dir = -start_point;

    // NOTE(lubo):
    //   Stage 0 - finding a simplex that encloses the origin
    //   Stage 1 - finding the closest simplex to the origin
    b32x stage = 0;

    b32x contained_origin = false;
    while(!contained_origin && iteration_count < max_allowed_iterations)
    {
        v3 new_point = support_function(scale_a, scale_b, orientation_a, orientation_b, poly_a, poly_b, radius_a, radius_b, delta, dir);
        float new_point_score = new_point.dot(dir);

        if(stage == 0 && new_point_score < float_tiny)
        {
            if(calc_distance)
            {
                stage = 1;
            }
            else
            {
                break;
            }
        }

        if(stage == 1)
        {
            float old_point_score = points[0].dot(dir);
            if(new_point_score-old_point_score <= 0.0001f)
            {
                collision_info.point_count = point_count;

                // TODO(lubo): Ensure order?
                // NOTE(lubo): The order breaks in stage 1 (searching closest feature):
                //
                // +
                //
                //    B A
                //
                //        C
                //
                // BC is in correct order
                // BC finds A
                // AB forms the new simplex
                // AB does not find closer point
                // AB is in reverse order
                // AB is incorrectly chosen as closest feature (instead of B)
                //
                //collision_info.distance = distance_to_closest_feature(points, point_count, dir);
                collision_info.distance = distance_to_closest_feature_unordered(points, point_count, dir);

                break;
            }
        }

        points[point_count] = new_point;
        point_count += 1;

        contained_origin = do_simplex(points, &dir, &point_count);

        // TODO(lubo): Is this still a problem? Probably yes..... ?
        // STUDY(lubo): BUG When we look for closest simplex
        // and errouneously enclose origin because of degenerate
        // geometry just halt :DDD
        if(stage == 1 && contained_origin == true)
        {
            // NOTE(lubo): Actually let's break here. We want this to be reliable. If this happens try to handle it properly.
            //InvalidCodePath;

            // NOTE(lubo): Example of failure:
            // camera
//         +        pos {x=18.9328766 y=47.9470024 z=-2.02284360 ...}   v3

            // obstacle
//         +  quaternion      -0.155942202
//-0.705814779
//-0.674746871
//0.149078086
//         +        pos {x=-1.00000000 y=0.000000000 z=-0.000000000 ...}    v3


            contained_origin = false;
            v3 origin = {0,0,0};
            collision_info.distance = distance_to_plane(origin, dir, points[0]);
            break;
        }

        if(contained_origin && calc_penetration && point_count == 4)
        {
            assert(point_count == 4);

            collision_info.distance = epa(scale_a, scale_b, orientation_a, orientation_b, poly_a, poly_b, radius_a, radius_b, delta, points);
        }

        iteration_count += 1;
    }

    collision_info.collides = contained_origin;
    return collision_info;
}
*/
