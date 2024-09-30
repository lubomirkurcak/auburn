use local_minkowski_diff::LocalMinkowskiDiff2d;

use super::*;

impl SdfvCommonRel2d<false, false, Box2d> for Box2d {
    fn sdfv_common_rel(&self, other: &Box2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, false>::sdfv_minkowski(&diff)
    }
}

impl SdfvCommonRel2d<false, true, Box2d> for Box2d {
    fn sdfv_common_rel(&self, other: &Box2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<false, true>::sdfv_minkowski(&diff)
    }
}

impl SdfvCommonRel2d<true, false, Box2d> for Box2d {
    fn sdfv_common_rel(&self, other: &Box2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<true, false>::sdfv_minkowski(&diff)
    }
}

impl SdfvCommonRel2d<true, true, Box2d> for Box2d {
    fn sdfv_common_rel(&self, other: &Box2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let diff = LocalMinkowskiDiff2d::raw(self, other, rel);
        SdfvMinkowski2d::<true, true>::sdfv_minkowski(&diff)
    }
}

/*
impl<const COMPUTE_PENETRATION: bool, const COMPUTE_DISTANCE: bool>
    SdfvCommonRel2d<COMPUTE_PENETRATION, COMPUTE_DISTANCE, Box2d> for Box2d
{
    fn sdfv_common_rel(&self, b: &Box2d, rel: &impl Transformation2d) -> (bool, Vec2) {
        let b_center = rel.apply_origin();
        let towards_self = -b_center;
        let p = b.extreme_point_t(rel, towards_self);

        let collides;

        if b_center.x.is_sign_positive() {
            if b_center.y.is_sign_positive() {
                collides = p.x < self.halfsize.x && p.y < self.halfsize.y
            } else {
                collides = p.x < self.halfsize.x && p.y >= -self.halfsize.y
            }
        } else {
            if b_center.y.is_sign_positive() {
                collides = p.x >= -self.halfsize.x && p.y < self.halfsize.y
            } else {
                collides = p.x >= -self.halfsize.x && p.y >= -self.halfsize.y
            }
        }

        if collides {
            let mut penetration = Vec2::NAN;

            if COMPUTE_PENETRATION {
                if b_center.x.is_sign_positive() {
                    if b_center.y.is_sign_positive() {
                        let px = p.x - self.halfsize.x;
                        let py = p.y - self.halfsize.y;

                        if px > py {
                            penetration = Vec2::new(px, 0.0);
                        } else {
                            penetration = Vec2::new(0.0, py);
                        }
                    } else {
                        let px = p.x - self.halfsize.x;
                        let py = -p.y - self.halfsize.y;

                        if px > py {
                            penetration = Vec2::new(px, 0.0);
                        } else {
                            penetration = Vec2::new(0.0, -py);
                        }
                    }
                } else {
                    if b_center.y.is_sign_positive() {
                        let px = -p.x - self.halfsize.x;
                        let py = p.y - self.halfsize.y;

                        if px > py {
                            penetration = Vec2::new(-px, 0.0);
                        } else {
                            penetration = Vec2::new(0.0, py);
                        }
                    } else {
                        let px = -p.x - self.halfsize.x;
                        let py = -p.y - self.halfsize.y;

                        if px > py {
                            penetration = Vec2::new(-px, 0.0);
                        } else {
                            penetration = Vec2::new(0.0, -py);
                        }
                    }
                }
            }

            (true, penetration)
        } else {
            let mut distance = Vec2::NAN;

            if COMPUTE_DISTANCE {
                if b_center.x.is_sign_positive() {
                    if b_center.y.is_sign_positive() {
                        let px = p.x - self.halfsize.x;
                        let py = p.y - self.halfsize.y;

                        if px.is_sign_positive() {
                            if py.is_sign_positive() {
                                distance = Vec2::new(px, py);
                            } else {
                                distance = Vec2::new(px, 0.0);
                            }
                        } else {
                            // py *must* be positive
                            distance = Vec2::new(0.0, py);
                        }
                    } else {
                        let px = p.x - self.halfsize.x;
                        let py = -p.y - self.halfsize.y;

                        if px.is_sign_positive() {
                            if py.is_sign_positive() {
                                distance = Vec2::new(px, -py);
                            } else {
                                distance = Vec2::new(px, 0.0);
                            }
                        } else {
                            // py *must* be positive
                            distance = Vec2::new(0.0, -py);
                        }
                    }
                } else {
                    if b_center.y.is_sign_positive() {
                        let px = -p.x - self.halfsize.x;
                        let py = p.y - self.halfsize.y;

                        if px.is_sign_positive() {
                            if py.is_sign_positive() {
                                distance = Vec2::new(-px, py);
                            } else {
                                distance = Vec2::new(-px, 0.0);
                            }
                        } else {
                            // py *must* be positive
                            distance = Vec2::new(0.0, py);
                        }
                    } else {
                        let px = -p.x - self.halfsize.x;
                        let py = -p.y - self.halfsize.y;

                        if px.is_sign_positive() {
                            if py.is_sign_positive() {
                                distance = Vec2::new(-px, -py);
                            } else {
                                distance = Vec2::new(-px, 0.0);
                            }
                        } else {
                            // py *must* be positive
                            distance = Vec2::new(0.0, -py);
                        }
                    }
                }
            }

            (false, distance)
        }
    }
}
*/

// TODO: This is probably incorrect
impl SdfRel2d<Box2d> for Box2d {
    fn sdf_rel(&self, b: &Box2d, rel: &impl Transformation2d) -> f32 {
        let b_center = rel.apply_origin();
        let towards_self = -b_center;
        let p = b.extreme_point_t(rel, towards_self);
        let px = p.x.abs() - self.halfsize.x;
        let py = p.y.abs() - self.halfsize.y;

        if self.collides_rel(b, rel) {
            px.min(py)
        } else {
            if px.is_sign_negative() {
                py
            } else if py.is_sign_negative() {
                px
            } else {
                Vec2::new(px, py).length()
            }
        }
    }
}
