use minkowski_diff::MinkowskiDiff2d;

use crate::{debug, error, info, trace, warn};

use super::*;

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

