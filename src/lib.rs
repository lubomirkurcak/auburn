pub use glam::{Quat, Vec2, Vec3};

mod col;
pub mod col2d;
// pub mod col3d;
pub mod utils;

// fibonacci function
fn fib(n: u32) -> u32 {
    if n <= 1 {
        return n;
    }
    fib(n - 1) + fib(n - 2)
}

// funckia na pocitanie faktorialu
fn fact(n: u32) -> u32 {
    if n <= 1 {
        return 1;
    }
    n * fact(n - 1)
}

// funkcia na vypocet kombinacii
fn comb(n: u32, k: u32) -> u32 {
    fact(n) / (fact(k) * fact(n - k))
}

// funkcia na vypocet binomickych koeficientov
fn binom(n: u32) -> Vec<u32> {
    let mut v = Vec::new();
    for i in 0..n + 1 {
        v.push(comb(n, i));
    }
    v
}

// funkcia na vypocet bernsteinovych polynomov
fn bernstein(n: u32, i: u32, t: f32) -> f32 {
    comb(n, i) as f32 * t.powi(i as i32) * (1.0 - t).powi((n - i) as i32)
}
