//! Common mathematical operations

use arrayvec::ArrayVec;

/// Find real roots of cubic equation.
///
/// Assumes c3 is nonzero.
///
/// See: <http://mathworld.wolfram.com/CubicFormula.html>
///
/// Returns values of x for which c0 + c1 x + c2 x² + c3 x³ = 0.
pub fn solve_cubic(c0: f64, c1: f64, c2: f64, c3: f64) -> ArrayVec<[f64; 3]> {
    let mut result = ArrayVec::new();
    let c3_recip = c3.recip();
    let c2 = c2 * c3_recip;
    let c1 = c1 * c3_recip;
    let c0 = c0 * c3_recip;
    let q = c1 * (1.0 / 3.0) - c2 * c2 * (1.0 / 9.0); // Q
    let r = (1.0 / 6.0) * c2 * c1 - (1.0 / 27.0) * c2.powi(3) - c0 * 0.5; // R
    let d = q.powi(3) + r * r; // D
    let x0 = c2 * (1.0 / 3.0);
    if d > 0.0 {
        let sq = d.sqrt();
        let t1 = (r + sq).cbrt() + (r - sq).cbrt();
        result.push(t1 - x0);
    } else if d == 0.0 {
        let t1 = -r.cbrt();
        let x1 = t1 - x0;
        result.push(x1);
        result.push(-2.0 * t1 - x0);
    } else {
        let sq = (-d).sqrt();
        let rho = r.hypot(sq);
        let th = sq.atan2(r) * (1.0 / 3.0);
        let cbrho = rho.cbrt();
        let c = th.cos();
        let ss3 = th.sin() * 3.0f64.sqrt();
        result.push(2.0 * cbrho * c - x0);
        result.push(-cbrho * (c + ss3) - x0);
        result.push(-cbrho * (c - ss3) - x0);
    }
    result
}

/// Find real roots of quadratic equation.
///
/// Returns values of x for which c0 + c1 x + c2 x² = 0.
pub fn solve_quadratic(c0: f64, c1: f64, c2: f64) -> ArrayVec<[f64; 2]> {
    let mut result = ArrayVec::new();
    // TODO: make this more robust to very small values.
    if c2 == 0.0 {
        if c1 == 0.0 {
            return result;
        }
        result.push(-c0 / c1);
        return result;
    }
    let d = c1 * c1 - 4.0 * c2 * c0;
    let denom = -0.5 / c2;
    if d == 0.0 {
        result.push(c1 * denom);
    } else if d > 0.0 {
        let q = d.sqrt();
        result.push((c1 - q) * denom);
        result.push((c1 + q) * denom);
    }
    result
}

/// Tables of Legendre-Gauss quadrature coefficients, adapted from:
/// <https://pomax.github.io/bezierinfo/legendre-gauss.html>

pub const GAUSS_LEGENDRE_COEFFS_3: &[(f64, f64)] = &[
    (0.8888888888888888,  0.0000000000000000),
    (0.5555555555555556,  -0.7745966692414834),
    (0.5555555555555556,  0.7745966692414834),
];

pub const GAUSS_LEGENDRE_COEFFS_5: &[(f64, f64)] = &[
    (0.5688888888888889,  0.0000000000000000),
    (0.4786286704993665,  -0.5384693101056831),
    (0.4786286704993665,  0.5384693101056831),
    (0.2369268850561891,  -0.9061798459386640),
    (0.2369268850561891,  0.9061798459386640),
];

pub const GAUSS_LEGENDRE_COEFFS_7: &[(f64, f64)] = &[
    (0.4179591836734694,  0.0000000000000000),
    (0.3818300505051189,  0.4058451513773972),
    (0.3818300505051189,  -0.4058451513773972),
    (0.2797053914892766,  -0.7415311855993945),
    (0.2797053914892766,  0.7415311855993945),
    (0.1294849661688697,  -0.9491079123427585),
    (0.1294849661688697,  0.9491079123427585),
];

pub const GAUSS_LEGENDRE_COEFFS_9: &[(f64, f64)] = &[
    (0.3302393550012598,  0.0000000000000000),
    (0.1806481606948574,  -0.8360311073266358),
    (0.1806481606948574,  0.8360311073266358),
    (0.0812743883615744,  -0.9681602395076261),
    (0.0812743883615744,  0.9681602395076261),
    (0.3123470770400029,  -0.3242534234038089),
    (0.3123470770400029,  0.3242534234038089),
    (0.2606106964029354,  -0.6133714327005904),
    (0.2606106964029354,  0.6133714327005904),
];

pub const GAUSS_LEGENDRE_COEFFS_11: &[(f64, f64)] = &[
    (0.2729250867779006,  0.0000000000000000),
    (0.2628045445102467,  -0.2695431559523450),
    (0.2628045445102467,  0.2695431559523450),
    (0.2331937645919905,  -0.5190961292068118),
    (0.2331937645919905,  0.5190961292068118),
    (0.1862902109277343,  -0.7301520055740494),
    (0.1862902109277343,  0.7301520055740494),
    (0.1255803694649046,  -0.8870625997680953),
    (0.1255803694649046,  0.8870625997680953),
    (0.0556685671161737,  -0.9782286581460570),
    (0.0556685671161737,  0.9782286581460570),
];

pub const GAUSS_LEGENDRE_COEFFS_24: &[(f64, f64)] = &[
    (0.1279381953467522,  -0.0640568928626056),
    (0.1279381953467522,  0.0640568928626056),
    (0.1258374563468283,  -0.1911188674736163),
    (0.1258374563468283,  0.1911188674736163),
    (0.1216704729278034,  -0.3150426796961634),
    (0.1216704729278034,  0.3150426796961634),
    (0.1155056680537256,  -0.4337935076260451),
    (0.1155056680537256,  0.4337935076260451),
    (0.1074442701159656,  -0.5454214713888396),
    (0.1074442701159656,  0.5454214713888396),
    (0.0976186521041139,  -0.6480936519369755),
    (0.0976186521041139,  0.6480936519369755),
    (0.0861901615319533,  -0.7401241915785544),
    (0.0861901615319533,  0.7401241915785544),
    (0.0733464814110803,  -0.8200019859739029),
    (0.0733464814110803,  0.8200019859739029),
    (0.0592985849154368,  -0.8864155270044011),
    (0.0592985849154368,  0.8864155270044011),
    (0.0442774388174198,  -0.9382745520027328),
    (0.0442774388174198,  0.9382745520027328),
    (0.0285313886289337,  -0.9747285559713095),
    (0.0285313886289337,  0.9747285559713095),
    (0.0123412297999872,  -0.9951872199970213),
    (0.0123412297999872,  0.9951872199970213),
];

#[cfg(test)]
mod tests {
    use arrayvec::{Array, ArrayVec};
    use crate::common::*;

    fn verify<T: Array<Item = f64>>(mut roots: ArrayVec<T>, expected: &[f64]) {
        //println!("{:?} {:?}", roots, expected);
        assert!(expected.len() == roots.len());
        let epsilon = 1e-6;
        roots.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for i in 0..expected.len() {
            assert!((roots[i] - expected[i]).abs() < epsilon);
        }
    }

    #[test]
    fn test_solve_cubic() {
        verify(solve_cubic(-5.0, 0.0, 0.0, 1.0), &[5.0f64.cbrt()]);
        verify(solve_cubic(-5.0, -1.0, 0.0, 1.0), &[1.90416085913492]);
        verify(solve_cubic(0.0, -1.0, 0.0, 1.0), &[-1.0, 0.0, 1.0]);
        verify(solve_cubic(-2.0, -3.0, 0.0, 1.0), &[-1.0, 2.0]);
        verify(solve_cubic(2.0, -3.0, 0.0, 1.0), &[-2.0, 1.0]);
        verify(solve_cubic(2.0 - 1e-12, 5.0, 4.0, 1.0), &[-2.0, -1.0, -1.0]);
        verify(solve_cubic(2.0 + 1e-12, 5.0, 4.0, 1.0), &[-2.0]);
    }

    #[test]
    fn test_solve_quadratic() {
        verify(solve_quadratic(-5.0, 0.0, 1.0), &[-5.0f64.sqrt(), 5.0f64.sqrt()]);
        verify(solve_quadratic(5.0, 0.0, 1.0), &[]);
        verify(solve_quadratic(5.0, 1.0, 0.0), &[-5.0]);
        verify(solve_quadratic(1.0, 2.0, 1.0), &[-1.0]);
    }
}