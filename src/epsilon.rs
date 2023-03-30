use core::ops::Mul;

const EPSILON: f64 = f64::EPSILON;
const MAX_10_EXP: i32 = f64::MAX_10_EXP;

/// `kurbo`'s epsilon (ε).
#[repr(packed)]
#[derive(Copy, Clone, Debug)]
pub struct Epsilon {
    /// Value of this epsilon. By default, [`f64::EPSILON`].
    pub value: f64,
    /// Magnitude of this epsilon. By default, [`f64::MAX_10_EXP`].
    pub magnitude: i32,
}

impl Default for Epsilon {
    #[inline]
    fn default() -> Epsilon {
        Self::new()
    }
}

impl Epsilon {
    /// Raises or lowers the magnitude of `kurbo`'s epsilon by `magnitude`, returning a new
    /// [`Epsilon`].
    ///
    /// Returns a new `Epsilon` representing **ε × 10_ᵐ_** where _m_ = magnitude.
    #[inline]
    pub const fn ten_pow(magnitude: i32) -> Self {
        debug_assert!(MAX_10_EXP + magnitude <= MAX_10_EXP);

        Epsilon {
            magnitude: MAX_10_EXP + magnitude,
            value: f64::mul(EPSILON, 10.0 * magnitude as f64)
        }
    }
}

impl Epsilon {
    /// Create `kurbo`'s default epsilon.
    #[inline]
    pub const fn new() -> Self {
        Epsilon {
            value: EPSILON,
            magnitude: MAX_10_EXP
        }
    }
}
