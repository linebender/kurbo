// Copyright 2018 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Common mathematical operations

#![allow(missing_docs)]

#[cfg(not(feature = "std"))]
mod sealed {
    /// A [sealed trait](https://predr.ag/blog/definitive-guide-to-sealed-traits-in-rust/)
    /// which stops [`super::FloatFuncs`] from being implemented outside bazo. This could
    /// be relaxed in the future if there is are good reasons to allow external impls.
    /// The benefit from being sealed is that we can add methods without breaking downstream
    /// implementations.
    pub trait FloatFuncsSealed {}
}

/// Defines a trait that chooses between libstd or libm implementations of float methods.
///
/// Some methods will eventually became available in core by
/// [`core_float_math`](https://github.com/rust-lang/rust/issues/137578)
macro_rules! define_float_funcs {
    ($(
        fn $name:ident(self $(,$arg:ident: $arg_ty:ty)*) -> $ret:ty
        => $lname:ident/$lfname:ident;
    )+) => {

        /// Since core doesn't depend upon libm, this provides libm implementations
        /// of float functions which are typically provided by the std library, when
        /// the `std` feature is not enabled.
        ///
        /// For documentation see the respective functions in the std library.
        #[cfg(not(feature = "std"))]
        pub trait FloatFuncs : Sized + sealed::FloatFuncsSealed {
            /// For documentation see <https://doc.rust-lang.org/std/primitive.f64.html#method.signum>
            ///
            /// Special implementation, because libm doesn't have it.
            fn signum(self) -> Self;

            /// For documentation see <https://doc.rust-lang.org/std/primitive.f64.html#method.rem_euclid>
            ///
            /// Special implementation, because libm doesn't have it.
            fn rem_euclid(self, rhs: Self) -> Self;

            $(fn $name(self $(,$arg: $arg_ty)*) -> $ret;)+
        }

        #[cfg(not(feature = "std"))]
        impl sealed::FloatFuncsSealed for f32 {}

        #[cfg(not(feature = "std"))]
        impl FloatFuncs for f32 {
            #[inline]
            fn signum(self) -> f32 {
                if self.is_nan() {
                    f32::NAN
                } else {
                    1.0_f32.copysign(self)
                }
            }

            #[inline]
            fn rem_euclid(self, rhs: Self) -> Self {
                let r = self % rhs;
                if r < 0.0 {
                    r + rhs.abs()
                } else {
                    r
                }
            }

            $(fn $name(self $(,$arg: $arg_ty)*) -> $ret {
                #[cfg(feature = "libm")]
                return libm::$lfname(self $(,$arg as _)*);

                #[cfg(not(feature = "libm"))]
                compile_error!("bazo requires either the `std` or `libm` feature")
            })+
        }

        #[cfg(not(feature = "std"))]
        impl sealed::FloatFuncsSealed for f64 {}
        #[cfg(not(feature = "std"))]
        impl FloatFuncs for f64 {
            #[inline]
            fn signum(self) -> f64 {
                if self.is_nan() {
                    f64::NAN
                } else {
                    1.0_f64.copysign(self)
                }
            }

            #[inline]
            fn rem_euclid(self, rhs: Self) -> Self {
                let r = self % rhs;
                if r < 0.0 {
                    r + rhs.abs()
                } else {
                    r
                }
            }

            $(fn $name(self $(,$arg: $arg_ty)*) -> $ret {
                #[cfg(feature = "libm")]
                return libm::$lname(self $(,$arg as _)*);

                #[cfg(not(feature = "libm"))]
                compile_error!("bazo requires either the `std` or `libm` feature")
            })+
        }
    }
}

define_float_funcs! {
    fn abs(self) -> Self => fabs/fabsf;
    fn acos(self) -> Self => acos/acosf;
    fn atan2(self, other: Self) -> Self => atan2/atan2f;
    fn cbrt(self) -> Self => cbrt/cbrtf;
    fn ceil(self) -> Self => ceil/ceilf;
    fn cos(self) -> Self => cos/cosf;
    fn copysign(self, sign: Self) -> Self => copysign/copysignf;
    fn floor(self) -> Self => floor/floorf;
    fn hypot(self, other: Self) -> Self => hypot/hypotf;
    fn ln(self) -> Self => log/logf;
    fn log2(self) -> Self => log2/log2f;
    fn mul_add(self, a: Self, b: Self) -> Self => fma/fmaf;
    fn powi(self, n: i32) -> Self => pow/powf;
    fn powf(self, n: Self) -> Self => pow/powf;
    fn round(self) -> Self => round/roundf;
    fn sin(self) -> Self => sin/sinf;
    fn sin_cos(self) -> (Self, Self) => sincos/sincosf;
    fn sqrt(self) -> Self => sqrt/sqrtf;
    fn tan(self) -> Self => tan/tanf;
    fn trunc(self) -> Self => trunc/truncf;
}

/// Adds convenience methods to `f32` and `f64`.
pub trait FloatExt<T> {
    /// Rounds to the nearest integer away from zero,
    /// unless the provided value is already an integer.
    ///
    /// It is to `ceil` what `trunc` is to `floor`.
    ///
    /// # Examples
    ///
    /// ```
    /// use bazo::common::FloatExt;
    ///
    /// let f = 3.7_f64;
    /// let g = 3.0_f64;
    /// let h = -3.7_f64;
    /// let i = -5.1_f32;
    ///
    /// assert_eq!(f.expand(), 4.0);
    /// assert_eq!(g.expand(), 3.0);
    /// assert_eq!(h.expand(), -4.0);
    /// assert_eq!(i.expand(), -6.0);
    /// ```
    fn expand(&self) -> T;
}

impl FloatExt<f64> for f64 {
    #[inline]
    fn expand(&self) -> f64 {
        self.abs().ceil().copysign(*self)
    }
}

impl FloatExt<f32> for f32 {
    #[inline]
    fn expand(&self) -> f32 {
        self.abs().ceil().copysign(*self)
    }
}
