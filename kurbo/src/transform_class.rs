// Copyright 2025 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Classification for how two transforms differ.

/// Default tolerance used by [`crate::Affine::transform_class_to`].
///
/// This is intended to treat matrices produced by common floating point
/// operations (such as `sin_cos`) as "exact enough" for classification, while
/// still being strict.
pub const DEFAULT_TRANSFORM_CLASS_EPSILON: f64 = 1e-12;

/// A conservative classification for how a recorded operation can be replayed
/// under a different transform.
///
/// This is primarily intended for comparing two transforms by classifying their
/// *delta* transform (for example `current * recorded.inverse()`), rather than
/// classifying a single transform in isolation.
///
/// Classes are ordered from most restrictive to most permissive, so
/// `actual <= valid_under` means that a delta classified as `actual` can be
/// used where `valid_under` is allowed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TransformClass {
    /// Valid only when the compared transforms are exactly equal.
    Exact,
    /// Valid when transforms differ by a pure translation (linear part is identity).
    TranslateOnly,
    /// Valid under affine transforms whose linear part is a uniform-scale
    /// orthogonal transform (rotation/reflection, no shear).
    ///
    /// Note: the name comes from downstream usage; mathematically this is a
    /// "similarity transform" when the scale is non-`1`.
    Orthonormal,
    /// Valid under any affine transform.
    Affine,
}

impl TransformClass {
    #[inline]
    const fn rank(self) -> u8 {
        match self {
            Self::Exact => 0,
            Self::TranslateOnly => 1,
            Self::Orthonormal => 2,
            Self::Affine => 3,
        }
    }

    /// Returns `true` if this class includes all transforms in `other`.
    ///
    /// This is useful when a cached operation is valid under `self`, and a
    /// transform delta has been classified as `other`.
    #[inline]
    pub const fn contains(self, other: Self) -> bool {
        other.rank() <= self.rank()
    }

    /// Returns `true` if every transform in this class is included in `other`.
    #[inline]
    pub const fn is_contained_by(self, other: Self) -> bool {
        other.contains(self)
    }

    /// Return the least permissive class that contains both `self` and `other`.
    ///
    /// This can be used as the step function when folding over transform
    /// classes:
    ///
    /// ```
    /// # use kurbo::TransformClass;
    /// let classes = [
    ///     TransformClass::Exact,
    ///     TransformClass::TranslateOnly,
    ///     TransformClass::Orthonormal,
    /// ];
    ///
    /// let combined = classes
    ///     .into_iter()
    ///     .fold(TransformClass::default(), TransformClass::union);
    ///
    /// assert_eq!(combined, TransformClass::Orthonormal);
    /// ```
    #[inline]
    pub const fn union(self, other: Self) -> Self {
        if self.rank() >= other.rank() {
            self
        } else {
            other
        }
    }
}

impl Default for TransformClass {
    #[inline]
    fn default() -> Self {
        Self::Exact
    }
}

#[cfg(test)]
mod tests {
    use super::TransformClass;

    #[test]
    fn class_containment() {
        assert!(TransformClass::Exact.contains(TransformClass::Exact));
        assert!(!TransformClass::Exact.contains(TransformClass::TranslateOnly));

        assert!(TransformClass::TranslateOnly.contains(TransformClass::Exact));
        assert!(TransformClass::TranslateOnly.contains(TransformClass::TranslateOnly));
        assert!(!TransformClass::TranslateOnly.contains(TransformClass::Orthonormal));

        assert!(TransformClass::Orthonormal.contains(TransformClass::TranslateOnly));
        assert!(TransformClass::Affine.contains(TransformClass::Orthonormal));
        assert!(TransformClass::Orthonormal.is_contained_by(TransformClass::Affine));
        assert!(!TransformClass::Affine.is_contained_by(TransformClass::Orthonormal));
    }

    #[test]
    fn class_order_matches_containment() {
        let classes = [
            TransformClass::Exact,
            TransformClass::TranslateOnly,
            TransformClass::Orthonormal,
            TransformClass::Affine,
        ];

        for valid_under in classes {
            for actual in classes {
                assert_eq!(valid_under.contains(actual), actual <= valid_under);
            }
        }
    }

    #[test]
    fn class_union() {
        assert_eq!(
            TransformClass::Exact.union(TransformClass::TranslateOnly),
            TransformClass::TranslateOnly
        );
        assert_eq!(
            TransformClass::Orthonormal.union(TransformClass::TranslateOnly),
            TransformClass::Orthonormal
        );
        assert_eq!(
            TransformClass::Affine.union(TransformClass::Exact),
            TransformClass::Affine
        );
    }

    #[test]
    fn class_fold() {
        let classes = [
            TransformClass::Exact,
            TransformClass::TranslateOnly,
            TransformClass::Orthonormal,
        ];

        let class = classes
            .into_iter()
            .fold(TransformClass::default(), TransformClass::union);

        assert_eq!(class, TransformClass::Orthonormal);
    }
}
