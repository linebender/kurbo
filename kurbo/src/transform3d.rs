// Copyright 2026 the Kurbo Authors
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! 3D homogeneous transforms.

use core::ops::{Mul, MulAssign};

use crate::{Affine, Point, TranslateScale};

#[cfg(not(feature = "std"))]
use crate::common::FloatFuncs;

/// A 3D homogeneous transform stored as a column-major 4 by 4 matrix.
///
/// The coefficients use the same order as CSS `matrix3d()` and
/// [`DOMMatrix`](https://drafts.csswg.org/geometry/#dommatrix):
///
/// ```text
/// | x' |   | m11 m21 m31 m41 |   | x |
/// | y' | = | m12 m22 m32 m42 | * | y |
/// | z' |   | m13 m23 m33 m43 |   | z |
/// | w' |   | m14 m24 m34 m44 |   | w |
/// ```
///
/// The translation terms are `m41`, `m42`, and `m43`. Multiplication follows
/// the same convention as [`Affine`]: `(A * B) * p == A * (B * p)`, so `B` is
/// applied first and `A` is applied second.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Transform3d([[f64; 4]; 4]);

impl Transform3d {
    /// The identity transform.
    pub const IDENTITY: Self = Self::scale(1.0);

    /// Construct a transform from 16 coefficients in column-major order.
    ///
    /// This is the same order used by CSS `matrix3d()` and `DOMMatrix` arrays:
    /// `[m11, m12, m13, m14, m21, ... m44]`.
    #[inline]
    #[must_use]
    #[rustfmt::skip]
    pub const fn new(coeffs: [f64; 16]) -> Self {
        Self([
            [coeffs[0], coeffs[1], coeffs[2], coeffs[3]],
            [coeffs[4], coeffs[5], coeffs[6], coeffs[7]],
            [coeffs[8], coeffs[9], coeffs[10], coeffs[11]],
            [coeffs[12], coeffs[13], coeffs[14], coeffs[15]],
        ])
    }

    /// Construct a transform from its four columns.
    ///
    /// Each column contains the contribution of one input coordinate to the
    /// output `[x, y, z, w]`. When transforming a column vector
    /// `[x, y, z, w]`, `col0` is multiplied by the input `x`, `col1` by the
    /// input `y`, `col2` by the input `z`, and `col3` by the input `w`; the
    /// scaled columns are then added. For points with `w == 1`, `col3` is the
    /// translation column.
    #[inline]
    #[must_use]
    pub const fn from_cols(col0: [f64; 4], col1: [f64; 4], col2: [f64; 4], col3: [f64; 4]) -> Self {
        Self([col0, col1, col2, col3])
    }

    /// Construct a transform from a column-major two-dimensional array.
    ///
    /// The first index selects a column and the second index selects a row.
    #[inline]
    #[must_use]
    pub const fn from_cols_array_2d(cols: [[f64; 4]; 4]) -> Self {
        Self(cols)
    }

    /// Return this transform as 16 coefficients in column-major order.
    ///
    /// This is the same order accepted by [`Transform3d::new`].
    #[inline]
    #[must_use]
    #[rustfmt::skip]
    pub const fn as_coeffs(self) -> [f64; 16] {
        let c = self.0;
        [
            c[0][0], c[0][1], c[0][2], c[0][3],
            c[1][0], c[1][1], c[1][2], c[1][3],
            c[2][0], c[2][1], c[2][2], c[2][3],
            c[3][0], c[3][1], c[3][2], c[3][3],
        ]
    }

    /// Return this transform as a column-major two-dimensional array.
    ///
    /// The first index selects a column and the second index selects a row.
    #[inline]
    #[must_use]
    pub const fn to_cols_array_2d(self) -> [[f64; 4]; 4] {
        self.0
    }

    /// Return column `i`.
    ///
    /// # Panics
    ///
    /// Panics if `i >= 4`.
    #[inline]
    #[must_use]
    pub const fn col(self, i: usize) -> [f64; 4] {
        self.0[i]
    }

    /// Return row `i` in conventional matrix notation.
    ///
    /// This gathers the `i`th coefficient from each stored column. Row 0 holds
    /// the coefficients used to compute the output x coordinate, row 1 the
    /// output y coordinate, and so on.
    ///
    /// # Panics
    ///
    /// Panics if `i >= 4`.
    #[inline]
    #[must_use]
    pub const fn row(self, i: usize) -> [f64; 4] {
        let c = self.0;
        [c[0][i], c[1][i], c[2][i], c[3][i]]
    }

    /// Construct the 3D transform with the same effect as a 2D [`Affine`] in
    /// the xy plane.
    ///
    /// Points are embedded as `[x, y, 0, 1]`; the resulting transform applies
    /// the affine transform to x and y and leaves z unchanged.
    #[inline]
    #[must_use]
    pub const fn from_affine(affine: Affine) -> Self {
        let [a, b, c, d, e, f] = affine.as_coeffs();
        Self::from_cols(
            [a, b, 0.0, 0.0],
            [c, d, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [e, f, 0.0, 1.0],
        )
    }

    /// Return whether this transform can be exactly represented as [`Affine`].
    ///
    /// This follows the CSS definition of a 2D matrix: the transform must not
    /// use z coordinates or perspective, and `m33` and `m44` must be exactly 1.
    #[inline]
    #[must_use]
    pub const fn is_2d(self) -> bool {
        let c = self.0;
        c[2][0] == 0.0
            && c[2][1] == 0.0
            && c[0][2] == 0.0
            && c[1][2] == 0.0
            && c[3][2] == 0.0
            && c[0][3] == 0.0
            && c[1][3] == 0.0
            && c[2][3] == 0.0
            && c[2][2] == 1.0
            && c[3][3] == 1.0
    }

    /// Return whether this transform is 3D affine rather than projective.
    ///
    /// A 3D affine transform has bottom row `[0, 0, 0, 1]` in conventional
    /// matrix notation. It can translate, rotate, scale, and shear in 3D, but
    /// it cannot express perspective.
    #[inline]
    #[must_use]
    pub const fn is_affine_3d(self) -> bool {
        let c = self.0;
        c[0][3] == 0.0 && c[1][3] == 0.0 && c[2][3] == 0.0 && c[3][3] == 1.0
    }

    /// Convert this transform to [`Affine`] if it is exactly a 2D transform.
    #[inline]
    #[must_use]
    pub const fn to_affine(self) -> Option<Affine> {
        if self.is_2d() {
            let c = self.0;
            Some(Affine::new([
                c[0][0], c[0][1], c[1][0], c[1][1], c[3][0], c[3][1],
            ]))
        } else {
            None
        }
    }

    /// Construct a transform representing a 3D translation.
    #[inline]
    #[must_use]
    pub const fn translate(x: f64, y: f64, z: f64) -> Self {
        Self::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [x, y, z, 1.0],
        )
    }

    /// Construct a transform representing a uniform 3D scale.
    #[inline]
    #[must_use]
    pub const fn scale(scale: f64) -> Self {
        Self::scale_non_uniform(scale, scale, scale)
    }

    /// Construct a transform representing a non-uniform 3D scale.
    #[inline]
    #[must_use]
    pub const fn scale_non_uniform(x: f64, y: f64, z: f64) -> Self {
        Self::from_cols(
            [x, 0.0, 0.0, 0.0],
            [0.0, y, 0.0, 0.0],
            [0.0, 0.0, z, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    }

    /// Construct a transform representing rotation around the x axis.
    ///
    /// The angle is expressed in radians.
    #[inline]
    #[must_use]
    pub fn rotate_x(radians: f64) -> Self {
        let (s, c) = radians.sin_cos();
        Self::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c, s, 0.0],
            [0.0, -s, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    }

    /// Construct a transform representing rotation around the y axis.
    ///
    /// The angle is expressed in radians.
    #[inline]
    #[must_use]
    pub fn rotate_y(radians: f64) -> Self {
        let (s, c) = radians.sin_cos();
        Self::from_cols(
            [c, 0.0, -s, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [s, 0.0, c, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    }

    /// Construct a transform representing rotation around the z axis.
    ///
    /// The convention matches [`Affine::rotate`]: a positive angle rotates the
    /// positive x direction into the positive y direction. The angle is
    /// expressed in radians.
    #[inline]
    #[must_use]
    pub fn rotate_z(radians: f64) -> Self {
        let (s, c) = radians.sin_cos();
        Self::from_cols(
            [c, s, 0.0, 0.0],
            [-s, c, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        )
    }

    /// Construct a CSS-style perspective transform.
    ///
    /// This sets `m34` to `-1 / depth`, so a point with positive z moves
    /// toward `w == 0` as z approaches `depth`. This matches CSS
    /// `perspective(depth)`.
    ///
    /// If `depth` is zero, the perspective coefficient is infinite.
    #[inline]
    #[must_use]
    pub fn perspective(depth: f64) -> Self {
        Self::from_cols(
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, -depth.recip()],
            [0.0, 0.0, 0.0, 1.0],
        )
    }

    /// Return a transform with a translation applied before `self`.
    #[inline]
    #[must_use]
    pub fn pre_translate(self, x: f64, y: f64, z: f64) -> Self {
        self * Self::translate(x, y, z)
    }

    /// Return a transform with a translation applied after `self`.
    #[inline]
    #[must_use]
    pub fn then_translate(self, x: f64, y: f64, z: f64) -> Self {
        Self::translate(x, y, z) * self
    }

    /// Return a transform with a uniform scale applied before `self`.
    #[inline]
    #[must_use]
    pub fn pre_scale(self, scale: f64) -> Self {
        self * Self::scale(scale)
    }

    /// Return a transform with a uniform scale applied after `self`.
    #[inline]
    #[must_use]
    pub fn then_scale(self, scale: f64) -> Self {
        Self::scale(scale) * self
    }

    /// Return a transform with a non-uniform scale applied before `self`.
    #[inline]
    #[must_use]
    pub fn pre_scale_non_uniform(self, x: f64, y: f64, z: f64) -> Self {
        self * Self::scale_non_uniform(x, y, z)
    }

    /// Return a transform with a non-uniform scale applied after `self`.
    #[inline]
    #[must_use]
    pub fn then_scale_non_uniform(self, x: f64, y: f64, z: f64) -> Self {
        Self::scale_non_uniform(x, y, z) * self
    }

    /// Return a transform with an x-axis rotation applied before `self`.
    #[inline]
    #[must_use]
    pub fn pre_rotate_x(self, radians: f64) -> Self {
        self * Self::rotate_x(radians)
    }

    /// Return a transform with an x-axis rotation applied after `self`.
    #[inline]
    #[must_use]
    pub fn then_rotate_x(self, radians: f64) -> Self {
        Self::rotate_x(radians) * self
    }

    /// Return a transform with a y-axis rotation applied before `self`.
    #[inline]
    #[must_use]
    pub fn pre_rotate_y(self, radians: f64) -> Self {
        self * Self::rotate_y(radians)
    }

    /// Return a transform with a y-axis rotation applied after `self`.
    #[inline]
    #[must_use]
    pub fn then_rotate_y(self, radians: f64) -> Self {
        Self::rotate_y(radians) * self
    }

    /// Return a transform with a z-axis rotation applied before `self`.
    #[inline]
    #[must_use]
    pub fn pre_rotate_z(self, radians: f64) -> Self {
        self * Self::rotate_z(radians)
    }

    /// Return a transform with a z-axis rotation applied after `self`.
    #[inline]
    #[must_use]
    pub fn then_rotate_z(self, radians: f64) -> Self {
        Self::rotate_z(radians) * self
    }

    /// Transform a 2D point on the z=0 plane into homogeneous 3D coordinates.
    ///
    /// The input is treated as `[x, y, 0, 1]`. The returned array is
    /// `[x, y, z, w]` after transformation, with no perspective division.
    #[inline]
    #[must_use]
    pub const fn transform_point2_homogeneous(self, point: Point) -> [f64; 4] {
        let c = self.0;
        [
            c[0][0] * point.x + c[1][0] * point.y + c[3][0],
            c[0][1] * point.x + c[1][1] * point.y + c[3][1],
            c[0][2] * point.x + c[1][2] * point.y + c[3][2],
            c[0][3] * point.x + c[1][3] * point.y + c[3][3],
        ]
    }

    /// Project a transformed 2D point back to 2D.
    ///
    /// The input is treated as `[x, y, 0, 1]`. This uses the CSS rendering
    /// convention for points: it divides by `w` only when `w > 0`, and returns
    /// `None` when the point is at or behind the viewer.
    #[inline]
    #[must_use]
    pub fn project_point2(self, point: Point) -> Option<Point> {
        let [x, y, _, w] = self.transform_point2_homogeneous(point);
        if w > 0.0 {
            Some(Point::new(x / w, y / w))
        } else {
            None
        }
    }

    /// Return whether this transform is the identity transform.
    #[inline]
    #[must_use]
    pub const fn is_identity(self) -> bool {
        let c = self.0;
        c[0][0] == 1.0
            && c[0][1] == 0.0
            && c[0][2] == 0.0
            && c[0][3] == 0.0
            && c[1][0] == 0.0
            && c[1][1] == 1.0
            && c[1][2] == 0.0
            && c[1][3] == 0.0
            && c[2][0] == 0.0
            && c[2][1] == 0.0
            && c[2][2] == 1.0
            && c[2][3] == 0.0
            && c[3][0] == 0.0
            && c[3][1] == 0.0
            && c[3][2] == 0.0
            && c[3][3] == 1.0
    }

    /// Return whether every coefficient is finite.
    ///
    /// See [`f64::is_finite`].
    #[inline]
    #[must_use]
    pub const fn is_finite(self) -> bool {
        let c = self.0;
        c[0][0].is_finite()
            && c[0][1].is_finite()
            && c[0][2].is_finite()
            && c[0][3].is_finite()
            && c[1][0].is_finite()
            && c[1][1].is_finite()
            && c[1][2].is_finite()
            && c[1][3].is_finite()
            && c[2][0].is_finite()
            && c[2][1].is_finite()
            && c[2][2].is_finite()
            && c[2][3].is_finite()
            && c[3][0].is_finite()
            && c[3][1].is_finite()
            && c[3][2].is_finite()
            && c[3][3].is_finite()
    }

    /// Return whether any coefficient is NaN.
    ///
    /// See [`f64::is_nan`].
    #[inline]
    #[must_use]
    pub const fn is_nan(self) -> bool {
        let c = self.0;
        c[0][0].is_nan()
            || c[0][1].is_nan()
            || c[0][2].is_nan()
            || c[0][3].is_nan()
            || c[1][0].is_nan()
            || c[1][1].is_nan()
            || c[1][2].is_nan()
            || c[1][3].is_nan()
            || c[2][0].is_nan()
            || c[2][1].is_nan()
            || c[2][2].is_nan()
            || c[2][3].is_nan()
            || c[3][0].is_nan()
            || c[3][1].is_nan()
            || c[3][2].is_nan()
            || c[3][3].is_nan()
    }
}

impl Default for Transform3d {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl From<Affine> for Transform3d {
    #[inline]
    fn from(affine: Affine) -> Self {
        Self::from_affine(affine)
    }
}

impl From<TranslateScale> for Transform3d {
    #[inline]
    fn from(ts: TranslateScale) -> Self {
        Self::from_affine(Affine::from(ts))
    }
}

impl Mul for Transform3d {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self {
        let a = self.0;
        let b = rhs.0;
        let mut out = [[0.0; 4]; 4];
        let mut col = 0;
        while col < 4 {
            let mut row = 0;
            while row < 4 {
                out[col][row] = a[0][row] * b[col][0]
                    + a[1][row] * b[col][1]
                    + a[2][row] * b[col][2]
                    + a[3][row] * b[col][3];
                row += 1;
            }
            col += 1;
        }
        Self(out)
    }
}

impl MulAssign for Transform3d {
    #[inline]
    fn mul_assign(&mut self, rhs: Self) {
        *self = self.mul(rhs);
    }
}

impl Mul<[f64; 4]> for Transform3d {
    type Output = [f64; 4];

    #[inline]
    fn mul(self, rhs: [f64; 4]) -> [f64; 4] {
        let c = self.0;
        [
            c[0][0] * rhs[0] + c[1][0] * rhs[1] + c[2][0] * rhs[2] + c[3][0] * rhs[3],
            c[0][1] * rhs[0] + c[1][1] * rhs[1] + c[2][1] * rhs[2] + c[3][1] * rhs[3],
            c[0][2] * rhs[0] + c[1][2] * rhs[1] + c[2][2] * rhs[2] + c[3][2] * rhs[3],
            c[0][3] * rhs[0] + c[1][3] * rhs[1] + c[2][3] * rhs[2] + c[3][3] * rhs[3],
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec2;

    fn assert_near(a: f64, b: f64) {
        assert!((a - b).abs() < 1e-9, "{a} != {b}");
    }

    fn point_assert_near(a: Point, b: Point) {
        assert_near(a.x, b.x);
        assert_near(a.y, b.y);
    }

    #[test]
    fn default_is_identity() {
        assert_eq!(Transform3d::default(), Transform3d::IDENTITY);
        assert!(Transform3d::IDENTITY.is_identity());
    }

    #[test]
    fn coeffs_are_css_matrix3d_order() {
        let coeffs = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ];
        let transform = Transform3d::new(coeffs);

        assert_eq!(transform.as_coeffs(), coeffs);
        assert_eq!(transform.col(0), [1.0, 2.0, 3.0, 4.0]);
        assert_eq!(transform.col(3), [13.0, 14.0, 15.0, 16.0]);
        assert_eq!(transform.row(0), [1.0, 5.0, 9.0, 13.0]);
        assert_eq!(transform.row(3), [4.0, 8.0, 12.0, 16.0]);
    }

    #[test]
    fn round_trip_cols_array_2d() {
        let transform = Transform3d::translate(5.0, 6.0, 7.0);
        let cols = transform.to_cols_array_2d();

        assert_eq!(Transform3d::from_cols_array_2d(cols), transform);
    }

    #[test]
    fn affine_embeds_as_xy_plane_transform() {
        let affine = Affine::new([2.0, 3.0, 5.0, 7.0, 11.0, 13.0]);
        let transform = Transform3d::from_affine(affine);

        assert_eq!(
            transform,
            Transform3d::from_cols(
                [2.0, 3.0, 0.0, 0.0],
                [5.0, 7.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [11.0, 13.0, 0.0, 1.0],
            )
        );
        assert_eq!(transform.to_affine(), Some(affine));
    }

    #[test]
    fn affine_maps_points_like_transform3d() {
        let affine = Affine::new([2.0, 3.0, 5.0, 7.0, 11.0, 13.0]);
        let point = Point::new(17.0, 19.0);

        let affine_point = affine * point;
        let projected = Transform3d::from_affine(affine)
            .project_point2(point)
            .unwrap();

        point_assert_near(projected, affine_point);
    }

    #[test]
    fn translate_scale_embeds_like_affine() {
        let ts = TranslateScale::new(Vec2::new(5.0, 6.0), 2.0);

        assert_eq!(
            Transform3d::from(ts),
            Transform3d::from_cols(
                [2.0, 0.0, 0.0, 0.0],
                [0.0, 2.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [5.0, 6.0, 0.0, 1.0],
            )
        );
    }

    #[test]
    fn multiplication_applies_rhs_first() {
        let scale = Transform3d::scale_non_uniform(2.0, 3.0, 1.0);
        let translate = Transform3d::translate(5.0, 7.0, 0.0);

        let combined = translate * scale;
        let point = combined.project_point2(Point::new(11.0, 13.0)).unwrap();

        point_assert_near(point, Point::new(27.0, 46.0));
    }

    #[test]
    fn pre_and_then_match_affine_order() {
        let point = Point::new(3.0, 4.0);

        let pre = Transform3d::IDENTITY
            .pre_translate(10.0, 0.0, 0.0)
            .pre_scale(2.0)
            .project_point2(point)
            .unwrap();
        let then = Transform3d::IDENTITY
            .then_translate(10.0, 0.0, 0.0)
            .then_scale(2.0)
            .project_point2(point)
            .unwrap();

        point_assert_near(pre, Point::new(16.0, 8.0));
        point_assert_near(then, Point::new(26.0, 8.0));
    }

    #[test]
    fn rotate_z_matches_affine_rotate() {
        let radians = core::f64::consts::FRAC_PI_2;
        let point = Point::new(3.0, 4.0);

        let affine_point = Affine::rotate(radians) * point;
        let transform_point = Transform3d::rotate_z(radians)
            .project_point2(point)
            .unwrap();

        point_assert_near(transform_point, affine_point);
    }

    #[test]
    fn x_and_y_rotations_move_z() {
        let rx = Transform3d::rotate_x(core::f64::consts::FRAC_PI_2);
        let ry = Transform3d::rotate_y(core::f64::consts::FRAC_PI_2);

        let x_result = rx * [0.0, 1.0, 0.0, 1.0];
        let y_result = ry * [1.0, 0.0, 0.0, 1.0];

        assert_near(x_result[0], 0.0);
        assert_near(x_result[1], 0.0);
        assert_near(x_result[2], 1.0);
        assert_near(y_result[0], 0.0);
        assert_near(y_result[1], 0.0);
        assert_near(y_result[2], -1.0);
    }

    #[test]
    fn perspective_uses_css_w_convention() {
        let in_front = Transform3d::perspective(50.0) * Transform3d::translate(0.0, 0.0, 25.0);
        let at_viewer = Transform3d::perspective(50.0) * Transform3d::translate(0.0, 0.0, 50.0);
        let behind = Transform3d::perspective(50.0) * Transform3d::translate(0.0, 0.0, 100.0);

        assert_eq!(
            in_front.transform_point2_homogeneous(Point::new(10.0, 20.0)),
            [10.0, 20.0, 25.0, 0.5]
        );
        point_assert_near(
            in_front.project_point2(Point::new(10.0, 20.0)).unwrap(),
            Point::new(20.0, 40.0),
        );
        assert_eq!(
            at_viewer.transform_point2_homogeneous(Point::new(10.0, 20.0))[3],
            0.0
        );
        assert!(at_viewer.project_point2(Point::new(10.0, 20.0)).is_none());
        assert_eq!(
            behind.transform_point2_homogeneous(Point::new(10.0, 20.0))[3],
            -1.0
        );
        assert!(behind.project_point2(Point::new(10.0, 20.0)).is_none());
    }

    #[test]
    fn dimensional_checks_are_exact() {
        assert!(Transform3d::IDENTITY.is_2d());
        assert!(Transform3d::rotate_z(0.5).is_2d());
        assert!(Transform3d::translate(1.0, 2.0, 3.0).is_affine_3d());

        assert!(!Transform3d::translate(1.0, 2.0, 3.0).is_2d());
        assert!(!Transform3d::rotate_x(0.5).is_2d());
        assert!(!Transform3d::perspective(100.0).is_2d());
        assert!(!Transform3d::perspective(100.0).is_affine_3d());
        assert_eq!(Transform3d::perspective(100.0).to_affine(), None);
    }

    #[test]
    fn finite_and_nan_checks() {
        let mut finite = Transform3d::IDENTITY;
        assert!(finite.is_finite());
        assert!(!finite.is_nan());

        finite.0[0][0] = f64::INFINITY;
        assert!(!finite.is_finite());
        assert!(!finite.is_nan());

        finite.0[0][0] = f64::NAN;
        assert!(!finite.is_finite());
        assert!(finite.is_nan());
    }
}
