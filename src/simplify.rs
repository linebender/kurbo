//! Simplification of a Bézier path

use crate::CubicBez;

/// Compute moment integrals.
pub fn moment_integrals(c: CubicBez) -> (f64, f64, f64) {
    let (x0, y0) = (c.p0.x, c.p0.y);
    let (x1, y1) = (c.p1.x, c.p1.y);
    let (x2, y2) = (c.p2.x, c.p2.y);
    let (x3, y3) = (c.p3.x, c.p3.y);
    let r0 = x0 * y3;
    let r1 = x0 * y0;
    let r2 = x0 * y1;
    let r3 = 3. * y2;
    let r4 = r3 * x0;
    let r5 = x1 * y3;
    let r6 = 3. * r5;
    let r7 = x2 * y3;
    let r8 = 45. * x1;
    let r9 = 45. * x0;
    let r10 = 15. * r0;
    let r11 = 18. * x0;
    let r12 = 12. * r0;
    let r13 = 27. * y2;
    let r14 = x0.powi(2);
    let r15 = 105. * y1;
    let r16 = 30. * y2;
    let r17 = x1.powi(2);
    let r18 = x2.powi(2);
    let r19 = x3.powi(2);
    let r20 = 45. * y2;
    let r21 = y0.powi(2);
    let r22 = y1.powi(2);
    let r23 = y2.powi(2);
    let r24 = y3.powi(2);

    let a = -r0 - 10. * r1 - 6. * r2 - r3 * x1 - r4 - r6 - 6. * r7
        + 6. * x1 * y0
        + 3. * x2 * y0
        + 3. * x2 * y1
        + x3 * y0
        + 3. * x3 * y1
        + 6. * x3 * y2
        + 10. * x3 * y3;
    let x = -5. * r0 * x3
        - r10 * x1
        - r11 * x2 * y2
        - r12 * x2
        - r13 * r17
        - r13 * x1 * x2
        - r14 * r15
        - r14 * r16
        - 280. * r14 * y0
        - 5. * r14 * y3
        + 45. * r17 * y0
        - 18. * r17 * y3
        + 18. * r18 * y0
        + 27. * r18 * y1
        - 45. * r18 * y3
        + 5. * r19 * y0
        + 30. * r19 * y1
        + 105. * r19 * y2
        + 280. * r19 * y3
        - r2 * r8
        - r4 * x3
        - 30. * r5 * x3
        - r7 * r8
        - 105. * r7 * x3
        - r9 * x1 * y2
        + 105. * x0 * x1 * y0
        + 30. * x0 * x2 * y0
        + 5. * x0 * x3 * y0
        + 3. * x0 * x3 * y1
        + 45. * x1 * x2 * y0
        + 27. * x1 * x2 * y1
        + 12. * x1 * x3 * y0
        + 15. * x2 * x3 * y0
        + 18. * x1 * x3 * y1
        + 45. * x2 * x3 * y1
        + 45. * x2 * x3 * y2;
    let y = -5. * r0 * y0
        - r1 * r15
        - r1 * r16
        - r10 * y2
        - r11 * r23
        - r12 * y1
        - r13 * x1 * y1
        - r2 * r20
        - r20 * r5
        - r20 * r7
        - 140. * r21 * x0
        + 105. * r21 * x1
        + 30. * r21 * x2
        + 5. * r21 * x3
        - r22 * r9
        + 27. * r22 * x2
        + 18. * r22 * x3
        - 27. * r23 * x1
        + 45. * r23 * x3
        - 5. * r24 * x0
        - 30. * r24 * x1
        - 105. * r24 * x2
        + 140. * r24 * x3
        - 18. * r5 * y1
        - r6 * y0
        + 45. * x1 * y0 * y1
        + 45. * x2 * y0 * y1
        + 18. * x2 * y0 * y2
        + 3. * x2 * y0 * y3
        + 27. * x2 * y1 * y2
        + 15. * x3 * y0 * y1
        + 12. * x3 * y0 * y2
        + 5. * x3 * y0 * y3
        + 45. * x3 * y1 * y2
        + 30. * x3 * y1 * y3
        + 105. * x3 * y2 * y3;
    (a * (1. / 20.), x * (1. / 840.), y * (1. / 420.))
}

/// Compute moment integrals.
pub fn moment_integrals2(c: CubicBez) -> (f64, f64, f64) {
    let (x0, y0) = (c.p0.x, c.p0.y);
    let (x1, y1) = (c.p1.x - x0, c.p1.y - y0);
    let (x2, y2) = (c.p2.x - x0, c.p2.y - y0);
    let (x3, y3) = (c.p3.x - x0, c.p3.y - y0);

    let r0 = 3. * x1;
    let r1 = 3. * y1;
    let r2 = x2 * y3;
    let r3 = x3 * y2;
    let r4 = x3 * y3;
    let r5 = 27. * y1;
    let r6 = x1 * x2;
    let r7 = 27. * y2;
    let r8 = 45. * r2;
    let r9 = 18. * x3;
    let r10 = x1 * y1;
    let r11 = 30. * x1;
    let r12 = 45. * x3;
    let r13 = x2 * y1;
    let r14 = 45. * r3;
    let r15 = x1.powi(2);
    let r16 = 18. * y3;
    let r17 = x2.powi(2);
    let r18 = 45. * y3;
    let r19 = x3.powi(2);
    let r20 = 30. * y1;
    let r21 = y2.powi(2);
    let r22 = y3.powi(2);
    let r23 = y1.powi(2);
    let a = -r0 * y2 - r0 * y3 + r1 * x2 + r1 * x3 - 6. * r2 + 6. * r3 + 10. * r4;

    // Scale and add chord
    let lift = x3 * y0;
    let area = a * 0.05 + lift;
    let x = r10 * r9 - r11 * r4 + r12 * r13 + r14 * x2 - r15 * r16 - r15 * r7 - r17 * r18
        + r17 * r5
        + r19 * r20
        + 105. * r19 * y2
        + 280. * r19 * y3
        - 105. * r2 * x3
        + r5 * r6
        - r6 * r7
        - r8 * x1;
    let y = -r10 * r16 - r10 * r7 - r11 * r22 + r12 * r21 + r13 * r7 + r14 * y1 - r18 * x1 * y2
        + r20 * r4
        - 27. * r21 * x1
        - 105. * r22 * x2
        + 140. * r22 * x3
        + r23 * r9
        + 27. * r23 * x2
        + 105. * r3 * y3
        - r8 * y2;

    let mx = x * (1. / 840.) + x0 * area + 0.5 * x3 * lift;
    let my = y * (1. / 420.) + y0 * a * 0.1 + x0 * lift;

    (area, mx, my)
}
