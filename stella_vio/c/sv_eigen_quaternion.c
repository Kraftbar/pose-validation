/* SPDX-License-Identifier: MPL-2.0 */
/* See sv_eigen_quaternion.h (MPL-2.0, Eigen-derived formulas). */
#include "sv_eigen_quaternion.h"
#include <math.h>

#define M(i, j) m[(j) * 3 + (i)]
#define O(i, j) out[(j) * 3 + (i)]

/* Scalar unrolling of Eigen's SSE2/ARM64
 * quat_product<Architecture::Target,...,double>::run (Geometry_SIMD.h).
 * b_xy=[bx,by], b_zw=[bz,bw]; a_xx/a_yy/a_zz/a_ww broadcast a's lanes.
 *   t1 = a_ww*b_xy + a_yy*b_zw          t2 = a_zz*b_xy - a_xx*b_zw
 *   res.xy = paddsub(t1, preverse(t2))  -> res.x = t1.x - t2.y, res.y = t1.y + t2.x
 *   t1 = a_ww*b_zw - a_yy*b_xy          t2 = a_zz*b_zw + a_xx*b_xy
 *   res.zw = preverse(paddsub(preverse(t1), t2))
 *          -> res.z = t1.y + t2.x (pre-swap t1), res.w = t1.x - t2.y (pre-swap t1)
 * Unrolled to scalar (each term itself a single pmul, so no further
 * per-lane packing ambiguity): */
void sv_quat_mul(const sv_quat* a, const sv_quat* b, sv_quat* out) {
    const double ax = a->x, ay = a->y, az = a->z, aw = a->w;
    const double bx = b->x, by = b->y, bz = b->z, bw = b->w;

    const double t1x = aw * bx + ay * bz;
    const double t1y = aw * by + ay * bw;
    const double t2x = az * bx - ax * bz;
    const double t2y = az * by - ax * bw;
    const double rx = t1x - t2y;
    const double ry = t1y + t2x;

    const double u1x = aw * bz - ay * bx;
    const double u1y = aw * bw - ay * by;
    const double u2x = az * bz + ax * bx;
    const double u2y = az * bw + ax * by;
    const double rz = u1x + u2y;
    const double rw = u1y - u2x;

    out->x = rx;
    out->y = ry;
    out->z = rz;
    out->w = rw;
}

/* QuaternionBase::_transformVector:
 *   uv = 2 * vec().cross(v)
 *   result = v + w*uv + vec().cross(uv) */
void sv_quat_map(const sv_quat* q, const double v[3], double out[3]) {
    const double qx = q->x, qy = q->y, qz = q->z, qw = q->w;

    double uv[3];
    uv[0] = qy * v[2] - qz * v[1];
    uv[1] = qz * v[0] - qx * v[2];
    uv[2] = qx * v[1] - qy * v[0];
    uv[0] += uv[0];
    uv[1] += uv[1];
    uv[2] += uv[2];

    double cross2[3];
    cross2[0] = qy * uv[2] - qz * uv[1];
    cross2[1] = qz * uv[0] - qx * uv[2];
    cross2[2] = qx * uv[1] - qy * uv[0];

    out[0] = v[0] + qw * uv[0] + cross2[0];
    out[1] = v[1] + qw * uv[1] + cross2[1];
    out[2] = v[2] + qw * uv[2] + cross2[2];
}

void sv_quat_to_mat3(const sv_quat* q, double out[9]) {
    const double x = q->x, y = q->y, z = q->z, w = q->w;
    const double tx = 2.0 * x, ty = 2.0 * y, tz = 2.0 * z;
    const double twx = tx * w, twy = ty * w, twz = tz * w;
    const double txx = tx * x, txy = ty * x, txz = tz * x;
    const double tyy = ty * y, tyz = tz * y, tzz = tz * z;

    O(0, 0) = 1.0 - (tyy + tzz);
    O(0, 1) = txy - twz;
    O(0, 2) = txz + twy;
    O(1, 0) = txy + twz;
    O(1, 1) = 1.0 - (txx + tzz);
    O(1, 2) = tyz - twx;
    O(2, 0) = txz - twy;
    O(2, 1) = tyz + twx;
    O(2, 2) = 1.0 - (txx + tyy);
}

void sv_quat_normalize(sv_quat* q) {
    const double a0 = q->x * q->x;
    const double a1 = q->y * q->y;
    const double a2 = q->z * q->z;
    const double a3 = q->w * q->w;
    const double sq = (a0 + a2) + (a1 + a3);
    const double n = sqrt(sq);
    q->x /= n;
    q->y /= n;
    q->z /= n;
    q->w /= n;
}

/* internal::quaternionbase_assign_impl<Other,3,3>::run (Shoemake 1987). */
void sv_quat_from_mat3(const double m[9], sv_quat* out) {
    /* mat.trace(): diagonal().sum() over 3 scalars -- right-associative
     * (matches the "row 2 remainder" reduction rule in sv_linalg.c). */
    double t = M(0, 0) + (M(1, 1) + M(2, 2));

    if (t > 0.0) {
        t = sqrt(t + 1.0);
        out->w = 0.5 * t;
        t = 0.5 / t;
        out->x = (M(2, 1) - M(1, 2)) * t;
        out->y = (M(0, 2) - M(2, 0)) * t;
        out->z = (M(1, 0) - M(0, 1)) * t;
    } else {
        int i = 0;
        if (M(1, 1) > M(0, 0)) {
            i = 1;
        }
        if (M(2, 2) > M(i, i)) {
            i = 2;
        }
        int j = (i + 1) % 3;
        int k = (j + 1) % 3;

        double coeffs[4]; /* indexed like Eigen's coeffs(): [x, y, z, w] */
        t = sqrt(M(i, i) - M(j, j) - M(k, k) + 1.0);
        coeffs[i] = 0.5 * t;
        t = 0.5 / t;
        coeffs[3] = (M(k, j) - M(j, k)) * t;
        coeffs[j] = (M(j, i) + M(i, j)) * t;
        coeffs[k] = (M(k, i) + M(i, k)) * t;

        out->x = coeffs[0];
        out->y = coeffs[1];
        out->z = coeffs[2];
        out->w = coeffs[3];
    }
}
