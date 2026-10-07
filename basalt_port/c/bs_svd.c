/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* See bs_svd.h.  Eigen 3.4.0 JacobiSVD for a fixed 4x4 float matrix: no QR preconditioner (square), scale = max |A|, work = A / scale (true
 * division), cyclic sweeps over (p, q), p = 1..3, q = 0..p-1 with the threshold max(FLT_MIN, 2 eps maxDiagEntry), real 2x2 Jacobi SVD,
 * rotations applied to work (left, right) and V (right); then |diag| * scale and a selection sort (first maximum, strict >) of the singular
 * values with the V columns.  Every rotation is element-wise `c*x + s*y` / `-s*x + c*y` (no FMA, so packet and scalar paths agree). */
#include <float.h>
#include <math.h>
#include <string.h>
#include "bs_svd.h"

/* JacobiRotation<float>::makeJacobi(x, y, z) */
static void make_jacobi(float x, float y, float z, float* c, float* s) {
    const float deno = 2.0f * fabsf(y);
    float tau, w, t, sign_t, n;
    if (deno < FLT_MIN) { *c = 1.0f; *s = 0.0f; return; }
    tau = (x - z) / deno;
    w = sqrtf(tau * tau + 1.0f);
    if (tau > 0.0f) t = 1.0f / (tau + w);
    else t = 1.0f / (tau - w);
    sign_t = t > 0.0f ? 1.0f : -1.0f;
    n = 1.0f / sqrtf(t * t + 1.0f);
    *s = -sign_t * (y / fabsf(y)) * fabsf(t) * n;
    *c = n;
}

/* internal::real_2x2_jacobi_svd */
static void real_2x2_jacobi_svd(float m00, float m01, float m10, float m11, float* lc, float* ls, float* rc, float* rs) {
    const float t = m00 + m11;
    const float d = m10 - m01;
    float r1c, r1s, n00, n01, n11, c2, s2;
    if (fabsf(d) < FLT_MIN) { r1s = 0.0f; r1c = 1.0f; }
    else {
        const float u = t / d;
        const float tmp = sqrtf(1.0f + u * u);
        r1s = 1.0f / tmp;
        r1c = u / tmp;
    }
    /* m.applyOnTheLeft(0, 1, rot1) */
    n00 = r1c * m00 + r1s * m10;
    n01 = r1c * m01 + r1s * m11;
    n11 = -r1s * m01 + r1c * m11;
    make_jacobi(n00, n01, n11, &c2, &s2);
    /* j_left = rot1 * j_right.transpose() (j_right.transpose() = (c2, -s2)) */
    *lc = r1c * c2 - r1s * (-s2);
    *ls = r1c * (-s2) + r1s * c2;
    *rc = c2;
    *rs = s2;
}

int bs_jacobisvd4f(const float A[16], float V[16], float sv[4]) {
    const float precision = 2.0f * FLT_EPSILON;
    float W[16], scale = 0.0f, maxDiag = 0.0f;
    int i, p, q, finished;
    for (i = 0; i < 16; ++i) {
        const float a = fabsf(A[i]);
        if (a != a) { scale = a; break; }     /* maxCoeff<PropagateNaN> */
        if (a > scale) scale = a;
    }
    if (!isfinite(scale)) { memset(V, 0, 16 * sizeof(float)); memset(sv, 0, 4 * sizeof(float)); return 1; }
    if (scale == 0.0f) scale = 1.0f;
    for (i = 0; i < 16; ++i) W[i] = A[i] / scale;
    for (i = 0; i < 16; ++i) V[i] = (i % 5 == 0) ? 1.0f : 0.0f;
    for (i = 0; i < 4; ++i) { const float a = fabsf(W[i + 4 * i]); if (a > maxDiag) maxDiag = a; }
    finished = 0;
    while (!finished) {
        finished = 1;
        for (p = 1; p < 4; ++p)
            for (q = 0; q < p; ++q) {
                float threshold = precision * maxDiag;
                if (threshold < FLT_MIN) threshold = FLT_MIN;       /* maxi(considerAsZero, precision * maxDiagEntry) */
                if (fabsf(W[p + 4 * q]) > threshold || fabsf(W[q + 4 * p]) > threshold) {
                    float lc, ls, rc, rs, a1, a2;
                    finished = 0;
                    real_2x2_jacobi_svd(W[p + 4 * p], W[p + 4 * q], W[q + 4 * p], W[q + 4 * q], &lc, &ls, &rc, &rs);
                    for (i = 0; i < 4; ++i) {                      /* work.applyOnTheLeft(p, q, j_left): rows */
                        const float x = W[p + 4 * i], y = W[q + 4 * i];
                        W[p + 4 * i] = lc * x + ls * y;
                        W[q + 4 * i] = -ls * x + lc * y;
                    }
                    for (i = 0; i < 4; ++i) {                      /* work.applyOnTheRight(p, q, j_right): columns, rotation (c, -s) */
                        const float x = W[i + 4 * p], y = W[i + 4 * q];
                        W[i + 4 * p] = rc * x + (-rs) * y;
                        W[i + 4 * q] = -(-rs) * x + rc * y;
                    }
                    for (i = 0; i < 4; ++i) {                      /* V.applyOnTheRight(p, q, j_right) */
                        const float x = V[i + 4 * p], y = V[i + 4 * q];
                        V[i + 4 * p] = rc * x + (-rs) * y;
                        V[i + 4 * q] = -(-rs) * x + rc * y;
                    }
                    a1 = fabsf(W[p + 4 * p]); a2 = fabsf(W[q + 4 * q]);
                    if (a2 < a1) a2 = a1;                          /* maxi(abs(pp), abs(qq)) */
                    if (maxDiag < a2) maxDiag = a2;                /* maxi(maxDiagEntry, ...) */
                }
            }
    }
    for (i = 0; i < 4; ++i) sv[i] = fabsf(W[i + 4 * i]);
    for (i = 0; i < 4; ++i) sv[i] *= scale;
    for (i = 0; i < 4; ++i) {
        int pos = i, j;
        float best = sv[i];
        for (j = i + 1; j < 4; ++j) if (sv[j] > best) { best = sv[j]; pos = j; }
        if (best == 0.0f) break;
        if (pos != i) {
            float t = sv[i]; sv[i] = sv[pos]; sv[pos] = t;
            for (j = 0; j < 4; ++j) { t = V[j + 4 * i]; V[j + 4 * i] = V[j + 4 * pos]; V[j + 4 * pos] = t; }
        }
    }
    return 0;
}

void bs_triangulate_f(const float f0[3], const float f1[3], const bs_se3f* T_0_1, float out[4]) {
    float P1[12], P2[12], A[16], V[16], sv[4], wp[4], inv3[3], nrm;
    bs_se3f inv;
    int c, r;
    for (c = 0; c < 4; ++c) for (r = 0; r < 3; ++r) P1[r + 3 * c] = (r == c) ? 1.0f : 0.0f;       /* P1.setIdentity() */
    bs_se3f_inverse(T_0_1, &inv);
    bs_se3f_matrix3x4(&inv, P2);
    for (c = 0; c < 4; ++c) {
        A[0 + 4 * c] = f0[0] * P1[2 + 3 * c] - f0[2] * P1[0 + 3 * c];
        A[1 + 4 * c] = f0[1] * P1[2 + 3 * c] - f0[2] * P1[1 + 3 * c];
        A[2 + 4 * c] = f1[0] * P2[2 + 3 * c] - f1[2] * P2[0 + 3 * c];
        A[3 + 4 * c] = f1[1] * P2[2 + 3 * c] - f1[2] * P2[1 + 3 * c];
    }
    bs_jacobisvd4f(A, V, sv);
    for (r = 0; r < 4; ++r) wp[r] = V[r + 4 * 3];
    inv3[0] = wp[0]; inv3[1] = wp[1]; inv3[2] = wp[2];
    nrm = sqrtf(bs_v3f_sqn(inv3));                               /* worldPoint.head<3>().norm() */
    for (r = 0; r < 4; ++r) wp[r] = wp[r] / nrm;                 /* worldPoint /= norm */
    inv3[0] = wp[0]; inv3[1] = wp[1]; inv3[2] = wp[2];
    if (bs_v3f_dot(f0, inv3) < 0.0f) for (r = 0; r < 4; ++r) wp[r] = wp[r] * -1.0f;
    for (r = 0; r < 4; ++r) out[r] = wp[r];
}

void bs_stereographic_project_f(const float p3d[4], float res[2]) {
    const float sq = sqrtf(bs_v3f_sqn(p3d));
    const float norm = p3d[2] + sq;
    const float norm_inv = 1.0f / norm;
    res[0] = p3d[0] * norm_inv;
    res[1] = p3d[1] * norm_inv;
}
