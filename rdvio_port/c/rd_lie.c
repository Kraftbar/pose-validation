/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/* See rd_lie.h for provenance and licences. */
#include "rd_lie.h"
#include <float.h>
#include <math.h>

void rd_hat(const double w[3], double out[9]) {
    /* (matrix<3>() << 0, -z, y, z, 0, -x, -y, x, 0) row by row, stored column-major */
    out[0] = 0.0;   out[3] = -w[2]; out[6] = w[1];
    out[1] = w[2];  out[4] = 0.0;   out[7] = -w[0];
    out[2] = -w[1]; out[5] = w[0];  out[8] = 0.0;
}

ok_quat rd_quat_conj(ok_quat q) { ok_quat r; r.x = -q.x; r.y = -q.y; r.z = -q.z; r.w = q.w; return r; }

/* Eigen cross() for double 3-vectors (no SSE specialisation for double) */
static void cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

void rd_quat_rotate(const ok_quat* q, const double v[3], double out[3]) {
    const double vec[3] = {q->x, q->y, q->z};
    double uv[3], c[3];
    cross3(vec, v, uv);
    uv[0] += uv[0]; uv[1] += uv[1]; uv[2] += uv[2];
    cross3(vec, uv, c);
    /* v + w*uv + vec.cross(uv): left to right */
    out[0] = (v[0] + q->w * uv[0]) + c[0];
    out[1] = (v[1] + q->w * uv[1]) + c[1];
    out[2] = (v[2] + q->w * uv[2]) + c[2];
}

ok_quat rd_expmap(const double w[3]) {
    const double angle = ok_v3_norm(w);
    /* w.stableNormalized(): n / (sqrt(|n/m|^2) * m), m = max |w_i|; unchanged if that squared norm is not > 0 */
    double m = fabs(w[0]);
    if (fabs(w[1]) > m) m = fabs(w[1]);
    if (fabs(w[2]) > m) m = fabs(w[2]);
    double axis[3] = {w[0], w[1], w[2]};
    {
        const double a = w[0] / m, b = w[1] / m, c = w[2] / m;
        const double z = (a * a + b * b) + c * c;
        if (z > 0.0) {
            const double d = sqrt(z) * m;
            axis[0] = w[0] / d; axis[1] = w[1] / d; axis[2] = w[2] / d;
        }
    }
    /* Quaterniond = AngleAxisd */
    const double ha = 0.5 * angle;
    ok_quat q;
    q.w = cos(ha);
    const double s = sin(ha);
    q.x = s * axis[0]; q.y = s * axis[1]; q.z = s * axis[2];
    return q;
}

void rd_logmap(const ok_quat* q, double out[3]) {
    const double vec[3] = {q->x, q->y, q->z};
    double n = ok_v3_norm(vec);
    double angle, axis[3];
    /* n < eps would use stableNorm(); not reachable with a normalised quaternion except for exact zero (see HANDOVER) */
    if (n != 0.0) {
        angle = 2.0 * atan2(n, fabs(q->w));
        if (q->w < 0.0) n = -n;
        axis[0] = vec[0] / n; axis[1] = vec[1] / n; axis[2] = vec[2] / n;
    } else {
        angle = 0.0; axis[0] = 1.0; axis[1] = 0.0; axis[2] = 0.0;
    }
    out[0] = angle * axis[0]; out[1] = angle * axis[1]; out[2] = angle * axis[2];
}

void rd_right_jacobian(const double w[3], double out[9]) {
    const double eps = DBL_EPSILON;
    const double root2_eps = sqrt(eps);
    const double root4_eps = sqrt(root2_eps);
    const double qdrt720 = sqrt(sqrt(720.0));
    const double qdrt5040 = sqrt(sqrt(5040.0));
    const double sqrt24 = sqrt(24.0);
    const double sqrt120 = sqrt(120.0);
    const double angle = ok_v3_norm(w);
    const double cangle = cos(angle);
    const double sangle = sin(angle);
    const double angle2 = angle * angle;
    double cos_term, sin_term;
    if (angle > root4_eps * qdrt720) {
        cos_term = (1 - cangle) / angle2;
    } else {
        cos_term = 0.5;
        if (angle > root2_eps * sqrt24) cos_term -= angle2 / 24.0;
    }
    if (angle > root4_eps * qdrt5040) {
        sin_term = (angle - sangle) / (angle * angle2);
    } else {
        sin_term = 1.0 / 6.0;
        if (angle > root2_eps * sqrt120) sin_term -= angle2 / 120.0;
    }
    double H[9], sH[9], HH[9];
    int i;
    rd_hat(w, H);
    for (i = 0; i < 9; ++i) sH[i] = sin_term * H[i];
    ok_m3_mul(sH, H, HH);                     /* (sin_term * hat_w) * hat_w, lazy 3x3 product */
    for (i = 0; i < 9; ++i) {
        const double id = (i % 4 == 0) ? 1.0 : 0.0;
        out[i] = (id - cos_term * H[i]) + HH[i];
    }
}

void rd_s2_tangential_basis(const double x[3], double out[6]) {
    int d = 0, i;
    double e[3] = {0, 0, 0}, b1[3], b2[3];
    for (i = 1; i < 3; ++i)
        if (fabs(x[i]) > fabs(x[d])) d = i;
    e[(d + 1) % 3] = 1.0;
    cross3(x, e, b1);
    ok_v3_normalized(b1, b1);                 /* normalized(): z > 0 ? v / sqrt(z) : v */
    cross3(x, b1, b2);
    ok_v3_normalized(b2, b2);
    for (i = 0; i < 3; ++i) { out[i] = b1[i]; out[3 + i] = b2[i]; }
}

void rd_s2_tangential_basis_barrel(const double x[3], double out[6]) {
    double e[3] = {0, 0, 0}, b1[3], b2[3];
    int i;
    if (fabs(x[2]) < 0.866) e[2] = 1.0; else e[1] = 1.0;
    cross3(x, e, b1);
    ok_v3_normalized(b1, b1);
    cross3(x, b1, b2);
    ok_v3_normalized(b2, b2);
    for (i = 0; i < 3; ++i) { out[i] = b1[i]; out[3 + i] = b2[i]; }
}

#define M(i, j) m[(i) + 3 * (j)]
static double cofactor3(const double* m, int i, int j) {
    const int i1 = (i + 1) % 3, i2 = (i + 2) % 3, j1 = (j + 1) % 3, j2 = (j + 2) % 3;
    return M(i1, j1) * M(i2, j2) - M(i1, j2) * M(i2, j1);
}
void rd_inverse3(const double m[9], double out[9]) {
    double c0[3], r[9];
    c0[0] = cofactor3(m, 0, 0); c0[1] = cofactor3(m, 1, 0); c0[2] = cofactor3(m, 2, 0);
    /* (c0 .* m.col(0)).sum() */
    const double det = (c0[0] * M(0, 0) + c0[1] * M(1, 0)) + c0[2] * M(2, 0);
    const double invdet = 1.0 / det;
    {
        const double c01 = cofactor3(m, 0, 1) * invdet;
        const double c11 = cofactor3(m, 1, 1) * invdet;
        const double c02 = cofactor3(m, 0, 2) * invdet;
        r[1 + 3 * 2] = cofactor3(m, 2, 1) * invdet;
        r[2 + 3 * 1] = cofactor3(m, 1, 2) * invdet;
        r[2 + 3 * 2] = cofactor3(m, 2, 2) * invdet;
        r[1 + 3 * 0] = c01;
        r[1 + 3 * 1] = c11;
        r[2 + 3 * 0] = c02;
        r[0 + 3 * 0] = c0[0] * invdet; r[0 + 3 * 1] = c0[1] * invdet; r[0 + 3 * 2] = c0[2] * invdet;
    }
    {
        int i;
        for (i = 0; i < 9; ++i) out[i] = r[i];
    }
}
#undef M
