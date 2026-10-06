/* SPDX-License-Identifier: MPL-2.0 */
/* RD-VIO pure-C port: Eigen 3.4.0 models for the initializer. See rd_sys_eigen.h. */
#include "rd_sys_eigen.h"
#include "../../stella_port/c/sv_eigen_svd.h"
#include <float.h>
#include <math.h>
#include <string.h>

int rd_svd3_solve(const double A[9], const double b[3], double x[3]) {
    double U[9], V[9], sv[3], t[3], thr;
    int nonzero = 0, r, i, k;
    sv_eigen_jacobisvd_3x3(A, U, V, sv);
    while (nonzero < 3 && sv[nonzero] != 0.0) nonzero++;   /* m_nonzeroSingularValues */
    thr = sv[0] * (3.0 * DBL_EPSILON);
    if (thr < DBL_MIN) thr = DBL_MIN;                      /* numext::maxi(s0 * threshold(), min()) */
    r = nonzero;
    while (r > 0 && sv[r - 1] < thr) --r;
    for (i = 0; i < r; ++i) {                              /* tmp = U.leftCols(r).adjoint() * b */
        double s = U[3 * i] * b[0];
        s += U[3 * i + 1] * b[1];
        s += U[3 * i + 2] * b[2];
        t[i] = s;
    }
    for (i = 0; i < r; ++i) t[i] = (1.0 / sv[i]) * t[i];  /* asDiagonal().inverse() * tmp */
    for (k = 0; k < 3; ++k) {                              /* x = V.leftCols(r) * tmp */
        double s = 0.0;
        if (r > 0) { s = V[k] * t[0]; for (i = 1; i < r; ++i) s += V[k + 3 * i] * t[i]; }
        x[k] = s;
    }
    return r;
}

int rd_quat_from_two_vectors(const double a[3], const double b[3], ok_quat* q) {
    double v0[3], v1[3], c, axis[3], s, invs;
    v0[0] = a[0]; v0[1] = a[1]; v0[2] = a[2]; ok_v3_normalized(a, v0);
    v1[0] = b[0]; v1[1] = b[1]; v1[2] = b[2]; ok_v3_normalized(b, v1);
    c = (v1[0] * v0[0] + v1[1] * v0[1]) + v1[2] * v0[2];
    if (c < -1.0 + 1e-12) return 0;                         /* dummy_precision<double> */
    axis[0] = v0[1] * v1[2] - v0[2] * v1[1];
    axis[1] = v0[2] * v1[0] - v0[0] * v1[2];
    axis[2] = v0[0] * v1[1] - v0[1] * v1[0];
    s = sqrt((1.0 + c) * 2.0);
    invs = 1.0 / s;
    q->x = axis[0] * invs; q->y = axis[1] * invs; q->z = axis[2] * invs;
    q->w = s * 0.5;
    return 1;
}

/* ---- Matrix4d::inverse (SSE2 Packet2d model) ---- */
typedef struct p2 { double a, b; } p2;
static p2 ld(const double* d) { p2 r; r.a = d[0]; r.b = d[1]; return r; }
static double ln(p2 v, int i) { return i ? v.b : v.a; }
static p2 swz(p2 x, p2 y, int mask) { p2 r; r.a = ln(x, mask & 1); r.b = ln(y, (mask >> 1) & 1); return r; }   /* _mm_shuffle_pd */
static p2 dup(p2 x, int i) { return swz(x, x, (i << 1) | i); }
static p2 mul(p2 x, p2 y) { p2 r; r.a = x.a * y.a; r.b = x.b * y.b; return r; }
static p2 sub(p2 x, p2 y) { p2 r; r.a = x.a - y.a; r.b = x.b - y.b; return r; }
static p2 add(p2 x, p2 y) { p2 r; r.a = x.a + y.a; r.b = x.b + y.b; return r; }
static void st(double* d, p2 v) { d[0] = v.a; d[1] = v.b; }
void rd_m4_inverse(const double A[16], double out[16]) {
    const p2 A1 = ld(A + 0), B1 = ld(A + 2), A2 = ld(A + 4), B2 = ld(A + 6), C1 = ld(A + 8), D1 = ld(A + 10), C2 = ld(A + 12), D2 = ld(A + 14);
    p2 dA, dB, dC, dD, DC1, DC2, AB1, AB2, d1, d2, det, rd, one;
    p2 iA1, iA2, iB1, iB2, iC1, iC2, iD1, iD2;
    dA = mul(A1, swz(A2, A2, 1)); dA = sub(dA, dup(dA, 1));
    dB = mul(B1, swz(B2, B2, 1)); dB = sub(dB, dup(dB, 1));
    dC = mul(C1, swz(C2, C2, 1)); dC = sub(dC, dup(dC, 1));
    dD = mul(D1, swz(D2, D2, 1)); dD = sub(dD, dup(dD, 1));
    AB1 = mul(B1, dup(A2, 1)); AB2 = mul(B2, dup(A1, 0));
    AB1 = sub(AB1, mul(B2, dup(A1, 1))); AB2 = sub(AB2, mul(B1, dup(A2, 0)));
    DC1 = mul(C1, dup(D2, 1)); DC2 = mul(C2, dup(D1, 0));
    DC1 = sub(DC1, mul(C2, dup(D1, 1))); DC2 = sub(DC2, mul(C1, dup(D2, 0)));
    d1 = mul(AB1, swz(DC1, DC2, 0)); d2 = mul(AB2, swz(DC1, DC2, 3));
    rd = add(d1, d2); rd = add(rd, dup(rd, 1));
    d1 = mul(dA, dD); d2 = mul(dB, dC);
    det = add(d1, d2); det = sub(det, rd); det = dup(det, 0);
    one.a = 1.0; one.b = 1.0;
    rd.a = one.a / det.a; rd.b = one.b / det.b;
    iD1 = mul(AB1, dup(C1, 0)); iD2 = mul(AB1, dup(C2, 0));
    iD1 = add(iD1, mul(AB2, dup(C1, 1))); iD2 = add(iD2, mul(AB2, dup(C2, 1)));
    dA = dup(dA, 0);
    iD1 = sub(mul(D1, dA), iD1); iD2 = sub(mul(D2, dA), iD2);
    iA1 = mul(DC1, dup(B1, 0)); iA2 = mul(DC1, dup(B2, 0));
    iA1 = add(iA1, mul(DC2, dup(B1, 1))); iA2 = add(iA2, mul(DC2, dup(B2, 1)));
    dD = dup(dD, 0);
    iA1 = sub(mul(A1, dD), iA1); iA2 = sub(mul(A2, dD), iA2);
    iB1 = mul(D1, swz(AB2, AB1, 1)); iB2 = mul(D2, swz(AB2, AB1, 1));
    iB1 = sub(iB1, mul(swz(D1, D1, 1), swz(AB2, AB1, 2))); iB2 = sub(iB2, mul(swz(D2, D2, 1), swz(AB2, AB1, 2)));
    dB = dup(dB, 0);
    iB1 = sub(mul(C1, dB), iB1); iB2 = sub(mul(C2, dB), iB2);
    iC1 = mul(A1, swz(DC2, DC1, 1)); iC2 = mul(A2, swz(DC2, DC1, 1));
    iC1 = sub(iC1, mul(swz(A1, A1, 1), swz(DC2, DC1, 2))); iC2 = sub(iC2, mul(swz(A2, A2, 1), swz(DC2, DC1, 2)));
    dC = dup(dC, 0);
    iC1 = sub(mul(B1, dC), iC1); iC2 = sub(mul(B2, dC), iC2);
    d1 = rd; d1.b = -d1.b;                          /* pxor with (0, -0): sign of lane 1 */
    d2 = rd; d2.a = -d2.a;                          /* pxor with (-0, 0): sign of lane 0 */
    st(out + 0, mul(swz(iA2, iA1, 3), d1)); st(out + 4, mul(swz(iA2, iA1, 0), d2));
    st(out + 2, mul(swz(iB2, iB1, 3), d1)); st(out + 6, mul(swz(iB2, iB1, 0), d2));
    st(out + 8, mul(swz(iC2, iC1, 3), d1)); st(out + 12, mul(swz(iC2, iC1, 0), d2));
    st(out + 10, mul(swz(iD2, iD1, 3), d1)); st(out + 14, mul(swz(iD2, iD1, 0), d2));
}

void rd_m4_mul(const double A[16], const double B[16], double out[16]) {
    double r[16];
    int i, j;
    for (j = 0; j < 4; ++j)
        for (i = 0; i < 4; ++i) r[i + 4 * j] = ((A[i] * B[4 * j] + A[i + 4] * B[1 + 4 * j]) + A[i + 8] * B[2 + 4 * j]) + A[i + 12] * B[3 + 4 * j];
    memcpy(out, r, sizeof r);
}

#define T3(i, j) mt[(i) + 3 * (j)]
static double cof3(const double* mt, int i, int j) {   /* cofactor_3x3<i, j> */
    const int i1 = (i + 1) % 3, i2 = (i + 2) % 3, j1 = (j + 1) % 3, j2 = (j + 2) % 3;
    return T3(i1, j1) * T3(i2, j2) - T3(i1, j2) * T3(i2, j1);
}
void rd_inverse3_t(const double m[9], double out[9]) {
    double mt[9], c0[3], r[9], det, invdet;
    int i, j;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) mt[i + 3 * j] = m[j + 3 * i];
    c0[0] = cof3(mt, 0, 0); c0[1] = cof3(mt, 1, 0); c0[2] = cof3(mt, 2, 0);
    det = c0[0] * T3(0, 0) + (c0[1] * T3(1, 0) + c0[2] * T3(2, 0));
    invdet = 1.0 / det;
    r[1 + 3 * 0] = cof3(mt, 0, 1) * invdet; r[1 + 3 * 1] = cof3(mt, 1, 1) * invdet; r[2 + 3 * 0] = cof3(mt, 0, 2) * invdet;
    r[1 + 3 * 2] = cof3(mt, 2, 1) * invdet; r[2 + 3 * 1] = cof3(mt, 1, 2) * invdet; r[2 + 3 * 2] = cof3(mt, 2, 2) * invdet;
    r[0] = c0[0] * invdet; r[3] = c0[1] * invdet; r[6] = c0[2] * invdet;
    memcpy(out, r, sizeof r);
}
#undef T3
void rd_m3_mul_tinv(const double a[9], const double b[9], double out[9]) {
    double r[9];
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) r[i + 3 * j] = (a[i] * b[3 * j] + a[i + 3] * b[1 + 3 * j]) + a[i + 6] * b[2 + 3 * j];
    memcpy(out, r, sizeof r);
}

double rd_epipolar_dist(const double F[9], const double p[2], const double q[2]) {
    double l[3], num;
    int i;
    for (i = 0; i < 3; ++i) l[i] = (F[i] * p[0] + F[i + 3] * p[1]) + F[i + 6];
    num = (q[0] * l[0] + q[1] * l[1]) + 1.0 * l[2];
    return fabs(num) / sqrt(l[0] * l[0] + l[1] * l[1]);
}
