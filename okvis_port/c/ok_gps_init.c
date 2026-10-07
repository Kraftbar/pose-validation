/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2-X GNSS initialisation helpers, see ok_gps_init.h for provenance and conventions. */
#include "ok_gps_init.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI_ 3.14159265358979323846 /* M_PI */

/* ---- std::mt19937 + libstdc++ uniform_int_distribution<int> ---- */
void ok_mt19937_seed(ok_mt19937* g, unsigned int seed) {
    int i;
    g->mt[0] = seed;
    for (i = 1; i < 624; ++i) g->mt[i] = 1812433253u * (g->mt[i - 1] ^ (g->mt[i - 1] >> 30)) + (unsigned int)i;
    g->idx = 624;
}
unsigned int ok_mt19937_next(ok_mt19937* g) {
    unsigned int y;
    if (g->idx >= 624) { /* _M_gen_rand */
        int k;
        for (k = 0; k < 624; ++k) {
            const unsigned int x = (g->mt[k] & 0x80000000u) | (g->mt[(k + 1) % 624] & 0x7fffffffu);
            unsigned int v = g->mt[(k + 397) % 624] ^ (x >> 1);
            if (x & 1u) v ^= 0x9908b0dfu;
            g->mt[k] = v;
        }
        g->idx = 0;
    }
    y = g->mt[g->idx++];
    y ^= (y >> 11);
    y ^= (y << 7) & 0x9d2c5680u;
    y ^= (y << 15) & 0xefc60000u;
    y ^= (y >> 18);
    return y;
}
int ok_uniform_int(ok_mt19937* g, int a, int b) {
    /* urngrange == UINT32_MAX > urange: downscaling with the 64-bit multiply (Lemire, "_S_nd") */
    const unsigned int range = (unsigned int)b - (unsigned int)a + 1u;
    unsigned long long product = (unsigned long long)ok_mt19937_next(g) * range;
    unsigned int low = (unsigned int)product;
    if (low < range) {
        const unsigned int threshold = (0u - range) % range;
        while (low < threshold) {
            product = (unsigned long long)ok_mt19937_next(g) * range;
            low = (unsigned int)product;
        }
    }
    return (int)((unsigned int)(product >> 32) + (unsigned int)a);
}

/* ---- Matrix3d::inverse() ---- */
#define M3(m, i, j) (m)[(i) + 3 * (j)]
static double cof3(const double* m, int i, int j) {
    const int i1 = (i + 1) % 3, i2 = (i + 2) % 3, j1 = (j + 1) % 3, j2 = (j + 2) % 3;
    return M3(m, i1, j1) * M3(m, i2, j2) - M3(m, i1, j2) * M3(m, i2, j1);
}
void ok_gps_inverse3(const double m[9], double out[9]) {
    double c0[3], r[9], det, invdet;
    c0[0] = cof3(m, 0, 0); c0[1] = cof3(m, 1, 0); c0[2] = cof3(m, 2, 0);
    det = (c0[0] * M3(m, 0, 0) + c0[1] * M3(m, 1, 0)) + c0[2] * M3(m, 2, 0); /* cwiseProduct(col0).sum(): Packet2d pair, then the tail */
    invdet = 1.0 / det;
    M3(r, 1, 0) = cof3(m, 0, 1) * invdet; M3(r, 1, 1) = cof3(m, 1, 1) * invdet; M3(r, 2, 0) = cof3(m, 0, 2) * invdet;
    M3(r, 1, 2) = cof3(m, 2, 1) * invdet; M3(r, 2, 1) = cof3(m, 1, 2) * invdet; M3(r, 2, 2) = cof3(m, 2, 2) * invdet;
    M3(r, 0, 0) = c0[0] * invdet; M3(r, 0, 1) = c0[1] * invdet; M3(r, 0, 2) = c0[2] * invdet;
    memcpy(out, r, sizeof r);
}

/* ---- Matrix4d::inverse(): Eigen's SSE2 Packet2d kernel (arch/SSE/Inverse.h) lane by lane; same model as
 * rdvio_port/c/rd_sys_eigen.c rd_m4_inverse ---- */
typedef struct p2 { double a, b; } p2;
static p2 ld(const double* d) { p2 r; r.a = d[0]; r.b = d[1]; return r; }
static double ln(p2 v, int i) { return i ? v.b : v.a; }
static p2 swz(p2 x, p2 y, int mask) { p2 r; r.a = ln(x, mask & 1); r.b = ln(y, (mask >> 1) & 1); return r; }   /* _mm_shuffle_pd */
static p2 dup(p2 x, int i) { return swz(x, x, (i << 1) | i); }
static p2 mul(p2 x, p2 y) { p2 r; r.a = x.a * y.a; r.b = x.b * y.b; return r; }
static p2 sub(p2 x, p2 y) { p2 r; r.a = x.a - y.a; r.b = x.b - y.b; return r; }
static p2 add(p2 x, p2 y) { p2 r; r.a = x.a + y.a; r.b = x.b + y.b; return r; }
static void st(double* d, p2 v) { d[0] = v.a; d[1] = v.b; }
void ok_gps_inverse4(const double A[16], double out[16]) {
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


/* ---- umeyamaTransform ---- */
/* H = W * G^T, W, G dynamic 3 x n (demeaned). Matrix3d = MatrixXd * Transpose<MatrixXd> with a dynamic inner size is a
 * GemmProduct: rows + cols + depth < 20 (n < 14) evaluates the coefficient-based lazy product, else the GEBP kernel. */
static void h_product(int n, const double* W, const double* G, double H[9]) {
    int i, j, k;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) {
            double s = W[i] * G[j];
            for (k = 1; k < n; ++k) s = s + W[i + 3 * k] * G[j + 3 * k];
            H[i + 3 * j] = s;
        }
}

int ok_gps_umeyama(int n, const double* gps, const double* world, ok_tf* T) {
    double cg[3] = {0.0, 0.0, 0.0}, cw[3] = {0.0, 0.0, 0.0};
    double *W, *G, *Rt, H[9], R[9], t[3], v[3], M[16];
    double A, B, theta, c, s;
    int i, k;
    if (n < 3) { ok_tf_identity(T); return 1; }
    W = (double*)malloc(sizeof(double) * 3 * (size_t)n);
    G = (double*)malloc(sizeof(double) * 3 * (size_t)n);
    for (i = 0; i < n; ++i) {
        for (k = 0; k < 3; ++k) { cg[k] += gps[3 * i + k]; cw[k] += world[3 * i + k]; }
    }
    for (k = 0; k < 3; ++k) { cg[k] = cg[k] / (double)n; cw[k] = cw[k] / (double)n; }
    for (i = 0; i < n; ++i)
        for (k = 0; k < 3; ++k) { G[3 * i + k] = gps[3 * i + k] - cg[k]; W[3 * i + k] = world[3 * i + k] - cw[k]; }
    if (n + 6 < 20) {
        h_product(n, W, G, H);
    } else {
        Rt = (double*)malloc(sizeof(double) * 3 * (size_t)n);
        for (i = 0; i < n; ++i)
            for (k = 0; k < 3; ++k) Rt[i + n * k] = G[k + 3 * i];
        ok_gemm(3, 3, n, W, Rt, H);
        free(Rt);
    }
    A = H[0 + 3 * 1] - H[1 + 3 * 0];
    B = H[0 + 3 * 0] + H[1 + 3 * 1];
    theta = PI_ / 2.0 - atan2(B, A);
    c = cos(theta); s = sin(theta);
    memset(R, 0, sizeof R);
    R[0] = c; R[3] = -s; R[1] = s; R[4] = c; R[8] = 1.0;
    ok_m3_mulv(R, cw, v);
    for (k = 0; k < 3; ++k) t[k] = cg[k] - v[k];
    memset(M, 0, sizeof M);
    M[0] = 1.0; M[5] = 1.0; M[10] = 1.0; M[15] = 1.0;
    for (i = 0; i < 3; ++i)
        for (k = 0; k < 3; ++k) M[i + 4 * k] = R[i + 3 * k];
    M[12] = t[0]; M[13] = t[1]; M[14] = t[2];
    ok_tf_from_m4(T, M, 1);
    free(W); free(G);
    return 0;
}

/* ---- estimateRigidRansac ---- */
void ok_gps_estimate_rigid_ransac(int n, const double* gps, const double* world, int iterations, int n_points,
                                  double inlier_threshold, double required_inlier_ratio, ok_rigid_result* out,
                                  int* inliers_out) {
    ok_mt19937 rng;
    double *gs, *ws;
    int *idxs, *cur, it, i, k;
    memset(out, 0, sizeof *out); /* RigidResult() is value-initialised */
    out->inlier_ratio = 0.0;
    if (n < 2 * n_points) return;
    ok_mt19937_seed(&rng, 42u);
    gs = (double*)malloc(sizeof(double) * 3 * (size_t)(n_points > 0 ? n_points : 1));
    ws = (double*)malloc(sizeof(double) * 3 * (size_t)(n_points > 0 ? n_points : 1));
    idxs = (int*)malloc(sizeof(int) * (size_t)(n_points > 0 ? n_points : 1));
    cur = (int*)malloc(sizeof(int) * (size_t)n);
    for (it = 0; it < iterations; ++it) {
        ok_tf T;
        int count = 0, nin = 0, dup;
        double ratio;
        while (count < n_points) {
            const int idx = ok_uniform_int(&rng, 0, n - 1);
            dup = 0;
            for (k = 0; k < count; ++k) if (idxs[k] == idx) { dup = 1; break; }
            if (!dup) idxs[count++] = idx;
        }
        for (k = 0; k < n_points; ++k) {
            memcpy(gs + 3 * k, gps + 3 * idxs[k], 3 * sizeof(double));
            memcpy(ws + 3 * k, world + 3 * idxs[k], 3 * sizeof(double));
        }
        ok_gps_umeyama(n_points, gs, ws, &T);
        for (i = 0; i < n; ++i) {
            double est[3], d[3], err;
            ok_m3_mulv(T.C, world + 3 * i, est);
            for (k = 0; k < 3; ++k) { est[k] = est[k] + T.r[k]; d[k] = gps[3 * i + k] - est[k]; }
            err = ok_v3_norm(d);
            if (err < inlier_threshold) cur[nin++] = i;
        }
        ratio = (double)nin / (double)n;
        if (ratio > out->inlier_ratio) {
            out->inlier_ratio = ratio;
            memcpy(out->R, T.C, sizeof out->R);
            memcpy(out->t, T.r, sizeof out->t);
            out->n_inliers = nin;
            if (inliers_out) memcpy(inliers_out, cur, sizeof(int) * (size_t)nin);
        }
        if (ratio > required_inlier_ratio) break;
    }
    free(gs); free(ws); free(idxs); free(cur);
}

/* ---- checkForGpsInit numeric core ---- */
void ok_gps_yaw_hessian(int n, const double* world, const double* cov, const double C[9], double Hess[16]) {
    int i, j, k;
    memset(Hess, 0, 16 * sizeof(double));
    for (i = 0; i < n; ++i) {
        double Ei[12], cp[3], X[9], Ci[9], M1[12], tmp2[16];
        memset(Ei, 0, sizeof Ei); /* 3x4 column-major */
        for (j = 0; j < 3; ++j) Ei[j + 3 * j] = -1.0;             /* -Identity(): off-diagonal zeros are -0.0 */
        for (j = 0; j < 3; ++j) for (k = 0; k < 3; ++k) if (j != k) Ei[j + 3 * k] = -0.0;
        ok_m3_mulv(C, world + 3 * i, cp);
        ok_kin_cross_mx(cp, X);
        Ei[9] = X[6]; Ei[10] = X[7]; Ei[11] = X[8];                /* crossMx(..).col(2) */
        ok_gps_inverse3(cov + 9 * i, Ci);
        for (j = 0; j < 4; ++j)                                    /* (Ei^T * Ci): 4x3 */
            for (k = 0; k < 3; ++k) {
                const double p0 = Ei[0 + 3 * j] * Ci[0 + 3 * k], p1 = Ei[1 + 3 * j] * Ci[1 + 3 * k], p2 = Ei[2 + 3 * j] * Ci[2 + 3 * k];
                M1[j + 4 * k] = (p0 + p1) + p2;
            }
        for (j = 0; j < 4; ++j)                                    /* M1 * Ei: 4x4 */
            for (k = 0; k < 4; ++k)
                tmp2[j + 4 * k] = (M1[j + 0] * Ei[0 + 3 * k] + M1[j + 4] * Ei[1 + 3 * k]) + M1[j + 8] * Ei[2 + 3 * k];
        for (j = 0; j < 16; ++j) Hess[j] = Hess[j] + tmp2[j];
    }
}

int ok_gps_init_core(int n, const double* gps, const double* world, const double* cov, int robust, ok_tf* T_GW,
                     double* yaw_error_deg, double* ransac_ratio) {
    double Hess[16], P[16];
    if (robust) {
        ok_rigid_result rr;
        double M[16];
        int i, k;
        ok_gps_estimate_rigid_ransac(n, gps, world, 20, 20, 4.0, 0.7, &rr, NULL);
        if (ransac_ratio) *ransac_ratio = rr.inlier_ratio;
        if (rr.inlier_ratio < 0.25) return 1;
        memset(M, 0, sizeof M);
        M[0] = 1.0; M[5] = 1.0; M[10] = 1.0; M[15] = 1.0;
        for (i = 0; i < 3; ++i)
            for (k = 0; k < 3; ++k) M[i + 4 * k] = rr.R[i + 3 * k];
        M[12] = rr.t[0]; M[13] = rr.t[1]; M[14] = rr.t[2];
        ok_tf_set_m4(T_GW, M, 1);
    } else {
        ok_gps_umeyama(n, gps, world, T_GW);
    }
    ok_gps_yaw_hessian(n, world, cov, T_GW->C, Hess);
    ok_gps_inverse4(Hess, P);
    *yaw_error_deg = sqrt(P[3 + 4 * 3]) / PI_ * 180.0;
    return 0;
}
