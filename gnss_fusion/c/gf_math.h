/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 * gf_math.h : tiny internal helpers (3x3 rotations, quaternions, 5x5 block Cholesky). Not part of the public API. */
#ifndef GF_MATH_H
#define GF_MATH_H
#include <math.h>
#include <string.h>

#define GF_NS 5   /* state per node: psi, px, py, pz, s */

static void gf_mat3_mul(const double *A, const double *B, double *C)   /* C = A B (row-major, C != A,B) */
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
}

static void gf_mat3_vec(const double *A, const double *v, double *o)
{
    o[0] = A[0] * v[0] + A[1] * v[1] + A[2] * v[2];
    o[1] = A[3] * v[0] + A[4] * v[1] + A[5] * v[2];
    o[2] = A[6] * v[0] + A[7] * v[1] + A[8] * v[2];
}

/* Rz(psi) v and Rz(psi)^T v, and the psi-derivatives of both */
static void gf_rz_vec(double psi, const double *v, double *o)
{ double c = cos(psi), s = sin(psi); o[0] = c * v[0] - s * v[1]; o[1] = s * v[0] + c * v[1]; o[2] = v[2]; }
static void gf_rzt_vec(double psi, const double *v, double *o)
{ double c = cos(psi), s = sin(psi); o[0] = c * v[0] + s * v[1]; o[1] = -s * v[0] + c * v[1]; o[2] = v[2]; }
static void gf_drz_vec(double psi, const double *v, double *o)
{ double c = cos(psi), s = sin(psi); o[0] = -s * v[0] - c * v[1]; o[1] = c * v[0] - s * v[1]; o[2] = 0.0; }
static void gf_drzt_vec(double psi, const double *v, double *o)
{ double c = cos(psi), s = sin(psi); o[0] = -s * v[0] + c * v[1]; o[1] = -c * v[0] - s * v[1]; o[2] = 0.0; }

static void gf_quat_to_mat(const double *q, double *R)   /* (x y z w), normalised here */
{
    double n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    double x, y, z, w;
    if (n < 1e-12) { x = y = z = 0.0; w = 1.0; } else { x = q[0] / n; y = q[1] / n; z = q[2] / n; w = q[3] / n; }
    R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w);     R[2] = 2 * (x * z + y * w);
    R[3] = 2 * (x * y + z * w);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
    R[6] = 2 * (x * z - y * w);     R[7] = 2 * (y * z + x * w);     R[8] = 1 - 2 * (x * x + y * y);
}

static void gf_mat_to_quat(const double *R, double *q)
{
    double t = R[0] + R[4] + R[8];
    if (t > 0) {
        double s = sqrt(t + 1) * 2;
        q[0] = (R[7] - R[5]) / s; q[1] = (R[2] - R[6]) / s; q[2] = (R[3] - R[1]) / s; q[3] = s / 4;
    } else {
        int i = 0;
        if (R[4] > R[0]) i = 1;
        if (R[8] > R[3 * i + i]) i = 2;
        int j = (i + 1) % 3, k = (i + 2) % 3;
        double s = sqrt(R[3 * i + i] - R[3 * j + j] - R[3 * k + k] + 1) * 2;
        q[i] = s / 4; q[j] = (R[3 * j + i] + R[3 * i + j]) / s; q[k] = (R[3 * k + i] + R[3 * i + k]) / s;
        q[3] = (R[3 * k + j] - R[3 * j + k]) / s;
    }
}

/* Rotation taking unit vector `up` (odometry frame) to +z: Rodrigues, same convention as tools/gnss_harness/robust_fusion.py */
static void gf_align_up(const double *up_in, double *R)
{
    double n = sqrt(up_in[0] * up_in[0] + up_in[1] * up_in[1] + up_in[2] * up_in[2]);
    double u[3] = { up_in[0] / n, up_in[1] / n, up_in[2] / n };
    double v[3] = { u[1], -u[0], 0.0 };            /* u x z */
    double c = u[2], s = sqrt(v[0] * v[0] + v[1] * v[1]);
    double K[9] = { 0, -v[2], v[1], v[2], 0, -v[0], -v[1], v[0], 0 };
    double K2[9];
    gf_mat3_mul(K, K, K2);
    for (int i = 0; i < 9; ++i) R[i] = ((i % 4) == 0 ? 1.0 : 0.0);
    if (s < 1e-9) {
        if (c < 0) { R[4] = -1.0; R[8] = -1.0; }   /* pointing down: flip about x */
        return;
    }
    for (int i = 0; i < 9; ++i) R[i] += K[i] + K2[i] * (1 - c) / (s * s);
}

/* Cholesky of the symmetric 5x5 A (lower, in L). Pivots are floored: the normal matrices carry a 1e-6 I regulariser. */
static void gf_chol5(const double *A, double *L)
{
    for (int i = 0; i < GF_NS; ++i)
        for (int j = 0; j <= i; ++j) {
            double s = A[GF_NS * i + j];
            for (int k = 0; k < j; ++k) s -= L[GF_NS * i + k] * L[GF_NS * j + k];
            if (i == j) L[GF_NS * i + i] = sqrt(s > 1e-14 ? s : 1e-14);
            else        L[GF_NS * i + j] = s / L[GF_NS * j + j];
        }
}
/* solve L L^T x = b in place for one column (stride 1 vector of 5) */
static void gf_chol5_solve(const double *L, double *b)
{
    for (int i = 0; i < GF_NS; ++i) {
        double s = b[i];
        for (int k = 0; k < i; ++k) s -= L[GF_NS * i + k] * b[k];
        b[i] = s / L[GF_NS * i + i];
    }
    for (int i = GF_NS - 1; i >= 0; --i) {
        double s = b[i];
        for (int k = i + 1; k < GF_NS; ++k) s -= L[GF_NS * k + i] * b[k];
        b[i] = s / L[GF_NS * i + i];
    }
}

/* Block-tridiagonal solve  H x = b,  H[k][k] = D[k], H[k][k+1] = O[k], H[k+1][k] = O[k]^T  (n blocks of 5).
 * D, b are destroyed; G is caller workspace of 25*(n-1) doubles. */
static void gf_block_thomas(int n, double *D, const double *O, double *b, double *G, double *x)
{
    double L[GF_NS * GF_NS], col[GF_NS];
    for (int k = 0; k < n - 1; ++k) {
        double *Dk = D + 25 * k, *Dn = D + 25 * (k + 1), *Gk = G + 25 * k;
        const double *Ok = O + 25 * k;
        double *bk = b + GF_NS * k, *bn = b + GF_NS * (k + 1);
        gf_chol5(Dk, L);
        for (int c = 0; c < GF_NS; ++c) {          /* G = Dk^-1 Ok */
            for (int r = 0; r < GF_NS; ++r) col[r] = Ok[GF_NS * r + c];
            gf_chol5_solve(L, col);
            for (int r = 0; r < GF_NS; ++r) Gk[GF_NS * r + c] = col[r];
        }
        gf_chol5_solve(L, bk);                      /* h = Dk^-1 bk */
        for (int i = 0; i < GF_NS; ++i) {           /* Dn -= Ok^T G ; bn -= Ok^T h */
            for (int j = 0; j < GF_NS; ++j) {
                double a = 0.0;
                for (int r = 0; r < GF_NS; ++r) a += Ok[GF_NS * r + i] * Gk[GF_NS * r + j];
                Dn[GF_NS * i + j] -= a;
            }
            double a = 0.0;
            for (int r = 0; r < GF_NS; ++r) a += Ok[GF_NS * r + i] * bk[r];
            bn[i] -= a;
        }
    }
    gf_chol5(D + 25 * (n - 1), L);
    for (int i = 0; i < GF_NS; ++i) x[GF_NS * (n - 1) + i] = b[GF_NS * (n - 1) + i];
    gf_chol5_solve(L, x + GF_NS * (n - 1));
    for (int k = n - 2; k >= 0; --k) {              /* x_k = h_k - G_k x_{k+1} */
        const double *Gk = G + 25 * k, *xn = x + GF_NS * (k + 1);
        for (int i = 0; i < GF_NS; ++i) {
            double a = b[GF_NS * k + i];
            for (int j = 0; j < GF_NS; ++j) a -= Gk[GF_NS * i + j] * xn[j];
            x[GF_NS * k + i] = a;
        }
    }
}
#endif
