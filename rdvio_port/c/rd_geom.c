/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/* See rd_geom.h for provenance and licences. Each statement notes the Eigen 3.4.0 expression form it reproduces.
 * Re-used read-only kernels (included by path, not copied): okvis_port/c/ok_eigen.c (3x3 rules), stella_port/c/sv_eigen_svd.c +
 * sv_eigen_qr.c (JacobiSVD), stella_port/c/sv_eigen_eigensolver.c (EigenSolver<Matrix<double,10,10>>). */
#include "rd_geom.h"
#include "rd_svd.h"
#include "../../stella_port/c/sv_eigen_svd.h"
#include "../../stella_port/c/sv_eigen_eigensolver.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef RD_GEMV94_PACKET_ROWS
#define RD_GEMV94_PACKET_ROWS 8
#endif
static void rd_gemv94(const double B[36], const double s[4], double out[9]);

/* ---- 3x3 helpers (column-major) ---- */
static void m3_t(const double a[9], double out[9]) { int i, j; for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) out[i + 3 * j] = a[j + 3 * i]; }
static void m3_neg(double a[9]) { int i; for (i = 0; i < 9; ++i) a[i] = -a[i]; }
/* MatrixBase::determinant(), 3x3 (Eigen 3.4: row-0 expansion, A - B + C) */
static double det3(const double m[9]) {
#define Mx(r, c) m[(r) + 3 * (c)]
    const double A = Mx(0, 0) * (Mx(1, 1) * Mx(2, 2) - Mx(1, 2) * Mx(2, 1));
    const double B = Mx(0, 1) * (Mx(1, 0) * Mx(2, 2) - Mx(1, 2) * Mx(2, 0));
    const double C = Mx(0, 2) * (Mx(1, 0) * Mx(2, 1) - Mx(1, 1) * Mx(2, 0));
#undef Mx
    return (A - B) + C;
}
/* A.transpose() * B : lhs row-major (Transpose of a column-major matrix) => every entry is the vectorised inner product ((a0b0+a1b1)+a2b2) */
static void m3_tmul(const double a[9], const double b[9], double out[9]) {
    double r[9]; int i, j;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) r[i + 3 * j] = (a[0 + 3 * i] * b[0 + 3 * j] + a[1 + 3 * i] * b[1 + 3 * j]) + a[2 + 3 * i] * b[2 + 3 * j];
    memcpy(out, r, sizeof r);
}
/* every coefficient scalar: (a * b) with a halving-tree inner sum a0 + (a1 + a2) (storage orders of product and destination disagree) */
static void m3_mul_tree(const double a[9], const double b[9], double out[9]) {
    double r[9]; int i, j;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) r[i + 3 * j] = a[i] * b[3 * j] + (a[i + 3] * b[1 + 3 * j] + a[i + 6] * b[2 + 3 * j]);
    memcpy(out, r, sizeof r);
}
static double v3_norm(const double v[3]) { return ok_v3_norm(v); }
static void v3_normalize(double v[3]) {  /* MatrixBase::normalize(): z = squaredNorm(); if (z > 0) v /= sqrt(z) */
    const double z = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    if (z > 0.0) { const double s = sqrt(z); v[0] /= s; v[1] /= s; v[2] /= s; }
}

/* ---------------------------------------------------------------- wahba.h */
void rd_solve_rotation_2pt(const double p1[2][3], const double p2[2][3], double R[9]) {
    double cov[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}, U[9], V[9], sv[3], Ut[9], VUt[9], E[9], VE[9];
    int i, a, b;
    for (i = 0; i < 2; ++i)
        for (a = 0; a < 3; ++a) for (b = 0; b < 3; ++b) cov[a + 3 * b] += p1[i][a] * p2[i][b];   /* cov += points1[i] * points2[i].transpose() */
    for (i = 0; i < 9; ++i) cov[i] = cov[i] * 0.5;
    sv_eigen_jacobisvd_3x3(cov, U, V, sv);
    m3_t(U, Ut);
    ok_m3_mul(V, Ut, VUt);                      /* (V * U.transpose()).determinant() : product evaluated into a temporary */
    for (i = 0; i < 9; ++i) E[i] = (i % 4 == 0) ? 1.0 : 0.0;
    E[8] = (det3(VUt) >= 0.0) ? 1.0 : -1.0;
    ok_m3_mul(V, E, VE);                        /* V * E * U.transpose() = (V * E) * U^T */
    ok_m3_mul(VE, Ut, R);
}

/* ------------------------------------------------------------- essential.cpp */
enum { XXX = 0, XXY, XYY, YYY, XXZ, XYZ, YYZ, XZZ, YZZ, ZZZ, XX, XY, YY, XZ, YZ, ZZ, X, Y, Z, I };
typedef struct { double v[20]; } poly;
static poly poly_add(const poly* a, const poly* b) { poly r; int i; for (i = 0; i < 20; ++i) r.v[i] = a->v[i] + b->v[i]; return r; }
static poly poly_sub(const poly* a, const poly* b) { poly r; int i; for (i = 0; i < 20; ++i) r.v[i] = a->v[i] - b->v[i]; return r; }
static poly poly_scale(double s, const poly* a) { poly r; int i; for (i = 0; i < 20; ++i) r.v[i] = s * a->v[i]; return r; }
static poly poly_mul(const poly* pa, const poly* pb) {   /* Polynomial::operator*, summand order as in the source */
    const double* v = pa->v; const double* b = pb->v;
    poly r;
    r.v[I] = v[I] * b[I];
    r.v[Z] = v[I] * b[Z] + v[Z] * b[I];
    r.v[Y] = v[I] * b[Y] + v[Y] * b[I];
    r.v[X] = v[I] * b[X] + v[X] * b[I];
    r.v[ZZ] = v[I] * b[ZZ] + v[Z] * b[Z] + v[ZZ] * b[I];
    r.v[YZ] = v[I] * b[YZ] + v[Z] * b[Y] + v[Y] * b[Z] + v[YZ] * b[I];
    r.v[XZ] = v[I] * b[XZ] + v[Z] * b[X] + v[X] * b[Z] + v[XZ] * b[I];
    r.v[YY] = v[I] * b[YY] + v[Y] * b[Y] + v[YY] * b[I];
    r.v[XY] = v[I] * b[XY] + v[Y] * b[X] + v[X] * b[Y] + v[XY] * b[I];
    r.v[XX] = v[I] * b[XX] + v[X] * b[X] + v[XX] * b[I];
    r.v[ZZZ] = v[I] * b[ZZZ] + v[Z] * b[ZZ] + v[ZZ] * b[Z] + v[ZZZ] * b[I];
    r.v[YZZ] = v[I] * b[YZZ] + v[Z] * b[YZ] + v[Y] * b[ZZ] + v[ZZ] * b[Y] + v[YZ] * b[Z] + v[YZZ] * b[I];
    r.v[XZZ] = v[I] * b[XZZ] + v[Z] * b[XZ] + v[X] * b[ZZ] + v[ZZ] * b[X] + v[XZ] * b[Z] + v[XZZ] * b[I];
    r.v[YYZ] = v[I] * b[YYZ] + v[Z] * b[YY] + v[Y] * b[YZ] + v[YZ] * b[Y] + v[YY] * b[Z] + v[YYZ] * b[I];
    r.v[XYZ] = v[I] * b[XYZ] + v[Z] * b[XY] + v[Y] * b[XZ] + v[X] * b[YZ] + v[YZ] * b[X] + v[XZ] * b[Y] + v[XY] * b[Z] + v[XYZ] * b[I];
    r.v[XXZ] = v[I] * b[XXZ] + v[Z] * b[XX] + v[X] * b[XZ] + v[XZ] * b[X] + v[XX] * b[Z] + v[XXZ] * b[I];
    r.v[YYY] = v[I] * b[YYY] + v[Y] * b[YY] + v[YY] * b[Y] + v[YYY] * b[I];
    r.v[XYY] = v[I] * b[XYY] + v[Y] * b[XY] + v[X] * b[YY] + v[YY] * b[X] + v[XY] * b[Y] + v[XYY] * b[I];
    r.v[XXY] = v[I] * b[XXY] + v[Y] * b[XX] + v[X] * b[XY] + v[XY] * b[X] + v[XX] * b[Y] + v[XXY] * b[I];
    r.v[XXX] = v[I] * b[XXX] + v[X] * b[XX] + v[XX] * b[X] + v[XXX] * b[I];
    return r;
}
/* sum of three polynomials as the unrolled non-vectorised redux: a0 + (a1 + a2) */
static poly poly_sum3(const poly* a0, const poly* a1, const poly* a2) { poly t = poly_add(a1, a2); return poly_add(a0, &t); }

#define E_(m, i, j) (m)[(i) + 3 * (j)]   /* 3x3 matrix of polynomials, column-major */

static void generate_nullspace_basis(const double p1[5][2], const double p2[5][2], double basis[36]) {
    double A[45], V[81], sv[9]; int i, j, b, rank;
    for (i = 0; i < 5; ++i) {
        const double x1[3] = {p1[i][0], p1[i][1], 1.0}, x2[3] = {p2[i][0], p2[i][1], 1.0};
        for (j = 0; j < 3; ++j) for (b = 0; b < 3; ++b) A[i + 5 * (j * 3 + b)] = x1[j] * x2[b];   /* h = x1 * x2^T; A.block<1,3>(i, 3j) = h.row(j) */
    }
    rd_jacobisvd_Nxc_v(A, 5, 9, V, sv, &rank);
    for (j = 0; j < 4; ++j) for (i = 0; i < 9; ++i) basis[i + 9 * j] = V[i + 9 * (5 + j)];
}

static void generate_polynomials(const double basis[36], double polys[200] /* 10 x 20 col-major */) {
    poly Ep[9], EEt[9], Ept[9], EEtE[9], sv[9], tr, s, detE, t1, t2, t3;
    int i, j, k;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) {   /* to_matrix(basis.col(c))(i,j) = basis(3j + i, c) */
        poly p; memset(&p, 0, sizeof p);
        p.v[X] = basis[(3 * j + i) + 9 * 0]; p.v[Y] = basis[(3 * j + i) + 9 * 1]; p.v[Z] = basis[(3 * j + i) + 9 * 2]; p.v[I] = basis[(3 * j + i) + 9 * 3];
        E_(Ep, i, j) = p;
    }
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) E_(Ept, i, j) = E_(Ep, j, i);
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {   /* Epoly * Epoly.transpose() : coefficient product, redux tree */
        poly a = poly_mul(&E_(Ep, i, 0), &E_(Ept, 0, j)), b = poly_mul(&E_(Ep, i, 1), &E_(Ept, 1, j)), c = poly_mul(&E_(Ep, i, 2), &E_(Ept, 2, j));
        E_(EEt, i, j) = poly_sum3(&a, &b, &c);
    }
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {   /* EEt * Epoly */
        poly a = poly_mul(&E_(EEt, i, 0), &E_(Ep, 0, j)), b = poly_mul(&E_(EEt, i, 1), &E_(Ep, 1, j)), c = poly_mul(&E_(EEt, i, 2), &E_(Ep, 2, j));
        E_(EEtE, i, j) = poly_sum3(&a, &b, &c);
    }
    tr = poly_sum3(&E_(EEt, 0, 0), &E_(EEt, 1, 1), &E_(EEt, 2, 2));   /* trace() = diagonal().sum() */
    s = poly_scale(0.5, &tr);                                           /* 0.5 * trace */
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {
        poly m = poly_mul(&s, &E_(Ep, i, j));                          /* (0.5 * trace) * Epoly : scalar on the left */
        E_(sv, i, j) = poly_sub(&E_(EEtE, i, j), &m);
    }
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) for (k = 0; k < 20; ++k) polys[(i * 3 + j) + 10 * k] = E_(sv, i, j).v[k];
    /* Epoly.determinant(): helper(0,1,2) - helper(1,0,2) + helper(2,0,1), helper(a,b,c) = m(0,a) * (m(1,b) m(2,c) - m(1,c) m(2,b)) */
#define HELP(out, a, b, c) do { poly x_ = poly_mul(&E_(Ep, 1, b), &E_(Ep, 2, c)), y_ = poly_mul(&E_(Ep, 1, c), &E_(Ep, 2, b)); poly d_ = poly_sub(&x_, &y_); out = poly_mul(&E_(Ep, 0, a), &d_); } while (0)
    HELP(t1, 0, 1, 2); HELP(t2, 1, 0, 2); HELP(t3, 2, 0, 1);
#undef HELP
    { poly d = poly_sub(&t1, &t2); detE = poly_add(&d, &t3); }
    for (k = 0; k < 20; ++k) polys[9 + 10 * k] = detE.v[k];
}

#define P_(r, c) polys[(r) + 10 * (c)]
static void generate_action_matrix(double polys[200], double action[100]) {
    int perm[10], i, j, k, c;
    for (i = 0; i < 10; ++i) perm[i] = i;
    for (i = 0; i < 10; ++i) {
        for (j = i + 1; j < 10; ++j)
            if (fabs(P_(perm[i], i)) < fabs(P_(perm[j], i))) { int t = perm[i]; perm[i] = perm[j]; perm[j] = t; }
        if (P_(perm[i], i) == 0) continue;
        { const double piv = P_(perm[i], i); for (c = 0; c < 20; ++c) P_(perm[i], c) /= piv; }
        for (j = i + 1; j < 10; ++j) { const double s = P_(perm[j], i); for (c = 0; c < 20; ++c) P_(perm[j], c) = P_(perm[j], c) - P_(perm[i], c) * s; }
    }
    for (i = 9; i > 0; --i)
        for (j = 0; j < i; ++j) { const double s = P_(perm[j], i); for (c = 0; c < 20; ++c) P_(perm[j], c) = P_(perm[j], c) - P_(perm[i], c) * s; }
    {
        static const int src[6] = {XXX, XXY, XYY, XXZ, XYZ, XZZ};
        memset(action, 0, 100 * sizeof(double));
        for (k = 0; k < 6; ++k) for (c = 0; c < 10; ++c) action[k + 10 * c] = -P_(perm[src[k]], XX + c);
        action[6 + 10 * (XX - XX)] = 1.0; action[7 + 10 * (XY - XX)] = 1.0; action[8 + 10 * (XZ - XX)] = 1.0; action[9 + 10 * (X - XX)] = 1.0;
    }
}
#undef P_

int rd_solve_essential_5pt(const double p1[5][2], const double p2[5][2], double E[10][9]) {
    double basis[36], polys[200], action[100], er[10], ei[10], vr[100], vi[100];
    int i, n = 0, a;
    generate_nullspace_basis(p1, p2, basis);
    generate_polynomials(basis, polys);
    generate_action_matrix(polys, action);
    if (sv_eigen_eigensolver10(action, er, ei, vr, vi)) return -1;
    for (i = 0; i < 10; ++i) {
        if (fabs(ei[i]) < 1.0e-10) {
            const double xw = vr[(X - XX) + 10 * i], yw = vr[(Y - XX) + 10 * i], zw = vr[(Z - XX) + 10 * i], w = vr[(I - XX) + 10 * i];
            const double s[4] = {xw / w, yw / w, zw / w, 1.0};
            /* to_matrix(basis * solution.homogeneous()): 9x4 * 4 coefficient product; see rd_gemv94 */
            double h[9];
            rd_gemv94(basis, s, h);
            for (a = 0; a < 9; ++a) E[n][a] = h[a];
            n++;
        }
    }
    return n;
}

/* basis * x.homogeneous(): Eigen evaluates a product with a Homogeneous<Vertical> operand as  dst = basis.leftCols(3) * x;  dst += basis.col(3).
 * The 9x3 * 3 coefficient product has packets over rows 0..7 (left fold ((a0+a1)+a2)) and a scalar last row (halving tree a0+(a1+a2)). */
static void rd_gemv94(const double B[36], const double s[4], double out[9]) {
    int i;
    for (i = 0; i < 9; ++i) {
        const double a = B[i] * s[0], b = B[i + 9] * s[1], c = B[i + 18] * s[2];
        const double p = (i < RD_GEMV94_PACKET_ROWS) ? ((a + b) + c) : (a + (b + c));
        out[i] = p + B[i + 27];
    }
}

void rd_decompose_essential(const double E[9], double R1[9], double R2[9], double T[3]) {
    double U[9], V[9], sv[3], VT[9], W[9], WT[9], UW[9];
    int i;
    sv_eigen_jacobisvd_3x3(E, U, V, sv);
    m3_t(V, VT);
    if (det3(U) < 0) m3_neg(U);
    if (det3(VT) < 0) m3_neg(VT);
    for (i = 0; i < 9; ++i) W[i] = 0.0;
    W[0 + 3 * 1] = 1.0; W[1 + 3 * 0] = -1.0; W[2 + 3 * 2] = 1.0;   /* W << 0, 1, 0, -1, 0, 0, 0, 0, 1 */
    ok_m3_mul(U, W, UW); ok_m3_mul(UW, VT, R1);
    m3_t(W, WT);
    ok_m3_mul(U, WT, UW); ok_m3_mul(UW, VT, R2);
    T[0] = U[6]; T[1] = U[7]; T[2] = U[8];
}

int rd_decompose_homography(const double H[9], double R1[9], double R2[9], double T1[3], double T2[3], double n1[3], double n2[3]) {
    double U[9], V[9], sv[3], Hn[9], Hnt[9], S[9], tmp[9];
    int i, j, pure = 1;
    sv_eigen_jacobisvd_3x3(H, U, V, sv);
    for (i = 0; i < 9; ++i) Hn[i] = H[i] / sv[1];
    m3_t(Hn, Hnt); (void)Hnt;
    m3_tmul(Hn, Hn, tmp);                        /* Hn.transpose() * Hn (lhs transpose: all rows left fold) */
    for (i = 0; i < 9; ++i) S[i] = tmp[i] - ((i % 4 == 0) ? 1.0 : 0.0);
#define S_(r, c) S[(r) + 3 * (c)]
    for (i = 0; i < 3 && pure; ++i) for (j = 0; j < 3 && pure; ++j) if (fabs(S_(i, j)) > 1e-3) pure = 0;
    if (pure) {
        double Id[9], UI[9], Vt[9];
        for (i = 0; i < 9; ++i) Id[i] = (i % 4 == 0) ? 1.0 : 0.0;
        ok_m3_mul(U, Id, UI); m3_t(V, Vt);
        /* U * Identity * V^T: (U*Id) has NoPreferredStorageOrder and V^T is row-major => the outer Product is row-major PlainObject, the
         * coefficient product is column-major, orders disagree: no packets, every entry a halving tree, then copied into R1 */
        m3_mul_tree(UI, Vt, R1);
        if (det3(R1) < 0) m3_neg(R1);
        memcpy(R2, R1, 72);
        for (i = 0; i < 3; ++i) T1[i] = T2[i] = n1[i] = n2[i] = 0.0;
    } else {
        const double Ms00 = S_(1, 2) * S_(1, 2) - S_(1, 1) * S_(2, 2);
        const double Ms11 = S_(0, 2) * S_(0, 2) - S_(0, 0) * S_(2, 2);
        const double Ms22 = S_(0, 1) * S_(0, 1) - S_(0, 0) * S_(1, 1);
        const double sMs00 = sqrt(Ms00), sMs11 = sqrt(Ms11), sMs22 = sqrt(Ms22);
        const double trace = S_(0, 0) + (S_(1, 1) + S_(2, 2));          /* diagonal().sum(): unrolled non-vectorised tree */
        const double nu = 2.0 * sqrt(((1 + trace - Ms00) - Ms11) - Ms22);
        const double tenormsq = (2 + trace) - nu;
        double ts1[3], ts2[3], nn1, nn2, R1m[9], R2m[9], Id[9];
        if (S_(0, 0) > S_(1, 1) && S_(0, 0) > S_(2, 2)) {
            const double e = (((S_(0, 1) * S_(0, 2) - S_(0, 0) * S_(1, 2)) < 0) ? -1 : 1);
            n1[0] = S_(0, 0); n1[1] = S_(0, 1) + sMs22; n1[2] = S_(0, 2) + e * sMs11;
            n2[0] = S_(0, 0); n2[1] = S_(0, 1) - sMs22; n2[2] = S_(0, 2) - e * sMs11;
            nn1 = v3_norm(n1); nn2 = v3_norm(n2);
            for (i = 0; i < 3; ++i) { ts1[i] = (nn1 * n2[i]) / S_(0, 0); ts2[i] = (nn2 * n1[i]) / S_(0, 0); }
        } else if (S_(1, 1) > S_(0, 0) && S_(1, 1) > S_(2, 2)) {
            const double e = (((S_(1, 1) * S_(0, 2) - S_(0, 1) * S_(1, 2)) < 0) ? -1 : 1);
            n1[0] = S_(0, 1) + sMs22; n1[1] = S_(1, 1); n1[2] = S_(1, 2) - e * sMs00;
            n2[0] = S_(0, 1) - sMs22; n2[1] = S_(1, 1); n2[2] = S_(1, 2) + e * sMs00;
            nn1 = v3_norm(n1); nn2 = v3_norm(n2);
            for (i = 0; i < 3; ++i) { ts2[i] = (nn2 * n1[i]) / S_(1, 1); ts1[i] = (nn1 * n2[i]) / S_(1, 1); }
        } else {
            const double e = (((S_(1, 2) * S_(0, 2) - S_(0, 1) * S_(2, 2)) < 0) ? -1 : 1);
            n1[0] = S_(0, 2) + e * sMs11; n1[1] = S_(1, 2) + sMs00; n1[2] = S_(2, 2);
            n2[0] = S_(0, 2) - e * sMs11; n2[1] = S_(1, 2) - sMs00; n2[2] = S_(2, 2);
            nn1 = v3_norm(n1); nn2 = v3_norm(n2);
            for (i = 0; i < 3; ++i) { ts1[i] = (nn1 * n2[i]) / S_(2, 2); ts2[i] = (nn2 * n1[i]) / S_(2, 2); }
        }
        v3_normalize(n1); v3_normalize(n2);
        for (i = 0; i < 3; ++i) { ts1[i] -= tenormsq * n1[i]; ts2[i] -= tenormsq * n2[i]; }
        for (i = 0; i < 9; ++i) Id[i] = (i % 4 == 0) ? 1.0 : 0.0;
        {   /* Hn * (Identity - (tstar / nu) * n^T) */
            double Q1[9], Q2[9];
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) { Q1[i + 3 * j] = Id[i + 3 * j] - (ts1[i] / nu) * n1[j]; Q2[i + 3 * j] = Id[i + 3 * j] - (ts2[i] / nu) * n2[j]; }
            ok_m3_mul(Hn, Q1, R1m); ok_m3_mul(Hn, Q2, R2m);
        }
        for (i = 0; i < 3; ++i) { ts1[i] *= 0.5; ts2[i] *= 0.5; }
        memcpy(R1, R1m, 72); memcpy(R2, R2m, 72);
        ok_m3_mulv(R1, ts1, T1); ok_m3_mulv(R2, ts2, T2);
    }
#undef S_
    return !pure;
}

void rd_solve_homography_4pt(const double p1[4][2], const double p2[4][2], double H[9]) {
    static const double sqrt2 = 1.4142135623730951;   /* sqrt(2.0) */
    double ma[2] = {0, 0}, mb[2] = {0, 0}, sa = 0, sb = 0, na[4][2], nb[4][2], A[72], V[81], sv[9], NH[9], Na[9], Nb[9], t[9];
    int i, rank;
    for (i = 0; i < 4; ++i) { ma[0] += p1[i][0]; ma[1] += p1[i][1]; mb[0] += p2[i][0]; mb[1] += p2[i][1]; }
    ma[0] /= 4; ma[1] /= 4; mb[0] /= 4; mb[1] /= 4;
    for (i = 0; i < 4; ++i) {
        const double da[2] = {p1[i][0] - ma[0], p1[i][1] - ma[1]}, db[2] = {p2[i][0] - mb[0], p2[i][1] - mb[1]};
        sa += sqrt(da[0] * da[0] + da[1] * da[1]);
        sb += sqrt(db[0] * db[0] + db[1] * db[1]);
    }
    sa = 1.0 / (sqrt2 * sa);
    sb = 1.0 / (sqrt2 * sb);
    for (i = 0; i < 4; ++i) {
        na[i][0] = (p1[i][0] - ma[0]) * sa; na[i][1] = (p1[i][1] - ma[1]) * sa;
        nb[i][0] = (p2[i][0] - mb[0]) * sb; nb[i][1] = (p2[i][1] - mb[1]) * sb;
    }
    memset(A, 0, sizeof A);
#define A_(r, c) A[(r) + 8 * (c)]
    for (i = 0; i < 4; ++i) {
        const double* a = na[i]; const double* b = nb[i];
        A_(i * 2, 1) = -a[0];  A_(i * 2, 2) = a[0] * b[1];  A_(i * 2, 4) = -a[1];  A_(i * 2, 5) = a[1] * b[1];  A_(i * 2, 7) = -1;  A_(i * 2, 8) = b[1];
        A_(i * 2 + 1, 0) = a[0];  A_(i * 2 + 1, 2) = -a[0] * b[0];  A_(i * 2 + 1, 3) = a[1];  A_(i * 2 + 1, 5) = -a[1] * b[0];  A_(i * 2 + 1, 6) = 1;  A_(i * 2 + 1, 8) = -b[0];
    }
#undef A_
    rd_jacobisvd_Nxc_v(A, 8, 9, V, sv, &rank);
    for (i = 0; i < 9; ++i) NH[i] = V[i + 9 * 8];                  /* to_matrix(V.col(8)): column-major fill of the segments */
    /* Nb << 1/sb, 0, mb0, 0, 1/sb, mb1, 0, 0, 1 ; Na << sa, 0, -sa*ma0, 0, sa, -sa*ma1, 0, 0, 1 (row-major comma initialiser) */
    Nb[0] = 1 / sb; Nb[3] = 0; Nb[6] = mb[0]; Nb[1] = 0; Nb[4] = 1 / sb; Nb[7] = mb[1]; Nb[2] = 0; Nb[5] = 0; Nb[8] = 1;
    Na[0] = sa; Na[3] = 0; Na[6] = -sa * ma[0]; Na[1] = 0; Na[4] = sa; Na[7] = -sa * ma[1]; Na[2] = 0; Na[5] = 0; Na[8] = 1;
    ok_m3_mul(Nb, NH, t);
    ok_m3_mul(t, Na, H);
}

/* ------------------------------------------------------------------ stereo.h */
static void tri_rows(const double* P, const double pt[3], double* A, int ld, int r0) {   /* two rows of A, column c at A[r + ld*c] */
    int c;
    for (c = 0; c < 4; ++c) {
        A[r0 + ld * c] = pt[0] * P[2 + 3 * c] - pt[2] * P[0 + 3 * c];
        A[r0 + 1 + ld * c] = pt[1] * P[2 + 3 * c] - pt[2] * P[1 + 3 * c];
    }
}

void rd_triangulate_point2(const double P1[12], const double P2[12], const double pt1[3], const double pt2[3], double out[4]) {
    double A[16], U[16], V[16], sv[4];
    tri_rows(P1, pt1, A, 4, 0);
    tri_rows(P2, pt2, A, 4, 2);
    sv_eigen_jacobisvd_4x4(A, U, V, sv);
    memcpy(out, V + 12, 32);
}

void rd_triangulate_point_n(int n, const double* Ps, const double* pts, double out[4]) {
    double A[4 * 2 * 64], V[16], sv[4];
    double* Ad = A; int i, rank;
    double* heap = 0;
    if (n > 64) { heap = (double*)malloc(sizeof(double) * 8 * (size_t)n); Ad = heap; }
    for (i = 0; i < n; ++i) tri_rows(Ps + 12 * i, pts + 3 * i, Ad, 2 * n, 2 * i);
    rd_jacobisvd_Nxc_v(Ad, 2 * n, 4, V, sv, &rank);
    memcpy(out, V + 12, 32);
    if (heap) free(heap);
}

/* ------------------------------------------------------------------ track.cpp */
void rd_frame_get_pose(const ok_quat* pose_q, const double pose_p[3], const ok_quat* cam_q, const double cam_p[3], ok_quat* q, double p[3]) {
    double r[3];
    int i;
    ok_quat_mul(pose_q, cam_q, q);
    rd_quat_rotate(pose_q, cam_p, r);
    for (i = 0; i < 3; ++i) p[i] = pose_p[i] + r[i];
}

int rd_track_triangulate(int n, const rd_obs* obs, double landmark[3]) {
    double* Ps = (double*)malloc(sizeof(double) * 12 * (size_t)(n ? n : 1));
    double* pts = (double*)malloc(sizeof(double) * 3 * (size_t)(n ? n : 1));
    double h[4];
    int i, valid = 1;
    for (i = 0; i < n; ++i) {
        ok_quat q; double p[3], R[9], Rp[3], qc_[1];
        ok_quat qc;
        (void)qc_;
        rd_frame_get_pose(&obs[i].pose_q, obs[i].pose_p, &obs[i].cam_q, obs[i].cam_p, &q, p);
        qc = rd_quat_conj(q);
        ok_quat_to_mat3(&qc, R);                      /* pose.q.conjugate().matrix() */
        ok_m3_mulv(R, p, Rp);                         /* R * pose.p */
        memcpy(Ps + 12 * i, R, 72);
        Ps[12 * i + 9] = -Rp[0]; Ps[12 * i + 10] = -Rp[1]; Ps[12 * i + 11] = -Rp[2];   /* T = -(R * p) */
        memcpy(pts + 3 * i, obs[i].keypoint, 24);
    }
    rd_triangulate_point_n(n, Ps, pts, h);
    for (i = 0; i < n; ++i) {
        const double* P = Ps + 12 * i;
        const double q2 = (P[2] * h[0] + P[5] * h[1]) + (P[8] * h[2] + P[11] * h[3]);   /* (Ps[i] * hlandmark)[2] : scalar row, halving tree */
        if (!(q2 * h[3] > 0)) { valid = 0; break; }
    }
    if (valid) { landmark[0] = h[0] / h[3]; landmark[1] = h[1] / h[3]; landmark[2] = h[2] / h[3]; }
    free(Ps); free(pts);
    return valid;
}

static void stable_normalized(const double v[3], double out[3]) {   /* MatrixBase::stableNormalized() */
    double m = fabs(v[0]);
    if (fabs(v[1]) > m) m = fabs(v[1]);
    if (fabs(v[2]) > m) m = fabs(v[2]);
    {
        const double a = v[0] / m, b = v[1] / m, c = v[2] / m;
        const double z = (a * a + b * b) + c * c;
        if (z > 0.0) { const double d = sqrt(z) * m; out[0] = v[0] / d; out[1] = v[1] / d; out[2] = v[2] / d; }
        else { out[0] = v[0]; out[1] = v[1]; out[2] = v[2]; }
    }
}

double rd_track_triangulation_angle(int n, const rd_obs* obs, const double p[3]) {
    double nref[3], d[3], q[3], mx = 0.0;
    int i;
    ok_quat oq; double op[3];
    rd_frame_get_pose(&obs[0].pose_q, obs[0].pose_p, &obs[0].cam_q, obs[0].cam_p, &oq, op);
    for (i = 0; i < 3; ++i) d[i] = p[i] - op[i];
    stable_normalized(d, nref);
    for (i = 0; i < n; ++i) {
        double nn[3], dot, ang;
        int k;
        rd_frame_get_pose(&obs[i].pose_q, obs[i].pose_p, &obs[i].cam_q, obs[i].cam_p, &oq, op);
        for (k = 0; k < 3; ++k) q[k] = p[k] - op[k];
        stable_normalized(q, nn);
        dot = (nn[0] * nref[0] + nn[1] * nref[1]) + nn[2] * nref[2];
        ang = acos(dot);
        if (mx < ang) mx = ang;                          /* std::max(max_angle, angle) returns the larger; ties/NaN keep max_angle */
    }
    return mx;
}

void rd_track_get_landmark_point(const rd_obs* first, double inv_depth, double out[3]) {
    ok_quat q; double p[3], r[3];
    int i;
    rd_frame_get_pose(&first->pose_q, first->pose_p, &first->cam_q, first->cam_p, &q, p);
    rd_quat_rotate(&q, first->keypoint, r);
    for (i = 0; i < 3; ++i) out[i] = r[i] / inv_depth + p[i];
}

double rd_track_set_landmark_point(const rd_obs* first, const double pt[3]) {
    ok_quat q, qc; double p[3], d[3], r[3];
    int i;
    rd_frame_get_pose(&first->pose_q, first->pose_p, &first->cam_q, first->cam_p, &q, p);
    for (i = 0; i < 3; ++i) d[i] = pt[i] - p[i];
    qc = rd_quat_conj(q);
    rd_quat_rotate(&qc, d, r);
    return 1.0 / ok_v3_norm(r);
}
