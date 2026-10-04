/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Eigen 3.4.0 JacobiSVD<Matrix<double, R, C>, ComputeFullV> (V and singular values only), derived from stella_port/c/sv_eigen_svd.c
 * (MPL-2.0, itself a model of Eigen JacobiSVD.h / SVDBase.h / Jacobi.h / RealSvd2x2.h) generalised from "N x 9" to "rows x cols"
 * (cols <= 9) for the RD-VIO shapes: fixed 5x9 and 8x9 (more columns than rows: QR of the adjoint) and Nx4 (dynamic rows).
 * Uses rdvio_port/c/rd_qr.c (a copy of stella_port/c/sv_eigen_qr.c with the dot_mode parameter).
 */
#include "rd_svd.h"
#include "rd_qr.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SV_EPS 2.2204460492503131e-16
#define SV_MIN DBL_MIN

static inline double *cp(double *base, int ld, int col) { return base + (size_t)col * (size_t)ld; }
static inline const double *ccp(const double *base, int ld, int col) { return base + (size_t)col * (size_t)ld; }

/* JacobiRotation<double>::makeJacobi(x,y,z) -- Jacobi.h line 92. */
static void make_jacobi(double x, double y, double z, double *c, double *s) {
    double deno = 2.0 * fabs(y);
    if (deno < SV_MIN) {
        *c = 1.0;
        *s = 0.0;
        return;
    }
    double tau = (x - z) / deno;
    double w = sqrt(tau * tau + 1.0);
    double t = (tau > 0.0) ? 1.0 / (tau + w) : 1.0 / (tau - w);
    double sign_t = (t > 0.0) ? 1.0 : -1.0;
    double n = 1.0 / sqrt(t * t + 1.0);
    *s = -sign_t * (y / fabs(y)) * fabs(t) * n;
    *c = n;
}

/* internal::real_2x2_jacobi_svd -- misc/RealSvd2x2.h. Real scalar case. */
static void real_2x2_jacobi_svd(double m00, double m01, double m10, double m11,
                                 double *lc, double *ls, double *rc, double *rs) {
    double t = m00 + m11;
    double d = m10 - m01;
    double rot1_c, rot1_s;
    if (fabs(d) < SV_MIN) {
        rot1_s = 0.0;
        rot1_c = 1.0;
    } else {
        double u = t / d;
        double tmp = sqrt(1.0 + u * u);
        rot1_s = 1.0 / tmp;
        rot1_c = u / tmp;
    }
    /* m.applyOnTheLeft(0,1,rot1) */
    double n00 = rot1_c * m00 + rot1_s * m10;
    double n01 = rot1_c * m01 + rot1_s * m11;
    double n11 = -rot1_s * m01 + rot1_c * m11;
    double rc_, rs_;
    make_jacobi(n00, n01, n11, &rc_, &rs_);
    /* j_left = rot1 * j_right.transpose() ; transpose() = (c, -s) */
    *lc = rot1_c * rc_ + rot1_s * rs_;
    *ls = rot1_s * rc_ - rot1_c * rs_;
    *rc = rc_;
    *rs = rs_;
}

/* MatrixBase::applyOnTheLeft(p,q,j): rotates rows p,q, looping the `ncols`
 * column entries of those rows (stride `ld` between successive entries). */
static void apply_rows(double *m, int ld, int ncols, int p, int q, double c, double s) {
    double *rp = m + p;
    double *rq = m + q;
    for (int i = 0; i < ncols; ++i) {
        double xi = rp[(size_t)i * ld];
        double yi = rq[(size_t)i * ld];
        rp[(size_t)i * ld] = c * xi + s * yi;
        rq[(size_t)i * ld] = -s * xi + c * yi;
    }
}

/* MatrixBase::applyOnTheRight(p,q,j): rotates columns p,q (each `nrows`
 * contiguous entries), using j.transpose() internally -- see Jacobi.h. */
static void apply_cols(double *m, int ld, int nrows, int p, int q, double c, double s) {
    double *cpp = cp(m, ld, p);
    double *cq = cp(m, ld, q);
    for (int i = 0; i < nrows; ++i) {
        double xi = cpp[i];
        double yi = cq[i];
        cpp[i] = c * xi - s * yi;
        cq[i] = s * xi + c * yi;
    }
}

/* Shared engine for JacobiSVD::compute() steps 2-4 (main sweep, sign fix,
 * sort) once workMatrix/U/V have been initialized by the (square or
 * QR-preconditioned) step 1. `work` is n x n (ld n). `U`, if non-NULL, is
 * Urows x n (ld Urows, Urows==n for our full-U square/QR-preconditioned
 * uses). `V`, if non-NULL, is Vrows x (>=n) (ld Vrows; only its first n
 * columns are touched, matching diagSize < cols for the N==8 shape). */
static void jacobi_svd_core(double *work, int n,
                             double *U, int Urows,
                             double *V, int Vrows,
                             double *sv, double scale, int *nonzero_out) {
    double maxDiagEntry = 0.0;
    for (int i = 0; i < n; ++i) {
        double a = fabs(work[i + (size_t)i * n]);
        if (a > maxDiagEntry) maxDiagEntry = a;
    }
    const double precision = 2.0 * SV_EPS;

    int finished = 0;
    while (!finished) {
        finished = 1;
        for (int p = 1; p < n; ++p) {
            for (int q = 0; q < p; ++q) {
                double threshold = precision * maxDiagEntry;
                if (threshold < SV_MIN) threshold = SV_MIN;
                double wpq = work[p + (size_t)q * n];
                double wqp = work[q + (size_t)p * n];
                if (fabs(wpq) > threshold || fabs(wqp) > threshold) {
                    finished = 0;
                    double mpp = work[p + (size_t)p * n];
                    double mqq = work[q + (size_t)q * n];
                    double lc, ls, rc, rs;
                    real_2x2_jacobi_svd(mpp, wpq, wqp, mqq, &lc, &ls, &rc, &rs);

                    apply_rows(work, n, n, p, q, lc, ls);
                    /* U.applyOnTheRight(p,q, j_left.transpose()): the
                     * transpose passed in cancels the one apply_cols()
                     * already bakes in for applyOnTheRight's own
                     * convention, leaving the *direct* (lc,ls) formula on
                     * U's columns -- i.e. pass (lc,-ls) here, not (lc,ls),
                     * so apply_cols's internal (c,-s) becomes (lc,ls). */
                    if (U) apply_cols(U, Urows, Urows, p, q, lc, -ls);

                    apply_cols(work, n, n, p, q, rc, rs);
                    if (V) apply_cols(V, Vrows, Vrows, p, q, rc, rs);

                    double a1 = fabs(work[p + (size_t)p * n]);
                    double a2 = fabs(work[q + (size_t)q * n]);
                    if (a1 > maxDiagEntry) maxDiagEntry = a1;
                    if (a2 > maxDiagEntry) maxDiagEntry = a2;
                }
            }
        }
    }

    for (int i = 0; i < n; ++i) {
        double a = work[i + (size_t)i * n];
        sv[i] = fabs(a);
        if (U && a < 0.0) {
            double *ci = cp(U, Urows, i);
            for (int r = 0; r < Urows; ++r) ci[r] = -ci[r];
        }
    }
    for (int i = 0; i < n; ++i) sv[i] *= scale;

    int nonzero = n;
    for (int i = 0; i < n; ++i) {
        int pos = i;
        double best = sv[i];
        for (int j = i + 1; j < n; ++j) {
            if (sv[j] > best) { best = sv[j]; pos = j; }
        }
        if (best == 0.0) { nonzero = i; break; }
        if (pos != i) {
            double t = sv[i]; sv[i] = sv[pos]; sv[pos] = t;
            if (U) {
                double *ci = cp(U, Urows, i), *cpos = cp(U, Urows, pos);
                for (int r = 0; r < Urows; ++r) { double tt = ci[r]; ci[r] = cpos[r]; cpos[r] = tt; }
            }
            if (V) {
                double *ci = cp(V, Vrows, i), *cpos = cp(V, Vrows, pos);
                for (int r = 0; r < Vrows; ++r) { double tt = ci[r]; ci[r] = cpos[r]; cpos[r] = tt; }
            }
        }
    }
    if (nonzero_out) *nonzero_out = nonzero;
}

void rd_jacobisvd_Nxc_v(const double *A, int N, int cols, double *V, double *sv, int *rank_out) {
    int n = (N < cols) ? N : cols; /* diagSize */
    const int dot_mode = (cols < 8) ? 1 : 0;   /* Max column count of the (QR / V) matrix < 8 selects the coefficient-based product, see rd_qr.c */

    double scale = 0.0;
    for (long i = 0; i < (long)N * cols; ++i) { double a = fabs(A[i]); if (a > scale) scale = a; }
    if (scale == 0.0) scale = 1.0;

    double work[81];
    memset(work, 0, sizeof(work));
    memset(V, 0, (size_t)cols * cols * sizeof(double));

    if (N > cols) {
        double *scaled = (double *)malloc((size_t)N * cols * sizeof(double));
        for (long i = 0; i < (long)N * cols; ++i) scaled[i] = A[i] / scale;

        double hcoeffs[9];
        int perm[9];
        int nz;
        double maxpiv;
        rd_qr_colpiv(scaled, N, cols, hcoeffs, perm, &nz, &maxpiv, dot_mode);

        for (int c = 0; c < cols; ++c)
            for (int r = 0; r <= c; ++r)
                work[r + (size_t)c * n] = scaled[r + (size_t)c * N];
        for (int j = 0; j < cols; ++j) V[perm[j] + (size_t)j * cols] = 1.0;
        free(scaled);
    } else if (N == cols) {
        for (long i = 0; i < (long)N * cols; ++i) work[i] = A[i] / scale;
        for (int i = 0; i < cols; ++i) V[i + (size_t)i * cols] = 1.0;
    } else {
        /* N < 9: precondition via QR of the adjoint (cols x N == 9 x N,
         * "more rows than cols" since N < cols). */
        double *adj = (double *)malloc((size_t)cols * N * sizeof(double));
        for (int i = 0; i < cols; ++i)
            for (int j = 0; j < N; ++j)
                adj[i + (size_t)j * cols] = A[j + (size_t)i * N] / scale;

        double hcoeffs[9];
        int *permN = (int *)malloc((size_t)N * sizeof(int));
        int nz;
        double maxpiv;
        rd_qr_colpiv(adj, cols, N, hcoeffs, permN, &nz, &maxpiv, (N < 8) ? 1 : 0);   /* QR matrix is Matrix<double,cols,N>: MaxCols = N */

        /* workMatrix = R(top N x N block).triangularView<Upper>().adjoint() */
        for (int c = 0; c < N; ++c)
            for (int r = 0; r <= c; ++r)
                work[c + (size_t)r * n] = adj[r + (size_t)c * cols];

        rd_qr_colpiv_householderq_full(adj, cols, N, hcoeffs, V, dot_mode);

        free(adj);
        free(permN);
    }

    int nonzero;
    jacobi_svd_core(work, n, NULL, 0, V, cols, sv, scale, &nonzero);
    for (int i = n; i < cols; ++i) sv[i] = 0.0;

    double diagSize_thr = (n > 1) ? (double)n : 1.0;
    double threshold = diagSize_thr * SV_EPS;
    double premultiplied = sv[0] * threshold;
    if (premultiplied < SV_MIN) premultiplied = SV_MIN;
    int i = nonzero - 1;
    while (i >= 0 && sv[i] < premultiplied) --i;
    if (rank_out) *rank_out = i + 1;
}
