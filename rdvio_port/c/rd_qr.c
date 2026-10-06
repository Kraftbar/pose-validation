/* SPDX-License-Identifier: MPL-2.0 */
/* Copy of stella_port/c/sv_eigen_qr.c (MPL-2.0) with symbols renamed to rd_qr_* and one added parameter, dot_mode (see
 * apply_householder_left), needed for Eigen::ColPivHouseholderQR<Matrix<double,Dynamic,4>> (RD-VIO multi-view triangulation). */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * See sv_eigen_qr.h for the Eigen sources this follows.
 */
#include "rd_qr.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SV_EPS 2.2204460492503131e-16 /* NumTraits<double>::epsilon() */
#define SV_MIN DBL_MIN                /* numeric_limits<double>::min() */

static inline double *col_ptr(double *base, int ld, int col) { return base + (size_t)col * (size_t)ld; }
static inline const double *ccol_ptr(const double *base, int ld, int col) { return base + (size_t)col * (size_t)ld; }

/* --------------------------------------------------------------------
 * Reduction over a contiguous run of doubles, following the shape of
 * Eigen's redux_impl<Func,Evaluator,LinearVectorizedTraversal,NoUnrolling>
 * for PacketSize==2 (SSE2 double), Func==scalar_sum_op applied to the
 * elementwise-squared expression (i.e. squaredNorm()/tailSqNorm()) -- see
 * Eigen/src/Core/Redux.h.
 *
 * KNOWN GAP (see stella_port/HANDOVER.md / the sv_eigen_svd porting
 * report): Redux.h's alignedStart is internal::first_default_aligned(xpr),
 * a *runtime* pointer-alignment probe. Empirically (bisected against real
 * Eigen's ColPivHouseholderQR on the N x 9 shapes this port targets, via
 * a side-by-side R-matrix dump), forcing alignedStart=0 unconditionally
 * -- i.e. assuming the vectorized pair-up always starts at the block's
 * first element, regardless of the column/tail's actual runtime offset --
 * matches Eigen exactly for the N==8 (transpose/adjoint) and N==9
 * (square, no QR) shapes and for most, but not all, of N in {16,100,500}.
 * The column/tail Block this is called on is a *dynamic*-offset Block of
 * a heap MatrixXd, so its evaluator's Alignment is compile-time Unaligned;
 * this constant-0 approximation is very likely masking that Eigen's
 * evaluator-level alignment trait (not just the runtime pointer used in
 * first_default_aligned) actually disables the aligned-pairing path for
 * such blocks -- i.e. Eigen probably takes a start-from-0 vectorized path
 * here regardless of pointer alignment, rather than genuinely 16-byte
 * probing. Left as the best-validated approximation; see the porting
 * report for the residual ~1-in-4 value mismatches this still leaves on
 * N=16/100/500 (root-caused to this function and/or the GEMV pairing in
 * apply_householder_left below, not to the QR/SVD algorithm structure,
 * pivoting or convergence logic, all of which were verified exact).
 * -------------------------------------------------------------------- */
static double redux_sumsq(const double *v, long n) {
    if (n <= 0) return 0.0;
    long alignedStart = 0;
    if (alignedStart > n) alignedStart = n;
    const long packetSize = 2;
    long alignedSize = ((n - alignedStart) / packetSize) * packetSize;
    long alignedSize2 = ((n - alignedStart) / (2 * packetSize)) * (2 * packetSize);
    long alignedEnd2 = alignedStart + alignedSize2;
    long alignedEnd = alignedStart + alignedSize;
    double res;
    if (alignedSize) {
        double p0_0 = v[alignedStart] * v[alignedStart];
        double p0_1 = v[alignedStart + 1] * v[alignedStart + 1];
        if (alignedSize > packetSize) {
            double p1_0 = v[alignedStart + 2] * v[alignedStart + 2];
            double p1_1 = v[alignedStart + 3] * v[alignedStart + 3];
            long idx;
            for (idx = alignedStart + 4; idx < alignedEnd2; idx += 4) {
                p0_0 += v[idx] * v[idx];
                p0_1 += v[idx + 1] * v[idx + 1];
                p1_0 += v[idx + 2] * v[idx + 2];
                p1_1 += v[idx + 3] * v[idx + 3];
            }
            p0_0 += p1_0;
            p0_1 += p1_1;
            if (alignedEnd > alignedEnd2) {
                p0_0 += v[alignedEnd2] * v[alignedEnd2];
                p0_1 += v[alignedEnd2 + 1] * v[alignedEnd2 + 1];
            }
        }
        res = p0_0 + p0_1;
        { long idx; for (idx = 0; idx < alignedStart; ++idx) res += v[idx] * v[idx]; }
        { long idx; for (idx = alignedEnd; idx < n; ++idx) res += v[idx] * v[idx]; }
    } else {
        res = v[0] * v[0];
        long idx;
        for (idx = 1; idx < n; ++idx) res += v[idx] * v[idx];
    }
    return res;
}

/* Same LinearVectorizedTraversal/NoUnrolling shape as redux_sumsq, but for
 * a genuine two-operand dot product (Func==scalar_sum_op over the
 * elementwise a[i]*b[i] expression). This is the path Eigen actually takes
 * for `essential.adjoint() * bottom` when `bottom` has exactly one column:
 * confirmed by instrumenting a scratch copy of Eigen (Redux.h / Dot.h) at
 * runs/stella_port/reference_init/eigen_instrumented and tracing a failing
 * fixture -- for ncols==1 the product evaluator dispatches to
 * dot_nocheck<...>::run (`a.transpose().binaryExpr<conj_prod>(b).sum()`),
 * i.e. plain redux over a product expression, NOT the RowMajor
 * general_matrix_vector_product kernel used for ncols>=2 (see
 * apply_householder_left below). Its stride-4 two-accumulator grouping
 * (packet_res0 over indices {0,1,4,5,8,9,...}, packet_res1 over
 * {2,3,6,7,...}, merged at the end) differs from the GEMV kernel's simple
 * consecutive-pair (lane0=evens, lane1=odds) grouping used elsewhere in
 * this file -- a 1-ULP-class mismatch traced to exactly this distinction
 * (R[7,8] on one N=16 fixture, always at the k==cols-2 step where the
 * trailing block is 1 column wide). */
static double redux_dot(const double *a, const double *b, long n) {
    if (n <= 0) return 0.0;
    const long packetSize = 2;
    long alignedSize = (n / packetSize) * packetSize;
    long alignedSize2 = (n / (2 * packetSize)) * (2 * packetSize);
    double res;
    if (alignedSize) {
        double p0_0 = a[0] * b[0];
        double p0_1 = a[1] * b[1];
        if (alignedSize > packetSize) {
            double p1_0 = a[2] * b[2];
            double p1_1 = a[3] * b[3];
            long idx;
            for (idx = 4; idx < alignedSize2; idx += 4) {
                p0_0 += a[idx] * b[idx];
                p0_1 += a[idx + 1] * b[idx + 1];
                p1_0 += a[idx + 2] * b[idx + 2];
                p1_1 += a[idx + 3] * b[idx + 3];
            }
            p0_0 += p1_0;
            p0_1 += p1_1;
            if (alignedSize > alignedSize2) {
                p0_0 += a[alignedSize2] * b[alignedSize2];
                p0_1 += a[alignedSize2 + 1] * b[alignedSize2 + 1];
            }
        }
        res = p0_0 + p0_1;
        { long idx; for (idx = alignedSize; idx < n; ++idx) res += a[idx] * b[idx]; }
    } else {
        res = a[0] * b[0];
        long idx;
        for (idx = 1; idx < n; ++idx) res += a[idx] * b[idx];
    }
    return res;
}

static double col_norm(const double *base, int ld, int col, int row0, int n) {
    return sqrt(redux_sumsq(ccol_ptr(base, ld, col) + row0, n));
}

/* --------------------------------------------------------------------
 * makeHouseholderInPlace: Eigen/src/Householder/Householder.h.
 * Operates on x = column `col` of `base`, rows [row0, row0+len).
 * On return: x[0] -> beta (returned), x[1..] -> essential vector, *tau set.
 * -------------------------------------------------------------------- */
static double make_householder_in_place(double *base, int ld, int col, int row0, int len, double *tau) {
    double *x = col_ptr(base, ld, col) + row0;
    double c0 = x[0];
    double beta;
    if (len == 1) {
        *tau = 0.0;
        beta = c0;
        return beta;
    }
    double tailSqNorm = redux_sumsq(x + 1, len - 1);
    if (tailSqNorm <= SV_MIN) {
        *tau = 0.0;
        beta = c0;
        for (int i = 1; i < len; ++i) x[i] = 0.0;
        return beta;
    }
    beta = sqrt(c0 * c0 + tailSqNorm);
    if (c0 >= 0.0) beta = -beta;
    double denom = c0 - beta;
    for (int i = 1; i < len; ++i) x[i] = x[i] / denom;
    *tau = (beta - c0) / beta;
    return beta;
}

/* --------------------------------------------------------------------
 * applyHouseholderOnTheLeft on the block base[r0..r0+nrows, c0..c0+ncols),
 * leading dimension ld, essential vector given explicitly (length nrows-1).
 * tmp is scratch, length >= ncols.
 * The `tmp[j] = dot(essential, bottom_col_j)` step replicates Eigen's
 * general_matrix_vector_product<...,RowMajor,...>::run kernel (see
 * GeneralMatrixVector.h): for double/SSE2 it always pairs up (j,j+1) from
 * index 0 (no alignment probing -- the kernel hardcodes Unaligned loads),
 * accumulates two lanes independently, adds them (predux) after the last
 * full pair, then appends any single leftover element -- NOT the same
 * alignedStart-aware splitting used by plain redux (squaredNorm/norm).
 * -------------------------------------------------------------------- */
static void apply_householder_left(double *base, int ld, int r0, int c0, int nrows, int ncols,
                                    const double *essential, double tau, double *tmp, double *scaled_essential, int dot_mode) {
    if (nrows == 1) {
        double f = 1.0 - tau;
        for (int j = 0; j < ncols; ++j) *(col_ptr(base, ld, c0 + j) + r0) *= f;
        return;
    }
    if (tau == 0.0) return;
    int n = nrows - 1;
    if (ncols == 1 || dot_mode) {
        /* dot_mode (rd_port): the matrix type has a SMALL compile-time column bound (Matrix<double,Dynamic,4>), so
         * essential.adjoint() * bottom is <1,Small,Large> = CoeffBasedProductMode: every tmp(j) is a dynamic redux over the product
         * expression (redux_dot), never the GEMV kernel (that is <1,Large,Large>, MaxCols >= 8: Nx9).
         * Eigen dispatches `essential.adjoint() * bottom` to dot_nocheck
         * (plain redux over a product expression), not the GEMV kernel,
         * when the "matrix" operand collapses to a single column. */
        int jj;
        for (jj = 0; jj < ncols; ++jj) {
            const double *bcol = ccol_ptr(base, ld, c0 + jj) + (r0 + 1);
            tmp[jj] = redux_dot(essential, bcol, n);
        }
    } else {
        for (int j = 0; j < ncols; ++j) {
            const double *bcol = ccol_ptr(base, ld, c0 + j) + (r0 + 1);
            double lane0 = 0.0, lane1 = 0.0;
            int i = 0;
            for (; i + 2 <= n; i += 2) {
                lane0 += essential[i] * bcol[i];
                lane1 += essential[i + 1] * bcol[i + 1];
            }
            double cc = lane0 + lane1;
            for (; i < n; ++i) cc += essential[i] * bcol[i];
            tmp[j] = cc;
        }
    }
    for (int j = 0; j < ncols; ++j) tmp[j] += *(col_ptr(base, ld, c0 + j) + r0);
    for (int j = 0; j < ncols; ++j) *(col_ptr(base, ld, c0 + j) + r0) -= tau * tmp[j];
    for (int i = 0; i < n; ++i) scaled_essential[i] = tau * essential[i];
    for (int j = 0; j < ncols; ++j) {
        double *bcol = col_ptr(base, ld, c0 + j) + (r0 + 1);
        double t = tmp[j];
        for (int i = 0; i < n; ++i) bcol[i] -= scaled_essential[i] * t;
    }
}

void rd_qr_colpiv(double *qr, int rows, int cols,
                         double *hcoeffs, int *perm,
                         int *nonzero_pivots_out, double *maxpivot_out, int dot_mode) {
    int size = rows < cols ? rows : cols;
    double *colNormsUpdated = (double *)malloc((size_t)cols * sizeof(double));
    double *colNormsDirect = (double *)malloc((size_t)cols * sizeof(double));
    int *transpositions = (int *)malloc((size_t)cols * sizeof(int));
    double *tmp = (double *)malloc((size_t)cols * sizeof(double));
    double *scaled = (double *)malloc((size_t)rows * sizeof(double));

    for (int k = 0; k < cols; ++k) {
        colNormsDirect[k] = col_norm(qr, rows, k, 0, rows);
        colNormsUpdated[k] = colNormsDirect[k];
    }

    double maxColNorm = colNormsUpdated[0];
    for (int k = 1; k < cols; ++k) if (colNormsUpdated[k] > maxColNorm) maxColNorm = colNormsUpdated[k];
    double threshold_helper = (maxColNorm * SV_EPS) * (maxColNorm * SV_EPS) / (double)rows;
    double norm_downdate_threshold = sqrt(SV_EPS);

    int nonzero_pivots = size;
    double maxpivot = 0.0;

    for (int k = 0; k < size; ++k) {
        int biggest_col_index = k;
        double biggest_col_val = colNormsUpdated[k];
        for (int j = k + 1; j < cols; ++j) {
            if (colNormsUpdated[j] > biggest_col_val) { biggest_col_val = colNormsUpdated[j]; biggest_col_index = j; }
        }
        double biggest_col_sq_norm = biggest_col_val * biggest_col_val;

        if (nonzero_pivots == size && biggest_col_sq_norm < threshold_helper * (double)(rows - k)) {
            nonzero_pivots = k;
        }

        transpositions[k] = biggest_col_index;
        if (k != biggest_col_index) {
            double *ck = col_ptr(qr, rows, k);
            double *cb = col_ptr(qr, rows, biggest_col_index);
            for (int i = 0; i < rows; ++i) { double t = ck[i]; ck[i] = cb[i]; cb[i] = t; }
            double t;
            t = colNormsUpdated[k]; colNormsUpdated[k] = colNormsUpdated[biggest_col_index]; colNormsUpdated[biggest_col_index] = t;
            t = colNormsDirect[k]; colNormsDirect[k] = colNormsDirect[biggest_col_index]; colNormsDirect[biggest_col_index] = t;
        }

        double tau;
        double beta = make_householder_in_place(qr, rows, k, k, rows - k, &tau);
        hcoeffs[k] = tau;
        *(col_ptr(qr, rows, k) + k) = beta;
        if (fabs(beta) > maxpivot) maxpivot = fabs(beta);

        double *essential = col_ptr(qr, rows, k) + (k + 1); /* length rows-k-1 */
        apply_householder_left(qr, rows, k, k + 1, rows - k, cols - k - 1, essential, tau, tmp, scaled, dot_mode);

        for (int j = k + 1; j < cols; ++j) {
            if (colNormsUpdated[j] != 0.0) {
                double temp = fabs(*(col_ptr(qr, rows, j) + k)) / colNormsUpdated[j];
                temp = (1.0 + temp) * (1.0 - temp);
                temp = temp < 0.0 ? 0.0 : temp;
                double ratio = colNormsUpdated[j] / colNormsDirect[j];
                double temp2 = temp * (ratio * ratio);
                if (temp2 <= norm_downdate_threshold) {
                    colNormsDirect[j] = col_norm(qr, rows, j, k + 1, rows - k - 1);
                    colNormsUpdated[j] = colNormsDirect[j];
                } else {
                    colNormsUpdated[j] *= sqrt(temp);
                }
            }
        }
    }

    for (int j = 0; j < cols; ++j) perm[j] = j;
    for (int k = 0; k < size; ++k) {
        int t = perm[k]; perm[k] = perm[transpositions[k]]; perm[transpositions[k]] = t;
    }

    if (nonzero_pivots_out) *nonzero_pivots_out = nonzero_pivots;
    if (maxpivot_out) *maxpivot_out = maxpivot;

    free(colNormsUpdated);
    free(colNormsDirect);
    free(transpositions);
    free(tmp);
    free(scaled);
}

void rd_qr_colpiv_householderq_full(const double *qr, int rows, int cols,
                                           const double *hcoeffs, double *Q, int dot_mode) {
    int vecs = rows < cols ? rows : cols; /* m_length == diagonalSize of the QR */
    memset(Q, 0, (size_t)rows * (size_t)rows * sizeof(double));
    for (int i = 0; i < rows; ++i) *(col_ptr(Q, rows, i) + i) = 1.0;

    double *tmp = (double *)malloc((size_t)rows * sizeof(double));
    double *scaled = (double *)malloc((size_t)rows * sizeof(double));
    double *essential = (double *)malloc((size_t)rows * sizeof(double));

    for (int k = vecs - 1; k >= 0; --k) {
        int cornerSize = rows - k; /* m_shift == 1 for householderQ() of a QR */
        int n = cornerSize - 1;
        for (int i = 0; i < n; ++i) essential[i] = *(ccol_ptr(qr, rows, k) + (k + 1 + i));
        apply_householder_left(Q, rows, k, k, cornerSize, cornerSize, essential, hcoeffs[k], tmp, scaled, dot_mode);
    }

    free(tmp);
    free(scaled);
    free(essential);
}

/* --------------------------------------------------------------------
 * FullPivHouseholderQR<MatrixXd>::computeInPlace + _solve_impl (vector rhs).
 * -------------------------------------------------------------------- */
int rd_qr_fullpiv_solve(const double *A, int rows, int cols, const double *b, double *x) {
    const int size = rows < cols ? rows : cols;
    double *qr = (double *)malloc(sizeof(double) * (size_t)rows * (size_t)cols);
    double *hc = (double *)calloc((size_t)(size ? size : 1), sizeof(double));
    double *tmp = (double *)calloc((size_t)(cols + 1), sizeof(double));
    double *sess = (double *)calloc((size_t)(rows + 1), sizeof(double));
    double *c = (double *)malloc(sizeof(double) * (size_t)(rows ? rows : 1));
    int *rt = (int *)calloc((size_t)(size ? size : 1), sizeof(int));
    int *ct = (int *)calloc((size_t)(size ? size : 1), sizeof(int));
    int *perm = (int *)malloc(sizeof(int) * (size_t)(cols ? cols : 1));
    int k, i, j, nonzero = size, rank = 0;
    double biggest = 0.0, maxpivot = 0.0;
    const double precision = SV_EPS * (double)size;
    memcpy(qr, A, sizeof(double) * (size_t)rows * (size_t)cols);
    for (k = 0; k < size; ++k) {
        /* maxCoeff(&row, &col) over |corner|: init with (0,0), then column 0 rows 1.., then the other columns; strict '>' */
        int br = k, bc = k;
        double best = fabs(qr[(size_t)k * rows + k]), beta;
        for (j = k; j < cols; ++j)
            for (i = (j == k ? k + 1 : k); i < rows; ++i) {
                const double v = fabs(qr[(size_t)j * rows + i]);
                if (v > best) { best = v; br = i; bc = j; }
            }
        if (k == 0) biggest = best;
        if (best <= biggest * precision) {     /* isMuchSmallerThan(biggest_in_corner, biggest, precision) */
            nonzero = k;
            for (i = k; i < size; ++i) { rt[i] = i; ct[i] = i; hc[i] = 0.0; }
            break;
        }
        rt[k] = br; ct[k] = bc;
        if (k != br)
            for (j = k; j < cols; ++j) { const double t = qr[(size_t)j * rows + k]; qr[(size_t)j * rows + k] = qr[(size_t)j * rows + br]; qr[(size_t)j * rows + br] = t; }
        if (k != bc)
            for (i = 0; i < rows; ++i) { const double t = qr[(size_t)k * rows + i]; qr[(size_t)k * rows + i] = qr[(size_t)bc * rows + i]; qr[(size_t)bc * rows + i] = t; }
        beta = make_householder_in_place(qr, rows, k, k, rows - k, &hc[k]);
        qr[(size_t)k * rows + k] = beta;
        if (fabs(beta) > maxpivot) maxpivot = fabs(beta);
        if (cols - k - 1 > 0)
            apply_householder_left(qr, rows, k, k + 1, rows - k, cols - k - 1, qr + (size_t)k * rows + k + 1, hc[k], tmp, sess, 0);
    }
    for (j = 0; j < cols; ++j) perm[j] = j;                       /* m_cols_permutation: transpositions on the right */
    for (k = 0; k < size; ++k) { const int t = perm[k]; perm[k] = perm[ct[k]]; perm[ct[k]] = t; }
    {   /* rank() */
        const double thr = fabs(maxpivot) * (SV_EPS * (double)size);
        for (i = 0; i < nonzero; ++i) rank += fabs(qr[(size_t)i * rows + i]) > thr;
    }
    if (rank == 0) {
        for (j = 0; j < cols; ++j) x[j] = 0.0;
    } else {
        memcpy(c, b, sizeof(double) * (size_t)rows);
        for (k = 0; k < rank; ++k) {
            const int rem = rows - k;
            { const double t = c[k]; c[k] = c[rt[k]]; c[rt[k]] = t; }
            /* c.bottomRightCorner(rem, 1).applyHouseholderOnTheLeft(qr.col(k).tail(rem - 1), h_k, temp): one column, inner product */
            apply_householder_left(c, rows, k, 0, rem, 1, qr + (size_t)k * rows + k + 1, hc[k], tmp, sess, 0);
        }
        {   /* triangular_solve_vector<OnTheLeft, Upper, ColMajor> on c.topRows(rank): panels of 8 from the bottom */
            const int n = rank;
            int pi;
            for (pi = n; pi > 0; pi -= 8) {
                const int pw = pi < 8 ? pi : 8, start = pi - pw;
                int kk;
                for (kk = 0; kk < pw; ++kk) {
                    const int ii = pi - kk - 1;
                    if (c[ii] != 0.0) {
                        const int r = pw - kk - 1, s0 = ii - r;
                        int q;
                        c[ii] /= qr[(size_t)ii * rows + ii];
                        for (q = 0; q < r; ++q) c[s0 + q] -= c[ii] * qr[(size_t)ii * rows + s0 + q];
                    }
                }
                if (start > 0) {                                /* res(0..start) += -1 * L(0..start, start..pi) * c(start..pi) */
                    int row;
                    for (row = 0; row < start; ++row) {
                        double acc = 0.0;
                        int col;
                        for (col = start; col < pi; ++col) acc += qr[(size_t)col * rows + row] * c[col];
                        c[row] = c[row] + acc * -1.0;
                    }
                }
            }
        }
        for (i = 0; i < rank; ++i) x[perm[i]] = c[i];
        for (i = rank; i < cols; ++i) x[perm[i]] = 0.0;
    }
    free(qr); free(hc); free(tmp); free(sess); free(c); free(rt); free(ct); free(perm);
    return rank;
}
