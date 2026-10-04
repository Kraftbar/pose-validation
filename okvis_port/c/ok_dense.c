/* SPDX-License-Identifier: MPL-2.0 */
/* See ok_dense.h (Eigen 3.4.0, MPL-2.0, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors). */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "ok_dense.h"

/* ------------------------------------------------- redux ------------------------------------------------- */
/* redux_impl<LinearVectorizedTraversal, NoUnrolling> with alignedStart = 0 over term(i). */
#define OK_REDUX_BODY(TERM)                                                      \
    double r0a, r0b, r1a, r1b, res;                                              \
    int i;                                                                       \
    const int alignedSize2 = (n / 4) * 4, alignedSize = (n / 2) * 2;             \
    if (alignedSize) {                                                           \
        r0a = TERM(0); r0b = TERM(1);                                            \
        if (alignedSize > 2) {                                                   \
            r1a = TERM(2); r1b = TERM(3);                                        \
            for (i = 4; i < alignedSize2; i += 4) {                              \
                r0a += TERM(i); r0b += TERM(i + 1);                              \
                r1a += TERM(i + 2); r1b += TERM(i + 3);                          \
            }                                                                    \
            r0a += r1a; r0b += r1b;                                              \
            if (alignedSize > alignedSize2) { r0a += TERM(alignedSize2); r0b += TERM(alignedSize2 + 1); } \
        }                                                                        \
        res = r0a + r0b;                                                         \
        for (i = alignedSize; i < n; ++i) res += TERM(i);                        \
    } else {                                                                     \
        res = TERM(0);                                                           \
        for (i = 1; i < n; ++i) res += TERM(i);                                  \
    }                                                                            \
    return res;

double ok_dyn_sqnorm(const double* v, int n) {
#define T(i) (v[i] * v[i])
    if (n <= 0) return 0.0;
    { OK_REDUX_BODY(T) }
#undef T
}
double ok_dyn_norm(const double* v, int n) { return sqrt(ok_dyn_sqnorm(v, n)); }
double ok_dyn_dot(const double* a, const double* b, int n) {
#define T(i) (a[i] * b[i])
    if (n <= 0) return 0.0;
    { OK_REDUX_BODY(T) }
#undef T
}
double ok_dyn_dot_model(const double* m, const double* r, int n) {
#define T(i) (m[i] * (r[i] + m[i] / 2.0))
    if (n <= 0) return 0.0;
    { OK_REDUX_BODY(T) }
#undef T
}
static double ok_dyn_sqnorm_diff(const double* a, const double* b, int n);
double ok_dyn_norm_diff(const double* a, const double* b, int n) { return sqrt(ok_dyn_sqnorm_diff(a, b, n)); }
static double ok_dyn_sqnorm_diff(const double* a, const double* b, int n) {
#define T(i) ((a[i] - b[i]) * (a[i] - b[i]))
    if (n <= 0) return 0.0;
    { OK_REDUX_BODY(T) }
#undef T
}
double ok_dyn_maxabs_diff(const double* a, const double* b, int n) {
    double m = 0.0;
    int i;
    if (n <= 0) return 0.0;
    m = fabs(a[0] - b[0]);
    for (i = 1; i < n; ++i) { const double v = fabs(a[i] - b[i]); if (v > m) m = v; }
    return m;
}

void ok_colwise_sqnorm_add(const double* m, int rows, int cols, int dst_parity, double* dst) {
    const int alignedStart = dst_parity & 1;
    const int alignedEnd = cols > alignedStart ? alignedStart + ((cols - alignedStart) / 2) * 2 : alignedStart;
    int j, r;
#define SQ(rr) (m[(rr) * cols + j] * m[(rr) * cols + j])
    for (j = 0; j < cols; ++j) {
        double s = SQ(0);
        if (j >= alignedStart && j < alignedEnd) {
            const int size4 = (rows - 1) & ~3;
            for (r = 1; r < size4; r += 4) s = s + ((SQ(r) + SQ(r + 1)) + (SQ(r + 2) + SQ(r + 3)));
            for (; r < rows; ++r) s = s + SQ(r);
        } else {
            for (r = 1; r < rows; ++r) s = s + SQ(r);
        }
        dst[j] += s;
    }
#undef SQ
}

/* ------------------------------------------------- GEMV ------------------------------------------------- */
void ok_gemv_col_kernel(int rows, int cols, const double* A, long lda, const double* x, long incx, double* y, double alpha) {
    int i, j;
    for (i = 0; i < rows; ++i) {
        double c = 0.0;
        for (j = 0; j < cols; ++j) c = c + A[i + j * lda] * x[j * incx];
        y[i] = c * alpha + y[i];
    }
}
void ok_gemv_col(int rows, int cols, const double* A, long lda, const double* x, long incx, double* y, double alpha) {
    if (rows == 1) {  /* GemvProduct::scaleAndAddTo: a runtime row vector falls back to dst += alpha * row.dot(x);
                       * the row of a column-major map has a dynamic inner stride -> no packets -> left fold */
        double d = A[0] * x[0];
        int j;
        for (j = 1; j < cols; ++j) d = d + A[j * lda] * x[j * incx];
        y[0] += alpha * d;
        return;
    }
    ok_gemv_col_kernel(rows, cols, A, lda, x, incx, y, alpha);
}
void ok_gemv_row_kernel(int rows, int cols, const double* A, long lda, const double* x, double* y, double alpha) {
    int i, j;
    for (i = 0; i < rows; ++i) {
        double l0 = 0.0, l1 = 0.0, cc;
        for (j = 0; j + 2 <= cols; j += 2) {
            l0 = l0 + A[i * lda + j] * x[j];
            l1 = l1 + A[i * lda + j + 1] * x[j + 1];
        }
        cc = l0 + l1;
        for (; j < cols; ++j) cc += A[i * lda + j] * x[j];
        y[i] += alpha * cc;
    }
}

void ok_gemv_row(int rows, int cols, const double* A, long lda, const double* x, double* y, double alpha) {
    if (rows == 1) {  /* inner-product fallback; the row of a row-major map is contiguous -> vectorised redux */
        y[0] += alpha * ok_dyn_dot(A, x, cols);
        return;
    }
    ok_gemv_row_kernel(rows, cols, A, lda, x, y, alpha);
}

/* ------------------------------------------------- GEBP ------------------------------------------------- */
void ok_gebp(int rows, int cols, int depth, const double* A, long ars, long acs, const double* B, long brs, long bcs,
             double alpha, double* R, long rrs, long rcs) {
    const int packet_cols4 = (cols / 4) * 4;
    const int peeled_mc2 = (rows / 4) * 4;
    const int peeled_mc1 = peeled_mc2 + ((rows - peeled_mc2) / 2) * 2;
    const int peeled_kc = depth & ~7;
    int i, j, k;
    for (i = 0; i < rows; ++i) {
        for (j = 0; j < cols; ++j) {
            double acc;
            if (i >= peeled_mc2 && i < peeled_mc1 && j < packet_cols4) {
                double c = 0.0, d = 0.0;
                for (k = 0; k < peeled_kc; ++k) {
                    const double p = A[i * ars + k * acs] * B[k * brs + j * bcs];
                    if ((k & 1) == 0) c = c + p; else d = d + p;
                }
                c = c + d;
                for (k = peeled_kc; k < depth; ++k) c = c + A[i * ars + k * acs] * B[k * brs + j * bcs];
                acc = c;
            } else {
                acc = 0.0;
                for (k = 0; k < depth; ++k) acc = acc + A[i * ars + k * acs] * B[k * brs + j * bcs];
            }
            R[i * rrs + j * rcs] = acc * alpha + R[i * rrs + j * rcs];
        }
    }
}

/* --------------------------------------------- blocking ------------------------------------------------ */
#define OK_L1 32768L
#define OK_L2 524288L
#define OK_L3 16777216L
void ok_blocking_sizes(long* k, long* m, long* n, int kc_factor) {
    const long mr = 4, nr = 4, k_peeling = 8;
    const long k_div = kc_factor * (mr * 8 + nr * 8), k_sub = mr * nr * 8;
    long max_kc, old_k, actual_l2 = 1572864, max_nc, lhs_bytes, remaining_l1, nc;
    long K = *k, M = *m, N = *n;
    long mx = K > M ? K : M;
    if (N > mx) mx = N;
    if (mx < 48) return;
    max_kc = ((OK_L1 - k_sub) / k_div) & ~(k_peeling - 1);
    if (max_kc < 1) max_kc = 1;
    old_k = K;
    if (K > max_kc) K = (K % max_kc) == 0 ? max_kc : max_kc - k_peeling * ((max_kc - 1 - (K % max_kc)) / (k_peeling * (K / max_kc + 1)));
    lhs_bytes = M * K * 8;
    remaining_l1 = OK_L1 - k_sub - lhs_bytes;
    if (remaining_l1 >= nr * 8 * K) max_nc = remaining_l1 / (K * 8);
    else max_nc = (3 * actual_l2) / (2 * 2 * max_kc * 8);
    nc = actual_l2 / (2 * K * 8);
    if (max_nc < nc) nc = max_nc;
    nc &= ~(nr - 1);
    if (N > nc) {
        N = (N % nc) == 0 ? nc : (nc - nr * ((nc - (N % nc)) / (nr * (N / nc + 1))));
    } else if (old_k == K) {
        const long problem_size = K * N * 8;
        long actual_lm = actual_l2, max_mc = M, mc;
        if (problem_size <= 1024) actual_lm = OK_L1;
        else if (OK_L3 != 0 && problem_size <= 32768) { actual_lm = OK_L2; if (max_mc > 576) max_mc = 576; }
        mc = actual_lm / (3 * K * 8);
        if (mc > max_mc) mc = max_mc;
        if (mc > mr) mc -= mc % mr;
        else if (mc == 0) return;
        M = (M % mc) == 0 ? mc : (mc - mr * ((mc - (M % mc)) / (mr * (M / mc + 1))));
    }
    *k = K; *m = M; *n = N;
}

/* ------------------------------- triangular_solve_matrix<OnTheRight> ----------------------------------- */
/* T(i,j) = T[i*trs + j*tcs] (trs/tcs encode TriStorageOrder), X(i,j) = X[i + j*ldx] (ColMajor, rows x size).
 * SmallPanelWidth = 4. */
void ok_trsm_right(int size, int rows, const double* T, long trs, long tcs, int mode_lower, double* X, long ldx,
                   int transpose_blocking) {
    long kc = size, mc_b, nc_b, mcb;
    int k2;
    if (transpose_blocking) { mc_b = size; nc_b = rows; } else { mc_b = rows; nc_b = size; }
    ok_blocking_sizes(&kc, &mc_b, &nc_b, 4);
    mcb = mc_b < rows ? mc_b : rows;  /* mc = min(rows, blocking.mc()) */
    if (mcb < 1) mcb = 1;
    for (k2 = mode_lower ? size : 0; mode_lower ? k2 > 0 : k2 < size; k2 = mode_lower ? k2 - (int)kc : k2 + (int)kc) {
        const int actual_kc = (int)((mode_lower ? k2 : size - k2) < kc ? (mode_lower ? k2 : size - k2) : kc);
        const int actual_k2 = mode_lower ? k2 - actual_kc : k2;
        const int startPanel = mode_lower ? 0 : k2 + actual_kc;
        const int rs = mode_lower ? actual_k2 : size - actual_k2 - actual_kc;
        int i2;
        for (i2 = 0; i2 < rows; i2 += (int)mcb) {
            const int actual_mc = (rows - i2) < mcb ? rows - i2 : (int)mcb;
            int j2;
            for (j2 = mode_lower ? (actual_kc - ((actual_kc % 4) ? (actual_kc % 4) : 4)) : 0;
                 mode_lower ? j2 >= 0 : j2 < actual_kc; j2 = mode_lower ? j2 - 4 : j2 + 4) {
                const int w = (actual_kc - j2) < 4 ? actual_kc - j2 : 4;
                const int absolute_j2 = actual_k2 + j2;
                const int panelOffset = mode_lower ? j2 + w : 0;
                const int panelLength = mode_lower ? actual_kc - j2 - w : j2;
                int k, k3, i;
                if (panelLength > 0) {
                    /* X(i2.., absolute_j2 + j) -= sum_k X(i2.., actual_k2 + panelOffset + k) T(actual_k2 + panelOffset + k, absolute_j2 + j) */
                    ok_gebp(actual_mc, w, panelLength, X + i2 + (long)(actual_k2 + panelOffset) * ldx, 1, ldx,
                            T + (long)(actual_k2 + panelOffset) * trs + (long)absolute_j2 * tcs, trs, tcs, -1.0,
                            X + i2 + (long)absolute_j2 * ldx, 1, ldx);
                }
                for (k = 0; k < w; ++k) {
                    const int j = mode_lower ? absolute_j2 + w - k - 1 : absolute_j2 + k;
                    double* r = X + i2 + (long)j * ldx;
                    double inv_rjj;
                    for (k3 = 0; k3 < k; ++k3) {
                        const int jj = mode_lower ? j + 1 + k3 : absolute_j2 + k3;
                        const double b = T[(long)jj * trs + (long)j * tcs];
                        const double* a = X + i2 + (long)jj * ldx;
                        for (i = 0; i < actual_mc; ++i) r[i] -= a[i] * b;
                    }
                    inv_rjj = 1.0 / T[(long)j * trs + (long)j * tcs];
                    for (i = 0; i < actual_mc; ++i) r[i] *= inv_rjj;
                }
            }
            if (rs > 0) {
                /* X(i2.., startPanel + j) -= sum_k X(i2.., actual_k2 + k) T(actual_k2 + k, startPanel + j) */
                ok_gebp(actual_mc, rs, actual_kc, X + i2 + (long)actual_k2 * ldx, 1, ldx,
                        T + (long)actual_k2 * trs + (long)startPanel * tcs, trs, tcs, -1.0,
                        X + i2 + (long)startPanel * ldx, 1, ldx);
            }
        }
    }
}

/* --------------------------------------------- LLT (Lower) --------------------------------------------- */
#define AA(i, j) a[(i) + (long)(j) * lda]
static int llt_unblocked(int n, double* a, long lda) {
    int k, i, j;
    for (k = 0; k < n; ++k) {
        const int rs = n - k - 1;
        double x = AA(k, k);
        if (k > 0) {  /* A10.squaredNorm(): strided row -> DefaultTraversal left fold */
            double s = AA(k, 0) * AA(k, 0);
            for (j = 1; j < k; ++j) s = s + AA(k, j) * AA(k, j);
            x -= s;
        }
        if (x <= 0.0) return k;
        x = sqrt(x);
        AA(k, k) = x;
        if (k > 0 && rs > 0)  /* A21.noalias() -= A20 * A10.adjoint(): ColMajor GEMV (rhs strided by lda), alpha = -1 */
            ok_gemv_col(rs, k, a + (k + 1), lda, a + k, lda, a + (k + 1) + (long)k * lda, -1.0);
        if (rs > 0) for (i = 0; i < rs; ++i) AA(k + 1 + i, k) /= x;
    }
    return -1;
}

/* A22.selfadjointView<Lower>().rankUpdate(A21, -1): general_matrix_matrix_triangular_product<ColMajor res, Lower>
 * with lhs = A21 (rs x bs, ColMajor), rhs = A21^T; one kc block (depth = bs). */
static void rank_update_lower(int size, int depth, const double* A21, long lda21, double* A22, long lda22) {
    long kc = depth, m = size, nn = size, mc;
    int i2, j;
    ok_blocking_sizes(&kc, &m, &nn, 1);
    mc = m < size ? m : size;
    if (mc > 4) mc = (mc / 4) * 4;
    if (mc < 1) mc = 1;
    for (i2 = 0; i2 < size; i2 += (int)mc) {
        const int actual_mc = (size - i2) < mc ? size - i2 : (int)mc;
        /* gebp(res.getSubMapper(i2, 0), ..., actual_mc, kc, min(size, i2), alpha) */
        if (i2 > 0)
            ok_gebp(actual_mc, i2 < size ? i2 : size, depth, A21 + i2, 1, lda21, A21, lda21, 1, -1.0, A22 + i2, 1, lda22);
        /* tribb_kernel (BlockSize 4, Lower) on the diagonal part rows/cols [i2, i2 + actual_mc) */
        for (j = 0; j < actual_mc; j += 4) {
            const int bsz = (actual_mc - j) < 4 ? actual_mc - j : 4;
            double buffer[16];
            int i1, j1;
            memset(buffer, 0, sizeof buffer);
            ok_gebp(bsz, bsz, depth, A21 + i2 + j, 1, lda21, A21 + i2 + j, lda21, 1, -1.0, buffer, 1, 4);
            for (j1 = 0; j1 < bsz; ++j1)
                for (i1 = j1; i1 < bsz; ++i1) A22[(i2 + j + i1) + (long)(i2 + j + j1) * lda22] += buffer[i1 + 4 * j1];
            if (j + bsz < actual_mc)  /* rows below the diagonal block within this chunk */
                ok_gebp(actual_mc - j - bsz, bsz, depth, A21 + i2 + j + bsz, 1, lda21, A21 + i2 + j, lda21, 1, -1.0,
                        A22 + (i2 + j + bsz) + (long)(i2 + j) * lda22, 1, lda22);
        }
    }
}

int ok_llt_lower(int n, double* a, long lda) {
    int k;
    long blockSize;
    if (n < 32) return llt_unblocked(n, a, lda);
    blockSize = n / 8;
    blockSize = (blockSize / 16) * 16;
    if (blockSize < 8) blockSize = 8;
    if (blockSize > 128) blockSize = 128;
    for (k = 0; k < n; k += (int)blockSize) {
        const int bs = (n - k) < blockSize ? n - k : (int)blockSize;
        const int rs = n - k - bs;
        int ret = llt_unblocked(bs, a + k + (long)k * lda, lda);
        if (ret >= 0) return k + ret;
        if (rs > 0) {
            /* A11.adjoint().triangularView<Upper>().solveInPlace<OnTheRight>(A21): T = L11^T (Upper), T(i,j) =
             * A11(j,i) -> trs = lda (row index i of T walks columns of A11... T[i*trs + j*tcs] = A11[j + i*lda]) */
            ok_trsm_right(bs, rs, a + k + (long)k * lda, lda, 1, 0, a + k + bs + (long)k * lda, lda, 0);
            rank_update_lower(rs, bs, a + k + bs + (long)k * lda, lda, a + k + bs + (long)(k + bs) * lda, lda);
        }
    }
    return -1;
}

void ok_llt_lower_solve(int n, const double* a, long lda, const double* b, double* x) {
    int pi, k, i;
    memcpy(x, b, sizeof(double) * (size_t)n);
    /* matrixL().solveInPlace(x): triangular_solve_vector<OnTheLeft, Lower, ColMajor>, PanelWidth 8 */
    for (pi = 0; pi < n; pi += 8) {
        const int w = (n - pi) < 8 ? n - pi : 8;
        const int endBlock = pi + w;
        int r;
        for (k = 0; k < w; ++k) {
            const int ii = pi + k;
            if (x[ii] != 0.0) {
                const int rr = w - k - 1;
                const int s = ii + 1;
                x[ii] /= AA(ii, ii);
                for (i = 0; i < rr; ++i) x[s + i] = x[s + i] - x[ii] * AA(s + i, ii);
            }
        }
        r = n - endBlock;
        if (r > 0) ok_gemv_col_kernel(r, w, a + endBlock + (long)pi * lda, lda, x + pi, 1, x + endBlock, -1.0);
    }
    /* matrixU().solveInPlace(x): triangular_solve_vector<OnTheLeft, Upper, RowMajor> on the adjoint:
     * lhs(i,j) = L(j,i) */
    for (pi = n; pi > 0; pi -= 8) {
        const int w = pi < 8 ? pi : 8;
        const int r = n - pi;
        if (r > 0) {
            const int startRow = pi - w;
            /* general_matrix_vector_product RowMajor: rows w, cols r, lhs(i,j) = L(pi + j, startRow + i) =
             * a[(pi + j) + (startRow + i)*lda]: a row-major view with row stride lda */
            ok_gemv_row_kernel(w, r, a + pi + (long)startRow * lda, lda, x + pi, x + startRow, -1.0);
        }
        for (k = 0; k < w; ++k) {
            const int ii = pi - k - 1;
            const int s = ii + 1;
            if (k > 0) {
                /* (row(i).segment(s,k).cwiseProduct(x.segment(s,k))).sum(): vectorised redux from 0 */
                double res;
                const int nn = k;
#define TT(q) (AA(s + (q), ii) * x[s + (q)])
                {
                    double r0a, r0b, r1a, r1b;
                    int q;
                    const int alignedSize2 = (nn / 4) * 4, alignedSize = (nn / 2) * 2;
                    if (alignedSize) {
                        r0a = TT(0); r0b = TT(1);
                        if (alignedSize > 2) {
                            r1a = TT(2); r1b = TT(3);
                            for (q = 4; q < alignedSize2; q += 4) { r0a += TT(q); r0b += TT(q + 1); r1a += TT(q + 2); r1b += TT(q + 3); }
                            r0a += r1a; r0b += r1b;
                            if (alignedSize > alignedSize2) { r0a += TT(alignedSize2); r0b += TT(alignedSize2 + 1); }
                        }
                        res = r0a + r0b;
                        for (q = alignedSize; q < nn; ++q) res += TT(q);
                    } else {
                        res = TT(0);
                        for (q = 1; q < nn; ++q) res += TT(q);
                    }
                }
#undef TT
                x[ii] -= res;
            }
            if (x[ii] != 0.0) x[ii] /= AA(ii, ii);
        }
    }
}
#undef AA

/* ------------------------------------- Ceres InvertPSDMatrix<Dynamic> ----------------------------------- */
int ok_invert_psd_dyn(int n, const double* m, double* out) {
    /* m_matrix = m.selfadjointView<Upper>() (row-major n x n, both triangles from the upper) */
    double* w = (double*)malloc(sizeof(double) * (size_t)n * (size_t)n);
    int i, j, ok;
    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j) w[i * n + j] = i <= j ? m[i * n + j] : m[j * n + i];
    /* llt_inplace<Upper>::blocked(m_matrix) == Lower::blocked(m_matrix.transpose()): the transpose of the row-major
     * matrix is column-major data with lda = n */
    ok = ok_llt_lower(n, w, n) < 0;
    /* dst = Identity (row-major); matrixL().solveInPlace(dst): L = U^T with U in the upper triangle of w (row-major
     * view) -> swapped to OnTheRight / Upper with the triangular matrix in RowMajor storage, the rhs as ColMajor
     * (dst row-major viewed column-major), blocking built for a RowMajor rhs */
    for (i = 0; i < n; ++i) for (j = 0; j < n; ++j) out[i * n + j] = i == j ? 1.0 : 0.0;
    ok_trsm_right(n, n, w, n, 1, 0, out, n, 1);
    /* matrixU().solveInPlace(dst): U row-major -> swapped to OnTheRight / Lower with the triangular matrix in
     * ColMajor storage: T(i,j) = w[j*n + i] */
    ok_trsm_right(n, n, w, 1, n, 1, out, n, 1);
    free(w);
    return ok;
}
