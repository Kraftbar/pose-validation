/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 *
 * Eigen 3.4.0 (MPL-2.0) bit-exact evaluation-order models of the DYNAMIC-size dense kernels Ceres 2.2.0 drives in
 * the OKVIS2 solver (module M4): redux (squaredNorm / norm / dot / lpNorm<Infinity> of dynamic vectors),
 * GEMV (GeneralMatrixVector.h, column- and row-major kernels), GEBP (GeneralBlockPanelKernel.h, SSE2 double:
 * mr = 4, nr = 4, pk = 8, no FMA), the triangular matrix solver with the triangular matrix on the right
 * (TriangularSolverMatrix.h), the triangular vector solvers (TriangularSolverVector.h), the symmetric rank update
 * (GeneralMatrixMatrixTriangular.h / SelfadjointProduct.h), LLT (Cholesky/LLT.h: unblocked for n < 32, blocked
 * above) and the blocking heuristic (GeneralBlockPanelKernel.h evaluateProductBlockingSizesHeuristic, single
 * thread, L1 32 KB / L2 512 KB / L3 16 MB as queried on the reference machine: for the sizes of this module the
 * blocking only chunks rows at multiples of 4, which never changes a summation).
 * Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors.
 *
 * Reference flags: g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, SSE2 baseline (Packet2d), no FMA.
 * Strides are in doubles; "lda" is the leading dimension of column-major data.
 */
#ifndef OK_DENSE_H
#define OK_DENSE_H

/* ---- redux (Redux.h, LinearVectorizedTraversal, NoUnrolling, alignedStart = 0: the expressions are
 * cwiseAbs2 / cwiseProduct / difference expressions without DirectAccess) ----
 * two 2-lane accumulators: lanes (0,1) and (2,3) of every group of four, then res0 += res1, a trailing pair,
 * predux = lane0 + lane1, then the odd tail scalar. */
double ok_dyn_sqnorm(const double* v, int n);                       /* v.squaredNorm() */
double ok_dyn_norm(const double* v, int n);                         /* v.norm() = sqrt(squaredNorm) */
double ok_dyn_dot(const double* a, const double* b, int n);         /* a.dot(b) */
double ok_dyn_dot_model(const double* m, const double* r, int n);   /* m.dot(r + m/2.0) (Ceres model cost change) */
double ok_dyn_norm_diff(const double* a, const double* b, int n);   /* (a - b).norm() */
double ok_dyn_maxabs_diff(const double* a, const double* b, int n); /* (a - b).lpNorm<Infinity>() */

/* `Map<VectorXd>(dst, cols) += Map<const RowMajorMatrix>(m, rows, cols).colwise().squaredNorm()` (Ceres
 * BlockSparseMatrix::SquaredColumnNorm): a LinearVectorized assignment peeled by the destination address
 * (dst_parity = ((dst - base) & 1) for a 16-byte aligned base): packet columns reduce the rows with
 * packetwise_redux_impl's tree p0 + ((p1+p2)+(p3+p4)) + ..., peeled columns with the scalar left fold. */
void ok_colwise_sqnorm_add(const double* m, int rows, int cols, int dst_parity, double* dst);

/* ---- GEMV: y += alpha * A x ----
 * column-major kernel: per row one accumulator over the columns in order starting from 0, then y + alpha*acc.
 * A(i,j) = A[i + j*lda]. (block_cols = cols for cols < 128.) */
void ok_gemv_col(int rows, int cols, const double* A, long lda, const double* x, long incx, double* y, double alpha);
/* row-major kernel: per row packets of two columns (even/odd lanes), predux = lane0 + lane1, scalar tail, then
 * y + alpha*acc. A(i,j) = A[i*lda + j]. */
void ok_gemv_row(int rows, int cols, const double* A, long lda, const double* x, double* y, double alpha);
/* The *_kernel variants are general_matrix_vector_product::run itself (as the triangular vector solvers call it);
 * ok_gemv_col / ok_gemv_row are the product-expression entry (GemvProduct::scaleAndAddTo), which for a runtime
 * row vector (rows == 1) falls back to dst += alpha * row.dot(x): a left fold for a column-major map's row
 * (dynamic inner stride, no packets), the vectorised redux for a row-major map's row. */
void ok_gemv_col_kernel(int rows, int cols, const double* A, long lda, const double* x, long incx, double* y, double alpha);
void ok_gemv_row_kernel(int rows, int cols, const double* A, long lda, const double* x, double* y, double alpha);

/* ---- GEBP: R(i,j) += alpha * sum_k A(i,k) B(k,j) with the gebp_kernel accumulation structure ----
 * rows [0, 4*(rows/4)): one chain per entry; the next 2*((rows%4)/2) rows: for column groups of four, even-k chain
 * C and odd-k chain D over the first 8*(depth/8) k's, C + D, then the remaining k's into C (single chain for the
 * leftover columns); the last odd row: one chain. Store: R + alpha*acc (acc*alpha + R).
 * A(i,k) = A[i*ars + k*acs], B(k,j) = B[k*brs + j*bcs], R(i,j) = R[i*rrs + j*rcs]. */
void ok_gebp(int rows, int cols, int depth, const double* A, long ars, long acs, const double* B, long brs, long bcs,
             double alpha, double* R, long rrs, long rcs);

/* evaluateProductBlockingSizesHeuristic (single thread) for double: in/out k, m, n */
void ok_blocking_sizes(long* k, long* m, long* n, int kc_factor);

/* ---- triangular_solve_matrix<OnTheRight>: X T = B solved in place for X (B overwritten) ----
 * mode_lower: T is lower (else upper); T(i,j) = T[i*trs + j*tcs]; X is rows x size with X(i,j) = X[i + j*ldx]
 * (column-major, unit inner stride); blocking as the triangular_solver_selector builds it (KcFactor 4) for a
 * column-major (transpose = 0) or row-major-flagged (transpose = 1) right-hand side. */
void ok_trsm_right(int size, int rows, const double* T, long trs, long tcs, int mode_lower, double* X, long ldx,
                   int transpose_blocking);

/* ---- LLT<MatrixType, Lower> in place on column-major data a(i,j) = a[i + j*lda] (lower triangle read and
 * written; the strict upper triangle is left untouched). Returns -1 on success, else the failing pivot index
 * (Eigen's NumericalIssue). Unblocked for n < 32, blocked (blockSize = clamp((n/8/16)*16, 8, 128)) above. ---- */
int ok_llt_lower(int n, double* a, long lda);
/* LLT<..., Lower>::solve for one right-hand side: x = b; L x = b (TriangularSolverVector ColMajor Lower);
 * L^T x = x (RowMajor Upper) */
void ok_llt_lower_solve(int n, const double* a, long lda, const double* b, double* x);

/* ---- Ceres InvertPSDMatrix<Eigen::Dynamic>(assume_full_rank = true, m): row-major n x n in and out:
 * m.selfadjointView<Upper>().llt().solve(Identity). Returns 0 if the LLT failed (out then holds Eigen's result
 * of solving with the partial factor, as Eigen does not check). ---- */
int ok_invert_psd_dyn(int n, const double* m, double* out);

#endif
