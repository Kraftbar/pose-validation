/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * OKVIS2 pure-C port, module 4a: Ceres Solver 2.2.0 "small BLAS" kernels (internal/ceres/small_blas.h,
 * small_blas_generic.h): the hand-written loops Ceres uses whenever at least one matrix dimension is dynamic
 * (CUSTOM_BLAS=ON in the reference build). They define the floating-point summation order of the Jacobian
 * manifold products, the gradient, the Jacobian-vector products of the trust-region loop, the Schur
 * elimination with dynamic block sizes and the J^T J inner products.
 *
 * Derived from Ceres Solver (http://ceres-solver.org), Copyright 2023 Google Inc. All rights reserved.
 * BSD-3-Clause (see okvis_port/LICENSES/ceres-BSD-3-Clause.txt): redistribution requires retaining this notice;
 * the name of Google Inc. may not be used to endorse derived products.
 *
 * C99, no library headers. All matrices are ROW-major (Ceres' convention). `op` is kOperation: +1 adds into
 * the destination, -1 subtracts, 0 assigns. Every scalar accumulation is a left fold that starts at 0.0
 * (tmp = 0.0; tmp += a*b; ...), columns are processed in groups: an odd last column first, then a pair, then
 * groups of four (MMM_mat1x4 / MTM_mat1x4 / MVM_mat4x1 / MTV_mat4x1 with four independent accumulators).
 */
#ifndef OK_BLAS_H
#define OK_BLAS_H

/* MatrixMatrixMultiplyNaive<Dynamic,...,op>: C(start_row.., start_col..) op= A (ra x ca) * B (rb x cb);
 * C has col_stride_c columns per row (row_stride_c is only asserted upstream). */
void ok_mmm(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int start_row_c,
            int start_col_c, int col_stride_c, int op);
/* MatrixTransposeMatrixMultiplyNaive: C op= A^T (A is ra x ca) * B (rb x cb), rb == ra. */
void ok_mtm(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int start_row_c,
            int start_col_c, int col_stride_c, int op);
/* MatrixVectorMultiply<Dynamic,Dynamic,op>: c op= A (ra x ca) * b */
void ok_mv(const double* A, int ra, int ca, const double* b, double* c, int op);
/* MatrixTransposeVectorMultiply<Dynamic,Dynamic,op>: c op= A^T * b (A is ra x ca, b has ra entries) */
void ok_mtv(const double* A, int ra, int ca, const double* b, double* c, int op);

#endif
