/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M4: the statically-sized kernels of Ceres' SchurEliminator<2, 3, 3> (the specialisation Ceres picks for
 * the pose-only problems of the Initializer: rows of 2, one 3-dimensional e block (the rotation) and 3-dimensional f blocks).
 * With all four template dimensions static, small_blas.h takes the EIGEN path (Eigen 3.4.0 lazy coefficient-based products on
 * row-major Maps) instead of the naive loops used by the fully dynamic eliminator (ok_blas.c), and kEBlockSize = 3 selects
 * InvertPSDMatrix<3> and a fixed-size `inverse * y_block`.
 *
 * Derived from Ceres Solver 2.2.0 (BSD-3-Clause, Copyright 2023 Google Inc.: small_blas.h, invert_psd_matrix.h,
 * schur_eliminator_impl.h) and Eigen 3.4.0 (MPL-2.0, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors).
 * Row-major matrices throughout (Ceres' EigenTypes<r, c>::Matrix). op: +1 adds into the destination, -1 subtracts, 0 assigns.
 * The matrix-vector kernels (MatrixVectorMultiply / MatrixTransposeVectorMultiply) always use Ceres' naive loops, whatever the template
 * dimensions: ok_blas.c (ok_mv / ok_mtv) serves the static eliminator as well.
 */
#ifndef RD_STATIC_H
#define RD_STATIC_H
/* MatrixTransposeMatrixMultiply<2,3,2,3,op>: C(3x3 at col stride cs) op= A^T B, A, B 2x3 */
void rd_st_mtm_2_3(const double* A, const double* B, double* C, int cs, int op);
/* MatrixTransposeMatrixMultiply<3,3,3,3,op>: C(3x3 at col stride cs) op= A^T B */
void rd_st_mtm_3_3(const double* A, const double* B, double* C, int cs, int op);
/* MatrixMatrixMultiply<3,3,3,3,op>: C op= A B */
void rd_st_mmm_3_3(const double* A, const double* B, double* C, int cs, int op);
/* InvertPSDMatrix<3>(assume_full_rank = true, m) = m.inverse() (kSize < 5): closed-form 3x3 inverse of the full row-major matrix */
void rd_st_invert_psd3(const double* m, double* out);
/* out = InvertPSDMatrix<3>(m) * y (SchurEliminator::BackSubstitute, `y_block = inverse * y_block`) */
void rd_st_inv_times_y(const double* inv, const double* y, double* out);
#endif
