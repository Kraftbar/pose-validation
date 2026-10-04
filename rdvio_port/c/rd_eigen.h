/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 *
 * Eigen 3.4.0 (MPL-2.0) bit-exact evaluation-order models used by the RD-VIO port, on top of okvis_port/c/ok_dense + ok_eigen:
 * fixed-size Matrix<double,N,N>::inverse() for N > 4 (PartialPivLU::inverse(): unblocked_lu + two triangular_solve_matrix<OnTheLeft>),
 * and LLT(M).matrixL().transpose().
 * Derived from Eigen LU/PartialPivLU.h, Core/products/TriangularSolverMatrix.h, Cholesky/LLT.h.
 * Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors. Column-major data. */
#ifndef RD_EIGEN_H
#define RD_EIGEN_H
/* Matrix<double,n,n>::inverse() with 5 <= n <= 16 (fixed size, unblocked LU). Returns the first zero pivot index or -1. */
int rd_inverse_ppl(int n, const double* a, double* inv);
/* Eigen::LLT<Matrix<double,n,n>>(info).matrixL().transpose() assigned to a dense matrix (out = L^T, zeros below the diagonal).
 * `info` is read in full (copied as Eigen does). Returns -1 on success else the failing pivot (out holds the partial factor). */
int rd_llt_sqrt_info(int n, const double* info, double* out);
#endif
