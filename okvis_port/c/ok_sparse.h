/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 *
 * Eigen 3.4.0 (MPL-2.0) sparse kernels used by Ceres' SPARSE_NORMAL_CHOLESKY with EIGEN_SPARSE (module M4):
 *   - AMDOrdering<int> (OrderingMethods/Ordering.h + Amd.h): ok_amd.c, the stella_port transliteration
 *     (stella_port/c/sv_eigen_amd.c) copied unchanged apart from the entry-point name. Ceres applies it to the
 *     BLOCK Hessian pattern (reorder_program.cc, OrderingForSparseNormalCholeskyUsingEigenSparse), and the
 *     scalar factorisation then runs with NaturalOrdering (AreJacobianColumnsOrdered -> OrderingType::NATURAL).
 *   - SimplicialLDLT<SparseMatrix<double>, Upper, NaturalOrdering<int>> (SparseCholesky/SimplicialCholesky.h,
 *     SimplicialCholesky_impl.h factorize_preordered<DoLDLT = true>, SparseCore/TriangularSolver.h): ok_sparse.c.
 *     Eigen's LDL code is adapted upstream from Timothy Davis's LDL under a licence grant for MPL-2.0 distribution
 *     inside Eigen (notice at the top of SimplicialCholesky_impl.h); this is a transliteration of the Eigen file.
 * Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors.
 *
 * Input convention (Ceres eigensparse.cc): the J^T J of Ceres is a CompressedRowSparseMatrix in LOWER_TRIANGULAR
 * block storage; mapped as a column-major Eigen::SparseMatrix it is the UPPER triangle (plus the strictly-lower
 * parts of the diagonal blocks, which factorize_preordered skips: only entries with row <= column are read).
 * Ap[n+1], Ai[Ap[n]], Ax: that CSC, inner indices in storage order (Ceres emits them sorted).
 */
#ifndef OK_SPARSE_H
#define OK_SPARSE_H

/* Eigen::AMDOrdering<int>: perm[k] = index of the node eliminated k-th (used by Ceres as
 * parameter_blocks[k] = old[perm[k]]). Returns 0. */
int ok_amd_order(int n, const int* Ap, const int* Ai, int* perm);

typedef struct ok_ldlt {
    int n, nnz_l, ok;
    int* parent;
    int* nz_per_col;
    int* Lp;
    int* Li;
    double* Lx;
    double* D;      /* m_diag */
    double* y;
    int* pattern;
    int* tags;
} ok_ldlt;

/* analyzePattern (natural ordering): elimination tree + column counts from the pattern (entries with row < col) */
void ok_ldlt_analyze(ok_ldlt* f, int n, const int* Ap, const int* Ai);
/* factorize: returns 1 on success (Eigen: info == Success), 0 when a pivot d == 0 (NumericalIssue) */
int ok_ldlt_factorize(ok_ldlt* f, const int* Ap, const int* Ai, const double* Ax);
/* solve: x = L^-T D^-1 L^-1 b (unit lower L, diagonal multiply by (1/d)) */
void ok_ldlt_solve(const ok_ldlt* f, const double* b, double* x);
void ok_ldlt_free(ok_ldlt* f);

#endif
