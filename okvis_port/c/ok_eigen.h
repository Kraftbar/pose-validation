/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 *
 * Eigen 3.4.0 (MPL-2.0) bit-exact evaluation-order models used by the OKVIS2 port.
 * Derived from Eigen's GeneralBlockPanelKernel.h, SelfadjointMatrixVector.h, Tridiagonalization.h,
 * SelfAdjointEigenSolver.h, Householder.h, Jacobi.h and the 3x3 product rules measured in
 * stella_port/HANDOVER.md ("Eigen 3x3 double evaluation-order rules").
 * Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors.
 *
 * Reference flags: g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, SSE2 baseline, no FMA.
 * All matrices are column-major (m[row + nrows*col]) like Eigen's default.
 */
#ifndef OK_EIGEN_H
#define OK_EIGEN_H

/* ---- quaternion (Eigen::Quaterniond coeffs order x,y,z,w) ---- */
typedef struct ok_quat { double x, y, z, w; } ok_quat;
void ok_quat_mul(const ok_quat* a, const ok_quat* b, ok_quat* out);   /* a*b, SSE2 quat_product<double> */
void ok_quat_to_mat3(const ok_quat* q, double out[9]);                /* toRotationMatrix */
void ok_quat_normalize(ok_quat* q);                                   /* coeffs().normalize() */
ok_quat ok_quat_normalized(ok_quat q);                                /* normalized() */
ok_quat ok_quat_inverse(ok_quat q);                                   /* inverse(): conjugate().coeffs()/squaredNorm() */
double ok_quat_squared_norm(const ok_quat* q);                        /* (x^2+z^2)+(y^2+w^2) */

/* ---- 3x3 / 3-vector (stella rules: rows 0-1 left-assoc, row 2 right-assoc) ---- */
void ok_m3_mul(const double a[9], const double b[9], double out[9]);        /* A*B and A*B^T-with-materialised-transpose */
void ok_m3_mulv(const double a[9], const double v[3], double out[3]);       /* A*v */
void ok_m3_mulv_lhsT(const double a[9], const double v[3], double out[3]);  /* A^T*v, all rows left-assoc */
double ok_v3_norm(const double v[3]);                                       /* sqrt(L(x.x)) */
void ok_v3_normalized(const double v[3], double out[3]);                    /* x / sqrt(L) */

/* ---- general matrix-matrix product, Eigen's gebp kernel for double/SSE2 (mr=4, nr=4, pk=8),
 * small-problem path (single kc/mc/nc block). out(rows x cols) = L(rows x depth) * R(depth x cols),
 * all column-major, dst assumed zero-initialised by Eigen (alpha = 1). ---- */
void ok_gemm(int rows, int cols, int depth, const double* L, const double* R, double* out);

/* ---- SelfAdjointEigenSolver<Matrix<double,n,n>> (lower triangle referenced), n <= OK_EIG_MAX: eigenvalues
 * ascending, eigenvectors in columns (column-major n x n). n == 3 takes Eigen's closed-form 3x3 tridiagonalisation
 * (tridiagonalization_inplace_selector<_,3,false>), the other sizes the Householder path; the QL iteration and the
 * first-minimum selection sort are shared. hc_parity: alignment parity in doubles (0 = 16-byte aligned) of the
 * solver's m_hcoeffs[0]: it decides the symv peeling of the tridiagonalisation and only matters for n > 8 (fixed
 * 15x15 member and heap-allocated dynamic matrices: 0). Returns 0 on success (1: NoConvergence, 2: bad n). ---- */
#define OK_EIG_MAX 32
int ok_selfadjoint_eig(int n, const double* a, double* evals, double* evecs, int hc_parity);
int ok_selfadjoint_eig15(const double a[225], double evals[15], double evecs[225]); /* = ok_selfadjoint_eig(15, ..., 0) */

#endif
