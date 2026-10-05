/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 * Eigen 3.4 EigenSolver<Matrix<double,N,N>> (non-symmetric, real Schur) for N = 8 and N = 10, column-major, SSE2 evaluation
 * order of the reference build; plus the std::complex<double> operations the OpenGV solvers use. */
#ifndef OK_EIGEN_EIGSOLVER_H
#define OK_EIGEN_EIGSOLVER_H

/* EigenSolver<Matrix<double,N,N>>::compute(a, true) followed by eigenvectors(): eigenvalues (real, imag) and the unit-norm
 * complex eigenvectors (columns of vr + i vi). Returns 0 on success (-1: NoConvergence / NumericalIssue). */
int ok_eigensolver8(const double a[64], double er[8], double ei[8], double vr[64], double vi[64]);
int ok_eigensolver10(const double a[100], double er[10], double ei[10], double vr[100], double vi[100]);

/* std::complex<double> (a + ib) / (c + id) (libgcc __divdc3, finite inputs) */
void ok_cdiv(double a, double b, double c, double d, double* re, double* im);
/* std::complex<double> * (the plain (ac - bd, ad + bc) of -O2 without NaN recovery, inputs finite) */
void ok_cmul(double a, double b, double c, double d, double* re, double* im);
/* std::sqrt(std::complex<double>) (glibc csqrt) */
void ok_csqrt(double a, double b, double* re, double* im);

#endif
