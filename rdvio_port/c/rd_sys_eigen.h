/* SPDX-License-Identifier: MPL-2.0 */
/* RD-VIO pure-C port: Eigen 3.4.0 evaluation-order models used by the initializer (module M9). This Source Code Form is subject
 * to the terms of the Mozilla Public License, v. 2.0 (Eigen); http://mozilla.org/MPL/2.0/. C99. */
#ifndef RD_SYS_EIGEN_H
#define RD_SYS_EIGEN_H
#include "../../okvis_port/c/ok_eigen.h"
/* JacobiSVD<Matrix3d>(A, ComputeFullU | ComputeFullV).solve(b): x = V_r * (diag(1 / s_r) * (U_r^T b)), r = rank()
 * (singular values >= max(s_0 * 3 eps, DBL_MIN)); every product a left fold. A column-major. Returns the rank. */
int rd_svd3_solve(const double A[9], const double b[3], double x[3]);
/* Quaternion::FromTwoVectors(a, b); returns 0 in the antiparallel branch (an SVD of [a; b]: not ported) */
int rd_quat_from_two_vectors(const double a[3], const double b[3], ok_quat* q);
#endif
