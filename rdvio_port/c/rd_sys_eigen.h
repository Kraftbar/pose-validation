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
/* Matrix4d::inverse(): Eigen 3.4.0 compute_inverse_size4<Architecture::Target, double> (SSE2 packets, LU/arch/InverseSize4.h),
 * transcribed lane by lane. A, out column-major. */
void rd_m4_inverse(const double A[16], double out[16]);
/* Matrix4d * Matrix4d: lazy coefficient-based product, packets down each column (etor_product_packet_impl): every entry a
 * left fold over k. Products of products are evaluated into temporaries, so chains are left-to-right rd_m4_mul calls. */
void rd_m4_mul(const double A[16], const double B[16], double out[16]);
/* M.transpose().inverse() of a Matrix3d: the Transpose stays an expression (nested_eval), so the determinant
 * (cofactors_col0 .* matrix.col(0)).sum() reads a strided column and takes the unvectorized tree c0 m0 + (c1 m1 + c2 m2) */
void rd_inverse3_t(const double m[9], double out[9]);
/* M.transpose().inverse() * B (Matrix3d): measured against Eigen 3.4.0 -- unlike A * B (rows 0-1 left folds, row 2
 * a0 + (a1 + a2), ok_m3_mul) every row is a left fold */
void rd_m3_mul_tinv(const double Mtinv[9], const double B[9], double out[9]);
/* |q.homogeneous()^T (F p.homogeneous())| / |(F p.homogeneous()).head<2>()|. F * p.homogeneous() is the homogeneous product
 * F.leftCols<2>() * p + F.col(2) (rows: (F(i,0) x + F(i,1) y) + F(i,2)); the 1x3 * 3x1 inner product is a vectorized redux of
 * the evaluated [x, y, 1]: (x l0 + y l1) + 1 l2 */
double rd_epipolar_dist(const double F[9], const double p[2], const double q[2]);
#endif
