/* SPDX-License-Identifier: MPL-2.0 */
/* Copied from stella_port/c/sv_eigen_svd.h (same author, MPL-2.0) with the identifiers renamed sv_eigen_ -> ok_eigen_; no other change. */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * C99 port of Eigen 3.4's Eigen::JacobiSVD<MatrixType, ColPivHouseholderQR
 * Preconditioner> (double scalar, ComputeFullU|ComputeFullV), restricted to
 * the two shapes stella_vslam's initializer solvers actually instantiate
 * (see external/candidates/stella_vslam/src/stella_vslam/solve/
 * {homography_solver,fundamental_solver,essential_solver}.cc):
 *   - Eigen::JacobiSVD<Eigen::Matrix3d>              (square, no QR precond)
 *   - Eigen::JacobiSVD<Eigen::Matrix<double,Dynamic,9>>  (N x 9, N>=8)
 * Follows (clean room, Eigen source only, MPL-2.0):
 *   external/eigen/Eigen/src/SVD/JacobiSVD.h
 *   external/eigen/Eigen/src/SVD/SVDBase.h
 *   external/eigen/Eigen/src/Jacobi/Jacobi.h
 *   external/eigen/Eigen/src/misc/RealSvd2x2.h
 * and ok_eigen_qr.h for the QR preconditioning step used by the N x 9 case.
 *
 * Only the outputs stella's solvers actually read are computed:
 *   - Matrix3d case: full U, full V and the 3 singular values.
 *   - N x 9 case: full V (9x9), the 9 singular values and rank() (used by
 *     homography_solver::compute_H_21). U is intentionally never built --
 *     stella asks Eigen for ComputeFullU|ComputeFullV on this shape too,
 *     but never reads svd.matrixU() for it, and U does not feed back into
 *     the computation of V/singular values, so skipping its O(N^2)
 *     Householder-sequence expansion changes nothing this port needs to
 *     match bit-for-bit.
 */
#ifndef OK_EIGEN_SVD_H
#define OK_EIGEN_SVD_H

#ifdef __cplusplus
extern "C" {
#endif

/* A, U, V column-major 3x3 (9 doubles each); sv[3] descending. */
void ok_eigen_jacobisvd_3x3(const double A[9], double U[9], double V[9], double sv[3]);

/* Eigen::JacobiSVD<Matrix4d>: A, U, V column-major 4x4; sv[4] descending. */
void ok_eigen_jacobisvd_4x4(const double A[16], double U[16], double V[16], double sv[4]);

/* A: column-major N x 9 (N >= 8, N doubles per column, 9 columns).
 * V: column-major 9x9 output. sv[9] descending. *rank_out: SVDBase::rank().
 */
void ok_eigen_jacobisvd_Nx9_v(const double *A, int N, double V[81], double sv[9], int *rank_out);

#ifdef __cplusplus
}
#endif

#endif /* OK_EIGEN_SVD_H */
