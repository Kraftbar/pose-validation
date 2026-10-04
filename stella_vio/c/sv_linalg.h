/* SPDX-License-Identifier: MPL-2.0 */
#ifndef SV_LINALG_H
#define SV_LINALG_H

/* Small fixed-size (3x3 matrix / 3-vector) double-precision linear algebra
 * helpers shared by the module-3 solve/init files, reproducing the exact
 * Eigen 3.4 formulas/evaluation order used by stella_vslam's Mat33_t/Vec3_t
 * operations (Eigen::Matrix3d is column-major) -- MPL-2.0 (Eigen-derived):
 *   external/eigen/Eigen/src/LU/InverseImpl.h (compute_inverse<...,3>,
 *     cofactor_3x3: inv(i,j) = cofactor_3x3<j,i>(m) / det, det computed from
 *     the col-0 cofactors dotted with col 0)
 *   external/eigen/Eigen/src/LU/Determinant.h (determinant_impl<...,3>,
 *     bruteforce_det3_helper: row-0 cofactor expansion)
 * Matrices are stored column-major (m[col*3+row]), matching Eigen::Matrix3d
 * and this port's sv_eigen_svd.h convention.
 */

typedef struct sv_vec3 {
    double x[3];
} sv_vec3;

/* m: column-major 3x3 (9 doubles). Also correct for `A * B.transpose()`
 * (measured: same mixed rows-0-1-L/row-2-R rule as plain A*B -- see
 * sv_linalg.c and HANDOVER.md's "Eigen 3x3 double evaluation-order rules"). */
void sv_mat3_mul(const double a[9], const double b[9], double out[9]);
void sv_mat3_mulv(const double a[9], const double v[3], double out[3]);
/* `a.transpose() * v` where the transpose is used inline (lazy) in the
 * original Eigen expression, not materialized first -- measured as ALL
 * rows left-associative (NOT the mixed rule) -- see sv_linalg.c. */
void sv_mat3_mulv_lhs_transposed(const double a[9], const double v[3], double out[3]);
/* `a.transpose() * b` (matrix*matrix, LHS transpose inline) -- also ALL
 * rows left-associative (bisected against real Eigen, see sv_linalg.c). */
void sv_mat3_mul_lhs_transposed(const double a[9], const double b[9], double out[9]);
void sv_mat3_transpose(const double a[9], double out[9]);
double sv_mat3_det(const double m[9]);
/* Eigen Matrix3d::inverse() (cofactor/adjugate, see header). */
void sv_mat3_inverse(const double m[9], double out[9]);

double sv_vec3_dot(const double a[3], const double b[3]);
void sv_vec3_cross(const double a[3], const double b[3], double out[3]);
double sv_vec3_norm(const double a[3]);

#endif /* SV_LINALG_H */
