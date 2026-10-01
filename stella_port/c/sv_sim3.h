/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_SIM3_H
#define SV_SIM3_H

#include "sv_eigen_quaternion.h"

/* g2o::Sim3 (external/candidates/g2o/g2o/types/sim3/sim3.h, BSD-2, H. Strasdat
 * 2011): rotation quaternion r, translation t, scale s, with the operations
 * stella_vslam's loop closing uses -- constructor from (R, t, s) and from a
 * seven-vector tangent update (exp map), log(), inverse(), operator*, map().
 * The Eigen evaluation order of every expression is reproduced with the
 * MPL-2.0 primitives sv_eigen_quaternion / sv_linalg / sv_eigen_lu3 and
 * verified against real g2o + Eigen 3.4 (stella_port/reference_tools/
 * eigen_shape_tests_sim3.cc, check_sv_sim3.c is not needed: the test links
 * this file directly). Matrices are column-major (m[col*3+row]).
 *
 * Tangent order (g2o): [omega(3); upsilon(3); sigma]. */
typedef struct sv_sim3 {
    sv_quat r;
    double t[3];
    double s;
} sv_sim3;

/* Sim3() : identity */
void sv_sim3_identity(sv_sim3* out);
/* Sim3(const Quaternion&, t, s) : copies then normalizeRotation() */
void sv_sim3_from_quat(const sv_quat* q, const double t[3], double s, sv_sim3* out);
/* Sim3(const Matrix3&, t, s) : Quaternion(R) then normalizeRotation() */
void sv_sim3_from_rot(const double R[9], const double t[3], double s, sv_sim3* out);
/* Sim3(const Vector7&) : exp map (no normalizeRotation, like upstream) */
void sv_sim3_exp(const double update[7], sv_sim3* out);
/* log() */
void sv_sim3_log(const sv_sim3* a, double out[7]);
/* inverse() */
void sv_sim3_inverse(const sv_sim3* a, sv_sim3* out);
/* operator* : out may alias a or b */
void sv_sim3_mul(const sv_sim3* a, const sv_sim3* b, sv_sim3* out);
/* map(xyz) = s * (r * xyz) + t */
void sv_sim3_map(const sv_sim3* a, const double xyz[3], double out[3]);
/* Rotation()/scale()/translation() helpers */
void sv_sim3_rotation_matrix(const sv_sim3* a, double R[9]);

#endif
