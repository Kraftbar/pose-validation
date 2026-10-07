/* SPDX-License-Identifier: MIT AND BSD-3-Clause AND MPL-2.0 */
/* Basalt pure-C port, module M1: Sophus 1.24.6 (MIT) SO3 / SE3 and basalt-headers (BSD-3) sophus_utils Jacobians,
 * as instantiated by the executed EuRoC VIO path, for float (X = f) and double (X = d).
 * Eigen 3.4.0 evaluation order: bs_eigenf.h (MPL-2.0). Reference flags: g++ 13 -O2 -DNDEBUG -ffp-contract=off
 * -fno-fast-math, SSE2 baseline. Quaternions are stored like Eigen::Quaternion coeffs (x, y, z, w); matrices are
 * column-major (m[row + nrows*col]).
 *
 * Executed-path audit (grep of estimator, linearization_abs_qr, imu, optical flow, ba_utils), type in brackets:
 *   SO3 exp(Vec3) [f]  (PoseState::incPose, IntegratedImuMeasurement::propagateState / residual)
 *   SO3::log [f]       (residual, PoseVelState::diff)           SO3 * SO3 [f]   SO3::inverse [f]   SO3::matrix [f]
 *   SO3 * Vec3 [f]     (predictState, propagateState)           SO3::hat [f, d] (d: computeEssential)
 *   rightJacobianSO3 / rightJacobianInvSO3 / leftJacobianInvSO3 [f] (propagateState, residual)
 *   SE3 * SE3, SE3::inverse, matrix(), matrix3x4(), Adj() [f] (computeRelPose, triangulate, measure), rotationMatrix() [d]
 *   SE3<float> <- SE3<double> cast (calib.T_i_c), SO3(Quaternion)/setQuaternion normalisation [f, d]
 *   computeRelPose (ba_utils.h) with and without the 6x6 Jacobians [f]
 *   Eigen::Quaternion::FromTwoVectors [f] (estimator initial orientation)
 * leftJacobianSO3 and the Sim3 / decoupled-SE3 Jacobians are ported or skipped as noted in basalt_port/HANDOVER.md.
 * SOPHUS_ENSURE aborts (exp: |q|^2 != 1, log: tiny |w|) are not reproduced; callers must not hit them.
 */
#ifndef BS_LIE_H
#define BS_LIE_H

#include "bs_eigenf.h"

typedef struct bs_quatf { float x, y, z, w; } bs_quatf;
typedef struct bs_quatd { double x, y, z, w; } bs_quatd;
typedef bs_quatf bs_so3f;
typedef bs_quatd bs_so3d;
typedef struct bs_se3f { bs_so3f so3; float t[3]; } bs_se3f;
typedef struct bs_se3d { bs_so3d so3; double t[3]; } bs_se3d;

#define BS_LIE_DECL(S, X) \
  void bs_so3##X##_identity(bs_so3##X* o); \
  void bs_so3##X##_normalize(bs_so3##X* q);                                          /* SO3Base::normalize */ \
  void bs_so3##X##_from_quat(const bs_quat##X* q, bs_so3##X* o);                     /* SO3(QuaternionBase) / setQuaternion: copy + normalize */ \
  void bs_so3##X##_exp(const S w[3], bs_so3##X* o);                                  /* SO3::exp */ \
  void bs_so3##X##_log(const bs_so3##X* q, S o[3]);                                  /* SO3::log */ \
  void bs_so3##X##_mul(const bs_so3##X* a, const bs_so3##X* b, bs_so3##X* o);        /* a*b incl. normalisation */ \
  void bs_so3##X##_inverse(const bs_so3##X* q, bs_so3##X* o);                        /* conjugate + normalisation */ \
  void bs_so3##X##_matrix(const bs_so3##X* q, S o[9]);                               /* toRotationMatrix */ \
  void bs_so3##X##_act(const bs_so3##X* q, const S p[3], S o[3]);                    /* SO3 * Vec3 */ \
  void bs_so3##X##_hat(const S w[3], S o[9]); \
  void bs_so3##X##_vee(const S m[9], S o[3]); \
  void bs_right_jacobian_so3##X(const S phi[3], S J[9]);        /* rightJacobianSO3 */ \
  void bs_right_jacobian_inv_so3##X(const S phi[3], S J[9]);    /* rightJacobianInvSO3 */ \
  void bs_left_jacobian_so3##X(const S phi[3], S J[9]);         /* leftJacobianSO3 */ \
  void bs_left_jacobian_inv_so3##X(const S phi[3], S J[9]);     /* leftJacobianInvSO3 */ \
  void bs_se3##X##_identity(bs_se3##X* o); \
  void bs_se3##X##_make(const bs_so3##X* r, const S t[3], bs_se3##X* o); \
  void bs_se3##X##_mul(const bs_se3##X* a, const bs_se3##X* b, bs_se3##X* o); \
  void bs_se3##X##_inverse(const bs_se3##X* a, bs_se3##X* o); \
  void bs_se3##X##_act(const bs_se3##X* a, const S p[3], S o[3]);                    /* SE3 * Vec3 */ \
  void bs_se3##X##_matrix(const bs_se3##X* a, S o[16]);                              /* 4x4 */ \
  void bs_se3##X##_matrix3x4(const bs_se3##X* a, S o[12]); \
  void bs_se3##X##_adj(const bs_se3##X* a, S o[36]);                                 /* Adj(), 6x6 [trans; rot] */ \
  void bs_inc_pose##X(const S inc[6], bs_se3##X* T);                                 /* PoseState::incPose */ \
  void bs_compute_rel_pose##X(const bs_se3##X* T_w_i_h, const bs_se3##X* T_i_c_h, const bs_se3##X* T_w_i_t, \
                              const bs_se3##X* T_i_c_t, S* d_rel_d_h /* 36 or NULL */, S* d_rel_d_t /* 36 or NULL */, \
                              bs_se3##X* out);                                       /* basalt::computeRelPose */ \
  int bs_quat##X##_from_two_vectors(const S a[3], const S b[3], bs_quat##X* o);      /* Quaternion::FromTwoVectors; 0 = antipodal (SVD branch, unsupported) */

BS_LIE_DECL(float, f)
BS_LIE_DECL(double, d)

/* SE3<NewScalar> SE3::cast<NewScalar>(): SO3 constructed from the cast quaternion (normalised), translation cast */
void bs_so3_f_from_d(const bs_so3d* q, bs_so3f* o);
void bs_so3_d_from_f(const bs_so3f* q, bs_so3d* o);
void bs_se3_f_from_d(const bs_se3d* a, bs_se3f* o);
void bs_se3_d_from_f(const bs_se3f* a, bs_se3d* o);

#endif
