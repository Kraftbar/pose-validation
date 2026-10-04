/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M1: IMU pre-integration (rdvio_estimation/preintegrator.{h,cpp}) and its residual / Jacobians
 * (rdvio_estimation/ceres/preintegration_factor.h: CeresPreIntegrationErrorFactor, CeresPreIntegrationPriorFactor) plus the
 * quaternion manifold Plus (ceres/quaternion_parameterization.h).
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified.
 * Eigen 3.4.0 evaluation-order models: MPL-2.0. See rdvio_port/NOTICE.
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> <limits.h>. Column-major matrices, quaternions (x,y,z,w).
 *
 * Dump record layouts (rdvio_port/reference/patches/0003-preintegration-dump.patch), native endian:
 *   integ.bin : u32 n, u32 flags(1 jac, 2 cov), f64 t, bg[3], ba[3], cov_w[9], cov_a[9], cov_bg[9], cov_ba[9], n x {t, w[3], a[3]},
 *               u32 ret, [ret: f64 delta.t, q[4], p[3], v[3], cov[225], sqrt_inv_cov[225], dq_dbg[9], dp_dbg[9], dp_dba[9], dv_dbg[9], dv_dba[9]]
 *   pred.bin  : old q[4] p[3] v[3] bg[3] ba[3], delta t q[4] p[3] v[3], new q[4] p[3] v[3] bg[3] ba[3]
 *   pie.bin   : u32 has_jac, u32 mask(10 bits), u32 via_prior, 10 param blocks (4,3,3,3,3,4,3,3,3,3), imu_i q[4] p[3], imu_j q[4] p[3],
 *               bg_i_0[3], ba_i_0[3], delta t q[4] p[3] v[3], jac dq_dbg dp_dbg dp_dba dv_dbg dv_dba (5x9), sqrt_inv_cov[225],
 *               residual[15], then for every set mask bit k: 15 x size(k) doubles, row-major
 *   plus.bin  : q[4], dq[3], out[4]
 */
#ifndef RD_IMU_H
#define RD_IMU_H
#include "rd_lie.h"

#define RD_GRAVITY_NOMINAL 9.80665

typedef struct rd_imu_sample { double t; double w[3]; double a[3]; } rd_imu_sample;

typedef struct rd_delta {
    double t;
    ok_quat q;
    double p[3], v[3];
    double cov[225];          /* 15x15 ordered q, p, v, bg, ba */
    double sqrt_inv_cov[225];
} rd_delta;

typedef struct rd_pre_jac { double dq_dbg[9], dp_dbg[9], dp_dba[9], dv_dbg[9], dv_dba[9]; } rd_pre_jac;

typedef struct rd_preint {
    double cov_w[9], cov_a[9], cov_bg[9], cov_ba[9];  /* continuous noise covariances */
    rd_delta delta;
    rd_pre_jac jac;
} rd_preint;

void rd_pi_reset(rd_preint* pi);
void rd_pi_increment(rd_preint* pi, double dt, const rd_imu_sample* d, const double bg[3], const double ba[3], int compute_jacobian,
                     int compute_covariance);
/* returns 0 for an empty data list (state untouched), else 1 */
int rd_pi_integrate(rd_preint* pi, const rd_imu_sample* data, int n, double t, const double bg[3], const double ba[3],
                    int compute_jacobian, int compute_covariance);
void rd_pi_compute_sqrt_inv_cov(rd_preint* pi);

typedef struct rd_motion { double v[3], bg[3], ba[3]; } rd_motion;
typedef struct rd_pose { ok_quat q; double p[3]; } rd_pose;
void rd_pi_predict(const rd_preint* pi, const rd_pose* old_pose, const rd_motion* old_motion, rd_pose* new_pose, rd_motion* new_motion);

/* QuaternionParameterization::Plus */
void rd_quat_plus(const double q[4], const double dq[3], double out[4]);

/* CeresPreIntegrationErrorFactor::Evaluate. params[0..9] = q_i(4) p_i(3) v_i(3) bg_i(3) ba_i(3) q_j(4) p_j(3) v_j(3) bg_j(3) ba_j(3).
 * jac[k] (k = 0..9) may be NULL; otherwise it receives the row-major 15 x size(k) Jacobian (size 4,3,3,3,3,4,3,3,3,3). */
void rd_pie_eval(const rd_preint* pre, const ok_quat* imu_i_q, const double imu_i_p[3], const ok_quat* imu_j_q, const double imu_j_p[3],
                 const double bg_i_0[3], const double ba_i_0[3], const double* const params[10], double residual[15], double* jac[10]);
#endif
