/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 1: IMU propagation / preintegration (okvis::ceres::ImuError).
 *
 * Derived from OKVIS2 (okvis_ceres/src/ImuError.cpp, okvis_kinematics, okvis_time, okvis_ceres/ode):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see docs/okvis2_license_audit.md and okvis_port/LICENSES). Time/Duration semantics
 *   derive from ROS time (Willow Garage, BSD). Eigen-derived evaluation-order models live in ok_eigen.c
 *   (MPL-2.0). Redistribution requires retaining these notices; names of ETH Zurich, Imperial College London
 *   and TUM may not be used to endorse derived products.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> <limits.h> only. Matrices are column-major.
 * Bit-exact against the single-threaded reference build (see okvis_port/HANDOVER.md).
 *
 * ---- record layouts of the reference dumps (okvis_port/reference/patches/0002-imu-error-dump.patch) ----
 * native endian; "meas" = u64 n, then n x {u32 sec, u32 nsec, f64 gyr[3], f64 acc[3]};
 * "params" = f64 {sigma_g_c, sigma_a_c, sigma_gw_c, sigma_aw_c, g, g_max, a_max}; "time" = u32 sec, u32 nsec.
 *   imu_prop.bin    : meas, params, f64 T0[7] (r[3], q xyzw), f64 sb0[9], time t_start, time t_end,
 *                     u32 had_cov, u32 had_jac, i64 ret, f64 T_out[7], f64 sb_out[9],
 *                     i64 ret1, f64 T1_out[7], f64 sb1_out[9], f64 cov[225], f64 jac[225]
 *                     (ret1/T1/sb1/cov/jac: a second call with covariance+jacobian requested)
 *   imu_preint.bin  : f64 sb[9], u64 steps, SNAPSHOT(full)        (state after redoPreintegration)
 *   imu_append.bin  : SNAPSHOT(full, pre), f64 sb[9], meas(new), time t_1, i64 ret, SNAPSHOT(post, meas = size only)
 *   imu_eval.bin    : see check_ok_imu.c (check_eval) for the field order
 *   imu_initpose.bin: meas, f64 T_out[7]
 * SNAPSHOT = meas|u64 size, params, time t0, time t1, f64 Delta_q[4 xyzw], C_integral[9], C_doubleintegral[9],
 *   acc_integral[3], acc_doubleintegral[3], cross[9], dalpha_db_g[9], dv_db_g[9], dp_db_g[9], P_delta[225],
 *   sb_ref[9], u32 redo, u32 redo_counter, information[225], sqrt_information[225], u32 n, n x dPdsigma[225]
 */
#ifndef OK_IMU_H
#define OK_IMU_H

#include <stdint.h>
#include <stdlib.h>
#include "ok_eigen.h"
#include "ok_time.h"

typedef struct ok_imu_meas { ok_time t; double gyr[3]; double acc[3]; } ok_imu_meas;
typedef struct ok_imu_params {
    double sigma_g_c, sigma_a_c, sigma_gw_c, sigma_aw_c; /* noise densities */
    double g;                                            /* gravity [m/s^2] */
    double g_max, a_max;                                 /* saturation */
} ok_imu_params;

/* (a - b).toSec() with okvis::Time / Duration semantics */
double ok_time_diff_sec(ok_time a, ok_time b);
int ok_time_lt(ok_time a, ok_time b);
int ok_time_eq(ok_time a, ok_time b);

/* ImuError::propagation: pose T_WS = [r(3), q(x,y,z,w)] (q must be unit, as stored by Transformation) and
 * speed/biases sb = [v_W(3), b_g(3), b_a(3)] are advanced in place. cov / jac (column-major 15x15) optional.
 * Returns the number of integration steps, -1 if the measurements do not reach t_end. */
int ok_imu_propagation(const ok_imu_meas* meas, size_t n, const ok_imu_params* p, double T_WS[7], double sb[9],
                       ok_time t_start, ok_time t_end, double* cov, double* jac);

/* ImuError::initPose: T_WS = [r(3), q(x,y,z,w)] from the mean accelerometer reading (assumes no acceleration).
 * Returns 0 for an empty deque (T_WS = identity), 1 otherwise. */
int ok_imu_init_pose(const ok_imu_meas* meas, size_t n, double T_WS[7]);

/* ImuError state (preintegration with the reference biases sb_ref). meas is owned (malloc). */
typedef struct ok_imu_error {
    ok_imu_params params;
    ok_time t0, t1;
    ok_imu_meas* meas; size_t n_meas, cap_meas;
    ok_quat delta_q;
    double C_integral[9], C_doubleintegral[9], acc_integral[3], acc_doubleintegral[3], cross[9];
    double dalpha_db_g[9], dv_db_g[9], dp_db_g[9];
    double P_delta[225];
    double sb_ref[9];
    int redo, redo_counter;
    double information[225], sqrt_information[225];
    double dPdsigma[4][225];
} ok_imu_error;

void ok_imu_error_init(ok_imu_error* e, const ok_imu_meas* meas, size_t n, const ok_imu_params* p, ok_time t0, ok_time t1);
void ok_imu_error_free(ok_imu_error* e);
/* ImuError::redoPreintegration; returns the number of steps (or -1) */
int ok_imu_redo_preintegration(ok_imu_error* e, const double sb[9]);
/* ImuError::append (IMU-state merge) */
int ok_imu_append(ok_imu_error* e, const double sb[9], const ok_imu_meas* meas, size_t n, ok_time t_1);
/* ImuError::EvaluateWithMinimalJacobians via Evaluate (ceres call): residuals[15], jacobians[4] row-major
 * 15x7, 15x9, 15x7, 15x9 (any entry may be NULL). params = {pose0[7], sb0[9], pose1[7], sb1[9]}.
 * Returns 1 (success flag of the evaluation). */
int ok_imu_evaluate(ok_imu_error* e, const double* const params[4], double residuals[15], double* const jacobians[4]);

#endif
