/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2-X GNSS error terms and the 4-DoF pose manifold, bit-exact C99 port.
 *
 * Derived from OKVIS2-X (ethz-mrl/OKVIS2-X commit 38043e4, okvis_ceres/{include/okvis/ceres,src}/
 * GpsErrorSynchronous.{hpp,cpp}, GpsErrorAsynchronous.{hpp,cpp}, PoseLocalParameterization.{hpp,cpp}):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2025, Mobile Robotics Lab / Technical University of Munich and ETH Zurich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The evaluation-order models of the Eigen 3.4.0
 *   expressions are MPL-2.0 (Eigen, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors). The names of
 *   the copyright holders may not be used to endorse derived products.
 *
 * C99, <math.h> <string.h> + the ported kernels (ok_imu, ok_kin, ok_param, ok_err, ok_eigen, ok_gps_init's inverse3).
 * Matrices are column-major unless stated; output Jacobians are ROW-major like the Eigen::RowMajor maps of the C++.
 * A pose block is [r(3), q(x,y,z,w)]. Reference flags: g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, Eigen 3.4.0 SSE2.
 * Verified against the real classes by okvis_port/reference_tools/okvis_gps_test.cc (tolerance 0, memcmp).
 *
 * As upstream, a minimal Jacobian is written only when the corresponding full Jacobian pointer is non-NULL; every
 * Evaluate returns 1 (the C++ returns true). Objects are caller-owned; evaluation of a GpsErrorAsynchronous mutates
 * it (re-preintegration), so it needs external synchronisation (the C++ guards that with a mutex).
 *
 * IMU propagation: GpsErrorAsynchronous::redoPreintegration is line for line ImuError::redoPreintegration of OKVIS2
 * (X's ImuError.cpp is identical to external/vio/okvis2's apart from the licence banner) except that (a) the
 * covariance recursion (dPdsigma_, P_delta_) is skipped when useImuCovariance is false and (b) the final
 * PseudoInverse::symmSqrtU / information_ update of ImuError is absent. Neither difference touches the integrals,
 * the sub-Jacobians or (when on) P_delta_, so this port calls ok_imu_redo_preintegration (the validated kernel) and
 * simply ignores the extra information / sqrt_information it leaves in the ok_imu_error.
 *
 * Deliberate deviations from the C++ (undefined behaviour or logging there):
 *   - upstream dereferences (it + 1) of the last measurement (past-the-end iterator); the value is only used when the
 *     coverage condition (front <= tk, back >= tg) is violated, which the caller must not do. n_meas == 0 returns 0.
 *   - OKVIS_ASSERT_* / LOG are compiled out in the reference build (NDEBUG).
 */
#ifndef OK_GPS_H
#define OK_GPS_H

#include <stddef.h>
#include "ok_imu.h"
#include "ok_kin.h"
#include "ok_param.h"

/* ---------------------------------------------- GpsErrorSynchronous ---------------------------------------------- */
typedef struct ok_gps_sync {
    double meas[3];        /* measurement_ */
    double lever[3];       /* gpsParameters_.r_SA */
    double info[9];        /* information_ */
    double covariance[9];  /* covariance_ = information.inverse() */
    double sqrt_info[9];   /* squareRootInformation_ = LLT(information).matrixL().transpose() */
} ok_gps_sync;
/* GpsErrorSynchronous(cameraId, measurement, information, gpsParameters) */
void ok_gps_sync_init(ok_gps_sync* e, const double meas[3], const double info[9], const double lever[3]);
void ok_gps_sync_set_information(ok_gps_sync* e, const double info[9]);
/* params {T_WS[7], T_GW[7]}; residuals[3]; jac[2] row-major 3x7 each; jacmin[2] row-major 3x6 each (any may be NULL, as may
 * jac / jacmin themselves). Evaluate == EvaluateWithMinimalJacobians(..., NULL). */
int ok_gps_sync_evaluate(const ok_gps_sync* e, const double* const params[2], double res[3], double* const* jac,
                         double* const* jacmin);

/* --------------------------------------------- GpsErrorAsynchronous --------------------------------------------- */
typedef struct ok_gps_async {
    double meas[3];        /* measurement_ */
    double lever[3];       /* gpsParameters_.r_SA */
    double info[9];        /* gpsInformation_ */
    double covariance[9];  /* gpsCovariance_ = information.inverse() */
    double sqrt_info[9];   /* squareRootInformation_ (GPS + IMU covariance), rewritten by every Evaluate */
    double error[3];       /* error_ : the unweighted error of the last Evaluate */
    ok_imu_error imu;      /* tk_ = imu.t0, tg_ = imu.t1, the measurement deque, the preintegration state */
    int use_imu_covariance; /* static GpsErrorAsynchronous::useImuCovariance (default 1) */
    int redo_always;        /* static GpsErrorAsynchronous::redoPropagationAlways (default 0) */
} ok_gps_async;
/* GpsErrorAsynchronous(measurement, information, imuMeasurements, imuParameters, tk, tg, gpsParameters) */
void ok_gps_async_init(ok_gps_async* e, const double meas[3], const double info[9], const double lever[3],
                       const ok_imu_meas* imu, size_t n, const ok_imu_params* p, ok_time tk, ok_time tg);
/* the (sigma_x, sigma_y, sigma_z) constructor: information = diag(1 / sigma^2) */
void ok_gps_async_init_sigma(ok_gps_async* e, const double meas[3], const double sigma[3], const double lever[3],
                             const ok_imu_meas* imu, size_t n, const ok_imu_params* p, ok_time tk, ok_time tg);
void ok_gps_async_free(ok_gps_async* e);
void ok_gps_async_set_information(ok_gps_async* e, const double info[9]);
/* params {T_WS[7] at tk, speed_and_bias[9] at tk, T_GW[7]}; residuals[3]; jac[3] row-major 3x7, 3x9, 3x7; jacmin[3]
 * row-major 3x6, 3x9, 3x6. */
int ok_gps_async_evaluate(ok_gps_async* e, const double* const params[3], double res[3], double* const* jac,
                          double* const* jacmin);
/* applyPreInt(T_WS_in, sb_in, T_WS_prop): propagates a given pose to tg with the CURRENT preintegration state (no
 * re-preintegration; it only updates redo_). T_in is a cached Transformation (ok_tf_from_rq(..., cached = 1)). */
int ok_gps_async_apply_preint(ok_gps_async* e, const ok_tf* T_in, const double sb_in[9], ok_tf* T_prop);

/* -------------------------------------------------- PoseManifold4d -------------------------------------------------
 * x = [r(3), q(x,y,z,w)] (7), tangent = (dx, dy, dz, dyaw) (4). */
#define OK_POSE4_AMBIENT_SIZE 7
#define OK_POSE4_TANGENT_SIZE 4
int ok_pose4_plus(const double x[7], const double delta[4], double out[7]);       /* plus / Plus */
int ok_pose4_minus(const double y[7], const double x[7], double out[4]);          /* minus / Minus (y - x) */
int ok_pose4_plus_jacobian(const double x[7], double J[28]);                       /* 7x4 row-major */
int ok_pose4_minus_jacobian(const double x[7], double J[28]);                      /* 4x7 row-major */
/* ceres::Manifold::RightMultiplyByPlusJacobian (not overridden by PoseManifold4d): out(num_rows x 4) =
 * A(num_rows x 7) * PlusJacobian(x), all row-major, as Ceres' Eigen expression evaluates it. */
int ok_pose4_right_multiply(const double x[7], int num_rows, const double* A, double* out);

#endif
