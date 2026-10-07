/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2-X GNSS fusion, ViGraph side: the gpsStatus_ state machine (ViGraph.cpp addGpsMeasurements, addGpsMeasurement,
 * initializationStrategy, checkForGpsInit, addGpsInitFactors, needsGpsReInit, needsFull / Pos / InitialGpsAlignment, the
 * reset functions, freeze / unfreeze of T_GW) on top of the graph accessors of ok_vigraph.h (the T_GW block, the per-state
 * GpsFactors) and the validated leaves ok_gps.{h,c} / ok_gps_init.{h,c}.
 *
 * Derived from OKVIS2-X (ethz-mrl/OKVIS2-X commit 38043e4, okvis_ceres/src/ViGraph.cpp), BSD-3-Clause, Copyright (c) 2015
 * Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab / Imperial College London, 2025 Mobile Robotics Lab /
 * Technical University of Munich and ETH Zurich (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt), with the Eigen 3.4.0
 * evaluation-order models of ok_eigen.c (MPL-2.0). Redistribution requires retaining these notices.
 *
 * Not ported: robust_gps_init = true (checkValidGpsMeasurements, the RANSAC branch's Align4DoF_Ceres refinement), geodetic data
 * types. Quirks kept: the reverse-order iteration of fixes and states, the gpsInitMap_ multimap (insert after equal keys) whose
 * entries repeat the residuals of a state once per entry, the duplicated boundary IMU measurement of gpsInitImuQueue_.
 *
 * OKVIS_PORT_GNSS_LOG=<file>: observe-only event log (see ok_vggps.c); the same records come from patch 0016 of the X reference.
 */
#ifndef OK_VGGPS_H
#define OK_VGGPS_H
#include "ok_vigraph.h"

enum { OK_GPS_OFF = 0, OK_GPS_IDLE = 1, OK_GPS_INITIALISING = 2, OK_GPS_INITIALISED = 3, OK_GPS_REINITIALISING = 4 };

/* ViSlamBackend::addGps on one graph: ok_vg_gps_enable + the policy object */
int ok_vgps_add_gps(ok_vg* g, const double r_SA[3], double yaw_error_threshold, int robust);
int ok_vgps_status(const ok_vg* g);
void ok_vgps_set_status(ok_vg* g, int status);
void ok_vgps_set_name(ok_vg* g, int name);       /* log tag: 0 realtime, 1 full */

/* addGpsMeasurements(deque, imuDeque, sids): fixes in deque order; sids (may be NULL, capacity >= n) receives the state ids in
 * deque order (push_front of the reverse iteration), *nsids their number. Returns the C++ bool. */
int ok_vgps_add_measurements(ok_vg* g, const ok_gps_fix* m, int n, const ok_imu_meas* imu, size_t nimu, uint64_t* sids, int* nsids);
/* addGpsMeasurement(poseId, meas, imu): the C++ bool */
int ok_vgps_add_measurement(ok_vg* g, uint64_t sid, const ok_gps_fix* m, const ok_imu_meas* imu, size_t nimu);
/* checkValidGpsMeasurements (robust_gps_init only): out (capacity n) = the accepted fixes in the C++ push order (reverse of the input); returns the count */
int ok_vgps_check_valid_measurements(ok_vg* g, const ok_gps_fix* in, int n, ok_gps_fix* out);
/* initializationStrategy(T_GW_est): the C++ bool; T_GW_est = coefficients [r, q xyzw] of the cached Transformation */
int ok_vgps_initialization_strategy(ok_vg* g, double T_GW_est[7]);
void ok_vgps_add_init_factors(ok_vg* g);
int ok_vgps_needs_reinit(ok_vg* g);
void ok_vgps_reinit(ok_vg* g);
int ok_vgps_needs_full_alignment(ok_vg* g, uint64_t* loss, uint64_t* align, double T_GW_new[7]);
int ok_vgps_needs_pos_alignment(ok_vg* g, uint64_t* loss, uint64_t* align, double pos_error[3]);
int ok_vgps_needs_initial_alignment(const ok_vg* g);
void ok_vgps_reset_full_alignment(ok_vg* g);
void ok_vgps_reset_pos_alignment(ok_vg* g);
void ok_vgps_reset_initial_alignment(ok_vg* g);
void ok_vgps_freeze(ok_vg* g);                 /* freezeGpsExtrinsics */
void ok_vgps_unfreeze(ok_vg* g);
int ok_vgps_is_fixed(const ok_vg* g);
int ok_vgps_is_observable(const ok_vg* g);

/* the observe-only log (NULL when OKVIS_PORT_GNSS_LOG is unset) */
void ok_gnss_logf(const char* fmt, ...);
int ok_gnss_log_enabled(void);
#endif
