/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, modules M8 + M11: the synchronous system (THREADING=OFF), i.e. rdvio::Handler (rdvio/src/handler.cpp),
 * rdvio::FeatureTracker (feature_tracker.cpp: run, get_latest_state) and rdvio::Frontend (frontend.cpp) on modules M6
 * (map), M7 / rd_sys_image (images), M9 (initializer) and M10 (sliding-window tracker).
 *   Handler        : gyroscope / accelerometer pairing (linear interpolation of the gyroscope at accelerometer times), IMU
 *                    samples handed to the pending frames, predict_pose (propagate_state over the frontal IMU samples)
 *   FeatureTracker : per frame: CLAHE + pyramid, re-prediction of the frames after the latest optimized one, the IMU sample
 *                    at the previous image time, preintegration, LK tracking from the previous frame, the latest state,
 *                    keypoint detection on every sliding_window_tracker_frequent-th frame, the map trim
 *   Frontend       : initialization attempts on every issued frame until one succeeds, then mirror_frame + track
 * The members the synchronous flow never calls (synchronize_keymap, mirror_map, mirror_lastframe, attach_latest_frame,
 * solve_pnp) are not ported; the keymap is created like the C++ (its Map construction is a map event) and stays empty.
 * One system per process: the map hooks and the image statics are process-wide, like the C++ id counters and statics.
 * Derived from RD-VIO (Apache-2.0). C99.
 */
#ifndef RD_SYS_H
#define RD_SYS_H
#include "rd_map.h"
#include "rd_sys_config.h"
#include "rd_sys_image.h"
#include "rd_sys_init.h"
#include "rd_sys_swt.h"
#include "rd_imu_parsac.h"

enum { RD_SYS_INITIALIZING = 0, RD_SYS_TRACKING, RD_SYS_CRASH, RD_SYS_UNKNOWN };

typedef struct rd_sys rd_sys;

/* cfg is copied. parsac (may be NULL): the PARSAC estimators of module M10. pnp_mask NULL: the native
 * find_pnp_matrix_parsac_imu (rd_imu_parsac.c) if a solve_pnp_6pt is set (rd_sys_set_pnp_solver), else judge_track_status
 * is skipped; ess_mask NULL: the native find_essential_matrix_parsac (module M3,
 * with the process-wide bin confidences of the C++). sv: Ceres solve observers (may be NULL). map: extra map-event observers
 * (event only; may be NULL) */
rd_sys* rd_sys_create(const rd_cfg* cfg, const rd_swt_hooks* parsac, const rd_sv_hooks* sv, const rd_map_hooks* map);
void rd_sys_free(rd_sys* s);
/* Handler::track_gyroscope / track_accelerometer / track_camera; *out (may be NULL) = the returned predict_pose output */
void rd_sys_track_gyroscope(rd_sys* s, double t, double x, double y, double z, rd_pose* out);
void rd_sys_track_accelerometer(rd_sys* s, double t, double x, double y, double z, rd_pose* out);
/* takes the image reference (refs = 1 from rd_sys_image_new) */
void rd_sys_track_camera(rd_sys* s, rd_sys_image* image, rd_pose* out);
int rd_sys_state(const rd_sys* s);
/* native essential PARSAC calls with a point outside the (-1, 1)^2 grid (undefined in the C++; the mask is set to all 1) */
long rd_sys_parsac_grid_errors(const rd_sys* s);
/* solve_pnp_6pt (OpenCV EPnP + Rodrigues in float) for the native IMU-PARSAC; set before the first frame */
void rd_sys_set_pnp_solver(rd_sys* s, rd_pnp6_fn solve, void* ctx);
/* Handler::get_latest_state: the feature tracker's latest state (t = 0 and a zero quaternion when there is none) */
void rd_sys_latest_state(const rd_sys* s, double* t, rd_pose* pose);

#endif
