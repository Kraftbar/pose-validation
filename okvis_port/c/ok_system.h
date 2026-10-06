/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 8: the system driver (okvis_multisensor_processing ThreadedSlam.cpp: init, processFrame,
 * optimisePublishMarginalise, stopThreading; Frontend::detectAndDescribe; ViSlamBackend::writeFinalCsvTrajectory;
 * TrajectoryOutput::writeStateToCsv) run sequentially, i.e. in the schedule of the deterministic reference (patch 0001:
 * the realtime optimisation and the loop-closure optimisation finish within the frame that starts them; patch 0006: no
 * published state is dropped).
 *
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt) with the Eigen 3.4.0 evaluation-order models of ok_eigen.c (MPL-2.0).
 * Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <stdlib.h> <string.h> <stdio.h> (the CSV writers) only.
 *
 * ---- feeding ----
 * In the order of the EuRoC DatasetReader: for every image time t, first every IMU measurement up to and including the
 * first one later than t + 0.021 s (ok_sys_add_imu), then the images of t (ok_sys_add_frame). A frame is processed when it
 * is added (ThreadedSlam::processFrame finds its IMU then). Images are 8-bit grayscale, rows x cols of the camera, one per
 * camera (NULL for a camera without an image at t).
 *
 * ---- what is not ported ----
 * enforce_realtime (wall-clock budgets), parallel detection (no numerical effect), the CNN, depth / virtual cameras,
 * IMU-less operation (the constant-velocity pose guess), do_final_ba (ok_vsb_do_final_ba is not exercised), the
 * multi-session components, the visualisation and the realtime IMU propagation of the publisher (not part of the CSVs),
 * Frontend::clear / estimator.clear after a failed initialisation (ok_sys_add_frame returns -2 there).
 */
#ifndef OK_SYSTEM_H
#define OK_SYSTEM_H
#include <stdint.h>
#include <stdio.h>
#include "ok_config.h"
#include "ok_frontend.h"

/* the ViSlamBackend calls ThreadedSlam makes (constructor, processFrame, optimisePublishMarginalise). A table passed to
 * ok_sys_new must be complete (on_features optional); without one the ok_vsb_* functions are called directly. A validation
 * harness compares the calls with the reference log before executing them. */
typedef struct ok_sys_be {
    void* ctx;
    int (*add_imu)(void* ctx, const ok_vg_imu_cfg* c);
    int (*add_camera)(void* ctx, int do_extrinsics, double sigma_r, double sigma_alpha);
    int (*add_states)(void* ctx, ok_time t, const ok_imu_meas* meas, size_t n, int as_keyframe, double kptradius, int ncam,
                      const ok_vsb_cam_in* cams);
    int (*set_keyframe)(void* ctx, uint64_t id, int flag);
    int (*optimise_realtime)(void* ctx, int num_iter, int num_threads, int verbose, int only_newest, int is_initialised,
                             uint64_t** updated, int* nupdated);
    int (*synchronise)(void* ctx, uint64_t** updated, int* nupdated);
    int (*apply_strategy)(void* ctx, size_t num_kf, size_t num_lc, size_t num_imu, int expand, uint64_t** affected, int* naffected);
    int (*optimise_full)(void* ctx, int num_iter, int num_threads, int verbose);
    /* observer of the BRISK output of every camera (keypoints x y size, 48-byte descriptors), before addStates */
    void (*on_features)(void* ctx, ok_time t, int cam, size_t n, const float* kp, const unsigned char* desc);
} ok_sys_be;

/* one published state (TrajectoryOutput row): time, T_WS = [r, q xyzw], speed and biases [v_W, b_g, b_a] */
typedef struct ok_sys_state { ok_time t; uint64_t id; double T_WS[7]; double sb[9]; } ok_sys_state;
typedef void (*ok_sys_publish_fn)(void* ctx, const ok_sys_state* s);

typedef struct ok_sys ok_sys;

/* b: the backend to drive (NULL: a new one that solves natively); est: the frontend's backend calls (NULL: direct);
 * be: ThreadedSlam's backend calls (NULL: direct); vocabulary: the DBoW2 payload of ok_dbow.h (NULL: no loop closures
 * even if the config asks for them). Returns NULL on an unsupported configuration (message in err). */
ok_sys* ok_sys_new(const ok_cfg* cfg, ok_vsb* b, const ok_fe_est* est, const ok_sys_be* be,
                   const unsigned char* vocabulary, size_t nvocabulary, char* err, size_t errlen);
void ok_sys_free(ok_sys* s);
ok_vsb* ok_sys_backend(ok_sys* s);
void ok_sys_set_publish(ok_sys* s, ok_sys_publish_fn fn, void* ctx);

/* ThreadedSlam::addImuMeasurement (stamp, accelerometers, gyroscopes) */
int ok_sys_add_imu(ok_sys* s, ok_time t, const double acc[3], const double gyr[3]);
/* ThreadedSlam::addImages + processFrame. Returns 1 processed, 0 dropped (startup: no IMU before the frame, or too few
 * keypoints before initialisation), -1 the IMU does not reach t + 0.02 s yet, -2 not ported (see above). */
int ok_sys_add_frame(ok_sys* s, ok_time t, const unsigned char* const* images);

/* TrajectoryOutput::writeStateToCsv / ViSlamBackend::writeFinalCsvTrajectory (rpg = false: the EuRoC-style rows) */
void ok_sys_write_csv_header(FILE* f);
void ok_sys_write_state_csv(FILE* f, const ok_sys_state* s);
int ok_sys_write_final_csv(ok_sys* s, FILE* f);

#endif
