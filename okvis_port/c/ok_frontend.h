/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 7b: the frontend data association (okvis_frontend Frontend.{hpp,cpp}: matchToMap(+ByThread),
 * matchMotionStereo, matchStereo, removeOutliers, doWeNeedANewKeyframe, the RANSAC glue, the Frame*Adapter
 * correspondence lists) and stereo_triangulation.cpp (triangulateFast).
 *
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt) with the Eigen 3.4.0 evaluation-order models of ok_eigen.c (MPL-2.0)
 * and the keypoint rasterisation of OpenCV 4.6 `cv::circle` (BSD-3-Clause / Apache-2.0, see okvis_port/NOTICE).
 * Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only.
 *
 * ---- scope ----
 * `ok_fe_data_association` is Frontend::dataAssociationAndInitialization for the configurations the port targets
 * (radial-tangential / equidistant pinhole, IMU on, 1-4 cameras). BRISK detection / description is Codex's leaf: the
 * descriptors of a frame are handed in with ok_fe_add_frame. What is NOT native yet (reached through hooks of ok_fe_est,
 * answered by the replay harness from the reference log):
 *   - the OpenGV RANSAC runs (GP3P absolute pose, rotation-only and Stewenius relative pose; module M7c);
 *   - place recognition (DBoW2 query, verifyRecognisedPlace) with attemptLoopClosure / addLoopClosureFrame (module M7d).
 * The frontend reads the estimator (ViSlamBackend, module 6) through the const accessors of ok_vslam.h and acts on it
 * only through the ok_fe_est table, whose entries mirror the ViSlamBackend methods the frontend calls one to one.
 *
 * ---- descriptor / RANSAC records of the reference (patch 0012; framed records in problem.bin, native endian) ----
 *   161 DESC    u32 ncam, ncam x { u32 nkp, nkp x 48 bytes }     (right after the ADDSTATES entry record of the frame)
 *   162 RANSAC  u32 kind (0 GP3P 3d2d, 1 rotation-only 2d2d, 2 Stewenius 2d2d, 3 GP3P of verifyRecognisedPlace),
 *               u32 numCorrespondences, u32 iterations, u32 nInliers, nInliers x i32, u32 rows, u32 cols,
 *               rows*cols x f64 (column-major model)
 */
#ifndef OK_FRONTEND_H
#define OK_FRONTEND_H
#include <stddef.h>
#include <stdint.h>
#include "ok_vslam.h"
#include "ok_eigen.h"

#define OK_FE_MAXCAM OK_VSB_MAXCAM

/* triangulation::triangulateFast: p1, e1, p2, e2 in the world frame, returns the homogeneous point */
void ok_fe_triangulate_fast(const double p1[3], const double e1[3], const double p2[3], const double e2[3], double sigma,
                            int* is_valid, int* is_parallel, double hp[4]);

typedef struct ok_fe_ransac {
    int iterations, ninliers;
    int* inliers;                   /* malloc'd by the hook, freed by the frontend */
    int rows, cols;
    double model[16];               /* column-major (3x4 transformation / 3x3 rotation) */
} ok_fe_ransac;

typedef struct ok_fe_params {
    double matching_threshold;      /* briskMatchingThreshold_ (frontend_parameters.matching_threshold) */
    float keyframe_overlap;         /* keyframeInsertionOverlapThreshold_ */
    int num_matching_threads;       /* the segmentation of the keypoints (run sequentially) */
    int imu_use;
    int do_loop_closures;
    int realtime_num_threads;       /* passed on to optimiseRealtimeGraph (no numerical effect) */
} ok_fe_params;

/* the ViSlamBackend calls the frontend makes (write path) and the hooks for what is not ported yet */
typedef struct ok_fe_est {
    void* ctx;
    uint64_t (*add_landmark)(void* ctx, const double hp[4], int initialised);
    int (*add_observation)(void* ctx, uint64_t lm, uint64_t state, uint32_t cam, uint32_t kp, int use_cauchy);
    int (*remove_observation)(void* ctx, uint64_t state, uint32_t cam, uint32_t kp);
    int (*set_observation_information)(void* ctx, uint64_t state, uint32_t cam, uint32_t kp, const double info[4]);
    int (*set_landmark)(void* ctx, uint64_t id, const double hp[4], int initialised);
    int (*merge_landmark)(void* ctx, uint64_t from, uint64_t into);
    int (*merge_landmarks)(void* ctx, const uint64_t* from, const uint64_t* into, int n);
    int (*set_pose)(void* ctx, uint64_t id, const double T7[7]);
    int (*optimise_realtime)(void* ctx, int num_iter, int num_threads, int verbose, int only_newest, int is_initialised);
    int (*clean_unobserved_landmarks)(void* ctx);
    void (*set_landmark_id)(void* ctx, uint64_t frame, uint32_t cam, uint32_t kp, uint64_t id);
    /* OpenGV RANSAC (M7c): kind as in record 162; returns 1 and fills `out` (out->inliers malloc'd) */
    int (*ransac)(void* ctx, int kind, int num_correspondences, ok_fe_ransac* out);
    /* the loop-closure block of dataAssociationAndInitialization (M7d): query + verify + attemptLoopClosure +
     * addLoopClosureFrame; returns 1 when a loop closure frame was added (landmarks: malloc'd, ascending) */
    int (*place_recognition)(void* ctx, uint64_t frame, uint64_t** landmarks, int* nlandmarks);
} ok_fe_est;

typedef struct ok_fe ok_fe;
ok_fe* ok_fe_new(const ok_vsb* b, const ok_fe_params* p, const ok_fe_est* est);
void ok_fe_free(ok_fe* f);
/* the descriptors of the multiframe `frame` (state id): nkp[c] x 48 bytes per camera */
void ok_fe_add_frame(ok_fe* f, uint64_t frame, int ncam, const int* nkp, const unsigned char* const* desc);
/* Frontend::dataAssociationAndInitialization; *as_keyframe is the decision. Returns the C++ bool (trackingQuality >= 0.01,
 * not modelled: always 1). */
int ok_fe_data_association(ok_fe* f, uint64_t frame, int* as_keyframe);
int ok_fe_is_initialised(const ok_fe* f);

#endif
