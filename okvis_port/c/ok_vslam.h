/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 6: ViSlamBackend (okvis_ceres ViSlamBackend.{hpp,cpp}): the estimator POLICY that drives
 * the two graphs of module 5d: the realtime window graph and the full graph, the IMU frame / keyframe / loop-closure
 * frame sets, `eliminateImuFrames`, `applyStrategy` (keyframe -> pose-graph conversion with the maximum-covisibility
 * spanning tree, freezing, loop-closure frame conversion, frontier expansion), `optimiseRealtimeGraph`,
 * `optimiseFullGraph`, `attemptLoopClosure` (drift heuristic + rigid re-alignment), `addLoopClosureFrame`,
 * `synchroniseRealtimeAndFullGraph`, `cleanUnobservedLandmarks`, landmark merging and `doFinalBa`.
 *
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt) with the Eigen 3.4.0 evaluation-order models of ok_eigen.c (MPL-2.0)
 * and the keypoint rasterisation of OpenCV 4.6 `cv::circle` (BSD-3-Clause / Apache-2.0, see okvis_port/NOTICE).
 * Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> <limits.h> only.
 *
 * ---- what the backend takes as input and what it decides ----
 * Inputs are the calls the frontend / ThreadedSlam make (the `ok_vsb_*` functions mirror the C++ methods one to one) and
 * the multiframe data they read (keypoint positions / sizes, image size, the camera model, the landmark id of every
 * keypoint, which the frontend writes through MultiFrame::setLandmarkId = ok_vsb_set_landmark_id). Everything the
 * backend DECIDES is a call into one of its two graphs: each one is reported to hooks.trace in the layout of the
 * patch-0010 record of that mutation (see ok_vigraph.h), so a harness can regenerate and compare the whole mutation
 * stream, and each graph optimise() goes through hooks.solve (the logged solver result, or a native solve).
 *
 * ---- the backend entry records (patch 0011; framed records in problem.bin, native endian) ----
 *   They carry NO graph pointer; the record is written when the C++ method is entered (args) and, for tag | 0x100,
 *   when it returns (results). Time = u32 sec, u32 nsec; meas / T7 / kid as in ok_vigraph.h.
 *   128 ADDCAM   u32 do_extrinsics, f64 sigma_r, f64 sigma_alpha
 *   129 ADDIMU   as the graph record 34
 *   130 ADDSTATES Time, meas, u32 asKeyframe, f64 kptradius, multiframe
 *                 multiframe = u32 ncam, ncam x { u32 hlen, header[hlen] (camera model, ok_cam.h), T7 T_SC, u32 rows,
 *                   u32 cols, u32 nkp, nkp x {f32 x, f32 y, f32 size}, u32 nz, nz x {u32 kp, u64 landmark id} }
 *   131 SETKF u64 id, u32 flag        132 ADDLM_ID u64 id, f64 hp[4], u32 init     133 ADDLM_NEW f64 hp[4], u32 init
 *   134 SETLM u64 id, f64 hp[4], u32 init       135 SETCLASS u64 id, u32 class
 *   136 ADDOBS u64 landmark, u64 state, u32 cam, u32 kp, u32 useCauchy    137 RMOBS u64 state, u32 cam, u32 kp
 *   138 SETOBSINFO u64 state, u32 cam, u32 kp, f64 information[4]
 *   139 MERGELMS u32 n, n x u64 from, u32 n, n x u64 into     140 MERGELM u64 from, u64 into
 *   141 APPLYSTRATEGY u64 numKeyframes, numLoopClosureFrames, numImuFrames, u32 expand   res: u32 n, n x u64 affected
 *   142 OPTRT u32 numIter, numThreads, verbose, onlyNewestState, isInitialised          res: u32 n, n x u64 updated
 *   143 OPTFULL u32 numIter, numThreads, verbose
 *   144 SYNC (no args)                                                                  res: u32 n, n x u64 updated
 *   145 CLEANLM (no args)                                                               res: u32 removed
 *   146 LCATTEMPT u64 pose_i, pose_j, T7 T_Si_Sj, f64 information[36] (col-major), f64 drift   res: u32 ret, u32 skipFull
 *   147 ADDLCFRAME u64 id, u32 skipFull                                                 res: u32 n, n x u64 landmarks
 *   148 SETPOSE u64 id, T7   149 SETSB u64 id, f64 sb[9]   150 SETEXTR u64 id, u32 cam, T7   151 CLEAR
 *   153 FINALBA u32 numIter, f64 extrinsicsPositionUncertainty, f64 extrinsicsOrientationUncertainty
 *   160 ML       u64 frame, u32 cam, u32 kp, u64 landmark id       (every MultiFrame::setLandmarkId, whoever calls it)
 *   161 DESC / 162 RANSAC (patch 0012, the frontend's inputs; layouts in ok_frontend.h) are not backend calls: skipped here
 */
#ifndef OK_VSLAM_H
#define OK_VSLAM_H
#include <stddef.h>
#include <stdint.h>
#include "ok_vigraph.h"

enum { OK_B_ADDCAM = 128, OK_B_ADDIMU, OK_B_ADDSTATES, OK_B_SETKF, OK_B_ADDLM_ID, OK_B_ADDLM_NEW, OK_B_SETLM, OK_B_SETCLASS,
       OK_B_ADDOBS, OK_B_RMOBS, OK_B_SETOBSINFO, OK_B_MERGELMS, OK_B_MERGELM, OK_B_APPLYSTRATEGY, OK_B_OPTRT, OK_B_OPTFULL,
       OK_B_SYNC, OK_B_CLEANLM, OK_B_LCATTEMPT, OK_B_ADDLCFRAME, OK_B_SETPOSE, OK_B_SETSB, OK_B_SETEXTR, OK_B_CLEAR,
       OK_B_DETRADIUS, OK_B_FINALBA, OK_B_ML = 160, OK_B_DESC = 161, OK_B_RANSAC = 162, OK_B_RESULT = 0x100 };

#define OK_VSB_MAXCAM 4

typedef struct ok_vsb ok_vsb;

/* the keypoint data of a multiframe as the overlap computation needs it */
typedef struct ok_vsb_cam_view {
    int rows, cols, nkp, images_cleared;      /* images_cleared: MultiFrame::clearAllImages() was called (image(i).empty()) */
    double T_SC[7];                           /* MultiFrame::T_SC(i) (set by addStates; the NCameraSystem extrinsics) */
    float* kp;                                /* nkp x {x, y, size} */
    uint64_t* lm;                             /* landmark id per keypoint */
} ok_vsb_cam_view;
typedef struct ok_vsb_frame_view { int alive, ncam; ok_vsb_cam_view cam[OK_VSB_MAXCAM]; } ok_vsb_frame_view;
/* ViSlamBackend::overlapFraction(frameA, frameB) with kptradius_ (NaN when both images are cleared, as in the C++) */
double ok_vsb_overlap(const ok_vsb_frame_view* a, const ok_vsb_frame_view* b, double kptradius);

/* Eigen 3.4.0 AngleAxisd(q), Quaterniond(AngleAxisd) and QuaternionBase::angularDistance */
double ok_v3_stable_norm(const double v[3]);
void ok_quat_to_angle_axis(const ok_quat* q, double* angle, double axis[3]);
ok_quat ok_quat_from_angle_axis(double angle, const double axis[3]);
double ok_quat_angular_distance(const ok_quat* a, const ok_quat* b);

/* growable sorted set of u64 ids (std::set<StateId> / std::set<LandmarkId>) */
typedef struct ok_idset { uint64_t* a; int n, cap; } ok_idset;
int ok_idset_has(const ok_idset* s, uint64_t v);
int ok_idset_add(ok_idset* s, uint64_t v);                  /* 1 if inserted */
int ok_idset_del(ok_idset* s, uint64_t v);                  /* 1 if erased */
void ok_idset_free(ok_idset* s);

typedef struct ok_vsb_hooks {
    void* ctx;
    /* One call per graph-level mutation, in program order. graph: 0 realtime, 1 full, -1 not a graph record (op OK_B_ML).
     * op: the patch-0010 tag (OK_M_*) or OK_B_ML. a / r: the argument and result bytes in the layout of that record, with
     * every pointer field (ADDEXTOBS source term, MST created-term, CONVOBS term, SYNCIMU source graph) zeroed. */
    void (*trace)(void* ctx, int graph, int op, const void* a, size_t alen, const void* r, size_t rlen);
    /* ViGraph::optimise on graph `graph` (the graph is in the state before the solve; the call sites have already
     * made the same trace call for the OPT args through `trace_opt`): returns the solver result bytes in the OPT
     * result layout (changed blocks in ok_vg_blocks order, changed IMU terms, termination, iterations), malloc'd.
     * The backend applies them to the graph. Return 0 on failure. */
    int (*solve)(void* ctx, int graph, ok_vg* g, int max_iter, unsigned char** res, size_t* rlen);
} ok_vsb_hooks;

/* one camera of a multiframe */
typedef struct ok_vsb_cam_in {
    const unsigned char* header; size_t hlen;       /* camera model, ok_cam.h layout (as logged by ADDOBS) */
    double T_SC[7];
    int rows, cols, nkp;
    const float* kp;                                /* nkp x {x, y, size} */
    int nz; const uint32_t* nz_kp; const uint64_t* nz_id;   /* keypoints that already carry a landmark id */
} ok_vsb_cam_in;

ok_vsb* ok_vsb_new(const ok_vsb_hooks* h);
void ok_vsb_free(ok_vsb* b);
ok_vg* ok_vsb_graph(ok_vsb* b, int which);          /* 0 realtime, 1 full */

int ok_vsb_add_camera(ok_vsb* b, int do_extrinsics, double sigma_r, double sigma_alpha);
int ok_vsb_add_imu(ok_vsb* b, const ok_vg_imu_cfg* c);
int ok_vsb_add_states(ok_vsb* b, ok_time t, const ok_imu_meas* meas, size_t n, int as_keyframe, double kptradius, int ncam, const ok_vsb_cam_in* cams);
void ok_vsb_set_landmark_id(ok_vsb* b, uint64_t frame, uint32_t cam, uint32_t kp, uint64_t id);   /* MultiFrame::setLandmarkId (frontend) */
int ok_vsb_set_keyframe(ok_vsb* b, uint64_t id, int flag);
int ok_vsb_add_landmark_id(ok_vsb* b, uint64_t id, const double hp[4], int initialised);
uint64_t ok_vsb_add_landmark(ok_vsb* b, const double hp[4], int initialised);
int ok_vsb_set_landmark(ok_vsb* b, uint64_t id, const double hp[4], int initialised);
int ok_vsb_set_landmark_classification(ok_vsb* b, uint64_t id, int classification);
int ok_vsb_add_observation(ok_vsb* b, uint64_t lm, uint64_t state, uint32_t cam, uint32_t kp, int use_cauchy);
int ok_vsb_remove_observation(ok_vsb* b, uint64_t state, uint32_t cam, uint32_t kp);
int ok_vsb_set_observation_information(ok_vsb* b, uint64_t state, uint32_t cam, uint32_t kp, const double info[4]);
int ok_vsb_merge_landmark(ok_vsb* b, uint64_t from, uint64_t into);
int ok_vsb_merge_landmarks(ok_vsb* b, const uint64_t* from, const uint64_t* into, int n);
int ok_vsb_set_pose(ok_vsb* b, uint64_t id, const double T7[7]);
int ok_vsb_set_speed_and_bias(ok_vsb* b, uint64_t id, const double sb[9]);
int ok_vsb_set_extrinsics(ok_vsb* b, uint64_t id, int cam, const double T7[7]);

/* results are malloc'd arrays (ascending ids for sets, call order for vectors) */
int ok_vsb_apply_strategy(ok_vsb* b, size_t num_kf, size_t num_lc, size_t num_imu, int expand, uint64_t** affected, int* naffected);
int ok_vsb_optimise_realtime(ok_vsb* b, int num_iter, int num_threads, int verbose, int only_newest, int is_initialised, uint64_t** updated, int* nupdated);
int ok_vsb_optimise_full(ok_vsb* b, int num_iter, int num_threads, int verbose);
int ok_vsb_synchronise(ok_vsb* b, uint64_t** updated, int* nupdated);
int ok_vsb_clean_unobserved_landmarks(ok_vsb* b);
int ok_vsb_attempt_loop_closure(ok_vsb* b, uint64_t pose_i, uint64_t pose_j, const double T_Si_Sj[7], const double information[36], double drift_percentage, int* skip_full_graph_optimisation);
int ok_vsb_add_loop_closure_frame(ok_vsb* b, uint64_t id, int skip_full_graph_optimisation, uint64_t** landmarks, int* nlandmarks);
int ok_vsb_do_final_ba(ok_vsb* b, int num_iter, double ext_pos_unc, double ext_ori_unc);   /* ported from the source, not exercised by EuRoC (do_final_ba is off) */
int ok_vsb_clear(ok_vsb* b);

/* read-only state */
int ok_vsb_needs_full_graph_optimisation(const ok_vsb* b);
int ok_vsb_is_loop_closing(const ok_vsb* b);
int ok_vsb_is_loop_closure_available(const ok_vsb* b);
const ok_idset* ok_vsb_key_frames(const ok_vsb* b);
const ok_idset* ok_vsb_imu_frames(const ok_vsb* b);
const ok_idset* ok_vsb_loop_closure_frames(const ok_vsb* b);
uint64_t ok_vsb_current_state_id(const ok_vsb* b);
uint64_t ok_vsb_most_overlapped_state_id(const ok_vsb* b, uint64_t frame, int consider_loop_closure_frames);
double ok_vsb_overlap_fraction(const ok_vsb* b, uint64_t frame_a, uint64_t frame_b);

/* read access for the frontend (module 7b): the multiframe of a state (NULL if absent), the number of multiframes
 * (ViSlamBackend::numFrames), the camera model of camera `cam` (as logged by addStates), isInImuWindow */
const ok_vsb_frame_view* ok_vsb_frame(const ok_vsb* b, uint64_t id);
int ok_vsb_num_frames(const ok_vsb* b);
const ok_cam* ok_vsb_camera(const ok_vsb* b, int cam);
int ok_vsb_is_in_imu_window(const ok_vsb* b, uint64_t id);

/* ViGraph::optimise on the C graph (ok_vsolve.c): builds the Ceres problem from the graph's Problem bookkeeping in program
 * order, runs the module-4 solver and writes the changes back (parameter blocks in place, IMU re-integration state);
 * returns the OPT-record result bytes (malloc'd). ok_vsb_solve_native is the same as an ok_vsb_hooks.solve hook. */
int ok_vg_solve_native(ok_vg* g, int max_iter, unsigned char** res, size_t* rlen);
int ok_vsb_solve_native(void* ctx, int graph, ok_vg* g, int max_iter, unsigned char** res, size_t* rlen);

/* OpenCV 4.6 cv::circle(img, center, radius, 255, FILLED) on a rows x cols CV_8UC1 buffer (exposed for the unit test) */
void ok_vsb_circle_filled(unsigned char* img, int rows, int cols, int cx, int cy, int radius);

#endif
