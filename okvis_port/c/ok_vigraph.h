/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 5d: the ViGraph / ViGraphEstimator state and every graph mutation
 * (okvis_ceres ViGraph.{hpp,cpp}, ViGraphEstimator.cpp, okvis_util MstGraph.hpp), expressed on the ceres::Problem
 * bookkeeping of ok_problem.c so that the Problem calls a mutation makes (and therefore the PROGRAM ORDER the solver
 * of module 4 depends on) are reproduced.
 *
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt) with the Eigen 3.4.0 evaluation-order models of ok_eigen.c (MPL-2.0).
 * Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only.
 *
 * ---- what a graph is ----
 * States (pose / speed-and-bias / extrinsics blocks, IMU links, priors, observations, pose-graph links), landmarks,
 * the global observation map, the cached covisibilities, the MST scratch of buildMst and the never-shrinking anyState
 * map. A parameter block is a heap object whose ADDRESS is its Problem "pointer" (ceres identifies blocks by the user
 * double*); a residual block's id is the address of the object that owns it (observation, link). Every Problem call is
 * appended to an event queue (ok_vg_event) that the replay harness drains and compares with the Problem log of the
 * reference (problem.bin, patch 0009) after each mutation.
 *
 * ---- the mutation log (patch 0010; framed records in problem.bin, tags >= 32; native endian) ----
 *   record = u64 graph (the ViGraph*), u32 alen, args[alen], results.       Time = u32 sec, u32 nsec;
 *   meas = u64 n, n x {Time, f64 gyr[3], f64 acc[3]};   T7 = f64 r[3], q[4] (xyzw);   kid = u64 frame, u32 cam, u32 kp.
 *   Written when the OUTERMOST mutation returns, i.e. after the P_* records of the Problem calls it made.
 *   32 NEW (ctor)        args: u64 problem
 *   33 ADDCAM            args: u32 do_extrinsics, f64 sigma_r, f64 sigma_alpha                   res: -
 *   34 ADDIMU            args: u32 use, T7 T_BS, f64 a_max, g_max, sigma_g_c, sigma_bg, sigma_a_c, sigma_ba, sigma_gw_c,
 *                              sigma_aw_c, f64 g0[3], a0[3], g
 *   35 INIT / 36 PROP    args: Time, meas, [INIT: u32 ncam, ncam x T7 T_SC | PROP: u32 isKeyframe]
 *                        res: u64 id, f64 pose[7], f64 speedAndBias[9]   (the new state's blocks)
 *   37 ADDLM_ID: u64 id, f64 hp[4], u32 initialised      38 ADDLM_NEW: f64 hp[4], u32 init; res u64 id
 *   39 RMLM: u64 id    40 SETLM_INIT: u64 id, u32 init    41 SETLMQ: u64 id, f64 quality
 *   42 SETLM_FULL: u64 id, f64 hp[4], u32 init    43 SETLM: u64 id, f64 hp[4]    44 SETCLASS: u64 id, u32 classification
 *   45 ADDOBS: u64 lm, kid, u32 useCauchy, f64 meas[2], f64 size, cam header (ok_cam.h)
 *   46 ADDEXTOBS: u64 lm, kid, u32 useCauchy, u64 src_ptr, reprojection payload (ok_cam header, meas, information)
 *   47 RMOBS: kid    48 COVIS: res (empty = cached) u32 1, u32 n, n x {u64 a, u32 m, m x {u64 b, u32 count}},
 *                              u32 nvisible, nvisible x u64
 *   49 ADDRELPOSE: u64 id0, id1, T7 T_S0S1, f64 information[36] (col-major)    50 RMRELPOSE: u64 id0, id1
 *   51 SETKF: u64 id, u32 flag    52 SETPOSE: u64 id, T7    53 SETSB: u64 id, f64 sb[9]    54 SETEXTR: u64 id, u32 cam, T7
 *   55 UPDLM: -     57 CLEANLM: res u32 count     58 RMALLOBS: u64 id
 *   56 OPT (optimise): args: u32 maxIterations, u32 linear_solver_type, f64 function_tolerance,
 *                        u32 nstates, nstates x {u64 id, u32 isKeyframe, Time, u32 nobs, ntp, ntpc, nrel},
 *                        u32 nblk, nblk x {u64 ptr, u32 size, u32 flags (1 fixed, 2 initialised), u64 fnv(values),
 *                          [landmark (size 4): f64 quality, u32 classification, u64 id]},
 *                        u32 nimu, nimu x {u64 state id, u64 fnv(ImuError snapshot without measurements)};
 *                      res:  u32 nchanged, nchanged x {u32 block index, f64 values[size]},
 *                        u32 nimu_changed, x {u64 state id, u32 redo, u32 redoCounter, f64 sb_ref[9]},
 *                        u32 termination_type, u32 num_iterations
 *                      (the blocks are listed per state in map order: pose, speed-and-bias, extrinsics not seen
 *                      before; then the landmarks)
 *   59 ELIM: u64 id, u64 refId; res: u64 keyframeId, T7 T_Sk_S, f64 v_Sk[3], u64 fnv(post-merge IMU snapshot)
 *   60 MERGELM: u64 from, into    61/62 FREEZE_POSES/UNFREEZE_POSES: u64 id [, u32 removeInCeres]
 *   63/64 FREEZE_SB/UNFREEZE_SB: same
 *   65 MST: args u32 n, n x u64 states, u32 m, m x u64 statesToConsider; res: (u32 0 | u32 1, u32 nedges, nedges x {u64 a, u64 b}
 *                        (MST edges as frame ids), u32 ncreated, x {u64 ref, u64 other, u64 term_ptr},
 *                        u32 nremovedTp, x {u64 a, u64 b}, u32 nremovedObs, x kid)
 *   66 ADDEXTTP: u64 ref, u64 other, u32 present, payload (TwoPoseStandardGraphErrorConst: ok_graph.h type 8)
 *   67 RMTPC_ALL: u64 id    68 RMTPC: u64 i, u64 j    70 RMSBPRIOR: u64 id; res u32 ret
 *   69 CONVOBS: u64 id; res: u32 ctr, u32 nobs, nobs x {kid, u64 reprojectionError ptr, u64 landmark id},
 *                        u32 nlm, x u64, u32 nconnected, x u64
 *   71 CLEAR: res u64 new problem    72 EXTVAR / 73 SOFTEXT (not used by the shipped configs)
 *   Direct accesses of ViSlamBackend to a graph (poke records):
 *   96 FIX_ADD (initial fixation PoseError on the newest pose)  97 FIX_REMOVE  98 LM_CONST: u32 constant (all landmarks)
 *   99 SETINFO: kid, f64 information[4]   100 COPYSTATE: u64 id, T7, f64 sb[9] (writes into this graph's blocks)
 *   101 SYNCIMU: u64 src graph, u64 state id (ImuError::syncFrom of the previous IMU link)
 */
#ifndef OK_VIGRAPH_H
#define OK_VIGRAPH_H
#include <stddef.h>
#include <stdint.h>
#include "ok_cam.h"
#include "ok_err.h"
#include "ok_gps.h"
#include "ok_graph.h"
#include "ok_imu.h"
#include "ok_kin.h"
#include "ok_problem.h"
#include "ok_twopose.h"

enum { OK_M_NEW = 32, OK_M_ADDCAM, OK_M_ADDIMU, OK_M_INIT, OK_M_PROP, OK_M_ADDLM_ID, OK_M_ADDLM_NEW, OK_M_RMLM,
       OK_M_SETLM_INIT, OK_M_SETLMQ, OK_M_SETLM_FULL, OK_M_SETLM, OK_M_SETCLASS, OK_M_ADDOBS, OK_M_ADDEXTOBS,
       OK_M_RMOBS, OK_M_COVIS, OK_M_ADDRELPOSE, OK_M_RMRELPOSE, OK_M_SETKF, OK_M_SETPOSE, OK_M_SETSB, OK_M_SETEXTR,
       OK_M_UPDLM, OK_M_OPT, OK_M_CLEANLM, OK_M_RMALLOBS, OK_M_ELIM, OK_M_MERGELM, OK_M_FREEZE_POSES,
       OK_M_UNFREEZE_POSES, OK_M_FREEZE_SB, OK_M_UNFREEZE_SB, OK_M_MST, OK_M_ADDEXTTP, OK_M_RMTPC_ALL, OK_M_RMTPC,
       OK_M_CONVOBS, OK_M_RMSBPRIOR, OK_M_CLEAR, OK_M_EXTVAR, OK_M_SOFTEXT,
       OK_M_POKE_FIX_ADD = 96, OK_M_POKE_FIX_REMOVE, OK_M_POKE_LM_CONST, OK_M_POKE_SETINFO, OK_M_POKE_COPYSTATE,
       OK_M_POKE_SYNCIMU };

/* ---- the Problem calls a mutation made (compare with problem.bin) ---- */
typedef struct ok_vg_event {
    int kind;                 /* OK_P_ADDPARAM, OK_P_SETMANIFOLD, OK_P_ADDRESID, OK_P_RMRESID, OK_P_RMPARAM, OK_P_SETCONST, OK_P_SETVAR */
    uint64_t a, b;            /* ADDPARAM: ptr, size; SETMANIFOLD: ptr, manifold kind (1 pose, 2 hpoint); ADDRESID: rb; others: ptr / rb */
    int loss, nb;             /* ADDRESID: loss function present (0 none, 1 Cauchy), parameter blocks */
    uint64_t v[OK_PB_MAXB];
} ok_vg_event;

typedef struct ok_vg_blk {    /* a parameter block: PoseParameterBlock (7), SpeedAndBiasParameterBlock (9), HomogeneousPointParameterBlock (4) */
    double x[9];
    int size, fixed, initialised;
    int kind;                 /* the manifold set on it: 0 none, 1 PoseManifold, 2 HomogeneousPointManifold, 3 PoseManifold4d (T_GW) */
    uint64_t id;
    ok_time ts;
} ok_vg_blk;

typedef struct ok_vg ok_vg;

typedef struct ok_vg_imu_cfg {
    int use;
    double T_BS[7];
    double a_max, g_max, sigma_g_c, sigma_bg, sigma_a_c, sigma_ba, sigma_gw_c, sigma_aw_c, g0[3], a0[3], g;
} ok_vg_imu_cfg;

typedef struct ok_vg_kid { uint64_t frame; uint32_t cam, kp; } ok_vg_kid;

/* results of convertToPoseGraphMst */
typedef struct ok_vg_mst_result {
    int ret;
    int nmst; uint64_t (*mst)[2];                       /* MST edges as frame ids */
    int ncreated; uint64_t (*created)[2]; void** created_term;   /* (reference, other) and the term (ok_twopose*) */
    int nremoved_tp; uint64_t (*removed_tp)[2];
    int nremoved_obs; ok_vg_kid* removed_obs;
} ok_vg_mst_result;
/* results of convertToObservations; err points to terms owned by the result */
typedef struct ok_vg_conv_result {
    int ctr;
    int nobs; ok_vg_kid* kid; ok_reproj_err** err; uint64_t* lm;
    int nlm; uint64_t* lms; int nconnected; uint64_t* connected;
    int* cauchy;              /* per observation: the term had the Cauchy loss (obs.lossFunction != nullptr) */
} ok_vg_conv_result;

ok_vg* ok_vg_new(void);
void ok_vg_free(ok_vg* g);
uint64_t ok_vg_problem_events(ok_vg* g, const ok_vg_event** ev, int* n);   /* pending events; ok_vg_events_clear() drops them */
void ok_vg_events_clear(ok_vg* g);
/* Enabled by default for replay. Standalone drivers may disable the diagnostic queue. */
void ok_vg_record_events(ok_vg* g, int enabled);
const ok_problem* ok_vg_problem(const ok_vg* g);

/* configuration */
int ok_vg_add_camera(ok_vg* g, int do_extrinsics, double sigma_r, double sigma_alpha);
int ok_vg_add_imu(ok_vg* g, const ok_vg_imu_cfg* c);

/* states */
uint64_t ok_vg_add_states_initialise(ok_vg* g, ok_time t, const ok_imu_meas* meas, size_t n, int ncam, const double (*T_SC)[7]);
uint64_t ok_vg_add_states_propagate(ok_vg* g, ok_time t, const ok_imu_meas* meas, size_t n, int is_keyframe);

/* landmarks */
int ok_vg_add_landmark_id(ok_vg* g, uint64_t id, const double hp[4], int initialised);
uint64_t ok_vg_add_landmark(ok_vg* g, const double hp[4], int initialised);
int ok_vg_remove_landmark(ok_vg* g, uint64_t id);
int ok_vg_set_landmark_initialised(ok_vg* g, uint64_t id, int initialised);
int ok_vg_set_landmark_quality(ok_vg* g, uint64_t id, double quality);
int ok_vg_set_landmark(ok_vg* g, uint64_t id, const double hp[4], int has_init, int initialised);
int ok_vg_set_landmark_classification(ok_vg* g, uint64_t id, int classification);

/* observations */
int ok_vg_add_observation(ok_vg* g, uint64_t lm, ok_vg_kid kid, int use_cauchy, const ok_cam* cam, const double meas[2], double size);
int ok_vg_add_external_observation(ok_vg* g, uint64_t lm, ok_vg_kid kid, int use_cauchy, const ok_reproj_err* src);
int ok_vg_remove_observation(ok_vg* g, ok_vg_kid kid);
int ok_vg_remove_all_observations(ok_vg* g, uint64_t state);

/* covisibilities */
int ok_vg_compute_covisibilities(ok_vg* g);                                   /* returns the C++ bool; *computed_now set when it did work */
int ok_vg_covisibilities_dirty(const ok_vg* g);
int ok_vg_covisibilities(const ok_vg* g, uint64_t i, uint64_t j);
int ok_vg_covis_size(const ok_vg* g);                                         /* outer map size */
/* iterate the cache: n pairs (a, b, count) ascending (a, b) and the visible frames */
int ok_vg_covis_pairs(const ok_vg* g, const uint64_t (**ab)[2], const int** count);
int ok_vg_visible_frames(const ok_vg* g, const uint64_t** ids);

/* relative pose constraints */
int ok_vg_add_relative_pose_constraint(ok_vg* g, uint64_t id0, uint64_t id1, const double T7[7], const double info_cm[36]);
int ok_vg_remove_relative_pose_constraint(ok_vg* g, uint64_t id0, uint64_t id1);

/* state accessors / setters */
int ok_vg_set_keyframe(ok_vg* g, uint64_t id, int flag);
int ok_vg_set_pose(ok_vg* g, uint64_t id, const double T7[7]);
int ok_vg_set_speed_and_bias(ok_vg* g, uint64_t id, const double sb[9]);
int ok_vg_set_extrinsics(ok_vg* g, uint64_t id, int cam, const double T7[7]);
int ok_vg_pose_values(const ok_vg* g, uint64_t id, double out7[7]);
int ok_vg_extrinsics_values(const ok_vg* g, uint64_t id, int cam, double out7[7]);
int ok_vg_sb_values(const ok_vg* g, uint64_t id, double out9[9]);

/* estimator */
void ok_vg_update_landmarks(ok_vg* g);
int ok_vg_clean_unobserved_landmarks(ok_vg* g);
int ok_vg_eliminate_state_by_imu_merge(ok_vg* g, uint64_t id, uint64_t ref, uint64_t* kf, double T_Sk_S[7], double v_Sk[3]);
int ok_vg_merge_landmark(ok_vg* g, uint64_t from, uint64_t into);
/* eliminateStateByImuMerge plus the hash of the post-merge IMU snapshot that patch 0010 logs in the result */
int ok_vg_eliminate_state_by_imu_merge_h(ok_vg* g, uint64_t id, uint64_t ref, uint64_t* kf, double T_Sk_S[7], double v_Sk[3], uint64_t* imu_hash);
/* cleanUnobservedLandmarks(&removed): arrays of the removed landmarks (ascending id), the observation each had (if any); malloc'd */
int ok_vg_clean_unobserved_landmarks_ex(ok_vg* g, uint64_t** lms, ok_vg_kid** kids, int** has_kid, int* nrem);
int ok_vg_freeze_poses_until(ok_vg* g, uint64_t id, int remove_in_ceres);
int ok_vg_unfreeze_poses_from(ok_vg* g, uint64_t id);
int ok_vg_freeze_sb_until(ok_vg* g, uint64_t id, int remove_in_ceres);
int ok_vg_unfreeze_sb_from(ok_vg* g, uint64_t id);
int ok_vg_convert_to_pose_graph_mst(ok_vg* g, const uint64_t* states, int n, const uint64_t* consider, int m, ok_vg_mst_result* out);
void ok_vg_mst_result_free(ok_vg_mst_result* r);
int ok_vg_add_external_two_pose_link(ok_vg* g, uint64_t ref, uint64_t other, const ok_tp_std* term);
int ok_vg_remove_two_pose_const_links(ok_vg* g, uint64_t id);
int ok_vg_remove_two_pose_const_link(ok_vg* g, uint64_t i, uint64_t j);
int ok_vg_convert_to_observations(ok_vg* g, uint64_t id, ok_vg_conv_result* out);
void ok_vg_conv_result_free(ok_vg_conv_result* r);
int ok_vg_remove_speed_and_bias_prior(ok_vg* g, uint64_t id);
/* the clone of the TwoPoseStandardGraphError of (ref, other) as TwoPoseStandardGraphErrorConst; 0 if there is none */
int ok_vg_clone_two_pose_const(const ok_vg* g, uint64_t ref, uint64_t other, ok_tp_std* out);

/* direct accesses of ViSlamBackend */
int ok_vg_poke_fixation_add(ok_vg* g);
int ok_vg_poke_fixation_remove(ok_vg* g);
int ok_vg_poke_landmarks_constant(ok_vg* g, int constant);
int ok_vg_poke_set_observation_information(ok_vg* g, ok_vg_kid kid, const double info[4]);
int ok_vg_poke_copy_state(ok_vg* g, uint64_t id, const double T7[7], const double sb[9]);
int ok_vg_poke_sync_imu(ok_vg* g, const ok_vg* src, uint64_t id);

/* ---- read access for ViSlamBackend (module M6) ---- */
typedef struct ok_vg_state_view { uint64_t id; int is_kf, pose_fixed, sb_fixed, nobs, ntp, ntpc, nrel, has_prev_imu; ok_time ts; } ok_vg_state_view;
typedef struct ok_vg_lm_view { uint64_t id; double hp[4]; int initialised; double quality; int nobs; } ok_vg_lm_view;
int ok_vg_state_count(const ok_vg* g);
int ok_vg_state_at(const ok_vg* g, int idx, ok_vg_state_view* v);         /* states_ in ascending id order */
int ok_vg_state_find(const ok_vg* g, uint64_t id, ok_vg_state_view* v);   /* 0 if absent (v may be NULL) */
int ok_vg_state_index(const ok_vg* g, uint64_t id);                       /* -1 if absent */
int ok_vg_state_obs(const ok_vg* g, uint64_t id, ok_vg_kid** kids, uint64_t** lms);   /* observations in key order; malloc'd */
int ok_vg_landmark_count(const ok_vg* g);
uint64_t ok_vg_landmark_id_at(const ok_vg* g, int i);                      /* landmarks_ in ascending id order */
int ok_vg_landmark_find(const ok_vg* g, uint64_t id, ok_vg_lm_view* v);
int ok_vg_landmark_obs(const ok_vg* g, uint64_t id, ok_vg_kid** kids);    /* in key order; malloc'd */
int ok_vg_obs_find(const ok_vg* g, ok_vg_kid kid, uint64_t* lm, const ok_reproj_err** err, int* cauchy);
int ok_vg_anystate_get(const ok_vg* g, uint64_t id, uint64_t* kf, double T7[7], double v3[3]);
/* anyState_ in ascending id order (ViSlamBackend::writeFinalCsvTrajectory): the reference keyframe (0 if none), time, T_Sk_S, v_Sk */
int ok_vg_anystate_count(const ok_vg* g);
int ok_vg_anystate_at(const ok_vg* g, int i, uint64_t* id, uint64_t* kf, ok_time* ts, double T7[7], double v3[3]);
int ok_vg_imu_use(const ok_vg* g);
int ok_vg_num_cameras(const ok_vg* g);
void ok_vg_set_solver_options(ok_vg* g, int linear_solver_type, double function_tolerance);   /* Solver::Options as the OPT record logs them */
int ok_vg_solver_type(const ok_vg* g);
double ok_vg_function_tolerance(const ok_vg* g);
int ok_vg_pose_fixed(const ok_vg* g, uint64_t id);
/* (state0, state1) of the links stored in a state (kind 0 relative pose, 1 two-pose, 2 two-pose const), in map order; malloc'd */
int ok_vg_state_links(const ok_vg* g, uint64_t id, int kind, uint64_t (**pairs)[2]);
int ok_vg_rel_link_get(const ok_vg* g, uint64_t s0, uint64_t s1, double T7[7], double info_cm[36]);

/* ---- observation of the state for the replay harness ---- */
int ok_vg_num_states(const ok_vg* g);
int ok_vg_num_landmarks(const ok_vg* g);
int ok_vg_num_observations(const ok_vg* g);
/* the blocks in the order patch 0010 lists them in OPT records (per state: pose, speed-and-bias, new extrinsics; then
 * landmarks); landmark_id / quality / classification are filled for landmarks */
typedef struct ok_vg_blkref { ok_vg_blk* b; int is_landmark; uint64_t lm_id; double quality; int classification; } ok_vg_blkref;
int ok_vg_blocks(ok_vg* g, ok_vg_blkref** out);        /* malloc'd array, count returned */
typedef struct ok_vg_state_info { uint64_t id; int is_kf; ok_time ts; int nobs, ntp, ntpc, nrel; } ok_vg_state_info;
int ok_vg_state_infos(const ok_vg* g, ok_vg_state_info** out);
typedef struct ok_vg_imuref { uint64_t state_id; ok_imu_error* e; } ok_vg_imuref;
int ok_vg_imu_links(ok_vg* g, ok_vg_imuref** out);
/* the solver's changes: parameter values written back, IMU term redo (the preintegration is a function of the
 * reference biases alone, so a single redo reproduces the state after however many re-integrations the solve did) */
void ok_vg_blk_set(ok_vg_blk* b, const double* x);
void ok_vg_imu_apply_redo(ok_imu_error* e, int redo, int counter, const double sb_ref[9]);

/* residual blocks (for comparison with the Problem snapshot): the term behind a residual-block handle */
typedef struct ok_vg_resid {
    int type;                 /* ok_solve.h OK_SV_T_* */
    int loss, nb;
    uint64_t blk[OK_PB_MAXB];
    const void* term;
} ok_vg_resid;
int ok_vg_find_resid(const ok_vg* g, uint64_t rb, ok_vg_resid* out);
int ok_vg_is_constant(const ok_vg* g, uint64_t ptr);

/* serialisations of the terms exactly as patch 0008 dumps them (ok_solve.h PROBLEM payloads) */
size_t ok_vg_imu_snapshot(const ok_imu_error* e, int with_meas, int zero_uninit, unsigned char** out); /* malloc'd */
size_t ok_vg_reproj_payload(const ok_reproj_err* e, unsigned char** out);
size_t ok_vg_pose_payload(const ok_pose_err* e, unsigned char** out);
size_t ok_vg_sab_payload(const ok_sab_err* e, unsigned char** out);
size_t ok_vg_relpose_payload(const ok_relpose_err* e, unsigned char** out);
size_t ok_vg_tp_payload(const ok_tp_std* e, unsigned char** out);
uint64_t ok_vg_fnv(const void* p, size_t n);
void ok_vg_imu_copy(ok_imu_error* dst, const ok_imu_error* src);

/* ---- OKVIS2-X GNSS: the graph side (the T_GW block, the per-state GpsFactors); the state machine is ok_vggps.c ---- */
typedef struct ok_gps_fix { ok_time t; double pos[3]; double cov[9]; } ok_gps_fix;   /* GpsMeasurement (cartesian): position, covariance (3x3 col-major) */
/* addGps: enables the GNSS terms of this graph (call before the first addStates; addStatesInitialise then creates the T_GW block even
 * though no fix was seen yet, as upstream) */
int ok_vg_gps_enable(ok_vg* g, const double r_SA[3], double yaw_error_threshold, int robust);
int ok_vg_gps_enabled(const ok_vg* g);
const double* ok_vg_gps_r_SA(const ok_vg* g);
double ok_vg_gps_yaw_error_threshold(const ok_vg* g);
int ok_vg_gps_robust(const ok_vg* g);
const ok_imu_params* ok_vg_imu_params(const ok_vg* g);
/* T_GW = states_.begin()->second.T_GW (the one shared block): coefficients [r, q xyzw] */
void ok_vg_gps_get_T_GW(const ok_vg* g, double T7[7]);
void ok_vg_gps_set_T_GW(ok_vg* g, const double T7[7]);                 /* setGpsExtrinsics: T_GW->setEstimate */
void ok_vg_gps_set_const(ok_vg* g, int constant);                     /* freeze / unfreezeGpsExtrinsics (the Problem call only) */
/* GpsFactors of a state (std::vector, push order) */
int ok_vg_gps_nfactors(const ok_vg* g, uint64_t sid);
ok_gps_async* ok_vg_gps_factor(const ok_vg* g, uint64_t sid, int i);
/* GpsFactors.push_back(new GpsErrorAsynchronous(...)) of ViGraph::addGpsMeasurement; the residual (Cauchy(3), blocks pose / speed-and-bias /
 * T_GW) is added now when add_residual, else later by ok_vg_gps_add_residuals */
void ok_vg_gps_push_factor(ok_vg* g, uint64_t sid, const ok_gps_fix* f, const ok_imu_meas* imu, size_t n, int add_residual);
void ok_vg_gps_add_residuals(ok_vg* g, uint64_t sid);                 /* AddResidualBlock for every factor of the state (addGpsInitFactors body) */
void ok_vg_gps_set_mode(ok_vg* g, uint64_t sid, int mode);
int ok_vg_gps_mode(const ok_vg* g, uint64_t sid);
/* the policy object (ok_vggps.c) hangs off the graph; `removed` is called when a state was eliminated */
void ok_vg_gps_set_policy(ok_vg* g, void* policy, void (*free_fn)(void*), void (*removed)(void*, uint64_t));
void* ok_vg_gps_policy(const ok_vg* g);

#endif
