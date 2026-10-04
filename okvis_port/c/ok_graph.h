/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 5b: ViGraph numerics outside the solver (okvis_ceres ViGraph::updateLandmarks) and the
 * readers of the module-5 reference dumps (patch 0009) shared by the harnesses.
 *
 * Derived from OKVIS2 (okvis_ceres/src/ViGraph.cpp):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The Eigen 3.4.0 evaluation-order models
 *   (3x3 products, fixed-size vector norms, SelfAdjointEigenSolver<Matrix3d>) are MPL-2.0 (Eigen, Copyright (C)
 *   Gael Guennebaud, Benoit Jacob and the Eigen authors). Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only.
 *
 * ---- graph.bin (patch 0009, OKVIS_PORT_GRAPH_DUMP_DIR): framed records u32 tag, u64 len, payload (native endian) ----
 *   "reproj" below = u64 plen, then the ReprojectionError payload of patch 0008 (cam header of ok_cam.h: u32 tag, u32 w,
 *   u32 h, f64 fu fv cu cv, u32 nd, f64 d[nd]; f64 meas[2]; f64 information[4] row-major); "T7" = f64 r[3], q[4] (xyzw).
 *   G_TP_COMPUTE (1): TwoPoseStandardGraphError::compute() (recorded when it actually computes):
 *                u64 term_ptr, u32 kind (7), u64 refId, u64 otherId, u32 numCams, u32 stayConst, u32 isComputed_before,
 *                2 x {u32 present, u64 block_id, f64 snapshot[7], f64 live[7]}      (poseParameterBlockInfos_),
 *                u32 nextr, nextr x {u32 present, u64 block_id, u32 idx, f64 snapshot[7]}   (extrinsics infos),
 *                u32 nlm, nlm x {u64 id, u32 vec_idx, u32 idx, f64 snapshot[4]}     (landmark infos, ids ascending),
 *                u32 ngroups, ngroups x {u64 landmark_id, u32 nobs, nobs x {u64 frameId, u32 cam, u32 kp,
 *                  u32 loss (0 none / 1 CauchyLoss / 2 other), u32 isMarginalised, u32 isDuplication, u64 pose_id,
 *                  u64 hpoint_id, u32 hpoint_initialised, f64 hpoint[4], u64 extr_id, reproj}}   (observations_),
 *                -- outputs: u32 ret, f64 H00_[36] (column-major), f64 b0_[6], f64 J_[36] (column-major), f64 DeltaX_[6],
 *                T7 linearisationPoint_T_S0S1_, u32 relPoseSet, u32 nlm_S0, nlm_S0 x {u64 id, f64 hp_S0[4]} (landmarks_),
 *                per observation (same order) u32 isMarginalised_after
 *   G_TP_CONVERT (2): convertToReprojectionErrors(): u64 term_ptr, u32 kind, T7 T_WS0 (live reference pose), u32 nlm,
 *                nlm x {u64 id, f64 hp_S0[4], f64 hp_W[4]}, u32 nobs_out, u32 ndup
 *   G_TP_EVAL (3): sampled EvaluateWithMinimalJacobians of TwoPose{Standard,Extrinsics}GraphError{,Const}:
 *                u64 term_ptr, u32 kind (7-10), u64 plen, payload (ok_solve.h, PROBLEM types 7-10), u32 nb,
 *                nb x f64 params[7], u32 have_jac, nb x u32 jac_nonnull, u32 have_jacmin, nb x u32 jacmin_nonnull,
 *                u32 ret, u32 nres, f64 residuals[nres], per block with jac_nonnull: f64 J[nres*7] (row-major),
 *                per block with jac_nonnull && jacmin_nonnull: f64 Jmin[nres*6] (row-major)
 *   G_LM_UPDATE (4): sampled ViGraph::updateLandmarks(): u64 call, u32 nlm_total, u32 sub, u32 nrec,
 *                nrec x {u64 id, f64 hp[4], u32 nobs, nobs x {u64 frameId, u32 cam, u32 kp, f64 pose[7], f64 extr[7],
 *                reproj}, f64 quality_after, u32 initialised_after, f64 hp_after[4]}
 */
#ifndef OK_GRAPH_H
#define OK_GRAPH_H
#include <stddef.h>
#include <stdint.h>
#include "ok_twopose.h"

enum { OK_G_TP_COMPUTE = 1, OK_G_TP_CONVERT = 2, OK_G_TP_EVAL = 3, OK_G_LM_UPDATE = 4 };

/* Vector4d::norm() (fixed size 4: the vectorised redux (p0+p2)+(p1+p3) under the square root) */
double ok_v4_norm(const double v[4]);

/* One observation of a landmark as ViGraph::updateLandmarks reads it: the state's pose and extrinsics blocks and the
 * reprojection error of the keypoint. */
typedef struct ok_lm_obs { uint64_t frame_id; int cam, kp; double pose[7]; double extr[7]; ok_reproj_err err; } ok_lm_obs;
/* The per-landmark body of ViGraph::updateLandmarks: hp (the HomogeneousPointParameterBlock parameters) is updated in
 * place when the landmark is behind a camera and gets reset along the best ray; quality (max(0, .)) and the
 * initialisation flag are returned. Observations in keypoint-id order (std::map). */
void ok_graph_update_landmark(double hp[4], const ok_lm_obs* obs, int nobs, double* quality, int* initialised);

/* ---- dump readers (also used by check_ok_solve.c) ----
 * Return the number of bytes consumed, or -1 on a malformed record. */
int ok_reproj_payload_read(const unsigned char* p, size_t len, ok_reproj_err* e);  /* cam header, meas, information */
int ok_tp_payload_read(const unsigned char* p, size_t len, int kind, ok_tp_std* s, ok_tp_ext* x); /* PROBLEM types 7-10 */

#endif
