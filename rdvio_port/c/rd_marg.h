/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M5: the marginalisation prior (rdvio_estimation/marginalization_factor.h, ceres/marginalization_factor.h:
 * CeresMarginalizationFactor): Evaluate (residual + Jacobians against the kept linearisation points) and marginalize(index) (information
 * matrix of the window from the previous prior, the IMU factors and the visual factors of the victim frame, Schur complement of the
 * landmarks and of the victim frame, eigendecomposition with the 1e-8 cut, square-root information and information vector).
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified. Eigen 3.4.0
 * evaluation-order models (dynamic GEMM / GEMV, SelfAdjointEigenSolver<MatrixXd>): MPL-2.0 (Copyright (C) Gael Guennebaud, Benoit Jacob and
 * the Eigen authors). See rdvio_port/NOTICE. C99, <stdint.h> <math.h> <stdlib.h> <string.h>.
 *
 * Column-major matrices; Jacobians handed to / returned by rd_marg_eval are ROW-major (Ceres), N x 4 (quaternion) and N x 3.
 *
 * ---- dump record (rdvio_port/reference/patches/0007-m5-marginalization-dump.patch, channel `marg` -> marg.bin), native endian ----
 *   u32 nf (map frames), u32 index, u32 flags (1: stage matrices follow), u32 nff (frames of the factor before the call)
 *   nff x { i32 map_pos, lin pose q[4] p[3], lin motion v[3] bg[3] ba[3] }, sqrt_inv_cov[N*N], infovec[N]      (N = 15 nff)
 *   nf x { u64 id, q[4] p[3] v[3] bg[3] ba[3], imu q[4] p[3], u32 has_pre, [pre: t q[4] p[3] v[3] dq_dbg dp_dbg dp_dba dv_dbg dv_dba sqrt_inv_cov[225]] }
 *   u32 ntracks, ntracks x { u64 id, u32 ref_pos, f64 inv_depth, u32 nobs, nobs x { i32 tgt_pos, z[3] z_ref[3] cam_ref{q[4] p[3]} cam_tgt{q[4] p[3]}
 *                            sqrt_inv_cov[4] } }
 *   outputs: u32 nff2, nff2 x { lin pose, lin motion }, sqrt_inv_cov[N2*N2], infovec[N2];
 *            flags & 1: pose_motion_infomat[N2*N2] (after the Schur complements), pose_motion_infovec[N2], eigenvalues[N2], eigenvectors[N2*N2]
 *   eval.bin (oracle only): u32 nff, then the factor payload (u32 nff, nff x {lin pose, lin motion}, sqrt_inv_cov, infovec), params (nff x 16: q p v bg ba),
 *            u32 have_jac, u64 mask (bit 5i+k = Jacobian block set), residual[N], for each set bit k: N x size(k) row-major (size 4 for k % 5 == 0, else 3)
 */
#ifndef RD_MARG_H
#define RD_MARG_H
#include <stdint.h>
#include "rd_imu.h"
#include "rd_factor.h"

#define RD_ES_SIZE 15
#define RD_ES_Q 0
#define RD_ES_P 3
#define RD_ES_V 6
#define RD_ES_BG 9
#define RD_ES_BA 12

typedef struct rd_marg {
    int nf;
    uint64_t* ids;          /* identity of every frame of the factor (the real code keeps Frame pointers) */
    rd_pose* lin_pose;
    rd_motion* lin_motion;
    double* sqrt_inv_cov;   /* N x N column-major, N = 15 nf */
    double* infovec;        /* N */
    int eig_info;           /* ComputationInfo of the last eigendecomposition (0 Success, 1 NoConvergence: Eigen still returns the unsorted result) */
} rd_marg;

/* a window frame as the marginalisation sees it (Map::get_frame(i)) */
typedef struct rd_marg_frame {
    uint64_t id;
    rd_pose pose;
    rd_motion motion;
    rd_extrinsic imu;
    const rd_preint* kpre;  /* keyframe_preintegration (needed for the frames j = index, index + 1, j > 0) */
} rd_marg_frame;

/* one observation of a victim-frame track in a frame other than the reference frame */
typedef struct rd_marg_obs {
    int tgt;                /* map position of the observing frame; < 0 = not in the map (skipped) */
    double z[3], z_ref[3], sqrt_inv_cov[4];
    rd_extrinsic cam_ref, cam_tgt;
} rd_marg_obs;
/* a track of the victim frame that is valid with a keyframe as first frame; observations in keypoint_map() order, reference excluded */
typedef struct rd_marg_track {
    uint64_t id;
    int ref;                /* map position of the first frame */
    double inv_depth;
    int nobs;
    const rd_marg_obs* obs;
} rd_marg_track;

/* optional stage outputs of marginalize (malloc'd, freed by rd_marg_dbg_free) */
typedef struct rd_marg_dbg { int n; double *infomat, *infovec, *evals, *evecs; } rd_marg_dbg;
void rd_marg_dbg_free(rd_marg_dbg* d);

void rd_marg_free(rd_marg* m);
/* MarginalizationFactor::MarginalizationFactor(Map*): the first nf_map - 1 frames, zero prior except 1e15 on q and p of the first frame */
void rd_marg_init(rd_marg* m, int nf_map, const rd_marg_frame* frames);
/* CeresMarginalizationFactor::Evaluate. params[5 i + 0..4] = q p v bg ba of frame i of the factor; jacobians[5 i + k] may be NULL. Returns 1. */
int rd_marg_eval(const rd_marg* m, const double* const* params, double* residuals, double* const* jacobians);
/* CeresMarginalizationFactor::marginalize(index). frames = the nf_map frames of the map; tracks as above. Returns 0 on success, nonzero if a frame of the
 * factor is not in the map or the eigensolver fails. dbg may be NULL. */
int rd_marg_marginalize(rd_marg* m, int nf_map, const rd_marg_frame* frames, int index, int ntracks, const rd_marg_track* tracks, rd_marg_dbg* dbg);
#endif
