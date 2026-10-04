/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 5a: the landmark-eliminated relative-pose terms of the pose graph
 * (okvis_ceres: TwoPoseGraphError, TwoPoseStandardGraphError, TwoPoseStandardGraphErrorConst,
 * TwoPoseExtrinsicsGraphError, TwoPoseExtrinsicsGraphErrorConst) and okvis::PseudoInverse.
 *
 * Derived from OKVIS2 (okvis_ceres/include/okvis/ceres/{TwoPoseGraphError,TwoPoseExtrinsicsGraphError}.hpp,
 * src/<same names>.cpp, include/okvis/PseudoInverse.hpp):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The Cauchy robustification inside compute()
 *   follows Ceres Solver's corrector.cc (Copyright 2023 Google Inc., BSD-3-Clause). The evaluation-order models of
 *   the Eigen 3.4.0 expressions (small lazy products, SelfAdjointEigenSolver, GEMV) are MPL-2.0 (Eigen, Copyright
 *   (C) Gael Guennebaud, Benoit Jacob and the Eigen authors). Redistribution requires retaining these notices; the
 *   names of ETH Zurich, Imperial College London, TUM and Google may not be used to endorse derived products.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only. Matrices stored in the structs are COLUMN-major; Jacobian
 * output buffers are ROW-major like the Eigen::RowMajor maps of the C++ code. `jac` / `jacmin` mimic
 * `double** jacobians` / `double** jacobiansMinimal` (NULL = nullptr, entries may be NULL): a minimal Jacobian is
 * written only when the corresponding full one is requested, exactly like the C++ code (which dereferences
 * jacobians[] whenever either array is non-null, so jacobians == NULL && jacobiansMinimal != NULL never occurs).
 *
 * What the pipeline uses (EuRoC mono/stereo, no online extrinsics calibration): TwoPoseStandardGraphError in the
 * realtime graph (created by ViGraphEstimator::convertToPoseGraphMst: addObservation + compute, back-converted by
 * convertToReprojectionErrors) and its cloneTwoPoseGraphErrorConst() = TwoPoseStandardGraphErrorConst in the full
 * graph. The *Extrinsics* variants (online extrinsics calibration) are ported for their Evaluate only and verified
 * by the random test (okvis_twopose_test.cc); their compute() is not ported.
 */
#ifndef OK_TWOPOSE_H
#define OK_TWOPOSE_H
#include <stdint.h>
#include "ok_err.h"
#include "ok_kin.h"

/* ---- okvis::PseudoInverse (SelfAdjointEigenSolver based), n <= OK_EIG_MAX, column-major ----
 * tolerance = max(epsilon, epsilon * n * max eigenvalue); rank = #eigenvalues > tolerance (may be NULL). */
void ok_pinv_symm(int n, const double* a, double* result, double epsilon, int* rank);       /* V diag(sel) V^T */
void ok_pinv_symm_sqrt(int n, const double* a, double* result, double epsilon, int* rank);  /* V diag(sqrt(sel)) */
void ok_pinv_symm_sqrt_u(int n, const double* a, double* result, double epsilon, int* rank);/* diag(sqrt(sel)) V^T */

/* ---- TwoPoseStandardGraphError / TwoPoseStandardGraphErrorConst: the linearised relative-pose term ---- */
typedef struct ok_tp_std {
    int is_computed;        /* TwoPoseStandardGraphError::isComputed_ (the Const term behaves as computed) */
    double DeltaX[6];       /* DeltaX_ */
    double J[36];           /* J_ (6x6, column-major) */
    ok_tf lin_T_S0S1;       /* linearisationPoint_T_S0S1_ (cached Transformation) */
} ok_tp_std;
/* EvaluateWithMinimalJacobians (Evaluate == jacmin NULL). params: T_WS0[7], T_WS1[7]; residuals[6]; jac[2] row-major
 * 6x7; jacmin[2] row-major 6x6. Returns 0 (false) when !is_computed (nothing written), else 1. */
int ok_tp_std_evaluate(const ok_tp_std* e, const double* const params[2], double res[6], double* const* jac,
                       double* const* jacmin);

/* ---- TwoPoseExtrinsicsGraphError(Const): residual dimension n = 6 + 6 * nextr ---- */
#define OK_TP_MAXEXTR 4
typedef struct ok_tp_ext {
    int is_computed;
    int n;                          /* 6 + 6 * nextr */
    int nextr;                      /* linearisationPoints_T_SC_.size() == extrinsicsParameterBlockInfos_.size() */
    double DeltaX[6 + 6 * OK_TP_MAXEXTR];
    double J[(6 + 6 * OK_TP_MAXEXTR) * (6 + 6 * OK_TP_MAXEXTR)];  /* n x n column-major, leading dimension n */
    ok_tf lin_T_S0S1;
    int extr_present[OK_TP_MAXEXTR];
    ok_tf lin_T_SC[OK_TP_MAXEXTR];  /* linearisationPoints_T_SC_[i] when present */
} ok_tp_ext;
/* params: T_WS0[7], T_WS1[7], T_SC_i[7] (nextr blocks); residuals[n]; jac[2+nextr] row-major n x 7; jacmin n x 6 */
int ok_tp_ext_evaluate(const ok_tp_ext* e, const double* const* params, double* res, double* const* jac,
                       double* const* jacmin);

/* ---- TwoPoseGraphError bookkeeping + TwoPoseStandardGraphError::compute() / convertToReprojectionErrors() ----
 * The observations are stored per landmark (std::map<uint64_t, std::vector<Observation>>: landmark ids ascending,
 * insertion order inside); the parameter-block "infos" hold the parameter values copied when the block was first
 * seen by addObservation (the C++ ParameterBlockInfo), the live pose values are what compute() reads through
 * parameterBlock->estimate(). */
typedef struct ok_tp_obs {
    uint64_t frame_id; int cam, kp;
    int loss;                       /* 0 none, 1 CauchyLoss(1.0) */
    int is_marginalised, is_duplication;
    uint64_t pose_id, hpoint_id, extr_id;
    int hpoint_initialised;
    double hpoint[4];               /* the observation's hPoint parameters (bookkeeping only) */
    ok_reproj_err err;              /* the (cloned) reprojection error */
} ok_tp_obs;
typedef struct ok_tp_group { uint64_t lm_id; int nobs, cap; ok_tp_obs* obs; } ok_tp_group;
typedef struct ok_tp_lm_S0 { uint64_t id; double hp_S0[4]; } ok_tp_lm_S0;
typedef struct ok_twopose {
    uint64_t ref_id, other_id;
    int num_cams, stay_const;
    /* ParameterBlockInfo<7> poseParameterBlockInfos_[2]: present, id, parameters (snapshot), and the live block */
    int pose_present[2]; uint64_t pose_id[2]; double pose_snapshot[2][7]; double pose_live[2][7];
    /* extrinsicsParameterBlockInfos_ (numCams or 2*numCams entries): present, id, idx, parameters (snapshot) */
    int nextr; int extr_present[2 * OK_TP_MAXEXTR]; uint64_t extr_id[2 * OK_TP_MAXEXTR]; int extr_idx[2 * OK_TP_MAXEXTR];
    double extr_snapshot[2 * OK_TP_MAXEXTR][7];
    /* landmarkParameterBlockInfos_ + landmarkParameterBlockId2idx_ (ids ascending): id, vector index, idx, parameters */
    int nlm, cap_lm; uint64_t* lm_id; int* lm_vec_idx; int* lm_idx; double (*lm_snapshot)[4];
    int sparse_size;
    /* observations_ */
    int ngroups, cap_groups; ok_tp_group* groups;
    /* results of compute() */
    ok_tp_std term;                 /* isComputed_, DeltaX_, J_, linearisationPoint_T_S0S1_ */
    double H00[36], b0[6];          /* H00_, b0_ */
    int rel_pose_set;
    int nlm_S0, cap_lm_S0; ok_tp_lm_S0* lm_S0;   /* landmarks_: landmark coordinates in S0 (ids ascending) */
} ok_twopose;

void ok_twopose_init(ok_twopose* t, uint64_t ref_id, uint64_t other_id, int num_cams, int stay_const);
void ok_twopose_free(ok_twopose* t);
/* TwoPoseGraphError::addObservation (weight == 1.0 as the pipeline calls it): clones the reprojection error (and
 * re-sets its information when is_duplication), registers the pose / landmark / extrinsics infos on first sight.
 * pose / hpoint / extr: the parameter blocks (values as they are now; the snapshots are taken here). Returns 1. */
int ok_twopose_add_observation(ok_twopose* t, uint64_t frame_id, int cam, int kp, const ok_reproj_err* err, int loss,
                               uint64_t pose_id, const double pose[7], uint64_t hpoint_id, const double hpoint[4],
                               int hpoint_initialised, uint64_t extr_id, const double extr[7], int is_duplication);
/* TwoPoseStandardGraphError::compute(); pose_live[0] must hold the reference pose block's current parameters.
 * Returns 1 (also when already computed: nothing is done then). */
int ok_twopose_compute(ok_twopose* t);
/* TwoPoseStandardGraphError::convertToReprojectionErrors: the marginalised observations get their landmark mapped back
 * to world coordinates with the live reference pose T_WS0 (coeffs[7]); out_hp_W[i] receives the new homogeneous
 * point of observation i (in observations_ order, marginalised ones only, the others are skipped: `out_count`),
 * out_dup counts the duplications. Then clears the bookkeeping like the C++ code. Returns the number of
 * observations returned. */
int ok_twopose_convert(ok_twopose* t, const double T_WS0_live[7], double (*out_hp_W)[4], int max_out, int* out_dup);

#endif
