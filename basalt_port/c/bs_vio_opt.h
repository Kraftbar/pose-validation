/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Basalt port, module M7: SqrtKeypointVioEstimator<float>::optimize (basalt/src/vi_estimator/sqrt_keypoint_vio.cpp, BSD-3-Clause,
 * (c) 2019 Usenko, Demmel) with its Levenberg-Marquardt loop, the dense Eigen 3.4.0 float LDLT solve of H + diag(max(lambda diag, min_lambda))
 * (MPL-2.0 evaluation order model: Eigen/src/Cholesky/LDLT.h unblocked, Core/products/TriangularSolverVector.h), and the estimator state
 * container (the members of SqrtKeypointVioEstimator / BundleAdjustmentBase that optimize() and marginalize() read and write; M8 adds
 * bs_marg.h, M9 will add the per-frame driver).  C99, float, bit-exact against g++ 13 -O2 -ffp-contract=off -fno-fast-math (SSE2).
 *
 * Executed path (euroc_config.json): vio_use_lm, ABS_QR, no Jacobian scaling, no damping rotation (vio_lm_{pose,landmark}_damping_variant 1), 3 solve
 * retries (never taken on MH_01).  Conventions as bs_linabsqr.h: float, column-major, quaternions x y z w, maps (std::map) are arrays sorted by key. */
#ifndef BS_VIO_OPT_H
#define BS_VIO_OPT_H

#include <stddef.h>
#include <stdint.h>
#include "bs_linabsqr.h"

/* MargLinData<float> (is_sqrt = true), owning */
typedef struct bs_marg_data {
    bs_aom_item* item; int n; int total;   /* order.abs_order_map sorted by t_ns, order.total_size */
    int rows, cols;                        /* H is rows x cols column-major, cols == total */
    float* H;
    float* b;                              /* rows entries */
} bs_marg_data;

typedef struct bs_kf_count { int64_t t_ns; int n; } bs_kf_count;   /* std::map<int64_t, int> entry */

/* SqrtKeypointVioEstimator<float> state used by optimize() / marginalize() (and by the M9 driver) */
typedef struct bs_vio {
    bs_ba ba;                              /* poses / states arrays (owned, sorted by t_ns), lmdb (owned), calibration, obs_std_dev, huber_thresh */
    int cap_poses, cap_states;
    bs_imu_meas* imu; int n_imu, cap_imu;  /* imu_meas map: key == start_t_ns, sorted */
    bs_marg_data marg;
    float g[3], gyro_bias_sqrt_weight[3], accel_bias_sqrt_weight[3];
    int64_t* kf_ids; int n_kf, cap_kf;     /* std::set<int64_t> kf_ids, sorted */
    bs_kf_count* num_points_kf; int n_npk, cap_npk;   /* std::map<int64_t, int> num_points_kf, sorted */
    int64_t last_state_t_ns;
    int take_kf, frames_after_kf;
    int max_states, max_kfs;
    double kf_marg_feature_ratio;          /* config.vio_kf_marg_feature_ratio (compared in double) */
    int opt_started;
    /* LM */
    float lambda, min_lambda, max_lambda, lambda_vee;
    double lm_lambda_initial;
    int max_iterations;                    /* config.vio_max_iterations */
} bs_vio;

void bs_vio_init(bs_vio* v);               /* zero state; defaults of euroc_config.json (max_states 3, max_kfs 7, lambda 1e-4 / 1e-6 / 1e2, 7 iterations, ratio 0.1, lambda_vee 2) */
void bs_vio_destroy(bs_vio* v);

/* sorted-map helpers (insert keeps the order; the caller fills the returned slot).  Return NULL on a duplicate key. */
bs_frame_pose* bs_vio_pose_insert(bs_vio* v, int64_t t_ns);
bs_frame_state* bs_vio_state_insert(bs_vio* v, int64_t t_ns);
bs_imu_meas* bs_vio_imu_insert(bs_vio* v, int64_t start_t_ns);
void bs_vio_kf_insert(bs_vio* v, int64_t t_ns);
void bs_vio_npk_set(bs_vio* v, int64_t t_ns, int n);
int bs_vio_npk_get(const bs_vio* v, int64_t t_ns, int* n);                 /* returns 0 if absent */
void bs_marg_data_set(bs_marg_data* m, const bs_aom_item* items, int n, int total, int rows, int cols, const float* H, const float* b);   /* deep copy */

/* The estimator constructor's prior (marg_data.H / b, sqrt form) and initialize()'s order {t_ns: (0, 15)}: H = diag(sqrt(pose_w) x3, 0, 0, sqrt(pose_w), 0 x3, sqrt(ba_w) x3, sqrt(bg_w) x3),
 * b = 0 (15 x 15); the weights are config.vio_init_pose_weight / vio_init_ba_weight / vio_init_bg_weight (1e8, 10, 100 in euroc_config.json). */
void bs_vio_init_marg_prior(bs_vio* v, int64_t t_ns, double init_pose_weight, double init_ba_weight, double init_bg_weight);

/* ---- dense LDLT (Eigen::LDLT<Eigen::Ref<MatX>>, in place, lower, unblocked) ---- */
/* ldlt(H_copy): factorise the n x n column-major matrix in place (lower triangle + diagonal read and written), trans[k] = transpositions */
void bs_ldlt_factor(float* A, int n, int* trans);
/* x = ldlt.solve(b) for the factor above (x may alias b) */
void bs_ldlt_solve(const float* A, int n, const int* trans, const float* b, float* x);

/* ---- optimize() ---- */
typedef struct bs_opt_info {
    int ran;                               /* optimize() body executed (opt_started || states > 4) */
    int it, it_rejected, converged, terminated;
    int steps;                             /* inner LM evaluations (accepted + rejected) */
    int retries;                           /* solve retries (non-finite increments) */
    int invalid_linearization;             /* linearizeProblem reported numerically_valid == false (the C++ asserts) */
    int layout_error;                      /* marg order does not match the aom (the C++ asserts) */
    int nonfinite_increment;               /* increment still non-finite after the 3 retries: the C++ then aborts in SO3::exp (SOPHUS_ENSURE); the C returns */
} bs_opt_info;

/* observer of the LM loop (optional).  phase 0: after the solve (H, b, inc = solution before the negation known), before backup/applyInc;
 * phase 1: after applyInc and the cost evaluation, before the accept / reject decision (state still holds the step). */
typedef struct bs_opt_step {
    int phase;
    int it, j;
    float lambda_before;
    float error_total;
    int n; const float* H; const float* b; const float* inc;   /* inc = the applied (negated) increment (phase 1) */
    float l_diff, f_diff, relative_decrease, step_norminf, after_vi, after_marg;
    int accepted, step_valid;
    float lambda_after;                    /* phase 1: lambda after the decision (accept / reject update applied) */
    const bs_vio* vio;
} bs_opt_step;
typedef void (*bs_opt_cb)(void* ctx, const bs_opt_step* s);

/* SqrtKeypointVioEstimator<float>::optimize() */
void bs_vio_optimize(bs_vio* v, bs_opt_info* info, bs_opt_cb cb, void* ctx);

/* the aom of optimize(): poses first (kf poses, ascending), then states.  Caller frees *items. */
void bs_vio_build_aom(const bs_vio* v, bs_aom_item** items, bs_aom* aom);

/* PoseStateWithLin(const PoseVelBiasStateWithLin&) (marginalize: a vel/bias-marginalised key frame state becomes a pose) */
void bs_pose_from_state(const bs_frame_state* s, bs_frame_pose* p);

extern long bs_ldlt_stats[4];              /* factorisations, zero-diagonal early exits, nonfinite solves, transpositions != identity */

#endif
