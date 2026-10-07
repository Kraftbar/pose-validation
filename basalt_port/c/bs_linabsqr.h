/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Basalt port, module M6: LinearizationAbsQR<float, 6> (basalt/linearization/linearization_abs_qr.cpp, landmark_block_abs_dynamic.hpp,
 * imu_block.hpp, BSD-3-Clause, (c) 2019 Usenko, Demmel) plus the estimator-side pieces that evaluate the same problem
 * (BundleAdjustmentBase::computeError / computeDelta / linearizeMargPrior / computeMargPriorError / computeMargPriorModelCostChange,
 * ScBundleAdjustmentBase::computeImuError), C99, float, bit-exact against g++ 13 -O2 -ffp-contract=off -fno-fast-math (SSE2, no FMA,
 * Eigen 3.4.0 MPL-2.0 evaluation orders, TBB at parallelism 1 = serial index order, see PLAN.md 4b).
 *
 * Executed path (EuRoC stereo VIO, vio_linearization_type ABS_QR, Huber, use_householder, use_valid_projections_only; no Jacobian scaling,
 * no pose / landmark damping, no outlier output): LinearizationBase::create (abs_qr), linearizeProblem, performQR, get_dense_H_b,
 * backSubstitute (optimize loop); get_dense_Q2Jp_Q2r (marginalize, used_frames = kfs_to_marg, lost_landmarks); computeError,
 * computeMargPriorError, computeImuError (after each LM step).  Not ported (not on the path): setPoseDamping, setLandmarkDamping,
 * scaleJl_cols / scaleJp_cols, getJp_diag2, Givens QR, marg_scaling, the non-sqrt marginalisation prior.
 *
 * Dense Eigen float kernels (GEBP incl. the blocking heuristic, GEMV) assume the cache sizes of the reference machine (bs_linabsqr_dense.inc).
 * Conventions: float, column-major matrices unless noted, quaternions x y z w (bs_lie.h). */
#ifndef BS_LINABSQR_H
#define BS_LINABSQR_H

#include <stddef.h>
#include <stdint.h>
#include "bs_lie.h"
#include "bs_cam.h"
#include "bs_imu.h"
#include "bs_lmdb.h"

/* ---- estimator state (BundleAdjustmentBase members) */
typedef struct bs_frame_pose {            /* aligned_map<int64_t, PoseStateWithLin<float>> entry (sorted by t_ns) */
    int64_t t_ns;
    int linearized;
    bs_se3f lin;                          /* getPoseLin() */
    bs_se3f cur;                          /* T_w_i_current (used by getPose() when linearized) */
    float delta[6];
} bs_frame_pose;

typedef struct bs_frame_state {           /* aligned_map<int64_t, PoseVelBiasStateWithLin<float>> entry (sorted by t_ns) */
    int64_t t_ns;
    bs_pvb_with_lin s;                    /* linearized flag, state_linearized, state_current */
    float delta[15];
} bs_frame_state;

typedef struct bs_aom_item { int64_t t_ns; int start; int size; } bs_aom_item;
typedef struct bs_aom {                   /* AbsOrderMap: abs_order_map sorted by t_ns */
    const bs_aom_item* item;
    int n;
    int total_size;
} bs_aom;

typedef struct bs_ba {                    /* BundleAdjustmentBase<float> */
    bs_frame_pose* poses; int n_poses;    /* caller-owned arrays, sorted by t_ns */
    bs_frame_state* states; int n_states;
    bs_lmdb lmdb;
    float obs_std_dev, huber_thresh;
    bs_se3f T_i_c[2];                     /* calib.T_i_c (calib.cast<float>()) */
    bs_ds_f cam[2];                       /* calib.intrinsics: double sphere */
} bs_ba;

typedef struct bs_marg_lin {              /* MargLinData<float> (is_sqrt only) */
    bs_aom order;
    int rows, cols;                       /* H is rows x cols, column-major; cols == order.total_size */
    const float* H;
    const float* b;                       /* rows */
} bs_marg_lin;

typedef struct bs_imu_lin {               /* ImuLinData<float>: imu_meas is a std::map<int64_t, ...> iterated in key order */
    float g[3], gyro_bias_weight_sqrt[3], accel_bias_weight_sqrt[3];
    int n;
    bs_imu_meas** meas;                   /* sorted by key */
} bs_imu_lin;

void bs_ba_init(bs_ba* ba);
void bs_ba_destroy(bs_ba* ba);            /* destroys the lmdb only */

/* ---- BundleAdjustmentBase / ScBundleAdjustmentBase evaluation functions (all return what the C++ computes into its out-parameter) */
float bs_ba_compute_error(const bs_ba* ba);                                                   /* computeError(error) without outliers */
void bs_ba_compute_delta(const bs_ba* ba, const bs_aom* order, float* delta);                 /* computeDelta (delta has order->total_size entries) */
float bs_ba_marg_prior_error(const bs_ba* ba, const bs_marg_lin* mld);                         /* computeMargPriorError */
float bs_ba_marg_prior_model_cost_change(const bs_ba* ba, const bs_marg_lin* mld, const float* marg_pose_inc);
/* computeImuError over the whole imu_meas map (gyro/accel_bias_weight = sqrt_weight.array().square()) */
void bs_ba_compute_imu_error(const bs_ba* ba, const bs_aom* aom, bs_imu_meas* const* meas, int n_meas, const float g[3],
                             const float gyro_bias_weight[3], const float accel_bias_weight[3], float* imu_error, float* bg_error, float* ba_error);

/* ---- PoseStateWithLin::applyInc / PoseVelBiasStateWithLin::applyInc (optimize() applies the LM increment with these) */
void bs_pose_apply_inc(bs_frame_pose* p, const float inc[6]);
void bs_state_apply_inc(bs_frame_state* s, const float inc[15]);

/* ---- LinearizationAbsQR<float, 6> */
typedef struct bs_linabsqr bs_linabsqr;

/* ctor: used_frames / lost_landmarks as sets (arrays); pass n_used < 0 / n_lost < 0 for a null pointer.  Keeps pointers to ba, aom, marg, imu. */
bs_linabsqr* bs_la_create(bs_ba* ba, const bs_aom* aom, const bs_marg_lin* marg, const bs_imu_lin* imu,
                          const int64_t* used_frames, int n_used, const int64_t* lost_landmarks, int n_lost);
void bs_la_destroy(bs_linabsqr* la);
float bs_la_linearize_problem(bs_linabsqr* la, int* numerically_valid);
void bs_la_perform_qr(bs_linabsqr* la);
float bs_la_back_substitute(bs_linabsqr* la, const float* pose_inc);                              /* returns l_diff; updates the landmarks */
void bs_la_get_dense_H_b(const bs_linabsqr* la, float* H /* n x n */, float* b);                 /* n = aom->total_size */
int bs_la_dense_Q2_rows(const bs_linabsqr* la);                                                    /* rows of Q2Jp / Q2r */
void bs_la_get_dense_Q2Jp_Q2r(const bs_linabsqr* la, float* Q2Jp /* rows x n */, float* Q2r);
int bs_la_num_landmark_blocks(const bs_linabsqr* la);
int64_t bs_la_landmark_id(const bs_linabsqr* la, int i);                                           /* landmark_ids[i] */

extern const long bs_la_cache_sizes[3];   /* Eigen l1 / l2 / l3 cache sizes assumed by the GEMM blocking model; the oracle checks them against Eigen */
extern int bs_la_unsupported;   /* counts statement forms that take Eigen's tiny-size coefficient product (not modelled) */

/* ---- test hooks (kernel models and linearizePoint) */
void bs_la_t_gemm(int m, int n, int k, const float* a, int ars, int acs, const float* b, int brs, int bcs, float* c, int ldc);
void bs_la_t_gemv(int m, int n, const float* a, int ars, int acs, const float* x, int xinc, float* y);
float bs_la_t_back_substitute(float* st, int num_rows, int num_cols, int padding_idx, int lm_idx, const float* pose_inc, float l_diff_in, float direction[2],
                              float* inv_dist, float inc_out[3], float QJinc_head3[3]);
void bs_la_t_householder_qr(float* st, int num_rows, int num_cols, int padding_idx, int lm_idx);
void bs_stereo_unproject_f(const float proj[2], float res[4], float* d_r_d_p /* 4x2 or NULL */);
/* linearizePoint<float, DoubleSphere>: returns valid; d_res_d_xi 2x6, d_res_d_p 2x3 (either may be NULL, as in computeError) */
int bs_la_linearize_point(const float obs[2], const bs_keypoint* kp, const float T_t_h[16], const bs_ds_f* cam, float res[2],
                          float* d_res_d_xi, float* d_res_d_p);

#endif
