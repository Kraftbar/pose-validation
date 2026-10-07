/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Basalt port, module M8: SqrtKeypointVioEstimator<float>::marginalize() (basalt/src/vi_estimator/sqrt_keypoint_vio.cpp) and
 * MargHelper<float>::marginalizeHelperSqrtToSqrt (basalt/src/vi_estimator/marg_helper.cpp, BSD-3-Clause, (c) 2019 Usenko, Demmel;
 * Eigen 3.4.0 Householder evaluation orders, MPL-2.0), on the state container of bs_vio_opt.h.  C99, float, bit-exact against
 * g++ 13 -O2 -ffp-contract=off -fno-fast-math.
 *
 * Executed path (euroc_config.json): is_lin_sqrt (ABS_QR) && marg_data.is_sqrt -> get_dense_Q2Jp_Q2r + marginalizeHelperSqrtToSqrt (SqToSq / SqToSqrt,
 * the debug nullspace logging and the out_marg_queue copy are not on the path).  What the C++ does to members the C state container does not hold
 * (prev_opt_flow_res: erase(states_to_marg_all), erase(poses_to_marg)) is reported in bs_marg_result for the M9 driver. */
#ifndef BS_MARG_H
#define BS_MARG_H

#include <stdint.h>
#include "bs_vio_opt.h"

typedef struct bs_marg_result {
    int marginalized;                      /* the `if (frame_poses.size() > max_kfs || frame_states.size() >= max_states)` branch ran */
    int64_t last_state_to_marg;
    bs_aom_item* aom; int aom_n, aom_total;                 /* the aom of the marginalisation (poses, then states up to last_state_to_marg) */
    int64_t* kf_ids_all; int n_kf_all;                      /* kf_ids before the selection */
    int64_t* kfs_to_marg; int n_kfs;                        /* sets, ascending */
    int64_t* poses_to_marg; int n_poses_to_marg;
    int64_t* states_to_marg_all; int n_states_all;          /* frame_states, imu_meas and prev_opt_flow_res entries erased */
    int64_t* states_to_marg_vel_bias; int n_states_vb;      /* frame_states and imu_meas erased, turned into frame_poses */
    int* idx_to_keep; int n_keep; int* idx_to_marg; int n_marg;
    int q2_rows;                                            /* rows of Q2Jp / Q2r */
    float* b_new; int n_b_new;                              /* marg_b_new (before `marg_data.b -= marg_data.H * delta`), for the replay checks */
    int layout_error;                                       /* a BASALT_ASSERT of the C++ would fire (prior order vs aom, missing pose / state, no kf chosen, state already linearized) */
} bs_marg_result;

/* marginalize(num_points_connected, lost_landmaks): num_points_connected = std::map<int64_t,int> (sorted array), lost = the unordered_set as an array
 * (iterated only to removeLandmark, order irrelevant for the result).  Does nothing (res->marginalized = 0) unless opt_started and the size condition holds. */
void bs_vio_marginalize(bs_vio* v, const bs_kf_count* num_points_connected, int n_connected, const int64_t* lost, int n_lost, bs_marg_result* res);
void bs_marg_result_free(bs_marg_result* res);

/* MargHelper<float>::marginalizeHelperSqrtToSqrt.  Q2Jp (rows x cols column-major) and Q2r are modified (as in the C++).  keep / marg = the sorted index sets.
 * Output: *H_out = (keep_valid_rows x keep) column-major, *b_out (keep_valid_rows); both malloc'ed. */
void bs_marg_helper_sqrt_to_sqrt(float* Q2Jp, int rows, int cols, float* Q2r, const int* keep, int nkeep, const int* marg, int nmarg,
                                 float** H_out, int* H_rows, float** b_out);

extern long bs_marg_stats[4];   /* marginalizations, kfs chosen by the feature-ratio criterion, kfs chosen by the DSO score, rank-deficient columns */
extern void (*bs_marg_dbg_hook)(const float* Q2Jp, int rows, int cols, const float* Q2r, const int* keep, int nkeep, const int* marg, int nmarg);   /* test hook */
extern int bs_marg_oob;   /* out-of-bounds reads of the C++ helper (UB) seen so far */

#endif
