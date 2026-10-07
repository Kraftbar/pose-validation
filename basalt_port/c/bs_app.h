/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Basalt port, module M9: the application layer of SqrtKeypointVioEstimator<float> (basalt/vi_estimator/sqrt_keypoint_vio.cpp, BSD-3-Clause,
 * (c) 2019 Usenko, Demmel): initialize() (the proc_func loop: IMU queue pops, first-state set-up, IMU pre-integration between frames),
 * measure() (predictState, new state, landmark bookkeeping, key frame decision, triangulation, lost landmarks, optimize + marginalize,
 * output state), the estimator constructor (calibration cast to float, marginalisation prior) and the config / calibration JSON readers.
 * Threadless: the C driver feeds one optical-flow result per frame (FIFO order is the only semantics of the C++ queues).  C99, float.
 *
 * Executed path: stereo EuRoC, ABS_QR, vio_use_lm, vio_sqrt_marg, vio_marg_lost_landmarks, no jacobian scaling, no damping variants
 * (bs_app_cfg_load refuses any other setting). */
#ifndef BS_APP_H
#define BS_APP_H

#include <stddef.h>
#include <stdint.h>
#include "bs_flow.h"
#include "bs_marg.h"

typedef struct bs_app_cfg {
    /* VioConfig (the keys the executed path reads) */
    int max_states, max_kfs, min_frames_after_kf, max_iterations, marg_lost_landmarks;
    double new_kf_keypoints_thresh, obs_std_dev, obs_huber_thresh, min_triangulation_dist, kf_marg_feature_ratio;
    double lm_lambda_initial, lm_lambda_min, lm_lambda_max;
    double init_pose_weight, init_ba_weight, init_bg_weight;
    /* Calibration<double> */
    double intr[2][6];                    /* fx fy cx cy xi alpha */
    double T_i_c[2][7];                   /* px py pz qx qy qz qw, the quaternion exactly as stored in the json (not normalised) */
    double accel_bias_full[9], gyro_bias_full[12];
    double accel_noise_std[3], gyro_noise_std[3], accel_bias_std[3], gyro_bias_std[3];
    double imu_update_rate;
} bs_app_cfg;

/* euroc_config.json defaults + euroc_ds_calib.json structure; returns 0 on success, else a message in err (>= 160 bytes) */
int bs_app_cfg_load(bs_app_cfg* c, const char* config_path, const char* calib_path, char* err, size_t err_n);

typedef struct bs_imu_raw { int64_t t_ns; double accel[3], gyro[3]; } bs_imu_raw;   /* ImuData<double> */

typedef struct bs_app_out {               /* PoseVelBiasState<double> as the driver stores it (t_ns, T_w_i) */
    int64_t t_ns;
    double t[3];
    double q[4];                          /* x y z w */
} bs_app_out;

typedef struct bs_frame_obs { int64_t t_ns; bs_flow_obs obs[2]; } bs_frame_obs;   /* prev_opt_flow_res entry (owned copy) */

struct bs_app;
/* per-frame observer (optional), called at the end of measure() with the decisions of that frame (for the replay checks) */
typedef struct bs_app_frame_info {
    int64_t t_ns;
    int meas_used;                        /* an IMU measurement was integrated for this frame (not the first frame) */
    int connected0, unconnected0;         /* i == 0 counts */
    int take_kf, landmarks_added;         /* key frame taken (before the reset), landmarks created */
    int n_lost;
    const int64_t* unconnected_ids; int n_unconnected;   /* iteration order of unconnected_obs0 */
    const int64_t* lost_ids;
    const bs_opt_info* opt;
    const bs_marg_result* marg;
} bs_app_frame_info;
typedef void (*bs_app_frame_cb)(void* ctx, const bs_app_frame_info* fi, const struct bs_app* app);

typedef struct bs_app {
    bs_app_cfg cfg;
    bs_vio v;
    float accel_cov[3], gyro_cov[3];
    float calib_accel_bias[3], calib_accel_scale[9], calib_gyro_bias[3], calib_gyro_scale[9];   /* CalibAccelBias / CalibGyroBias<float> */
    const bs_imu_raw* imu; size_t n_imu, imu_pos;   /* imu_data_queue */
    int have_data; bs_imudata data;       /* the proc_func `data` pointer (calibrated float sample), have_data == 0: nullptr */
    int initialized;
    int have_prev; int64_t prev_t_ns;     /* prev_frame */
    bs_frame_obs* prev; int n_prev, cap_prev;   /* prev_opt_flow_res, sorted by t */
    bs_app_frame_cb cb; void* cb_ctx;
    long stat_frames, stat_landmarks_added, stat_kf, stat_triangulate;
    int fatal;                            /* a condition under which the C++ aborts was hit (message in fatal_msg) */
    char fatal_msg[160];
} bs_app;

/* estimator constructor + initialize(bg, ba) with zero biases: returns 0 on success */
int bs_app_init(bs_app* a, const bs_app_cfg* cfg);
void bs_app_destroy(bs_app* a);
/* the IMU queue content (the driver pushes every sample, then nullptr); the first sample is popped here, as proc_func does before its loop */
int bs_app_set_imu(bs_app* a, const bs_imu_raw* imu, size_t n);
/* one iteration of the proc_func while loop for a flow result (obs[2] = OpticalFlowResult::observations).
 * Returns 0: processed, *out valid; 1: the loop ended (IMU data exhausted: `if (!data.get()) break;`), no output; -1: fatal (a.fatal_msg). */
int bs_app_frame(bs_app* a, int64_t t_ns, const bs_flow_obs obs[2], bs_app_out* out);

#endif
