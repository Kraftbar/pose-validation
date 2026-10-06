/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M11: the configuration (rdvio::Config defaults, rdvio_extra/src/yaml_config.cpp) read from the
 * device (sensor) yaml and the slam (setting) yaml. Values are taken verbatim (quaternions are not normalised); numbers go
 * through strtod / strtoull like yaml-cpp's stream conversion. Derived from RD-VIO (Apache-2.0). C99.
 */
#ifndef RD_SYS_CONFIG_H
#define RD_SYS_CONFIG_H
#include <stddef.h>
#include "../../okvis_port/c/ok_eigen.h"

typedef struct rd_cfg {
    /* device */
    int resolution[2];
    double K[9];                               /* camera_intrinsic, column-major */
    double distortion[4];
    size_t camera_distortion_flag;
    double camera_time_offset;
    ok_quat q_bc; double p_bc[3];              /* camera_to_body */
    ok_quat q_bi; double p_bi[3];              /* imu_to_body */
    double keypoint_noise_cov[4];              /* 2x2, column-major */
    double cov_g[9], cov_a[9], cov_bg[9], cov_ba[9];
    /* slam */
    ok_quat q_bo; double p_bo[3];
    size_t sliding_window_size, sliding_window_subframe_size, sliding_window_force_keyframe_landmarks, sliding_window_tracker_frequent;
    double feature_tracker_min_keypoint_distance;
    size_t feature_tracker_max_keypoint_detection, feature_tracker_max_init_frames, feature_tracker_max_frames;
    double feature_tracker_clahe_clip_limit;
    size_t feature_tracker_clahe_width, feature_tracker_clahe_height;
    int feature_tracker_predict_keypoints;
    size_t initializer_keyframe_num, initializer_keyframe_gap, initializer_min_matches;
    double initializer_min_parallax;
    size_t initializer_min_triangulation, initializer_min_landmarks;
    int initializer_refine_imu;
    size_t solver_iteration_limit;
    double solver_time_limit, rotation_misalignment_threshold, rotation_ransac_threshold;
    int random;
    int parsac_flag;
    double parsac_dynamic_probability, parsac_threshold, parsac_norm_scale;
    size_t parsac_keyframe_check_size;
} rd_cfg;

/* 0 on success, else a message in err */
int rd_cfg_load(const char* slam_yaml, const char* device_yaml, rd_cfg* c, char* err, size_t errlen);

#endif
