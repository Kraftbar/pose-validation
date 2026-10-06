/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * OKVIS2 pure-C port, module 8: the configuration (okvis_common ViParametersReader.cpp, parameters.hpp) read from the
 * OKVIS2 YAML file.
 *
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). Redistribution requires retaining this notice.
 *
 * C99, <stdlib.h> <string.h> <stdio.h> <ctype.h> only. The reader understands the YAML subset the OKVIS2 configs use
 * (block mappings by indentation, `- ` sequences, flow `[...]` / `{...}` spanning lines, `#` comments, the `%YAML` line);
 * numbers go through strtod / strtol like cv::FileStorage, booleans follow ViParametersReader::parseEntry (int != 0 or the
 * words true / yes / y / on, false / no / n / off). Camera extrinsics: Transformation(Matrix4d T_SC), then
 * Transformation(r, q.normalized()) as in readConfigFile. Not read: CNN, depth-camera and output options (unused).
 */
#ifndef OK_CONFIG_H
#define OK_CONFIG_H
#include <stddef.h>
#include "ok_vigraph.h"

#define OK_CFG_MAXCAM 4

typedef struct ok_cfg_cam {
    int dist;                  /* OK_CAM_RADTAN / OK_CAM_EQUIDISTANT */
    int w, h;
    double fu, fv, cu, cv, d[4];
    double T_SC[7];            /* r, q (x y z w) of the NCameraSystem extrinsics */
    int used;                  /* slam_use starts with "okvis" */
} ok_cfg_cam;

typedef struct ok_cfg {
    int ncam;
    ok_cfg_cam cam[OK_CFG_MAXCAM];
    /* camera_parameters */
    double timestamp_tolerance, image_delay;
    int nsync, sync_cameras[OK_CFG_MAXCAM];
    int do_extrinsics;
    double sigma_r, sigma_alpha;
    /* imu_parameters (ViSlamBackend::addImu input) */
    ok_vg_imu_cfg imu;
    /* frontend_parameters */
    double detection_threshold, absolute_threshold, matching_threshold, keyframe_overlap;
    int octaves, max_num_keypoints, num_matching_threads, parallelise_detection, use_cnn;
    /* estimator_parameters */
    int num_keyframes, num_loop_closure_frames, num_imu_frames, do_loop_closures, do_final_ba, enforce_realtime;
    int realtime_min_iterations, realtime_max_iterations, realtime_num_threads, full_graph_iterations, full_graph_num_threads;
    double realtime_time_limit, p_dbow, drift_percentage;
} ok_cfg;

/* 0 on success; on failure a message in err */
int ok_cfg_load(const char* path, ok_cfg* c, char* err, size_t errlen);

#endif
