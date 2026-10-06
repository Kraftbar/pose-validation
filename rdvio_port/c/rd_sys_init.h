/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M9: rdvio::Initializer (rdvio/src/initializer.cpp) on the C map layer.
 *   mirror_keyframe_map: keyframe_num clones of the feature-tracking map's frames, keyframe_gap apart, ending at the issued
 *                        frame; tracks re-created between consecutive keyframes; each keyframe gets the IMU data of the frames
 *                        since the previous keyframe
 *   initialize          : init_sfm (homography / essential RANSAC with config.random() as seed, the 8 (R, T) hypotheses scored by
 *                        triangulation, PnP of the middle frames by Ceres, more triangulation, the visual BA, the prune of invalid
 *                        tracks), init_imu (gyro bias by JacobiSVD, gravity / scale / velocities by FullPivHouseholderQR, the
 *                        gravity refinement, apply_init with FromTwoVectors), the visual-inertial BA. On success the map is handed
 *                        to the sliding-window tracker (rd_sys_init_take_map).
 * Derived from RD-VIO (Apache-2.0). C99.
 */
#ifndef RD_SYS_INIT_H
#define RD_SYS_INIT_H
#include "rd_map.h"
#include "rd_sys_config.h"
#include "rd_solve.h"

typedef struct rd_init {
    const rd_cfg* cfg;
    rd_map* map;                       /* std::unique_ptr<Map> map (NULL: none) */
    double bg[3], ba[3], gravity[3], scale;
    double (*vel)[3]; size_t nvel;     /* velocities */
    const rd_sv_hooks* hooks;          /* observers of the Ceres solves (harness), may be NULL */
    /* optional checkpoints (harness): after init_sfm (before the BA prune result is known: called with ok), after init_imu */
    void (*on_stage)(void* ctx, const struct rd_init* in, int stage, int ok);
    void* ctx;
} rd_init;

void rd_init_create(rd_init* in, const rd_cfg* cfg);
void rd_init_destroy(rd_init* in);
void rd_init_mirror_keyframe_map(rd_init* in, rd_map* feature_tracking_map, uint64_t init_frame_id);
/* returns 1 when initialized: the caller takes the map (rd_init_take_map) and builds the sliding-window tracker */
int rd_init_initialize(rd_init* in);
rd_map* rd_init_take_map(rd_init* in);

/* Frame::set_pose(sensor, pose): pose.q = q * sensor.q_cs^*, pose.p = p - pose.q * sensor.p_cs */
void rd_frame_set_pose(rd_frame* f, const ok_quat* sensor_q, const double sensor_p[3], const ok_quat* q, const double p[3]);
/* shared with module M10: Frame::get_pose(camera); Track::get_landmark_point / set_landmark_point (first keypoint);
 * Track::triangulate (observations in keypoint_map order; m_life = 1 when valid; returns 0 for std::nullopt) */
void rd_sys_cam_pose(const rd_frame* f, ok_quat* q, double p[3]);
void rd_sys_get_landmark_point(const rd_track* t, double p[3]);
void rd_sys_set_landmark_point(rd_track* t, const double p[3]);
int rd_sys_track_triangulate(rd_track* t, double p[3]);

#endif
