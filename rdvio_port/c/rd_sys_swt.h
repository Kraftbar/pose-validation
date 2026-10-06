/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M10: rdvio::SlidingWindowTracker (rdvio/src/sliding_window_tracker.cpp) on the C map layer.
 *   create          : the constructor (preintegration of every keyframe with the previous frame's biases)
 *   mirror_frame    : clone of the feature-tracking frame (its IMU data extended back to the last mirrored frame), the
 *                     tracks re-linked from the last keyframe / subframe, the trash prune, preintegration + prediction
 *   track           : [parsac] judge_track_status (PnP inlier mask, predicted epipolar geometry from the IMU poses, the
 *                     median thresholds, OUTLIER / STATIC tags) and update_track_status (essential-matrix masks against the
 *                     last keyframes, STATIC tags in both maps); localize_newframe; manage_keyframe; on a keyframe
 *                     track_landmark, refine_window (the window BA with the marginalization prior), slide_window
 *                     (marginalization, module M5); otherwise refine_subwindow
 *   latest_state    : get_latest_state
 * The two PARSAC estimators (find_pnp_matrix_parsac_imu, find_essential_matrix_parsac) are hooks: only their inlier masks
 * are used. Without a hook the step is skipped (judge returns false; a frame's 2D-2D check is skipped), which is NOT the
 * reference behaviour when parsac_flag is set.
 * Derived from RD-VIO (Apache-2.0). C99.
 */
#ifndef RD_SYS_SWT_H
#define RD_SYS_SWT_H
#include "rd_map.h"
#include "rd_marg.h"
#include "rd_sys_config.h"
#include "rd_solve.h"

typedef struct rd_swt rd_swt;
typedef struct rd_swt_hooks {
    void* ctx;
    /* find_pnp_matrix_parsac_imu(P3D, P2D, lens, Rcw, tcw, 0.20, 1.0, mask, inv_f): fill mask[n]. p3d n x 3, p2d n x 2 */
    void (*pnp_mask)(void* ctx, size_t n, const double* p3d, const double* p2d, const size_t* lens, const double Rcw[9],
                     const double tcw[3], double inv_f, char* mask);
    /* find_essential_matrix_parsac(pts1, pts2, mask, threshold): fill mask[n]. pts n x 2 */
    void (*ess_mask)(void* ctx, size_t n, const double* pts1, const double* pts2, double threshold, char* mask);
    /* checkpoints in track() (harness): stage 11 judge (value = result), 12 update, 13 localize, 14 manage (value = keyframe),
     * 15 track_landmark, 16 refine_window, 17 slide_window, 18 refine_subwindow */
    void (*stage)(void* ctx, rd_swt* s, int stage, int value);
} rd_swt_hooks;

struct rd_swt {
    const rd_cfg* cfg;
    rd_map* map;                       /* the keyframe map (owned) */
    rd_map* ft;                        /* feature_tracking_map (not owned; update_track_status) */
    rd_marg* marg;                     /* map->marginalization_factor: NULL until the first refine_window */
    double m_th;
    const rd_sv_hooks* sv_hooks;       /* observers of the Ceres solves, may be NULL */
    rd_swt_hooks hooks;
};

void rd_swt_create(rd_swt* s, rd_map* keyframe_map, const rd_cfg* cfg);
void rd_swt_destroy(rd_swt* s);
void rd_swt_mirror_frame(rd_swt* s, rd_map* feature_tracking_map, uint64_t frame_id);
int rd_swt_track(rd_swt* s);
void rd_swt_latest_state(const rd_swt* s, double* t, rd_pose* pose, rd_motion* motion);
/* Map::marginalize_frame's marginalization_factor->marginalize(index): the rd_map_hooks.marginalize of the keyframe map */
void rd_swt_marginalize(rd_swt* s, size_t index);

#endif
