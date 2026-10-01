/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD 2-Clause License
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 * Full BSD-2 notice text in sv_system.c. */
#ifndef SV_SYSTEM_H
#define SV_SYSTEM_H

#include <stdint.h>
#include "sv_extract.h"
#include "sv_loop.h"
#include "sv_relocalizer.h"

/* Top-level system of the stella_vslam port (monocular, perspective, DETERMINISTIC single-threaded build):
 *   system::feed_monocular_frame -> tracking_module::feed_frame (initialize / track / relocalize / lost /
 *   reset / keyframe insertion) -> mapping_module::mapping_with_new_keyframe (inline, like the reference's
 *   synchronous entry points) -> [system::synchronize_background_modules] global_optimization_module::
 *   run_step (loop detection, validation, correct_loop, loop BA) for every keyframe the mapper queued.
 * Nothing is teacher forced: every piece of state (map, tracker, mapper hidden state, loop detector state, BoW
 * database, keyframe lifetimes) is derived from the system's own previous state. The only inputs are the
 * gray image (the exact image orb_extractor::extract() sees, see sv_run.c) and the timestamp.
 *
 * The library never prints and never touches files. */

typedef struct sv_system_params {
    sv_camera_params cam;              /* fx fy cx cy k1 k2 p1 p2 k3 */
    int cols, rows;
    sv_orb_params orb;
    const sv_bow_vocab* vocab;
    double init_retry_threshold_time;  /* Tracking.init_retry_threshold_time, 5.0 */
    /* The deterministic reference (system::startup_single_threaded) never runs a mapping thread, so
     * mapping_module::resume() (called at the end of every correct_loop()) returns early on `is_terminated_` and the
     * mapper stays "pause requested" for the rest of the run: keyframe_inserter::new_keyframe_is_needed() then
     * returns false for every later frame. 0 (default) reproduces that; 1 gives the threaded upstream behaviour
     * (mapper resumed after every loop correction). */
    int resume_mapper_after_loop;
    int enable_loop_closure;           /* 1 (default): the global optimization module runs */
} sv_system_params;

/* Fills stella's reference TUM config (stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml). */
void sv_system_params_default(sv_system_params* p, const sv_bow_vocab* vocab);

typedef struct sv_system sv_system;

/* per-frame report of sv_system_feed() (everything the reference driver dumps for a frame) */
typedef struct sv_frame_result {
    unsigned int frame_id;
    double timestamp;
    int tracking_state_before;     /* 0 Initializing, 1 Tracking, 2 Lost */
    int tracking_state_after;
    /* tracking_module::last_track_path_ etc. keep their value across frames in which track() does not run; this
     * report reproduces that persistence so that it can be diffed against frame_trace.tsv */
    int track_path;                /* sv_tr_path */
    int initial_pose_valid;
    double initial_pose[16];       /* column-major */
    int pose_valid;                /* the pose feed_frame() returned (curr_frm_.get_pose_wc()) */
    double pose_wc[16];            /* column-major [rot_wc | trans_wc] */
    unsigned int num_tracked, num_reliable;
    int ref_kf;                    /* curr_frm_.ref_keyfrm_->id_ after the frame, -1 none */
    sv_tr_kf_decision decision;    /* keyframe_inserter::get_last_decision() (persists) */
    int decision_evaluated;
    int track_succeeded;
    int inserted_kf;               /* keyframe id or -1 */
    int initialized;               /* a map was created in this frame */
    int reset_happened;
    unsigned int n_global_steps;   /* keyframes processed by the global optimization module in this frame */
    int loop_accepted;             /* correct_loop() ran (and the map was corrected) */
    int loop_cur_kf, loop_cand_kf;
    unsigned int n_keyframes, n_landmarks;   /* map after the frame (get_num_keyframes / get_num_landmarks) */
    unsigned int n_local_kfs, n_local_lms;   /* local map of this frame (0 when tracking did not get that far) */
    const unsigned int* local_kfs;           /* borrowed, valid until the next sv_system_feed() */
    const unsigned int* local_lms;
} sv_frame_result;

sv_system* sv_system_create(const sv_system_params* p);
void sv_system_destroy(sv_system* s);

/* feed_monocular_frame(): gray = rows * cols bytes (row-major, step == cols). Returns 0, or -1 on failure. */
int sv_system_feed(sv_system* s, const uint8_t* gray, double timestamp, sv_frame_result* out);

/* --- read-only views for drivers / tests --- */
const sv_tr_map* sv_system_map(const sv_system* s);
sv_mapping* sv_system_mapping(sv_system* s);
const sv_loop* sv_system_loop(const sv_system* s);
const sv_tracker* sv_system_tracker(const sv_system* s);
/* the current frame's per-keypoint landmark ids (tracking_module::curr_frm_), -1 = none; n = keypoints */
const int* sv_system_curr_landmarks(const sv_system* s, unsigned int* n);
/* map counters */
unsigned int sv_system_num_landmarks(const sv_system* s);
/* run statistics */
typedef struct sv_system_stats {
    unsigned int frames, resets, relocalizations, keyframes_inserted, global_steps, loops_accepted;
    unsigned int lost_frames;
    unsigned int erased_keyframes, destroyed_keyframes;
} sv_system_stats;
void sv_system_get_stats(const sv_system* s, sv_system_stats* st);
/* keyframe lifetime log: destruction (expiry) frame per keyframe id (-1 = not destroyed) and the erase frame */
int sv_system_kf_erased_frame(const sv_system* s, unsigned int kf_id);
int sv_system_kf_destroyed_frame(const sv_system* s, unsigned int kf_id);

/* system::save_frame_trajectory(): one entry per valid, not lost frame (frame ids ascending), with the FINAL keyframe
 * poses (pose_wc column-major). *entries is malloc'ed (free by caller). */
typedef struct sv_traj_entry {
    unsigned int frame_id;
    double timestamp;
    double pose_wc[16];            /* column-major */
    double quat_xyzw[4];           /* Eigen Quaterniond(rot_wc) coefficients */
} sv_traj_entry;
int sv_system_trajectory(const sv_system* s, sv_traj_entry** entries, unsigned int* n);

#endif /* SV_SYSTEM_H */
