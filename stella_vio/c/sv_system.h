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
#include "sv_imu.h"

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
    /* ---- stella_vio extensions. sv_system_params_default() sets the stella_vio defaults; the exact port is reproduced with
 * reinit_lost_sec = 0, init_max_level = 0 and the rest as listed in the comments (RESULTS.md) ---- */
    double reinit_lost_sec;            /* default 2.0; > 0: after this many seconds Lost (relocalization failing) archive the trajectory and
                                        * re-initialize into a NEW map (old map dropped, see RESULTS.md); 0 = never (exact port) */
    float init_parallax_deg;           /* initializer parallax threshold, deg (port: 1.0) */
    unsigned int init_min_tri;         /* initializer min triangulated points (port: 50) */
    unsigned int init_seeds;           /* init RANSAC seeds tried, best kept by valid points (port: 1 = seed 5489 only) */
    int init_refine;                   /* 1 = Sampson LM refinement of the initializer pose (PoseLib idea), default 0 */
    int init_lo;                       /* 1 = 5pt LO-RANSAC initializer tried first, default 0 */
    float init_lo_thr;                 /* init_lo inlier threshold, Sampson distance [px]; 0 = 2.0 */
    int pnp_lo;                        /* 1 = P3P LO-RANSAC for relocalization and loop-candidate PnP, default 0 */
    float init_par_frac;               /* 0 = parallax of the 50th point (port); else at this fraction of the valid points */
    unsigned int init_hamm;            /* initializer matcher: max Hamming distance (port: 50) */
    float init_ratio;                  /* initializer matcher: Lowe ratio (port: 0.9) */
    int init_max_level;                /* initializer matcher: highest octave of reference keypoints matched (port: 0) */
    unsigned int init_min_valid;       /* min matches / valid points (port: 50) */
    unsigned int init_confirm;         /* consecutive successful initialization attempts required before the map is built (port: 1) */
    /* ---- stella_vio IMU wiring (inert with imu == NULL) ---- */
    const sv_imu_buf* imu;             /* borrowed; IMU clock = camera clock + imu_toff */
    int gyro_mode;                     /* bit 0: gyro rotation prior for tracking; bit 1: dead-reckoning through Lost (default 0 = exact port) */
    double imu_toff;                   /* [s] */
    double imu_R_BC[9];                /* camera -> body, row-major */
    double imu_bg[3];                  /* gyro bias [rad/s] */
    double dr_max_sec;                 /* dead-reckoning window after the last good frame [s] */
    int gravity;                       /* 1: accumulate the up direction of each map from the accelerometer (sv_system_map_up) */
    /* R-frames (idea of RD-VIO): when tracking fails the camera is followed by a pure-rotation model (rotation RANSAC on bearing pairs to the
     * previous frame, fused with the gyro when available, position from the constant-velocity model, fading out) instead of going Lost; the
     * initializer then runs on the frames and its map is installed INTO the existing map with the rotation chain as the bridge */
    int rframe;                        /* 1: enabled (default 0) */
    double rframe_max_sec;             /* the chain gives up after this long (default 8 s) */
    double rframe_init_after;          /* the deferred initialization starts this long after the chain began (default 0.5 s) */
    double rframe_hold_sec;            /* the constant-velocity position extrapolation fades to a hold over this time (default 1 s) */
    int rframe_scale;                  /* scale of the bridged map: 0 median-depth prior, 1 speed prior, 2 geometric mean (default 1) */
    unsigned int rframe_gyro_max;      /* consecutive vision-less (gyro only) R-frames allowed (default 40) */
    double rframe_calib_sec;           /* the scale of a bridged map part is re-fitted after this long by matching its mean speed to the speed before the
                                        * gap (default 4 s; 0 = keep the initial scale prior) */
    int merge_maps;                    /* 1: keep the old map when a new one is started (reinit) and merge the maps again when place recognition
                                        * finds the old map (default 0: the old map is dropped, see RESULTS.md) */
    /* gait scale servo (section 16 of docs/gnss_vio_benchmark_20261001.md; opt-in, needs sv_system_push_speed() from the host): after every new keyframe
     * the metric distance walked over the last servo_win seconds (walking-speed epochs pushed by the host) is compared with the map's path length over
     * the same time; the ratio is held at the value of the first window (servo_dmin metres) of the map: the newest keyframe and its new landmarks are
     * scaled about the previous keyframe by exp(clip(servo_gain * ln(ratio / reference), +-servo_clip)). servo_gain 0 = off (default, exact path). */
    double servo_gain, servo_win, servo_dmin, servo_clip;
    unsigned int loop_cont;            /* loop detector: consecutive keyframes that must agree on a candidate set (0 = stella's 3); opt-in, section 16 */
    unsigned int loop_matches;         /* loop validation: matches needed (0 = stella's 20); opt-in, section 16 */
    unsigned int servo_k;              /* servo_mode 2: number of non-overlapping windows whose median ratio is the reference (default 3) */
    double servo_dead;                 /* dead band on ln(ratio / reference): only the excess is corrected (default 0) */
    double servo_gate, servo_href;     /* servo_gate > 0: no correction when |ln(ratio / reference)| > gate (default 0 = off); servo_href: metres of history needed (mode 1) */
    int servo_mode;                    /* reference of the servo: 0 = ratio of the first window, 1 = ratio over the history of the map before the window (see sv_system.c) */
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
    int rframe_kind;               /* stella_vio: 0 none, 1 R-frame (vision), 2 R-frame (gyro only), 3 bridged initialization */
    unsigned int rframe_inliers;   /* rotation inliers of the R-frame step */
    double rframe_par_deg;         /* median residual angle of the rotation inliers [deg] (the parallax the pure-rotation model leaves) */
    double cal_f, cal_vold, cal_vnew; /* stella_vio: scale calibration of a bridged part applied in this frame (cal_f != 0) */
    int loop_accepted;             /* correct_loop() ran (and the map was corrected) */
    int loop_cur_kf, loop_cand_kf;
    unsigned int n_keyframes, n_landmarks;   /* map after the frame (get_num_keyframes / get_num_landmarks) */
    unsigned int n_local_kfs, n_local_lms;   /* local map of this frame (0 when tracking did not get that far) */
    const unsigned int* local_kfs;           /* borrowed, valid until the next sv_system_feed() */
    const unsigned int* local_lms;
    /* stella_vio LIVE view of this frame (opt-in for the consumer, nothing else reads it): the pose AS TRACKED at the moment of the frame (pose_wc above), with the
     * labels the trajectory file would give the frame now (map labels can still be merged later, poses corrected by later BA / loops) */
    int live_valid;                /* the frame is a valid, not lost tracked (or R-frame) frame */
    int live_map_id, live_rframe, live_seg;
    unsigned int live_up_n;        /* gravity: frames accumulated for this map so far (0 = unknown) */
    double live_up[3];             /* up direction of the map at this moment, in the map's axes (unit) */
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
/* gravity up direction of a map in that map's world axes (unit vector), from n accumulated frames; returns n (0 = unknown, up untouched) */
unsigned int sv_system_map_up(const sv_system* s, int map_id, double up[3]);
/* one walking-speed epoch from the host (end time t on the frame clock, mean speed over the trailing epoch [m/s]); ignored while servo_gain == 0.
 * Epochs must come in time order, about 3 s apart; a stationary epoch is v = 0. A missing epoch (no valid speed) is a gap: no servo step over it. */
void sv_system_push_speed(sv_system* s, double t, double v);
/* trace of the servo decisions (t, map label, walked metres, map units, ratio, reference ratio (0 = not set yet), ln f applied); rows valid until the system is destroyed */
unsigned int sv_system_servo_log(const sv_system* s, const double (**rows)[7]);
/* map counters */
unsigned int sv_system_num_landmarks(const sv_system* s);
/* run statistics */
typedef struct sv_system_stats {
    unsigned int frames, resets, relocalizations, keyframes_inserted, global_steps, loops_accepted;
    unsigned int lost_frames;
    unsigned int reinits;          /* stella_vio: re-initializations into a new map after a failed relocalization */
    unsigned int merges;           /* stella_vio merge_maps: map merges */
    unsigned int servo_steps;      /* stella_vio gait scale servo: corrections applied */
    unsigned int rframes, rframes_gyro, rbridges, rfail; /* stella_vio R-frames: frames, of them gyro only, bridged initializations, chains that died */
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
    int map_id;                    /* stella_vio: 0 for the first map, +1 per re-initialization */
    int rframe;                    /* stella_vio: 1 = rotation-only frame (position extrapolated, not observed) */
    int seg;                       /* stella_vio: segment label (a bridged map part has its own; map_id is the merged label) */
} sv_traj_entry;
int sv_system_trajectory(const sv_system* s, sv_traj_entry** entries, unsigned int* n);

#endif /* SV_SYSTEM_H */
