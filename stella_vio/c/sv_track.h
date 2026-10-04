/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 * Full BSD-2 notice text in sv_track_frame.c. */
#ifndef SV_TRACK_H
#define SV_TRACK_H

#include <stdint.h>
#include "sv_types.h"
#include "sv_frame.h"
#include "sv_bow.h"
#include "sv_g2o_pose_optimizer.h"
#include "sv_imu.h"

/* Module 5 (tracking side) of the stella_vslam port, monocular / perspective:
 *   module::frame_tracker      (motion_based_track, bow_match_based_track,
 *                               robust_match_based_track, discard_outliers)
 *   match::projection          (match_current_and_last_frames,
 *                               match_frame_and_landmarks)
 *   match::robust              (brute_force_match, match_frame_and_keyframe)
 *   module::local_map_updater  (acquire_local_map)
 *   tracking_module            (track, update_last_frame, update_local_map,
 *                               search_local_landmarks,
 *                               optimize_current_frame_with_local_map,
 *                               update_motion_model, feed_frame's tracking
 *                               state update)
 *   module::keyframe_inserter  (new_keyframe_is_needed)
 *   data::frame / data::keyframe / data::landmark accessors used by the above.
 * Upstream sources: external/candidates/stella_vslam/src/stella_vslam/
 * {module/frame_tracker.cc, module/local_map_updater.cc,
 * module/keyframe_inserter.cc, tracking_module.cc, match/projection.cc,
 * match/robust.cc, match/base.h, data/frame.cc, data/keyframe.cc,
 * data/landmark.{h,cc}, camera/perspective.cc, util/angle.cc}.
 *
 * Data model: landmarks / keyframes are referenced by their (dense) ids; the
 * *map* (sv_tr_map) owns their records; "will_be_erased" == absent from the
 * map (that is how the reference's map snapshots express it: erased objects
 * are removed from the map database in the same step that sets the flag).
 * Frames keep per-keypoint landmark ids (SV_TR_NONE = -1), exactly the
 * upstream `landmarks_` vector (a frame may reference landmarks that were
 * erased after the frame was tracked -- those are skipped like upstream does).
 *
 * All matrices are column-major (m[col*3+row], m[col*4+row]).
 *
 * NOT ported (out of scope, never exercised on fr1_xyz/fr1_desk):
 * tracking_module::initialize(), relocalization, reset() on early loss,
 * marker handling, stereo/depth branches, temporal keyframes
 * (fixed_keyframe_id_threshold > 0).
 */

#define SV_TR_NONE (-1)
#define SV_TR_DESC_BYTES 32
#define SV_TR_MAX_LEVELS 16

typedef struct sv_tr_config {
    /* camera::perspective (monocular) */
    double fx, fy, cx, cy;
    sv_image_bounds bounds; /* camera->img_bounds_ (floats) */
    unsigned int num_grid_cols, num_grid_rows;
    /* feature::orb_params */
    float scale_factor;
    unsigned int num_levels;
    float scale_factors[SV_TR_MAX_LEVELS];
    float inv_scale_factors[SV_TR_MAX_LEVELS];
    float log_scale_factor;
    float inv_level_sigma_sq[SV_TR_MAX_LEVELS];
    /* tracking_module / frame_tracker */
    unsigned int num_matches_thr;               /* 10 */
    float margin_last_frame_projection;         /* 20.0 */
    float margin_local_map_projection;          /* 5.0 */
    float margin_local_map_projection_unstable; /* 20.0 */
    unsigned int max_num_local_keyfrms;         /* 60 */
    sv_pose_optimizer_params pose_opt;          /* {2, 2, 10} */
    /* module::keyframe_inserter (YAML-constructor defaults) */
    double max_interval;                        /* 1.0 */
    double min_interval;                        /* 0.1 */
    double max_distance;                        /* -1.0 */
    double min_distance;                        /* -1.0 */
    double lms_ratio_thr_almost_all_lms_are_tracked; /* 0.9 */
    double lms_ratio_thr_view_changed;          /* 0.5 */
    unsigned int enough_lms_thr;                /* 100 */
    /* bow */
    const sv_bow_vocab* vocab;
} sv_tr_config;

/* Fills the ORB tables (orb_params::calc_*) and the tracking / keyframe
 * inserter defaults the reference config uses. `bounds` must be
 * sv_compute_image_bounds() of the camera. */
void sv_tr_config_init(sv_tr_config* cfg, double fx, double fy, double cx, double cy,
                       const sv_image_bounds* bounds, const sv_bow_vocab* vocab);

/* ---- immutable per-frame / per-keyframe observation data ---- */
typedef struct sv_tr_obs {
    unsigned int num_kp;
    const sv_keypoint* kp;   /* borrowed: undistorted keypoints (x,y,octave,angle) */
    const uint8_t* desc;     /* borrowed: num_kp * 32 */
    sv_frame_grid grid;      /* owned */
    sv_bow_feat_vector bow_feat; /* owned; valid iff bow_ready */
    int bow_ready;
    double* bearings;        /* owned, lazily built (module 6): num_kp * 3, frm_obs_.bearings_ */
} sv_tr_obs;

int sv_tr_obs_init(sv_tr_obs* o, const sv_tr_config* cfg, const sv_keypoint* kp,
                   const uint8_t* desc, unsigned int num_kp);
void sv_tr_obs_free(sv_tr_obs* o);
/* frame::compute_bow / keyframe::compute_bow (BoWFeatVector at level 4). */
int sv_tr_obs_ensure_bow(sv_tr_obs* o, const sv_tr_config* cfg);

/* ---- data::frame ---- */
typedef struct sv_tr_frame {
    unsigned int id;
    double timestamp;
    sv_tr_obs* obs; /* borrowed */
    int pose_valid;
    double pose_cw[16], rot_cw[9], trans_cw[3], rot_wc[9], trans_wc[3];
    int* lm;        /* owned, obs->num_kp entries, SV_TR_NONE = no landmark */
    int ref_kf;     /* reference keyframe id, SV_TR_NONE = none */
} sv_tr_frame;

void sv_tr_frame_init(sv_tr_frame* f, unsigned int id, double timestamp, sv_tr_obs* obs);
void sv_tr_frame_free(sv_tr_frame* f);
int sv_tr_frame_copy(sv_tr_frame* dst, const sv_tr_frame* src); /* deep copy (lm array) */
/* data::frame::set_pose_cw */
void sv_tr_frame_set_pose_cw(sv_tr_frame* f, const double pose_cw[16]);

/* ---- data::landmark / data::keyframe records ---- */
typedef struct sv_tr_lm {
    unsigned int id;
    int alive;
    double pos_w[3];
    uint8_t desc[SV_TR_DESC_BYTES];
    double mean_normal[3];
    float min_valid_dist, max_valid_dist;
    unsigned int num_observed, num_observable;
    int ref_kf;
    unsigned int num_obs;     /* == num_observations() (monocular: 1 per observation) */
    unsigned int* obs_kf;     /* owned, ascending keyframe id */
    unsigned int* obs_idx;    /* owned */
} sv_tr_lm;

typedef struct sv_tr_kf {
    unsigned int id;
    int alive;
    double timestamp;
    double pose_cw[16], pose_wc[16], trans_wc[3];
    sv_tr_obs* obs; /* borrowed */
    int* lm;        /* owned, obs->num_kp: landmark id per keypoint or SV_TR_NONE */
    unsigned int n_covis;
    unsigned int* covis;      /* owned: ordered_covisibilities_ (ids) */
    unsigned int* covis_w;    /* owned: ordered_num_shared_lms_ */
    int parent;
    unsigned int n_children;
    unsigned int* children;   /* owned, ascending id */
    int is_root;              /* graph_node::is_spanning_root() (module 6); parent == SV_TR_NONE && !is_root
                               * is a keyframe whose spanning parent is not set yet */
} sv_tr_kf;

/* data::keyframe::set_pose_cw (pose_wc / trans_wc with the keyframe's own
 * expression: `trans_wc = -rot_wc * trans_cw`, rot_wc materialized). */
void sv_tr_kf_set_pose_cw(sv_tr_kf* k, const double pose_cw[16]);

/* Map database view. kfs[id] / lms[id] are NULL when the id is absent. */
typedef struct sv_tr_map {
    sv_tr_kf** kfs;
    unsigned int kf_cap;
    sv_tr_lm** lms;
    unsigned int lm_cap;
    unsigned int num_keyframes;
    unsigned int kf_floor;           /* stella_vio: keyframes of kept older maps; the "young map" rules (min_num_obs_thr, enough_keyfrms) count num_keyframes - kf_floor */
    int last_inserted_kf;            /* id or SV_TR_NONE */
    double last_inserted_timestamp;
    double last_inserted_trans_wc[3];
    unsigned int fixed_keyframe_id_threshold; /* only 0 supported */
    /* sv_system: optional id -> record table that ALSO holds erased keyframes (upstream keeps an erased
     * keyframe object alive, with its pose, while a frame still references it). NULL in the harnesses. */
    sv_tr_kf* const* kf_pool;
    unsigned int kf_pool_cap;
} sv_tr_map;

const sv_tr_lm* sv_tr_map_lm(const sv_tr_map* m, int id);
const sv_tr_kf* sv_tr_map_kf(const sv_tr_map* m, int id);
/* like sv_tr_map_kf, but falls back to the pool record of an erased keyframe (sv_system only) */
const sv_tr_kf* sv_tr_map_kf_any(const sv_tr_map* m, int id);

/* module::local_map_updater (acquire_local_map, keyframe_id_threshold == 0).
 * kfs = first_local_keyfrms ++ second_local_keyfrms, lms = local landmarks,
 * both in upstream order; nearest_covisibility = keyframe id or SV_TR_NONE. */
typedef struct sv_tr_local_map {
    unsigned int* kfs;
    unsigned int n_kfs, cap_kfs;
    unsigned int* lms;
    unsigned int n_lms, cap_lms;
    int nearest_covisibility;
} sv_tr_local_map;

/* `frm_lms`: the current frame's per-keypoint landmark ids. Returns 1 iff both
 * the local keyframes and the local landmarks were found. */
int sv_tr_acquire_local_map(const sv_tr_config* cfg, const sv_tr_map* map, const int* frm_lms,
                            unsigned int n_frm, sv_tr_local_map* out);
void sv_tr_local_map_free(sv_tr_local_map* m);

/* ---- decisions / results ---- */
typedef enum {
    SV_TR_PATH_NONE = 0,
    SV_TR_PATH_MOTION = 1,
    SV_TR_PATH_BOW = 2,
    SV_TR_PATH_ROBUST = 3,
    SV_TR_PATH_RELOC_BY_POSE = 4, /* sv_system: never used (no pose requests) */
    SV_TR_PATH_RELOC_AUTO = 5,    /* sv_system: Lost state, relocalizer via sv_tracker.reloc_hook */
    SV_TR_PATH_DEAD_RECKON = 6,   /* stella_vio: Lost state, gyro dead-reckoned pose + projection matching against the last good frame */
    SV_TR_PATH_RFRAME = 7         /* stella_vio: Lost state, R-frame pose + projection matching against the last good frame */
} sv_tr_path;

typedef struct sv_tr_kf_decision {
    unsigned int num_reliable_lms_ref, num_reliable_lms, num_tracked_lms;
    float distance_traveled;
    int max_interval_elapsed, min_interval_elapsed, max_distance_traveled, min_distance_traveled;
    int view_changed, not_enough_lms, enough_keyfrms, tracking_is_unstable;
    int almost_all_lms_are_tracked, mapper_is_skipping_localBA, mapper_paused_or_pausing;
    int verdict;
} sv_tr_kf_decision;

/* tracking_module state (the members that persist across frames). */
typedef struct sv_tracker {
    const sv_tr_config* cfg;
    int tracking_state; /* 0 Initializing, 1 Tracking, 2 Lost */
    int twist_valid;
    double twist[16];
    double last_cam_pose_from_ref_keyfrm[16];
    unsigned int last_reloc_frm_id;
    double last_reloc_frm_timestamp;
    sv_tr_frame last_frm;
    int last_frm_valid;
    sv_tr_frame curr_frm;

    /* sv_system (autonomous run): relocalization while Lost (tracking_module::track's relocalizer branch),
     * called with curr_frm installed (ref keyframe inherited from the last frame); returns 1 on success
     * and must set curr_frm's pose. NULL: no relocalization (module-5 harnesses). */
    int (*reloc_hook)(void* user, struct sv_tracker* t, const sv_tr_map* map);
    void* reloc_user;
    /* mapper_->is_paused() || pause_is_requested() as seen by keyframe_inserter (sv_system; 0 in the harnesses) */
    int mapper_paused;

    /* test hooks reproducing reference patch 0010 (opt-in fault injection):
     * skip the motion-model / BoW tracker for the next sv_tracker_track()
     * call. Always 0 in normal use. */
    int force_skip_motion, force_skip_bow;

    /* stella_vio gyro prior (all inert while imu == NULL or gyro_mode == 0: the exact-port path).
     * gyro_mode bit 0: rotation of the motion-model / BoW / robust initial pose from the gyro; bit 1: Lost dead-reckoning (see sv_tracking.c) */
    const sv_imu_buf* imu;
    int gyro_mode;
    double imu_toff;            /* IMU clock = camera clock + imu_toff [s] */
    double R_BC[9];             /* camera -> body (row-major) */
    double bg[3];               /* gyro bias [rad/s] */
    double lost_max_sec;        /* dead-reckoning is attempted for this long after the last good frame */
    int vel_valid;              /* constant world-frame velocity of the camera centre: vel_w per vel_dt seconds */
    double vel_w[3], vel_dt;
    sv_tr_frame good_frm;       /* last successfully tracked frame (gyro_mode bit 1, or keep_good) */
    int good_valid;
    int keep_good;              /* R-frames: keep good_frm */
    int r_pose_valid;           /* R-frames: r_pose (column-major pose_cw) is the rotation-chain pose of the frame about to be tracked */
    double r_pose[16];
    unsigned int n_gyro_track, n_dr_try, n_dr_ok;   /* statistics */

    /* per-frame outputs of sv_tracker_track() */
    int succeeded;
    sv_tr_path path;
    int initial_pose_valid;
    double initial_pose[16];    /* pose after track_current_frame(), before local map */
    unsigned int num_tracked_lms, num_reliable_lms;
    int optimize_ran;           /* optimize_current_frame_with_local_map() ran in this call (counters are then fresh) */
    sv_tr_local_map local;      /* local_map_updater result of this frame */
    sv_tr_kf_decision decision;
    int decision_evaluated;
} sv_tracker;

void sv_tracker_init(sv_tracker* t, const sv_tr_config* cfg);
void sv_tracker_free(sv_tracker* t);

/* tracking_module::feed_frame(), tracking part (state == Tracking):
 *   curr_frm = input; track(); new_keyframe_is_needed().
 * `input` supplies id/timestamp/obs only (its landmarks are ignored, as a
 * freshly extracted frame has none). `min_num_obs_thr` is derived from
 * map->num_keyframes exactly like feed_frame. Mutates landmark
 * num_observed/num_observable in `map` (upstream does). Returns 1 iff
 * tracking succeeded. Sets t->decision when new_keyframe_is_needed ran and
 * t->decision.verdict == 1 iff a keyframe would be inserted. */
int sv_tracker_track(sv_tracker* t, sv_tr_map* map, const sv_tr_frame* input);

/* Second half of feed_frame(): after (optional) keyframe insertion.
 * `inserted_kf_id` >= 0 sets curr_frm.ref_kf to the new keyframe (upstream:
 * insert_new_keyframe). Applies the tracking-state transition, computes
 * last_cam_pose_from_ref_keyfrm = curr.pose_cw * ref_kf->pose_wc using
 * `map` (which must then be the state AFTER mapping, containing the new
 * keyframe with its final pose), and sets last_frm = curr_frm. */
int sv_tracker_finish_frame(sv_tracker* t, const sv_tr_map* map, int inserted_kf_id);

/* module::keyframe_inserter::create_new_keyframe (monocular, no markers, no
 * depth): keyframe::make_keyframe(new_id, curr_frm) followed by
 * keyframe::update_landmarks() -- for every non-erased landmark of the frame,
 * in keypoint order: add_observation(kf, idx), update_mean_normal_and_obs_
 * scale_variance(), compute_descriptor(). Fills `kf` (caller storage; its
 * obs must be the frame's obs, kf->lm is allocated here), registers it in
 * map->kfs[new_id] (capacity must be > new_id) and updates the landmark
 * records of `map` in place. The mapping-module side of the insertion
 * (queueing, culling, triangulation, BA, map_db_->add_keyframe) is not part
 * of this function. Returns 0 on success. */
int sv_tr_create_new_keyframe(const sv_tr_config* cfg, sv_tr_map* map, const sv_tr_frame* curr,
                              unsigned int new_id, double timestamp, sv_tr_kf* kf);

/* ---- lower-level pieces (exposed for the harness) ---- */
unsigned int sv_tr_hamming(const uint8_t* a, const uint8_t* b);
float sv_tr_angle_diff(float a, float b);

/* camera::perspective::reproject_to_image. */
int sv_tr_reproject_to_image(const sv_tr_config* cfg, const double rot_cw[9], const double trans_cw[3],
                             const double pos_w[3], double reproj[2], float* x_right);

/* match::projection::match_current_and_last_frames (check_orientation=true). */
unsigned int sv_tr_match_current_and_last_frames(const sv_tr_config* cfg, const sv_tr_map* map,
                                                 sv_tr_frame* curr, const sv_tr_frame* last, float margin);

/* module::frame_tracker */
int sv_tr_motion_based_track(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr,
                             const sv_tr_frame* last, const double velocity[16]);
int sv_tr_bow_match_based_track(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr,
                                const sv_tr_frame* last, const sv_tr_kf* ref_kf);
int sv_tr_robust_match_based_track(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_frame* curr,
                                   const sv_tr_frame* last, const sv_tr_kf* ref_kf);

/* pose_optimizer_g2o::optimize(frame, ...) glue: builds the pose edges from
 * `curr`'s landmarks, runs sv_pose_optimizer_optimize. outlier[idx] (caller
 * allocated, num_kp entries) is set for every keypoint. Returns num valid
 * observations; *pose_out = optimized (or, when < 5 observations, the input)
 * pose_cw. */
unsigned int sv_tr_optimize_pose(const sv_tr_config* cfg, const sv_tr_map* map, const sv_tr_frame* curr,
                                 double pose_out[16], unsigned char* outlier);

/* ---- module 6 (mapping) helpers ---- */
/* landmark::update_mean_normal_and_obs_scale_variance() / compute_descriptor()
 * on a map record (sv_kf_insert.c); 0 on success. */
int sv_tr_lm_update_mean_normal_and_obs_scale_variance(const sv_tr_config* cfg, const sv_tr_map* map, sv_tr_lm* lm);
int sv_tr_lm_compute_descriptor(const sv_tr_map* map, sv_tr_lm* lm);
/* frame_observation::bearings_ (camera::perspective::convert_point_to_bearing),
 * built on first use. */
int sv_tr_obs_ensure_bearings(sv_tr_obs* o, const sv_tr_config* cfg);

#endif /* SV_TRACK_H */
