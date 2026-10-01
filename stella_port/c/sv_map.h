/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 * Full BSD-2 notice text in sv_map.c. */
#ifndef SV_MAP_H
#define SV_MAP_H

#include <stdint.h>
#include "sv_types.h"

/* Port of stella_vslam's map data model and monocular initial-map
 * creation: data::keyframe, data::landmark, data::graph_node (spanning
 * tree only -- covisibility connections are NOT populated by
 * module::initializer::create_map_for_monocular(); update_connections()
 * is only ever called later, from the mapping module, which is out of
 * this module's scope -- see stella_port/HANDOVER.md module-4a entry),
 * and data::map_database's id/keyframe/landmark bookkeeping, driven by
 * module::initializer::create_map_for_monocular()
 * (runs/stella_port/reference_build/src/src/stella_vslam/module/
 * initializer.cc) up to but NOT including its global bundle adjustment
 * call (optimize::global_bundle_adjuster -- ported separately, see
 * stella_port/c/sv_g2o*.{h,c}).
 *
 * Two-stage API, matching the harness's two dump snapshots:
 *   - sv_map_build_pre_ba(): builds the two initial keyframes and every
 *     triangulated landmark (make_keyframe, landmark ctor,
 *     connect_to_keyframe, compute_descriptor,
 *     update_mean_normal_and_obs_scale_variance, spanning-tree wiring),
 *     from module 3's already-exact initializer output (rot/trans,
 *     triangulated points, matches) -- this is the PRE-BA dump.
 *   - sv_map_apply_post_ba(): given INJECTED post-BA keyframe poses and
 *     landmark positions (the real global_bundle_adjuster is a separate,
 *     concurrently-developed module; this harness takes its output as a
 *     fixture, exactly as module::initializer::create_map_for_monocular
 *     itself only reads back keyframe/landmark state after the BA call
 *     returns -- BA mutates keyframe/landmark state in place, this
 *     function's caller does the same before calling it), recomputes
 *     everything AFTER BA: mean normal + ORB scale variance for every
 *     landmark, median-depth map scaling (keyframe::compute_median_depth,
 *     module::initializer::scale_map), and the scale verdict. This is
 *     the POST-BA dump.
 *
 * Pose convention: Mat44_t/Mat33_t are Eigen column-major, matching
 * sv_linalg.h/sv_eigen_svd.h (m[col*3+row] for 3x3, m[col*4+row] for
 * 4x4). Eigen-derived pose-frame math (pose_wc_/trans_wc_ derivation)
 * reuses sv_linalg.h's already-measured evaluation-order primitives.
 */

typedef struct sv_map_orb_params {
    float scale_factor; /* 1.2 */
    int num_levels; /* 8 */
    /* Derived (orb_params::calc_scale_factors/calc_inv_scale_factors):
     * scale_factors[0]=1, scale_factors[l]=scale_factor*scale_factors[l-1];
     * inv_scale_factors[0]=1, inv_scale_factors[l]=(1/scale_factor)*inv_scale_factors[l-1].
     * Filled by sv_map_orb_params_init(); num_levels entries. */
    float scale_factors[32];
    float inv_scale_factors[32];
} sv_map_orb_params;

void sv_map_orb_params_init(sv_map_orb_params* p, float scale_factor, int num_levels);

/* One keyframe: id, camera pose (pose_cw, column-major 4x4), and derived
 * pose_wc/trans_wc (data::keyframe::set_pose_cw). frm_obs is the keyframe's
 * OWN keypoint set (module 1 output for that frame_id): needed to read
 * back an observed landmark's octave (compute_orb_scale_variance) and the
 * representative-descriptor candidate rows (landmark::compute_descriptor).
 * markers/graph covisibility are out of scope (see header comment above)
 * and always empty/trivial for a 2-keyframe init map. */
typedef struct sv_map_keyframe {
    unsigned int id;
    double pose_cw[16]; /* column-major 4x4 */
    double pose_wc[16];
    double trans_wc[3];

    const sv_keypoint* keypts; /* borrowed, num_keypts entries */
    const uint8_t* descriptors; /* borrowed, num_keypts*32 bytes, row-major */
    unsigned int num_keypts;

    const sv_map_orb_params* orb_params; /* borrowed */

    /* graph_node: spanning tree only (see header comment). -1 = none. */
    int spanning_parent_id;
    int spanning_root_id;
    unsigned int spanning_children[8];
    unsigned int num_spanning_children;
} sv_map_keyframe;

void sv_map_keyframe_init(sv_map_keyframe* kf, unsigned int id,
                          const double pose_cw_colmajor[16],
                          const sv_keypoint* keypts, const uint8_t* descriptors,
                          unsigned int num_keypts,
                          const sv_map_orb_params* orb_params);

/* data::keyframe::set_pose_cw() on an already-initialized sv_map_keyframe:
 * overwrites pose_cw and re-derives pose_wc/trans_wc, leaving every other
 * field (id, keypts/descriptors, spanning tree) untouched. Used by the
 * harness to inject an external (real global_bundle_adjuster) post-BA
 * pose for curr_keyfrm before calling sv_map_apply_post_ba(). */
void sv_map_keyframe_set_pose_cw(sv_map_keyframe* kf, const double pose_cw_colmajor[16]);

/* One (keyframe_id, keypoint idx) observation of a landmark. */
typedef struct sv_map_observation {
    unsigned int keyframe_id;
    unsigned int idx;
} sv_map_observation;

typedef struct sv_map_landmark {
    unsigned int id;
    unsigned int first_keyfrm_id;
    double pos_w[3];

    sv_map_observation observations[8];
    unsigned int num_observations;
    unsigned int ref_keyfrm_id; /* == the keyframe passed to the landmark ctor
                                  * (curr_keyfrm for every module-4a landmark) */

    uint8_t descriptor[32]; /* landmark::compute_descriptor() result */

    double mean_normal[3];
    float min_valid_dist;
    float max_valid_dist;

    unsigned int num_observable; /* == 1, never touched in this module */
    unsigned int num_observed; /* == 1, never touched in this module */
} sv_map_landmark;

/* One initial-map keyframe pair + its landmarks, module::initializer's
 * create_map_for_monocular() state (map_database next_keyframe_id_/
 * next_landmark_id_ start at 0 -- this port only ever builds ONE initial
 * map, so ids are always keyframe 0/1 and landmark 0..n-1). */
typedef struct sv_map_init_map {
    sv_map_keyframe init_keyfrm; /* id 0 */
    sv_map_keyframe curr_keyfrm; /* id 1 */
    sv_map_landmark* landmarks; /* caller-allocated, cap entries */
    unsigned int num_landmarks;

    /* POST-BA only (sv_map_apply_post_ba() fills these): */
    float median_scale;
    double inv_median_scale;
    double applied_scale; /* inv_median_scale * scaling_factor_ (1.0) */
    int reset_wrong_init; /* 1 iff module::initializer would set state_=Wrong
                            * and return false (tracked landmarks < 50 AND
                            * median_scale < 0) */
} sv_map_init_map;

/* Builds init_keyfrm (id 0, pose = Identity) and curr_keyfrm (id 1, pose
 * from rot_ref_to_cur/trans_ref_to_cur, column-major 3x3/3-vector) plus
 * every triangulated landmark, exactly matching
 * module::initializer::create_map_for_monocular()'s pre-BA block:
 *   - init_matches[i] (>=0, length num_kp_ref) gives ref_idx -> curr_idx;
 *     is_triangulated[i] (length num_kp_ref) invalidates non-triangulated
 *     matches first (matching create_map_for_monocular's own loop).
 *   - triangulated_pts: num_kp_ref*3 doubles, row-major (idx*3+{0,1,2}),
 *     already-triangulated 3D points in the ref camera frame (module 3's
 *     initialize::perspective output -- taken as exact input here, NOT
 *     recomputed).
 * landmarks_out must have capacity >= num_kp_ref entries. Returns the
 * number of landmarks created (== map->num_landmarks). */
unsigned int sv_map_build_pre_ba(
    unsigned int ref_frame_id, unsigned int cur_frame_id,
    const sv_keypoint* ref_keypts, const uint8_t* ref_descriptors, unsigned int num_kp_ref,
    const sv_keypoint* cur_keypts, const uint8_t* cur_descriptors, unsigned int num_kp_cur,
    const sv_map_orb_params* orb_params,
    const double rot_ref_to_cur[9], const double trans_ref_to_cur[3],
    const int* init_matches, const unsigned char* is_triangulated,
    const double* triangulated_pts,
    sv_map_landmark* landmarks_out,
    sv_map_init_map* map);

/* POST-BA: given the map from sv_map_build_pre_ba() with curr_keyfrm.pose_cw
 * and every landmark's pos_w already OVERWRITTEN in place by the caller
 * with injected post-BA values (init_keyfrm's pose is assumed unchanged --
 * global_bundle_adjuster fixes keyframe 0 -- caller must still call
 * sv_map_keyframe_init or otherwise refresh pose_wc/trans_wc if it did
 * change), recomputes: every landmark's mean normal + ORB scale variance,
 * then module::initializer::scale_map()'s median-depth scaling (using
 * data::keyframe::compute_median_depth(abs=true) on init_keyfrm) and
 * re-recomputes mean normal + ORB scale variance again post-scale.
 * min_num_triangulated_pts: module::initializer's min_num_triangulated_pts_
 * (default 50), for the wrong-init check. scaling_factor: module::
 * initializer's scaling_factor_ (default 1.0). */
void sv_map_apply_post_ba(sv_map_init_map* map,
                          unsigned int min_num_triangulated_pts,
                          double scaling_factor);

#endif /* SV_MAP_H */
