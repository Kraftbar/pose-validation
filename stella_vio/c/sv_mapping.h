/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 * Full BSD-2 notice text in sv_mapping.c. */
#ifndef SV_MAPPING_H
#define SV_MAPPING_H

#include "sv_track.h"
#include "sv_rbtree.h"

/* Module 6 (mapping module) of the stella_vslam port, monocular / perspective,
 * DETERMINISTIC build, synchronous single-threaded:
 *   mapping_module::mapping_with_new_keyframe (store_new_keyframe,
 *     create_new_landmarks, triangulate_with_two_keyframes, update_new_keyframe,
 *     fuse_landmark_duplication, local BA call + application of its results)
 *   module::local_map_cleaner (remove_invalid_landmarks,
 *     remove_redundant_keyframes, count_redundant_observations)
 *   data::graph_node (update_connections, add/erase_connection,
 *     erase_all_connections, update_covisibility_orders, recover_spanning_connections)
 *   data::keyframe::prepare_for_erasing, data::landmark::{add_observation,
 *     erase_observation, prepare_for_erasing, replace, connect_to_keyframe}
 *   match::bow_tree::match_for_triangulation, match::fuse::detect_duplication
 *   module::two_view_triangulator, solve::triangulator (4x4 JacobiSVD overload),
 *   solve::essential_solver::create_E_21.
 * Upstream: external/candidates/stella_vslam/src/stella_vslam/{mapping_module.cc,
 * module/local_map_cleaner.cc, module/two_view_triangulator.{h,cc},
 * data/{graph_node,keyframe,landmark}.cc, match/{bow_tree,fuse}.cc,
 * solve/{triangulator.h,essential_solver.cc}, optimize/local_bundle_adjuster_g2o.cc}.
 *
 * The data model is sv_track.h's (sv_tr_map / sv_tr_kf / sv_tr_lm). The state
 * that snapshots do not expose is held in sv_mapping:
 *   - fresh_landmarks_ (local_map_cleaner), std::list with duplicates;
 *   - every keyframe's connected_keyfrms_and_num_shared_lms_ (full count map;
 *     the *ordered* covisibility lists live in sv_tr_kf::covis/covis_w);
 *   - landmark::first_keyfrm_id_; map_database::next_landmark_id_.
 * Erased objects are removed from the map tables (NULL slot, alive = 0) but
 * the records are never freed here (the owner of the pool does that), so
 * pointers held across one step stay valid like upstream's shared_ptrs. */

typedef struct sv_mapping_tri {
    unsigned int ngh_id;
    unsigned int n_matches;
    unsigned int (*matches)[2]; /* (idx in current keyframe, idx in neighbor) */
    unsigned int n_acc;
    unsigned int* acc_ids;
    double (*acc_pos)[3];
} sv_mapping_tri;

/* per-step trace, the C mirror of reference patch 0005's mapping_pass_trace */
typedef struct sv_mapping_trace {
    unsigned int cur_id;
    unsigned int n_culled_lms;
    unsigned int* culled_lms;
    unsigned int n_tri, cap_tri;
    sv_mapping_tri* tri;
    unsigned int n_replaced;
    unsigned int (*replaced)[2]; /* (replaced away, replaced by), ascending first */
    unsigned int n_culled_kfs;
    unsigned int* culled_kfs;
    int local_ba_invoked;
    /* diagnostics */
    unsigned int n_span_ties; /* recover_spanning_connections: equal-count candidates (pointer-ordered upstream) */
    unsigned int n_dup_connect; /* fuse: same landmark connected twice to a keyframe (order-dependent upstream) */
    unsigned int n_stale_neighbor; /* covisibility list entry of an erased but not yet destroyed keyframe (skipped) */
} sv_mapping_trace;

typedef struct sv_mapping {
    const sv_tr_config* cfg;
    sv_tr_map* map;
    /* parameters (reference config) */
    unsigned int min_num_shared_lms;     /* 15 (system_params min_num_shared_lms) */
    unsigned int num_cov_gen;            /* 20 */
    unsigned int num_cov_fuse;           /* 20 */
    double baseline_dist_thr_ratio;      /* 0.02 */
    float residual_rad_thr;              /* float(0.2f * M_PI / 180.0) */
    double observed_ratio_thr;           /* 0.3 */
    unsigned int num_reliable_keyfrms;   /* 2 */
    double redundant_obs_ratio_thr;      /* 0.9 */
    unsigned int top_n_covis_to_search;  /* 30 */
    unsigned int ba_first_iter, ba_second_iter; /* 5, 10 */
    float level_sigma_sq[SV_TR_MAX_LEVELS];
    /* hidden state */
    unsigned int n_fresh, cap_fresh;
    unsigned int* fresh;
    sv_rbtree* conn;          /* per keyframe: connected_keyfrms_and_num_shared_lms_ (std::map model) */
    unsigned int conn_cap;
    unsigned char* expired;   /* per keyframe id: its object has been destroyed (weak_ptr expired) */
    unsigned int expired_cap;
    unsigned int* lm_first_kf;
    unsigned int lm_first_cap;
    unsigned int next_landmark_id;
    /* allocation hook for new landmark records (NULL: calloc + table growth) */
    sv_tr_lm* (*alloc_lm)(void* user, unsigned int id);
    void* alloc_user;
    /* sv_system hooks (NULL in the harnesses):
     *   is_protected: keyframe::cannot_be_erased_ (set by the loop closer); prepare_for_erasing() is then a no-op
     *                 (the culled-keyframe trace still lists the id, like upstream);
     *   on_erase:     called inside prepare_for_erasing() once the keyframe's connections and spanning tree are
     *                 repaired and right before it leaves the map: map_db->replace_reference_keyframe(kf, parent)
     *                 and bow_db->erase_keyframe(kf). parent_id = get_spanning_parent() (SV_TR_NONE for none). */
    int (*is_protected)(void* user, unsigned int kf_id);
    void (*on_erase)(void* user, unsigned int kf_id, int parent_id);
    void* hook_user;
} sv_mapping;

void sv_mapping_init(sv_mapping* m, const sv_tr_config* cfg, sv_tr_map* map);
void sv_mapping_free(sv_mapping* m);
void sv_mapping_trace_free(sv_mapping_trace* t);

/* Mapping state helpers for the harness. */
void sv_mapping_set_lm_first_kf(sv_mapping* m, unsigned int lm_id, unsigned int first_kf);
/* Object lifetime: upstream keyframes are shared_ptr objects; an erased keyframe stays alive
 * (and comparable through its weak_ptr keys) until the last holder drops it. The lifetime
 * schedule is not derivable from the algorithm (it depends on the tracker and the loop
 * detector), so the caller injects it: expired != 0 from the moment the object is destroyed. */
void sv_mapping_set_expired(sv_mapping* m, unsigned int kf_id, int expired);
int sv_mapping_is_expired(const sv_mapping* m, unsigned int kf_id);
/* graph_node::get_covisibilities(): ordered ids without expired entries; weights as
 * get_num_shared_landmarks() reports them (a lookup in the connected map -- 0 when the map
 * cannot find the key, see sv_rbtree.h). Returns the count; arrays capacity >= kf->n_covis. */
unsigned int sv_mapping_covisibilities(sv_mapping* m, const sv_tr_kf* kf, unsigned int* ids, unsigned int* weights);
/* in-order dump of a keyframe's connected map: id (0xFFFFFFFF for an expired key) and count.
 * Returns the number of entries written (capacity = n_max). */
unsigned int sv_mapping_dump_conn(sv_mapping* m, unsigned int kf_id, unsigned int* ids, unsigned int* weights, unsigned int n_max);

/* mapping_module::mapping_with_new_keyframe() for keyframe `cur_id`, which
 * must already be in the map (sv_tr_create_new_keyframe registered it).
 * Returns 0 on success. */
int sv_mapping_step(sv_mapping* m, unsigned int cur_id, sv_mapping_trace* tr);

/* ---- pieces exposed for tests ---- */
/* solve::essential_solver::create_E_21(rot_1w, trans_1w, rot_2w, trans_2w) */
void sv_map_create_E_21(const double rot_1w[9], const double trans_1w[3], const double rot_2w[9],
                        const double trans_2w[3], double E[9]);
/* solve::triangulator::triangulate(bearing_1, bearing_2, Mat44 cam_pose_1, Mat44 cam_pose_2);
 * poses column-major 4x4 */
void sv_map_triangulate_poses(const double b1[3], const double b2[3], const double pose1[16],
                              const double pose2[16], double pos[3]);
/* keyframe::compute_median_depth(abs) */
float sv_map_kf_median_depth(const sv_tr_map* map, const sv_tr_kf* kf, int use_abs);
/* match::bow_tree(0.95, false)::match_for_triangulation; returns the number of matches,
 * pairs (idx_1, idx_2) ascending idx_1; *pairs_out malloc'ed (free by caller). */
unsigned int sv_map_match_for_triangulation(const sv_tr_config* cfg, sv_tr_kf* kf1, sv_tr_kf* kf2, const double E_12[9],
                                            float residual_rad_thr, unsigned int (**pairs_out)[2]);

#endif /* SV_MAPPING_H */
