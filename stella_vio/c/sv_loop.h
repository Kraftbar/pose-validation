/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 * Full BSD-2 notice text in sv_loop.c. */
#ifndef SV_LOOP_H
#define SV_LOOP_H

#include "sv_bow_db.h"
#include "sv_mapping.h"
#include "sv_sim3.h"

/* Module 7 (loop closing) of the stella_vslam port, monocular / perspective, DETERMINISTIC build,
 * synchronous single-threaded:
 *   module::loop_detector           (detect_loop_candidates, find_continuously_detected_keyframe_sets,
 *                                    compute_min_score_in_covisibilities, validate_candidates,
 *                                    select_loop_candidate_via_Sim3, final projection re-search)
 *   match::projection               (match_frame_and_keyframe(pose), match_by_Sim3_transform,
 *                                    match_keyframes_mutually)
 *   optimize::transform_optimizer   (sv_g2o_sim3.h)
 *   global_optimization_module      (correct_loop and its helpers)
 *   optimize::graph_optimizer       (sv_g2o_sim3.h pose graph)
 *   module::loop_bundle_adjuster    (optimize: global BA + spanning-tree pose propagation + landmark update)
 * Upstream: external/candidates/stella_vslam/src/stella_vslam/{module/loop_detector.cc,
 * global_optimization_module.cc, module/loop_bundle_adjuster.cc, optimize/{graph_optimizer,
 * transform_optimizer,global_bundle_adjuster}.cc, match/projection.cc, match/fuse.cc,
 * data/graph_node.cc}.
 *
 * The data model is sv_track.h's map (sv_tr_map) plus module 6's sv_mapping context (covisibility maps,
 * landmark helpers). What snapshots do not expose lives in sv_loop: cont_detected_keyfrm_sets_,
 * prev_loop_correct_keyfrm_id_, loop edges, the has_representative_descriptor / has_valid_prediction_parameters
 * cache flags of the landmarks. The C library never prints: it reports every event of the reference trace
 * (patch 0013, util/loop_trace.h) through the `trace` callback so that the harness can diff the exact text.
 *
 * NOT ported / injected (see HANDOVER "Module 7"): solve::pnp_solver (Codex's relocalization leaf, reserved) --
 * the RANSAC result is injected through `pnp`; tracker_->replace_landmarks_in_last_frm(); keyframe erasure
 * flags (set_not_to_be_erased); the tree SHAPE of connected maps that contain expired keys. */

#define SV_LOOP_NONE 0xFFFFFFFFu

/* one typed field of a trace event */
typedef struct sv_tf {
    char k;               /* 'u' unsigned, 'i' signed, 'd' double, 'L' id list, 'A' keypoint -> landmark association,
                           'T' triples a:b:c (3n ints), 'P' pairs a:b (2n ints) */
    long i;               /* 'u' / 'i' */
    double d;             /* 'd' */
    const int* l;         /* 'L' ids, 'A' per-keypoint landmark id (-1 = none) */
    unsigned int n;       /* 'L' / 'A' length */
} sv_tf;

typedef void (*sv_loop_trace_fn)(void* user, const char* tag, const sv_tf* f, int nf);

typedef struct sv_loop_pnp {
    int valid;
    double pose_rm[16];              /* best_cam_pose, row-major Mat44 */
    unsigned int n_inliers;
    const unsigned int* inliers;     /* indices into valid_indices */
} sv_loop_pnp;

/* solve::pnp_solver::{find_via_ransac, solution_is_valid, get_best_cam_pose, get_inlier_flags} for the
 * candidate `cand_id`; `n_valid` is the size of the valid-index list the solver was built with. Returns 0 on
 * success (result stays owned by the callee until the next call). */
typedef int (*sv_loop_pnp_fn)(void* user, unsigned int cand_id, unsigned int n_valid, sv_loop_pnp* out);

/* solve::pnp_solver(valid_bearings, octaves, valid_points, scale_factors, 10, use_fixed_seed) followed by
 * find_via_ransac(30, false), with the solver's inputs (sv_system wires Codex's sv_pnp_ransac here). bearings /
 * points: n_valid * 3 doubles, octaves: n_valid ints. Same result convention as sv_loop_pnp_fn. */
typedef int (*sv_loop_pnp_ransac_fn)(void* user, const double* bearings, const double* points, const int* octaves,
                                     unsigned int n_valid, sv_loop_pnp* out);

typedef struct sv_loop_set {
    unsigned int* ids;               /* ascending, SV_LOOP_NONE last */
    unsigned int n;
    unsigned int lead;               /* lead_keyfrm_ id */
    unsigned int continuity;
} sv_loop_set;

typedef struct sv_loop {
    const sv_tr_config* cfg;
    sv_mapping* mp;
    sv_tr_map* map;
    /* parameters */
    unsigned int num_final_matches_thr;      /* 40 */
    unsigned int min_continuity;             /* 3 */
    int reject_by_graph_distance;            /* 0 */
    int min_distance_on_graph;               /* 50 */
    unsigned int num_matches_thr;            /* 20 */
    unsigned int num_matches_thr_brute_force;/* 0 (not supported) */
    unsigned int num_optimized_inliers_thr;  /* 20 */
    unsigned int top_n_covisibilities_to_search; /* 0 */
    float num_common_words_thr_ratio;        /* 0.8f */
    unsigned int thr_opt1, thr_a, thr_b;     /* 10, 25, 40 (hard-coded upstream; env override in the reference) */
    unsigned int thr_neighbor_keyframes;     /* GlobalOptimizer.thr_neighbor_keyframes = 15 */
    unsigned int min_num_shared_lms_graph;   /* GraphOptimizer.min_num_shared_lms = 100 */
    unsigned int loop_ba_num_iter;           /* GlobalOptimizer.num_iter = 10 */
    /* state */
    sv_loop_set* prev;
    unsigned int n_prev;
    unsigned int prev_loop_correct_keyfrm_id;
    unsigned int* loop_edges_n;              /* per keyframe id */
    unsigned int** loop_edges;
    unsigned int edges_cap;
    unsigned char* flag_desc;                /* per landmark id: has_representative_descriptor_ */
    unsigned char* flag_pred;                /* has_valid_prediction_parameters_ */
    unsigned int flag_cap;
    sv_bow_vector* bow;                      /* per keyframe id (lazily computed) */
    unsigned char* bow_ready;
    unsigned int bow_cap;
    /* per-step results */
    unsigned int* to_validate;               /* loop_candidates_to_validate_ (ascending ids) */
    unsigned int n_to_validate;
    int selected;                            /* selected_candidate_ id or -1 */
    sv_sim3 sim3_world_to_curr;
    int* match_cand;                         /* curr_match_lms_observed_in_cand_ (per keypoint, -1 none) */
    unsigned int n_match_cand;
    unsigned int* match_covis;               /* curr_match_lms_observed_in_cand_covis_ */
    unsigned int n_match_covis;
    /* hooks */
    sv_loop_trace_fn trace;
    void* trace_user;
    sv_loop_pnp_fn pnp;
    void* pnp_user;
    sv_loop_pnp_ransac_fn pnp_ransac; /* preferred over `pnp` when set (autonomous run) */
    void* pnp_ransac_user;
    /* tracker_->replace_landmarks_in_last_frm(replaced_lms): (from id, to id) pairs, ascending `from`, once per
     * correct_loop() after the duplicated landmarks were resolved */
    void (*replaced_hook)(void* user, const int* pairs, unsigned int n_pairs);
    void* replaced_user;
    void (*hook)(void* user, int phase); /* 1 = after the pose graph optimization (before the loop BA) */
    void* hook_user;
    /* Optional persistent BoW database (the system keeps one anyway): when `ext_db` is set, sv_loop_detect queries it
     * directly instead of rebuilding a database from `db_ids`. `ext_dbk(user, id)` returns the database entry of
     * keyframe `id`, or NULL when it is not registered. The content must equal `db_ids` (same candidate set). */
    sv_bow_db* ext_db;
    const sv_bow_db_keyframe* (*ext_dbk)(void* user, unsigned int id);
    void* ext_db_user;
    /* stella_vio: map merge. The reference refuses a loop whose two keyframes lie in different spanning trees ("merge two spanning
     * trees: not yet implemented"). With merge_maps = 1 the tree of the current keyframe is moved into the candidate's frame by the
     * Sim3 of the loop (all its keyframes and landmarks), hooked below its candidate, and the ordinary correct_loop() then fuses the
     * duplicated landmarks and runs the pose graph + loop BA over the joined tree. */
    int merge_maps;
    void (*merge_hook)(void* user, unsigned int cur_id, unsigned int cand_id, double new_to_old_scale);
    void* merge_user;
    int merged_last;                         /* the last sv_loop_correct() merged two trees */
    unsigned int n_merges;
    /* diagnostics */
    unsigned int n_expired_in_loop;          /* expired connected-map keys met while correcting a loop (tree shape unknown) */
    unsigned int n_stale_covis;              /* covisibility entries of erased-but-not-destroyed keyframes (skipped) */
    unsigned int n_dup_connect;
} sv_loop;

void sv_loop_init(sv_loop* L, const sv_tr_config* cfg, sv_mapping* mp);
void sv_loop_free(sv_loop* L);
/* loop edges of a keyframe (snapshot input) */
void sv_loop_set_loop_edges(sv_loop* L, unsigned int kf, const unsigned int* ids, unsigned int n);
unsigned int sv_loop_get_loop_edges(const sv_loop* L, unsigned int kf, const unsigned int** ids);
/* cont_detected_keyfrm_sets_ (teacher-forced from the previous step's trace) */
void sv_loop_clear_prev(sv_loop* L);
void sv_loop_add_prev(sv_loop* L, unsigned int lead, unsigned int continuity, const unsigned int* ids, unsigned int n);

/* loop_detector::detect_loop_candidates() for keyframe `cur_id`, BoW database content = db_ids (ascending).
 * The current keyframe is added to the (rebuilt) database afterwards like upstream; the caller rebuilds the
 * database for every step, so nothing is kept. Returns 1 iff loop_candidates_to_validate_ is non-empty. */
int sv_loop_detect(sv_loop* L, unsigned int cur_id, const unsigned int* db_ids, unsigned int n_db);

/* loop_detector::validate_candidates(): 1 iff a loop was selected (L->selected, L->sim3_world_to_curr,
 * L->match_cand, L->match_covis are then set) */
int sv_loop_validate(sv_loop* L, unsigned int cur_id);

/* global_optimization_module::correct_loop() for the selected candidate, including the pose graph and the
 * loop BA (BA optimizer inputs are read from / results written to the map). Returns 0 on success. */
int sv_loop_correct(sv_loop* L, unsigned int cur_id);

#endif /* SV_LOOP_H */
