/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_BUNDLE_ADJUSTER_H
#define SV_BUNDLE_ADJUSTER_H

#include "sv_g2o_ba.h"

/* stella_vslam's bundle-adjustment orchestration on top of sv_g2o_ba.h --
 * BSD (stella-vslam BSD-2, AIST 2019 / stella-cv 2022; g2o BSD-2 notice):
 *   external/candidates/stella_vslam/src/stella_vslam/optimize/
 *     local_bundle_adjuster_g2o.cc   (optimize)
 *     global_bundle_adjuster.cc      (optimize_impl, optimize_for_initialization,
 *                                     optimize)
 *     internal/se3/shot_vertex_container.h, internal/landmark_vertex_container.h
 *     internal/se3/reproj_edge_wrapper.h
 *
 * The map model of this port (sv_map.h) does not yet carry covisibility
 * graphs, landmark observation lists or keypoint arrays, so the
 * orchestration consumes a read-only *map view*: exactly the data the
 * upstream functions read from data::keyframe / data::landmark while they run
 * (keyframe id/erased/spanning-root/pose, per-index landmark slots,
 * per-observation undistorted keypoint + octave, landmark id/erased/position
 * and its observation list in the map's id order, the camera intrinsics and
 * the ORB inverse level sigma^2 table). It returns the results instead of
 * mutating the map: optimized keyframe poses (Mat44, what set_pose_cw()
 * receives), optimized landmark positions and the list of outlier
 * observations (keyframe id, landmark id) that the caller must erase
 * (`keyfrm->erase_landmark(lm); lm->erase_observation(...)`, then
 * `compute_descriptor` / `update_mean_normal_and_obs_scale_variance`).
 * Vertex / edge creation order reproduces upstream exactly, including the
 * libstdc++ unordered_map iteration order of the local keyframe / landmark
 * containers (sv_umap_order.h).
 */

typedef struct sv_bav_kp {
    unsigned int idx;
    float x, y;
    int octave;
} sv_bav_kp;

typedef struct sv_bav_kf {
    unsigned int id;
    int erased;
    int spanning_root;
    double pose_cw[16]; /* row-major Mat44 */
    int has_slots;
    int n_slots;
    const unsigned int* slots; /* landmark id per keypoint index, 0xFFFFFFFF = none */
    int n_kp;
    const sv_bav_kp* kps;      /* only the indices referenced by observations, ascending idx */
} sv_bav_kf;

typedef struct sv_bav_lm {
    unsigned int id;
    int erased;
    double pos[3];
    int n_obs;
    const unsigned int* obs_kf; /* observation list in map (keyframe-id) order */
    const unsigned int* obs_idx;
} sv_bav_lm;

typedef struct sv_bav_view {
    const sv_bav_kf* kfs; /* ascending id */
    int n_kfs;
    const sv_bav_lm* lms; /* ascending id */
    int n_lms;
    double fx, fy, cx, cy;
    int n_isq;
    const float* isq; /* orb_params_->inv_level_sigma_sq_ */
    unsigned int fixed_keyframe_id_threshold;
} sv_bav_view;

typedef struct sv_bav_stage {
    unsigned int requested_iters;
    int returned_iters;
    unsigned int flag_after;
    int stopped_by_terminate;
    int n_levels;
    unsigned char* levels; /* edge level at the start of the stage */
    int n_iters;
    sv_ba_iter* iters;
} sv_bav_stage;

typedef struct sv_bav_result {
    sv_ba_graph g;             /* the graph as built and finally optimized */
    sv_ba_vertex* v0;          /* vertices / edges exactly as built (before any optimization) */
    sv_ba_edge* e0;
    int nv0, ne0;
    unsigned int* vtx_owner;   /* per vertex: keyframe id / landmark id */
    unsigned int* edge_kf_id;  /* per edge */
    unsigned int* edge_lm_id;
    unsigned int* edge_idx;
    int n_stages;
    sv_bav_stage stages[2];
    int n_outliers;
    unsigned int (*outliers)[2]; /* (keyframe id, landmark id) */
    int n_applied;
    unsigned int* applied_kf;
    double (*applied_pose)[16]; /* row-major Mat44 handed to set_pose_cw() */
    int n_opt_lm;               /* global BA: landmarks whose vertex was kept */
    unsigned int* opt_lm;
    int ran_optimize;
    int returned_ok;
} sv_bav_result;

void sv_bav_result_free(sv_bav_result* r);

/* util::converter::to_g2o_SE3 (row-major Mat44 -> SE3Quat, incl. normalizeRotation)
 * and converter::to_eigen_mat(SE3Quat). */
void sv_bav_pose_from_mat44(const double m[16], sv_se3* out);
void sv_bav_mat44_from_pose(const sv_se3* pose, double m[16]);

/* local_bundle_adjuster_g2o::optimize(map_db, curr_keyfrm, force_stop_flag).
 * covis: curr_keyfrm->graph_node_->get_covisibilities() ids in that order.
 * force_stop_flag may be NULL. Returns 0 (result->ran_optimize == 0 if the
 * function returned before the first optimization because the flag was
 * already set). */
int sv_bav_local(const sv_bav_view* view, unsigned int curr_id, const unsigned int* covis, int n_covis,
                 unsigned int num_first_iter, unsigned int num_second_iter, int use_additional_keyframes,
                 int* force_stop_flag, sv_bav_result* out);

/* global_bundle_adjuster::optimize_for_initialization(keyfrms, lms, {}, gain_threshold,
 * fix_markers, force_stop_flag) with num_iter / use_huber_kernel from the constructor.
 * keyfrm_ids / lm_ids: the vectors passed by the caller (0xFFFFFFFF = null). */
int sv_bav_global_init(const sv_bav_view* view, const unsigned int* keyfrm_ids, int n_keyfrms,
                       const unsigned int* lm_ids, int n_lms, unsigned int num_iter, int use_huber,
                       double gain_threshold, int* force_stop_flag, sv_bav_result* out);

/* global_bundle_adjuster::optimize(keyfrms, ...) (loop BA): the landmark list
 * is derived from the keyframes' landmark slots like upstream. */
int sv_bav_global_loop(const sv_bav_view* view, const unsigned int* keyfrm_ids, int n_keyfrms,
                       unsigned int num_iter, int use_huber, int* force_stop_flag, sv_bav_result* out);

#endif /* SV_BUNDLE_ADJUSTER_H */
