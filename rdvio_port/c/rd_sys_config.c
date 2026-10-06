/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port, module M11: configuration. See rd_sys_config.h. */
#include "rd_sys_config.h"
#include "rd_yaml.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct rd { char* err; size_t errlen; int bad; } rd;
static void fail(rd* r, const char* what, const char* key) {
    if (!r->bad && r->err) snprintf(r->err, r->errlen, "%s: %s", what, key);
    r->bad = 1;
}
static int num(const void* node, double* out) {
    const char* s = rd_yaml_scalar(node);
    char* e;
    if (!s || !*s) return 0;
    *out = strtod(s, &e);
    while (*e && isspace((unsigned char)*e)) e++;
    return *e == 0;
}
/* optional (slam) or required (device) scalar */
static void get_d(rd* r, const void* root, const char* key, double* v, int required) {
    const void* n = rd_yaml_find(root, key);
    if (!n) { if (required) fail(r, "missing", key); return; }
    if (!num(n, v)) fail(r, "not a number", key);
}
static void get_z(rd* r, const void* root, const char* key, size_t* v, int required) {
    const void* n = rd_yaml_find(root, key);
    const char* s = n ? rd_yaml_scalar(n) : NULL;
    char* e;
    if (!n) { if (required) fail(r, "missing", key); return; }
    if (!s) { fail(r, "not an integer", key); return; }
    *v = (size_t)strtoull(s, &e, 10);
    if (*e) fail(r, "not an integer", key);
}
static void get_b(rd* r, const void* root, const char* key, int* v) {
    const void* n = rd_yaml_find(root, key);
    const char* s = n ? rd_yaml_scalar(n) : NULL;
    char w[8];
    size_t i;
    if (!n) return;
    if (!s) { fail(r, "not a boolean", key); return; }
    for (i = 0; i + 1 < sizeof w && s[i]; ++i) w[i] = (char)tolower((unsigned char)s[i]);
    w[i] = 0;
    if (!strcmp(w, "true") || !strcmp(w, "yes") || !strcmp(w, "on") || !strcmp(w, "y")) *v = 1;
    else if (!strcmp(w, "false") || !strcmp(w, "no") || !strcmp(w, "off") || !strcmp(w, "n")) *v = 0;
    else fail(r, "not a boolean", key);
}
static void get_v(rd* r, const void* root, const char* key, double* v, int n, int required) {
    const void* node = rd_yaml_find(root, key);
    int i;
    if (!node) { if (required) fail(r, "missing", key); return; }
    if (rd_yaml_seq_len(node) != n) { fail(r, "wrong length", key); return; }
    for (i = 0; i < n; ++i) if (!num(rd_yaml_seq_at(node, i), &v[i])) fail(r, "not a number in", key);
}
/* assign_matrix: mat(i, j) = node[i * cols + j] (row-major list into a column-major matrix) */
static void get_m(rd* r, const void* root, const char* key, double* m, int rows, int cols) {
    double v[16];
    int i, j;
    get_v(r, root, key, v, rows * cols, 1);
    for (i = 0; i < rows; ++i) for (j = 0; j < cols; ++j) m[i + rows * j] = v[i * cols + j];
}
static void get_q(rd* r, const void* root, const char* key, ok_quat* q, int required) {
    double v[4] = {q->x, q->y, q->z, q->w};
    get_v(r, root, key, v, 4, required);
    q->x = v[0]; q->y = v[1]; q->z = v[2]; q->w = v[3];
}

int rd_cfg_load(const char* slam_yaml, const char* device_yaml, rd_cfg* c, char* err, size_t errlen) {
    void* slam = rd_yaml_load(slam_yaml);
    void* dev = rd_yaml_load(device_yaml);
    rd r;
    double v[4];
    memset(c, 0, sizeof *c);
    r.err = err; r.errlen = errlen; r.bad = 0;
    if (err && errlen) err[0] = 0;
    if (!slam || !dev) { fail(&r, "cannot read", !slam ? slam_yaml : device_yaml); rd_yaml_free(slam); rd_yaml_free(dev); return -1; }
    /* Config defaults (rdvio/src/config.cpp) */
    c->q_bc.w = 1; c->q_bi.w = 1; c->q_bo.w = 1;
    c->sliding_window_size = 10; c->sliding_window_subframe_size = 3; c->sliding_window_force_keyframe_landmarks = 35;
    c->sliding_window_tracker_frequent = 1;
    c->feature_tracker_min_keypoint_distance = 20.0; c->feature_tracker_max_keypoint_detection = 150;
    c->feature_tracker_max_init_frames = 60; c->feature_tracker_max_frames = 200;
    c->feature_tracker_clahe_clip_limit = 6.0; c->feature_tracker_clahe_width = 8; c->feature_tracker_clahe_height = 8;
    c->feature_tracker_predict_keypoints = 1;
    c->initializer_keyframe_num = 8; c->initializer_keyframe_gap = 5; c->initializer_min_matches = 50;
    c->initializer_min_parallax = 10; c->initializer_min_triangulation = 50; c->initializer_min_landmarks = 30;
    c->initializer_refine_imu = 1;
    c->solver_iteration_limit = 10; c->solver_time_limit = 1.0e6;
    c->rotation_misalignment_threshold = 0.1; c->rotation_ransac_threshold = 10;
    c->random = 648;
    c->parsac_flag = 0; c->parsac_dynamic_probability = 0.0; c->parsac_threshold = 3.0; c->parsac_norm_scale = 1.0;
    c->parsac_keyframe_check_size = 3;
    /* device */
    get_v(&r, dev, "cam0.intrinsics", v, 4, 1);
    c->K[0] = v[0]; c->K[4] = v[1]; c->K[6] = v[2]; c->K[7] = v[3]; c->K[8] = 1.0;   /* setIdentity, then (0,0) (1,1) (0,2) (1,2) */
    get_v(&r, dev, "cam0.distortion", c->distortion, 4, 1);
    get_z(&r, dev, "cam0.camera_distortion_flag", &c->camera_distortion_flag, 1);
    {   /* read by the driver, not by rdvio::Config */
        const char* m = rd_yaml_scalar(rd_yaml_find(dev, "cam0.distortion_model"));
        c->distortion_equidistant = m && !strcmp(m, "equidistant");
    }
    get_d(&r, dev, "cam0.time_offset", &c->camera_time_offset, 1);
    { double res[2] = {0, 0}; get_v(&r, dev, "cam0.resolution", res, 2, 1); c->resolution[0] = (int)res[0]; c->resolution[1] = (int)res[1]; }
    get_q(&r, dev, "cam0.extrinsic.q_bc", &c->q_bc, 1);
    get_v(&r, dev, "cam0.extrinsic.p_bc", c->p_bc, 3, 1);
    get_m(&r, dev, "cam0.noise", c->keypoint_noise_cov, 2, 2);
    get_q(&r, dev, "imu.extrinsic.q_bi", &c->q_bi, 1);
    get_v(&r, dev, "imu.extrinsic.p_bi", c->p_bi, 3, 1);
    get_m(&r, dev, "imu.noise.cov_g", c->cov_g, 3, 3);
    get_m(&r, dev, "imu.noise.cov_a", c->cov_a, 3, 3);
    get_m(&r, dev, "imu.noise.cov_bg", c->cov_bg, 3, 3);
    get_m(&r, dev, "imu.noise.cov_ba", c->cov_ba, 3, 3);
    /* slam */
    get_q(&r, slam, "output.q_bo", &c->q_bo, 0);
    get_v(&r, slam, "output.p_bo", c->p_bo, 3, 0);
    get_z(&r, slam, "sliding_window.size", &c->sliding_window_size, 0);
    get_z(&r, slam, "sliding_window.subframe_size", &c->sliding_window_subframe_size, 0);
    get_z(&r, slam, "sliding_window.force_keyframe_landmarks", &c->sliding_window_force_keyframe_landmarks, 0);
    get_z(&r, slam, "sliding_window.tracker_frequent", &c->sliding_window_tracker_frequent, 0);
    get_d(&r, slam, "feature_tracker.min_keypoint_distance", &c->feature_tracker_min_keypoint_distance, 0);
    get_z(&r, slam, "feature_tracker.max_keypoint_detection", &c->feature_tracker_max_keypoint_detection, 0);
    get_z(&r, slam, "feature_tracker.max_init_frames", &c->feature_tracker_max_init_frames, 0);
    get_z(&r, slam, "feature_tracker.max_frames", &c->feature_tracker_max_frames, 0);
    get_d(&r, slam, "feature_tracker.clahe_clip_limit", &c->feature_tracker_clahe_clip_limit, 0);
    get_z(&r, slam, "feature_tracker.clahe_width", &c->feature_tracker_clahe_width, 0);
    get_z(&r, slam, "feature_tracker.clahe_height", &c->feature_tracker_clahe_height, 0);
    get_b(&r, slam, "feature_tracker.predict_keypoints", &c->feature_tracker_predict_keypoints);
    get_z(&r, slam, "initializer.keyframe_num", &c->initializer_keyframe_num, 0);
    get_z(&r, slam, "initializer.keyframe_gap", &c->initializer_keyframe_gap, 0);
    get_z(&r, slam, "initializer.min_matches", &c->initializer_min_matches, 0);
    get_d(&r, slam, "initializer.min_parallax", &c->initializer_min_parallax, 0);
    get_z(&r, slam, "initializer.min_triangulation", &c->initializer_min_triangulation, 0);
    get_z(&r, slam, "initializer.min_landmarks", &c->initializer_min_landmarks, 0);
    get_b(&r, slam, "initializer.refine_imu", &c->initializer_refine_imu);
    get_z(&r, slam, "solver.iteration_limit", &c->solver_iteration_limit, 0);
    get_d(&r, slam, "solver.time_limit", &c->solver_time_limit, 0);
    get_b(&r, slam, "parsac.parsac_flag", &c->parsac_flag);
    get_d(&r, slam, "parsac.dynamic_probability", &c->parsac_dynamic_probability, 0);
    get_d(&r, slam, "parsac.threshold", &c->parsac_threshold, 0);
    get_d(&r, slam, "parsac.norm_scale", &c->parsac_norm_scale, 0);
    get_z(&r, slam, "parsac.keyframe_check_size", &c->parsac_keyframe_check_size, 0);
    get_d(&r, slam, "rotation.misalignment_threshold", &c->rotation_misalignment_threshold, 0);
    get_d(&r, slam, "rotation.ransac_threshold", &c->rotation_ransac_threshold, 0);
    rd_yaml_free(slam); rd_yaml_free(dev);
    return r.bad ? -1 : 0;
}
