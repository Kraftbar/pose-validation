/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port, modules M8 + M11: Handler, FeatureTracker, Frontend. See rd_sys.h. */
#include "rd_sys.h"
#include "rd_lie.h"
#include "rd_ransac.h"
#include "rd_imu_parsac.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NIL64 ((uint64_t)-1)

/* ---- FIFO of fixed-size elements (std::deque) ---- */
typedef struct q { unsigned char* p; size_t head, n, cap, sz; } q;
static void* q_at(const q* d, size_t i) { return d->p + (d->head + i) * d->sz; }
static void q_push(q* d, const void* e) {
    if (d->head + d->n == d->cap) {
        if (d->head > d->cap / 2) { memmove(d->p, d->p + d->head * d->sz, d->n * d->sz); d->head = 0; }
        else { d->cap = d->cap * 2 + 16; d->p = (unsigned char*)realloc(d->p, d->cap * d->sz); }
    }
    memcpy(d->p + (d->head + d->n) * d->sz, e, d->sz);
    d->n++;
}
static void q_pop(q* d) { d->head++; d->n--; if (!d->n) d->head = 0; }
static void q_clear(q* d) { d->head = 0; d->n = 0; }

typedef struct gyro { double t, w[3]; } gyro;
typedef struct accel { double t, a[3]; } accel;

struct rd_sys {
    rd_cfg cfg;
    /* Handler */
    q gyros, accels, imus, frames, frontal;   /* gyro, accel, rd_imu_sample, rd_frame*, rd_imu_sample */
    double latest_timestamp;
    rd_pose latest_pose;
    /* FeatureTracker */
    rd_map *ft, *keymap;
    int has_state;
    double st_t; rd_pose st_pose; rd_motion st_motion;
    /* Frontend */
    rd_init* init;
    rd_swt* swt;
    double fe_t; uint64_t fe_id; rd_pose fe_pose; rd_motion fe_motion;
    rd_swt_hooks parsac;                       /* the caller's estimators */
    rd_parsac_state ess;                       /* find_essential_matrix_parsac's static binConfidences */
    rd_parsac_state pnp;                       /* find_pnp_matrix_parsac_imu's static binConfidences */
    rd_pnp6_fn pnp6; void* pnp6_ctx;           /* solve_pnp_6pt (EPnP) for the native IMU-PARSAC */
    long parsac_grid_errors;
    const rd_sv_hooks* sv;
    rd_map_hooks user;
};

/* ---- map hooks: the keyframe map's marginalization, the user's event observer ---- */
static void hook_event(void* ctx, const rd_map_event* e) { rd_sys* s = (rd_sys*)ctx; if (s->user.event) s->user.event(s->user.ctx, e); }
static void hook_marginalize(void* ctx, rd_map* m, size_t index) { rd_sys* s = (rd_sys*)ctx; (void)m; if (s->swt) rd_swt_marginalize(s->swt, index); }

/* ---- the PARSAC estimators of module M10: the caller's, else the native essential PARSAC (module M3) ---- */
static void pnp_tramp(void* ctx, size_t n, const double* p3d, const double* p2d, const size_t* lens, const double Rcw[9],
                      const double tcw[3], double inv_f, char* mask) {
    rd_sys* s = (rd_sys*)ctx;
    double T[16];
    if (s->parsac.pnp_mask) { s->parsac.pnp_mask(s->parsac.ctx, n, p3d, p2d, lens, Rcw, tcw, inv_f, mask); return; }
    /* find_pnp_matrix_parsac_imu(P3D, P2D, lens, Rcw, tcw, 0.20, 1.0, mask, 1 / fx) with its defaults 0.999, 1000, seed 0 */
    if (rd_find_pnp_matrix_parsac_imu(&s->pnp, n, p3d, p2d, lens, Rcw, tcw, 0.20, 1.0, mask, inv_f, 0.999, 1000, 0, s->pnp6,
                                      s->pnp6_ctx, T) == (size_t)-1) {
        memset(mask, 1, n);                           /* a point outside the PARSAC grid: undefined behaviour in the C++ */
        s->parsac_grid_errors++;
    }
}
static void ess_tramp(void* ctx, size_t n, const double* p1, const double* p2, double threshold, char* mask) {
    rd_sys* s = (rd_sys*)ctx;
    double E[9];
    if (s->parsac.ess_mask) { s->parsac.ess_mask(s->parsac.ctx, n, p1, p2, threshold, mask); return; }
    /* find_essential_matrix_parsac(pts1, pts2, mask, threshold) with its defaults 0.999, 1000, seed 0 */
    if (rd_find_essential_matrix_parsac(&s->ess, n, p1, p2, mask, threshold, 0.999, 1000, 0, E) == (size_t)-1) {
        memset(mask, 1, n);                           /* a point outside the PARSAC grid: undefined behaviour in the C++ */
        s->parsac_grid_errors++;
    }
}

static void pose_of(const rd_frame* f, rd_pose* p) { p->q = f->pose_q; memcpy(p->p, f->pose_p, sizeof p->p); }
static void set_pose(rd_frame* f, const rd_pose* p) { f->pose_q = p->q; memcpy(f->pose_p, p->p, sizeof f->pose_p); }
static void predict(const rd_frame* fi, rd_frame* fj) {    /* PreIntegrator::predict(frame_i, frame_j) */
    rd_pose op, np;
    rd_motion nm;
    pose_of(fi, &op);
    rd_pi_predict(&fj->preint, &op, &fi->motion, &np, &nm);
    set_pose(fj, &np);
    fj->motion = nm;
}

rd_sys* rd_sys_create(const rd_cfg* cfg, const rd_swt_hooks* parsac, const rd_sv_hooks* sv, const rd_map_hooks* map) {
    rd_sys* s = (rd_sys*)calloc(1, sizeof *s);
    rd_map_hooks h;
    s->cfg = *cfg;
    s->gyros.sz = sizeof(gyro); s->accels.sz = sizeof(accel); s->imus.sz = sizeof(rd_imu_sample);
    s->frames.sz = sizeof(rd_frame*); s->frontal.sz = sizeof(rd_imu_sample);
    if (parsac) s->parsac = *parsac;
    s->parsac.stage = NULL;
    rd_parsac_state_init(&s->ess);
    rd_parsac_state_init(&s->pnp);
    s->sv = sv;
    if (map) s->user = *map;
    memset(&h, 0, sizeof h);
    h.ctx = s; h.event = hook_event; h.marginalize = hook_marginalize;
    rd_map_set_hooks(&h);
    s->ft = rd_map_new();                          /* FeatureTracker: map, keymap */
    s->keymap = rd_map_new();
    s->init = (rd_init*)malloc(sizeof(rd_init));   /* Frontend: the initializer (no map yet) */
    rd_init_create(s->init, &s->cfg);
    s->init->hooks = sv;
    s->fe_id = NIL64;
    s->fe_pose.q.w = 1.0;
    s->st_pose.q.w = 1.0;
    s->latest_pose.q.w = 1.0;
    return s;
}
void rd_sys_free(rd_sys* s) {
    size_t i;
    if (!s) return;
    for (i = 0; i < s->frames.n; ++i) rd_frame_free(*(rd_frame**)q_at(&s->frames, i));
    if (s->swt) { rd_swt_destroy(s->swt); free(s->swt); }
    if (s->init) { rd_init_destroy(s->init); free(s->init); }
    rd_map_free(s->keymap);
    rd_map_free(s->ft);
    free(s->gyros.p); free(s->accels.p); free(s->imus.p); free(s->frames.p); free(s->frontal.p);
    rd_map_set_hooks(NULL);
    free(s);
}

/* ------------------------------------------------------------------------------------------------------------------ Frontend */
static void frontend_issue(rd_sys* s, uint64_t id) {
    if (s->init) {
        rd_init_mirror_keyframe_map(s->init, s->ft, id);
        if (rd_init_initialize(s->init)) {
            double t; rd_pose p; rd_motion m;
            s->swt = (rd_swt*)malloc(sizeof(rd_swt));
            rd_swt_create(s->swt, rd_init_take_map(s->init), &s->cfg);
            memset(&s->swt->hooks, 0, sizeof s->swt->hooks);
            s->swt->hooks.ctx = s;
            if (s->parsac.pnp_mask || s->pnp6) s->swt->hooks.pnp_mask = pnp_tramp;
            s->swt->hooks.ess_mask = ess_tramp;
            s->swt->sv_hooks = s->sv;
            s->swt->ft = s->ft;
            rd_swt_latest_state(s->swt, &t, &p, &m);
            s->fe_t = t; s->fe_id = id; s->fe_pose = p; s->fe_motion = m;
            rd_init_destroy(s->init); free(s->init); s->init = NULL;
        }
    } else if (s->swt) {
        rd_swt_mirror_frame(s->swt, s->ft, id);
        if (rd_swt_track(s->swt)) {
            double t; rd_pose p; rd_motion m;
            rd_swt_latest_state(s->swt, &t, &p, &m);
            s->fe_t = t; s->fe_id = id; s->fe_pose = p; s->fe_motion = m;
        }                                             /* track() always returns true */
    }
}

/* ------------------------------------------------------------------------------------------------------------ FeatureTracker */
static int lk_cb(void* ctx, rd_frame* f, rd_frame* next, const double* curr, double* next_px, char* status, size_t n) {
    const rd_sys* s = (const rd_sys*)ctx;
    return rd_sys_image_track((const rd_sys_image*)f->image, (const rd_sys_image*)next->image, curr, next_px,
                              s->cfg.feature_tracker_predict_keypoints, status, n);
}
static int detect_cb(void* ctx, rd_frame* f, const double* px, size_t n, double** out, size_t* nout) {
    const rd_sys* s = (const rd_sys*)ctx;
    double* k = (double*)malloc(sizeof(double) * 2 * (n + 1));
    size_t nk = n;
    if (n) memcpy(k, px, sizeof(double) * 2 * n);
    if (!rd_sys_image_detect((rd_sys_image*)f->image, &k, &nk, s->cfg.feature_tracker_max_keypoint_detection,
                             s->cfg.feature_tracker_min_keypoint_distance)) { free(k); return 0; }
    *out = k; *nout = nk;
    return 1;
}
static void ft_run(rd_sys* s, rd_frame* frame) {
    rd_map* m = s->ft;
    const uint64_t lid = s->fe_id;
    const int is_init = lid != NIL64;
    const int tag = !is_init || frame->id % s->cfg.sliding_window_tracker_frequent == 0;
    rd_sys_image_preprocess((rd_sys_image*)frame->image, s->cfg.feature_tracker_clahe_clip_limit,
                            (int)s->cfg.feature_tracker_clahe_width, (int)s->cfg.feature_tracker_clahe_height);
    if (rd_map_frame_num(m) > 0) {
        rd_frame* last;
        rd_track_cfg tc;
        if (is_init) {
            const size_t idx = rd_map_frame_index_by_id(m, lid);
            if (idx != RD_NIL) {
                rd_frame* lf = rd_map_get_frame(m, idx);
                size_t j;
                set_pose(lf, &s->fe_pose);
                lf->motion = s->fe_motion;
                for (j = idx + 1; j < rd_map_frame_num(m); ++j) {
                    rd_frame* fi = rd_map_get_frame(m, j - 1);
                    rd_frame* fj = rd_map_get_frame(m, j);
                    rd_pi_integrate(&fj->preint, fj->data, (int)fj->ndata, fj->t, fi->motion.bg, fi->motion.ba, 0, 0);
                    predict(fi, fj);
                }
            } else {
                s->has_state = 0;                     /* "SWT cannot catch up." */
            }
        }
        last = rd_map_get_frame(m, rd_map_frame_num(m) - 1);
        if (last->ndata) {
            if (!frame->ndata || frame->data[0].t - last->t > 1.0e-5) {
                rd_imu_sample imu = last->data[last->ndata - 1];
                imu.t = last->t;
                rd_imu_list_insert(&frame->data, &frame->ndata, &frame->cdata, 0, &imu, 1);
            }
        }
        rd_pi_integrate(&frame->preint, frame->data, (int)frame->ndata, frame->t, last->motion.bg, last->motion.ba, 0, 0);
        frame->delta_q = frame->preint.delta.q;      /* what rd_frame_track_keypoints reads as next->preintegration.delta.q */
        tc.predict_keypoints = s->cfg.feature_tracker_predict_keypoints;
        tc.rotation_ransac_threshold = s->cfg.rotation_ransac_threshold;
        tc.rotation_misalignment_threshold = s->cfg.rotation_misalignment_threshold;
        tc.min_keypoint_distance = s->cfg.feature_tracker_min_keypoint_distance;
        rd_frame_track_keypoints(last, frame, &tc, lk_cb, s);
        if (is_init) {
            predict(last, frame);
            s->has_state = 1;
            s->st_t = frame->t; pose_of(frame, &s->st_pose); s->st_motion = frame->motion;
        }
        rd_sys_image_release_buffer((rd_sys_image*)last->image);
    }
    if (tag) rd_frame_detect_keypoints(frame, detect_cb, s);
    rd_map_attach_frame(m, frame, RD_NIL);
    while (rd_map_frame_num(m) > (is_init ? s->cfg.feature_tracker_max_frames : s->cfg.feature_tracker_max_init_frames) &&
           rd_map_get_frame(m, 0)->id < lid)
        rd_map_erase_frame(m, 0);
    if (tag) frontend_issue(s, rd_map_get_frame(m, rd_map_frame_num(m) - 1)->id);
}

/* ------------------------------------------------------------------------------------------------------------------- Handler */
static void propagate_state(double* st, rd_pose* pose, rd_motion* motion, double t, const double w[3], const double a[3]) {
    const double g[3] = {0, 0, -RD_GRAVITY_NOMINAL};
    const double dt = t - *st;
    double d[3], r[3], acc[3], wd[3];
    ok_quat e, qn;
    int i;
    for (i = 0; i < 3; ++i) d[i] = a[i] - motion->ba[i];
    rd_quat_rotate(&pose->q, d, r);
    for (i = 0; i < 3; ++i) acc[i] = g[i] + r[i];
    for (i = 0; i < 3; ++i) pose->p[i] = (pose->p[i] + dt * motion->v[i]) + ((0.5 * dt) * dt) * acc[i];
    rd_quat_rotate(&pose->q, d, r);
    for (i = 0; i < 3; ++i) motion->v[i] = motion->v[i] + dt * (g[i] + r[i]);
    for (i = 0; i < 3; ++i) wd[i] = (w[i] - motion->bg[i]) * dt;
    e = rd_expmap(wd);
    ok_quat_mul(&pose->q, &e, &qn);
    pose->q = ok_quat_normalized(qn);
    *st = t;
}
static void predict_pose(rd_sys* s, double t, rd_pose* out) {
    rd_pose o;
    if (s->has_state) {
        double st = s->st_t;
        rd_pose pose = s->st_pose;
        rd_motion motion = s->st_motion;
        double r[3];
        size_t i;
        while (s->frontal.n && ((rd_imu_sample*)q_at(&s->frontal, 0))->t <= st) q_pop(&s->frontal);
        for (i = 0; i < s->frontal.n; ++i) {
            const rd_imu_sample* imu = (const rd_imu_sample*)q_at(&s->frontal, i);
            if (imu->t <= t) propagate_state(&st, &pose, &motion, imu->t, imu->w, imu->a);
        }
        ok_quat_mul(&pose.q, &s->cfg.q_bo, &o.q);
        rd_quat_rotate(&pose.q, s->cfg.p_bo, r);
        for (i = 0; i < 3; ++i) o.p[i] = pose.p[i] + r[i];
    } else {
        memset(&o, 0, sizeof o);
    }
    if (out) *out = o;
}
static void track_imu(rd_sys* s, const rd_imu_sample* imu) {
    q_push(&s->frontal, imu);
    q_push(&s->imus, imu);
    while (s->imus.n && s->frames.n) {
        rd_imu_sample* front = (rd_imu_sample*)q_at(&s->imus, 0);
        rd_frame* f = *(rd_frame**)q_at(&s->frames, 0);
        if (front->t <= f->t) {
            rd_imu_list_insert(&f->data, &f->ndata, &f->cdata, f->ndata, front, 1);
            q_pop(&s->imus);
        } else {
            q_pop(&s->frames);
            ft_run(s, f);
        }
    }
}
void rd_sys_track_gyroscope(rd_sys* s, double t, double x, double y, double z, rd_pose* out) {
    gyro g;
    if (s->accels.n) {
        if (t < ((accel*)q_at(&s->accels, 0))->t) {
            q_clear(&s->gyros);
        } else {
            while (s->accels.n && t >= ((accel*)q_at(&s->accels, 0))->t) {
                const accel acc = *(accel*)q_at(&s->accels, 0);
                const gyro* g0 = (const gyro*)q_at(&s->gyros, 0);
                const double lambda = (acc.t - g0->t) / (t - g0->t);
                const double xyz[3] = {x, y, z};
                rd_imu_sample imu;
                int i;
                imu.t = acc.t;
                for (i = 0; i < 3; ++i) { imu.w[i] = g0->w[i] + lambda * (xyz[i] - g0->w[i]); imu.a[i] = acc.a[i]; }
                track_imu(s, &imu);
                q_pop(&s->accels);
            }
            if (s->accels.n)
                while (s->gyros.n && ((gyro*)q_at(&s->gyros, 0))->t < t) q_pop(&s->gyros);
        }
    }
    g.t = t; g.w[0] = x; g.w[1] = y; g.w[2] = z;
    q_push(&s->gyros, &g);
    predict_pose(s, t, out);
}
void rd_sys_track_accelerometer(rd_sys* s, double t, double x, double y, double z, rd_pose* out) {
    if (s->gyros.n && t >= ((gyro*)q_at(&s->gyros, 0))->t) {
        const gyro* back = (const gyro*)q_at(&s->gyros, s->gyros.n - 1);
        rd_imu_sample imu;
        int i;
        imu.t = t; imu.a[0] = x; imu.a[1] = y; imu.a[2] = z;
        if (t > back->t) {
            accel a;
            while (s->gyros.n > 1) q_pop(&s->gyros);
            a.t = t; a.a[0] = x; a.a[1] = y; a.a[2] = z;
            q_push(&s->accels, &a);
        } else if (t == back->t) {
            while (s->gyros.n > 1) q_pop(&s->gyros);
            memcpy(imu.w, ((gyro*)q_at(&s->gyros, 0))->w, sizeof imu.w);
            track_imu(s, &imu);
        } else {                                      /* gyroscopes.front().t <= t < gyroscopes.back().t */
            const gyro *g0, *g1;
            double lambda;
            while (t >= ((gyro*)q_at(&s->gyros, 1))->t) q_pop(&s->gyros);
            g0 = (const gyro*)q_at(&s->gyros, 0); g1 = (const gyro*)q_at(&s->gyros, 1);
            lambda = (t - g0->t) / (g1->t - g0->t);
            for (i = 0; i < 3; ++i) imu.w[i] = g0->w[i] + lambda * (g1->w[i] - g0->w[i]);
            track_imu(s, &imu);
        }
    }
    predict_pose(s, t, out);
}
void rd_sys_track_camera(rd_sys* s, rd_sys_image* image, rd_pose* out) {
    rd_frame* f = rd_frame_new();
    rd_pose o;
    memcpy(f->K, s->cfg.K, sizeof f->K);
    f->image = &image->base;
    f->t = image->base.t;
    f->sqrt_inv_cov[0] = f->K[0]; f->sqrt_inv_cov[1] = f->K[1]; f->sqrt_inv_cov[2] = f->K[3]; f->sqrt_inv_cov[3] = f->K[4];
    f->sqrt_inv_cov[0] /= sqrt(s->cfg.keypoint_noise_cov[0]);
    f->sqrt_inv_cov[3] /= sqrt(s->cfg.keypoint_noise_cov[3]);
    f->cam_q = s->cfg.q_bc; memcpy(f->cam_p, s->cfg.p_bc, sizeof f->cam_p);
    f->imu_q = s->cfg.q_bi; memcpy(f->imu_p, s->cfg.p_bi, sizeof f->imu_p);
    memcpy(f->preint.cov_a, s->cfg.cov_a, sizeof f->preint.cov_a);
    memcpy(f->preint.cov_w, s->cfg.cov_g, sizeof f->preint.cov_w);
    memcpy(f->preint.cov_ba, s->cfg.cov_ba, sizeof f->preint.cov_ba);
    memcpy(f->preint.cov_bg, s->cfg.cov_bg, sizeof f->preint.cov_bg);
    q_push(&s->frames, &f);
    predict_pose(s, image->base.t, &o);
    if (image->base.t > s->latest_timestamp) { s->latest_pose = o; s->latest_timestamp = image->base.t; }
    if (out) *out = o;
}

long rd_sys_parsac_grid_errors(const rd_sys* s) { return s->parsac_grid_errors; }
void rd_sys_set_pnp_solver(rd_sys* s, rd_pnp6_fn solve, void* ctx) { s->pnp6 = solve; s->pnp6_ctx = ctx; }
int rd_sys_state(const rd_sys* s) { return s->init ? RD_SYS_INITIALIZING : s->swt ? RD_SYS_TRACKING : RD_SYS_UNKNOWN; }
void rd_sys_latest_state(const rd_sys* s, double* t, rd_pose* pose) {
    if (s->has_state) { *t = s->st_t; *pose = s->st_pose; }
    else { *t = 0.0; memset(pose, 0, sizeof *pose); }
}
