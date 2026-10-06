/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 8: the system driver (see ok_system.h for scope and licence). */
#include "ok_system.h"
#include "ok_brisk.h"
#include "ok_cam.h"
#include "ok_imu.h"
#include "ok_kin.h"
#include <stdlib.h>
#include <string.h>

#define IMU_TEMPORAL_OVERLAP 0.02         /* ThreadedSlam.cpp imuTemporalOverlap */

struct ok_sys {
    ok_cfg cfg;
    ok_vsb* b; int own_b;
    ok_fe* fe;
    ok_fe_est est;
    ok_sys_be be;
    ok_brisk_context* brisk;
    double kptradius;
    unsigned char hdr[OK_CFG_MAXCAM][12 + 32 + 4 + 8 * OK_CAM_MAX_DIST]; size_t hlen[OK_CFG_MAXCAM];   /* ok_cam.h header */
    float* rays[OK_CFG_MAXCAM]; float* jacs[OK_CFG_MAXCAM]; int maps_ready;
    ok_imu_meas* q; size_t qhead, qn, qcap;        /* imuMeasurementsReceived_ (qhead .. qn-1 pending) */
    ok_imu_meas* dq; size_t dn, dcap;              /* imuMeasurementDeque_ */
    int first_frame;
    int last_init; double last_T[7], last_sb[9]; ok_time last_t;   /* lastOptimisedState_ */
    ok_sys_publish_fn publish; void* publish_ctx;
};

/* ------------------------------------------------------------------ direct backend calls (no harness) */
static uint64_t d_add_landmark(void* c, const double hp[4], int init) { return ok_vsb_add_landmark((ok_vsb*)c, hp, init); }
static int d_add_observation(void* c, uint64_t lm, uint64_t st, uint32_t cam, uint32_t kp, int cauchy) { return ok_vsb_add_observation((ok_vsb*)c, lm, st, cam, kp, cauchy); }
static int d_remove_observation(void* c, uint64_t st, uint32_t cam, uint32_t kp) { return ok_vsb_remove_observation((ok_vsb*)c, st, cam, kp); }
static int d_set_observation_information(void* c, uint64_t st, uint32_t cam, uint32_t kp, const double info[4]) { return ok_vsb_set_observation_information((ok_vsb*)c, st, cam, kp, info); }
static int d_set_landmark(void* c, uint64_t id, const double hp[4], int init) { return ok_vsb_set_landmark((ok_vsb*)c, id, hp, init); }
static int d_merge_landmark(void* c, uint64_t from, uint64_t into) { return ok_vsb_merge_landmark((ok_vsb*)c, from, into); }
static int d_merge_landmarks(void* c, const uint64_t* from, const uint64_t* into, int n) { return ok_vsb_merge_landmarks((ok_vsb*)c, from, into, n); }
static int d_set_pose(void* c, uint64_t id, const double T7[7]) { return ok_vsb_set_pose((ok_vsb*)c, id, T7); }
static int d_optimise_realtime_fe(void* c, int ni, int nt, int vb, int on, int ii) {
    uint64_t* up = NULL; int nu = 0, r;
    r = ok_vsb_optimise_realtime((ok_vsb*)c, ni, nt, vb, on, ii, &up, &nu);
    free(up);
    return r;
}
static int d_clean(void* c) { return ok_vsb_clean_unobserved_landmarks((ok_vsb*)c); }
static void d_set_landmark_id(void* c, uint64_t fr, uint32_t cam, uint32_t kp, uint64_t id) { ok_vsb_set_landmark_id((ok_vsb*)c, fr, cam, kp, id); }
static int d_attempt_loop_closure(void* c, uint64_t old_id, uint64_t new_id, const double T7[7], const double H36[36], double drift, int* skip) {
    return ok_vsb_attempt_loop_closure((ok_vsb*)c, old_id, new_id, T7, H36, drift, skip);
}
static int d_add_loop_closure_frame(void* c, uint64_t id, int skip, uint64_t** lms, int* nl) { return ok_vsb_add_loop_closure_frame((ok_vsb*)c, id, skip, lms, nl); }

static int b_add_imu(void* c, const ok_vg_imu_cfg* i) { return ok_vsb_add_imu((ok_vsb*)c, i); }
static int b_add_camera(void* c, int d, double sr, double sa) { return ok_vsb_add_camera((ok_vsb*)c, d, sr, sa); }
static int b_add_states(void* c, ok_time t, const ok_imu_meas* m, size_t n, int kf, double kr, int nc, const ok_vsb_cam_in* cams) {
    return ok_vsb_add_states((ok_vsb*)c, t, m, n, kf, kr, nc, cams);
}
static int b_set_keyframe(void* c, uint64_t id, int fl) { return ok_vsb_set_keyframe((ok_vsb*)c, id, fl); }
static int b_optimise_realtime(void* c, int ni, int nt, int vb, int on, int ii, uint64_t** up, int* nu) { return ok_vsb_optimise_realtime((ok_vsb*)c, ni, nt, vb, on, ii, up, nu); }
static int b_synchronise(void* c, uint64_t** up, int* nu) { return ok_vsb_synchronise((ok_vsb*)c, up, nu); }
static int b_apply_strategy(void* c, size_t k, size_t l, size_t i, int ex, uint64_t** a, int* na) { return ok_vsb_apply_strategy((ok_vsb*)c, k, l, i, ex, a, na); }
static int b_optimise_full(void* c, int ni, int nt, int vb) { return ok_vsb_optimise_full((ok_vsb*)c, ni, nt, vb); }

/* ------------------------------------------------------------------ helpers */
static void put32(unsigned char* p, size_t* o, uint32_t v) { memcpy(p + *o, &v, 4); *o += 4; }
static void put64f(unsigned char* p, size_t* o, double v) { memcpy(p + *o, &v, 8); *o += 8; }

static void dq_push_back(ok_sys* s, const ok_imu_meas* m) {
    if (s->dn == s->dcap) { s->dcap = s->dcap ? 2 * s->dcap : 64; s->dq = (ok_imu_meas*)realloc(s->dq, sizeof *s->dq * s->dcap); }
    s->dq[s->dn++] = *m;
}
static void dq_push_front(ok_sys* s, const ok_imu_meas* m) {
    dq_push_back(s, m);
    memmove(s->dq + 1, s->dq, sizeof *s->dq * (s->dn - 1));
    s->dq[0] = *m;
}
static ok_imu_meas dq_pop_front(ok_sys* s) {
    const ok_imu_meas m = s->dq[0];
    memmove(s->dq, s->dq + 1, sizeof *s->dq * (s->dn - 1));
    s->dn--;
    return m;
}
/* imuMeasurementsReceived_.PopNonBlocking into the deque */
static int pop_imu(ok_sys* s) {
    if (s->qhead == s->qn) return 0;
    dq_push_back(s, &s->q[s->qhead++]);
    if (s->qhead == s->qn) s->qhead = s->qn = 0;
    return 1;
}
static ok_time time_plus(ok_time t, double d) { ok_time o; ok_time_add(t, ok_duration_from_sec(d), &o); return o; }
static ok_time time_minus(ok_time t, double d) { ok_time o; ok_time_sub_duration(t, ok_duration_from_sec(d), &o); return o; }

/* ------------------------------------------------------------------ construction (ThreadedSlam::init) */
ok_sys* ok_sys_new(const ok_cfg* cfg, ok_vsb* b, const ok_fe_est* est, const ok_sys_be* be,
                   const unsigned char* vocabulary, size_t nvocabulary, char* err, size_t errlen) {
    ok_sys* s;
    ok_fe_params fp;
    int c;
    if (err && errlen) err[0] = 0;
    if (!cfg->imu.use || cfg->enforce_realtime || cfg->use_cnn || cfg->do_extrinsics || cfg->do_final_ba || cfg->octaves != 0 ||
        cfg->absolute_threshold != (double)(int)cfg->absolute_threshold || cfg->ncam > OK_VSB_MAXCAM || cfg->image_delay != 0.0) {
        if (err) snprintf(err, errlen, "unsupported configuration (needs IMU, octaves 0, an integral absolute_threshold, no "
                                       "enforce_realtime / CNN / online extrinsics / final BA / image delay)");
        return NULL;
    }
    for (c = 0; c < cfg->ncam; ++c) {
        if (!cfg->cam[c].used) {
            if (err) snprintf(err, errlen, "camera %d: slam_use other than okvis is not ported", c);
            return NULL;
        }
    }
    s = (ok_sys*)calloc(1, sizeof *s);
    s->cfg = *cfg;
    if (b) s->b = b; else { ok_vsb_hooks h; memset(&h, 0, sizeof h); h.solve = ok_vsb_solve_native; s->b = ok_vsb_new(&h); s->own_b = 1; }
    if (est) s->est = *est;
    else {
        s->est.ctx = s->b;
        s->est.add_landmark = d_add_landmark; s->est.add_observation = d_add_observation; s->est.remove_observation = d_remove_observation;
        s->est.set_observation_information = d_set_observation_information; s->est.set_landmark = d_set_landmark;
        s->est.merge_landmark = d_merge_landmark; s->est.merge_landmarks = d_merge_landmarks; s->est.set_pose = d_set_pose;
        s->est.optimise_realtime = d_optimise_realtime_fe; s->est.clean_unobserved_landmarks = d_clean;
        s->est.set_landmark_id = d_set_landmark_id;
        s->est.attempt_loop_closure = d_attempt_loop_closure; s->est.add_loop_closure_frame = d_add_loop_closure_frame;
    }
    if (be) s->be = *be;
    else {
        s->be.ctx = s->b;
        s->be.add_imu = b_add_imu; s->be.add_camera = b_add_camera; s->be.add_states = b_add_states;
        s->be.set_keyframe = b_set_keyframe; s->be.optimise_realtime = b_optimise_realtime; s->be.synchronise = b_synchronise;
        s->be.apply_strategy = b_apply_strategy; s->be.optimise_full = b_optimise_full;
    }

    /* Frontend setters + ViSlamBackend::addImu / addCamera (one per camera) / setDetectorUniformityRadius */
    memset(&fp, 0, sizeof fp);
    fp.matching_threshold = cfg->matching_threshold; fp.keyframe_overlap = (float)cfg->keyframe_overlap;
    fp.num_matching_threads = cfg->num_matching_threads; fp.imu_use = cfg->imu.use; fp.do_loop_closures = cfg->do_loop_closures;
    fp.realtime_num_threads = cfg->realtime_num_threads; fp.p_dbow = cfg->p_dbow; fp.drift_percentage = cfg->drift_percentage;
    fp.realtime_max_iterations = cfg->realtime_max_iterations;
    s->be.add_imu(s->be.ctx, &cfg->imu);
    for (c = 0; c < cfg->ncam; ++c) s->be.add_camera(s->be.ctx, cfg->do_extrinsics, cfg->sigma_r, cfg->sigma_alpha);
    s->kptradius = 0.09 * cfg->detection_threshold / 36.0;
    s->fe = ok_fe_new(s->b, &fp, &s->est);
    if (vocabulary && ok_fe_set_vocabulary(s->fe, vocabulary, nvocabulary)) {
        if (err) snprintf(err, errlen, "cannot parse the vocabulary");
        ok_sys_free(s);
        return NULL;
    }
    s->brisk = ok_brisk_create(NULL);
    for (c = 0; c < cfg->ncam; ++c) {          /* the camera model blob addStates reads (ok_cam.h header layout) */
        const ok_cfg_cam* k = &cfg->cam[c];
        size_t o = 0; int i;
        put32(s->hdr[c], &o, (uint32_t)k->dist); put32(s->hdr[c], &o, (uint32_t)k->w); put32(s->hdr[c], &o, (uint32_t)k->h);
        put64f(s->hdr[c], &o, k->fu); put64f(s->hdr[c], &o, k->fv); put64f(s->hdr[c], &o, k->cu); put64f(s->hdr[c], &o, k->cv);
        put32(s->hdr[c], &o, 4u);
        for (i = 0; i < 4; ++i) put64f(s->hdr[c], &o, k->d[i]);
        s->hlen[c] = o;
    }
    s->first_frame = 1;
    return s;
}

void ok_sys_free(ok_sys* s) {
    int c;
    if (!s) return;
    ok_fe_free(s->fe);
    if (s->own_b) ok_vsb_free(s->b);
    ok_brisk_destroy(s->brisk);
    for (c = 0; c < OK_CFG_MAXCAM; ++c) { free(s->rays[c]); free(s->jacs[c]); }
    free(s->q); free(s->dq);
    free(s);
}
ok_vsb* ok_sys_backend(ok_sys* s) { return s->b; }
void ok_sys_set_publish(ok_sys* s, ok_sys_publish_fn fn, void* ctx) { s->publish = fn; s->publish_ctx = ctx; }

int ok_sys_add_imu(ok_sys* s, ok_time t, const double acc[3], const double gyr[3]) {
    ok_imu_meas m;
    m.t = t;
    memcpy(m.acc, acc, sizeof m.acc); memcpy(m.gyr, gyr, sizeof m.gyr);
    if (s->qn == s->qcap) { s->qcap = s->qcap ? 2 * s->qcap : 1024; s->q = (ok_imu_meas*)realloc(s->q, sizeof *s->q * s->qcap); }
    s->q[s->qn++] = m;
    return 1;
}

/* ------------------------------------------------------------------ Frontend::detectAndDescribe on every camera */
typedef struct feats { size_t n[OK_CFG_MAXCAM]; float* kp[OK_CFG_MAXCAM]; unsigned char* desc[OK_CFG_MAXCAM]; } feats;
static void feats_free(feats* f) { int c; for (c = 0; c < OK_CFG_MAXCAM; ++c) { free(f->kp[c]); free(f->desc[c]); } }

static int detect_all(ok_sys* s, ok_time t, const double T_WS7[7], const unsigned char* const* images, feats* f) {
    ok_tf T_WS;
    int c;
    size_t i;
    memset(f, 0, sizeof *f);
    if (!s->maps_ready) {                       /* setCameraProperties at the first detectAndDescribe, for every camera */
        for (c = 0; c < s->cfg.ncam; ++c) {
            const ok_cfg_cam* k = &s->cfg.cam[c];
            ok_cam cam;
            ok_cam_init(&cam, k->dist, k->w, k->h, k->fu, k->fv, k->cu, k->cv, k->d);
            s->rays[c] = (float*)malloc(sizeof(float) * 3 * (size_t)k->w * (size_t)k->h);
            s->jacs[c] = (float*)malloc(sizeof(float) * 6 * (size_t)k->w * (size_t)k->h);
            ok_cam_awareness_maps(&cam, s->rays[c], s->jacs[c]);
        }
        s->maps_ready = 1;
    }
    ok_tf_set_coeffs(&T_WS, T_WS7, 1);
    for (c = 0; c < s->cfg.ncam; ++c) {
        const ok_cfg_cam* k = &s->cfg.cam[c];
        ok_tf T_SC, T_WC, T_CW;
        const double g_W[3] = {0.0, 0.0, -1.0};
        double e[3];
        float dir[3];
        ok_brisk_keypoint* kp = NULL;
        size_t nk = 0;
        if (!images[c]) continue;
        ok_tf_set_coeffs(&T_SC, k->T_SC, 1);
        ok_tf_mul(&T_WS, &T_SC, &T_WC, 1);                  /* T_WC = T_WS * T_SC */
        ok_tf_inverse(&T_WC, &T_CW, 1);
        ok_m3_mulv(T_CW.C, g_W, e);                         /* extraction direction: T_WC.inverse().C() * g_W */
        dir[0] = (float)e[0]; dir[1] = (float)e[1]; dir[2] = (float)e[2];
        if (ok_brisk_detect(images[c], k->w, k->h, s->cfg.detection_threshold, (int)s->cfg.absolute_threshold,
                            (size_t)s->cfg.max_num_keypoints, &kp, &nk, NULL) ||
            ok_brisk_describe(s->brisk, images[c], k->w, k->h, s->rays[c], s->jacs[c], (float)k->fu, dir, kp, &nk, &f->desc[c], NULL)) {
            free(kp);
            return -1;
        }
        f->kp[c] = (float*)malloc(sizeof(float) * 3 * (nk ? nk : 1));
        for (i = 0; i < nk; ++i) { f->kp[c][3 * i] = kp[i].x; f->kp[c][3 * i + 1] = kp[i].y; f->kp[c][3 * i + 2] = kp[i].size; }
        f->n[c] = nk;
        free(kp);
        if (s->be.on_features) s->be.on_features(s->be.ctx, t, c, nk, f->kp[c], f->desc[c]);
    }
    return 0;
}

/* ------------------------------------------------------------------ ThreadedSlam::optimisePublishMarginalise */
static void optimise_publish_marginalise(ok_sys* s, uint64_t id, ok_time t) {
    uint64_t* ids = NULL;
    int n = 0;
    ok_sys_state st;
    const ok_vg* g;
    s->be.optimise_realtime(s->be.ctx, s->cfg.realtime_max_iterations, s->cfg.realtime_num_threads, 0, 0, ok_fe_is_initialised(s->fe), &ids, &n);
    free(ids); ids = NULL; n = 0;
    if (ok_vsb_is_loop_closure_available(s->b)) { s->be.synchronise(s->be.ctx, &ids, &n); free(ids); ids = NULL; n = 0; }
    g = ok_vsb_graph(s->b, 0);
    memset(&st, 0, sizeof st);
    st.t = t; st.id = id;
    ok_vg_pose_values(g, id, st.T_WS);
    ok_vg_sb_values(g, id, st.sb);
    if (s->publish) s->publish(s->publish_ctx, &st);
    s->be.apply_strategy(s->be.ctx, (size_t)s->cfg.num_keyframes, (size_t)s->cfg.num_loop_closure_frames, (size_t)s->cfg.num_imu_frames, 1, &ids, &n);
    free(ids);
}

/* ------------------------------------------------------------------ ThreadedSlam::processFrame */
int ok_sys_add_frame(ok_sys* s, ok_time t, const unsigned char* const* images) {
    const ok_time t_hi = time_plus(t, IMU_TEMPORAL_OVERLAP);
    double T7[7];
    feats f;
    int ran_detection = 0, c, as_kf = 0;
    size_t nkp_total = 0;
    uint64_t id;
    ok_vsb_cam_in cams[OK_CFG_MAXCAM];

    if (s->first_frame) {
        ok_imu_meas front;
        if (s->qhead == s->qn) return -1;
        front = s->q[s->qhead];
        if (ok_time_le(time_minus(t, IMU_TEMPORAL_OVERLAP), front.t)) return 0;   /* startup: frame without older IMU */
        if (ok_time_lt(s->q[s->qn - 1].t, t_hi)) return -1;
        do {
            if (!pop_imu(s)) return -1;
        } while (ok_time_lt(s->dq[s->dn - 1].t, t_hi));
        s->first_frame = 0;
    } else {
        while (ok_time_lt(s->dq[s->dn - 1].t, t_hi)) {
            if (!pop_imu(s)) return -1;
        }
    }
    if (!s->last_init) {
        ok_imu_init_pose(s->dq, s->dn, T7);
        if (detect_all(s, t, T7, images, &f)) { feats_free(&f); return -2; }
        for (c = 0; c < s->cfg.ncam; ++c) nkp_total += f.n[c];
        if (!ok_fe_is_initialised(s->fe) && nkp_total < 15) { feats_free(&f); return 0; }
        ran_detection = 1;
    } else {
        ok_imu_params ip;
        double sb[9];
        ok_tf c0;
        ok_tf_convert(&c0, s->last_T);
        T7[0] = c0.r[0]; T7[1] = c0.r[1]; T7[2] = c0.r[2];
        T7[3] = c0.q.x; T7[4] = c0.q.y; T7[5] = c0.q.z; T7[6] = c0.q.w;
        memcpy(sb, s->last_sb, sizeof sb);
        memset(&ip, 0, sizeof ip);
        ip.sigma_g_c = s->cfg.imu.sigma_g_c; ip.sigma_a_c = s->cfg.imu.sigma_a_c;
        ip.sigma_gw_c = s->cfg.imu.sigma_gw_c; ip.sigma_aw_c = s->cfg.imu.sigma_aw_c;
        ip.g = s->cfg.imu.g; ip.g_max = s->cfg.imu.g_max; ip.a_max = s->cfg.imu.a_max;
        ok_imu_propagation(s->dq, s->dn, &ip, T7, sb, s->last_t, t, NULL, NULL);
    }
    if (!ran_detection) {
        if (detect_all(s, t, T7, images, &f)) { feats_free(&f); return -2; }
        for (c = 0; c < s->cfg.ncam; ++c) nkp_total += f.n[c];
        if (!ok_fe_is_initialised(s->fe) && nkp_total < 15) { feats_free(&f); return 0; }
    }

    /* lastOptimisedState_ = the newest state now */
    if (ok_vsb_num_frames(s->b) > 0) {
        const ok_vg* g = ok_vsb_graph(s->b, 0);
        const uint64_t cid = ok_vsb_current_state_id(s->b);
        ok_vg_state_view sv;
        ok_vg_pose_values(g, cid, s->last_T);
        ok_vg_sb_values(g, cid, s->last_sb);
        ok_vg_state_find(g, cid, &sv);
        s->last_t = sv.ts;
        s->last_init = 1;
        /* remove IMU measurements from the deque, keeping the last one before lastOptimisedState_.timestamp - overlap */
        {
            const ok_time lo = time_minus(s->last_t, IMU_TEMPORAL_OVERLAP);
            while (s->dn > 0 && ok_time_lt(s->dq[0].t, lo)) {
                const ok_imu_meas m = dq_pop_front(s);
                if (s->dn == 0 || ok_time_gt(s->dq[0].t, lo)) { dq_push_front(s, &m); break; }
            }
        }
    }

    /* estimator.addStates(multiFrame, imuMeasurementDeque_, asKeyframe = false) */
    memset(cams, 0, sizeof cams);
    for (c = 0; c < s->cfg.ncam; ++c) {
        cams[c].header = s->hdr[c]; cams[c].hlen = s->hlen[c];
        memcpy(cams[c].T_SC, s->cfg.cam[c].T_SC, sizeof cams[c].T_SC);
        cams[c].rows = s->cfg.cam[c].h; cams[c].cols = s->cfg.cam[c].w;
        cams[c].nkp = (int)f.n[c]; cams[c].kp = f.kp[c];
    }
    if (!s->be.add_states(s->be.ctx, t, s->dq, s->dn, 0, s->kptradius, s->cfg.ncam, cams)) { feats_free(&f); return 0; }
    id = ok_vsb_current_state_id(s->b);
    {
        int nk[OK_CFG_MAXCAM];
        const unsigned char* d[OK_CFG_MAXCAM];
        for (c = 0; c < s->cfg.ncam; ++c) { nk[c] = (int)f.n[c]; d[c] = f.desc[c]; }
        ok_fe_add_frame(s->fe, id, s->cfg.ncam, nk, d);
    }
    feats_free(&f);
    if (!ok_fe_data_association(s->fe, id, &as_kf) && !ok_fe_is_initialised(s->fe)) return -2;
    s->be.set_keyframe(s->be.ctx, id, as_kf);
    optimise_publish_marginalise(s, id, t);
    if (ok_vsb_needs_full_graph_optimisation(s->b))
        s->be.optimise_full(s->be.ctx, s->cfg.full_graph_iterations, s->cfg.full_graph_num_threads, 0);
    return 1;
}

/* ------------------------------------------------------------------ CSV output */
static void write_time(FILE* f, ok_time t) { fprintf(f, "%u%09u", t.sec, t.nsec); }
void ok_sys_write_csv_header(FILE* f) {
    fprintf(f, "timestamp, p_WS_W_x, p_WS_W_y, p_WS_W_z, q_WS_x, q_WS_y, q_WS_z, q_WS_w, v_WS_W_x, v_WS_W_y, v_WS_W_z, "
               "b_g_x, b_g_y, b_g_z, b_a_x, b_a_y, b_a_z\n");
}
static void write_values(FILE* f, const double T7[7], const double sb[9]) {
    int i;
    for (i = 0; i < 7; ++i) fprintf(f, ", %.18e", T7[i]);
    for (i = 0; i < 9; ++i) fprintf(f, ", %.18e", sb[i]);
}
void ok_sys_write_state_csv(FILE* f, const ok_sys_state* s) {
    write_time(f, s->t);
    write_values(f, s->T_WS, s->sb);
    fprintf(f, ", \n");
}
int ok_sys_write_final_csv(ok_sys* s, FILE* f) {
    const ok_vg* g = ok_vsb_graph(s->b, 0);
    const int n = ok_vg_anystate_count(g);
    int i;
    ok_sys_write_csv_header(f);
    for (i = 0; i < n; ++i) {
        uint64_t id, kf;
        ok_time ts;
        double T_Sk_S[7], v_Sk[3], T7[7], sb[9];
        ok_vg_anystate_at(g, i, &id, &kf, &ts, T_Sk_S, v_Sk);
        if (kf) {
            /* reconstruct from the keyframe: T_WS = pose(kf) * T_Sk_S (TransformationCacheless), v = C_WSk * v_Sk */
            double Tk[7], sbk[9];
            ok_tf a, b, o;
            ok_vg_pose_values(g, kf, Tk);
            ok_vg_sb_values(g, kf, sbk);
            ok_tf_set_coeffs(&a, Tk, 0);
            ok_tf_set_coeffs(&b, T_Sk_S, 0);
            ok_tf_mul(&a, &b, &o, 0);
            T7[0] = o.r[0]; T7[1] = o.r[1]; T7[2] = o.r[2];
            T7[3] = o.q.x; T7[4] = o.q.y; T7[5] = o.q.z; T7[6] = o.q.w;
            ok_tf_mul_v3(&a, v_Sk, sb, 0);
            memcpy(sb + 3, sbk + 3, 6 * sizeof(double));
        } else {
            ok_vg_state_view sv;
            ok_vg_pose_values(g, id, T7);
            ok_vg_sb_values(g, id, sb);
            ok_vg_state_find(g, id, &sv);
            ts = sv.ts;
        }
        write_time(f, ts);
        write_values(f, T7, sb);
        fprintf(f, ", %llu\n", (unsigned long long)kf);
    }
    return 0;
}
