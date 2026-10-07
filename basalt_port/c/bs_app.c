/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Basalt port, module M9: see bs_app.h.  Every function follows sqrt_keypoint_vio.cpp statement by statement. */
#include "bs_app.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bs_svd.h"

/* ------------------------------------------------------------------------------------------------ json (flat key scans) */

static char* slurp(const char* path) {
    FILE* f = fopen(path, "rb");
    long n;
    char* b;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    b = (char*)malloc((size_t)n + 1);
    if (b && fread(b, 1, (size_t)n, f) != (size_t)n) { free(b); b = NULL; }
    if (b) b[n] = 0;
    fclose(f);
    return b;
}

/* pointer to the value after "key": (whitespace skipped), or NULL */
static const char* jkey(const char* buf, const char* key) {
    char pat[128];
    const char* q;
    snprintf(pat, sizeof pat, "\"%s\"", key);
    q = strstr(buf, pat);
    if (!q) return NULL;
    q += strlen(pat);
    while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') q++;
    if (*q != ':') return NULL;
    q++;
    while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') q++;
    return q;
}
static int jnum(const char* buf, const char* key, double* out) {
    const char* q = jkey(buf, key);
    char* end;
    if (!q) return 1;
    *out = strtod(q, &end);
    return end == q;
}
static int jbool(const char* buf, const char* key, int* out) {
    const char* q = jkey(buf, key);
    if (!q) return 1;
    if (!strncmp(q, "true", 4)) { *out = 1; return 0; }
    if (!strncmp(q, "false", 5)) { *out = 0; return 0; }
    return 1;
}
static int jstr_is(const char* buf, const char* key, const char* want) {
    const char* q = jkey(buf, key);
    size_t n = strlen(want);
    return q && *q == '"' && !strncmp(q + 1, want, n) && q[1 + n] == '"';
}
/* n numbers of an array value */
static int jarr(const char* buf, const char* key, double* out, int n) {
    const char* q = jkey(buf, key);
    int i;
    if (!q || *q != '[') return 1;
    q++;
    for (i = 0; i < n; ++i) {
        char* end;
        while (*q == ' ' || *q == ',' || *q == '\t' || *q == '\n' || *q == '\r') q++;
        out[i] = strtod(q, &end);
        if (end == q) return 1;
        q = end;
    }
    return 0;
}

int bs_app_cfg_load(bs_app_cfg* c, const char* config_path, const char* calib_path, char* err, size_t err_n) {
    char* b = slurp(config_path);
    char* cb;
    double v;
    int bv, rc = 0;
    bs_flow_calib fc;
    int cam, k;
    memset(c, 0, sizeof *c);
    if (err_n) err[0] = 0;
    if (!b) { snprintf(err, err_n, "cannot read %s", config_path); return 1; }
    /* VioConfig defaults (vio_config.cpp), overridden by the keys of the json */
    c->max_states = 3; c->max_kfs = 7; c->min_frames_after_kf = 5; c->max_iterations = 7; c->marg_lost_landmarks = 1;
    c->new_kf_keypoints_thresh = 0.7; c->obs_std_dev = 0.5; c->obs_huber_thresh = 1.0; c->min_triangulation_dist = 0.05; c->kf_marg_feature_ratio = 0.1;
    c->lm_lambda_initial = 1e-4; c->lm_lambda_min = 1e-6; c->lm_lambda_max = 1e2;
    c->init_pose_weight = 1e8; c->init_ba_weight = 1e1; c->init_bg_weight = 1e2;
#define INT(key, dst) do { if (!jnum(b, "config." key, &v)) (dst) = (int)v; } while (0)
#define DBL(key, dst) do { if (!jnum(b, "config." key, &v)) (dst) = v; } while (0)
    INT("vio_max_states", c->max_states); INT("vio_max_kfs", c->max_kfs); INT("vio_min_frames_after_kf", c->min_frames_after_kf);
    INT("vio_max_iterations", c->max_iterations);
    DBL("vio_new_kf_keypoints_thresh", c->new_kf_keypoints_thresh); DBL("vio_obs_std_dev", c->obs_std_dev);
    DBL("vio_obs_huber_thresh", c->obs_huber_thresh); DBL("vio_min_triangulation_dist", c->min_triangulation_dist);
    DBL("vio_kf_marg_feature_ratio", c->kf_marg_feature_ratio);
    DBL("vio_lm_lambda_initial", c->lm_lambda_initial); DBL("vio_lm_lambda_min", c->lm_lambda_min); DBL("vio_lm_lambda_max", c->lm_lambda_max);
    DBL("vio_init_pose_weight", c->init_pose_weight); DBL("vio_init_ba_weight", c->init_ba_weight); DBL("vio_init_bg_weight", c->init_bg_weight);
#undef INT
#undef DBL
    if (!jbool(b, "config.vio_marg_lost_landmarks", &bv)) c->marg_lost_landmarks = bv;
    /* settings the C port does not implement */
    if (jkey(b, "config.vio_linearization_type") && !jstr_is(b, "config.vio_linearization_type", "ABS_QR")) { snprintf(err, err_n, "only vio_linearization_type ABS_QR is ported"); rc = 1; }
    if (!jbool(b, "config.vio_sqrt_marg", &bv) && !bv) { snprintf(err, err_n, "only vio_sqrt_marg true is ported"); rc = 1; }
    if (jbool(b, "config.vio_use_lm", &bv) || !bv) { snprintf(err, err_n, "only vio_use_lm true is ported"); rc = 1; }
    if (jbool(b, "config.vio_scale_jacobian", &bv) || bv) { snprintf(err, err_n, "only vio_scale_jacobian false is ported"); rc = 1; }
    if (!jnum(b, "config.vio_lm_pose_damping_variant", &v) && v != 1) { snprintf(err, err_n, "only vio_lm_pose_damping_variant 1 is ported"); rc = 1; }
    if (!jnum(b, "config.vio_lm_landmark_damping_variant", &v) && v != 1) { snprintf(err, err_n, "only vio_lm_landmark_damping_variant 1 is ported"); rc = 1; }
    if (!jbool(b, "config.vio_debug", &bv) && bv) { snprintf(err, err_n, "vio_debug is not ported"); rc = 1; }
    if (!jbool(b, "config.vio_extended_logging", &bv) && bv) { snprintf(err, err_n, "vio_extended_logging is not ported"); rc = 1; }
    if (jkey(b, "config.optical_flow_type") && !jstr_is(b, "config.optical_flow_type", "frame_to_frame")) { snprintf(err, err_n, "only optical_flow_type frame_to_frame is ported"); rc = 1; }
    free(b);
    if (rc) return rc;

    if (bs_flow_calib_load(calib_path, &fc)) { snprintf(err, err_n, "cannot read intrinsics / T_imu_cam of %s", calib_path); return 1; }
    for (cam = 0; cam < 2; ++cam) {
        for (k = 0; k < 6; ++k) c->intr[cam][k] = fc.intr[cam][k];
        for (k = 0; k < 7; ++k) c->T_i_c[cam][k] = fc.T_i_c[cam][k];
    }
    cb = slurp(calib_path);
    if (!cb) { snprintf(err, err_n, "cannot read %s", calib_path); return 1; }
    if (jarr(cb, "calib_accel_bias", c->accel_bias_full, 9) || jarr(cb, "calib_gyro_bias", c->gyro_bias_full, 12) ||
        jarr(cb, "accel_noise_std", c->accel_noise_std, 3) || jarr(cb, "gyro_noise_std", c->gyro_noise_std, 3) ||
        jarr(cb, "accel_bias_std", c->accel_bias_std, 3) || jarr(cb, "gyro_bias_std", c->gyro_bias_std, 3) ||
        jnum(cb, "imu_update_rate", &c->imu_update_rate)) { snprintf(err, err_n, "missing IMU calibration keys in %s", calib_path); rc = 1; }
    if (!jnum(cb, "cam_time_offset_ns", &v) && v != 0) { snprintf(err, err_n, "cam_time_offset_ns != 0 is not used by the C++ estimator either (commented out), but is unexpected"); rc = 1; }
    free(cb);
    return rc;
}

/* ------------------------------------------------------------------------------------------------ small helpers */

static void fatal(bs_app* a, const char* msg) {
    if (!a->fatal) { a->fatal = 1; snprintf(a->fatal_msg, sizeof a->fatal_msg, "%s", msg); }
}
/* std::map<int64_t, int> as a sorted array */
typedef struct kc_map { bs_kf_count* e; int n, cap; } kc_map;
static void kc_incr(kc_map* m, int64_t t) {
    int lo = 0, hi = m->n;
    while (lo < hi) { int mid = (lo + hi) / 2; if (m->e[mid].t_ns < t) lo = mid + 1; else hi = mid; }
    if (lo < m->n && m->e[lo].t_ns == t) { m->e[lo].n++; return; }
    if (m->n == m->cap) { m->cap = m->cap ? 2 * m->cap : 16; m->e = (bs_kf_count*)realloc(m->e, sizeof(bs_kf_count) * (size_t)m->cap); }
    memmove(&m->e[lo + 1], &m->e[lo], sizeof(bs_kf_count) * (size_t)(m->n - lo));
    m->e[lo].t_ns = t; m->e[lo].n = 1;
    m->n++;
}

static const bs_flow_kp* obs_find(const bs_flow_obs* o, int64_t id) {
    int lo = 0, hi = o->n;
    while (lo < hi) { int m = (lo + hi) / 2; if ((int64_t)o->kp[m].id < id) lo = m + 1; else hi = m; }
    return (lo < o->n && (int64_t)o->kp[lo].id == id) ? &o->kp[lo] : NULL;
}

static bs_frame_state* find_state(bs_vio* v, int64_t t) {
    int lo = 0, hi = v->ba.n_states;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.states[m].t_ns < t) lo = m + 1; else hi = m; }
    return (lo < v->ba.n_states && v->ba.states[lo].t_ns == t) ? &v->ba.states[lo] : NULL;
}

/* getPoseStateWithLin(t).getPose() */
static int get_pose(const bs_vio* v, int64_t t, bs_se3f* out) {
    int lo = 0, hi = v->ba.n_poses;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.poses[m].t_ns < t) lo = m + 1; else hi = m; }
    if (lo < v->ba.n_poses && v->ba.poses[lo].t_ns == t) { *out = v->ba.poses[lo].linearized ? v->ba.poses[lo].cur : v->ba.poses[lo].lin; return 1; }
    lo = 0; hi = v->ba.n_states;
    while (lo < hi) { int m = (lo + hi) / 2; if (v->ba.states[m].t_ns < t) lo = m + 1; else hi = m; }
    if (lo < v->ba.n_states && v->ba.states[lo].t_ns == t) {          /* PoseStateWithLin(const PoseVelBiasStateWithLin&) */
        const bs_frame_state* s = &v->ba.states[lo];
        bs_se3f lin = {{s->s.lin.s.q[0], s->s.lin.s.q[1], s->s.lin.s.q[2], s->s.lin.s.q[3]}, {s->s.lin.s.p[0], s->s.lin.s.p[1], s->s.lin.s.p[2]}};
        if (s->s.linearized) {
            float d6[6];
            int i;
            for (i = 0; i < 6; ++i) d6[i] = s->delta[i];
            bs_inc_posef(d6, &lin);
        }
        *out = lin;
        return 1;
    }
    return 0;
}

static void pvstate_to_se3(const bs_pvstate* s, bs_se3f* o) {
    o->so3.x = s->q[0]; o->so3.y = s->q[1]; o->so3.z = s->q[2]; o->so3.w = s->q[3];
    o->t[0] = s->p[0]; o->t[1] = s->p[1]; o->t[2] = s->p[2];
}

static const bs_pvbstate* state_get(const bs_frame_state* s) { return s->s.linearized ? &s->s.cur : &s->s.lin; }   /* getState() */

/* ------------------------------------------------------------------------------------------------ IMU queue */

/* popFromImuDataQueue (cast to float) + calib_accel_bias / calib_gyro_bias getCalibrated, as proc_func does after every pop */
static void imu_pop(bs_app* a) {
    if (a->imu_pos < a->n_imu) {
        const bs_imu_raw* r = &a->imu[a->imu_pos++];
        float t[3];
        int i;
        a->data.t_ns = r->t_ns;
        for (i = 0; i < 3; ++i) { a->data.accel[i] = (float)r->accel[i]; a->data.gyro[i] = (float)r->gyro[i]; }
        bs_m3f_mulv(a->calib_accel_scale, a->data.accel, t);
        for (i = 0; i < 3; ++i) a->data.accel[i] = (a->data.accel[i] + t[i]) - a->calib_accel_bias[i];     /* raw + scale * raw - bias */
        bs_m3f_mulv(a->calib_gyro_scale, a->data.gyro, t);
        for (i = 0; i < 3; ++i) a->data.gyro[i] = (a->data.gyro[i] + t[i]) - a->calib_gyro_bias[i];
        a->have_data = 1;
    } else {
        a->have_data = 0;
    }
}

/* ------------------------------------------------------------------------------------------------ constructor */

int bs_app_init(bs_app* a, const bs_app_cfg* cfg) {
    bs_vio* v = &a->v;
    int c, i;
    memset(a, 0, sizeof *a);
    a->cfg = *cfg;
    bs_vio_init(v);
    /* SqrtKeypointVioEstimator ctor */
    v->g[0] = 0.0f; v->g[1] = 0.0f; v->g[2] = (float)-9.81;            /* g_.cast<Scalar>() of constants::g */
    v->ba.obs_std_dev = (float)cfg->obs_std_dev;
    v->ba.huber_thresh = (float)cfg->obs_huber_thresh;
    for (c = 0; c < 2; ++c) {                                           /* calib = calib_.cast<Scalar>() */
        bs_se3d sd;
        bs_ds_cast_f(&v->ba.cam[c], cfg->intr[c]);
        sd.so3.x = cfg->T_i_c[c][3]; sd.so3.y = cfg->T_i_c[c][4]; sd.so3.z = cfg->T_i_c[c][5]; sd.so3.w = cfg->T_i_c[c][6];
        sd.t[0] = cfg->T_i_c[c][0]; sd.t[1] = cfg->T_i_c[c][1]; sd.t[2] = cfg->T_i_c[c][2];
        bs_se3_f_from_d(&sd, &v->ba.T_i_c[c]);
    }
    {
        float rate = (float)cfg->imu_update_rate, srate = sqrtf(rate);
        for (i = 0; i < 3; ++i) {
            const float an = (float)cfg->accel_noise_std[i] * srate;    /* dicrete_time_accel_noise_std() */
            const float gn = (float)cfg->gyro_noise_std[i] * srate;
            a->accel_cov[i] = an * an;                                  /* .array().square() */
            a->gyro_cov[i] = gn * gn;
            v->gyro_bias_sqrt_weight[i] = 1.0f / (float)cfg->gyro_bias_std[i];
            v->accel_bias_sqrt_weight[i] = 1.0f / (float)cfg->accel_bias_std[i];
        }
    }
    {   /* CalibAccelBias::getBiasAndScale / CalibGyroBias::getBiasAndScale of the float-cast parameters */
        float pa[9], pg[12];
        for (i = 0; i < 9; ++i) pa[i] = (float)cfg->accel_bias_full[i];
        for (i = 0; i < 12; ++i) pg[i] = (float)cfg->gyro_bias_full[i];
        for (i = 0; i < 3; ++i) { a->calib_accel_bias[i] = pa[i]; a->calib_gyro_bias[i] = pg[i]; }
        memset(a->calib_accel_scale, 0, sizeof a->calib_accel_scale);
        a->calib_accel_scale[0] = pa[3]; a->calib_accel_scale[1] = pa[4]; a->calib_accel_scale[2] = pa[5];
        a->calib_accel_scale[4] = pa[6]; a->calib_accel_scale[5] = pa[7]; a->calib_accel_scale[8] = pa[8];
        for (i = 0; i < 9; ++i) a->calib_gyro_scale[i] = pg[3 + i];
    }
    v->max_states = cfg->max_states; v->max_kfs = cfg->max_kfs;
    v->kf_marg_feature_ratio = cfg->kf_marg_feature_ratio;
    v->lm_lambda_initial = cfg->lm_lambda_initial;
    v->lambda = (float)cfg->lm_lambda_initial; v->min_lambda = (float)cfg->lm_lambda_min; v->max_lambda = (float)cfg->lm_lambda_max; v->lambda_vee = 2.0f;
    v->max_iterations = cfg->max_iterations;
    v->opt_started = 0; v->take_kf = 1; v->frames_after_kf = 0;
    return 0;
}

void bs_app_destroy(bs_app* a) {
    int i;
    for (i = 0; i < a->n_prev; ++i) { free(a->prev[i].obs[0].kp); free(a->prev[i].obs[1].kp); }
    free(a->prev);
    bs_vio_destroy(&a->v);
    memset(a, 0, sizeof *a);
}

int bs_app_set_imu(bs_app* a, const bs_imu_raw* imu, size_t n) {
    a->imu = imu; a->n_imu = n; a->imu_pos = 0;
    imu_pop(a);                                       /* data = popFromImuDataQueue(); BASALT_ASSERT_MSG(data, "first IMU measurment is nullptr") */
    if (!a->have_data) { fatal(a, "first IMU measurement is nullptr"); return 1; }
    return 0;
}

/* ------------------------------------------------------------------------------------------------ measure() */

typedef struct kobs { bs_tcid t; float pos[2]; } kobs;

static int measure(bs_app* a, int64_t t_ns, const bs_flow_obs obs[2], const bs_imu_meas* meas, bs_app_out* out) {
    bs_vio* v = &a->v;
    bs_app_frame_info fi;
    int connected0 = 0, i, f, k, n_added = 0, took_kf = 0;
    kc_map npc = {NULL, 0, 0};
    bs_htab unc;
    int64_t* unc_ids = NULL; int n_unc = 0;
    int64_t* lost = NULL; int n_lost = 0;
    bs_opt_info info;
    bs_marg_result res;
    memset(&fi, 0, sizeof fi);

    if (meas) {
        bs_frame_state* ls = find_state(v, v->last_state_t_ns);
        bs_pvbstate next;
        bs_frame_state* ns;
        bs_imu_meas* slot;
        int lo, hi;
        if (!ls || ls->t_ns != meas->start_t_ns || t_ns != meas->delta.t_ns + meas->start_t_ns || !(meas->delta.t_ns > 0)) { fatal(a, "measure: BASALT_ASSERT on the IMU measurement"); return -1; }
        next = *state_get(ls);                                                         /* PoseVelBiasState next_state = ...getState() */
        bs_imu_predict_state(meas, &state_get(ls)->s, v->g, &next.s);                  /* predictState */
        v->last_state_t_ns = t_ns;
        next.s.t_ns = t_ns;
        ns = bs_vio_state_insert(v, t_ns);                                             /* frame_states[t] = PoseVelBiasStateWithLin(next_state) */
        if (!ns) { fatal(a, "measure: duplicate frame state"); return -1; }
        ns->s.linearized = 0; ns->s.lin = next; ns->s.cur = next;
        for (i = 0; i < 15; ++i) ns->delta[i] = 0.0f;
        lo = 0; hi = v->n_imu;                                                         /* imu_meas[start] = *meas */
        while (lo < hi) { int m = (lo + hi) / 2; if (v->imu[m].start_t_ns < meas->start_t_ns) lo = m + 1; else hi = m; }
        if (lo < v->n_imu && v->imu[lo].start_t_ns == meas->start_t_ns) slot = &v->imu[lo];
        else slot = bs_vio_imu_insert(v, meas->start_t_ns);
        *slot = *meas;
        fi.meas_used = 1;
    }

    /* prev_opt_flow_res[t] = opt_flow_meas */
    {
        bs_frame_obs* e;
        if (a->n_prev && a->prev[a->n_prev - 1].t_ns >= t_ns) { fatal(a, "measure: frame time not increasing"); return -1; }
        if (a->n_prev == a->cap_prev) { a->cap_prev = a->cap_prev ? 2 * a->cap_prev : 16; a->prev = (bs_frame_obs*)realloc(a->prev, sizeof(bs_frame_obs) * (size_t)a->cap_prev); }
        e = &a->prev[a->n_prev++];
        e->t_ns = t_ns;
        for (k = 0; k < 2; ++k) {
            e->obs[k].n = obs[k].n; e->obs[k].cap = obs[k].n;
            e->obs[k].kp = (bs_flow_kp*)malloc(sizeof(bs_flow_kp) * (size_t)(obs[k].n ? obs[k].n : 1));
            if (obs[k].n) memcpy(e->obs[k].kp, obs[k].kp, sizeof(bs_flow_kp) * (size_t)obs[k].n);
        }
    }

    /* Make new residual for existing keypoints */
    bs_htab_init(&unc, BS_HK_U64);
    for (i = 0; i < 2; ++i) {
        bs_tcid target; target.frame_id = t_ns; target.cam_id = (uint64_t)i;
        for (k = 0; k < obs[i].n; ++k) {
            const int64_t kpt_id = (int)obs[i].kp[k].id;                                /* int kpt_id = kv_obs.first */
            const bs_keypoint* lm = bs_lmdb_get_landmark(&v->ba.lmdb, kpt_id);
            if (lm) {
                const float pos[2] = {obs[i].kp[k].m[4], obs[i].kp[k].m[5]};
                bs_lmdb_add_observation(&v->ba.lmdb, target, kpt_id, pos);
                kc_incr(&npc, lm->host.frame_id);
                if (i == 0) connected0++;
            } else if (i == 0) {
                int ins;
                bs_htab_insert(&unc, kpt_id, 0, &ins);
            }
        }
    }
    {
        bs_hnode* n;
        for (n = unc.before_begin.next; n; n = n->next) n_unc++;
        unc_ids = (int64_t*)malloc(sizeof(int64_t) * (size_t)(n_unc ? n_unc : 1));
        for (n = unc.before_begin.next, i = 0; n; n = n->next) unc_ids[i++] = n->k0;
    }
    fi.connected0 = connected0; fi.unconnected0 = n_unc;

    /* Scalar(connected0) / (connected0 + unconnected_obs0.size()) < thresh && frames_after_kf > min_frames_after_kf */
    {
        const size_t denom = (size_t)connected0 + (size_t)n_unc;                         /* int + size_t */
        if ((float)connected0 / (float)denom < (float)a->cfg.new_kf_keypoints_thresh && v->frames_after_kf > a->cfg.min_frames_after_kf) v->take_kf = 1;
    }

    if (v->take_kf) {
        bs_tcid tcidl;
        const float min_d2 = (float)(a->cfg.min_triangulation_dist * a->cfg.min_triangulation_dist);
        kobs* kp = (kobs*)malloc(sizeof(kobs) * (size_t)(2 * (a->n_prev + 1)));
        took_kf = 1;
        v->take_kf = 0;
        v->frames_after_kf = 0;
        bs_vio_kf_insert(v, v->last_state_t_ns);
        tcidl.frame_id = t_ns; tcidl.cam_id = 0;
        for (i = 0; i < n_unc; ++i) {
            const int64_t lm_id = unc_ids[i];
            int nk = 0, valid_kp = 0, j;
            for (f = 0; f < a->n_prev; ++f)
                for (k = 0; k < 2; ++k) {
                    const bs_flow_kp* it = obs_find(&a->prev[f].obs[k], lm_id);
                    if (it) {
                        kp[nk].t.frame_id = a->prev[f].t_ns; kp[nk].t.cam_id = (uint64_t)k;
                        kp[nk].pos[0] = it->m[4]; kp[nk].pos[1] = it->m[5];
                        nk++;
                    }
                }
            for (j = 0; j < nk && !valid_kp; ++j) {
                const bs_flow_kp* o0 = obs_find(&obs[0], lm_id);
                const float* p1 = kp[j].pos;
                float p0[2], p0_3d[4], p1_3d[4], dist2, tri[4];
                bs_se3f Pl, Po, Pli, T_i0_i1, T_a, T_0_1;
                if (!o0) { fatal(a, "measure: .at(lm_id) of the current cam0 observations"); free(kp); return -1; }
                p0[0] = o0->m[4]; p0[1] = o0->m[5];
                if (!bs_ds_unproject_f(&v->ba.cam[0], p0, p0_3d, NULL, NULL)) continue;
                if (!bs_ds_unproject_f(&v->ba.cam[kp[j].t.cam_id], p1, p1_3d, NULL, NULL)) continue;
                if (!get_pose(v, tcidl.frame_id, &Pl) || !get_pose(v, kp[j].t.frame_id, &Po)) { fatal(a, "measure: Could not find pose"); free(kp); return -1; }
                bs_se3f_inverse(&Pl, &Pli);
                bs_se3f_mul(&Pli, &Po, &T_i0_i1);                                                    /* getPose().inverse() * getPose() */
                {
                    bs_se3f Tinv;
                    bs_se3f_inverse(&v->ba.T_i_c[0], &Tinv);
                    bs_se3f_mul(&Tinv, &T_i0_i1, &T_a);
                    bs_se3f_mul(&T_a, &v->ba.T_i_c[kp[j].t.cam_id], &T_0_1);                         /* T_i_c[0].inverse() * T_i0_i1 * T_i_c[cam] */
                }
                dist2 = bs_v3f_sqn(T_0_1.t);
                if (dist2 < min_d2) continue;
                bs_triangulate_f(p0_3d, p1_3d, &T_0_1, tri);
                a->stat_triangulate++;
                if (isfinite(tri[0]) && isfinite(tri[1]) && isfinite(tri[2]) && isfinite(tri[3]) && tri[3] > 0 && tri[3] < 3.0) {
                    float dir[2];
                    bs_stereographic_project_f(tri, dir);
                    bs_lmdb_add_landmark(&v->ba.lmdb, lm_id, dir, tri[3], tcidl);
                    n_added++;
                    valid_kp = 1;
                }
            }
            if (valid_kp)
                for (j = 0; j < nk; ++j) bs_lmdb_add_observation(&v->ba.lmdb, kp[j].t, lm_id, kp[j].pos);
        }
        free(kp);
        bs_vio_npk_set(v, t_ns, n_added);
        a->stat_kf++;
        a->stat_landmarks_added += n_added;
    } else {
        v->frames_after_kf++;
    }
    fi.take_kf = took_kf; fi.landmarks_added = n_added;

    if (a->cfg.marg_lost_landmarks) {
        const bs_hnode* n;
        int cap = 0;
        for (n = v->ba.lmdb.kpts.before_begin.next; n; n = n->next) {
            const int connected = obs_find(&obs[0], n->k0) != NULL || obs_find(&obs[1], n->k0) != NULL;
            if (!connected) {
                if (n_lost == cap) { cap = cap ? 2 * cap : 64; lost = (int64_t*)realloc(lost, sizeof(int64_t) * (size_t)cap); }
                lost[n_lost++] = n->k0;
            }
        }
    }
    fi.n_lost = n_lost; fi.lost_ids = lost; fi.unconnected_ids = unc_ids; fi.n_unconnected = n_unc; fi.t_ns = t_ns;

    /* optimize_and_marg */
    memset(&info, 0, sizeof info);
    bs_vio_optimize(v, &info, NULL, NULL);
    if (info.invalid_linearization || info.layout_error || info.nonfinite_increment) { fatal(a, "optimize: a condition under which the C++ aborts"); goto fail; }
    memset(&res, 0, sizeof res);
    bs_vio_marginalize(v, npc.e, npc.n, lost, n_lost, &res);
    if (res.layout_error) { fatal(a, "marginalize: a condition under which the C++ aborts"); bs_marg_result_free(&res); goto fail; }
    for (i = 0; i < res.n_states_all + res.n_poses_to_marg; ++i) {                      /* prev_opt_flow_res.erase(id) */
        const int64_t id = i < res.n_states_all ? res.states_to_marg_all[i] : res.poses_to_marg[i - res.n_states_all];
        for (f = 0; f < a->n_prev; ++f)
            if (a->prev[f].t_ns == id) {
                free(a->prev[f].obs[0].kp); free(a->prev[f].obs[1].kp);
                memmove(&a->prev[f], &a->prev[f + 1], sizeof(bs_frame_obs) * (size_t)(a->n_prev - f - 1));
                a->n_prev--;
                break;
            }
    }
    fi.opt = &info; fi.marg = &res;
    if (a->cb) a->cb(a->cb_ctx, &fi, a);
    bs_marg_result_free(&res);

    /* out_state_queue->push(PoseVelBiasState<double>(frame_states.at(last_state_t_ns).getState().cast<double>())) */
    {
        bs_frame_state* ls = find_state(v, v->last_state_t_ns);
        bs_se3f T;
        bs_se3d Td;
        if (!ls) { fatal(a, "measure: frame_states.at(last_state_t_ns)"); goto fail; }
        pvstate_to_se3(&state_get(ls)->s, &T);
        bs_se3_d_from_f(&T, &Td);
        out->t_ns = state_get(ls)->s.t_ns;
        out->t[0] = Td.t[0]; out->t[1] = Td.t[1]; out->t[2] = Td.t[2];
        out->q[0] = Td.so3.x; out->q[1] = Td.so3.y; out->q[2] = Td.so3.z; out->q[3] = Td.so3.w;
    }
    bs_htab_destroy(&unc, NULL);
    free(unc_ids); free(lost); free(npc.e);
    a->stat_frames++;
    return 0;
fail:
    bs_htab_destroy(&unc, NULL);
    free(unc_ids); free(lost); free(npc.e);
    return -1;
}

/* ------------------------------------------------------------------------------------------------ initialize() loop body */

int bs_app_frame(bs_app* a, int64_t t_ns, const bs_flow_obs obs[2], bs_app_out* out) {
    bs_vio* v = &a->v;
    bs_imu_meas meas;
    int have_meas = 0;
    if (a->fatal) return -1;
    if (!a->initialized) {
        const float zero3[3] = {0.0f, 0.0f, 0.0f}, unit_z[3] = {0.0f, 0.0f, 1.0f};
        bs_quatf q;
        bs_frame_state* st;
        bs_imu_meas* m0;
        while (a->data.t_ns < t_ns) {                                                   /* Skipping IMU data.. */
            imu_pop(a);
            if (!a->have_data) break;
        }
        if (!a->have_data) { fatal(a, "initialize: IMU data exhausted before the first frame (the C++ dereferences a null pointer)"); return -1; }
        if (!bs_quatf_from_two_vectors(a->data.accel, unit_z, &q)) { fatal(a, "initialize: FromTwoVectors antipodal branch (SVD) is not ported"); return -1; }
        v->last_state_t_ns = t_ns;
        m0 = bs_vio_imu_insert(v, t_ns);
        bs_imu_init(m0, t_ns, zero3, zero3);
        st = bs_vio_state_insert(v, t_ns);
        st->s.linearized = 1;
        {
            bs_pvbstate s0;
            memset(&s0, 0, sizeof s0);
            s0.s.t_ns = t_ns;
            bs_so3f so;
            bs_so3f_from_quat(&q, &so);                                                 /* T_w_i_init.setQuaternion(q) */
            s0.s.q[0] = so.x; s0.s.q[1] = so.y; s0.s.q[2] = so.z; s0.s.q[3] = so.w;
            s0.s.p[0] = s0.s.p[1] = s0.s.p[2] = 0.0f;
            st->s.lin = s0; st->s.cur = s0;
        }
        memset(st->delta, 0, sizeof st->delta);
        bs_vio_init_marg_prior(v, t_ns, a->cfg.init_pose_weight, a->cfg.init_ba_weight, a->cfg.init_bg_weight);
        a->initialized = 1;
    }

    if (a->have_prev) {
        const bs_frame_state* ls = find_state(v, v->last_state_t_ns);
        if (!ls) { fatal(a, "initialize: frame_states.at(last_state_t_ns)"); return -1; }
        bs_imu_init(&meas, a->prev_t_ns, state_get(ls)->bg, state_get(ls)->ba);
        have_meas = 1;
        if (!(a->prev_t_ns < t_ns)) { fatal(a, "duplicate frame timestamps?! zero time delta leads to invalid IMU integration."); return -1; }
        while (a->have_data && a->data.t_ns <= a->prev_t_ns) imu_pop(a);
        if (!a->have_data) { fatal(a, "initialize: IMU data exhausted while skipping (the C++ dereferences a null pointer)"); return -1; }
        while (a->data.t_ns <= t_ns) {
            bs_imu_integrate(&meas, &a->data, a->accel_cov, a->gyro_cov);
            imu_pop(a);
            if (!a->have_data) break;
        }
        if (meas.start_t_ns + meas.delta.t_ns < t_ns) {
            int64_t tmp;
            if (!a->have_data) return 1;                                                /* `if (!data.get()) break;` */
            tmp = a->data.t_ns;
            a->data.t_ns = t_ns;
            bs_imu_integrate(&meas, &a->data, a->accel_cov, a->gyro_cov);
            a->data.t_ns = tmp;
        }
    }

    if (measure(a, t_ns, obs, have_meas ? &meas : NULL, out)) return -1;
    a->have_prev = 1; a->prev_t_ns = t_ns;
    return 0;
}
