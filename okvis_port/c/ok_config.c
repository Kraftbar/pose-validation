/* SPDX-License-Identifier: BSD-3-Clause */
/* OKVIS2 pure-C port, module 8: configuration reader (see ok_config.h for scope and licence). */
#include "ok_config.h"
#include "ok_cam.h"
#include "ok_kin.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------ YAML subset -> tree */
enum { Y_SCALAR, Y_SEQ, Y_MAP };
typedef struct yn {
    int kind;
    char* s;                    /* scalar text (trimmed) */
    int n, cap;
    char** keys;                /* map keys */
    struct yn** v;              /* seq items / map values */
} yn;

typedef struct ytext { char* t; size_t n, pos; } ytext;

static yn* yn_new(int kind) { yn* y = (yn*)calloc(1, sizeof *y); y->kind = kind; return y; }
static void yn_add(yn* y, char* key, yn* v) {
    if (y->n == y->cap) {
        y->cap = y->cap ? 2 * y->cap : 8;
        y->v = (yn**)realloc(y->v, sizeof(yn*) * (size_t)y->cap);
        y->keys = (char**)realloc(y->keys, sizeof(char*) * (size_t)y->cap);
    }
    y->keys[y->n] = key; y->v[y->n] = v; y->n++;
}
static void yn_free(yn* y) {
    int i;
    if (!y) return;
    for (i = 0; i < y->n; ++i) { free(y->keys[i]); yn_free(y->v[i]); }
    free(y->keys); free(y->v); free(y->s); free(y);
}
static char* trimdup(const char* a, const char* b) {
    char* s;
    while (a < b && isspace((unsigned char)*a)) a++;
    while (b > a && isspace((unsigned char)b[-1])) b--;
    s = (char*)malloc((size_t)(b - a) + 1);
    memcpy(s, a, (size_t)(b - a)); s[b - a] = 0;
    return s;
}
static yn* scalar(const char* a, const char* b) { yn* y = yn_new(Y_SCALAR); y->s = trimdup(a, b); return y; }

static int at_end(const ytext* x) { return x->pos >= x->n; }
static char cur(const ytext* x) { return at_end(x) ? 0 : x->t[x->pos]; }
static void skip_sp(ytext* x) { while (!at_end(x) && (cur(x) == ' ' || cur(x) == '\t' || cur(x) == '\r')) x->pos++; }
static void skip_ws(ytext* x) { while (!at_end(x) && isspace((unsigned char)cur(x))) x->pos++; }
static void to_eol(ytext* x) { while (!at_end(x) && cur(x) != '\n') x->pos++; if (!at_end(x)) x->pos++; }
/* the next line with content: returns its indentation (-1 at the end) and leaves pos at its start */
static int next_line(ytext* x) {
    for (;;) {
        size_t p = x->pos;
        int ind = 0;
        if (at_end(x)) return -1;
        while (p < x->n && (x->t[p] == ' ' || x->t[p] == '\t')) { p++; ind++; }
        if (p < x->n && x->t[p] != '\n' && x->t[p] != '\r') return ind;
        while (p < x->n && x->t[p] != '\n') p++;
        x->pos = p < x->n ? p + 1 : p;
    }
}

static yn* parse_flow(ytext* x) {
    skip_ws(x);
    if (cur(x) == '[') {
        yn* y = yn_new(Y_SEQ);
        x->pos++;
        for (;;) {
            skip_ws(x);
            if (at_end(x)) break;
            if (cur(x) == ']') { x->pos++; break; }
            yn_add(y, NULL, parse_flow(x));
            skip_ws(x);
            if (cur(x) == ',') x->pos++;
        }
        return y;
    }
    if (cur(x) == '{') {
        yn* y = yn_new(Y_MAP);
        x->pos++;
        for (;;) {
            size_t k0;
            skip_ws(x);
            if (at_end(x)) break;
            if (cur(x) == '}') { x->pos++; break; }
            k0 = x->pos;
            while (!at_end(x) && cur(x) != ':' && cur(x) != '}') x->pos++;
            if (cur(x) != ':') break;
            { char* key = trimdup(x->t + k0, x->t + x->pos); x->pos++; yn_add(y, key, parse_flow(x)); }
            skip_ws(x);
            if (cur(x) == ',') x->pos++;
        }
        return y;
    }
    {
        const size_t a = x->pos;
        while (!at_end(x) && cur(x) != ',' && cur(x) != ']' && cur(x) != '}' && cur(x) != '\n') x->pos++;
        return scalar(x->t + a, x->t + x->pos);
    }
}

/* value after "key:" or "- " on the current line */
static yn* parse_block(ytext* x, int ind);
static yn* parse_value(ytext* x, int ind) {
    skip_sp(x);
    if (at_end(x) || cur(x) == '\n') {
        int ci;
        to_eol(x);
        ci = next_line(x);
        if (ci > ind) {
            const char c0 = x->t[x->pos + (size_t)ci];
            if (c0 == '[' || c0 == '{') { yn* y; x->pos += (size_t)ci; y = parse_flow(x); to_eol(x); return y; }
            return parse_block(x, ci);
        }
        return scalar("", "");
    }
    if (cur(x) == '[' || cur(x) == '{') { yn* y = parse_flow(x); to_eol(x); return y; }
    { const size_t a = x->pos; while (!at_end(x) && cur(x) != '\n') x->pos++; { yn* y = scalar(x->t + a, x->t + x->pos); to_eol(x); return y; } }
}

static yn* parse_block(ytext* x, int ind) {
    yn* y = NULL;
    for (;;) {
        const int li = next_line(x);
        if (li != ind) break;
        x->pos += (size_t)li;
        if (cur(x) == '-' && (x->pos + 1 >= x->n || isspace((unsigned char)x->t[x->pos + 1]))) {
            if (!y) y = yn_new(Y_SEQ);
            if (y->kind != Y_SEQ) break;
            x->pos++;
            yn_add(y, NULL, parse_value(x, ind));
        } else {
            const size_t k0 = x->pos;
            if (!y) y = yn_new(Y_MAP);
            if (y->kind != Y_MAP) break;
            while (!at_end(x) && cur(x) != ':' && cur(x) != '\n') x->pos++;
            if (cur(x) != ':') { to_eol(x); continue; }
            { char* key = trimdup(x->t + k0, x->t + x->pos); x->pos++; yn_add(y, key, parse_value(x, ind)); }
        }
    }
    return y ? y : scalar("", "");
}

static yn* yparse_file(const char* path) {
    FILE* f = fopen(path, "rb");
    ytext x;
    size_t i;
    int line_start = 1;
    yn* root;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END); x.n = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    x.t = (char*)malloc(x.n + 1);
    if (fread(x.t, 1, x.n, f) != x.n) { fclose(f); free(x.t); return NULL; }
    fclose(f);
    x.t[x.n] = 0;
    /* blank out comments and the %YAML directive */
    for (i = 0; i < x.n; ++i) {
        const char c = x.t[i];
        if (c == '\n') { line_start = 1; continue; }
        if ((c == '#' && (line_start || isspace((unsigned char)x.t[i - 1]) || x.t[i - 1] == ',')) || (c == '%' && line_start)) {
            while (i < x.n && x.t[i] != '\n') x.t[i++] = ' ';
            line_start = 1;
            continue;
        }
        if (!isspace((unsigned char)c)) line_start = 0;
    }
    x.pos = 0;
    root = parse_block(&x, next_line(&x) < 0 ? 0 : next_line(&x));
    free(x.t);
    return root;
}

static const yn* yget(const yn* y, const char* key) {
    int i;
    if (!y || y->kind != Y_MAP) return NULL;
    for (i = 0; i < y->n; ++i) if (!strcmp(y->keys[i], key)) return y->v[i];
    return NULL;
}

/* ------------------------------------------------------------------ typed reads (ViParametersReader::parseEntry) */
typedef struct rd { char* err; size_t errlen; int bad; } rd;
static void fail(rd* r, const char* what, const char* name) {
    if (!r->bad) snprintf(r->err, r->errlen, "%s parameter %s", what, name);
    r->bad = 1;
}
static int num(const yn* y, double* out) {
    char* e;
    if (!y || y->kind != Y_SCALAR || !y->s[0]) return 0;
    *out = strtod(y->s, &e);
    while (*e && isspace((unsigned char)*e)) e++;
    return *e == 0;
}
static double rdd(rd* r, const yn* m, const char* name) {
    double v = 0.0;
    if (!num(yget(m, name), &v)) fail(r, "missing real", name);
    return v;
}
static int rdi(rd* r, const yn* m, const char* name) {
    const yn* y = yget(m, name);
    char* e;
    long v;
    if (!y || y->kind != Y_SCALAR || !y->s[0]) { fail(r, "missing integer", name); return 0; }
    v = strtol(y->s, &e, 10);
    if (*e) fail(r, "missing integer", name);
    return (int)v;
}
static int rdb(rd* r, const yn* m, const char* name) {
    const yn* y = yget(m, name);
    char w[16];
    char* e;
    long v;
    size_t i;
    if (!y || y->kind != Y_SCALAR || !y->s[0]) { fail(r, "missing boolean", name); return 0; }
    v = strtol(y->s, &e, 10);
    if (e != y->s && *e == 0) return v != 0;
    for (i = 0; i + 1 < sizeof w && y->s[i] && y->s[i] != ' '; ++i) w[i] = (char)tolower((unsigned char)y->s[i]);
    w[i] = 0;
    if (!strcmp(w, "false") || !strcmp(w, "no") || !strcmp(w, "n") || !strcmp(w, "off")) return 0;
    if (!strcmp(w, "true") || !strcmp(w, "yes") || !strcmp(w, "y") || !strcmp(w, "on")) return 1;
    fail(r, "uninterpretable boolean", name);
    return 0;
}
static void rdv(rd* r, const yn* m, const char* name, double* out, int n) {
    const yn* y = yget(m, name);
    int i;
    if (!y || y->kind != Y_SEQ || y->n < n) { fail(r, "missing real array", name); return; }
    for (i = 0; i < n; ++i) if (!num(y->v[i], &out[i])) fail(r, "bad number in", name);
}
/* Transformation(Matrix4d) from 16 row-major numbers -> r, q coefficients */
static void m4_tf(const double rowmajor[16], ok_tf* t) {
    double m[16];
    int i, j;
    for (i = 0; i < 4; ++i) for (j = 0; j < 4; ++j) m[i + 4 * j] = rowmajor[4 * i + j];
    ok_tf_from_m4(t, m, 1);
}

int ok_cfg_load(const char* path, ok_cfg* c, char* err, size_t errlen) {
    yn* root = yparse_file(path);
    const yn *cams, *cp, *oc, *imu, *fe, *es;
    rd r;
    int i;
    memset(c, 0, sizeof *c);
    r.err = err; r.errlen = errlen; r.bad = 0;
    if (err && errlen) err[0] = 0;
    if (!root) { snprintf(err, errlen, "cannot read %s", path); return -1; }
    cams = yget(root, "cameras");
    if (!cams || cams->kind != Y_SEQ || cams->n < 1 || cams->n > OK_CFG_MAXCAM) { fail(&r, "missing", "cameras"); goto done; }
    c->ncam = cams->n;
    for (i = 0; i < cams->n; ++i) {
        const yn* cm = cams->v[i];
        ok_cfg_cam* k = &c->cam[i];
        double T[16], dim[2], f[2], pp[2];
        const yn* dt = yget(cm, "distortion_type");
        const yn* su = yget(cm, "slam_use");
        ok_tf t0, t1;
        ok_quat qn;
        rdv(&r, cm, "T_SC", T, 16); rdv(&r, cm, "image_dimension", dim, 2); rdv(&r, cm, "distortion_coefficients", k->d, 4);
        rdv(&r, cm, "focal_length", f, 2); rdv(&r, cm, "principal_point", pp, 2);
        if (r.bad) goto done;
        if (!dt || dt->kind != Y_SCALAR) { fail(&r, "missing", "distortion_type"); goto done; }
        if (!strcmp(dt->s, "radialtangential") || !strcmp(dt->s, "plumb_bob")) k->dist = OK_CAM_RADTAN;
        else if (!strcmp(dt->s, "equidistant")) k->dist = OK_CAM_EQUIDISTANT;
        else { fail(&r, "unsupported distortion_type (radialtangential8 is not ported)", dt->s); goto done; }
        k->w = (int)dim[0]; k->h = (int)dim[1];
        k->fu = f[0]; k->fv = f[1]; k->cu = pp[0]; k->cv = pp[1];
        k->used = su && su->kind == Y_SCALAR ? strncmp(su->s, "okvis", 5) == 0 : 1;
        m4_tf(T, &t0);                                              /* calib.T_SC = Transformation(T_SC) */
        qn = ok_quat_normalized(t0.q);
        ok_tf_from_rq(&t1, t0.r, &qn, 1);                           /* Transformation(r, q.normalized()) */
        k->T_SC[0] = t1.r[0]; k->T_SC[1] = t1.r[1]; k->T_SC[2] = t1.r[2];
        k->T_SC[3] = t1.q.x; k->T_SC[4] = t1.q.y; k->T_SC[5] = t1.q.z; k->T_SC[6] = t1.q.w;
    }
    cp = yget(root, "camera_parameters");
    c->timestamp_tolerance = rdd(&r, cp, "timestamp_tolerance");
    { const yn* s = yget(cp, "sync_cameras");
      if (!s || s->kind != Y_SEQ) fail(&r, "missing real array", "sync_cameras");
      else for (i = 0; i < s->n && c->nsync < OK_CFG_MAXCAM; ++i) { double v; if (num(s->v[i], &v)) c->sync_cameras[c->nsync++] = (int)v; } }
    c->image_delay = rdd(&r, cp, "image_delay");
    oc = yget(cp, "online_calibration");
    c->do_extrinsics = rdb(&r, oc, "do_extrinsics");
    c->sigma_r = rdd(&r, oc, "sigma_r");
    c->sigma_alpha = rdd(&r, oc, "sigma_alpha");
    imu = yget(root, "imu_parameters");
    c->imu.use = rdb(&r, imu, "use");
    { double T[16]; ok_tf t; rdv(&r, imu, "T_BS", T, 16); m4_tf(T, &t);
      c->imu.T_BS[0] = t.r[0]; c->imu.T_BS[1] = t.r[1]; c->imu.T_BS[2] = t.r[2];
      c->imu.T_BS[3] = t.q.x; c->imu.T_BS[4] = t.q.y; c->imu.T_BS[5] = t.q.z; c->imu.T_BS[6] = t.q.w; }
    c->imu.a_max = rdd(&r, imu, "a_max"); c->imu.g_max = rdd(&r, imu, "g_max");
    c->imu.sigma_g_c = rdd(&r, imu, "sigma_g_c"); c->imu.sigma_bg = rdd(&r, imu, "sigma_bg");
    c->imu.sigma_a_c = rdd(&r, imu, "sigma_a_c"); c->imu.sigma_ba = rdd(&r, imu, "sigma_ba");
    c->imu.sigma_gw_c = rdd(&r, imu, "sigma_gw_c"); c->imu.sigma_aw_c = rdd(&r, imu, "sigma_aw_c");
    rdv(&r, imu, "a0", c->imu.a0, 3); rdv(&r, imu, "g0", c->imu.g0, 3);
    c->imu.g = rdd(&r, imu, "g");
    fe = yget(root, "frontend_parameters");
    c->detection_threshold = rdd(&r, fe, "detection_threshold"); c->absolute_threshold = rdd(&r, fe, "absolute_threshold");
    c->matching_threshold = rdd(&r, fe, "matching_threshold"); c->octaves = rdi(&r, fe, "octaves");
    c->max_num_keypoints = rdi(&r, fe, "max_num_keypoints"); c->keyframe_overlap = rdd(&r, fe, "keyframe_overlap");
    c->use_cnn = rdb(&r, fe, "use_cnn"); c->parallelise_detection = rdb(&r, fe, "parallelise_detection");
    c->num_matching_threads = rdi(&r, fe, "num_matching_threads");
    es = yget(root, "estimator_parameters");
    c->num_keyframes = rdi(&r, es, "num_keyframes"); c->num_loop_closure_frames = rdi(&r, es, "num_loop_closure_frames");
    c->num_imu_frames = rdi(&r, es, "num_imu_frames"); c->do_loop_closures = rdb(&r, es, "do_loop_closures");
    c->do_final_ba = rdb(&r, es, "do_final_ba"); c->enforce_realtime = rdb(&r, es, "enforce_realtime");
    c->realtime_min_iterations = rdi(&r, es, "realtime_min_iterations"); c->realtime_max_iterations = rdi(&r, es, "realtime_max_iterations");
    c->realtime_time_limit = rdd(&r, es, "realtime_time_limit"); c->realtime_num_threads = rdi(&r, es, "realtime_num_threads");
    c->full_graph_iterations = rdi(&r, es, "full_graph_iterations"); c->full_graph_num_threads = rdi(&r, es, "full_graph_num_threads");
    c->p_dbow = rdd(&r, es, "p_dbow"); c->drift_percentage = rdd(&r, es, "drift_percentage_heuristic");
    {   /* OKVIS2-X: gps_parameters (ViParametersReader::getGpsCalibration; a map block declares a GPS) */
        const yn* gp = yget(root, "gps_parameters");
        c->has_gps = 0;
        if (gp && gp->kind == Y_MAP) {
            const yn* ty = yget(gp, "data_type");
            c->has_gps = 1;
            if (!ty || ty->kind != Y_SCALAR) fail(&r, "missing string", "data_type");
            else { strncpy(c->gps_type, ty->s, sizeof c->gps_type - 1); c->gps_cartesian = !strcmp(ty->s, "cartesian"); }
            rdv(&r, gp, "r_SA", c->gps_r_SA, 3);
            c->gps_yaw_error_threshold = rdd(&r, gp, "yaw_error_threshold");
            c->gps_robust_init = rdb(&r, gp, "robust_gps_init");
        }
    }
done:
    yn_free(root);
    return r.bad ? -1 : 0;
}
