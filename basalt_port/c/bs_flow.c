/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Basalt pure-C port, module M5 (part 2): FrameToFrameOpticalFlow. See bs_flow.h. */
#include "bs_flow.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bs_cam.h"
#include "bs_lie.h"

struct bs_flow {
    bs_flow_config cfg;
    int ncam;
    bs_ds_f cam[2];
    float E[16];                 /* 4x4 column-major */
    float pat[2 * BS_PAT];
    bs_pyr pyr[2], old_pyr[2];
    int have_frame;
    int64_t t_ns;
    size_t frame_counter;
    uint64_t last_keypoint_id;
    bs_flow_obs obs[2];
};

/* ------------------------------------------------------------------ obs arrays */

static int obs_push(bs_flow_obs *o, uint64_t id, const float m[6]) {
    if (o->n == o->cap) {
        const int nc = o->cap ? 2 * o->cap : 256;
        bs_flow_kp *p = (bs_flow_kp *)realloc(o->kp, (size_t)nc * sizeof *p);
        if (!p) return 0;
        o->kp = p;
        o->cap = nc;
    }
    o->kp[o->n].id = id;
    memcpy(o->kp[o->n].m, m, 6 * sizeof(float));
    o->n++;
    return 1;
}

/* ------------------------------------------------------------------ config / calibration json (flat key scan) */

static char *slurp(const char *path) {
    FILE *f = fopen(path, "rb");
    long n;
    char *b;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    b = (char *)malloc((size_t)n + 1);
    if (b && fread(b, 1, (size_t)n, f) != (size_t)n) { free(b); b = NULL; }
    if (b) b[n] = 0;
    fclose(f);
    return b;
}

/* value of "key": after position *pos; advances *pos past the number. returns 0 on success */
static int scan_num(const char *buf, const char **pos, const char *key, double *out) {
    char pat[96];
    const char *q;
    char *end;
    snprintf(pat, sizeof pat, "\"%s\"", key);
    q = strstr(*pos, pat);
    if (!q) return 1;
    q += strlen(pat);
    while (*q == ' ' || *q == ':' || *q == '\t' || *q == '\n' || *q == '\r') q++;
    *out = strtod(q, &end);
    if (end == q) return 1;
    *pos = end;
    (void)buf;
    return 0;
}

void bs_flow_config_default(bs_flow_config *c) {
    c->pattern = 51;
    c->levels = 3;
    c->max_iterations = 5;
    c->grid_size = 50;
    c->skip_frames = 1;
    c->max_recovered_dist2 = 0.04f;
    c->epipolar_error = 0.005f;
}

int bs_flow_config_load(const char *path, bs_flow_config *c) {
    char *b = slurp(path);
    const char *p;
    double v;
    int bad = 0;
    if (!b) return 1;
#define NUM(key, dst, T) do { p = b; if (scan_num(b, &p, "config.optical_flow_" key, &v)) bad = 1; else (dst) = (T)v; } while (0)
    NUM("detection_grid_size", c->grid_size, int);
    NUM("max_recovered_dist2", c->max_recovered_dist2, float);   /* VioConfig stores float: the double 0.04 is rounded when loaded */
    NUM("pattern", c->pattern, int);
    NUM("max_iterations", c->max_iterations, int);
    NUM("epipolar_error", c->epipolar_error, float);
    NUM("levels", c->levels, int);
    NUM("skip_frames", c->skip_frames, int);
#undef NUM
    free(b);
    return bad;
}

int bs_flow_calib_load(const char *path, bs_flow_calib *c) {
    char *b = slurp(path);
    static const char *pk[7] = {"px", "py", "pz", "qx", "qy", "qz", "qw"};
    static const char *ik[6] = {"fx", "fy", "cx", "cy", "xi", "alpha"};
    const char *p;
    int cam, k, bad = 0;
    if (!b) return 1;
    c->ncam = 2;
    p = strstr(b, "\"T_imu_cam\"");
    if (!p) bad = 1;
    for (cam = 0; cam < 2 && !bad; cam++)
        for (k = 0; k < 7; k++) if (scan_num(b, &p, pk[k], &c->T_i_c[cam][k])) bad = 1;
    p = bad ? NULL : strstr(b, "\"intrinsics\"");
    if (!p) bad = 1;
    for (cam = 0; cam < 2 && !bad; cam++) {
        const char *q = strstr(p, "\"camera_type\"");
        if (!q || !strstr(q, "\"ds\"") || strstr(q, "\"ds\"") > strstr(q, "\"intrinsics\"")) { bad = 1; break; }
        p = q;
        for (k = 0; k < 6; k++) if (scan_num(b, &p, ik[k], &c->intr[cam][k])) bad = 1;
    }
    free(b);
    return bad;
}

/* computeEssential(T_0_1): E.setZero(); E.topLeftCorner<3,3>() = SO3d::hat(t.normalized()) * R   (double), then Ed.cast<float>() */
void bs_flow_essential(const bs_flow_calib *c, float E[16]) {
    bs_se3d T0, T1, inv0, Tij;
    double tn[3], hat[9], R[9], P[9];
    int i, j;
    {
        const double *a = c->T_i_c[0], *b = c->T_i_c[1];
        T0.t[0] = a[0]; T0.t[1] = a[1]; T0.t[2] = a[2];
        T0.so3.x = a[3]; T0.so3.y = a[4]; T0.so3.z = a[5]; T0.so3.w = a[6];
        T1.t[0] = b[0]; T1.t[1] = b[1]; T1.t[2] = b[2];
        T1.so3.x = b[3]; T1.so3.y = b[4]; T1.so3.z = b[5]; T1.so3.w = b[6];
    }
    bs_se3d_inverse(&T0, &inv0);
    bs_se3d_mul(&inv0, &T1, &Tij);
    bs_so3d_matrix(&Tij.so3, R);
    bs_v3d_normalized(Tij.t, tn);
    bs_so3d_hat(tn, hat);
    bs_m3d_mul(hat, R, P);
    for (i = 0; i < 16; i++) E[i] = 0.0f;
    for (j = 0; j < 3; j++)
        for (i = 0; i < 3; i++) E[i + 4 * j] = (float)P[i + 3 * j];
}

/* ------------------------------------------------------------------ tracking */

static bs_imgv lvl_view(const bs_pyr *p, int l) {
    bs_imgv v;
    v.p = bs_pyr_lvl(p, l, &v.w, &v.h);
    v.pitch = p->pitch;
    return v;
}

/* trackPoint: old_tr is the transform in the old image (translation = patch centre), tr in/out (starts as a copy of old_tr) */
int bs_flow_track_point(const bs_flow *f, const bs_pyr *old_pyr, const bs_pyr *pyr, const float old_tr[6], float tr[6]) {
    int valid = 1, level;
    float lin[4];
    tr[0] = 1.0f; tr[1] = 0.0f; tr[2] = 0.0f; tr[3] = 1.0f;       /* transform.linear().setIdentity() */
    for (level = f->cfg.levels; level >= 0 && valid; level--) {
        const float scale = (float)(1 << level);
        float pos[2];
        bs_patch p;
        bs_imgv v1 = lvl_view(old_pyr, level), v2;
        tr[4] = tr[4] / scale;
        tr[5] = tr[5] / scale;
        pos[0] = old_tr[4] / scale;
        pos[1] = old_tr[5] / scale;
        bs_patch_set(&p, &v1, f->pat, pos);
        valid &= p.valid;
        if (valid) {
            v2 = lvl_view(pyr, level);
            valid &= bs_track_point_at_level(&v2, &p, f->pat, f->cfg.max_iterations, tr);
        }
        tr[4] = tr[4] * scale;
        tr[5] = tr[5] * scale;
    }
    /* transform.linear() = old_transform.linear() * transform.linear() (2x2 * 2x2) */
    lin[0] = old_tr[0] * tr[0] + old_tr[2] * tr[1];
    lin[1] = old_tr[1] * tr[0] + old_tr[3] * tr[1];
    lin[2] = old_tr[0] * tr[2] + old_tr[2] * tr[3];
    lin[3] = old_tr[1] * tr[2] + old_tr[3] * tr[3];
    memcpy(tr, lin, sizeof lin);
    return valid;
}

int bs_flow_track_points(const bs_flow *f, const bs_pyr *pyr1, const bs_pyr *pyr2, const bs_flow_obs *in, bs_flow_obs *out) {
    int r;
    out->n = 0;
    for (r = 0; r < in->n; r++) {
        const float *t1 = in->kp[r].m;
        float t2[6], t1rec[6];
        int valid;
        memcpy(t2, t1, sizeof t2);
        valid = bs_flow_track_point(f, pyr1, pyr2, t1, t2);
        if (valid) {
            memcpy(t1rec, t2, sizeof t1rec);
            valid = bs_flow_track_point(f, pyr2, pyr1, t2, t1rec);
            if (valid) {
                const float dx = t1[4] - t1rec[4], dy = t1[5] - t1rec[5];
                const float dist2 = dx * dx + dy * dy;
                if (dist2 < f->cfg.max_recovered_dist2)
                    if (!obs_push(out, in->kp[r].id, t2)) return BS_IMG_NOMEM;
            }
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ addPoints / filterPoints */

static int add_points(bs_flow *f) {
    bs_flow_obs new0 = {0, 0, 0}, new1 = {0, 0, 0};
    const bs_pyr *p0 = &f->pyr[0];
    double *cur = NULL, *out = NULL;
    int n0 = f->obs[0].n, i, n, cap, rc = 0, lw, lh;
    const uint16_t *l0 = bs_pyr_lvl(p0, 0, &lw, &lh);
    cap = (lh / f->cfg.grid_size + 2) * (lw / f->cfg.grid_size + 2);
    cur = (double *)malloc(sizeof(double) * 2 * (size_t)(n0 + 1));
    out = (double *)malloc(sizeof(double) * 2 * (size_t)cap);
    if (!cur || !out) { rc = BS_IMG_NOMEM; goto done; }
    for (i = 0; i < n0; i++) { cur[2 * i] = (double)f->obs[0].kp[i].m[4]; cur[2 * i + 1] = (double)f->obs[0].kp[i].m[5]; }
    n = bs_detect_keypoints(l0, p0->pitch, lw, lh, f->cfg.grid_size, 1, cur, n0, out, cap);
    if (n < 0) { rc = 10; goto done; }
    for (i = 0; i < n; i++) {
        float m[6] = {1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
        m[4] = (float)out[2 * i];
        m[5] = (float)out[2 * i + 1];
        if (!obs_push(&f->obs[0], f->last_keypoint_id, m) || !obs_push(&new0, f->last_keypoint_id, m)) { rc = BS_IMG_NOMEM; goto done; }
        f->last_keypoint_id++;
    }
    if (f->ncam > 1) {
        rc = bs_flow_track_points(f, &f->pyr[0], &f->pyr[1], &new0, &new1);
        for (i = 0; !rc && i < new1.n; i++)        /* observations.at(1).emplace(kv): the ids are new, hence larger than all present ones */
            if (!obs_push(&f->obs[1], new1.kp[i].id, new1.kp[i].m)) rc = BS_IMG_NOMEM;
    }
done:
    free(cur); free(out); free(new0.kp); free(new1.kp);
    return rc;
}

/* p0.transpose() * E * p1 = (p0^T E) * p1: the 1x4 temporary p0^T E has packet dots (l0 + l2) + (l1 + l3), then the same dot with p1 */
float bs_flow_epipolar(const float E[16], const float p0[4], const float p1[4]) {
    float t[4];
    int k;
    for (k = 0; k < 4; k++) {
        const float *c = &E[4 * k];
        t[k] = (p0[0] * c[0] + p0[2] * c[2]) + (p0[1] * c[1] + p0[3] * c[3]);
    }
    return (t[0] * p1[0] + t[2] * p1[2]) + (t[1] * p1[1] + t[3] * p1[3]);
}

static void filter_points(bs_flow *f) {
    int i, j, n1 = f->obs[1].n, m = 0;
    int *rm;
    if (f->ncam < 2 || n1 == 0) return;
    rm = (int *)calloc((size_t)n1, sizeof(int));
    if (!rm) return;
    /* kpid / proj0 / proj1 in obs[1] order, matched against obs[0] (both id-sorted: merge walk = std::map::find) */
    j = 0;
    for (i = 0; i < n1; i++) {
        const bs_flow_kp *k1 = &f->obs[1].kp[i];
        float p0[4], p1[4], e;
        int ok0, ok1;
        while (j < f->obs[0].n && f->obs[0].kp[j].id < k1->id) j++;
        if (j >= f->obs[0].n || f->obs[0].kp[j].id != k1->id) continue;
        {
            const float x0[2] = {f->obs[0].kp[j].m[4], f->obs[0].kp[j].m[5]}, x1[2] = {k1->m[4], k1->m[5]};
            ok0 = bs_ds_unproject_f(&f->cam[0], x0, p0, NULL, NULL);
            ok1 = bs_ds_unproject_f(&f->cam[1], x1, p1, NULL, NULL);
        }
        if (ok0 && ok1) {
            e = bs_flow_epipolar(f->E, p0, p1);
            if ((double)fabsf(e) > (double)f->cfg.epipolar_error) { rm[i] = 1; m++; }
        } else {
            rm[i] = 1; m++;
        }
    }
    if (m) {
        int w = 0;
        for (i = 0; i < n1; i++)
            if (!rm[i]) f->obs[1].kp[w++] = f->obs[1].kp[i];
        f->obs[1].n = w;
    }
    free(rm);
}

/* ------------------------------------------------------------------ frame */

bs_flow *bs_flow_new(const bs_flow_config *cfg, const bs_flow_calib *cal) {
    bs_flow *f = (bs_flow *)calloc(1, sizeof *f);
    int i;
    if (!f) return NULL;
    f->cfg = *cfg;
    f->ncam = cal->ncam;
    if (f->ncam < 1 || f->ncam > 2 || !bs_pattern_init(f->pat, cfg->pattern)) { free(f); return NULL; }
    for (i = 0; i < f->ncam; i++) bs_ds_cast_f(&f->cam[i], cal->intr[i]);
    if (f->ncam > 1) bs_flow_essential(cal, f->E);
    return f;
}

void bs_flow_free(bs_flow *f) {
    int i;
    if (!f) return;
    for (i = 0; i < 2; i++) { bs_pyr_free(&f->pyr[i]); bs_pyr_free(&f->old_pyr[i]); free(f->obs[i].kp); }
    free(f);
}

int bs_flow_process(bs_flow *f, int64_t t_ns, const uint16_t *const *img, int w, int h) {
    int i, rc;
    if (w < 32 || h < 32) return 10;
    f->t_ns = t_ns;
    if (!f->have_frame) {
        for (i = 0; i < f->ncam; i++) if ((rc = bs_pyr_set(&f->pyr[i], img[i], w, h, f->cfg.levels))) return rc;
        f->have_frame = 1;
        f->obs[0].n = f->obs[1].n = 0;
    } else {
        bs_flow_obs nw[2] = {{0, 0, 0}, {0, 0, 0}};
        bs_pyr tmp;
        for (i = 0; i < f->ncam; i++) { tmp = f->old_pyr[i]; f->old_pyr[i] = f->pyr[i]; f->pyr[i] = tmp; }   /* old_pyramid = pyramid */
        for (i = 0; i < f->ncam; i++) if ((rc = bs_pyr_set(&f->pyr[i], img[i], w, h, f->cfg.levels))) return rc;
        for (i = 0; i < f->ncam; i++) {
            nw[i].kp = NULL; nw[i].n = nw[i].cap = 0;
            rc = bs_flow_track_points(f, &f->old_pyr[i], &f->pyr[i], &f->obs[i], &nw[i]);
            if (rc) { free(nw[0].kp); free(nw[1].kp); return rc; }
        }
        for (i = 0; i < f->ncam; i++) { free(f->obs[i].kp); f->obs[i] = nw[i]; }
    }
    if ((rc = add_points(f))) return rc;
    filter_points(f);
    f->frame_counter++;
    return 0;
}

const bs_flow_obs *bs_flow_result(const bs_flow *f, int cam) { return &f->obs[cam]; }
uint64_t bs_flow_last_keypoint_id(const bs_flow *f) { return f->last_keypoint_id; }
