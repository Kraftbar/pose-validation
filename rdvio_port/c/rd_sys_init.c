/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port, module M9: Initializer. See rd_sys_init.h. */
#include "rd_sys_init.h"
#include "rd_geom.h"
#include "rd_lie.h"
#include "rd_qr.h"
#include "rd_ransac.h"
#include "rd_solver_glue.h"
#include "rd_sys_eigen.h"
#include "../../okvis_port/c/ok_kin.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

void rd_init_create(rd_init* in, const rd_cfg* cfg) { memset(in, 0, sizeof *in); in->cfg = cfg; }
void rd_init_destroy(rd_init* in) { rd_map_free(in->map); in->map = NULL; free(in->vel); in->vel = NULL; }
rd_map* rd_init_take_map(rd_init* in) { rd_map* m = in->map; in->map = NULL; return m; }

void rd_frame_set_pose(rd_frame* f, const ok_quat* sensor_q, const double sensor_p[3], const ok_quat* q, const double p[3]) {
    const ok_quat sc = rd_quat_conj(*sensor_q);
    double r[3];
    ok_quat_mul(q, &sc, &f->pose_q);
    rd_quat_rotate(&f->pose_q, sensor_p, r);
    f->pose_p[0] = p[0] - r[0]; f->pose_p[1] = p[1] - r[1]; f->pose_p[2] = p[2] - r[2];
}
static void cam_pose(const rd_frame* f, ok_quat* q, double p[3]) { rd_frame_get_pose(&f->pose_q, f->pose_p, &f->cam_q, f->cam_p, q, p); }
static void imu_pose(const rd_frame* f, ok_quat* q, double p[3]) { rd_frame_get_pose(&f->pose_q, f->pose_p, &f->imu_q, f->imu_p, q, p); }
static void obs_of(const rd_frame* f, size_t kp, rd_obs* o) {
    o->pose_q = f->pose_q; memcpy(o->pose_p, f->pose_p, sizeof o->pose_p);
    o->cam_q = f->cam_q; memcpy(o->cam_p, f->cam_p, sizeof o->cam_p);
    memcpy(o->keypoint, f->bearing + 3 * kp, sizeof o->keypoint);
}
/* Track::set_landmark_point (first keypoint) */
static void set_landmark_point(rd_track* t, const double p[3]) { rd_obs o; obs_of(t->ref[0].frame, t->ref[0].kp, &o); t->inv_depth = rd_track_set_landmark_point(&o, p); }
/* Track::triangulate: observations in keypoint_map order; sets m_life = 1 when valid */
static int track_triangulate(rd_track* t, double p[3]) {
    rd_obs* o = (rd_obs*)malloc(sizeof(rd_obs) * (t->nref ? t->nref : 1));
    size_t i;
    int ok;
    for (i = 0; i < t->nref; ++i) obs_of(t->ref[i].frame, t->ref[i].kp, &o[i]);
    ok = rd_track_triangulate((int)t->nref, o, p);
    free(o);
    if (ok) t->life = 1;
    return ok;
}
#define HAS(t, tag) (((t)->tags & RD_TAG(tag)) != 0)
static void set_tag(uint32_t* tags, int tag, int on) { if (on) *tags |= RD_TAG(tag); else *tags &= ~RD_TAG(tag); }

/* ------------------------------------------------------------------------------------------------------------------ mirror */
void rd_init_mirror_keyframe_map(rd_init* in, rd_map* ft, uint64_t init_frame_id) {
    const size_t last = rd_map_frame_index_by_id(ft, init_frame_id);
    const size_t gap = in->cfg->initializer_keyframe_gap, num = in->cfg->initializer_keyframe_num;
    const size_t distance = gap * (num - 1);
    size_t* idx;
    size_t i, j;
    rd_map* m;
    if (last < distance) { rd_map_free(in->map); in->map = NULL; return; }
    idx = (size_t*)malloc(sizeof(size_t) * num);
    for (i = 0; i < num; ++i) idx[i] = last - distance + i * gap;
    m = rd_map_new();                                 /* map = std::make_unique<Map>(): the new map exists before the old dies */
    rd_map_free(in->map);
    in->map = m;
    for (i = 0; i < num; ++i) rd_map_attach_frame(m, rd_frame_clone(rd_map_get_frame(ft, idx[i])), RD_NIL);
    for (j = 1; j < rd_map_frame_num(m); ++j) {
        rd_frame* oi = rd_map_get_frame(ft, idx[j - 1]);
        rd_frame* oj = rd_map_get_frame(ft, idx[j]);
        rd_frame* ni = rd_map_get_frame(m, j - 1);
        rd_frame* nj = rd_map_get_frame(m, j);
        size_t ki, f;
        for (ki = 0; ki < oi->nkp; ++ki) {
            rd_track* t = oi->track[ki];
            if (t) {
                const size_t kj = rd_track_keypoint_index(t, oj);
                if (kj != RD_NIL) rd_track_add_keypoint(rd_frame_get_track(ni, ki, NULL), nj, kj);
            }
        }
        nj->ndata = 0;
        for (f = idx[j - 1]; f < idx[j]; ++f) {
            const rd_frame* of = rd_map_get_frame(ft, f + 1);
            rd_imu_list_insert(&nj->data, &nj->ndata, &nj->cdata, nj->ndata, of->data, of->ndata);
        }
    }
    free(idx);
}

/* ------------------------------------------------------------------------------------------------------------------ init_sfm */
/* (P * q) for a 3x4 column-major P: rows 0-1 left folds, row 2 (p0 + p1) + (p2 + p3) */
static void p34_mul(const double P[12], const double q[4], double out[3]) {
    int r;
    for (r = 0; r < 2; ++r) out[r] = ((P[r] * q[0] + P[r + 3] * q[1]) + P[r + 6] * q[2]) + P[r + 9] * q[3];
    out[2] = (P[2] * q[0] + P[5] * q[1]) + (P[8] * q[2] + P[11] * q[3]);
}
static void m3_transpose(const double R[9], double Rt[9]) { int i, j; for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) Rt[i + 3 * j] = R[j + 3 * i]; }

static int init_sfm(rd_init* in) {
    rd_map* m = in->map;
    rd_frame* fi = rd_map_get_frame(m, 0);
    rd_frame* fj = rd_map_get_frame(m, rd_map_frame_num(m) - 1);
    const size_t n0 = fi->nkp;
    double* pi = (double*)malloc(sizeof(double) * 2 * (n0 ? n0 : 1));
    double* pj = (double*)malloc(sizeof(double) * 2 * (n0 ? n0 : 1));
    size_t* mi = (size_t*)malloc(sizeof(size_t) * (n0 ? n0 : 1));
    char* mask = (char*)malloc(n0 ? n0 : 1);
    double Rs[8][9], Ts[8][3], H[9], E[9], RH1[9], RH2[9], TH1[3], TH2[3], nH1[3], nH2[3], RE1[9], RE2[9], TE[3];
    double* pts[8]; char* st[8]; size_t cnt[8]; double score[8];
    double total_parallax = 0, tmp[3];
    int common = 0, ok = 0;
    size_t n = 0, k, best = 0, h, i, j;
    for (i = 0; i < 8; ++i) { pts[i] = NULL; st[i] = NULL; }
    for (k = 0; k < fi->nkp; ++k) {
        rd_track* t = fi->track[k];
        size_t kj;
        double a[2], b[2], d0, d1;
        if (!t) continue;
        kj = rd_track_keypoint_index(t, fj);
        if (kj == RD_NIL) continue;
        pi[2 * n] = fi->bearing[3 * k] / fi->bearing[3 * k + 2]; pi[2 * n + 1] = fi->bearing[3 * k + 1] / fi->bearing[3 * k + 2];
        pj[2 * n] = fj->bearing[3 * kj] / fj->bearing[3 * kj + 2]; pj[2 * n + 1] = fj->bearing[3 * kj + 1] / fj->bearing[3 * kj + 2];
        mi[n] = k;
        rd_apply_k(fi->bearing + 3 * k, fi->K, a);
        rd_apply_k(fj->bearing + 3 * kj, fj->K, b);
        d0 = a[0] - b[0]; d1 = a[1] - b[1];
        total_parallax += sqrt(d0 * d0 + d1 * d1);
        common++; n++;
    }
    if (common < (int)in->cfg->initializer_min_matches) goto done;
    total_parallax /= (double)(common > 1 ? common : 1);
    if (total_parallax < in->cfg->initializer_min_parallax) goto done;
    rd_find_homography_matrix(n, pi, pj, mask, 0.7 / fi->K[0], 0.999, 1000, in->cfg->random, H);
    if (!rd_decompose_homography(H, RH1, RH2, TH1, TH2, nH1, nH2)) goto done;   /* pure rotation */
    memcpy(tmp, TH1, sizeof tmp); ok_v3_normalized(tmp, TH1);
    memcpy(tmp, TH2, sizeof tmp); ok_v3_normalized(tmp, TH2);
    rd_find_essential_matrix(n, pi, pj, mask, 0.7 / fi->K[0], 0.999, 1000, in->cfg->random, E);
    rd_decompose_essential(E, RE1, RE2, TE);
    memcpy(tmp, TE, sizeof tmp); ok_v3_normalized(tmp, TE);
    memcpy(Rs[0], RH1, 72); memcpy(Rs[1], RH1, 72); memcpy(Rs[2], RH2, 72); memcpy(Rs[3], RH2, 72);
    memcpy(Rs[4], RE1, 72); memcpy(Rs[5], RE1, 72); memcpy(Rs[6], RE2, 72); memcpy(Rs[7], RE2, 72);
    for (i = 0; i < 3; ++i) {
        Ts[0][i] = TH1[i]; Ts[1][i] = -TH1[i]; Ts[2][i] = TH2[i]; Ts[3][i] = -TH2[i];
        Ts[4][i] = TE[i]; Ts[5][i] = -TE[i]; Ts[6][i] = TE[i]; Ts[7][i] = -TE[i];
    }
    for (h = 0; h < 8; ++h) {                         /* [1.1] triangulation of every (R, T) */
        double P1[12], P2[12];
        pts[h] = (double*)calloc(3 * (n ? n : 1), sizeof(double));
        st[h] = (char*)calloc(n ? n : 1, 1);
        cnt[h] = 0; score[h] = 0;
        memset(P1, 0, sizeof P1); P1[0] = 1; P1[4] = 1; P1[8] = 1;
        memcpy(P2, Rs[h], 72); memcpy(P2 + 9, Ts[h], 24);
        for (j = 0; j < n; ++j) {
            const double hi[3] = {pi[2 * j], pi[2 * j + 1], 1.0}, hj[3] = {pj[2 * j], pj[2 * j + 1], 1.0};
            double q[4], q1[3], q2[3];
            rd_triangulate_point2(P1, P2, hi, hj, q);
            p34_mul(P1, q, q1);
            p34_mul(P2, q, q2);
            if (q1[2] * q[3] > 0 && q2[2] * q[3] > 0) {
                if (q1[2] / q[3] < 100 && q2[2] / q[3] < 100) {
                    double e0, e1, f0, f1;
                    pts[h][3 * j] = q[0] / q[3]; pts[h][3 * j + 1] = q[1] / q[3]; pts[h][3 * j + 2] = q[2] / q[3];
                    st[h][j] = 1;
                    cnt[h]++;
                    e0 = q1[0] / q1[2] - pi[2 * j]; e1 = q1[1] / q1[2] - pi[2 * j + 1];
                    f0 = q2[0] / q2[2] - pj[2 * j]; f1 = q2[1] / q2[2] - pj[2 * j + 1];
                    score[h] += 0.5 * ((e0 * e0 + e1 * e1) + (f0 * f0 + f1 * f1));
                }
            }
        }
        if (cnt[h] > in->cfg->initializer_min_triangulation && score[h] < score[best]) best = h;
        else if (cnt[h] > cnt[best]) best = h;
    }
    if (cnt[best] < in->cfg->initializer_min_triangulation) goto done;
    {   /* [2.1] init states */
        const ok_quat qid = {0, 0, 0, 1};
        const double zero[3] = {0, 0, 0};
        double Rt[9], v[3], p[3];
        ok_quat q;
        rd_frame_set_pose(fi, &fi->cam_q, fi->cam_p, &qid, zero);
        m3_transpose(Rs[best], Rt);
        q = ok_quat_from_mat3(Rt);                    /* pose.q = init_R.transpose() */
        ok_m3_mulv_lhsT(Rs[best], Ts[best], v);       /* init_R.transpose() * init_T */
        p[0] = -v[0]; p[1] = -v[1]; p[2] = -v[2];
        rd_frame_set_pose(fj, &fj->cam_q, fj->cam_p, &q, p);
        for (k = 0; k < n; ++k) {
            rd_track* t;
            if (st[best][k] == 0) continue;
            t = fi->track[mi[k]];
            set_landmark_point(t, pts[best] + 3 * k);
            set_tag(&t->tags, RD_TT_VALID, 1); set_tag(&t->tags, RD_TT_TRIANGULATED, 1);
        }
    }
    for (j = 1; j + 1 < rd_map_frame_num(m); ++j) {   /* [2.2] the middle frames by PnP (Ceres) */
        rd_frame* a = rd_map_get_frame(m, j - 1);
        rd_frame* b = rd_map_get_frame(m, j);
        rd_frame* f0 = rd_map_get_frame(m, 0);
        rd_solver* s;
        ok_quat q; double p[3];
        cam_pose(a, &q, p);
        rd_frame_set_pose(b, &b->cam_q, b->cam_p, &q, p);
        s = rd_solver_create((int)in->cfg->solver_iteration_limit);
        rd_solver_add_frame_states(s, b, 1);
        for (k = 0; k < b->nkp; ++k) {
            rd_track* t = b->track[k];
            if (!t) continue;
            if (rd_track_keypoint_index(t, f0) == RD_NIL) continue;
            if (HAS(t, RD_TT_VALID) && HAS(t, RD_TT_TRIANGULATED)) rd_solver_add_rpp(s, b, t);
        }
        rd_solver_solve(s, in->hooks);
        rd_solver_free(s);
    }
    for (i = 0; i < rd_map_track_num(m); ++i) {      /* [2.3] triangulate more points */
        rd_track* t = rd_map_get_track(m, i);
        double p[3];
        if (HAS(t, RD_TT_VALID)) continue;
        if (track_triangulate(t, p)) { set_landmark_point(t, p); set_tag(&t->tags, RD_TT_VALID, 1); set_tag(&t->tags, RD_TT_TRIANGULATED, 1); }
    }
    {   /* [3.1] bundle adjustment */
        rd_solver* s = rd_solver_create((int)in->cfg->solver_iteration_limit);
        rd_track** seen = (rd_track**)malloc(sizeof(rd_track*) * (rd_map_track_num(m) + 1));
        size_t nseen = 0, q;
        int usable;
        rd_map_get_frame(m, 0)->tags |= RD_TAG(RD_FT_FIX_POSE);
        for (i = 0; i < rd_map_frame_num(m); ++i) rd_solver_add_frame_states(s, rd_map_get_frame(m, i), 0);
        for (i = 0; i < rd_map_frame_num(m); ++i) {
            rd_frame* f = rd_map_get_frame(m, i);
            for (j = 0; j < f->nkp; ++j) {
                rd_track* t = f->track[j];
                int dup = 0;
                if (!t || !HAS(t, RD_TT_VALID)) continue;
                for (q = 0; q < nseen; ++q) if (seen[q] == t) { dup = 1; break; }
                if (dup) continue;
                seen[nseen++] = t;
                rd_solver_add_track_states(s, t);
            }
        }
        for (i = 0; i < rd_map_frame_num(m); ++i) {
            rd_frame* f = rd_map_get_frame(m, i);
            for (j = 0; j < f->nkp; ++j) {
                rd_track* t = f->track[j];
                if (!t || !(HAS(t, RD_TT_VALID) && HAS(t, RD_TT_TRIANGULATED))) continue;
                if (f == t->ref[0].frame) continue;
                rd_solver_add_rpe(s, f, j);
            }
        }
        usable = rd_solver_solve(s, in->hooks);
        rd_solver_free(s); free(seen);
        if (!usable) goto done;
    }
    /* [3.2] cleanup invalid points: rd_init_initialize prunes right after (landmark.reprojection_error is never written: 0) */
    ok = 1;
done:
    for (i = 0; i < 8; ++i) { free(pts[i]); free(st[i]); }
    free(pi); free(pj); free(mi); free(mask);
    return ok;
}
static int prune_invalid(void* ctx, const rd_track* t) { (void)ctx; return !HAS(t, RD_TT_VALID); }

/* ------------------------------------------------------------------------------------------------------------------ init_imu */
static void preintegrate(rd_init* in) {
    size_t j;
    for (j = 1; j < rd_map_frame_num(in->map); ++j) {
        rd_frame* f = rd_map_get_frame(in->map, j);
        rd_pi_integrate(&f->preint, f->data, (int)f->ndata, f->t, in->bg, in->ba, 1, 0);
    }
}
static void solve_gyro_bias(rd_init* in) {
    double A[9] = {0}, b[3] = {0};
    size_t j;
    int r, c, k;
    preintegrate(in);
    for (j = 1; j < rd_map_frame_num(in->map); ++j) {
        const rd_frame* fi = rd_map_get_frame(in->map, j - 1);
        const rd_frame* fj = rd_map_get_frame(in->map, j);
        const double* M = fj->preint.jac.dq_dbg;    /* column-major 3x3 */
        ok_quat qi, qj, a, ac, d;
        double pi[3], pj[3], w[3], v[3];
        imu_pose(fi, &qi, pi);
        imu_pose(fj, &qj, pj);
        for (r = 0; r < 3; ++r)                       /* A += dq_dbg^T dq_dbg: every coefficient a left fold */
            for (c = 0; c < 3; ++c) {
                double s = M[3 * r] * M[3 * c];
                for (k = 1; k < 3; ++k) s += M[k + 3 * r] * M[k + 3 * c];
                A[r + 3 * c] = A[r + 3 * c] + s;
            }
        ok_quat_mul(&qi, &fj->preint.delta.q, &a);   /* logmap((pose_i.q * dq)^* * pose_j.q) */
        ac = rd_quat_conj(a);
        ok_quat_mul(&ac, &qj, &d);
        rd_logmap(&d, w);
        ok_m3_mulv_lhsT(M, w, v);                     /* dq_dbg^T * w */
        for (r = 0; r < 3; ++r) b[r] = b[r] + v[r];
    }
    rd_svd3_solve(A, b, in->bg);
}
#define AT(A, rows, r, c) (A)[(size_t)(r) + (size_t)(rows) * (size_t)(c)]
static void solve_gravity_scale_velocity(rd_init* in) {
    const int N = (int)rd_map_frame_num(in->map);
    const int rows = (N - 1) * 6, cols = 3 + 1 + 3 * N;
    double* A = (double*)calloc((size_t)rows * (size_t)cols, sizeof(double));
    double* b = (double*)calloc((size_t)rows, sizeof(double));
    double* x = (double*)calloc((size_t)cols, sizeof(double));
    int j, i, r, c;
    preintegrate(in);
    for (j = 1; j < N; ++j) {
        const rd_frame* fi = rd_map_get_frame(in->map, (size_t)(j - 1));
        const rd_frame* fj = rd_map_get_frame(in->map, (size_t)j);
        const rd_delta* d = &fj->preint.delta;
        ok_quat cqi, cqj; double cpi[3], cpj[3], r1[3], r2[3], r3[3];
        const double s0 = -0.5 * d->t * d->t, s1 = -d->t;
        i = j - 1;
        cam_pose(fi, &cqi, cpi); cam_pose(fj, &cqj, cpj);
        for (r = 0; r < 3; ++r) for (c = 0; c < 3; ++c) {
            const double id = r == c ? 1.0 : 0.0;
            AT(A, rows, i * 6 + r, c) = s0 * id;                     /* -0.5 dt dt * I (off-diagonal -0.0) */
            AT(A, rows, i * 6 + r, 4 + i * 3 + c) = s1 * id;         /* -dt * I */
            AT(A, rows, i * 6 + 3 + r, c) = s1 * id;
            AT(A, rows, i * 6 + 3 + r, 4 + i * 3 + c) = -id;         /* -I */
            AT(A, rows, i * 6 + 3 + r, 4 + j * 3 + c) = id;          /* I */
        }
        for (r = 0; r < 3; ++r) AT(A, rows, i * 6 + r, 3) = cpj[r] - cpi[r];
        rd_quat_rotate(&fi->pose_q, d->p, r1);
        rd_quat_rotate(&fj->pose_q, fj->cam_p, r2);
        rd_quat_rotate(&fi->pose_q, fi->cam_p, r3);
        for (r = 0; r < 3; ++r) b[i * 6 + r] = r1[r] + (r2[r] - r3[r]);
        rd_quat_rotate(&fi->pose_q, d->v, r1);
        for (r = 0; r < 3; ++r) b[i * 6 + 3 + r] = r1[r];
    }
    rd_qr_fullpiv_solve(A, rows, cols, b, x);
    {
        double g[3];
        g[0] = x[0]; g[1] = x[1]; g[2] = x[2];
        ok_v3_normalized(x, g);
        for (r = 0; r < 3; ++r) in->gravity[r] = g[r] * RD_GRAVITY_NOMINAL;
    }
    in->scale = x[3];
    for (i = 0; i < N; ++i) for (r = 0; r < 3; ++r) in->vel[i][r] = x[4 + i * 3 + r];
    free(A); free(b); free(x);
}
static void refine_scale_velocity_via_gravity(rd_init* in) {
    const double damp = 0.1;
    const int N = (int)rd_map_frame_num(in->map);
    const int rows = (N - 1) * 6, cols = 2 + 1 + 3 * N;
    double* A = (double*)calloc((size_t)rows * (size_t)cols, sizeof(double));
    double* b = (double*)calloc((size_t)rows, sizeof(double));
    double* x = (double*)calloc((size_t)cols, sizeof(double));
    double Tg[6];
    int j, i, r, c;
    preintegrate(in);
    rd_s2_tangential_basis(in->gravity, Tg);
    for (j = 1; j < N; ++j) {
        const rd_frame* fi = rd_map_get_frame(in->map, (size_t)(j - 1));
        const rd_frame* fj = rd_map_get_frame(in->map, (size_t)j);
        const rd_delta* d = &fj->preint.delta;
        ok_quat cqi, cqj; double cpi[3], cpj[3], r1[3], r2[3], r3[3];
        const double s0 = -0.5 * d->t * d->t, s1 = -d->t, s2 = 0.5 * d->t * d->t;
        i = j - 1;
        cam_pose(fi, &cqi, cpi); cam_pose(fj, &cqj, cpj);
        for (r = 0; r < 3; ++r) {
            for (c = 0; c < 2; ++c) { AT(A, rows, i * 6 + r, c) = s0 * Tg[r + 3 * c]; AT(A, rows, i * 6 + 3 + r, c) = s1 * Tg[r + 3 * c]; }
            AT(A, rows, i * 6 + r, 2) = cpj[r] - cpi[r];
            for (c = 0; c < 3; ++c) {
                const double id = r == c ? 1.0 : 0.0;
                AT(A, rows, i * 6 + r, 3 + i * 3 + c) = s1 * id;
                AT(A, rows, i * 6 + 3 + r, 3 + i * 3 + c) = -id;
                AT(A, rows, i * 6 + 3 + r, 3 + j * 3 + c) = id;
            }
        }
        rd_quat_rotate(&fi->pose_q, d->p, r1);
        rd_quat_rotate(&fj->pose_q, fj->cam_p, r2);
        rd_quat_rotate(&fi->pose_q, fi->cam_p, r3);
        for (r = 0; r < 3; ++r) b[i * 6 + r] = (s2 * in->gravity[r] + r1[r]) + (r2[r] - r3[r]);
        rd_quat_rotate(&fi->pose_q, d->v, r1);
        for (r = 0; r < 3; ++r) b[i * 6 + 3 + r] = d->t * in->gravity[r] + r1[r];
    }
    rd_qr_fullpiv_solve(A, rows, cols, b, x);
    {
        double g[3], gn[3];
        for (r = 0; r < 3; ++r) g[r] = in->gravity[r] + ((damp * Tg[r]) * x[0] + (damp * Tg[r + 3]) * x[1]);   /* (damp Tg) dg */
        memcpy(gn, g, sizeof gn);
        ok_v3_normalized(g, gn);
        for (r = 0; r < 3; ++r) in->gravity[r] = gn[r] * RD_GRAVITY_NOMINAL;
    }
    in->scale = x[2];
    for (i = 0; i < N; ++i) for (r = 0; r < 3; ++r) in->vel[i][r] = x[3 + i * 3 + r];
    free(A); free(b); free(x);
}
static int apply_init(rd_init* in) {
    const double gn[3] = {0, 0, -RD_GRAVITY_NOMINAL};
    ok_quat q;
    size_t i, final_point_num = 0;
    int r;
    rd_quat_from_two_vectors(in->gravity, gn, &q);
    for (i = 0; i < rd_map_frame_num(in->map); ++i) {
        rd_frame* f = rd_map_get_frame(in->map, i);
        ok_quat iq, nq; double ip[3], rp[3], np[3];
        imu_pose(f, &iq, ip);
        ok_quat_mul(&q, &iq, &nq);
        rd_quat_rotate(&q, ip, rp);
        for (r = 0; r < 3; ++r) np[r] = in->scale * rp[r];
        rd_frame_set_pose(f, &f->imu_q, f->imu_p, &nq, np);
        rd_quat_rotate(&q, in->vel[i], f->motion.v);
        memcpy(f->motion.bg, in->bg, sizeof f->motion.bg);
        f->motion.ba[0] = 0; f->motion.ba[1] = 0; f->motion.ba[2] = 0;
    }
    for (i = 0; i < rd_map_track_num(in->map); ++i) {
        rd_track* t = rd_map_get_track(in->map, i);
        double p[3];
        if (track_triangulate(t, p)) {
            set_landmark_point(t, p);
            set_tag(&t->tags, RD_TT_VALID, 1); set_tag(&t->tags, RD_TT_TRIANGULATED, 1);
            final_point_num++;
        } else {
            set_tag(&t->tags, RD_TT_VALID, 0);
        }
    }
    return final_point_num >= in->cfg->initializer_min_landmarks;
}
static int init_imu(rd_init* in) {
    const size_t N = rd_map_frame_num(in->map);
    size_t i;
    memset(in->bg, 0, sizeof in->bg); memset(in->ba, 0, sizeof in->ba); memset(in->gravity, 0, sizeof in->gravity);
    in->scale = 1;
    in->vel = (double(*)[3])realloc(in->vel, sizeof(double[3]) * (N ? N : 1));
    for (i = in->nvel; i < N; ++i) memset(in->vel[i], 0, sizeof in->vel[i]);   /* velocities.resize(N, Zero) keeps old entries */
    in->nvel = N;
    solve_gyro_bias(in);
    solve_gravity_scale_velocity(in);
    if (in->scale < 0.001 || in->scale > 1.0) return 0;
    if (!in->cfg->initializer_refine_imu) return apply_init(in);
    refine_scale_velocity_via_gravity(in);
    if (in->scale < 0.001 || in->scale > 1.0) return 0;
    return apply_init(in);
}

/* ------------------------------------------------------------------------------------------------------------------ initialize */
int rd_init_initialize(rd_init* in) {
    rd_map* m = in->map;
    rd_solver* s;
    rd_track** seen;
    size_t nseen = 0, i, j, q;
    if (!m) return 0;
    if (!init_sfm(in)) { if (in->on_stage) in->on_stage(in->ctx, in, 1, 0); return 0; }
    rd_map_prune_tracks(m, prune_invalid, NULL);
    if (in->on_stage) in->on_stage(in->ctx, in, 1, 1);
    if (!init_imu(in)) { if (in->on_stage) in->on_stage(in->ctx, in, 2, 0); return 0; }
    if (in->on_stage) in->on_stage(in->ctx, in, 2, 1);
    rd_map_get_frame(m, 0)->tags |= RD_TAG(RD_FT_FIX_POSE);
    s = rd_solver_create((int)in->cfg->solver_iteration_limit);
    for (i = 0; i < rd_map_frame_num(m); ++i) rd_solver_add_frame_states(s, rd_map_get_frame(m, i), 1);
    seen = (rd_track**)malloc(sizeof(rd_track*) * (rd_map_track_num(m) + 1));
    for (i = 0; i < rd_map_frame_num(m); ++i) {
        rd_frame* f = rd_map_get_frame(m, i);
        for (j = 0; j < f->nkp; ++j) {
            rd_track* t = f->track[j];
            int dup = 0;
            if (!t || !HAS(t, RD_TT_VALID)) continue;
            for (q = 0; q < nseen; ++q) if (seen[q] == t) { dup = 1; break; }
            if (dup) continue;
            seen[nseen++] = t;
            rd_solver_add_track_states(s, t);
        }
    }
    for (i = 0; i < rd_map_frame_num(m); ++i) {
        rd_frame* f = rd_map_get_frame(m, i);
        for (j = 0; j < f->nkp; ++j) {
            rd_track* t = f->track[j];
            if (!t || !(HAS(t, RD_TT_VALID) && HAS(t, RD_TT_TRIANGULATED))) continue;
            if (f == t->ref[0].frame) continue;
            rd_solver_add_rpe(s, f, j);
        }
    }
    for (j = 1; j < rd_map_frame_num(m); ++j) {
        rd_frame* fi = rd_map_get_frame(m, j - 1);
        rd_frame* fj = rd_map_get_frame(m, j);
        if (rd_pi_integrate(&fj->preint, fj->data, (int)fj->ndata, fj->t, fi->motion.bg, fi->motion.ba, 1, 1))
            rd_solver_add_pie(s, fi, fj, &fj->preint);
    }
    rd_solver_solve(s, in->hooks);
    rd_solver_free(s); free(seen);
    for (i = 0; i < rd_map_frame_num(m); ++i) rd_map_get_frame(m, i)->tags |= RD_TAG(RD_FT_KEYFRAME);
    return 1;
}
