/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port, module M10: SlidingWindowTracker. See rd_sys_swt.h. */
#include "rd_sys_swt.h"
#include "rd_geom.h"
#include "rd_lie.h"
#include "rd_solver_glue.h"
#include "rd_sys_eigen.h"
#include "rd_sys_init.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HAS(x, tag) (((x)->tags & RD_TAG(tag)) != 0)
static void set_tag(uint32_t* tags, int tag, int on) { if (on) *tags |= RD_TAG(tag); else *tags &= ~RD_TAG(tag); }
static int all3(const rd_track* t) { return HAS(t, RD_TT_VALID) && HAS(t, RD_TT_TRIANGULATED) && HAS(t, RD_TT_STATIC); }
static rd_frame* last_frame(const rd_map* m) { return rd_map_get_frame(m, rd_map_frame_num(m) - 1); }
static rd_frame* last_sub_or_self(rd_frame* f) { return f->nsub ? f->sub[f->nsub - 1] : f; }
static void stage(rd_swt* s, int st, int v) { if (s->hooks.stage) s->hooks.stage(s->hooks.ctx, s, st, v); }

/* preintegration.integrate(t, bg, ba, true, true) of frame f */
static int integrate(rd_frame* f, const rd_frame* bias) {
    return rd_pi_integrate(&f->preint, f->data, (int)f->ndata, f->t, bias->motion.bg, bias->motion.ba, 1, 1);
}
/* PreIntegrator::predict(old_frame, new_frame) with new_frame's preintegration */
static void predict(const rd_frame* fi, rd_frame* fj) {
    rd_pose op, np;
    rd_motion nm;
    op.q = fi->pose_q; memcpy(op.p, fi->pose_p, sizeof op.p);
    rd_pi_predict(&fj->preint, &op, &fi->motion, &np, &nm);
    fj->pose_q = np.q; memcpy(fj->pose_p, np.p, sizeof fj->pose_p);
    fj->motion = nm;
}

void rd_swt_create(rd_swt* s, rd_map* keyframe_map, const rd_cfg* cfg) {
    size_t j;
    memset(s, 0, sizeof *s);
    s->cfg = cfg;
    s->map = keyframe_map;
    for (j = 1; j < rd_map_frame_num(s->map); ++j) integrate(rd_map_get_frame(s->map, j), rd_map_get_frame(s->map, j - 1));
}
void rd_swt_destroy(rd_swt* s) {
    rd_map_free(s->map); s->map = NULL;
    if (s->marg) { rd_marg_free(s->marg); free(s->marg); s->marg = NULL; }
}

/* ------------------------------------------------------------------------------------------------------------- mirror_frame */
static int prune_trash(void* ctx, const rd_track* t) { (void)ctx; return HAS(t, RD_TT_TRASH) && !HAS(t, RD_TT_STATIC); }
void rd_swt_mirror_frame(rd_swt* s, rd_map* ft, uint64_t frame_id) {
    rd_map* m = s->map;
    rd_frame* nfi = last_sub_or_self(last_frame(m));
    const size_t ii = rd_map_frame_index_by_id(ft, nfi->id), jj = rd_map_frame_index_by_id(ft, frame_id);
    rd_frame *ofi, *ofj, *curr, *nfj;
    size_t index, ki;
    if (ii == RD_NIL || jj == RD_NIL) return;
    ofi = rd_map_get_frame(ft, ii);
    ofj = rd_map_get_frame(ft, jj);
    curr = rd_frame_clone(ofj);
    for (index = jj - 1; index > ii; --index) {
        const rd_frame* of = rd_map_get_frame(ft, index);
        rd_imu_list_insert(&curr->data, &curr->ndata, &curr->cdata, 0, of->data, of->ndata);
    }
    rd_map_attach_frame(m, rd_frame_clone(curr), RD_NIL);
    nfj = last_frame(m);
    for (ki = 0; ki < ofi->nkp; ++ki) {
        rd_track* t = ofi->track[ki];
        if (t) {
            const size_t kj = rd_track_keypoint_index(t, ofj);
            if (kj != RD_NIL) {
                rd_track* nt = rd_frame_get_track(nfi, ki, m);
                rd_track_add_keypoint(nt, nfj, kj);
                set_tag(&t->tags, RD_TT_TRASH, HAS(nt, RD_TT_TRASH) && !HAS(nt, RD_TT_STATIC));
            }
        }
    }
    rd_map_prune_tracks(m, prune_trash, NULL);
    integrate(nfj, nfi);
    predict(nfi, nfj);
    rd_frame_free(curr);                              /* the unique_ptr curr_frame dies at the end of the scope */
}

/* -------------------------------------------------------------------------------------------------------------- judge (PARSAC) */
/* predict_RT: P = Pwc^-1 PwI (Pwj^-1 Pwi) PwI^-1 Pwc with frame_i's camera / IMU extrinsics */
static void m4_of(const ok_quat* q, const double p[3], double M[16]) {
    double R[9];
    int i, j;
    ok_quat_to_mat3(q, R);
    memset(M, 0, 16 * sizeof(double)); M[15] = 1.0;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) M[i + 4 * j] = R[i + 3 * j];
    M[12] = p[0]; M[13] = p[1]; M[14] = p[2];
}
static void predict_rt(const rd_frame* fi, const rd_frame* fj, double R[9], double t[3]) {
    double Pwc[16], PwI[16], Pwi[16], Pwj[16], inv[16], Pji[16], P[16];
    int i, j;
    m4_of(&fi->cam_q, fi->cam_p, Pwc);
    m4_of(&fi->imu_q, fi->imu_p, PwI);
    m4_of(&fi->pose_q, fi->pose_p, Pwi);
    m4_of(&fj->pose_q, fj->pose_p, Pwj);
    rd_m4_inverse(Pwj, inv); rd_m4_mul(inv, Pwi, Pji);
    rd_m4_inverse(Pwc, inv); rd_m4_mul(inv, PwI, P);
    rd_m4_mul(P, Pji, P);
    rd_m4_inverse(PwI, inv); rd_m4_mul(P, inv, P);
    rd_m4_mul(P, Pwc, P);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) R[i + 3 * j] = P[i + 4 * j];
    t[0] = P[12]; t[1] = P[13]; t[2] = P[14];
}
static int cmp_double(const void* a, const void* b) {
    const double x = *(const double*)a, y = *(const double*)b;
    return x < y ? -1 : (y < x ? 1 : 0);
}
static int judge_track_status(rd_swt* s) {
    rd_map* m = s->map;
    rd_frame* curr = last_frame(m);
    rd_frame* kf = rd_map_get_frame(m, rd_map_frame_num(m) - 2);
    rd_frame* last = last_sub_or_self(kf);
    const size_t nkp = curr->nkp;
    double* p2d = (double*)malloc(sizeof(double) * 2 * (nkp + 1));
    double* p3d = (double*)malloc(sizeof(double) * 3 * (nkp + 1));
    size_t* lens = (size_t*)malloc(sizeof(size_t) * (nkp + 1));
    long* idx = (long*)malloc(sizeof(long) * (nkp + 1));
    char* mask = NULL;
    double* in_d = NULL;
    double* out_d = NULL;
    size_t np = 0, k, nin = 0, nout = 0;
    int ok = 0;
    integrate(curr, last);
    predict(last, curr);
    for (k = 0; k < nkp; ++k) {
        rd_track* t = curr->track[k];
        idx[k] = -1;
        if (t && HAS(t, RD_TT_VALID) && HAS(t, RD_TT_TRIANGULATED)) {
            const double* b = curr->bearing + 3 * k;
            p2d[2 * np] = b[0] / b[2]; p2d[2 * np + 1] = b[1] / b[2];
            rd_sys_get_landmark_point(t, p3d + 3 * np);
            lens[np] = (size_t)t->life;
            idx[k] = (long)np;
            np++;
        }
    }
    if (np < 20 || !s->hooks.pnp_mask) goto done;
    mask = (char*)calloc(np, 1);
    {   /* the camera pose and the PnP prior (Rcw, tcw) */
        ok_quat q, qi;
        double p[3], Rcw[9], tcw[3];
        int i;
        rd_sys_cam_pose(curr, &q, p);
        qi = ok_quat_inverse(q);
        ok_quat_to_mat3(&qi, Rcw);
        rd_quat_rotate(&qi, p, tcw);
        for (i = 0; i < 3; ++i) tcw[i] = tcw[i] * -1.0;
        s->hooks.pnp_mask(s->hooks.ctx, np, p3d, p2d, lens, Rcw, tcw, 1.0 / curr->K[0], mask);
    }
    {
        double R[9], t[3], tx[9], E[9], Kti[9], Ki[9], F[9], Ft[9];
        int i, j;
        predict_rt(kf, curr, R, t);
        memset(tx, 0, sizeof tx);                     /* compute_essential_matrix: E = [t]x R */
        tx[0 + 3 * 1] = -t[2]; tx[0 + 3 * 2] = t[1]; tx[1 + 3 * 0] = t[2];
        tx[1 + 3 * 2] = -t[0]; tx[2 + 3 * 0] = -t[1]; tx[2 + 3 * 1] = t[0];
        ok_m3_mul(tx, R, E);
        rd_inverse3_t(kf->K, Kti);                    /* F = keyframe->K^T^-1 E curr_frame->K^-1 */
        rd_inverse3(curr->K, Ki);
        rd_m3_mul_tinv(Kti, E, F);
        ok_m3_mul(F, Ki, F);
        for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) Ft[i + 3 * j] = F[j + 3 * i];
        in_d = (double*)malloc(sizeof(double) * (np + 1));
        out_d = (double*)malloc(sizeof(double) * (np + 1));
        for (k = 0; k < nkp; ++k) {
            size_t jk;
            double p1[2], p2[2], err;
            if (idx[k] == -1) continue;
            jk = rd_track_keypoint_index(curr->track[k], kf);
            if (jk == RD_NIL) continue;
            rd_apply_k(kf->bearing + 3 * jk, kf->K, p1);
            rd_apply_k(curr->bearing + 3 * k, curr->K, p2);
            err = rd_epipolar_dist(F, p1, p2) + rd_epipolar_dist(Ft, p2, p1);
            if (mask[idx[k]]) in_d[nin++] = err;
            else out_d[nout++] = err;
        }
    }
    if (nin < 20 || nout < 20) goto done;
    qsort(in_d, nin, sizeof(double), cmp_double);
    qsort(out_d, nout, sizeof(double), cmp_double);
    {
        const double th1 = in_d[(size_t)((double)nin * 0.5)], th2 = out_d[(size_t)((double)nout * 0.5)];
        if (th2 < th1 * 2) goto done;                 /* ambiguity */
        s->m_th = (th1 + th2) / 2;
    }
    for (k = 0; k < nkp; ++k) {
        rd_track* t = curr->track[k];
        if (t && idx[k] != -1) {
            const int inlier = mask[idx[k]] != 0;
            set_tag(&t->tags, RD_TT_OUTLIER, !inlier);
            set_tag(&t->tags, RD_TT_STATIC, inlier);
        }
    }
    ok = 1;
done:
    free(p2d); free(p3d); free(lens); free(idx); free(mask); free(in_d); free(out_d);
    return ok;
}

/* filter_parsac_2d2d: pairs (ki, kj) of tracks seen in both frames; kj == 0 is skipped too (`if (size_t kj = ...)`).
 * Returns the number of mask entries (0: fewer than 10 pairs or no estimator). */
static size_t filter_parsac_2d2d(rd_swt* s, rd_frame* fi, rd_frame* fj, char** mask, size_t** pts_to_index) {
    double* p1 = (double*)malloc(sizeof(double) * 2 * (fi->nkp + 1));
    double* p2 = (double*)malloc(sizeof(double) * 2 * (fi->nkp + 1));
    size_t n = 0, ki;
    *pts_to_index = (size_t*)malloc(sizeof(size_t) * (fi->nkp + 1));
    *mask = NULL;
    for (ki = 0; ki < fi->nkp; ++ki) {
        rd_track* t = fi->track[ki];
        size_t kj;
        const double *a, *b;
        if (!t) continue;
        kj = rd_track_keypoint_index(t, fj);
        if (!kj || kj == RD_NIL) continue;
        a = fi->bearing + 3 * ki; b = fj->bearing + 3 * kj;
        p1[2 * n] = a[0] / a[2]; p1[2 * n + 1] = a[1] / a[2];
        p2[2 * n] = b[0] / b[2]; p2[2 * n + 1] = b[1] / b[2];
        (*pts_to_index)[n++] = kj;
    }
    if (n < 10 || !s->hooks.ess_mask) { free(p1); free(p2); return 0; }
    *mask = (char*)calloc(n, 1);
    s->hooks.ess_mask(s->hooks.ctx, n, p1, p2, s->m_th / fi->K[0], *mask);
    free(p1); free(p2);
    return n;
}
static void update_track_status(rd_swt* s) {
    rd_map* m = s->map;
    rd_frame* curr = last_frame(m);
    const size_t fid = rd_map_frame_index_by_id(s->ft, curr->id);
    const size_t nf = rd_map_frame_num(m);
    size_t *outlier, *matches, i, start, a;
    rd_frame* old;
    if (fid == RD_NIL) return;
    old = rd_map_get_frame(s->ft, fid);
    outlier = (size_t*)calloc(curr->nkp + 1, sizeof(size_t));
    matches = (size_t*)calloc(curr->nkp + 1, sizeof(size_t));
    a = nf - 1 - s->cfg->parsac_keyframe_check_size;  /* size_t arithmetic: wraps when the window is short */
    start = a > 0 ? a : 0;
    if (nf - 1 < start) start = nf - 1;
    for (i = start; i < nf - 1; ++i) {
        char* mask;
        size_t* pti;
        const size_t n = filter_parsac_2d2d(s, rd_map_get_frame(m, i), curr, &mask, &pti);
        size_t j;
        for (j = 0; j < n; ++j) {
            if (!mask[j]) outlier[pti[j]] += 1;
            matches[pti[j]] += 1;
        }
        free(mask); free(pti);
    }
    for (i = 0; i < curr->nkp; ++i) {
        rd_track* ct = curr->track[i];
        size_t j;
        if (!ct) continue;
        j = rd_track_keypoint_index(ct, old);       /* by frame id: the current frame's own keypoint */
        if (!j || j == RD_NIL) continue;
        {
            rd_track* ot = old->track[j];
            const size_t outlier_th = nf / 2;
            if (outlier[i] > outlier_th / 2 && (double)outlier[i] > 0.8 * (double)matches[i]) set_tag(&ct->tags, RD_TT_STATIC, 0);
            if (!HAS(ot, RD_TT_STATIC) || !HAS(ct, RD_TT_STATIC)) { set_tag(&ct->tags, RD_TT_STATIC, 0); set_tag(&ot->tags, RD_TT_STATIC, 0); }
        }
    }
    free(outlier); free(matches);
}

/* ---------------------------------------------------------------------------------------------------------- localize_newframe */
static void localize_newframe(rd_swt* s) {
    rd_map* m = s->map;
    rd_frame* fi = last_sub_or_self(rd_map_get_frame(m, rd_map_frame_num(m) - 2));
    rd_frame* fj = last_frame(m);
    rd_solver* sv = rd_solver_create((int)s->cfg->solver_iteration_limit);
    size_t k;
    rd_solver_add_frame_states(sv, fj, 1);
    rd_solver_add_pip(sv, fi, fj, &fj->preint);
    for (k = 0; k < fj->nkp; ++k) {
        rd_track* t = fj->track[k];
        if (t && all3(t)) rd_solver_add_rpp(sv, fj, t);
    }
    rd_solver_solve(sv, s->sv_hooks);
    rd_solver_free(sv);
}

/* ------------------------------------------------------------------------------------------------------------ manage_keyframe */
static int manage_keyframe(rd_swt* s) {
    rd_map* m = s->map;
    const size_t n = rd_map_frame_num(m);
    rd_frame* ki = rd_map_get_frame(m, n - 2);
    rd_frame* nj = rd_map_get_frame(m, n - 1);
    size_t mapped = 0, k;
    if (ki->nsub) {
        rd_frame* lastsub = ki->sub[ki->nsub - 1];
        if (HAS(lastsub, RD_FT_NO_TRANSLATION)) {
            if (!HAS(nj, RD_FT_NO_TRANSLATION)) {     /* [T]....<-[T] with [R] subframes: the last [R] becomes a keyframe */
                lastsub->tags |= RD_TAG(RD_FT_KEYFRAME);
                rd_map_attach_frame(m, rd_frame_sub_pop(ki), n - 1);
                nj->tags |= RD_TAG(RD_FT_KEYFRAME);
                return 1;
            }
        } else {
            if (HAS(nj, RD_FT_NO_TRANSLATION)) {      /* [T]....<-[R] with [T] subframes: lift the last [T], [R] under it */
                rd_frame* lifted = rd_frame_sub_pop(ki);
                lifted->tags |= RD_TAG(RD_FT_KEYFRAME);
                rd_frame_sub_push(lifted, rd_map_detach_frame(m, rd_map_frame_num(m) - 1));
                rd_map_attach_frame(m, lifted, RD_NIL);
                return 1;
            } else if (ki->nsub >= s->cfg->sliding_window_subframe_size) {
                nj->tags |= RD_TAG(RD_FT_KEYFRAME);
                return 1;
            }
        }
    }
    for (k = 0; k < nj->nkp; ++k) if (nj->track[k] && all3(nj->track[k])) mapped++;
    if (mapped < s->cfg->sliding_window_force_keyframe_landmarks) {
        nj->tags |= RD_TAG(RD_FT_KEYFRAME);
        return 1;
    }
    rd_frame_sub_push(ki, rd_map_detach_frame(m, rd_map_frame_num(m) - 1));
    return 0;
}

/* ------------------------------------------------------------------------------------------------------------- track_landmark */
static void track_landmark(rd_swt* s) {
    rd_frame* nj = last_frame(s->map);
    size_t k;
    for (k = 0; k < nj->nkp; ++k) {
        rd_track* t = nj->track[k];
        double p[3];
        if (!t || HAS(t, RD_TT_TRIANGULATED)) continue;
        if (rd_sys_track_triangulate(t, p)) {
            rd_sys_set_landmark_point(t, p);
            set_tag(&t->tags, RD_TT_TRIANGULATED, 1); set_tag(&t->tags, RD_TT_VALID, 1); set_tag(&t->tags, RD_TT_STATIC, 1);
        } else {                                      /* outlier */
            t->inv_depth = -1.0;
            set_tag(&t->tags, RD_TT_TRIANGULATED, 0); set_tag(&t->tags, RD_TT_VALID, 0);
        }
    }
}

/* ------------------------------------------------------------------------------------------------------------- refine_window */
static void marg_frames(rd_map* m, rd_marg_frame* fr) {
    size_t i;
    for (i = 0; i < rd_map_frame_num(m); ++i) {
        rd_frame* f = rd_map_get_frame(m, i);
        fr[i].id = f->id;
        fr[i].pose.q = f->pose_q; memcpy(fr[i].pose.p, f->pose_p, sizeof fr[i].pose.p);
        fr[i].motion = f->motion;
        fr[i].imu.q_cs = f->imu_q; memcpy(fr[i].imu.p_cs, f->imu_p, sizeof fr[i].imu.p_cs);
        fr[i].kpre = i > 0 ? &f->kpreint : NULL;
    }
}
/* the reprojection check of refine_window (keyframe observations only) */
static int check_rpe(const rd_track* t) {
    double x[3], rpe = 0.0, cnt = 0.0;
    size_t r;
    int valid = 1;
    rd_sys_get_landmark_point(t, x);
    for (r = 0; r < t->nref; ++r) {
        const rd_frame* f = t->ref[r].frame;
        ok_quat q, qc;
        double p[3], d[3], y[3], a[2], b[2];
        if (!HAS(f, RD_FT_KEYFRAME)) continue;
        rd_sys_cam_pose(f, &q, p);
        qc = rd_quat_conj(q);
        d[0] = x[0] - p[0]; d[1] = x[1] - p[1]; d[2] = x[2] - p[2];
        rd_quat_rotate(&qc, d, y);
        if (y[2] <= 1.0e-3 || y[2] > 50) { valid = 0; break; }
        rd_apply_k(y, f->K, a);
        rd_apply_k(f->bearing + 3 * t->ref[r].kp, f->K, b);
        a[0] -= b[0]; a[1] -= b[1];
        rpe += sqrt(a[0] * a[0] + a[1] * a[1]);
        cnt += 1.0;
    }
    return valid && (rpe / (cnt > 1.0 ? cnt : 1.0) < 3.0);
}
static void refine_window(rd_swt* s) {
    rd_map* m = s->map;
    const size_t nf = rd_map_frame_num(m);
    rd_solver* sv = rd_solver_create((int)s->cfg->solver_iteration_limit);
    char* visited = (char*)calloc(rd_map_track_num(m) + 1, 1);
    size_t i, j, k;
    if (!s->marg) {                                   /* Solver::create_marginalization_factor(map) */
        rd_marg_frame* fr = (rd_marg_frame*)calloc(nf + 1, sizeof(rd_marg_frame));
        marg_frames(m, fr);
        s->marg = (rd_marg*)calloc(1, sizeof(rd_marg));
        rd_marg_init(s->marg, (int)nf, fr);
        free(fr);
    }
    for (i = 0; i < nf; ++i) rd_solver_add_frame_states(sv, rd_map_get_frame(m, i), 1);
    for (i = 0; i < nf; ++i) {
        rd_frame* f = rd_map_get_frame(m, i);
        for (j = 0; j < f->nkp; ++j) {
            rd_track* t = f->track[j];
            if (!t || visited[t->map_index]) continue;
            visited[t->map_index] = 1;
            if (!HAS(t, RD_TT_VALID) || !HAS(t, RD_TT_STATIC)) continue;
            if (!HAS(t->ref[0].frame, RD_FT_KEYFRAME)) continue;
            rd_solver_add_track_states(sv, t);
        }
    }
    rd_solver_add_marg(sv, s->marg, m);
    for (i = 0; i < nf; ++i) {
        rd_frame* f = rd_map_get_frame(m, i);
        for (j = 0; j < f->nkp; ++j) {
            rd_track* t = f->track[j];
            if (!t || !all3(t)) continue;
            if (!HAS(t->ref[0].frame, RD_FT_KEYFRAME)) continue;
            if (f == t->ref[0].frame) continue;
            rd_solver_add_rpe(sv, f, j);
        }
    }
    for (j = 1; j < nf; ++j) {
        rd_frame* fi = rd_map_get_frame(m, j - 1);
        rd_frame* fj = rd_map_get_frame(m, j);
        fj->kpreint = fj->preint;                     /* keyframe_preintegration = preintegration (with its data) */
        fj->nkdata = 0;
        rd_imu_list_insert(&fj->kdata, &fj->nkdata, &fj->ckdata, 0, fj->data, fj->ndata);
        for (k = fi->nsub; k > 0; --k) {              /* the subframes' data in order, in front */
            const rd_frame* sf = fi->sub[k - 1];
            rd_imu_list_insert(&fj->kdata, &fj->nkdata, &fj->ckdata, 0, sf->data, sf->ndata);
        }
        if (rd_pi_integrate(&fj->kpreint, fj->kdata, (int)fj->nkdata, fj->t, fi->motion.bg, fi->motion.ba, 1, 1))
            rd_solver_add_pie(sv, fi, fj, &fj->kpreint);
    }
    rd_solver_solve(sv, s->sv_hooks);
    rd_solver_free(sv);
    free(visited);
    for (k = 0; k < rd_map_track_num(m); ++k) {
        rd_track* t = rd_map_get_track(m, k);
        if (HAS(t, RD_TT_TRIANGULATED)) set_tag(&t->tags, RD_TT_VALID, check_rpe(t));
        else t->inv_depth = -1.0;
    }
    for (k = 0; k < rd_map_track_num(m); ++k) {
        rd_track* t = rd_map_get_track(m, k);
        if (!HAS(t, RD_TT_VALID)) set_tag(&t->tags, RD_TT_TRASH, 1);
    }
}

/* -------------------------------------------------------------------------------------------------------------- slide_window */
static int map_pos(const rd_map* m, const rd_frame* f) {   /* frame_indices of marginalize (no FIDX event) */
    size_t i;
    for (i = 0; i < rd_map_frame_num(m); ++i) if (rd_map_get_frame(m, i) == f) return (int)i;
    return -1;
}
void rd_swt_marginalize(rd_swt* s, size_t index) {
    rd_map* m = s->map;
    const size_t nf = rd_map_frame_num(m);
    rd_frame* victim = rd_map_get_frame(m, index);
    rd_marg_frame* fr = (rd_marg_frame*)calloc(nf + 1, sizeof(rd_marg_frame));
    rd_marg_track* tr = (rd_marg_track*)calloc(victim->nkp + 1, sizeof(rd_marg_track));
    rd_marg_obs** obs = (rd_marg_obs**)calloc(victim->nkp + 1, sizeof(rd_marg_obs*));
    size_t j, r, nt = 0;
    marg_frames(m, fr);
    for (j = 0; j < victim->nkp; ++j) {
        rd_track* t = victim->track[j];
        const rd_frame* ref;
        rd_marg_obs* o;
        int no = 0;
        if (!t || !HAS(t, RD_TT_VALID)) continue;
        ref = t->ref[0].frame;
        if (!HAS(ref, RD_FT_KEYFRAME)) continue;
        o = (rd_marg_obs*)calloc(t->nref + 1, sizeof(rd_marg_obs));
        for (r = 0; r < t->nref; ++r) {
            const rd_frame* tgt = t->ref[r].frame;
            const int pos = map_pos(m, tgt);
            if (tgt == ref || pos < 0) continue;
            o[no].tgt = pos;
            memcpy(o[no].z, tgt->bearing + 3 * t->ref[r].kp, sizeof o[no].z);
            memcpy(o[no].z_ref, ref->bearing + 3 * t->ref[0].kp, sizeof o[no].z_ref);
            o[no].cam_ref.q_cs = ref->cam_q; memcpy(o[no].cam_ref.p_cs, ref->cam_p, sizeof o[no].cam_ref.p_cs);
            o[no].cam_tgt.q_cs = tgt->cam_q; memcpy(o[no].cam_tgt.p_cs, tgt->cam_p, sizeof o[no].cam_tgt.p_cs);
            memcpy(o[no].sqrt_inv_cov, tgt->sqrt_inv_cov, sizeof o[no].sqrt_inv_cov);
            no++;
        }
        tr[nt].id = t->id; tr[nt].ref = map_pos(m, ref); tr[nt].inv_depth = t->inv_depth;
        tr[nt].nobs = no; tr[nt].obs = o;
        obs[nt++] = o;
    }
    rd_marg_marginalize(s->marg, (int)nf, fr, (int)index, (int)nt, tr, NULL);
    for (j = 0; j < nt; ++j) free(obs[j]);
    free(obs); free(tr); free(fr);
}
static void slide_window(rd_swt* s) {
    rd_map* m = s->map;
    while (rd_map_frame_num(m) > s->cfg->sliding_window_size) {
        rd_frame* f = rd_map_get_frame(m, 0);
        size_t i;
        for (i = 0; i < f->nsub; ++i) rd_map_untrack_frame(m, f->sub[i]);
        rd_map_marginalize_frame(m, 0);
    }
}

/* ---------------------------------------------------------------------------------------------------------- refine_subwindow */
static void refine_subwindow(rd_swt* s) {
    rd_map* m = s->map;
    rd_frame* f = last_frame(m);
    rd_solver* sv;
    size_t i, j, k;
    if (!f->nsub) return;
    if (HAS(f->sub[0], RD_FT_NO_TRANSLATION)) {
        rd_frame* lastsub;
        if (f->nsub >= 9) {                           /* merge rotation-only subframes three by three */
            for (i = f->nsub / 3; i > 0; --i) {
                rd_frame* tgt = f->sub[i * 3 - 1];
                rd_imu_sample* buf = NULL;
                size_t nb = 0, cb = 0;
                for (j = i * 3 - 1; j > (i - 1) * 3; --j) {
                    rd_frame* src = f->sub[j - 1];
                    rd_imu_list_insert(&buf, &nb, &cb, 0, src->data, src->ndata);
                    rd_map_untrack_frame(m, src);
                    rd_frame_free(rd_frame_sub_take(f, j - 1));
                }
                rd_imu_list_insert(&tgt->data, &tgt->ndata, &tgt->cdata, 0, buf, nb);
                free(buf);
            }
        }
        sv = rd_solver_create((int)s->cfg->solver_iteration_limit);
        f->tags |= RD_TAG(RD_FT_FIX_POSE) | RD_TAG(RD_FT_FIX_MOTION);
        rd_solver_add_frame_states(sv, f, 1);
        for (i = 0; i < f->nsub; ++i) {
            rd_frame* sub = f->sub[i];
            rd_frame* prev = i == 0 ? f : f->sub[i - 1];
            rd_solver_add_frame_states(sv, sub, 1);
            integrate(sub, prev);
            rd_solver_add_pie(sv, prev, sub, &sub->preint);
        }
        lastsub = f->sub[f->nsub - 1];
        for (k = 0; k < lastsub->nkp; ++k) {
            rd_track* t = lastsub->track[k];
            if (!t || !HAS(t, RD_TT_VALID)) continue;
            if (HAS(t, RD_TT_TRIANGULATED)) { if (HAS(t, RD_TT_STATIC)) rd_solver_add_rpp(sv, lastsub, t); }
            else rd_solver_add_rop(sv, lastsub, t);
        }
    } else {
        sv = rd_solver_create((int)s->cfg->solver_iteration_limit);
        f->tags |= RD_TAG(RD_FT_FIX_POSE) | RD_TAG(RD_FT_FIX_MOTION);
        rd_solver_add_frame_states(sv, f, 1);
        for (i = 0; i < f->nsub; ++i) {
            rd_frame* sub = f->sub[i];
            rd_frame* prev = i == 0 ? f : f->sub[i - 1];
            rd_solver_add_frame_states(sv, sub, 1);
            integrate(sub, prev);
            rd_solver_add_pie(sv, prev, sub, &sub->preint);
            for (k = 0; k < sub->nkp; ++k) {
                rd_track* t = sub->track[k];
                if (!t || !all3(t)) continue;
                if (HAS(t->ref[0].frame, RD_FT_KEYFRAME)) rd_solver_add_rpp(sv, sub, t);
                /* upstream adds the KEYFRAME's factor at the SUBFRAME's keypoint index k (frame->reprojection_error_factors[k]) */
                else if (t->ref[0].frame->id > f->id) rd_solver_add_rpe(sv, f, k);
            }
        }
    }
    rd_solver_solve(sv, s->sv_hooks);
    rd_solver_free(sv);
    f->tags &= ~(RD_TAG(RD_FT_FIX_POSE) | RD_TAG(RD_FT_FIX_MOTION));
}

/* --------------------------------------------------------------------------------------------------------------------- track */
int rd_swt_track(rd_swt* s) {
    if (s->cfg->parsac_flag) {
        const int judged = judge_track_status(s);
        stage(s, 11, judged);
        if (judged) { update_track_status(s); stage(s, 12, 0); }
    }
    localize_newframe(s);
    stage(s, 13, 0);
    if (manage_keyframe(s)) {
        stage(s, 14, 1);
        track_landmark(s);   stage(s, 15, 0);
        refine_window(s);    stage(s, 16, 0);
        slide_window(s);     stage(s, 17, 0);
    } else {
        stage(s, 14, 0);
        refine_subwindow(s); stage(s, 18, 0);
    }
    return 1;
}

void rd_swt_latest_state(const rd_swt* s, double* t, rd_pose* pose, rd_motion* motion) {
    const rd_frame* f = last_sub_or_self(last_frame(s->map));
    *t = f->t;
    pose->q = f->pose_q; memcpy(pose->p, f->pose_p, sizeof pose->p);
    *motion = f->motion;
}
