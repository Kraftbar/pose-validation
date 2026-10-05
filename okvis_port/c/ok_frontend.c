/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 7b: the frontend data association. See ok_frontend.h for the notices and the scope.
 * Every function mirrors the C++ function of the same name (Frontend.cpp, stereo_triangulation.cpp, the OpenGV
 * adapters' correspondence lists); the Eigen expressions are written with the evaluation order measured against
 * Eigen 3.4.0 (dot / squaredNorm / norm: left fold, normalized(): x / sqrt(L), Matrix3d * Vector3d: ok_m3_mulv). */
#include "ok_frontend.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------------------------
 * Eigen small-vector models
 * ---------------------------------------------------------------------------------------------------------------- */
static double dot3(const double a[3], const double b[3]) { return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]; }
static void sub3(const double a[3], const double b[3], double o[3]) { o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2]; }
static void add3(const double a[3], const double b[3], double o[3]) { o[0] = a[0] + b[0]; o[1] = a[1] + b[1]; o[2] = a[2] + b[2]; }
static void scale3(double s, const double a[3], double o[3]) { o[0] = s * a[0]; o[1] = s * a[1]; o[2] = s * a[2]; }
static void div3(const double a[3], double s, double o[3]) { o[0] = a[0] / s; o[1] = a[1] / s; o[2] = a[2] / s; }
static void cross3(const double a[3], const double b[3], double o[3]) {
    const double x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0];
    o[0] = x; o[1] = y; o[2] = z;
}
static void nrm3(const double a[3], double o[3]) { double t[3]; memcpy(t, a, sizeof t); ok_v3_normalized(a, t); memcpy(o, t, sizeof t); }
static double norm3(const double a[3]) { return ok_v3_norm(a); }
static double norm2(const double a[2]) { return sqrt(a[0] * a[0] + a[1] * a[1]); }
static double dot2(const double a[2], const double b[2]) { return a[0] * b[0] + a[1] * b[1]; }
/* Vector4d::norm(): two SSE2 packets, sqrt((p0 + p2) + (p1 + p3)) of the squares */
static double norm4(const double a[4]) { return sqrt((a[0] * a[0] + a[2] * a[2]) + (a[1] * a[1] + a[3] * a[3])); }
static double stdmax(double a, double b) { return (a < b) ? b : a; }       /* std::max(a, b) */

/* ------------------------------------------------------------------------------------------------------------------
 * state
 * ---------------------------------------------------------------------------------------------------------------- */
typedef struct fe_cam { int nkp; unsigned char* desc; double* bp; unsigned char* bp_ok; } fe_cam;
typedef struct fe_frame { int alive, ncam; fe_cam cam[OK_FE_MAXCAM]; } fe_frame;

struct ok_fe {
    const ok_vsb* b;
    ok_fe_params p;
    ok_fe_est est;
    fe_frame* frames; int nframes_alloc;
    int is_initialised;
    ok_ncam ncam_sys; int ncam_built;
};

ok_fe* ok_fe_new(const ok_vsb* b, const ok_fe_params* p, const ok_fe_est* est) {
    ok_fe* f = (ok_fe*)calloc(1, sizeof *f);
    f->b = b; f->p = *p; f->est = *est;
    ok_ncam_init(&f->ncam_sys);
    return f;
}
void ok_fe_free(ok_fe* f) {
    int i, c;
    if (!f) return;
    for (i = 0; i < f->nframes_alloc; ++i)
        for (c = 0; c < f->frames[i].ncam; ++c) { free(f->frames[i].cam[c].desc); free(f->frames[i].cam[c].bp); free(f->frames[i].cam[c].bp_ok); }
    free(f->frames);
    if (f->ncam_built) ok_ncam_free(&f->ncam_sys);
    free(f);
}
int ok_fe_is_initialised(const ok_fe* f) { return f->is_initialised; }

void ok_fe_add_frame(ok_fe* f, uint64_t frame, int ncam, const int* nkp, const unsigned char* const* desc) {
    fe_frame* fr;
    const ok_vsb_frame_view* v = ok_vsb_frame(f->b, frame);
    int c, k;
    if (frame >= (uint64_t)f->nframes_alloc) {
        const int n = (int)frame * 2 + 16;
        f->frames = (fe_frame*)realloc(f->frames, sizeof(fe_frame) * (size_t)n);
        memset(f->frames + f->nframes_alloc, 0, sizeof(fe_frame) * (size_t)(n - f->nframes_alloc));
        f->nframes_alloc = n;
    }
    fr = &f->frames[frame];
    fr->alive = 1; fr->ncam = ncam;
    for (c = 0; c < ncam; ++c) {
        const ok_cam* cam = ok_vsb_camera(f->b, c);
        fr->cam[c].nkp = nkp[c];
        fr->cam[c].desc = (unsigned char*)malloc(48 * (size_t)(nkp[c] ? nkp[c] : 1));
        if (nkp[c]) memcpy(fr->cam[c].desc, desc[c], 48 * (size_t)nkp[c]);
        fr->cam[c].bp = (double*)calloc(3 * (size_t)(nkp[c] ? nkp[c] : 1), sizeof(double));
        fr->cam[c].bp_ok = (unsigned char*)calloc((size_t)(nkp[c] ? nkp[c] : 1), 1);
        /* Frame::computeBackProjections: backProject(Vector2d(kp.x, kp.y)) once per keypoint */
        for (k = 0; v && cam && k < nkp[c] && k < v->cam[c].nkp; ++k) {
            double ip[2], d[3];
            ip[0] = (double)v->cam[c].kp[3 * k]; ip[1] = (double)v->cam[c].kp[3 * k + 1];
            fr->cam[c].bp_ok[k] = (unsigned char)(ok_cam_back_project(cam, ip, d) ? 1 : 0);
            memcpy(fr->cam[c].bp + 3 * k, d, sizeof d);
        }
    }
    if (!f->ncam_built && v) {                                   /* the NCameraSystem (for hasOverlap) */
        int ok = 1;
        for (c = 0; c < ncam && ok; ++c) {
            ok_tf T; ok_tf_convert(&T, v->cam[c].T_SC);
            if (ok_ncam_add(&f->ncam_sys, ok_vsb_camera(f->b, c), &T)) ok = 0;
        }
        if (ok && ncam > 1) ok_ncam_compute_overlaps(&f->ncam_sys);
        f->ncam_built = 1;
    }
}

static const fe_frame* fe_fr(const ok_fe* f, uint64_t id) { return (id < (uint64_t)f->nframes_alloc && f->frames[id].alive) ? &f->frames[id] : NULL; }
static int has_overlap(const ok_fe* f, int a, int b) { return f->ncam_sys.overlaps[a][b] != 0; }   /* MultiFrame::hasOverlap */

/* ---- estimator reads ---- */
static const ok_vg* G0(const ok_fe* f) { return ok_vsb_graph((ok_vsb*)f->b, 0); }
static void pose_tf(const ok_fe* f, uint64_t id, ok_tf* t) {                       /* Transformation T = estimator.pose(id) */
    double c[7]; ok_vg_pose_values(G0(f), id, c); ok_tf_convert(t, c);
}
static void extr_tf(const ok_fe* f, uint64_t id, int cam, ok_tf* t) {
    double c[7]; ok_vg_extrinsics_values(G0(f), id, cam, c); ok_tf_convert(t, c);
}
static void tsc_tf(const ok_vsb_frame_view* v, int cam, ok_tf* t) { ok_tf_convert(t, v->cam[cam].T_SC); }
static int lm_added(const ok_fe* f, uint64_t id) { return ok_vg_landmark_find(G0(f), id, NULL); }
static int lm_initialised(const ok_fe* f, uint64_t id) { ok_vg_lm_view v; return ok_vg_landmark_find(G0(f), id, &v) && v.initialised; }
static int is_observed(const ok_fe* f, uint64_t frame, uint32_t cam, uint32_t kp) {
    ok_vg_kid k; uint64_t lm; const ok_reproj_err* e; int c;
    k.frame = frame; k.cam = cam; k.kp = kp;
    return ok_vg_obs_find(G0(f), k, &lm, &e, &c);
}
static int hamming48(const unsigned char* a, const unsigned char* b) {              /* brisk::Hamming::PopcntofXORed(a, b, 3) */
    int i, n = 0;
    for (i = 0; i < 48; ++i) { unsigned v = (unsigned)(a[i] ^ b[i]); while (v) { n += (int)(v & 1u); v >>= 1; } }
    return n;
}
static double cam_f(const ok_cam* c) { return 0.5 * (c->fu + c->fv); }

/* ------------------------------------------------------------------------------------------------------------------
 * the OpenGV adapters' correspondence lists
 * ---------------------------------------------------------------------------------------------------------------- */
/* FrameNoncentralAbsoluteAdapter: keypoints of `frame` whose landmark is added, has >= 2 observations, not at infinity */
static int abs_correspondences(const ok_fe* f, uint64_t frame, int** cam_idx, int** kp_idx) {
    const ok_vsb_frame_view* v = ok_vsb_frame(f->b, frame);
    int n = 0, cap = 64, im, k;
    *cam_idx = (int*)malloc(sizeof(int) * (size_t)cap); *kp_idx = (int*)malloc(sizeof(int) * (size_t)cap);
    for (im = 0; im < v->ncam; ++im)
        for (k = 0; k < v->cam[im].nkp; ++k) {
            const uint64_t id = v->cam[im].lm[k];
            ok_vg_lm_view lv;
            if (id == 0 || !ok_vg_landmark_find(G0(f), id, &lv)) continue;
            if (lv.nobs < 2) continue;
            if (fabs(lv.hp[3]) < 1.0e-8) continue;
            if (n == cap) { cap *= 2; *cam_idx = (int*)realloc(*cam_idx, sizeof(int) * (size_t)cap); *kp_idx = (int*)realloc(*kp_idx, sizeof(int) * (size_t)cap); }
            (*cam_idx)[n] = im; (*kp_idx)[n] = k; ++n;
        }
    return n;
}
/* FrameRelativeAdapter: matches (idxA ascending) between frame A camera `cam` and frame B camera `cam` via the landmark ids */
static int rel_correspondences(const ok_fe* f, uint64_t frame_a, uint64_t frame_b, int cam, int** idx_a, int** idx_b) {
    const ok_vsb_frame_view* a = ok_vsb_frame(f->b, frame_a);
    const ok_vsb_frame_view* b = ok_vsb_frame(f->b, frame_b);
    int n = 0, cap = 64, k, j;
    *idx_a = (int*)malloc(sizeof(int) * (size_t)cap); *idx_b = (int*)malloc(sizeof(int) * (size_t)cap);
    for (k = 0; k < a->cam[cam].nkp; ++k) {
        const uint64_t id = a->cam[cam].lm[k];
        int found = -1;
        if (id == 0) continue;
        for (j = 0; j < b->cam[cam].nkp; ++j) {                 /* idMap.insert keeps the smallest keypoint index of an id */
            const uint64_t idb = b->cam[cam].lm[j];
            if (idb == id && lm_added(f, idb)) { found = j; break; }
        }
        if (found < 0) continue;
        if (n == cap) { cap *= 2; *idx_a = (int*)realloc(*idx_a, sizeof(int) * (size_t)cap); *idx_b = (int*)realloc(*idx_b, sizeof(int) * (size_t)cap); }
        (*idx_a)[n] = k; (*idx_b)[n] = found; ++n;
    }
    return n;
}

/* ------------------------------------------------------------------------------------------------------------------
 * RANSAC glue
 * ---------------------------------------------------------------------------------------------------------------- */
static int run_ransac_3d2d(ok_fe* f, uint64_t frame, int initialise_pose, int remove_outliers) {
    int *ci, *ki, nc, ok = 0;
    ok_fe_ransac r;
    if (ok_vsb_num_frames(f->b) < 2) return 0;
    nc = abs_correspondences(f, frame, &ci, &ki);
    if (nc < 10) { free(ci); free(ki); return nc != 0; }           /* `return int(numCorrespondences)` as a bool */
    memset(&r, 0, sizeof r);
    if (!f->est.ransac || !f->est.ransac(f->est.ctx, 0, nc, &r)) { free(ci); free(ki); return 0; }
    if (r.ninliers >= 10 && (double)r.ninliers / (double)nc > 0.7) {
        if (remove_outliers) {
            unsigned char* inl = (unsigned char*)calloc((size_t)nc, 1);
            int k;
            for (k = 0; k < r.ninliers; ++k) if (r.inliers[k] >= 0 && r.inliers[k] < nc) inl[r.inliers[k]] = 1;
            for (k = 0; k < nc; ++k)
                if (!inl[k]) f->est.remove_observation(f->est.ctx, frame, (uint32_t)ci[k], (uint32_t)ki[k]);
            free(inl);
        }
        {   /* T_WS_mat = Identity; topLeftCorner<3,4>() = model; Transformation(T_WS_mat) */
            double m4[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            ok_tf T; double c7[7]; int i;
            for (i = 0; i < 12; ++i) m4[(i % 3) + 4 * (i / 3)] = r.model[i];
            ok_tf_from_m4(&T, m4, 1);
            c7[0] = T.r[0]; c7[1] = T.r[1]; c7[2] = T.r[2]; c7[3] = T.q.x; c7[4] = T.q.y; c7[5] = T.q.z; c7[6] = T.q.w;
            if (initialise_pose) f->est.set_pose(f->est.ctx, frame, c7);
        }
        ok = 1;
    }
    free(r.inliers); free(ci); free(ki);
    return ok;
}

/* returns the C++ int result (unused by the caller apart from rotationOnly) */
static int run_ransac_2d2d(ok_fe* f, uint64_t cur, uint64_t older, int initialise_pose, int remove_outliers, int* rotation_only) {
    int ncam = ok_vsb_frame(f->b, cur)->ncam, im, total = 0, rot_success = 0, rel_success = 0;
    *rotation_only = 0;
    for (im = 0; im < ncam; ++im) {
        int *ia, *ib, nc, k;
        ok_fe_ransac rr, rp;
        int rot_inl, rel_inl; float rot_ratio, rel_ratio;
        unsigned char* inl;
        nc = rel_correspondences(f, older, cur, im, &ia, &ib);
        if (nc < 10) { free(ia); free(ib); continue; }
        memset(&rr, 0, sizeof rr); memset(&rp, 0, sizeof rp);
        if (!f->est.ransac || !f->est.ransac(f->est.ctx, 1, nc, &rr)) { free(ia); free(ib); return -1; }
        rot_inl = rr.ninliers; rot_ratio = (float)rot_inl / (float)nc;
        if (!f->est.ransac(f->est.ctx, 2, nc, &rp)) { free(rr.inliers); free(ia); free(ib); return -1; }
        rel_inl = rp.ninliers; rel_ratio = (float)rel_inl / (float)nc;
        inl = (unsigned char*)calloc((size_t)nc, 1);
        if (rot_ratio > rel_ratio || rot_ratio > 0.8f) {
            if (rot_inl > 10) rot_success = 1;
            *rotation_only = 1;
            total += rot_inl;
            for (k = 0; k < rr.ninliers; ++k) if (rr.inliers[k] >= 0 && rr.inliers[k] < nc) inl[rr.inliers[k]] = 1;
        } else {
            if (rel_inl > 10 && rel_ratio > 0.8f) rel_success = 1;
            total += rel_inl;
            for (k = 0; k < rp.ninliers; ++k) if (rp.inliers[k] >= 0 && rp.inliers[k] < nc) inl[rp.inliers[k]] = 1;
        }
        free(rr.inliers); free(rp.inliers);
        if (!rot_success && !rel_success) { free(inl); free(ia); free(ib); continue; }
        {
            const ok_vsb_frame_view* mf = ok_vsb_frame(f->b, cur);
            for (k = 0; k < nc; ++k) {
                const int idxB = ib[k];
                if (remove_outliers && !inl[k]) {
                    if (mf->cam[im].lm[idxB] != 0) f->est.remove_observation(f->est.ctx, cur, (uint32_t)im, (uint32_t)idxB);
                }
            }
        }
        (void)initialise_pose;
        free(inl); free(ia); free(ib);
    }
    if (rel_success || rot_success) return total;
    *rotation_only = 1;
    return -1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * removeOutliers
 * ---------------------------------------------------------------------------------------------------------------- */
static int remove_outliers(ok_fe* f, uint64_t frame) {
    const ok_vsb_frame_view* cf = ok_vsb_frame(f->b, frame);
    ok_tf T_WS;
    int im, k, ctr = 0;
    pose_tf(f, frame, &T_WS);
    for (im = 0; im < cf->ncam; ++im) {
        ok_tf T_SC, T_WCi, T_CiW;
        const ok_cam* cam = ok_vsb_camera(f->b, im);
        tsc_tf(cf, im, &T_SC);
        ok_tf_mul(&T_WS, &T_SC, &T_WCi, 1);
        ok_tf_inverse(&T_WCi, &T_CiW, 1);
        for (k = 0; k < cf->cam[im].nkp; ++k) {
            const uint64_t lmId = cf->cam[im].lm[k];
            ok_vg_lm_view lv;
            if (!lmId) continue;
            if (ok_vg_landmark_find(G0(f), lmId, &lv)) {
                double pt[2], proj[2], hp_Ci[4], d[2];
                int remove = 0;
                pt[0] = (double)cf->cam[im].kp[3 * k]; pt[1] = (double)cf->cam[im].kp[3 * k + 1];
                ok_tf_mul_v4(&T_CiW, lv.hp, hp_Ci, 1);
                if (ok_cam_project_h(cam, hp_Ci, proj) == OK_PROJ_SUCCESSFUL) {
                    d[0] = proj[0] - pt[0]; d[1] = proj[1] - pt[1];
                    if (norm2(d) > 4.0) remove = 1;
                } else remove = 1;
                if (!remove) ctr++;
                else f->est.remove_observation(f->est.ctx, frame, (uint32_t)im, (uint32_t)k);
            }
        }
    }
    return ctr;
}

/* ------------------------------------------------------------------------------------------------------------------
 * doWeNeedANewKeyframe
 * ---------------------------------------------------------------------------------------------------------------- */
static int f2i(float v) { return (int)lrintf(v); }          /* cvRound(float) */

/* the matches / detections masks of one multiframe against the landmark filter (NULL = every non-zero id), added to the
 * counts; `images_cleared` frames are empty Mats (rows = cols = 0) and contribute nothing */
static void kf_counts(const ok_vsb_frame_view* fr, const ok_idset* only, ok_idset* lm_out, int* inter, int* uni, int* nkp_total) {
    int im, k, p, np;
    *inter = 0; *uni = 0;
    for (im = 0; im < fr->ncam; ++im) {
        const ok_vsb_cam_view* c = &fr->cam[im];
        int rows = c->images_cleared ? 0 : c->rows / 10, cols = c->images_cleared ? 0 : c->cols / 10;
        unsigned char *det, *mat;
        double radius = (double)(rows < cols ? rows : cols) * 0.09;
        np = rows * cols;
        if (nkp_total) *nkp_total += c->nkp;
        det = (unsigned char*)calloc((size_t)(np ? np : 1), 1);
        mat = (unsigned char*)calloc((size_t)(np ? np : 1), 1);
        if (np > 0)
            for (k = 0; k < c->nkp; ++k) {
                const float px = (float)((double)c->kp[3 * k] * 0.1), py = (float)((double)c->kp[3 * k + 1] * 0.1);
                const uint64_t id = c->lm[k];
                ok_vsb_circle_filled(det, rows, cols, f2i(px), f2i(py), (int)radius);
                if (id != 0 && (only ? ok_idset_has(only, id) : 1)) {
                    ok_vsb_circle_filled(mat, rows, cols, f2i(px), f2i(py), (int)radius);
                    if (lm_out) ok_idset_add(lm_out, id);
                }
            }
        else if (lm_out)
            for (k = 0; k < c->nkp; ++k) if (c->lm[k] != 0) ok_idset_add(lm_out, c->lm[k]);
        for (p = 0; p < np; ++p) {
            if (mat[p] && det[p]) (*inter)++;
            if (mat[p] || det[p]) (*uni)++;
        }
        free(det); free(mat);
    }
}

static int do_we_need_a_new_keyframe(ok_fe* f, uint64_t frame) {
    const ok_vsb_frame_view* cf = ok_vsb_frame(f->b, frame);
    ok_idset lm_ids, all;
    int inter, uni, num_kp = 0, age, i;
    double overlap, overlap_others = 0.0;
    memset(&lm_ids, 0, sizeof lm_ids); memset(&all, 0, sizeof all);
    if (ok_vsb_num_frames(f->b) < 4) return 1;
    if (!f->is_initialised) return 0;
    kf_counts(cf, NULL, &lm_ids, &inter, &uni, &num_kp);
    overlap = (double)inter / (double)uni;
    {   /* allFrames: keyFrames_ + loopClosureFrames_ + the keyframes of the IMU window (by age) */
        const ok_idset* kf = ok_vsb_key_frames(f->b); const ok_idset* lc = ok_vsb_loop_closure_frames(f->b);
        const ok_vg* g0 = G0(f);
        const int ns = ok_vg_state_count(g0);
        for (i = 0; i < kf->n; ++i) ok_idset_add(&all, kf->a[i]);
        for (i = 0; i < lc->n; ++i) ok_idset_add(&all, lc->a[i]);
        for (age = 0; age < ok_vsb_num_frames(f->b); ++age) {
            ok_vg_state_view sv;
            if (age >= ns) break;
            ok_vg_state_at(g0, ns - 1 - age, &sv);
            if (!ok_vsb_is_in_imu_window(f->b, sv.id)) break;
            if (sv.is_kf) ok_idset_add(&all, sv.id);
        }
    }
    for (i = 0; i < all.n; ++i) {
        const ok_vsb_frame_view* of = ok_vsb_frame(f->b, all.a[i]);
        int in2, un2;
        double o;
        if (!of) continue;
        kf_counts(of, &lm_ids, NULL, &in2, &un2, NULL);
        o = (double)in2 / (double)un2;
        overlap_others = stdmax(overlap_others, o);
    }
    overlap = (overlap_others < overlap) ? overlap_others : overlap;                     /* std::min(overlapOthers, overlap) */
    ok_idset_free(&lm_ids); ok_idset_free(&all);
    if (num_kp < 7 * cf->ncam) return 0;
    if ((float)overlap > f->p.keyframe_overlap) return 0;
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------
 * matchToMap
 * ---------------------------------------------------------------------------------------------------------------- */
typedef struct l2m {
    uint64_t id;
    int is3d;
    double proj[2];
    int nd;
    unsigned char desc[3][48];
    double e_W[3][3], r_W[3][3];
} l2m;

static int cmp_u64(const void* a, const void* b) { const uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b; return x < y ? -1 : x > y; }
static int idset_has_arr(const uint64_t* a, int n, uint64_t id) { return n >= 0 && a && bsearch(&id, a, (size_t)n, sizeof(uint64_t), cmp_u64) != NULL; }

typedef struct snap_lm { uint64_t id; ok_vg_lm_view v; ok_vg_kid* kids; int nobs; } snap_lm;
static int match_to_map(ok_fe* f, uint64_t cur, const uint64_t* lc, int nlc, int use_lc) {
    const ok_vg* g0 = G0(f);
    const ok_vsb_frame_view* cf;
    int ncam, im, k, nl, li, ctr = 0;
    ok_tf T_WS1;
    double repr_err = 0.0;
    l2m* lists[OK_FE_MAXCAM]; int nlists[OK_FE_MAXCAM];
    uint64_t *old_ids = NULL, *new_ids = NULL; int nold = 0, capold = 0;
    int num_init_iter = 2, second_ransac = 0, run_ransac;
    snap_lm* snap;
    if (ok_vsb_num_frames(f->b) < 2) return 0;
    cf = ok_vsb_frame(f->b, cur);
    ncam = cf->ncam;
    nl = ok_vg_landmark_count(g0);
    pose_tf(f, cur, &T_WS1);
    memset(lists, 0, sizeof lists); memset(nlists, 0, sizeof nlists);
    /* estimator.getLandmarks(pointMap): ONE snapshot (points, quality, observation sets) for all cameras */
    snap = (snap_lm*)calloc((size_t)(nl ? nl : 1), sizeof(snap_lm));
    for (li = 0; li < nl; ++li) {
        snap[li].id = ok_vg_landmark_id_at(g0, li);
        ok_vg_landmark_find(g0, snap[li].id, &snap[li].v);
        snap[li].nobs = ok_vg_landmark_obs(g0, snap[li].id, &snap[li].kids);
    }

    for (im = 0; im < ncam; ++im) {
        const ok_cam* cam = ok_vsb_camera(f->b, im);
        const int nkp = cf->cam[im].nkp;
        const double fl = cam_f(cam);
        const double repr_thr = f->p.imu_use ? 3.0 + fl * 0.06 : 3.0 + fl * 0.34;
        double max_u, max_v, focal_length;
        ok_tf T_SC, T_WC1, T_CW1;
        l2m* list = NULL; int n_list = 0, cap_list = 0;
        double* distances; uint64_t* lm_ids; size_t* ctrs; double* repr_errors;
        int t, nt = f->p.num_matching_threads;
        const fe_frame* fr = fe_fr(f, cur);
        if (nkp == 0) continue;
        max_u = (double)cam->w + repr_thr; max_v = (double)cam->h + repr_thr;
        tsc_tf(cf, im, &T_SC);
        ok_tf_mul(&T_WS1, &T_SC, &T_WC1, 1);
        ok_tf_inverse(&T_WC1, &T_CW1, 1);
        focal_length = cam->fu + cam->fv;
        for (li = 0; li < nl; ++li) {
            const uint64_t lid = snap[li].id;
            const ok_vg_lm_view lv = snap[li].v;
            l2m L; double p_W[3], r_W[3], e_W[3], rr, hp_C[4], kp[2];
            double best_scores[3] = {1.0, 1.0, 1.0};
            ok_vg_kid* kids = snap[li].kids; int nobs = snap[li].nobs, oi, o = 0;
            ok_proj_status st;
            if (use_lc && !idset_has_arr(lc, nlc, lid)) continue;
            memset(&L, 0, sizeof L);
            L.id = lid; L.is3d = 0;
            div3(lv.hp, lv.hp[3], p_W);
            sub3(p_W, T_WC1.r, r_W);
            nrm3(r_W, e_W);
            rr = stdmax(0.01, norm3(r_W));
            ok_tf_mul_v4(&T_CW1, lv.hp, hp_C, 1);
            st = ok_cam_project_h(cam, hp_C, kp);
            if (st == OK_PROJ_INVALID || st == OK_PROJ_BEHIND) continue;
            if (kp[0] < -repr_thr) continue;
            if (kp[1] < -repr_thr) continue;
            if (kp[0] > max_u) continue;
            if (kp[1] > max_v) continue;
            L.proj[0] = kp[0]; L.proj[1] = kp[1];
            for (oi = nobs - 1; oi >= 0; --oi) {
                const ok_vg_kid kid = kids[oi];
                ok_tf T_SC_old, T_WS_old, T_WC_old;
                double r_W_old[3], tmp[3], r_close_W[3], cos_vc, scale_change, score, worst_score = 0.0;
                int n, worst_idx = 0;
                tsc_tf(cf, (int)kid.cam, &T_SC_old);
                pose_tf(f, kid.frame, &T_WS_old);
                ok_tf_mul(&T_WS_old, &T_SC_old, &T_WC_old, 1);
                div3(lv.hp, lv.hp[3], tmp);
                sub3(tmp, T_WC_old.r, r_W_old);
                if (!L.is3d) {
                    double a[3], b2[3], c;
                    scale3(0.2 / focal_length / lv.quality, r_W_old, tmp);
                    sub3(r_W, tmp, r_close_W);
                    nrm3(r_W, a); nrm3(r_close_W, b2);
                    c = dot3(a, b2);
                    if (c > cos(10.0 / focal_length)) L.is3d = 1;
                }
                nrm3(r_W_old, tmp);
                cos_vc = dot3(e_W, tmp);
                if (cos_vc < cos(0.6) && !use_lc) continue;
                scale_change = fabs(rr - norm3(r_W_old)) / rr;
                if (scale_change > 0.5 && !use_lc) continue;
                score = 0.5 * (acos(cos_vc) / 0.6 + scale_change / 0.5);
                for (n = 0; n < 3; ++n) if (best_scores[n] > worst_score) { worst_score = best_scores[n]; worst_idx = n; }
                if (score < best_scores[worst_idx]) {
                    const fe_frame* of = fe_fr(f, kid.frame);
                    double e_C[3], en[3], Ce[3];
                    if (of && (int)kid.cam < of->ncam && (int)kid.kp < of->cam[kid.cam].nkp) {
                        memcpy(L.desc[worst_idx], of->cam[kid.cam].desc + 48 * (size_t)kid.kp, 48);
                        memcpy(e_C, of->cam[kid.cam].bp + 3 * (size_t)kid.kp, sizeof e_C);
                    } else memset(e_C, 0, sizeof e_C);
                    nrm3(e_C, en);
                    {   double C9[9]; memcpy(C9, T_WC_old.C, sizeof C9); ok_m3_mulv(C9, en, Ce); }
                    memcpy(L.e_W[worst_idx], Ce, sizeof Ce);
                    memcpy(L.r_W[worst_idx], T_WC_old.r, sizeof(double) * 3);
                    if (worst_idx > o) o = worst_idx;
                    best_scores[worst_idx] = score;
                }
            }
            L.nd = o + 1;
            if (n_list == cap_list) { cap_list = cap_list ? 2 * cap_list : 256; list = (l2m*)realloc(list, sizeof(l2m) * (size_t)cap_list); }
            list[n_list++] = L;
        }
        lists[im] = list; nlists[im] = n_list;

        /* multithreaded matching (the segments are run one after the other) */
        distances = (double*)malloc(sizeof(double) * (size_t)nkp); lm_ids = (uint64_t*)calloc((size_t)nkp, 8);
        ctrs = (size_t*)calloc((size_t)nt, sizeof(size_t)); repr_errors = (double*)calloc((size_t)nt, sizeof(double));
        for (k = 0; k < nkp; ++k) distances[k] = f->p.matching_threshold;
        for (t = 0; t < nt; ++t) {
            const size_t segment = (size_t)nkp / (size_t)nt, startK = segment * (size_t)t;
            const size_t endK = (size_t)t + 1 == (size_t)nt ? (size_t)nkp : startK + segment;
            const double thr2 = repr_thr * repr_thr;
            unsigned char* use = (unsigned char*)malloc((size_t)nkp);
            size_t kk; int ii;
            memset(use, 1, (size_t)nkp);
            for (kk = startK; kk < endK; ++kk) if (cf->cam[im].lm[kk] && !use_lc) use[kk] = 0;
            for (ii = 0; ii < n_list; ++ii) {
                const l2m* L = &list[ii];
                int d;
                if (!L->is3d) continue;
                if (use_lc && !idset_has_arr(lc, nlc, L->id)) continue;
                for (kk = startK; kk < endK; ++kk) {
                    double kpt[2], rd[2], r2;
                    if (!use[kk]) continue;
                    kpt[0] = (double)cf->cam[im].kp[3 * kk]; kpt[1] = (double)cf->cam[im].kp[3 * kk + 1];
                    rd[0] = L->proj[0] - kpt[0]; rd[1] = L->proj[1] - kpt[1];
                    r2 = dot2(rd, rd);
                    if (r2 > thr2) continue;
                    for (d = 0; d < L->nd; ++d) {
                        const double dist = (double)hamming48(fr->cam[im].desc + 48 * kk, L->desc[d]);
                        if (dist < distances[kk]) {
                            distances[kk] = dist; lm_ids[kk] = L->id; ctrs[t]++;
                            repr_errors[t] += sqrt(dot2(rd, rd));
                        }
                    }
                }
            }
            repr_errors[t] /= (double)ctrs[t];
            free(use);
        }
        for (t = 0; t < nt; ++t) repr_err += repr_errors[t];
        free(ctrs); free(repr_errors);
        /* insert the observations */
        for (k = 0; k < nkp; ++k) {
            const uint64_t previous = ok_vsb_frame(f->b, cur)->cam[im].lm[k];
            if (lm_ids[k]) {
                if (previous && use_lc) {
                    f->est.remove_observation(f->est.ctx, cur, (uint32_t)im, (uint32_t)k);
                    if (nold == capold) { capold = capold ? 2 * capold : 64; old_ids = (uint64_t*)realloc(old_ids, 8 * (size_t)capold); new_ids = (uint64_t*)realloc(new_ids, 8 * (size_t)capold); }
                    old_ids[nold] = previous; new_ids[nold] = lm_ids[k]; ++nold;
                }
                f->est.set_landmark_id(f->est.ctx, cur, (uint32_t)im, (uint32_t)k, lm_ids[k]);
                f->est.add_observation(f->est.ctx, lm_ids[k], cur, (uint32_t)im, (uint32_t)k, 1);
                ctr++;
            }
        }
        free(distances); free(lm_ids);
    }
    repr_err /= (double)((size_t)f->p.num_matching_threads * (size_t)ncam);

    /* remove outliers -- initialise the pose only without IMU or when matching with a large reprojection error */
    {
        const double fl0 = cam_f(ok_vsb_camera(f->b, 0));
        const double strict_thr = 3.0 + fl0 * 0.006;
        run_ransac = !f->p.imu_use;
        if (repr_err > strict_thr) {
            if (f->p.imu_use) run_ransac = 1;
            num_init_iter += 2;
        }
    }
    if (run_ransac) {
        const int success = run_ransac_3d2d(f, cur, run_ransac, 1);
        pose_tf(f, cur, &T_WS1);
        if (!success) { num_init_iter += 4; second_ransac = 1; }
    }
    if (!use_lc && ctr > 3) {
        f->est.optimise_realtime(f->est.ctx, num_init_iter, f->p.realtime_num_threads, 0, 1, f->is_initialised);
        remove_outliers(f, cur);
        f->est.optimise_realtime(f->est.ctx, 2, f->p.realtime_num_threads, 0, 1, f->is_initialised);
        pose_tf(f, cur, &T_WS1);
    }
    if (ctr <= 3 && f->is_initialised) second_ransac = 1;

    /* now the non-initialised (not yet 3d) ones */
    cf = ok_vsb_frame(f->b, cur);
    for (im = 0; im < ncam; ++im) {
        const ok_cam* cam = ok_vsb_camera(f->b, im);
        const int nkp = cf->cam[im].nkp;
        ok_tf T_SC, T_WC1, T_CW1;
        double *distances, *hps, focal, sigma, cos6s, *eWs;
        uint64_t *lm_ids, *previous_ids;
        unsigned char* use;
        int t, nt = f->p.num_matching_threads;
        const fe_frame* fr = fe_fr(f, cur);
        if (nkp == 0) continue;
        tsc_tf(cf, im, &T_SC);
        ok_tf_mul(&T_WS1, &T_SC, &T_WC1, 1);
        ok_tf_inverse(&T_WC1, &T_CW1, 1);
        distances = (double*)malloc(sizeof(double) * (size_t)nkp); lm_ids = (uint64_t*)calloc((size_t)nkp, 8);
        hps = (double*)calloc(4 * (size_t)nkp, sizeof(double));
        eWs = (double*)calloc(3 * (size_t)nkp, sizeof(double)); previous_ids = (uint64_t*)calloc((size_t)nkp, 8);
        use = (unsigned char*)calloc((size_t)nkp, 1);
        for (k = 0; k < nkp; ++k) distances[k] = f->p.matching_threshold;
        focal = 0.5 * (cam->fu + cam->fv);
        sigma = 1.0 / focal;
        cos6s = cos(6.0 * sigma);
        for (t = 0; t < nt; ++t) {
            const size_t segment = (size_t)nkp / (size_t)nt, startK = segment * (size_t)t;
            const size_t endK = (size_t)t + 1 == (size_t)nt ? (size_t)nkp : startK + segment;
            size_t kk; int ii, d;
            for (kk = startK; kk < endK; ++kk) {
                if (fr->cam[im].bp_ok[kk]) {
                    double e1_C[3], e1n[3], e1_W[3], C9[9];
                    memcpy(e1_C, fr->cam[im].bp + 3 * kk, sizeof e1_C);
                    nrm3(e1_C, e1n);
                    memcpy(C9, T_WC1.C, sizeof C9);
                    ok_m3_mulv(C9, e1n, e1_W);
                    memcpy(eWs + 3 * kk, e1_W, sizeof e1_W);
                    previous_ids[kk] = cf->cam[im].lm[kk];
                    if (previous_ids[kk] && !use_lc) continue;
                    use[kk] = 1;
                }
            }
            for (ii = 0; ii < nlists[im]; ++ii) {
                const l2m* L = &lists[im][ii];
                if (L->is3d) continue;
                if (use_lc && !idset_has_arr(lc, nlc, L->id)) continue;
                for (kk = startK; kk < endK; ++kk) {
                    if (!use[kk]) continue;
                    for (d = 0; d < L->nd; ++d) {
                        const double dist = (double)hamming48(fr->cam[im].desc + 48 * kk, L->desc[d]);
                        if (dist < distances[kk]) {
                            const double* e1_W = eWs + 3 * kk;
                            const double *e0_W = L->e_W[d], *r0_W = L->r_W[d];
                            double hp_W[4]; int valid = 0, parallel = 0; double p_W[3], dd[3];
                            if (dot3(e0_W, e1_W) < cos6s) {
                                double et_W[3], n0[3], n1[3], c0[3], c1[3], t2[3], t3[3];
                                sub3(T_WC1.r, r0_W, t2); nrm3(t2, et_W);
                                cross3(e0_W, et_W, c0); nrm3(c0, n0);
                                cross3(e1_W, et_W, c1); nrm3(c1, n1);
                                if (dot3(n0, n1) < cos6s) continue;
                                cross3(e0_W, e1_W, t2);
                                add3(n0, n0, t3); nrm3(t3, t3);
                                if (dot3(t2, t3) > 0.0) continue;
                            }
                            ok_fe_triangulate_fast(r0_W, e0_W, T_WC1.r, e1_W, sigma, &valid, &parallel, hp_W);
                            if (!valid) continue;
                            div3(hp_W, hp_W[3], p_W);
                            sub3(p_W, r0_W, dd);
                            if (norm3(dd) < 0.2) valid = 0;
                            sub3(p_W, T_WC1.r, dd);
                            if (norm3(dd) < 0.2) valid = 0;
                            if (!valid) continue;
                            if (L->id == previous_ids[kk]) break;           /* the match is already done */
                            distances[kk] = dist; lm_ids[kk] = L->id;
                            if (!parallel) memcpy(hps + 4 * kk, hp_W, sizeof hp_W);
                        }
                    }
                }
            }
        }
        /* insert the observations */
        for (k = 0; k < nkp; ++k) {
            const uint64_t previous = ok_vsb_frame(f->b, cur)->cam[im].lm[k];
            ok_vg_lm_view mpt;
            int bad = 0;
            double pt1[2], pt1p[2];
            ok_proj_status s1;
            if (!lm_ids[k]) continue;
            if (previous && use_lc) {
                f->est.remove_observation(f->est.ctx, cur, (uint32_t)im, (uint32_t)k);
                if (nold == capold) { capold = capold ? 2 * capold : 64; old_ids = (uint64_t*)realloc(old_ids, 8 * (size_t)capold); new_ids = (uint64_t*)realloc(new_ids, 8 * (size_t)capold); }
                old_ids[nold] = previous; new_ids[nold] = lm_ids[k]; ++nold;
            }
            ok_vg_landmark_find(g0, lm_ids[k], &mpt);
            if (norm4(hps + 4 * k) > 1.0e-22) {
                int oi, no;
                ok_vg_kid* kids;
                no = ok_vg_landmark_obs(g0, lm_ids[k], &kids);
                for (oi = 0; oi < no && !bad; ++oi) {
                    const ok_vsb_frame_view* mf = ok_vsb_frame(f->b, kids[oi].frame);
                    const ok_cam* c2 = ok_vsb_camera(f->b, (int)kids[oi].cam);
                    ok_tf T_WS, T_SC2, ti1, ti2, tt; double ptc[2], ptp[2], hpC[4], d[2];
                    ptc[0] = (double)mf->cam[kids[oi].cam].kp[3 * kids[oi].kp]; ptc[1] = (double)mf->cam[kids[oi].cam].kp[3 * kids[oi].kp + 1];
                    pose_tf(f, kids[oi].frame, &T_WS);
                    extr_tf(f, kids[oi].frame, (int)kids[oi].cam, &T_SC2);
                    ok_tf_inverse(&T_SC2, &ti1, 1); ok_tf_inverse(&T_WS, &ti2, 1);
                    ok_tf_mul(&ti1, &ti2, &tt, 1);
                    ok_tf_mul_v4(&tt, hps + 4 * k, hpC, 1);
                    if (ok_cam_project_h(c2, hpC, ptp) == OK_PROJ_SUCCESSFUL) { d[0] = ptc[0] - ptp[0]; d[1] = ptc[1] - ptp[1]; bad = !(norm2(d) < 4.0); }
                    else bad = 1;
                }
                free(kids);
                if (bad) continue;
            }
            pt1[0] = (double)cf->cam[im].kp[3 * k]; pt1[1] = (double)cf->cam[im].kp[3 * k + 1];
            if (norm4(hps + 4 * k) > 1.0e-22 && !mpt.initialised) {
                double hc[4], d[2];
                ok_tf_mul_v4(&T_CW1, hps + 4 * k, hc, 1);
                s1 = ok_cam_project_h(cam, hc, pt1p);
                d[0] = pt1[0] - pt1p[0]; d[1] = pt1[1] - pt1p[1];
                if (!(s1 == OK_PROJ_SUCCESSFUL && norm2(d) < 4.0)) continue;
                f->est.set_landmark(f->est.ctx, lm_ids[k], hps + 4 * k, 1);
            } else {
                double hc[4], d[2];
                ok_tf_mul_v4(&T_CW1, mpt.hp, hc, 1);
                s1 = ok_cam_project_h(cam, hc, pt1p);
                d[0] = pt1[0] - pt1p[0]; d[1] = pt1[1] - pt1p[1];
                if (!(s1 == OK_PROJ_SUCCESSFUL && norm2(d) < 4.0)) continue;
            }
            f->est.set_landmark_id(f->est.ctx, cur, (uint32_t)im, (uint32_t)k, lm_ids[k]);
            f->est.add_observation(f->est.ctx, lm_ids[k], cur, (uint32_t)im, (uint32_t)k, 1);
            ctr++;
        }
        free(distances); free(lm_ids); free(hps); free(eWs); free(previous_ids); free(use);
    }

    /* merge landmarks, if loop-closure matching */
    if (use_lc) f->est.merge_landmarks(f->est.ctx, old_ids, new_ids, nold);

    /* final two steps optimisation */
    if (second_ransac) {
        const int success = run_ransac_3d2d(f, cur, second_ransac, 1);
        pose_tf(f, cur, &T_WS1);
        if (!success) num_init_iter += 4;
        f->est.optimise_realtime(f->est.ctx, num_init_iter, f->p.realtime_num_threads, 0, 1, f->is_initialised);
    }
    for (im = 0; im < ncam; ++im) free(lists[im]);
    for (li = 0; li < nl; ++li) free(snap[li].kids);
    free(snap);
    free(old_ids); free(new_ids);
    return ctr;
}

/* ------------------------------------------------------------------------------------------------------------------
 * matchMotionStereo
 * ---------------------------------------------------------------------------------------------------------------- */
typedef struct match_info { double hp_W[4]; size_t k1; int matching, initialisable; double quality; } match_info;

typedef struct ov_pair { double first; uint64_t second; } ov_pair;
static int ov_less(const ov_pair* a, const ov_pair* b) { return a->first < b->first || (!(b->first < a->first) && a->second < b->second); }
static void ov_sort(ov_pair* a, int n) {          /* libstdc++ std::sort of a short range = __insertion_sort */
    int i, j;
    for (i = 1; i < n; ++i) {
        ov_pair v = a[i];
        if (ov_less(&v, &a[0])) { for (j = i; j > 0; --j) a[j] = a[j - 1]; a[0] = v; }
        else { for (j = i; ov_less(&v, &a[j - 1]); --j) a[j] = a[j - 1]; a[j] = v; }
    }
}

static int match_motion_stereo(ok_fe* f, uint64_t cur, int* rotation_only) {
    const ok_vg* g0 = G0(f);
    int ret_ctr = 0, ncam = ok_vsb_frame(f->b, cur)->ncam, i, im;
    ok_idset all; ov_pair* ov; int nov = 0;
    uint64_t prev_id;
    int first_frame = 1;
    ok_tf T_WS1;
    const int ns = ok_vg_state_count(g0);
    ok_vg_state_view sv;
    *rotation_only = 1;
    pose_tf(f, cur, &T_WS1);
    memset(&all, 0, sizeof all);
    {
        const ok_idset* kf = ok_vsb_key_frames(f->b); const ok_idset* imu = ok_vsb_imu_frames(f->b);
        for (i = 0; i < kf->n; ++i) ok_idset_add(&all, kf->a[i]);
        for (i = 0; i < imu->n; ++i) ok_idset_add(&all, imu->a[i]);
        for (i = 0; i < imu->n; ++i) {
            ok_vg_state_view v;
            if (ok_vg_state_find(g0, imu->a[i], &v) && !v.is_kf) ok_idset_del(&all, imu->a[i]);
        }
    }
    ok_vg_state_at(g0, ns - 2, &sv);                       /* stateIdByAge(1) */
    prev_id = sv.id;
    ov = (ov_pair*)malloc(sizeof(ov_pair) * (size_t)(all.n + 1));
    for (i = 0; i < all.n; ++i) {
        ov[nov].second = all.a[i];
        ov[nov].first = all.a[i] == prev_id ? 1.0 : ok_vsb_overlap_fraction(f->b, prev_id, all.a[i]);
        ++nov;
    }
    ov_sort(ov, nov);
    {
        uint64_t* match_ids = (uint64_t*)malloc(8 * (size_t)(nov + 1)); int nmatch = 0, mi;
        for (i = 0; i < nov; ++i) {
            if (ov[nov - i - 1].first <= 1.0e-8) break;
            match_ids[nmatch++] = ov[nov - i - 1].second;
        }
        for (mi = 0; mi < nmatch; ++mi) {
            const uint64_t older = match_ids[mi];
            ok_tf T_WS0;
            int rot_tmp = 0;
            pose_tf(f, older, &T_WS0);
            for (im = 0; im < ncam; ++im) {
                ok_tf T_SC0, T_SC1, T_WC0, T_WC1, T_C0W, T_C1W;
                const ok_vsb_frame_view *mf0 = ok_vsb_frame(f->b, older), *mf1 = ok_vsb_frame(f->b, cur);
                const fe_frame *fr0 = fe_fr(f, older), *fr1 = fe_fr(f, cur);
                const int k0Size = mf0->cam[im].nkp, k1Size = mf1->cam[im].nkp;
                const ok_cam* cam = ok_vsb_camera(f->b, im);
                const double f0 = cam_f(cam);
                int *k1s = (int*)malloc(sizeof(int) * (size_t)(k1Size + 1)), nk1s = 0, k1, k0;
                unsigned char* desc1 = (unsigned char*)malloc(48 * (size_t)(k1Size + 1));
                match_info* mi_arr = (match_info*)calloc((size_t)(k0Size + 1), sizeof(match_info));
                extr_tf(f, older, im, &T_SC0); extr_tf(f, cur, im, &T_SC1);
                ok_tf_mul(&T_WS0, &T_SC0, &T_WC0, 1); ok_tf_mul(&T_WS1, &T_SC1, &T_WC1, 1);
                ok_tf_inverse(&T_WC0, &T_C0W, 1); ok_tf_inverse(&T_WC1, &T_C1W, 1);
                for (k1 = 0; k1 < k1Size; ++k1) {
                    if (mf1->cam[im].lm[k1]) continue;
                    memcpy(desc1 + 48 * (size_t)nk1s, fr1->cam[im].desc + 48 * (size_t)k1, 48);
                    k1s[nk1s++] = k1;
                }
                for (k0 = 0; k0 < k0Size; ++k0) {                  /* the threads stride over k0; the work items are independent */
                    uint64_t id0 = mf0->cam[im].lm[k0];
                    double distances;                              /* uint32_t in C++ */
                    int initialisable = 0, kk;
                    double quality = 0.0, hps_W[4] = {0, 0, 0, 0};
                    size_t k1_max = 1000;
                    double size0, e0_C[3], e0n[3], e0_W[3], sigma, C9[9];
                    if (id0) {
                        if (!lm_added(f, id0)) continue;
                        if (lm_initialised(f, id0)) continue;
                    }
                    distances = (double)(uint32_t)f->p.matching_threshold;
                    size0 = (double)mf0->cam[im].kp[3 * k0 + 2];
                    if (!fr0->cam[im].bp_ok[k0]) continue;
                    memcpy(e0_C, fr0->cam[im].bp + 3 * (size_t)k0, sizeof e0_C);
                    memcpy(C9, T_WC0.C, sizeof C9);
                    ok_m3_mulv(C9, e0_C, e0n); nrm3(e0n, e0_W);
                    sigma = size0 / f0 * 0.125;
                    if (is_observed(f, older, (uint32_t)im, (uint32_t)k0)) continue;
                    for (kk = 0; kk < nk1s; ++kk) {
                        const uint32_t dist = (uint32_t)hamming48(fr0->cam[im].desc + 48 * (size_t)k0, desc1 + 48 * (size_t)kk);
                        if ((double)dist < distances) {
                            const int kq = k1s[kk];
                            int valid = 0, parallel = 0;
                            double e1_C[3], e1n[3], e1_W[3], hp_W[4], hp_C0[4], hp_C1[4], C1[9], dd;
                            if (!fr1->cam[im].bp_ok[kq]) continue;
                            memcpy(e1_C, fr1->cam[im].bp + 3 * (size_t)kq, sizeof e1_C);
                            memcpy(C1, T_WC1.C, sizeof C1);
                            ok_m3_mulv(C1, e1_C, e1n); nrm3(e1n, e1_W);
                            if (dot3(e0_W, e1_W) < 0.5) continue;
                            ok_fe_triangulate_fast(T_WC0.r, e0_W, T_WC1.r, e1_W, sigma, &valid, &parallel, hp_W);
                            if (!valid) continue;
                            ok_tf_mul_v4(&T_C0W, hp_W, hp_C0, 1);
                            ok_tf_mul_v4(&T_C1W, hp_W, hp_C1, 1);
                            dd = dot3(e0_W, e1_W);
                            if (dd < 0.8) valid = 0;
                            { const double w = hp_W[3]; hp_W[0] = hp_W[0] / w; hp_W[1] = hp_W[1] / w; hp_W[2] = hp_W[2] / w; hp_W[3] = hp_W[3] / w; }
                            if (hp_C0[2] / hp_C0[3] < 0.2) valid = 0;
                            if (hp_C1[2] / hp_C1[3] < 0.2) valid = 0;
                            if (valid) {
                                double a[3], b2[3], t1[3], t2[3];
                                k1_max = (size_t)kq; distances = (double)dist;
                                sub3(hp_W, T_WC0.r, t1); nrm3(t1, a);
                                sub3(hp_W, T_WC1.r, t2); nrm3(t2, b2);
                                quality = acos(dot3(a, b2));
                                memcpy(hps_W, hp_W, sizeof hps_W);
                                initialisable = !parallel;
                            }
                        }
                    }
                    if (distances < f->p.matching_threshold) {
                        double pt1[2], pt1p[2], hc[4], d[2];
                        ok_proj_status s1;
                        pt1[0] = (double)mf1->cam[im].kp[3 * k1_max]; pt1[1] = (double)mf1->cam[im].kp[3 * k1_max + 1];
                        ok_tf_mul_v4(&T_C1W, hps_W, hc, 1);
                        s1 = ok_cam_project_h(cam, hc, pt1p);
                        d[0] = pt1[0] - pt1p[0]; d[1] = pt1[1] - pt1p[1];
                        if (s1 == OK_PROJ_SUCCESSFUL && norm2(d) < 4.0) {
                            match_info m; memcpy(m.hp_W, hps_W, sizeof hps_W); m.k1 = k1_max; m.matching = 1; m.initialisable = initialisable; m.quality = quality;
                            mi_arr[k0] = m;
                        }
                    }
                }
                /* finally insert the actual matches */
                for (k0 = 0; k0 < k0Size; ++k0) {
                    const match_info* m = &mi_arr[k0];
                    uint64_t id0 = ok_vsb_frame(f->b, older)->cam[im].lm[k0], id1;
                    if (!m->matching) continue;
                    if (id0) {
                        if (lm_initialised(f, id0)) continue;
                        if (!lm_added(f, id0)) continue;
                    }
                    if (is_observed(f, older, (uint32_t)im, (uint32_t)k0)) continue;
                    id1 = ok_vsb_frame(f->b, cur)->cam[im].lm[m->k1];
                    if (id1) continue;
                    if (id0) {
                        ok_vg_lm_view lm;
                        ok_vg_landmark_find(g0, id0, &lm);
                        if (lm.quality < m->quality) f->est.set_landmark(f->est.ctx, id0, m->hp_W, m->initialisable);
                    } else {
                        id0 = f->est.add_landmark(f->est.ctx, m->hp_W, m->initialisable);
                        f->est.set_landmark_id(f->est.ctx, older, (uint32_t)im, (uint32_t)k0, id0);
                        f->est.add_observation(f->est.ctx, id0, older, (uint32_t)im, (uint32_t)k0, 1);
                    }
                    f->est.set_landmark_id(f->est.ctx, cur, (uint32_t)im, (uint32_t)m->k1, id0);
                    f->est.add_observation(f->est.ctx, id0, cur, (uint32_t)im, (uint32_t)m->k1, 1);
                    ret_ctr++;
                }
                free(k1s); free(desc1); free(mi_arr);
            }
            if (!f->is_initialised) run_ransac_2d2d(f, cur, older, 1, 1, &rot_tmp);
            if (first_frame) { *rotation_only = rot_tmp; first_frame = 0; }
        }
        free(match_ids);
    }
    free(ov); ok_idset_free(&all);
    return ret_ctr;
}

/* ------------------------------------------------------------------------------------------------------------------
 * matchStereo
 * ---------------------------------------------------------------------------------------------------------------- */
static void match_stereo(ok_fe* f, uint64_t mf_id, int as_keyframe) {
    const ok_vg* g0 = G0(f);
    const ok_vsb_frame_view* mf = ok_vsb_frame(f->b, mf_id);
    const fe_frame* fr = fe_fr(f, mf_id);
    const int ncam = mf->ncam;
    ok_tf T_WS;
    int im0, im1;
    pose_tf(f, mf_id, &T_WS);
    for (im0 = 0; im0 < ncam; ++im0) {
        ok_tf T_SC0;
        tsc_tf(mf, im0, &T_SC0);
        for (im1 = im0 + 1; im1 < ncam; ++im1) {
            ok_tf T_SC1, T_WC0, T_WC1, T_C0W, T_C1W;
            const ok_cam *cam0 = ok_vsb_camera(f->b, im0), *cam1 = ok_vsb_camera(f->b, im1);
            double f0, f1;
            int k0, k1;
            if (!has_overlap(f, im0, im1)) continue;
            tsc_tf(mf, im1, &T_SC1);
            ok_tf_mul(&T_WS, &T_SC0, &T_WC0, 1); ok_tf_mul(&T_WS, &T_SC1, &T_WC1, 1);
            ok_tf_inverse(&T_WC0, &T_C0W, 1); ok_tf_inverse(&T_WC1, &T_C1W, 1);
            f0 = cam_f(cam0); f1 = cam_f(cam1);
            for (k0 = 0; k0 < mf->cam[im0].nkp; ++k0) {
                double distances = f->p.matching_threshold;
                int initialisable = 0;
                double hps_W[4] = {0, 0, 0, 0};
                size_t k1_match = 0;
                mf = ok_vsb_frame(f->b, mf_id);
                for (k1 = 0; k1 < mf->cam[im1].nkp; ++k1) {
                    const double dist = (double)hamming48(fr->cam[im0].desc + 48 * (size_t)k0, fr->cam[im1].desc + 48 * (size_t)k1);
                    if (dist < distances) {
                        double size0 = (double)mf->cam[im0].kp[3 * k0 + 2], size1 = (double)mf->cam[im1].kp[3 * k1 + 2];
                        double sigma = stdmax(size0 / f0, size1 / f1) * 0.125;
                        double e0_C[3], e1_C[3], e0n[3], e1n[3], e0_W[3], e1_W[3], hp_W[4], hp_C0[4], hp_C1[4], C0[9], C1[9];
                        int valid = 0, parallel = 0;
                        if (!fr->cam[im0].bp_ok[k0]) continue;
                        if (!fr->cam[im1].bp_ok[k1]) continue;
                        memcpy(e0_C, fr->cam[im0].bp + 3 * (size_t)k0, sizeof e0_C); memcpy(e1_C, fr->cam[im1].bp + 3 * (size_t)k1, sizeof e1_C);
                        memcpy(C0, T_WC0.C, sizeof C0); memcpy(C1, T_WC1.C, sizeof C1);
                        ok_m3_mulv(C0, e0_C, e0n); nrm3(e0n, e0_W);
                        ok_m3_mulv(C1, e1_C, e1n); nrm3(e1n, e1_W);
                        ok_fe_triangulate_fast(T_WC0.r, e0_W, T_WC1.r, e1_W, sigma, &valid, &parallel, hp_W);
                        ok_tf_mul_v4(&T_C0W, hp_W, hp_C0, 1);
                        ok_tf_mul_v4(&T_C1W, hp_W, hp_C1, 1);
                        { const double w = hp_W[3]; hp_W[0] = hp_W[0] / w; hp_W[1] = hp_W[1] / w; hp_W[2] = hp_W[2] / w; hp_W[3] = hp_W[3] / w; }
                        if (hp_C0[2] / hp_C0[3] < 0.1) valid = 0;
                        if (hp_C1[2] / hp_C1[3] < 0.1) valid = 0;
                        if (dot3(e0_W, e1_W) < 0.8) valid = 0;
                        if (valid) { distances = dist; memcpy(hps_W, hp_W, sizeof hps_W); k1_match = (size_t)k1; initialisable = !parallel; }
                    }
                }
                if (distances < f->p.matching_threshold) {
                    uint64_t lm_id = 0, id0 = mf->cam[im0].lm[k0], id1 = mf->cam[im1].lm[k1_match];
                    int add0 = 0, add1 = 0;
                    if (id0 && id1) {
                        if (id0 != id1) { f->est.merge_landmark(f->est.ctx, id1, id0); id1 = id0; }
                        if (!lm_initialised(f, id0)) {
                            if (initialisable) f->est.set_landmark(f->est.ctx, id0, hps_W, 1);
                        }
                    } else if (id1) { lm_id = id1; add0 = 1; }
                    else if (id0) { lm_id = id0; add1 = 1; }
                    else {
                        if (!as_keyframe) continue;
                        add0 = 1; add1 = 1;
                        lm_id = f->est.add_landmark(f->est.ctx, hps_W, initialisable);
                    }
                    if (add0) {
                        ok_vg_lm_view lv; double pt0[2], pt0p[2], hc[4], d[2];
                        ok_vg_landmark_find(g0, lm_id, &lv);
                        pt0[0] = (double)mf->cam[im0].kp[3 * k0]; pt0[1] = (double)mf->cam[im0].kp[3 * k0 + 1];
                        ok_tf_mul_v4(&T_C0W, lv.hp, hc, 1);
                        if (ok_cam_project_h(cam0, hc, pt0p) == OK_PROJ_SUCCESSFUL) {
                            d[0] = pt0[0] - pt0p[0]; d[1] = pt0[1] - pt0p[1];
                            if (norm2(d) < 4.0) {
                                f->est.set_landmark_id(f->est.ctx, mf_id, (uint32_t)im0, (uint32_t)k0, lm_id);
                                f->est.add_observation(f->est.ctx, lm_id, mf_id, (uint32_t)im0, (uint32_t)k0, 1);
                            }
                        }
                    }
                    if (add1) {
                        ok_vg_lm_view lv; double pt1[2], pt1p[2], hc[4], d[2];
                        ok_vg_landmark_find(g0, lm_id, &lv);
                        pt1[0] = (double)mf->cam[im1].kp[3 * k1_match]; pt1[1] = (double)mf->cam[im1].kp[3 * k1_match + 1];
                        ok_tf_mul_v4(&T_C1W, lv.hp, hc, 1);
                        if (ok_cam_project_h(cam1, hc, pt1p) == OK_PROJ_SUCCESSFUL) {
                            d[0] = pt1[0] - pt1p[0]; d[1] = pt1[1] - pt1p[1];
                            if (norm2(d) < 4.0) {
                                f->est.set_landmark_id(f->est.ctx, mf_id, (uint32_t)im1, (uint32_t)k1_match, lm_id);
                                f->est.add_observation(f->est.ctx, lm_id, mf_id, (uint32_t)im1, (uint32_t)k1_match, 1);
                            }
                        }
                    }
                }
            }
        }
    }
}

/* ------------------------------------------------------------------------------------------------------------------
 * dataAssociationAndInitialization
 * ---------------------------------------------------------------------------------------------------------------- */
int ok_fe_data_association(ok_fe* f, uint64_t frame, int* as_keyframe) {
    int num3d = 0;
    if (ok_vsb_num_frames(f->b) > 1) {
        int rotation_only = 0;
        num3d = match_to_map(f, frame, NULL, 0, 0);
        /* trackingQuality is only logged (trackingLost_ stays false) */
        match_motion_stereo(f, frame, &rotation_only);
        if (!rotation_only || num3d > 5) {
            if (!f->is_initialised) f->is_initialised = 1;
        }
        *as_keyframe = do_we_need_a_new_keyframe(f, frame);
    } else {
        *as_keyframe = 1;
    }
    /* loop closures: place recognition, attemptLoopClosure, addLoopClosureFrame, matchToMap against the loop closure landmarks */
    if (f->p.do_loop_closures && !ok_vsb_is_loop_closing(f->b) && !ok_vsb_is_loop_closure_available(f->b)
        && !ok_vsb_needs_full_graph_optimisation(f->b) && f->is_initialised && f->est.place_recognition) {
        uint64_t* lc = NULL; int nlc = 0;
        if (f->est.place_recognition(f->est.ctx, frame, &lc, &nlc)) {
            match_to_map(f, frame, lc, nlc, 1);
            *as_keyframe = do_we_need_a_new_keyframe(f, frame);
        }
        free(lc);
    }
    /* do stereo match -- get new landmarks only when this is a keyframe */
    if (*as_keyframe) match_stereo(f, frame, *as_keyframe);
    remove_outliers(f, frame);
    f->est.clean_unobserved_landmarks(f->est.ctx);
    return 1;
}
