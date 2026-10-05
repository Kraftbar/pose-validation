/* OK_PORT_SOURCES: check_ok_frontend.c ok_dbow.c ok_place.c ok_place_dist.c ok_opengv.c ok_opengv_gp3p_gen.c ok_opengv_stew_gen.c ok_eigen_eigsolver8.c ok_eigen_eigsolver10.c ok_eigen_cx.c ok_eigen_svd.c ok_eigen_qr.c ok_eigen_fullpivlu.c ok_frontend.c ok_vslam.c ok_vsb_geom.c ok_vsolve.c ok_solve.c ok_solve_linear.c ok_sparse.c ok_amd.c ok_vigraph.c ok_problem.c ok_graph.c ok_twopose.c ok_err.c ok_param.c ok_triangulate.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_dense.c ok_blas.c */
/* Bit-exactness harness for okvis_port module 7b (the frontend data association: matchToMap, matchMotionStereo,
 * matchStereo, removeOutliers, doWeNeedANewKeyframe, triangulateFast, the OpenGV adapters' correspondence lists).
 *
 *   check_ok_frontend <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_backend_calls]
 *
 * Replays <dump_dir>/problem.bin (patches 0009-0012) like check_ok_vslam, except that the backend calls the FRONTEND makes
 * are not replayed from the log: after every addStates the C frontend (ok_frontend.c) runs on the logged keypoints
 * (ADDSTATES), the logged BRISK descriptors (record 161) and the C backend / graph state, and every call it makes into the
 * backend (addLandmark, addObservation, removeObservation, setLandmark, mergeLandmark(s), setPose, optimiseRealtimeGraph,
 * cleanUnobservedLandmarks, MultiFrame::setLandmarkId) is compared, tag and argument bytes, with the next logged backend
 * entry record before it is executed (the graph calls it causes are then compared as in check_ok_vslam). The keyframe
 * decision is compared with the logged setKeyframe call. The OpenGV RANSAC runs are native (module M7c): the adapter data
 * the C frontend builds and the result of the native run are compared bit for bit with ransac.bin (patch 0013: adapter inputs
 * + result of every run; without that file with the result records 162 of problem.bin), and every logged run, including the
 * place-recognition GP3P runs of kind 3, is also re-run natively on the LOGGED adapter data. Until M7d is ported the place
 * recognition block is answered from the log (the logged attemptLoopClosure / addLoopClosureFrame calls are executed and the
 * loop-closure landmarks handed back).
 * Last line: "<seq_label>: <mismatches>/<total>".
 */
#define OK_VSLAM_AS_LIB
#include "check_ok_vslam.c"
#undef main
#include "ok_frontend.h"

static cnt C_fe, C_fe_args, C_fe_kf, C_fe_ransac, C_fe_ransac_in, C_pl_add, C_pl_query, C_pl_verify;
static long G_pl_ub_skipped;
static long G_fe_calls, G_fe_frames, G_fe_ransac, G_fe_ransac_in[4], G_fe_lc, G_pl_adds, G_pl_queries, G_pl_verifies, G_ub_subst;
static ok_fe* V_fe;
static int G_ub_flag;            /* the RANSAC result of the current verifyRecognisedPlace call was replaced (degenerate sample)  */
static int G_place_native;       /* place.bin present: the C frontend runs the DBoW2 / verifyRecognisedPlace block natively */
static uint64_t G_cur_frame;

/* ---- records captured by the aux hook ---- */
typedef struct drec { int ncam; int nkp[OK_FE_MAXCAM]; unsigned char* desc[OK_FE_MAXCAM]; int valid; } drec;
static drec G_desc;
typedef struct rrec { uint32_t kind, ncorr, iters, ninl; int* inl; uint32_t rows, cols; double model[16]; } rrec;
static rrec* G_rq; static int G_rqn, G_rqcap;

static void aux_hook(uint32_t tag, const unsigned char* p, size_t len) {
    cur c; c.p = p; c.off = 0; c.len = len; c.bad = 0;
    if (tag == OK_B_DESC) {
        uint32_t nc = cu32(&c), i;
        for (i = 0; i < OK_FE_MAXCAM; ++i) { free(G_desc.desc[i]); G_desc.desc[i] = NULL; }
        G_desc.ncam = (int)nc; G_desc.valid = 0;
        if (nc > OK_FE_MAXCAM) return;
        for (i = 0; i < nc; ++i) {
            uint32_t nk = cu32(&c);
            G_desc.nkp[i] = (int)nk;
            G_desc.desc[i] = (unsigned char*)malloc(48 * (size_t)(nk ? nk : 1));
            if (c.off + 48 * (size_t)nk <= c.len) memcpy(G_desc.desc[i], c.p + c.off, 48 * (size_t)nk); else c.bad = 1;
            c.off += 48 * (size_t)nk;
        }
        G_desc.valid = !c.bad;
    } else if (tag == OK_B_RANSAC) {
        rrec r; uint32_t i;
        memset(&r, 0, sizeof r);
        r.kind = cu32(&c); r.ncorr = cu32(&c); r.iters = cu32(&c); r.ninl = cu32(&c);
        r.inl = (int*)malloc(sizeof(int) * (size_t)(r.ninl ? r.ninl : 1));
        for (i = 0; i < r.ninl; ++i) r.inl[i] = (int)cu32(&c);
        r.rows = cu32(&c); r.cols = cu32(&c);
        if (r.rows * r.cols <= 16) cf64n(&c, r.model, r.rows * r.cols); else c.bad = 1;
        if (c.bad || r.kind == 3) { free(r.inl); return; }     /* kind 3 = verifyRecognisedPlace (place recognition, answered from the log) */
        if (G_rqn == G_rqcap) { G_rqcap = G_rqcap ? 2 * G_rqcap : 8; G_rq = (rrec*)realloc(G_rq, sizeof(rrec) * (size_t)G_rqcap); }
        G_rq[G_rqn++] = r;
    }
}

/* ---- the frontend's calls into the backend: compare with the next logged entry record, then execute ---- */
static int fe_expect(int tag, const obuf* mine) {
    lrec lr;
    G_fe_calls++;
    if (!get_rec(&lr)) { chk(&C_fe, 0); DBG("the frontend made backend call %d beyond the end of the log", tag); return 0; }
    if (lr.tag != (uint32_t)tag) {
        chk(&C_fe, 0);
        DBG("frontend made backend call %d, the log has tag %u", tag, lr.tag);
        unget_rec(&lr);
        return 0;
    }
    chk(&C_fe, 1);
    { const long b0 = C_fe_args.bad; cmp_bytes(&C_fe_args, tag, 0, mine->p, mine->n, lr.p, lr.len);
      if (C_fe_args.bad != b0) { DBG("  ^ frontend call %d in frame %llu (call #%ld)", tag, (unsigned long long)G_cur_frame, G_fe_calls); } }
    lrec_free(&lr);
    return 1;
}
static void ob_f64n(obuf* b, const double* v, size_t n) { ob_raw(b, v, 8 * n); }

static uint64_t e_add_landmark(void* ctx, const double hp[4], int init) {
    obuf o; uint64_t id; (void)ctx; memset(&o, 0, sizeof o);
    ob_f64n(&o, hp, 4); ob_u32(&o, init ? 1u : 0u);
    fe_expect(OK_B_ADDLM_NEW, &o); free(o.p);
    id = ok_vsb_add_landmark(V_b, hp, init);
    return id;
}
static int e_add_observation(void* ctx, uint64_t lm, uint64_t st, uint32_t cam, uint32_t kp, int cauchy) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, lm); ob_u64(&o, st); ob_u32(&o, cam); ob_u32(&o, kp); ob_u32(&o, cauchy ? 1u : 0u);
    fe_expect(OK_B_ADDOBS, &o); free(o.p);
    return ok_vsb_add_observation(V_b, lm, st, cam, kp, cauchy);
}
static int e_remove_observation(void* ctx, uint64_t st, uint32_t cam, uint32_t kp) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, st); ob_u32(&o, cam); ob_u32(&o, kp);
    fe_expect(OK_B_RMOBS, &o); free(o.p);
    return ok_vsb_remove_observation(V_b, st, cam, kp);
}
static int e_set_observation_information(void* ctx, uint64_t st, uint32_t cam, uint32_t kp, const double info[4]) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, st); ob_u32(&o, cam); ob_u32(&o, kp); ob_f64n(&o, info, 4);
    fe_expect(OK_B_SETOBSINFO, &o); free(o.p);
    return ok_vsb_set_observation_information(V_b, st, cam, kp, info);
}
static int e_set_landmark(void* ctx, uint64_t id, const double hp[4], int init) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, id); ob_f64n(&o, hp, 4); ob_u32(&o, init ? 1u : 0u);
    fe_expect(OK_B_SETLM, &o); free(o.p);
    return ok_vsb_set_landmark(V_b, id, hp, init);
}
static int e_merge_landmark(void* ctx, uint64_t from, uint64_t into) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, from); ob_u64(&o, into);
    fe_expect(OK_B_MERGELM, &o); free(o.p);
    return ok_vsb_merge_landmark(V_b, from, into);
}
static int e_merge_landmarks(void* ctx, const uint64_t* from, const uint64_t* into, int n) {
    obuf o; int i; (void)ctx; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)n); for (i = 0; i < n; ++i) ob_u64(&o, from[i]);
    ob_u32(&o, (uint32_t)n); for (i = 0; i < n; ++i) ob_u64(&o, into[i]);
    fe_expect(OK_B_MERGELMS, &o); free(o.p);
    return ok_vsb_merge_landmarks(V_b, from, into, n);
}
static int e_set_pose(void* ctx, uint64_t id, const double T7[7]) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, id); ob_f64n(&o, T7, 7);
    fe_expect(OK_B_SETPOSE, &o); free(o.p);
    return ok_vsb_set_pose(V_b, id, T7);
}
static int e_optimise_realtime(void* ctx, int ni, int nt, int vb, int on, int ii) {
    obuf o; uint64_t* up; int nu, r; (void)ctx; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)ni); ob_u32(&o, (uint32_t)nt); ob_u32(&o, (uint32_t)vb); ob_u32(&o, (uint32_t)on); ob_u32(&o, (uint32_t)ii);
    fe_expect(OK_B_OPTRT, &o); free(o.p);
    r = ok_vsb_optimise_realtime(V_b, ni, nt, vb, on, ii, &up, &nu);
    res_ids(OK_B_OPTRT, up, nu); free(up);
    return r;
}
static int e_clean(void* ctx) {
    obuf o; int n; (void)ctx;
    { obuf e; memset(&e, 0, sizeof e); fe_expect(OK_B_CLEANLM, &e); free(e.p); }
    n = ok_vsb_clean_unobserved_landmarks(V_b);
    memset(&o, 0, sizeof o); ob_u32(&o, (uint32_t)n); expect_result(OK_B_CLEANLM, &o); free(o.p);
    return n;
}
static void e_set_landmark_id(void* ctx, uint64_t fr, uint32_t cam, uint32_t kp, uint64_t id) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, fr); ob_u32(&o, cam); ob_u32(&o, kp); ob_u64(&o, id);
    fe_expect(OK_B_ML, &o); free(o.p);
    ok_vsb_set_landmark_id(V_b, fr, cam, kp, id);
}

/* ---- OpenGV RANSAC: ransac.bin (patch 0013) ---- */
typedef struct rrun { uint32_t kind; int rel; int n; double* d; rrec res; int have_res; } rrun;   /* d: abs 19 x n, rel 8 x n doubles */
static rrun* G_rr; static int G_rrn, G_rrcap, G_rr_next, G_have_rbin;
static long G_ub_runs;      /* logged runs whose native re-run met a degenerate GP3P sample and differs (undefined behaviour upstream) */

static void load_ransac_bin(const char* dir) {
    char path[1024]; FILE* f; unsigned char hdr[12];
    snprintf(path, sizeof path, "%s/ransac.bin", dir);
    f = fopen(path, "rb");
    if (!f) return;
    G_have_rbin = 1;
    while (fread(hdr, 1, 12, f) == 12) {
        uint32_t tag; uint64_t len; unsigned char* p; cur c;
        memcpy(&tag, hdr, 4); memcpy(&len, hdr + 4, 8);
        p = (unsigned char*)malloc((size_t)(len ? len : 1));
        if (fread(p, 1, (size_t)len, f) != len) { free(p); break; }
        c.p = p; c.off = 0; c.len = (size_t)len; c.bad = 0;
        if (tag == 163 || tag == 164) {
            rrun r; uint32_t kind = cu32(&c), n = cu32(&c); const size_t per = tag == 163 ? 19 : 8;
            memset(&r, 0, sizeof r);
            r.kind = kind; r.rel = tag == 164; r.n = (int)n;
            r.d = (double*)malloc(sizeof(double) * per * (size_t)(n ? n : 1));
            cf64n(&c, r.d, per * n);
            if (c.bad) { free(r.d); free(p); break; }
            if (G_rrn == G_rrcap) { G_rrcap = G_rrcap ? 2 * G_rrcap : 64; G_rr = (rrun*)realloc(G_rr, sizeof(rrun) * (size_t)G_rrcap); }
            G_rr[G_rrn++] = r;
        } else if (tag == 162 && G_rrn > 0) {
            rrun* r = &G_rr[G_rrn - 1]; uint32_t i;
            r->res.kind = cu32(&c); r->res.ncorr = cu32(&c); r->res.iters = cu32(&c); r->res.ninl = cu32(&c);
            r->res.inl = (int*)malloc(sizeof(int) * (size_t)(r->res.ninl ? r->res.ninl : 1));
            for (i = 0; i < r->res.ninl; ++i) r->res.inl[i] = (int)cu32(&c);
            r->res.rows = cu32(&c); r->res.cols = cu32(&c);
            if (r->res.rows * r->res.cols <= 16) cf64n(&c, r->res.model, r->res.rows * r->res.cols); else c.bad = 1;
            r->have_res = !c.bad;
        }
        free(p);
    }
    fclose(f);
}
static void cmp_f64(cnt* c, const double* a, const double* b, int n) {
    int i;
    for (i = 0; i < n; ++i) { uint64_t x, y; memcpy(&x, a + i, 8); memcpy(&y, b + i, 8); chk_u(c, "ransac adapter data", x, y); }
}
/* compare a native result with a logged one (iterations, inlier set, model shape and bytes) */
static void cmp_result(cnt* c, const char* what, const ok_og_result* r, const rrec* l) {
    uint32_t i;
    chk_u(c, what, (uint64_t)r->iterations, l->iters);
    chk_u(c, what, (uint64_t)r->ninliers, l->ninl);
    for (i = 0; i < l->ninl && i < (uint32_t)r->ninliers; ++i) chk_u(c, what, (uint64_t)(uint32_t)r->inliers[i], (uint64_t)(uint32_t)l->inl[i]);
    if (l->ninl) {                          /* the model is only defined when an inlier set exists (the C++ matrix is uninitialised otherwise) */
        chk_u(c, what, (uint64_t)r->rows, l->rows); chk_u(c, what, (uint64_t)r->cols, l->cols);
        for (i = 0; i < l->rows * l->cols && i < 16; ++i) { uint64_t a, b; memcpy(&a, &r->model[i], 8); memcpy(&b, &l->model[i], 8); chk_u(c, what, a, b); }
    }
}
/* the native RANSAC on the LOGGED adapter data of every run, kinds 0-3 */
static void standalone_ransac(void) {
    int i, k;
    for (i = 0; i < G_rrn; ++i) {
        rrun* r = &G_rr[i]; ok_og_result res; int ran;
        if (!r->have_res || r->kind > 3) { chk(&C_fe_ransac_in, 0); DBG("ransac.bin run %d incomplete", i); continue; }
        if (!r->rel) {
            double *bearing = (double*)malloc(sizeof(double) * 3 * (size_t)(r->n ? r->n : 1)), *point = (double*)malloc(sizeof(double) * 3 * (size_t)(r->n ? r->n : 1)),
                   *offset = (double*)malloc(sizeof(double) * 3 * (size_t)(r->n ? r->n : 1)), *rot = (double*)malloc(sizeof(double) * 9 * (size_t)(r->n ? r->n : 1)),
                   *sigma = (double*)malloc(sizeof(double) * (size_t)(r->n ? r->n : 1));
            ok_og_abs a;
            for (k = 0; k < r->n; ++k) {
                const double* d = r->d + 19 * k;
                memcpy(bearing + 3 * k, d, 24); memcpy(point + 3 * k, d + 3, 24); memcpy(offset + 3 * k, d + 6, 24); memcpy(rot + 9 * k, d + 9, 72); sigma[k] = d[18];
            }
            a.n = r->n; a.bearing = bearing; a.point = point; a.offset = offset; a.rot = rot; a.sigma = sigma;
            ran = ok_og_ransac_abs(&a, 16, 50, &res);
            free(bearing); free(point); free(offset); free(rot); free(sigma);
        } else {
            double *f1 = (double*)malloc(sizeof(double) * 3 * (size_t)(r->n ? r->n : 1)), *f2 = (double*)malloc(sizeof(double) * 3 * (size_t)(r->n ? r->n : 1)),
                   *s1 = (double*)malloc(sizeof(double) * (size_t)(r->n ? r->n : 1)), *s2 = (double*)malloc(sizeof(double) * (size_t)(r->n ? r->n : 1));
            ok_og_rel a;
            for (k = 0; k < r->n; ++k) { const double* d = r->d + 8 * k; memcpy(f1 + 3 * k, d, 24); memcpy(f2 + 3 * k, d + 3, 24); s1[k] = d[6]; s2[k] = d[7]; }
            a.n = r->n; a.f1 = f1; a.f2 = f2; a.s1 = s1; a.s2 = s2;
            ran = r->kind == 1 ? ok_og_ransac_rotation(&a, 9, 50, &res) : ok_og_ransac_stewenius(&a, 9, 50, &res);
            free(f1); free(f2); free(s1); free(s2);
        }
        chk_u(&C_fe_ransac_in, "ransac kind", r->kind, r->res.kind);
        chk_u(&C_fe_ransac_in, "ransac correspondences", (uint64_t)r->n, r->res.ncorr);
        { cnt t; int was_debug = G_debug;
          memset(&t, 0, sizeof t);
          G_debug = 0;
          cmp_result(&t, "standalone ransac", &res, &r->res);
          G_debug = was_debug;
          if (t.bad && res.degenerate) {            /* a GP3P sample with a duplicated 3D point: undefined behaviour upstream (see ok_og_result) */
              G_ub_runs++;
              DBG("  ^ ransac.bin run %d kind %u n %d: %d degenerate sample(s) (undefined behaviour in the C++), iterations %d / %u, inliers %d / %u", i, r->kind, r->n, res.degenerate, res.iterations, r->res.iters, res.ninliers, r->res.ninl);
              C_fe_ransac_in.tot += t.tot - t.bad;     /* the equal values still count as compared */
          } else {
              C_fe_ransac_in.tot += t.tot; C_fe_ransac_in.bad += t.bad;
              if (t.bad) DBG("  ^ ransac.bin run %d kind %u n %d (iterations %d / %u, inliers %d / %u, ret %d)", i, r->kind, r->n, res.iterations, r->res.iters, res.ninliers, r->res.ninl, ran);
          } }
        free(res.inliers);
        G_fe_ransac_in[r->kind]++;
    }
}

/* every native run of the frontend: compare the adapter data it built and the result with the log */
static void e_ransac_observe(void* ctx, int kind, const ok_og_abs* abs, const ok_og_rel* rel, ok_og_result* res, int ran) {
    (void)ctx; (void)ran;
    G_fe_ransac++;
    if (G_have_rbin) {
        rrun* r;
        int k;
        while (!G_place_native && G_rr_next < G_rrn && G_rr[G_rr_next].kind == 3) G_rr_next++;   /* kind 3 = verifyRecognisedPlace: M7d */
        if (G_rr_next >= G_rrn) { chk(&C_fe_ransac, 0); DBG("native RANSAC kind %d beyond the end of ransac.bin", kind); return; }
        r = &G_rr[G_rr_next++];
        chk_u(&C_fe_ransac, "ransac kind", (uint64_t)kind, r->kind);
        if (abs) {
            chk_u(&C_fe_ransac, "abs adapter size", (uint64_t)abs->n, (uint64_t)r->n);
            if (!r->rel && abs->n == r->n) {
                for (k = 0; k < abs->n; ++k) {
                    const double* d = r->d + 19 * k;
                    cmp_f64(&C_fe_ransac, abs->bearing + 3 * k, d, 3);
                    cmp_f64(&C_fe_ransac, abs->point + 3 * k, d + 3, 3);
                    cmp_f64(&C_fe_ransac, abs->offset + 3 * k, d + 6, 3);
                    cmp_f64(&C_fe_ransac, abs->rot + 9 * k, d + 9, 9);
                    cmp_f64(&C_fe_ransac, abs->sigma + k, d + 18, 1);
                }
            }
        } else if (rel) {
            chk_u(&C_fe_ransac, "rel adapter size", (uint64_t)rel->n, (uint64_t)r->n);
            if (r->rel && rel->n == r->n) {
                for (k = 0; k < rel->n; ++k) {
                    const double* d = r->d + 8 * k;
                    cmp_f64(&C_fe_ransac, rel->f1 + 3 * k, d, 3);
                    cmp_f64(&C_fe_ransac, rel->f2 + 3 * k, d + 3, 3);
                    cmp_f64(&C_fe_ransac, rel->s1 + k, d + 6, 1);
                    cmp_f64(&C_fe_ransac, rel->s2 + k, d + 7, 1);
                }
            }
        }
        if (r->have_res) {
            cnt t; int was_debug = G_debug;
            memset(&t, 0, sizeof t); G_debug = 0;
            cmp_result(&t, "ransac result", res, &r->res);
            G_debug = was_debug;
            if (t.bad && res->degenerate) {     /* undefined behaviour upstream (see ok_og_result): continue with the logged result */
                int i;
                C_fe_ransac.tot += t.tot - t.bad; G_ub_subst++; G_ub_flag = 1;
                free(res->inliers); res->inliers = (int*)malloc(sizeof(int) * (size_t)(r->res.ninl ? r->res.ninl : 1));
                for (i = 0; i < (int)r->res.ninl; ++i) res->inliers[i] = r->res.inl[i];
                res->ninliers = (int)r->res.ninl; res->iterations = (int)r->res.iters;
                memcpy(res->model, r->res.model, sizeof res->model);
            } else { C_fe_ransac.tot += t.tot; C_fe_ransac.bad += t.bad; if (t.bad) { cnt d; G_debug = was_debug; memset(&d, 0, sizeof d); cmp_result(&d, "ransac result", res, &r->res); } }
        }
    } else {                                   /* tags without ransac.bin: the result record 162 of problem.bin (read ahead) */
        rrec r;
        if (G_rqn == 0) { lrec lr; if (read_rec(&lr)) q_push_back(&lr); }
        if (G_rqn == 0) { chk(&C_fe_ransac, 0); DBG("RANSAC kind %d run, no logged result", kind); return; }
        r = G_rq[0]; memmove(G_rq, G_rq + 1, sizeof(rrec) * (size_t)(G_rqn - 1)); G_rqn--;
        chk_u(&C_fe_ransac, "ransac kind", (uint64_t)kind, r.kind);
        chk_u(&C_fe_ransac, "ransac correspondences", (uint64_t)(abs ? abs->n : rel->n), r.ncorr);
        cmp_result(&C_fe_ransac, "ransac result", res, &r);
        free(r.inl);
    }
}


/* ---- place.bin (patch 0014): vocabulary, database adds, queries, verifyRecognisedPlace stages ---- */
typedef struct plrec { uint32_t tag; unsigned char* p; size_t len; } plrec;
static unsigned char* G_voc; static size_t G_voc_len;
static plrec *G_pl_add, *G_pl_q, *G_pl_v; static int G_pl_addn, G_pl_qn, G_pl_vn, G_pl_addcap, G_pl_qcap, G_pl_vcap, G_pl_addi, G_pl_qi, G_pl_vi;

static void pl_push(plrec** a, int* n, int* cap, plrec r) {
    if (*n == *cap) { *cap = *cap ? 2 * *cap : 64; *a = (plrec*)realloc(*a, sizeof(plrec) * (size_t)*cap); }
    (*a)[(*n)++] = r;
}
static void load_place_bin(const char* dir) {
    char path[1024]; FILE* f; unsigned char hdr[12];
    snprintf(path, sizeof path, "%s/place.bin", dir);
    f = fopen(path, "rb");
    if (!f) return;
    while (fread(hdr, 1, 12, f) == 12) {
        uint32_t tag; uint64_t len; plrec r;
        memcpy(&tag, hdr, 4); memcpy(&len, hdr + 4, 8);
        r.tag = tag; r.len = (size_t)len; r.p = (unsigned char*)malloc((size_t)(len ? len : 1));
        if (fread(r.p, 1, (size_t)len, f) != len) { free(r.p); break; }
        if (tag == 170) { G_voc = r.p; G_voc_len = r.len; }
        else if (tag == 171) pl_push(&G_pl_add, &G_pl_addn, &G_pl_addcap, r);
        else if (tag == 172) pl_push(&G_pl_q, &G_pl_qn, &G_pl_qcap, r);
        else if (tag == 173) pl_push(&G_pl_v, &G_pl_vn, &G_pl_vcap, r);
        else free(r.p);
    }
    fclose(f);
    G_place_native = G_voc != NULL;
}
static void cmp_bits(cnt* c, const char* what, double a, double b) { uint64_t x, y; memcpy(&x, &a, 8); memcpy(&y, &b, 8); chk_u(c, what, x, y); }
static void cmp_bow(cnt* c, const char* what, const ok_dbow_bow* bow, cur* k) {
    uint32_t n = cu32(k), i;
    chk_u(c, what, (uint64_t)bow->n, n);
    for (i = 0; i < n && !k->bad; ++i) {
        const uint32_t id = cu32(k); const double w = cf64(k);
        if ((int)i < bow->n) { chk_u(c, what, bow->id[i], id); cmp_bits(c, what, bow->w[i], w); }
    }
}
static void e_on_db_add(void* ctx, int entry, uint64_t frame, int nfeat, const ok_dbow_bow* bow) {
    cur k; (void)ctx;
    G_pl_adds++;
    if (G_pl_addi >= G_pl_addn) { chk(&C_pl_add, 0); DBG("database add beyond the end of place.bin"); return; }
    k.p = G_pl_add[G_pl_addi].p; k.off = 0; k.len = G_pl_add[G_pl_addi].len; k.bad = 0; G_pl_addi++;
    chk_u(&C_pl_add, "db add entry id", (uint64_t)entry, cu32(&k));
    chk_u(&C_pl_add, "db add frame", frame, cu64(&k));
    chk_u(&C_pl_add, "db add features", (uint64_t)nfeat, cu32(&k));
    cmp_bow(&C_pl_add, "db add bow", bow, &k);
}
static void e_on_query(void* ctx, uint64_t frame, int nfeat, const ok_dbow_bow* bow, int db_size, const ok_dbow_result* orig, int norig,
                       const uint64_t* sids, const double* sc, int nst) {
    cur k; uint32_t n, i; (void)ctx;
    G_pl_queries++;
    if (G_pl_qi >= G_pl_qn) { chk(&C_pl_query, 0); DBG("query beyond the end of place.bin"); return; }
    k.p = G_pl_q[G_pl_qi].p; k.off = 0; k.len = G_pl_q[G_pl_qi].len; k.bad = 0; G_pl_qi++;
    chk_u(&C_pl_query, "query which", 0, cu32(&k));
    chk_u(&C_pl_query, "query frame", frame, cu64(&k));
    chk_u(&C_pl_query, "query features", (uint64_t)nfeat, cu32(&k));
    cmp_bow(&C_pl_query, "query bow", bow, &k);
    chk_u(&C_pl_query, "query db size", (uint64_t)db_size, cu32(&k));
    n = cu32(&k);
    chk_u(&C_pl_query, "query results", (uint64_t)norig, n);
    for (i = 0; i < n && !k.bad; ++i) {
        const uint32_t id = cu32(&k); const double s = cf64(&k);
        if ((int)i < norig) { chk_u(&C_pl_query, "query result id", orig[i].id, id); cmp_bits(&C_pl_query, "query result score", orig[i].score, s); }
    }
    n = cu32(&k);
    chk_u(&C_pl_query, "query retained", (uint64_t)nst, n);
    for (i = 0; i < n && !k.bad; ++i) {
        const uint64_t id = cu64(&k); const double s = cf64(&k);
        if ((int)i < nst) { chk_u(&C_pl_query, "query retained id", sids[i], id); cmp_bits(&C_pl_query, "query retained score", sc[i], s); }
    }
}
static void e_on_verify(void* ctx, const ok_fe_verify* v) {
    cur k; uint32_t n, i; int j; cnt ub_save; int ub_debug; (void)ctx;
    G_pl_verifies++;
    if (G_pl_vi >= G_pl_vn) { chk(&C_pl_verify, 0); DBG("verifyRecognisedPlace beyond the end of place.bin"); return; }
    ub_save = C_pl_verify; ub_debug = G_debug;
    if (G_ub_flag) G_debug = 0;               /* a record whose RANSAC result was replaced may legitimately differ (two builds, see below) */
    k.p = G_pl_v[G_pl_vi].p; k.off = 0; k.len = G_pl_v[G_pl_vi].len; k.bad = 0; G_pl_vi++;
    { const long b0 = C_pl_verify.bad;
    chk_u(&C_pl_verify, "verify frame", v->frame, cu64(&k));
    chk_u(&C_pl_verify, "verify old frame", v->old_frame, cu64(&k));
    chk_u(&C_pl_verify, "verify minInliers", (uint64_t)v->min_inliers, cu32(&k));
    chk_u(&C_pl_verify, "verify exit code", (uint64_t)v->code, cu32(&k));
    chk_u(&C_pl_verify, "verify ctr", (uint64_t)v->ctr, cu32(&k));
    chk_u(&C_pl_verify, "verify points", (uint64_t)v->npoints, cu32(&k));
    chk_u(&C_pl_verify, "verify correspondences", (uint64_t)v->ncorr, cu32(&k));

    chk_u(&C_pl_verify, "verify inliers", (uint64_t)v->ninl, cu32(&k));
    { const double avg = cf64(&k); if (v->code >= 3) { float a = (float)v->avg, b = (float)avg; uint32_t x, y; memcpy(&x, &a, 4); memcpy(&y, &b, 4); chk_u(&C_pl_verify, "verify avg", x, y); } }
    { const uint32_t have = cu32(&k); double t[7]; cf64n(&k, t, 7); chk_u(&C_pl_verify, "verify have T0", (uint64_t)v->have_T0, have);
      if (have && v->have_T0) for (j = 0; j < 7; ++j) cmp_bits(&C_pl_verify, "verify T0", v->T0[j], t[j]); }
    { const uint32_t have = cu32(&k); double t[7]; cf64n(&k, t, 7); chk_u(&C_pl_verify, "verify have T1", (uint64_t)v->have_T1, have);
      if (have && v->have_T1) for (j = 0; j < 7; ++j) cmp_bits(&C_pl_verify, "verify T1", v->T1[j], t[j]); }
    { const uint32_t have = cu32(&k); double h[36]; cf64n(&k, h, 36); chk_u(&C_pl_verify, "verify have H", (uint64_t)v->have_H, have);
      if (have && v->have_H) for (j = 0; j < 36; ++j) cmp_bits(&C_pl_verify, "verify H", v->H[j], h[j]); }
    chk_u(&C_pl_verify, "verify additional outliers", (uint64_t)v->add_out, cu32(&k));
    chk_u(&C_pl_verify, "verify final inliers", (uint64_t)v->nfinal, cu32(&k));
    chk_u(&C_pl_verify, "verify ceres iterations", (uint64_t)v->ceres_iters, cu32(&k));
    chk_u(&C_pl_verify, "verify ceres termination", (uint64_t)v->ceres_term, cu32(&k));
    { const double c0 = cf64(&k), c1 = cf64(&k); if (v->have_T1) { cmp_bits(&C_pl_verify, "verify initial cost", v->c0, c0); cmp_bits(&C_pl_verify, "verify final cost", v->c1, c1); } }
    n = cu32(&k);
    chk_u(&C_pl_verify, "verify landmarks", (uint64_t)v->nlandmarks, n);
    for (i = 0; i < n && !k.bad; ++i) {
        const uint64_t id = cu64(&k); double hp[4]; cf64n(&k, hp, 4);
        if ((int)i < v->nlandmarks) { chk_u(&C_pl_verify, "verify landmark id", v->lm_ids[i], id); for (j = 0; j < 4; ++j) cmp_bits(&C_pl_verify, "verify landmark", v->lm_hp[4 * i + j], hp[j]); }
    }
    n = cu32(&k);
    chk_u(&C_pl_verify, "verify matches", (uint64_t)v->nmatches, n);
    for (i = 0; i < n && !k.bad; ++i) {
        const uint64_t fr = cu64(&k); const uint32_t cm = cu32(&k), kp = cu32(&k); const uint64_t lm = cu64(&k);
        if ((int)i < v->nmatches) { chk_u(&C_pl_verify, "verify match frame", v->m_frame[i], fr); chk_u(&C_pl_verify, "verify match cam", v->m_cam[i], cm); chk_u(&C_pl_verify, "verify match kp", v->m_kp[i], kp); chk_u(&C_pl_verify, "verify match lm", v->m_lm[i], lm); }
    }
    if (C_pl_verify.bad != b0) DBG("  ^ verifyRecognisedPlace %llu -> %llu (exit code %d)", (unsigned long long)v->frame, (unsigned long long)v->old_frame, v->code); }
    G_debug = ub_debug;
    if (G_ub_flag) {
        /* place.bin and the replayed backend log come from two builds, and the C++ result of a run with a degenerate GP3P sample depends
         * on the build (uninitialised storage): the record is compared in full, and if it differs it is set aside, not counted */
        if (C_pl_verify.bad != ub_save.bad) { G_pl_ub_skipped++; C_pl_verify = ub_save; }
        G_ub_flag = 0;
    }
}

/* the frontend's attemptLoopClosure / addLoopClosureFrame: compare with the next logged entry record, then execute */
static int e_attempt_loop_closure(void* ctx, uint64_t pi, uint64_t pj, const double T[7], const double H[36], double drift, int* skip) {
    obuf o; int ret; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, pi); ob_u64(&o, pj); ob_f64n(&o, T, 7); ob_f64n(&o, H, 36); ob_f64n(&o, &drift, 1);
    fe_expect(OK_B_LCATTEMPT, &o); free(o.p);
    G_fe_lc++;
    ret = ok_vsb_attempt_loop_closure(V_b, pi, pj, T, H, drift, skip);
    G_lc_ret = ret;
    memset(&o, 0, sizeof o); ob_u32(&o, ret ? 1u : 0u); ob_u32(&o, *skip ? 1u : 0u);
    expect_result(OK_B_LCATTEMPT, &o); free(o.p);
    return ret;
}
static int e_add_loop_closure_frame(void* ctx, uint64_t id, int skip, uint64_t** lms, int* nl) {
    obuf o; (void)ctx; memset(&o, 0, sizeof o);
    ob_u64(&o, id); ob_u32(&o, skip ? 1u : 0u);
    fe_expect(OK_B_ADDLCFRAME, &o); free(o.p);
    ok_vsb_add_loop_closure_frame(V_b, id, skip, lms, nl);
    { uint64_t* copy = (uint64_t*)malloc(8 * (size_t)(*nl ? *nl : 1)); if (*nl) memcpy(copy, *lms, 8 * (size_t)*nl); res_ids(OK_B_ADDLCFRAME, *lms, *nl); free(*lms); *lms = copy; }
    return 1;
}

/* place recognition oracle: execute the logged attemptLoopClosure / addLoopClosureFrame calls */
static int e_place_recognition(void* ctx, uint64_t frame, uint64_t** lms, int* nl) {
    lrec r; (void)ctx; (void)frame;
    *lms = NULL; *nl = 0;
    while (peek_rec(&r) && r.tag == OK_B_LCATTEMPT) {
        lrec lr;
        get_rec(&lr);
        chk(&C_fe, 1); G_fe_lc++;
        G_lc_ret = 0;
        handle_b(&lr);
        lrec_free(&lr);
        if (G_lc_ret) {
            lrec nx;
            if (peek_rec(&nx) && nx.tag == OK_B_ADDLCFRAME) {
                get_rec(&nx);
                handle_b(&nx);
                lrec_free(&nx);
                *nl = G_lc_nl;
                *lms = (uint64_t*)malloc(8 * (size_t)(G_lc_nl ? G_lc_nl : 1));
                if (G_lc_nl) memcpy(*lms, G_lc_lms, 8 * (size_t)G_lc_nl);
                return 1;
            }
            chk(&C_fe, 0); DBG("attemptLoopClosure succeeded but no addLoopClosureFrame follows");
            return 0;
        }
    }
    return 0;
}

static int G_akf = -1;
static void on_setkf(uint64_t id, int flag) {
    (void)id;
    if (G_akf >= 0) { chk_u(&C_fe_kf, "keyframe decision", (uint64_t)G_akf, (uint64_t)flag); G_akf = -1; }
}

static int fmismatches(void);
static void run_frontend(void) {
    int i, akf = 0;
    uint64_t id = ok_vsb_current_state_id(V_b);
    const unsigned char* d[OK_FE_MAXCAM];
    if (!G_desc.valid) { chk(&C_fe, 0); DBG("no descriptor record for frame %llu", (unsigned long long)id); return; }
    for (i = 0; i < G_desc.ncam; ++i) d[i] = G_desc.desc[i];
    G_cur_frame = id;
    ok_fe_add_frame(V_fe, id, G_desc.ncam, G_desc.nkp, d);
    ok_fe_data_association(V_fe, id, &akf);
    { static int shown; if (!shown && fmismatches() > 0) { shown = 1; DBG("first mismatches while processing frame %llu (frontend run, cameras %d)", (unsigned long long)id, G_desc.ncam); } }
    G_akf = akf; G_fe_frames++;
}

static int fmismatches(void) {
    return vmismatches() + (int)(C_fe.bad + C_fe_args.bad + C_fe_kf.bad + C_fe_ransac.bad + C_fe_ransac_in.bad + C_pl_add.bad + C_pl_query.bad + C_pl_verify.bad);
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "frontend";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max_calls = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    ok_vsb_hooks h;
    ok_fe_params fp;
    ok_fe_est est;
    lrec r;
    long nb_records = 0;
    int ngraph = 0;
    G_debug = getenv("OK_DEBUG") != NULL;
    if (getenv("OK_NATIVE_SOLVE")) { G_native = 1; G_native_every = atol(getenv("OK_NATIVE_SOLVE")); if (G_native_every < 1) G_native_every = 1; }
    load_ransac_bin(dir);
    load_place_bin(dir);
    standalone_ransac();
    if (getenv("OK_RANSAC_ONLY")) {            /* quick mode: only the native re-run of the logged RANSAC runs */
        printf("  runs per kind: %ld %ld %ld %ld, degenerate-sample runs not counted: %ld\n", G_fe_ransac_in[0], G_fe_ransac_in[1], G_fe_ransac_in[2], G_fe_ransac_in[3], G_ub_runs);
        printf("%s: %ld/%ld\n", label, C_fe_ransac_in.bad, C_fe_ransac_in.tot);
        return C_fe_ransac_in.bad == 0 && C_fe_ransac_in.tot > 0 ? 0 : 1;
    }
    snprintf(path, sizeof path, "%s/problem.bin", dir);
    V_f = fopen(path, "rb");
    if (!V_f) { printf("%s: 0/0\n", label); fprintf(stderr, "cannot open %s\n", path); return 1; }
    G_aux_hook = aux_hook; G_setkf_hook = on_setkf;
    while (ngraph < 2 && read_rec(&r)) {
        if (r.tag == OK_M_NEW) {
            cur c; uint64_t gptr; gslot* s; pslot* ps;
            c.p = r.p; c.off = 0; c.len = r.len; c.bad = 0;
            gptr = cu64(&c);
            s = &G_g[G_ng++]; memset(s, 0, sizeof *s);
            s->gptr = gptr; s->problem = G_last_new_problem;
            ps = ps_find(s->problem, 0);
            chk(&C_events, ps && ps->ev.n == 1 && ps->ev.a[0].kind == OK_P_NEW);
            if (ps) ps->ev.n = 0;
            ngraph++;
        }
        lrec_free(&r);
    }
    memset(&h, 0, sizeof h);
    h.trace = on_trace; h.solve = on_solve;
    V_b = ok_vsb_new(&h);
    G_g[0].g = ok_vsb_graph(V_b, 0); G_g[1].g = ok_vsb_graph(V_b, 1);
    memset(&fp, 0, sizeof fp);
    fp.matching_threshold = 60.0; fp.keyframe_overlap = 0.60f; fp.num_matching_threads = 4; fp.imu_use = 1;
    fp.do_loop_closures = 1; fp.realtime_num_threads = 1;
    memset(&est, 0, sizeof est);
    est.add_landmark = e_add_landmark; est.add_observation = e_add_observation; est.remove_observation = e_remove_observation;
    est.set_observation_information = e_set_observation_information; est.set_landmark = e_set_landmark;
    est.merge_landmark = e_merge_landmark; est.merge_landmarks = e_merge_landmarks; est.set_pose = e_set_pose;
    est.optimise_realtime = e_optimise_realtime; est.clean_unobserved_landmarks = e_clean; est.set_landmark_id = e_set_landmark_id;
    est.ransac_observe = e_ransac_observe;
    if (G_place_native) {
        est.on_db_add = e_on_db_add; est.on_query = e_on_query; est.on_verify = e_on_verify;
        est.attempt_loop_closure = e_attempt_loop_closure; est.add_loop_closure_frame = e_add_loop_closure_frame;
        fp.p_dbow = 0.4; fp.drift_percentage = 1.35; fp.realtime_max_iterations = 10;
    }
    est.place_recognition = e_place_recognition;
    V_fe = ok_fe_new(V_b, &fp, &est);
    if (G_place_native && ok_fe_set_vocabulary(V_fe, G_voc, G_voc_len)) { fprintf(stderr, "cannot parse the vocabulary record\n"); return 1; }
    while (get_rec(&r)) {
        if (r.tag >= 128) {
            nb_records++;
            handle_b(&r);
            if (r.tag == OK_B_ADDSTATES) run_frontend();
            if (max_calls > 0 && nb_records > max_calls) { lrec_free(&r); break; }
        } else {
            chk(&C_struct, 0);
            DBG("logged graph record %u (graph %llx) that the C backend did not make", r.tag, r.len >= 8 ? (unsigned long long)*(uint64_t*)r.p : 0ull);
        }
        lrec_free(&r);
    }
    fclose(V_f);
#define PK2(name, c) printf("  %-24s %ld/%ld\n", name, (c).bad, (c).tot)
    printf("  frames run through the C frontend: %ld, frontend backend calls compared: %ld, native RANSAC runs of the frontend: %ld (ransac.bin: %s), logged runs re-run natively on the logged adapter data: kind 0 %ld, 1 %ld, 2 %ld, 3 %ld (of which %ld differ through a degenerate GP3P sample, undefined behaviour upstream, not counted), loop-closure attempts replayed from the log: %ld, other backend records replayed: %ld\n",
           G_fe_frames, G_fe_calls, G_fe_ransac, G_have_rbin ? "yes" : "no", G_fe_ransac_in[0], G_fe_ransac_in[1], G_fe_ransac_in[2], G_fe_ransac_in[3], G_ub_runs, G_fe_lc, nb_records);
    PK2("frontend calls (tag)", C_fe); PK2("frontend call arguments", C_fe_args); PK2("keyframe decision", C_fe_kf);
    PK2("ransac adapter + result", C_fe_ransac); PK2("ransac standalone (logged inputs)", C_fe_ransac_in);
    if (G_place_native) {
        printf("  native place recognition: %ld database adds, %ld queries, %ld verifyRecognisedPlace calls, %ld attemptLoopClosure calls, %ld RANSAC results replaced by the logged ones (degenerate GP3P sample, undefined behaviour upstream; %ld of their place.bin stage records differ because that file comes from another build and are not counted)\n",
               G_pl_adds, G_pl_queries, G_pl_verifies, G_fe_lc, G_ub_subst, G_pl_ub_skipped);
        PK2("place: database adds", C_pl_add); PK2("place: queries", C_pl_query); PK2("place: verifyRecognisedPlace", C_pl_verify);
    }
    PK2("record stream", C_trace); PK2("call arguments", C_args); PK2("call results", C_res); PK2("backend results", C_bres);
    if (G_native) { printf("  native solves: %ld of %ld optimise calls\n", G_nnative, G_nsolves); PK2("native solve result", C_native); }
    PK2("optimise: options", C_opt_args); PK2("problem events", C_events);
    PK2("optimise: states", C_opt_state); PK2("optimise: blocks", C_opt_blocks); PK2("optimise: IMU terms", C_opt_imu);
    PK2("program order", C_program); PK2("structure", C_struct);
    {
        long tot = C_fe.tot + C_fe_args.tot + C_fe_kf.tot + C_fe_ransac.tot + C_fe_ransac_in.tot + C_pl_add.tot + C_pl_query.tot + C_pl_verify.tot + C_trace.tot + C_args.tot + C_res.tot + C_bres.tot + C_opt_args.tot + C_native.tot
                   + C_events.tot + C_opt_state.tot + C_opt_blocks.tot + C_opt_imu.tot + C_program.tot + C_struct.tot;
        printf("%s: %d/%ld\n", label, fmismatches(), tot);
        ok_fe_free(V_fe);
        ok_vsb_free(V_b);
        return fmismatches() == 0 && tot > 0 ? 0 : 1;
    }
}
