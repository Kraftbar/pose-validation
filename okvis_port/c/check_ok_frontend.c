/* OK_PORT_SOURCES: check_ok_frontend.c ok_frontend.c ok_vslam.c ok_vsb_geom.c ok_vsolve.c ok_solve.c ok_solve_linear.c ok_sparse.c ok_amd.c ok_vigraph.c ok_problem.c ok_graph.c ok_twopose.c ok_err.c ok_param.c ok_triangulate.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_dense.c ok_blas.c */
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
 * decision is compared with the logged setKeyframe call. Until M7c / M7d are ported two things are answered from the
 * log: the OpenGV RANSAC runs (record 162: kind, correspondence count, inliers, model) and the place recognition block
 * (the logged attemptLoopClosure / addLoopClosureFrame calls are executed and the loop-closure landmarks handed back).
 * Last line: "<seq_label>: <mismatches>/<total>".
 */
#define OK_VSLAM_AS_LIB
#include "check_ok_vslam.c"
#undef main
#include "ok_frontend.h"

static cnt C_fe, C_fe_args, C_fe_kf, C_fe_ransac;
static long G_fe_calls, G_fe_frames, G_fe_ransac, G_fe_lc;
static ok_fe* V_fe;
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

/* RANSAC oracle: record 162 (written right after the computeModel of the C++ run) */
static int e_ransac(void* ctx, int kind, int ncorr, ok_fe_ransac* out) {
    rrec r; (void)ctx;
    if (G_rqn == 0) { lrec lr; if (read_rec(&lr)) q_push_back(&lr); }     /* reads the 162 record (and one entry record after it) */
    if (G_rqn == 0) { chk(&C_fe_ransac, 0); DBG("RANSAC kind %d requested, no logged run", kind); return 0; }
    r = G_rq[0]; memmove(G_rq, G_rq + 1, sizeof(rrec) * (size_t)(G_rqn - 1)); G_rqn--;
    G_fe_ransac++;
    chk_u(&C_fe_ransac, "ransac kind", (uint64_t)kind, r.kind);
    chk_u(&C_fe_ransac, "ransac correspondences", (uint64_t)ncorr, r.ncorr);
    out->iterations = (int)r.iters; out->ninliers = (int)r.ninl; out->inliers = r.inl; out->rows = (int)r.rows; out->cols = (int)r.cols;
    memcpy(out->model, r.model, sizeof r.model);
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
    return vmismatches() + (int)(C_fe.bad + C_fe_args.bad + C_fe_kf.bad + C_fe_ransac.bad);
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
    est.ransac = e_ransac; est.place_recognition = e_place_recognition;
    V_fe = ok_fe_new(V_b, &fp, &est);
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
    printf("  frames run through the C frontend: %ld, frontend backend calls compared: %ld, RANSAC runs answered from the log: %ld, loop-closure attempts replayed from the log: %ld, other backend records replayed: %ld\n",
           G_fe_frames, G_fe_calls, G_fe_ransac, G_fe_lc, nb_records);
    PK2("frontend calls (tag)", C_fe); PK2("frontend call arguments", C_fe_args); PK2("keyframe decision", C_fe_kf); PK2("ransac log vs C list", C_fe_ransac);
    PK2("record stream", C_trace); PK2("call arguments", C_args); PK2("call results", C_res); PK2("backend results", C_bres);
    if (G_native) { printf("  native solves: %ld of %ld optimise calls\n", G_nnative, G_nsolves); PK2("native solve result", C_native); }
    PK2("optimise: options", C_opt_args); PK2("problem events", C_events);
    PK2("optimise: states", C_opt_state); PK2("optimise: blocks", C_opt_blocks); PK2("optimise: IMU terms", C_opt_imu);
    PK2("program order", C_program); PK2("structure", C_struct);
    {
        long tot = C_fe.tot + C_fe_args.tot + C_fe_kf.tot + C_fe_ransac.tot + C_trace.tot + C_args.tot + C_res.tot + C_bres.tot + C_opt_args.tot + C_native.tot
                   + C_events.tot + C_opt_state.tot + C_opt_blocks.tot + C_opt_imu.tot + C_program.tot + C_struct.tot;
        printf("%s: %d/%ld\n", label, fmismatches(), tot);
        ok_fe_free(V_fe);
        ok_vsb_free(V_b);
        return fmismatches() == 0 && tot > 0 ? 0 : 1;
    }
}
