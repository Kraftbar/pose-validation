/* OK_PORT_SOURCES: check_ok_vslam.c ok_vslam.c ok_vsb_geom.c ok_vsolve.c ok_solve.c ok_solve_linear.c ok_sparse.c ok_amd.c ok_vigraph.c ok_problem.c ok_graph.c ok_twopose.c ok_err.c ok_param.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_dense.c ok_blas.c */
/* Bit-exactness harness for okvis_port module 6 (ViSlamBackend: strategy, IMU-frame / keyframe / loop-closure frame sets,
 * pose-graph conversion, loop-closure alignment, synchronisation).
 *
 *   check_ok_vslam <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_backend_calls]
 *
 * Replays <dump_dir>/problem.bin (patches 0009, 0010 and 0011): the backend ENTRY records (tags 128.., the inputs the
 * frontend / ThreadedSlam drive: addStates with the multiframe keypoints, addLandmark, addObservation, applyStrategy,
 * optimiseRealtimeGraph, attemptLoopClosure, ..., and every MultiFrame::setLandmarkId) are executed on the C backend
 * (ok_vslam.c), which DECIDES which graph mutations to make. Every graph-level call the C backend makes is compared with
 * the next logged mutation record: tag, graph (realtime / full), argument bytes, result bytes (pointer fields masked),
 * and the Problem calls it caused (kind, identity of every block / residual block through a bijection, loss, order).
 * The program order at every Solve() is compared as in check_ok_vigraph; at every graph optimise() the C graph (states,
 * parameter blocks, IMU terms) must equal the logged pre-solve digest, then the logged solver output is applied.
 * The results of the backend calls (affected / updated state sets, loop-closure verdicts, ...) must equal the logged
 * result records. OK_DEBUG=1 prints the first mismatches. Last line: "<seq_label>: <mismatches>/<total>".
 */
#define main check_vigraph_unused_main
#include "check_ok_vigraph.c"
#undef main
#ifdef OK_VSLAM_AS_LIB                 /* included by check_ok_frontend.c: keep this harness' main out of the way */
#define main check_vslam_unused_main
#endif
#include "ok_vslam.h"

static FILE* V_f;
static unsigned char *V_buf; static size_t V_cap;
static cnt C_trace, C_args, C_res, C_bres, C_opt_args, C_native;
static int G_native; static long G_native_every = 1, G_nsolves, G_nnative;
static long G_calls; static long V_op[256];
static ok_vsb* V_b;

typedef struct lrec { uint32_t tag; size_t len; unsigned char* p; } lrec;
static void lrec_free(lrec* r) { free(r->p); r->p = NULL; r->len = 0; }

/* the Problem log records and the program-order check, as in check_ok_vigraph's main loop */
static void handle_problem_rec(const rec* r) {
    cur c; rev e; uint64_t pr;
    c.p = r->p; c.off = 0; c.len = (size_t)r->len; c.bad = 0;
    memset(&e, 0, sizeof e);
    e.kind = (int)r->tag;
    pr = cu64(&c);
    if (r->tag == OK_P_NEW) { pslot* s = ps_find(pr, 1); G_last_new_problem = pr; s->ev.n = 0; e.a = pr; revs_push(&s->ev, &e); return; }
    if (r->tag == OK_P_DELETE) { pslot* s = ps_find(pr, 0); if (s) s->alive = 0; return; }
    if (r->tag == OK_P_SOLVE) {
        gslot* gs = gs_by_problem(pr);
        if (gs && gs->g) {
            uint32_t np, nr, full, i; uint64_t hp, hr, sid;
            const ok_problem* pb = ok_vg_problem(gs->g);
            uint64_t *pp, *rr; int miss = 0;
            sid = cu64(&c); np = cu32(&c); nr = cu32(&c); hp = cu64(&c); hr = cu64(&c); full = cu32(&c); (void)sid;
            chk_u(&C_program, "np", (uint64_t)pb->np, np); chk_u(&C_program, "nr", (uint64_t)pb->nr, nr);
            pp = (uint64_t*)malloc(8 * (size_t)(pb->np + 1)); rr = (uint64_t*)malloc(8 * (size_t)(pb->nr + 1));
            { uint64_t *cp = (uint64_t*)malloc(8 * (size_t)(pb->np + 1)), *cr = (uint64_t*)malloc(8 * (size_t)(pb->nr + 1)); int k;
              ok_problem_program(pb, cp, cr);
              for (k = 0; k < pb->np; ++k) if (!u2u_get(&gs->p_c2r, cp[k], &pp[k])) { pp[k] = 0; miss++; }
              for (k = 0; k < pb->nr; ++k) if (!u2u_get(&gs->r_c2r, cr[k], &rr[k])) { rr[k] = 0; miss++; }
              free(cp); free(cr); }
            chk(&C_program, miss == 0);
            if (pb->np == (int)np) chk_u(&C_program, "param order hash", ok_vg_fnv(pp, 8 * (size_t)np), hp);
            if (pb->nr == (int)nr) chk_u(&C_program, "residual order hash", ok_vg_fnv(rr, 8 * (size_t)nr), hr);
            if (full && pb->np == (int)np && pb->nr == (int)nr) {
                for (i = 0; i < np; ++i) { uint64_t w = cu64(&c); chk_u(&C_program, "param order", pp[i], w); }
                for (i = 0; i < nr; ++i) { uint64_t w = cu64(&c); chk_u(&C_program, "residual order", rr[i], w); }
            }
            free(pp); free(rr);
        }
        return;
    }
    { pslot* s = ps_find(pr, 1);
      switch (r->tag) {
          case OK_P_ADDPARAM: e.a = cu64(&c); e.b = cu32(&c); break;
          case OK_P_SETMANIFOLD: e.a = cu64(&c); e.b = cu64(&c); break;
          case OK_P_ADDRESID: { uint32_t k; e.a = cu64(&c); e.cost = cu64(&c); e.loss = cu64(&c); e.nb = cu32(&c); if (e.nb > OK_PB_MAXB) { c.bad = 1; break; } for (k = 0; k < e.nb; ++k) e.v[k] = cu64(&c); break; }
          case OK_P_RMRESID: e.a = cu64(&c); break;
          case OK_P_RMPARAM: e.a = cu64(&c); e.ndeps = cu32(&c); break;
          case OK_P_SETCONST: case OK_P_SETVAR: e.a = cu64(&c); break;
          default: break;
      }
      revs_push(&s->ev, &e); }
}

/* the next record with tag >= 32 (Problem records are consumed on the way); look-ahead queue (unget / peek).
 * Records 161 (descriptors) and 162 (RANSAC) of patch 0012 are passed to G_aux_hook (check_ok_frontend) or skipped. */
static void (*G_aux_hook)(uint32_t tag, const unsigned char* p, size_t len);
static void (*G_setkf_hook)(uint64_t id, int flag);
/* called with every ADDSTATES before the C backend's addStates (check_ok_frontend: BRISK on the images may replace the
 * logged keypoints of cams); G_imu_cfg = the last ADDIMU configuration */
static void (*G_addstates_hook)(ok_time t, const ok_imu_meas* meas, size_t n, int ncam, ok_vsb_cam_in* cams);
static ok_vg_imu_cfg G_imu_cfg;
static lrec* G_q; static int G_qn, G_qcap;
static int read_rec(lrec* out) {
    rec r;
    for (;;) {
        if (!rd_rec(V_f, &r, &V_buf, &V_cap)) return 0;
        if (r.tag >= 1 && r.tag <= 10) { handle_problem_rec(&r); continue; }
        if (r.tag < 32) continue;
        if (r.tag == 161 || r.tag == 162) { if (G_aux_hook) G_aux_hook(r.tag, r.p, (size_t)r.len); continue; }
        out->tag = r.tag; out->len = (size_t)r.len;
        out->p = (unsigned char*)malloc(out->len ? out->len : 1);
        if (out->len) memcpy(out->p, r.p, out->len);
        return 1;
    }
}
static void q_push_front(const lrec* r) {
    if (G_qn == G_qcap) { G_qcap = G_qcap ? 2 * G_qcap : 8; G_q = (lrec*)realloc(G_q, sizeof(lrec) * (size_t)G_qcap); }
    memmove(G_q + 1, G_q, sizeof(lrec) * (size_t)G_qn); G_q[0] = *r; G_qn++;
}
static void q_push_back(const lrec* r) {
    if (G_qn == G_qcap) { G_qcap = G_qcap ? 2 * G_qcap : 8; G_q = (lrec*)realloc(G_q, sizeof(lrec) * (size_t)G_qcap); }
    G_q[G_qn++] = *r;
}
static int get_rec(lrec* out) {
    if (G_qn > 0) { *out = G_q[0]; memmove(G_q, G_q + 1, sizeof(lrec) * (size_t)(G_qn - 1)); G_qn--; return 1; }
    return read_rec(out);
}
static void unget_rec(const lrec* r) { q_push_front(r); }
/* make sure the queue holds at least one record (reads at most ONE record ahead: an entry record, which precedes the Problem
 * records of its own mutations, so the events of the C side and the log stay in step) */
#ifdef OK_VSLAM_AS_LIB
static int peek_rec(lrec* out) {
    if (G_qn == 0) { lrec r; if (!read_rec(&r)) return 0; q_push_back(&r); }
    *out = G_q[0];
    return 1;
}
#endif

/* zero the pointer fields of a logged record so that it can be compared with the C bytes (which carry zeros there) */
static void mask_ptrs(int op, int is_res, unsigned char* p, size_t n) {
    if (op == OK_M_ADDEXTOBS && !is_res && n >= 36) memset(p + 28, 0, 8);
    else if (op == OK_M_POKE_SYNCIMU && !is_res && n >= 8) memset(p, 0, 8);
    else if (op == OK_M_MST && is_res && n >= 4) {
        uint32_t ret; memcpy(&ret, p, 4);
        if (ret == 1) {
            size_t off = 4; uint32_t ne, nc, i;
            if (off + 4 > n) return;
            memcpy(&ne, p + off, 4); off += 4 + 16 * (size_t)ne;
            if (off + 4 > n) return;
            memcpy(&nc, p + off, 4); off += 4;
            for (i = 0; i < nc && off + 24 <= n; ++i) { memset(p + off + 16, 0, 8); off += 24; }
        }
    } else if (op == OK_M_CONVOBS && is_res && n >= 8) {
        uint32_t no, i; size_t off = 8;
        memcpy(&no, p + 4, 4);
        for (i = 0; i < no && off + 32 <= n; ++i) { memset(p + off + 16, 0, 8); off += 32; }
    }
}
static void cmp_bytes(cnt* c, int op, int is_res, const unsigned char* mine, size_t nm, unsigned char* theirs, size_t nt) {
    int eq;
    mask_ptrs(op, is_res, theirs, nt);
    eq = nm == nt && (nm == 0 || memcmp(mine, theirs, nm) == 0);
    chk(c, eq);
    if (!eq) {
        size_t j = 0;
        if (nm == nt) while (j < nm && mine[j] == theirs[j]) ++j;
        DBG("op %d %s differs: C %zu bytes, reference %zu, first difference at byte %zu", op, is_res ? "result" : "args", nm, nt, j);
        if (nm == nt && j + 8 <= nm && G_debug) {
            const size_t o = j & ~(size_t)7; double a = 0, bb2 = 0; uint64_t ua, ub;
            memcpy(&a, mine + o, 8); memcpy(&bb2, theirs + o, 8); memcpy(&ua, mine + o, 8); memcpy(&ub, theirs + o, 8);
            DBG("   u64 at %zu: C %llu ref %llu  (double %.17g vs %.17g)", o, (unsigned long long)ua, (unsigned long long)ub, a, bb2);
        }
    }
}

/* one graph-level call of the C backend: compare with the next logged record */
static void on_trace(void* ctx, int graph, int op, const void* a, size_t alen, const void* r, size_t rlen) {
    lrec lr; cur c; uint64_t gptr = 0; uint32_t logged_alen = 0;
    (void)ctx;
    G_calls++;
    if (op >= 0 && op < 256) V_op[op]++;
    if (!get_rec(&lr)) { chk(&C_trace, 0); DBG("C made a call (op %d) beyond the end of the log", op); return; }
    if (lr.tag != (uint32_t)op) {
        chk(&C_trace, 0);
        DBG("record mismatch: C made op %d (graph %d), the log has tag %u", op, graph, lr.tag);
        unget_rec(&lr);                       /* keep the logged record: the C code may catch up */
        return;
    }
    chk(&C_trace, 1);
    c.p = lr.p; c.off = 0; c.len = lr.len; c.bad = 0;
    if (op == OK_B_ML) {
        cmp_bytes(&C_args, op, 0, (const unsigned char*)a, alen, lr.p, lr.len);
        lrec_free(&lr);
        return;
    }
    gptr = cu64(&c); logged_alen = cu32(&c);
    if (graph < 0 || graph > 1 || gptr != G_g[graph].gptr) { chk(&C_trace, 0); DBG("op %d on graph %llx, C used graph %d", op, (unsigned long long)gptr, graph); }
    if (c.off + logged_alen > lr.len) { chk(&C_trace, 0); lrec_free(&lr); return; }
    if (op == OK_M_POKE_SYNCIMU && graph >= 0 && graph <= 1 && alen >= 16 && logged_alen >= 16) {                  /* the source graph of syncFrom */
        uint64_t src; memcpy(&src, lr.p + c.off, 8);
        chk_u(&C_args, "syncFrom source graph", src, G_g[1 - graph].gptr);
    }
    cmp_bytes(&C_args, op, 0, (const unsigned char*)a, alen, lr.p + c.off, logged_alen);
    cmp_bytes(&C_res, op, 1, (const unsigned char*)r, rlen, lr.p + c.off + logged_alen, lr.len - c.off - logged_alen);
    if (graph >= 0 && graph <= 1 && G_g[graph].g) {
        gslot* s = &G_g[graph];
        pslot* ps = ps_find(s->problem, 1);
        const ok_vg_event* cev = NULL; int ncev = 0;
        ok_vg_problem_events(s->g, &cev, &ncev);
        cmp_events(s, ps->ev.a, ps->ev.n, cev, ncev);
        ps->ev.n = 0;
        ok_vg_events_clear(s->g);
    }
    lrec_free(&lr);
}

/* ViGraph::optimise: compare the C graph with the logged pre-solve digest, hand back the logged solver output */
static int on_solve(void* ctx, int graph, ok_vg* g, int max_iter, unsigned char** res, size_t* rlen) {
    lrec lr; cur c, a, rs; uint64_t gptr; uint32_t alen, i;
    ok_vg_state_info* si; ok_vg_blkref* bl; ok_vg_imuref* il;
    int nsi, nbl, nil;
    uint32_t ns, nb, nimu;
    gslot* s = &G_g[graph];
    (void)ctx;
    G_calls++;
    if (!get_rec(&lr)) { chk(&C_trace, 0); return 0; }
    if (lr.tag != OK_M_OPT) { chk(&C_trace, 0); DBG("optimise expected, the log has tag %u", lr.tag); unget_rec(&lr); return 0; }
    chk(&C_trace, 1);
    c.p = lr.p; c.off = 0; c.len = lr.len; c.bad = 0;
    gptr = cu64(&c); alen = cu32(&c);
    chk_u(&C_trace, "optimise graph", gptr, s->gptr);
    a.p = c.p + c.off; a.off = 0; a.len = alen; a.bad = 0;
    rs.p = c.p + c.off + alen; rs.off = 0; rs.len = lr.len - c.off - alen; rs.bad = 0;
    chk_u(&C_opt_args, "max iterations", (uint64_t)max_iter, cu32(&a));
    chk_u(&C_opt_args, "linear solver type", (uint64_t)ok_vg_solver_type(g), cu32(&a));
    chk_d(&C_opt_args, "function tolerance", ok_vg_function_tolerance(g), cf64(&a));
    ns = cu32(&a);
    nsi = ok_vg_state_infos(g, &si);
    chk_u(&C_opt_state, "nstates", (uint64_t)nsi, ns);
    for (i = 0; i < ns && !a.bad; ++i) {
        uint64_t id = cu64(&a); uint32_t kf = cu32(&a); ok_time t = rd_time(&a); uint32_t no = cu32(&a), ntp = cu32(&a), ntpc = cu32(&a), nrel = cu32(&a);
        if ((int)i < nsi) {
            chk_u(&C_opt_state, "state id", si[i].id, id); chk_u(&C_opt_state, "isKeyframe", (uint64_t)si[i].is_kf, kf);
            chk_u(&C_opt_state, "ts sec", si[i].ts.sec, t.sec); chk_u(&C_opt_state, "ts nsec", si[i].ts.nsec, t.nsec);
            chk_u(&C_opt_state, "nobs", (uint64_t)si[i].nobs, no); chk_u(&C_opt_state, "ntp", (uint64_t)si[i].ntp, ntp);
            chk_u(&C_opt_state, "ntpc", (uint64_t)si[i].ntpc, ntpc); chk_u(&C_opt_state, "nrel", (uint64_t)si[i].nrel, nrel);
        }
    }
    free(si);
    nb = cu32(&a);
    nbl = ok_vg_blocks(g, &bl);
    chk_u(&C_opt_blocks, "nblocks", (uint64_t)nbl, nb);
    for (i = 0; i < nb && !a.bad; ++i) {
        uint64_t ptr = cu64(&a), hv; uint32_t size = cu32(&a), flags = cu32(&a);
        hv = cu64(&a);
        if (size == 4) { double q = cf64(&a); uint32_t cl = cu32(&a); uint64_t lid = cu64(&a);
            if ((int)i < nbl) { chk_d(&C_opt_blocks, "quality", bl[i].quality, q); chk_u(&C_opt_blocks, "classification", (uint64_t)(uint32_t)bl[i].classification, cl); chk_u(&C_opt_blocks, "landmark id", bl[i].lm_id, lid); } }
        if ((int)i >= nbl) continue;
        { int ok = 1; bind(s, &s->p_r2c, &s->p_c2r, ptr, (uint64_t)(uintptr_t)bl[i].b, "opt block", &ok); if (!ok) chk(&C_opt_blocks, 0); }
        chk_u(&C_opt_blocks, "block size", (uint64_t)bl[i].b->size, size);
        chk_u(&C_opt_blocks, "block flags", (uint64_t)(bl[i].b->fixed ? 1u : 0u) | (bl[i].b->initialised ? 2u : 0u), flags);
        chk_u(&C_opt_blocks, "block values (fnv)", ok_vg_fnv(bl[i].b->x, 8 * (size_t)bl[i].b->size), hv);
    }
    free(bl);
    nimu = cu32(&a);
    nil = ok_vg_imu_links(g, &il);
    chk_u(&C_opt_imu, "nimu", (uint64_t)nil, nimu);
    for (i = 0; i < nimu && !a.bad; ++i) {
        uint64_t id = cu64(&a), hv = cu64(&a);
        if ((int)i < nil) {
            unsigned char* snap; size_t n = ok_vg_imu_snapshot(il[i].e, 0, 1, &snap);
            chk_u(&C_opt_imu, "imu state id", il[i].state_id, id);
            chk_u(&C_opt_imu, "imu snapshot (fnv)", ok_vg_fnv(snap, n), hv);
            free(snap);
        }
    }
    free(il);
    { gslot* gs = s; pslot* ps = ps_find(gs->problem, 1);
      const ok_vg_event* cev = NULL; int ncev = 0;
      ok_vg_problem_events(gs->g, &cev, &ncev);
      cmp_events(gs, ps->ev.a, ps->ev.n, cev, ncev);
      ps->ev.n = 0; ok_vg_events_clear(gs->g); }
    if (a.bad) chk(&C_struct, 0);
    *rlen = rs.len;
    *res = (unsigned char*)malloc(rs.len ? rs.len : 1);
    if (rs.len) memcpy(*res, rs.p, rs.len);
    G_nsolves++;
    if (G_native && (G_nsolves - 1) % G_native_every == 0) {
        /* native mode: solve on the C graph instead of applying the logged output; the logged output is the reference */
        unsigned char* nres = NULL; size_t nlen = 0;
        const int ok = ok_vg_solve_native(g, max_iter, &nres, &nlen);
        G_nnative++;
        if (!ok) { chk(&C_native, 0); DBG("native solve could not build the problem"); }
        else {
            const int eq = nlen == rs.len && memcmp(nres, rs.p, nlen) == 0;
            chk(&C_native, eq);
            if (!eq) {
                size_t j = 0;
                if (nlen == rs.len) while (j < nlen && nres[j] == rs.p[j]) ++j;
                DBG("native solve %ld (graph %d, %d iterations) differs: C %zu bytes, logged %zu, first difference at byte %zu", G_nsolves, graph, max_iter, nlen, rs.len, j);
            }
            free(*res); *res = nres; *rlen = nlen;
        }
    }
    lrec_free(&lr);
    return 1;
}

/* the logged result record (tag | 0x100) of a backend call against the bytes the C backend returned */
typedef struct obuf { unsigned char* p; size_t n, cap; } obuf;
static void ob_raw(obuf* b, const void* d, size_t n) {
    if (b->n + n > b->cap) { b->cap = (b->n + n) * 2 + 64; b->p = (unsigned char*)realloc(b->p, b->cap); }
    memcpy(b->p + b->n, d, n); b->n += n;
}
static void ob_u32(obuf* b, uint32_t v) { ob_raw(b, &v, 4); }
static void ob_u64(obuf* b, uint64_t v) { ob_raw(b, &v, 8); }
static void expect_result(int tag, const obuf* mine) {
    lrec lr;
    if (!get_rec(&lr)) { chk(&C_bres, 0); DBG("result record %d missing", tag); return; }
    if (lr.tag != (uint32_t)(tag | OK_B_RESULT)) {
        chk(&C_bres, 0); DBG("result record %d expected, the log has tag %u", tag, lr.tag);
        unget_rec(&lr);
        return;
    }
    cmp_bytes(&C_bres, tag, 1, mine->p, mine->n, lr.p, lr.len);
    lrec_free(&lr);
}
static int G_lc_ret; static uint64_t* G_lc_lms; static int G_lc_nl;     /* last LCATTEMPT verdict / ADDLCFRAME landmarks (check_ok_frontend) */
static void res_ids(int tag, const uint64_t* ids, int n) {
    obuf o; int i; memset(&o, 0, sizeof o);
    ob_u32(&o, (uint32_t)n);
    for (i = 0; i < n; ++i) ob_u64(&o, ids[i]);
    expect_result(tag, &o);
    free(o.p);
}

/* ---------------------------------------------------------------- the backend entry records */
static void handle_b(const lrec* r) {
    cur a; a.p = r->p; a.off = 0; a.len = r->len; a.bad = 0;
    G_calls++;
    switch (r->tag) {
        case OK_B_ADDCAM: { int d = (int)cu32(&a); double sr = cf64(&a), sa = cf64(&a); ok_vsb_add_camera(V_b, d, sr, sa); break; }
        case OK_B_ADDIMU: {
            ok_vg_imu_cfg ic; memset(&ic, 0, sizeof ic);
            ic.use = (int)cu32(&a); cf64n(&a, ic.T_BS, 7);
            ic.a_max = cf64(&a); ic.g_max = cf64(&a); ic.sigma_g_c = cf64(&a); ic.sigma_bg = cf64(&a); ic.sigma_a_c = cf64(&a); ic.sigma_ba = cf64(&a);
            ic.sigma_gw_c = cf64(&a); ic.sigma_aw_c = cf64(&a); cf64n(&a, ic.g0, 3); cf64n(&a, ic.a0, 3); ic.g = cf64(&a);
            G_imu_cfg = ic;
            ok_vsb_add_imu(V_b, &ic);
            break;
        }
        case OK_B_ADDSTATES: {
            ok_time t = rd_time(&a); size_t n; ok_imu_meas* m = rd_meas(&a, &n);
            int as_kf = (int)cu32(&a); double kptr = cf64(&a);
            uint32_t nc = cu32(&a), c;
            ok_vsb_cam_in cams[OK_VSB_MAXCAM];
            uint32_t* nzk[OK_VSB_MAXCAM]; uint64_t* nzi[OK_VSB_MAXCAM]; float* kps[OK_VSB_MAXCAM]; unsigned char* hdr[OK_VSB_MAXCAM];
            memset(cams, 0, sizeof cams);
            if (nc > OK_VSB_MAXCAM) { chk(&C_struct, 0); free(m); return; }
            for (c = 0; c < nc; ++c) {
                uint32_t hl = cu32(&a), k, nkp, nz;
                hdr[c] = (unsigned char*)malloc(hl ? hl : 1);
                if (a.off + hl <= a.len) memcpy(hdr[c], a.p + a.off, hl); else a.bad = 1;
                a.off += hl;
                cams[c].header = hdr[c]; cams[c].hlen = hl;
                cf64n(&a, cams[c].T_SC, 7);
                cams[c].rows = (int)cu32(&a); cams[c].cols = (int)cu32(&a); nkp = cu32(&a); cams[c].nkp = (int)nkp;
                kps[c] = (float*)malloc(sizeof(float) * 3 * (size_t)(nkp ? nkp : 1));
                if (a.off + 12 * (size_t)nkp <= a.len) memcpy(kps[c], a.p + a.off, 12 * (size_t)nkp); else a.bad = 1;
                a.off += 12 * (size_t)nkp;
                cams[c].kp = kps[c];
                nz = cu32(&a);
                nzk[c] = (uint32_t*)malloc(sizeof(uint32_t) * (size_t)(nz ? nz : 1)); nzi[c] = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(nz ? nz : 1));
                for (k = 0; k < nz; ++k) { nzk[c][k] = cu32(&a); nzi[c][k] = cu64(&a); }
                cams[c].nz = (int)nz; cams[c].nz_kp = nzk[c]; cams[c].nz_id = nzi[c];
            }
            if (!a.bad && G_addstates_hook) G_addstates_hook(t, m, n, (int)nc, cams);
            if (!a.bad) ok_vsb_add_states(V_b, t, m, n, as_kf, kptr, (int)nc, cams);
            for (c = 0; c < nc; ++c) { free(hdr[c]); free(kps[c]); free(nzk[c]); free(nzi[c]); }
            free(m);
            break;
        }
        case OK_B_SETKF: { uint64_t id = cu64(&a); int fl = (int)cu32(&a); if (G_setkf_hook) G_setkf_hook(id, fl); ok_vsb_set_keyframe(V_b, id, fl); break; }
        case OK_B_ADDLM_ID: { uint64_t id = cu64(&a); double hp[4]; int in; cf64n(&a, hp, 4); in = (int)cu32(&a); ok_vsb_add_landmark_id(V_b, id, hp, in); break; }
        case OK_B_ADDLM_NEW: { double hp[4]; int in; cf64n(&a, hp, 4); in = (int)cu32(&a); ok_vsb_add_landmark(V_b, hp, in); break; }
        case OK_B_SETLM: { uint64_t id = cu64(&a); double hp[4]; cf64n(&a, hp, 4); ok_vsb_set_landmark(V_b, id, hp, (int)cu32(&a)); break; }
        case OK_B_SETCLASS: { uint64_t id = cu64(&a); ok_vsb_set_landmark_classification(V_b, id, (int)cu32(&a)); break; }
        case OK_B_ADDOBS: { uint64_t lm = cu64(&a), st = cu64(&a); uint32_t cam = cu32(&a), kp = cu32(&a); ok_vsb_add_observation(V_b, lm, st, cam, kp, (int)cu32(&a)); break; }
        case OK_B_RMOBS: { uint64_t st = cu64(&a); uint32_t cam = cu32(&a), kp = cu32(&a); ok_vsb_remove_observation(V_b, st, cam, kp); break; }
        case OK_B_SETOBSINFO: { uint64_t st = cu64(&a); uint32_t cam = cu32(&a), kp = cu32(&a); double info[4]; cf64n(&a, info, 4); ok_vsb_set_observation_information(V_b, st, cam, kp, info); break; }
        case OK_B_MERGELMS: {
            uint32_t n1 = cu32(&a), i, n2; uint64_t *from, *into;
            from = (uint64_t*)malloc(8 * (size_t)(n1 + 1));
            for (i = 0; i < n1; ++i) from[i] = cu64(&a);
            n2 = cu32(&a); into = (uint64_t*)malloc(8 * (size_t)(n2 + 1));
            for (i = 0; i < n2; ++i) into[i] = cu64(&a);
            if (n1 == n2 && !a.bad) ok_vsb_merge_landmarks(V_b, from, into, (int)n1);
            free(from); free(into);
            break;
        }
        case OK_B_MERGELM: { uint64_t f = cu64(&a), i = cu64(&a); ok_vsb_merge_landmark(V_b, f, i); break; }
        case OK_B_APPLYSTRATEGY: {
            uint64_t nk = cu64(&a), nl = cu64(&a), ni = cu64(&a); int ex = (int)cu32(&a); uint64_t* aff; int na;
            ok_vsb_apply_strategy(V_b, (size_t)nk, (size_t)nl, (size_t)ni, ex, &aff, &na);
            res_ids(OK_B_APPLYSTRATEGY, aff, na); free(aff);
            break;
        }
        case OK_B_OPTRT: {
            int ni = (int)cu32(&a), nt = (int)cu32(&a), vb = (int)cu32(&a), on = (int)cu32(&a), ii = (int)cu32(&a); uint64_t* up; int nu;
            ok_vsb_optimise_realtime(V_b, ni, nt, vb, on, ii, &up, &nu);
            res_ids(OK_B_OPTRT, up, nu); free(up);
            break;
        }
        case OK_B_OPTFULL: { int ni = (int)cu32(&a), nt = (int)cu32(&a), vb = (int)cu32(&a); ok_vsb_optimise_full(V_b, ni, nt, vb); break; }
        case OK_B_SYNC: { uint64_t* up; int nu; ok_vsb_synchronise(V_b, &up, &nu); res_ids(OK_B_SYNC, up, nu); free(up); break; }
        case OK_B_CLEANLM: { int n = ok_vsb_clean_unobserved_landmarks(V_b); obuf o; memset(&o, 0, sizeof o); ob_u32(&o, (uint32_t)n); expect_result(OK_B_CLEANLM, &o); free(o.p); break; }
        case OK_B_LCATTEMPT: {
            uint64_t pi = cu64(&a), pj = cu64(&a); double T[7], info[36], drift; int skip = 0, ret; obuf o;
            cf64n(&a, T, 7); cf64n(&a, info, 36); drift = cf64(&a);
            ret = ok_vsb_attempt_loop_closure(V_b, pi, pj, T, info, drift, &skip);
            G_lc_ret = ret;
            memset(&o, 0, sizeof o); ob_u32(&o, ret ? 1u : 0u); ob_u32(&o, skip ? 1u : 0u);
            expect_result(OK_B_LCATTEMPT, &o); free(o.p);
            break;
        }
        case OK_B_ADDLCFRAME: {
            uint64_t id = cu64(&a); int skip = (int)cu32(&a); uint64_t* lm; int nl;
            ok_vsb_add_loop_closure_frame(V_b, id, skip, &lm, &nl);
            res_ids(OK_B_ADDLCFRAME, lm, nl);
            free(G_lc_lms); G_lc_lms = lm; G_lc_nl = nl;
            break;
        }
        case OK_B_SETPOSE: { uint64_t id = cu64(&a); double T[7]; cf64n(&a, T, 7); ok_vsb_set_pose(V_b, id, T); break; }
        case OK_B_SETSB: { uint64_t id = cu64(&a); double sb[9]; cf64n(&a, sb, 9); ok_vsb_set_speed_and_bias(V_b, id, sb); break; }
        case OK_B_SETEXTR: { uint64_t id = cu64(&a); int cam = (int)cu32(&a); double T[7]; cf64n(&a, T, 7); ok_vsb_set_extrinsics(V_b, id, cam, T); break; }
        case OK_B_CLEAR: chk(&C_struct, 0); DBG("estimator.clear() is not ported"); break;
        case OK_B_FINALBA: chk(&C_struct, 0); DBG("doFinalBa is not exercised/ported here"); break;
        case OK_B_ML: { uint64_t fr = cu64(&a); uint32_t cam = cu32(&a), kp = cu32(&a); uint64_t id = cu64(&a); ok_vsb_set_landmark_id(V_b, fr, cam, kp, id); G_calls--; break; }
        default: chk(&C_struct, 0); DBG("unknown backend record %u", r->tag); break;
    }
    if (a.bad) { chk(&C_struct, 0); DBG("short backend record %u", r->tag); }
}

static int vmismatches(void) {
    return mismatches() + (int)(C_trace.bad + C_args.bad + C_res.bad + C_bres.bad + C_opt_args.bad + C_native.bad);
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "vslam";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max_calls = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    ok_vsb_hooks h;
    lrec r;
    long nb_records = 0, nb_ml = 0;
    int ngraph = 0;
    G_debug = getenv("OK_DEBUG") != NULL;
    if (getenv("OK_NATIVE_SOLVE")) { G_native = 1; G_native_every = atol(getenv("OK_NATIVE_SOLVE")); if (G_native_every < 1) G_native_every = 1; }
    snprintf(path, sizeof path, "%s/problem.bin", dir);
    V_f = fopen(path, "rb");
    if (!V_f) { printf("%s: 0/0\n", label); fprintf(stderr, "cannot open %s\n", path); return 1; }
    /* the two graph constructors come first (realtime, then full) */
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
    while (get_rec(&r)) {
        if (r.tag >= 128) {
            if (r.tag == OK_B_ML) nb_ml++;
            nb_records++;
            handle_b(&r);
            if (max_calls > 0 && nb_records > max_calls) { lrec_free(&r); break; }
        } else {
            chk(&C_struct, 0);
            DBG("logged graph record %u (graph %llx) that the C backend did not make", r.tag, r.len >= 8 ? (unsigned long long)*(uint64_t*)r.p : 0ull);
        }
        lrec_free(&r);
    }
    fclose(V_f);
#define PK2(name, c) printf("  %-24s %ld/%ld\n", name, (c).bad, (c).tot)
    { int k; printf("  graph calls per op:"); for (k = 0; k < 256; ++k) if (V_op[k]) printf(" %d:%ld", k, V_op[k]); printf("\n"); }
    printf("  backend records replayed: %ld (%ld MultiFrame::setLandmarkId), graph calls made by the C backend: %ld\n", nb_records, nb_ml, G_calls);
    PK2("record stream", C_trace); PK2("call arguments", C_args); PK2("call results", C_res); PK2("backend results", C_bres);
    if (G_native) { printf("  native solves: %ld of %ld optimise calls\n", G_nnative, G_nsolves); PK2("native solve result", C_native); }
    PK2("optimise: options", C_opt_args); PK2("problem events", C_events);
    PK2("optimise: states", C_opt_state); PK2("optimise: blocks", C_opt_blocks); PK2("optimise: IMU terms", C_opt_imu);
    PK2("program order", C_program); PK2("structure", C_struct);
    {
        long tot = C_trace.tot + C_args.tot + C_res.tot + C_bres.tot + C_opt_args.tot + C_native.tot + C_events.tot + C_opt_state.tot + C_opt_blocks.tot + C_opt_imu.tot + C_program.tot + C_struct.tot;
        printf("%s: %d/%ld\n", label, vmismatches(), tot);
        ok_vsb_free(V_b);
        return vmismatches() == 0 && tot > 0 ? 0 : 1;
    }
}
