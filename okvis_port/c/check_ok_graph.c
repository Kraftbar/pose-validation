/* OK_PORT_SOURCES: check_ok_graph.c ok_graph.c ok_twopose.c ok_err.c ok_param.c ok_cam.c ok_kin.c ok_eigen.c ok_dense.c ok_blas.c ok_time.c */
/* Bit-exactness harness for okvis_port module 5a/5b (TwoPose*GraphError terms, ViGraph::updateLandmarks).
 *
 *   check_ok_graph <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_records]
 *
 * Replays <dump_dir>/graph.bin (patch 0009, layout in ok_graph.h): every TwoPoseStandardGraphError::compute()
 * (the observations are re-added to a C term and compute() is rerun: H00_, b0_, J_, DeltaX_, the linearisation
 * point, the landmarks in S0 and the marginalised flags are compared), every convertToReprojectionErrors() (landmarks
 * mapped back to world), the sampled Evaluate records of all four TwoPose* classes (residuals, Jacobians, minimal
 * Jacobians, untouched buffers) and the sampled updateLandmarks() landmarks (quality, initialisation, reset point).
 * Prints per-kind lines "  <kind>: <mismatching values>/<compared values> (<failing records>/<records>)" and as the
 * LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0. OK_DEBUG=1 prints the first mismatches.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_graph.h"

typedef struct counts { long bad, tot, recs, badrecs; } counts;
static counts C_compute, C_convert, C_eval, C_lm;
static long G_struct_bad, G_skipped;
static int G_debug, G_printed;
static const char* G_kind = "";
static long G_rec;

typedef struct cur { const unsigned char* p; size_t off, len; int bad; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); else c->bad = 1; c->off += 4; return v; }
static uint64_t cu64(cur* c) { uint64_t v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static double cf64(cur* c) { double v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static void cf64n(cur* c, double* out, size_t n) { size_t i; for (i = 0; i < n; ++i) out[i] = cf64(c); }

static void mism(counts* c, const char* what, long idx, double got, double want) {
    c->bad++;
    if (G_debug && G_printed < 80) { G_printed++; printf("    MISMATCH %s rec %ld %s[%ld]: got %.17g want %.17g\n", G_kind, G_rec, what, idx, got, want); }
}
static int cmp_d(counts* c, const char* what, double got, double want) { c->tot++; if (memcmp(&got, &want, 8) != 0) { mism(c, what, 0, got, want); return 1; } return 0; }
static int cmp_i(counts* c, const char* what, long got, long want) { c->tot++; if (got != want) { mism(c, what, 0, (double)got, (double)want); return 1; } return 0; }
static int cmp_vec(counts* c, const char* what, const double* got, const double* want, long n) {
    long i; int bad = 0;
    for (i = 0; i < n; ++i) { c->tot++; if (memcmp(&got[i], &want[i], 8) != 0) { bad++; mism(c, what, i, got[i], want[i]); } }
    return bad;
}
static int cmp_t7(counts* c, const char* what, const ok_tf* T, const double w[7]) {
    double got[7] = {T->r[0], T->r[1], T->r[2], T->q.x, T->q.y, T->q.z, T->q.w};
    return cmp_vec(c, what, got, w, 7);
}
static void structural(const char* what) { G_struct_bad++; if (G_debug && G_printed < 80) { G_printed++; printf("    STRUCT %s rec %ld: %s\n", G_kind, G_rec, what); } }

/* reads a "reproj" payload (u64 plen + bytes) */
static int rd_reproj(cur* c, ok_reproj_err* e) {
    const uint64_t plen = cu64(c);
    int used;
    if (c->off + plen > c->len) { c->bad = 1; return 0; }
    used = ok_reproj_payload_read(c->p + c->off, (size_t)plen, e);
    c->off += (size_t)plen;
    return used > 0;
}

/* ---- G_TP_COMPUTE ---- */
static void do_compute(cur* c) {
    uint32_t kind, numCams, stayConst, computedBefore, nextr, nlm, ngroups, i, g, o;
    uint64_t ref, other;
    ok_twopose t;
    int bad = 0, k;
    int rec_pose_present[2]; uint64_t rec_pose_id[2]; double rec_pose_snap[2][7], rec_pose_live[2][7];
    int rec_extr_present[2 * OK_TP_MAXEXTR]; uint64_t rec_extr_id[2 * OK_TP_MAXEXTR]; int rec_extr_idx[2 * OK_TP_MAXEXTR]; double rec_extr_snap[2 * OK_TP_MAXEXTR][7];
    uint64_t* rec_lm_id; int* rec_lm_vec; int* rec_lm_idx; double (*rec_lm_snap)[4];
    int nobs_total = 0;
    G_kind = "compute";
    C_compute.recs++;
    cu64(c); kind = cu32(c); ref = cu64(c); other = cu64(c); numCams = cu32(c); stayConst = cu32(c); computedBefore = cu32(c);
    if (kind != 7 || numCams > OK_TP_MAXEXTR || computedBefore) { structural("compute header"); G_skipped++; return; }
    ok_twopose_init(&t, ref, other, (int)numCams, (int)stayConst);
    for (k = 0; k < 2; ++k) { rec_pose_present[k] = (int)cu32(c); rec_pose_id[k] = cu64(c); cf64n(c, rec_pose_snap[k], 7); cf64n(c, rec_pose_live[k], 7); }
    nextr = cu32(c);
    if (nextr > 2 * OK_TP_MAXEXTR) { structural("nextr"); ok_twopose_free(&t); return; }
    for (i = 0; i < nextr; ++i) { rec_extr_present[i] = (int)cu32(c); rec_extr_id[i] = cu64(c); rec_extr_idx[i] = (int)cu32(c); cf64n(c, rec_extr_snap[i], 7); }
    nlm = cu32(c);
    if (nlm > (1u << 20)) { structural("nlm"); ok_twopose_free(&t); return; }
    rec_lm_id = (uint64_t*)malloc(8 * (size_t)(nlm + 1)); rec_lm_vec = (int*)malloc(4 * (size_t)(nlm + 1)); rec_lm_idx = (int*)malloc(4 * (size_t)(nlm + 1)); rec_lm_snap = (double(*)[4])malloc(32 * (size_t)(nlm + 1));
    for (i = 0; i < nlm; ++i) { rec_lm_id[i] = cu64(c); rec_lm_vec[i] = (int)cu32(c); rec_lm_idx[i] = (int)cu32(c); cf64n(c, rec_lm_snap[i], 4); }
    ngroups = cu32(c);
    for (g = 0; g < ngroups && !c->bad; ++g) {
        const uint64_t lm_id = cu64(c);
        const uint32_t nobs = cu32(c);
        for (o = 0; o < nobs && !c->bad; ++o) {
            uint64_t frameId, pose_id, hpoint_id, extr_id; uint32_t cam, kp, loss, isMarg, isDup, hinit; double hpoint[4];
            ok_reproj_err err;
            const double *pose_params = NULL, *extr_params = NULL, *lm_params = NULL;
            uint32_t q;
            frameId = cu64(c); cam = cu32(c); kp = cu32(c); loss = cu32(c); isMarg = cu32(c); isDup = cu32(c);
            pose_id = cu64(c); hpoint_id = cu64(c); hinit = cu32(c); cf64n(c, hpoint, 4); extr_id = cu64(c);
            if (!rd_reproj(c, &err)) { structural("reproj payload"); break; }
            if (hpoint_id != lm_id) structural("obs landmark id");
            if (isMarg) structural("observation already marginalised");
            for (k = 0; k < 2; ++k) if (rec_pose_present[k] && rec_pose_id[k] == pose_id) pose_params = rec_pose_snap[k];
            {   /* the extrinsics infos are indexed by slot (offset + camera), not by id: all extrinsics blocks of a
                 * state share the state's id */
                const int pose_idx = (pose_id == ref) ? 0 : 1;
                const uint32_t slot = (stayConst ? 0u : (uint32_t)pose_idx * numCams) + cam;
                if (slot < nextr && rec_extr_present[slot] && rec_extr_id[slot] == extr_id) extr_params = rec_extr_snap[slot];
            }
            for (q = 0; q < nlm; ++q) if (rec_lm_id[q] == hpoint_id) lm_params = rec_lm_snap[q];
            if (!pose_params || !extr_params || !lm_params) { structural("observation refers to an unknown info"); continue; }
            if (loss > 1) structural("loss kind");
            if (!ok_twopose_add_observation(&t, frameId, (int)cam, (int)kp, &err, (int)loss, pose_id, pose_params, hpoint_id, lm_params, (int)hinit, extr_id, extr_params, (int)isDup))
                structural("addObservation failed");
            nobs_total++;
        }
    }
    if (c->bad) { structural("short compute record"); ok_twopose_free(&t); free(rec_lm_id); free(rec_lm_vec); free(rec_lm_idx); free(rec_lm_snap); return; }
    /* the live pose values (parameterBlock->estimate()) and a structural check of the rebuilt infos */
    for (k = 0; k < 2; ++k) {
        if (t.pose_present[k] != rec_pose_present[k] || (rec_pose_present[k] && t.pose_id[k] != rec_pose_id[k])) structural("pose info");
        memcpy(t.pose_live[k], rec_pose_live[k], sizeof(double) * 7);
    }
    if ((uint32_t)t.nextr != nextr) structural("extrinsics info count");
    for (i = 0; i < nextr && i < (uint32_t)t.nextr; ++i)
        if (t.extr_present[i] != rec_extr_present[i] || (rec_extr_present[i] && (t.extr_id[i] != rec_extr_id[i] || t.extr_idx[i] != rec_extr_idx[i]))) structural("extrinsics info");
    if ((uint32_t)t.nlm != nlm) structural("landmark info count");
    /* the vector index / sparse offset of a landmark info follow the global addObservation order, which the record
     * (grouped by landmark) does not keep; they are bookkeeping only (compute() marginalises per landmark), so they
     * are taken from the record and only the id set is checked */
    for (i = 0; i < nlm && i < (uint32_t)t.nlm; ++i) {
        if (t.lm_id[i] != rec_lm_id[i]) structural("landmark info");
        t.lm_vec_idx[i] = rec_lm_vec[i];
        t.lm_idx[i] = rec_lm_idx[i];
    }
    /* compute and compare */
    {
        uint32_t ret, relPoseSet, nlmS0;
        double H00[36], b0[6], J[36], DeltaX[6], lin[7];
        const int cret = ok_twopose_compute(&t);
        ret = cu32(c);
        cf64n(c, H00, 36); cf64n(c, b0, 6); cf64n(c, J, 36); cf64n(c, DeltaX, 6); cf64n(c, lin, 7);
        relPoseSet = cu32(c);
        bad += cmp_i(&C_compute, "ret", cret, ret);
        bad += cmp_vec(&C_compute, "H00", t.H00, H00, 36);
        bad += cmp_vec(&C_compute, "b0", t.b0, b0, 6);
        bad += cmp_vec(&C_compute, "J", t.term.J, J, 36);
        bad += cmp_vec(&C_compute, "DeltaX", t.term.DeltaX, DeltaX, 6);
        bad += cmp_t7(&C_compute, "lin_T_S0S1", &t.term.lin_T_S0S1, lin);
        bad += cmp_i(&C_compute, "relPoseSet", t.rel_pose_set, relPoseSet);
        nlmS0 = cu32(c);
        bad += cmp_i(&C_compute, "nlm_S0", t.nlm_S0, nlmS0);
        for (i = 0; i < nlmS0 && !c->bad; ++i) {
            const uint64_t id = cu64(c);
            double hp[4];
            cf64n(c, hp, 4);
            if ((int)i < t.nlm_S0) {
                bad += cmp_i(&C_compute, "lm_S0.id", (long)t.lm_S0[i].id, (long)id);
                bad += cmp_vec(&C_compute, "lm_S0", t.lm_S0[i].hp_S0, hp, 4);
            }
        }
        for (g = 0; g < (uint32_t)t.ngroups; ++g)
            for (o = 0; o < (uint32_t)t.groups[g].nobs; ++o) {
                const uint32_t m = cu32(c);
                bad += cmp_i(&C_compute, "isMarginalised", t.groups[g].obs[o].is_marginalised, m);
            }
        if (c->bad) structural("short compute outputs");
    }
    if (bad) C_compute.badrecs++;
    ok_twopose_free(&t);
    free(rec_lm_id); free(rec_lm_vec); free(rec_lm_idx); free(rec_lm_snap);
}

/* ---- G_TP_CONVERT ---- */
static void do_convert(cur* c) {
    uint32_t kind, nlm, i;
    double T7[7];
    ok_tf T_WS0;
    int bad = 0;
    G_kind = "convert";
    C_convert.recs++;
    cu64(c); kind = cu32(c);
    cf64n(c, T7, 7);
    if (kind != 7) { structural("convert kind"); return; }
    ok_tf_convert(&T_WS0, T7);
    nlm = cu32(c);
    for (i = 0; i < nlm && !c->bad; ++i) {
        double hp_S0[4], hp_W[4], got[4];
        cu64(c);
        cf64n(c, hp_S0, 4); cf64n(c, hp_W, 4);
        ok_tf_mul_v4(&T_WS0, hp_S0, got, 1); /* hp_W = T_WS0 * landmarks_.at(id) */
        bad += cmp_vec(&C_convert, "hp_W", got, hp_W, 4);
    }
    cu32(c); cu32(c);
    if (c->bad) structural("short convert record");
    if (bad) C_convert.badrecs++;
}

/* ---- G_TP_EVAL ---- */
static void do_eval(cur* c) {
    uint32_t kind, nb, have_jac, have_jacmin, ret, nres, k;
    uint64_t plen;
    ok_tp_std s; ok_tp_ext x;
    double params[2 + OK_TP_MAXEXTR][7];
    const double* pp[2 + OK_TP_MAXEXTR];
    uint32_t jn[2 + OK_TP_MAXEXTR], jmn[2 + OK_TP_MAXEXTR];
    double* jac[2 + OK_TP_MAXEXTR];
    double* jacmin[2 + OK_TP_MAXEXTR];
    double Jb[2 + OK_TP_MAXEXTR][(6 + 6 * OK_TP_MAXEXTR) * 7], Jmb[2 + OK_TP_MAXEXTR][(6 + 6 * OK_TP_MAXEXTR) * 6];
    double res[6 + 6 * OK_TP_MAXEXTR], want[(6 + 6 * OK_TP_MAXEXTR) * 7];
    int bad = 0, cret, used;
    const double SENT = -7.25e300;
    G_kind = "eval";
    C_eval.recs++;
    cu64(c); kind = cu32(c);
    plen = cu64(c);
    if (c->off + plen > c->len || kind < 7 || kind > 10) { structural("eval header"); return; }
    used = ok_tp_payload_read(c->p + c->off, (size_t)plen, (int)kind, &s, &x);
    c->off += (size_t)plen;
    if (used <= 0) { structural("eval payload"); return; }
    nb = cu32(c);
    if (nb > 2 + OK_TP_MAXEXTR || nb < 2) { structural("eval nb"); return; }
    for (k = 0; k < nb; ++k) { cf64n(c, params[k], 7); pp[k] = params[k]; }
    have_jac = cu32(c);
    for (k = 0; k < nb; ++k) jn[k] = cu32(c);
    have_jacmin = cu32(c);
    for (k = 0; k < nb; ++k) jmn[k] = cu32(c);
    ret = cu32(c); nres = cu32(c);
    if (nres > 6 + 6 * OK_TP_MAXEXTR) { structural("eval nres"); return; }
    for (k = 0; k < nb; ++k) {
        int i;
        for (i = 0; i < (int)nres * 7; ++i) Jb[k][i] = SENT;
        for (i = 0; i < (int)nres * 6; ++i) Jmb[k][i] = SENT;
        jac[k] = jn[k] ? Jb[k] : NULL;
        jacmin[k] = jmn[k] ? Jmb[k] : NULL;
    }
    if (kind == 7 || kind == 8) {
        if (kind == 8) s.is_computed = 1;
        cret = ok_tp_std_evaluate(&s, pp, res, have_jac ? jac : NULL, have_jacmin ? jacmin : NULL);
    } else {
        if (kind == 10) x.is_computed = 1;
        if ((uint32_t)x.n != nres || (uint32_t)x.nextr + 2 != nb) { structural("eval ext sizes"); return; }
        cret = ok_tp_ext_evaluate(&x, pp, res, have_jac ? jac : NULL, have_jacmin ? jacmin : NULL);
    }
    bad += cmp_i(&C_eval, "ret", cret, ret);
    cf64n(c, want, nres);
    bad += cmp_vec(&C_eval, "residuals", res, want, nres);
    for (k = 0; k < nb; ++k)
        if (have_jac && jn[k]) { cf64n(c, want, (size_t)nres * 7); bad += cmp_vec(&C_eval, "J", Jb[k], want, (long)nres * 7); }
    for (k = 0; k < nb; ++k) {
        if (have_jac && jn[k] && have_jacmin && jmn[k]) { cf64n(c, want, (size_t)nres * 6); bad += cmp_vec(&C_eval, "Jmin", Jmb[k], want, (long)nres * 6); }
        else if (have_jacmin && jmn[k]) { /* requested but never written by the C++ code: must stay untouched */
            int i, touched = 0;
            for (i = 0; i < (int)nres * 6; ++i) if (Jmb[k][i] != SENT) touched = 1;
            bad += cmp_i(&C_eval, "Jmin untouched", touched, 0);
        }
    }
    if (c->bad) structural("short eval record");
    if (bad) C_eval.badrecs++;
}

/* ---- G_LM_UPDATE ---- */
static void do_lm(cur* c) {
    uint32_t nlm_total, sub, nrec, r;
    G_kind = "lm";
    cu64(c); nlm_total = cu32(c); sub = cu32(c); nrec = cu32(c);
    (void)nlm_total; (void)sub;
    for (r = 0; r < nrec && !c->bad; ++r) {
        uint32_t nobs, o;
        double hp[4], quality, hp_after[4], cq;
        uint32_t init;
        int cinit, bad = 0;
        ok_lm_obs* obs;
        C_lm.recs++;
        cu64(c); cf64n(c, hp, 4); nobs = cu32(c);
        if (nobs > 100000) { structural("lm nobs"); return; }
        obs = (ok_lm_obs*)calloc(nobs ? nobs : 1, sizeof(ok_lm_obs));
        for (o = 0; o < nobs && !c->bad; ++o) {
            obs[o].frame_id = cu64(c); obs[o].cam = (int)cu32(c); obs[o].kp = (int)cu32(c);
            cf64n(c, obs[o].pose, 7); cf64n(c, obs[o].extr, 7);
            if (!rd_reproj(c, &obs[o].err)) { structural("lm reproj payload"); break; }
        }
        quality = cf64(c); init = cu32(c); cf64n(c, hp_after, 4);
        if (c->bad) { structural("short lm record"); free(obs); return; }
        ok_graph_update_landmark(hp, obs, (int)nobs, &cq, &cinit);
        bad += cmp_d(&C_lm, "quality", cq, quality);
        bad += cmp_i(&C_lm, "initialised", cinit, init);
        bad += cmp_vec(&C_lm, "hp_after", hp, hp_after, 4);
        if (bad) C_lm.badrecs++;
        free(obs);
    }
}

static void print_kind(const char* name, const counts* c) { printf("  %s: %ld/%ld (%ld/%ld records)\n", name, c->bad, c->tot, c->badrecs, c->recs); }

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "graph";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max_records = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    FILE* f;
    uint32_t tag; uint64_t len;
    unsigned char* buf = NULL; size_t cap = 0;
    long tot, bad;
    G_debug = getenv("OK_DEBUG") != NULL;
    snprintf(path, sizeof path, "%s/graph.bin", dir);
    f = fopen(path, "rb");
    if (!f) { printf("%s: 0/0\n", label); fprintf(stderr, "cannot open %s\n", path); return 1; }
    while (fread(&tag, 4, 1, f) == 1 && fread(&len, 8, 1, f) == 1) {
        cur c;
        if (len > (1ull << 32)) break;
        if (len > cap) { cap = (size_t)len * 2 + 1024; buf = (unsigned char*)realloc(buf, cap); }
        if (len && fread(buf, 1, (size_t)len, f) != (size_t)len) break;
        G_rec++;
        if (max_records > 0 && G_rec > max_records) break;
        c.p = buf; c.off = 0; c.len = (size_t)len; c.bad = 0;
        switch (tag) {
            case OK_G_TP_COMPUTE: do_compute(&c); break;
            case OK_G_TP_CONVERT: do_convert(&c); break;
            case OK_G_TP_EVAL: do_eval(&c); break;
            case OK_G_LM_UPDATE: do_lm(&c); break;
            default: G_struct_bad++; break;
        }
    }
    fclose(f);
    free(buf);
    print_kind("twopose_compute", &C_compute); print_kind("twopose_convert", &C_convert); print_kind("twopose_eval", &C_eval); print_kind("update_landmarks", &C_lm);
    printf("  structural errors %ld, skipped %ld\n", G_struct_bad, G_skipped);
    bad = C_compute.bad + C_convert.bad + C_eval.bad + C_lm.bad + G_struct_bad;
    tot = C_compute.tot + C_convert.tot + C_eval.tot + C_lm.tot + G_struct_bad;
    printf("%s: %ld/%ld\n", label, bad, tot);
    return bad == 0 && tot > 0 ? 0 : 1;
}
