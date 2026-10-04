/* OK_PORT_SOURCES: check_ok_imu.c ok_imu.c ok_eigen.c ok_time.c */
/* Bit-exactness harness for okvis_port module 1 (IMU propagation / preintegration).
 *
 *   check_ok_imu <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_records_per_kind]
 *
 * Replays every record of <dump_dir>/imu_{prop,preint,append,eval}.bin (layouts in ok_imu.h) through the
 * C module and compares all outputs bitwise (memcmp, so signed zeros count). Prints one line per kind
 *   "  <kind>: <mismatching values>/<compared values> (<failing records>/<records> records)"
 * and as the LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0.
 * (Dumps come from tools/run_okvis_reference.py --dump; EuRoC-derived, never committed.)
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_imu.h"

static FILE* G_f;
static long G_cat[5]; /* eval mismatches per output: J0 J1 J2 J3 residual */
static int G_eof;

static void rd(void* p, size_t n) {
    if (n && fread(p, 1, n, G_f) != n) G_eof = 1;
}
static uint32_t rd_u32(void) { uint32_t v = 0; rd(&v, 4); return v; }
static uint64_t rd_u64(void) { uint64_t v = 0; rd(&v, 8); return v; }
static double rd_f64(void) { double v = 0; rd(&v, 8); return v; }
static void rd_f64n(double* p, size_t n) { rd(p, 8 * n); }
static ok_time rd_time(void) { ok_time t; t.sec = rd_u32(); t.nsec = rd_u32(); return t; }
static ok_imu_meas* rd_meas(size_t* n) {
    size_t i;
    ok_imu_meas* m;
    *n = (size_t)rd_u64();
    if (G_eof || *n > (1u << 24)) { G_eof = 1; *n = 0; return NULL; }
    m = (ok_imu_meas*)malloc((*n ? *n : 1) * sizeof(ok_imu_meas));
    for (i = 0; i < *n; ++i) {
        m[i].t = rd_time();
        rd_f64n(m[i].gyr, 3);
        rd_f64n(m[i].acc, 3);
    }
    return m;
}
static ok_imu_params rd_params(void) {
    ok_imu_params p;
    p.sigma_g_c = rd_f64(); p.sigma_a_c = rd_f64(); p.sigma_gw_c = rd_f64(); p.sigma_aw_c = rd_f64();
    p.g = rd_f64(); p.g_max = rd_f64(); p.a_max = rd_f64();
    return p;
}

typedef struct counts { long bad, tot, recs, badrecs; } counts;
static int cmpv(counts* c, const double* a, const double* b, size_t n) {
    size_t i;
    int bad = 0;
    for (i = 0; i < n; ++i) {
        c->tot++;
        if (memcmp(&a[i], &b[i], 8) != 0) { c->bad++; bad++; }
    }
    return bad;
}
static int cmpi(counts* c, int64_t a, int64_t b) {
    c->tot++;
    if (a != b) { c->bad++; return 1; }
    return 0;
}

/* a full snapshot as written by ImuError::portSnapshot */
typedef struct snap {
    ok_imu_meas* meas; size_t n_meas; int has_meas;
    ok_imu_params params; ok_time t0, t1;
    double delta_q[4], C_integral[9], C_doubleintegral[9], acc_integral[3], acc_doubleintegral[3], cross[9];
    double dalpha_db_g[9], dv_db_g[9], dp_db_g[9], P_delta[225], sb_ref[9];
    uint32_t redo, redo_counter;
    double information[225], sqrt_information[225];
    uint32_t n_dps; double dPdsigma[4][225];
} snap;

static void rd_snap(snap* s, int with_meas) {
    uint32_t j;
    memset(s, 0, sizeof *s);
    if (with_meas) { s->meas = rd_meas(&s->n_meas); s->has_meas = 1; }
    else { s->n_meas = (size_t)rd_u64(); s->meas = NULL; }
    s->params = rd_params();
    s->t0 = rd_time(); s->t1 = rd_time();
    rd_f64n(s->delta_q, 4); rd_f64n(s->C_integral, 9); rd_f64n(s->C_doubleintegral, 9);
    rd_f64n(s->acc_integral, 3); rd_f64n(s->acc_doubleintegral, 3); rd_f64n(s->cross, 9);
    rd_f64n(s->dalpha_db_g, 9); rd_f64n(s->dv_db_g, 9); rd_f64n(s->dp_db_g, 9);
    rd_f64n(s->P_delta, 225); rd_f64n(s->sb_ref, 9);
    s->redo = rd_u32(); s->redo_counter = rd_u32();
    rd_f64n(s->information, 225); rd_f64n(s->sqrt_information, 225);
    s->n_dps = rd_u32();
    if (s->n_dps > 4) { G_eof = 1; s->n_dps = 0; }
    for (j = 0; j < s->n_dps; ++j) rd_f64n(s->dPdsigma[j], 225);
}

static void load_error_from_snap(ok_imu_error* e, const snap* s) {
    uint32_t j;
    ok_imu_error_init(e, s->meas, s->n_meas, &s->params, s->t0, s->t1);
    e->delta_q.x = s->delta_q[0]; e->delta_q.y = s->delta_q[1]; e->delta_q.z = s->delta_q[2]; e->delta_q.w = s->delta_q[3];
    memcpy(e->C_integral, s->C_integral, sizeof e->C_integral);
    memcpy(e->C_doubleintegral, s->C_doubleintegral, sizeof e->C_doubleintegral);
    memcpy(e->acc_integral, s->acc_integral, sizeof e->acc_integral);
    memcpy(e->acc_doubleintegral, s->acc_doubleintegral, sizeof e->acc_doubleintegral);
    memcpy(e->cross, s->cross, sizeof e->cross);
    memcpy(e->dalpha_db_g, s->dalpha_db_g, sizeof e->dalpha_db_g);
    memcpy(e->dv_db_g, s->dv_db_g, sizeof e->dv_db_g);
    memcpy(e->dp_db_g, s->dp_db_g, sizeof e->dp_db_g);
    memcpy(e->P_delta, s->P_delta, sizeof e->P_delta);
    memcpy(e->sb_ref, s->sb_ref, sizeof e->sb_ref);
    e->redo = (int)s->redo; e->redo_counter = (int)s->redo_counter;
    memcpy(e->information, s->information, sizeof e->information);
    memcpy(e->sqrt_information, s->sqrt_information, sizeof e->sqrt_information);
    for (j = 0; j < s->n_dps; ++j) memcpy(e->dPdsigma[j], s->dPdsigma[j], sizeof e->dPdsigma[j]);
}

/* compare an ok_imu_error against a snapshot; returns number of mismatching values in this record */
static int cmp_error_snap(counts* c, const ok_imu_error* e, const snap* s, const char* what, int verbose) {
    int bad = 0, b;
    uint32_t j;
    double dq[4] = {e->delta_q.x, e->delta_q.y, e->delta_q.z, e->delta_q.w};
#define CMP(name, a, b_, n) do { b = cmpv(c, a, b_, n); if (b && verbose) fprintf(stderr, "    %s: %s differs (%d values)\n", what, name, b); bad += b; } while (0)
    CMP("Delta_q", dq, s->delta_q, 4);
    CMP("C_integral", e->C_integral, s->C_integral, 9);
    CMP("C_doubleintegral", e->C_doubleintegral, s->C_doubleintegral, 9);
    CMP("acc_integral", e->acc_integral, s->acc_integral, 3);
    CMP("acc_doubleintegral", e->acc_doubleintegral, s->acc_doubleintegral, 3);
    CMP("cross", e->cross, s->cross, 9);
    CMP("dalpha_db_g", e->dalpha_db_g, s->dalpha_db_g, 9);
    CMP("dv_db_g", e->dv_db_g, s->dv_db_g, 9);
    CMP("dp_db_g", e->dp_db_g, s->dp_db_g, 9);
    CMP("P_delta", e->P_delta, s->P_delta, 225);
    CMP("sb_ref", e->sb_ref, s->sb_ref, 9);
    CMP("information", e->information, s->information, 225);
    CMP("sqrt_information", e->sqrt_information, s->sqrt_information, 225);
    for (j = 0; j < s->n_dps; ++j) CMP("dPdsigma", e->dPdsigma[j], s->dPdsigma[j], 225);
    bad += cmpi(c, e->redo, (int64_t)s->redo);
    bad += cmpi(c, e->redo_counter, (int64_t)s->redo_counter);
    bad += cmpi(c, (int64_t)e->n_meas, (int64_t)s->n_meas);
    bad += cmpi(c, e->t0.sec, s->t0.sec) + cmpi(c, e->t0.nsec, s->t0.nsec);
    bad += cmpi(c, e->t1.sec, s->t1.sec) + cmpi(c, e->t1.nsec, s->t1.nsec);
#undef CMP
    return bad;
}

static void report(const char* kind, const counts* c) {
    printf("  %s: %ld/%ld (%ld/%ld records)\n", kind, c->bad, c->tot, c->badrecs, c->recs);
}

/* ------------------------------------------------------------------ kinds ---------------------------- */

static void check_prop(const char* path, long max, counts* c) {
    long r;
    G_f = fopen(path, "rb");
    if (!G_f) { fprintf(stderr, "note: %s not present, skipped\n", path); return; }
    for (r = 0; max < 0 || r < max; ++r) {
        size_t n;
        ok_imu_meas* m;
        ok_imu_params p;
        double T0[7], sb0[9], To[7], sbo[9], T1o[7], sb1o[9], cov_ref[225], jac_ref[225];
        double T[7], sb[9], cov[225], jac[225];
        ok_time ts, te;
        uint32_t had_cov, had_jac;
        int64_t ret, ret1;
        int bad = 0, rc;
        m = rd_meas(&n);
        if (G_eof) { free(m); break; }
        p = rd_params(); rd_f64n(T0, 7); rd_f64n(sb0, 9); ts = rd_time(); te = rd_time();
        had_cov = rd_u32(); had_jac = rd_u32(); ret = (int64_t)rd_u64();
        rd_f64n(To, 7); rd_f64n(sbo, 9); ret1 = (int64_t)rd_u64(); rd_f64n(T1o, 7); rd_f64n(sb1o, 9);
        rd_f64n(cov_ref, 225); rd_f64n(jac_ref, 225);
        if (G_eof) { free(m); break; }
        (void)had_cov; (void)had_jac;
        memcpy(T, T0, sizeof T); memcpy(sb, sb0, sizeof sb);
        rc = ok_imu_propagation(m, n, &p, T, sb, ts, te, NULL, NULL);
        bad += cmpi(c, rc, ret);
        if (ret > 0) { bad += cmpv(c, T, To, 7); bad += cmpv(c, sb, sbo, 9); }
        memcpy(T, T0, sizeof T); memcpy(sb, sb0, sizeof sb);
        rc = ok_imu_propagation(m, n, &p, T, sb, ts, te, cov, jac);
        bad += cmpi(c, rc, ret1);
        if (ret1 > 0) {
            bad += cmpv(c, T, T1o, 7); bad += cmpv(c, sb, sb1o, 9);
            bad += cmpv(c, cov, cov_ref, 225); bad += cmpv(c, jac, jac_ref, 225);
        }
        c->recs++; if (bad) { c->badrecs++; if (c->badrecs <= 3) fprintf(stderr, "    prop record %ld: %d mismatches (n=%zu)\n", r, bad, n); }
        free(m);
    }
    fclose(G_f);
}

static void check_preint(const char* path, long max, counts* c) {
    long r;
    G_f = fopen(path, "rb");
    if (!G_f) { fprintf(stderr, "note: %s not present, skipped\n", path); return; }
    for (r = 0; max < 0 || r < max; ++r) {
        double sb[9];
        uint64_t steps;
        snap s;
        ok_imu_error e;
        int bad, rc;
        G_eof = 0;
        rd_f64n(sb, 9); steps = rd_u64(); rd_snap(&s, 1);
        if (G_eof) { free(s.meas); break; }
        ok_imu_error_init(&e, s.meas, s.n_meas, &s.params, s.t0, s.t1);
        e.redo = 1; /* state before the call is irrelevant: redoPreintegration resets everything */
        rc = ok_imu_redo_preintegration(&e, sb);
        /* redo_ / redoCounter_ are bookkeeping of Evaluate(), not of redoPreintegration */
        e.redo = (int)s.redo; e.redo_counter = (int)s.redo_counter;
        bad = cmpi(c, rc, (int64_t)steps);
        bad += cmp_error_snap(c, &e, &s, "preint", r < 3);
        c->recs++; if (bad) { c->badrecs++; if (c->badrecs <= 3) fprintf(stderr, "    preint record %ld: %d mismatches\n", r, bad); }
        ok_imu_error_free(&e); free(s.meas);
    }
    fclose(G_f);
}

static void check_append(const char* path, long max, counts* c) {
    long r;
    G_f = fopen(path, "rb");
    if (!G_f) { fprintf(stderr, "note: %s not present, skipped\n", path); return; }
    for (r = 0; max < 0 || r < max; ++r) {
        snap pre, post;
        double sb[9];
        size_t n;
        ok_imu_meas* nm;
        ok_time t_1;
        int64_t ret;
        ok_imu_error e;
        int bad, rc;
        G_eof = 0;
        rd_snap(&pre, 1);
        rd_f64n(sb, 9); nm = rd_meas(&n); t_1 = rd_time(); ret = (int64_t)rd_u64();
        rd_snap(&post, 0);
        if (G_eof) { free(pre.meas); free(nm); break; }
        load_error_from_snap(&e, &pre);
        rc = ok_imu_append(&e, sb, nm, n, t_1);
        bad = cmpi(c, rc, ret);
        if (ret > 0 || ret == 0) bad += cmp_error_snap(c, &e, &post, "append", r < 3);
        c->recs++; if (bad) { c->badrecs++; if (c->badrecs <= 3) fprintf(stderr, "    append record %ld: %d mismatches\n", r, bad); }
        ok_imu_error_free(&e); free(pre.meas); free(nm);
    }
    fclose(G_f);
}

static void check_eval(const char* path, long max, counts* c) {
    long r;
    G_f = fopen(path, "rb");
    if (!G_f) { fprintf(stderr, "note: %s not present, skipped\n", path); return; }
    for (r = 0; max < 0 || r < max; ++r) {
        uint32_t redoPre, redoCounterPre, okflag, redone, has[4];
        uint64_t nMeasPre;
        ok_time t0, t1;
        double g, sb_ref[9], dq[4], Ci[9], Cd[9], ai[3], ad[3], da[9], dv[9], dp[9], sqrtI[225];
        double pa[7], pb[9], pc[7], pd[9], res_ref[15], jref[4][135], res[15], jbuf[4][135];
        const double* params[4];
        double* jac[4];
        static const int jn[4] = {105, 135, 105, 135};
        snap s;
        ok_imu_error e;
        int bad = 0, k;
        G_eof = 0;
        memset(&s, 0, sizeof s);
        redoPre = rd_u32(); redoCounterPre = rd_u32(); nMeasPre = rd_u64(); t0 = rd_time(); t1 = rd_time();
        g = rd_f64(); rd_f64n(sb_ref, 9); rd_f64n(dq, 4); rd_f64n(Ci, 9); rd_f64n(Cd, 9); rd_f64n(ai, 3); rd_f64n(ad, 3);
        rd_f64n(da, 9); rd_f64n(dv, 9); rd_f64n(dp, 9); rd_f64n(sqrtI, 225);
        rd_f64n(pa, 7); rd_f64n(pb, 9); rd_f64n(pc, 7); rd_f64n(pd, 9);
        okflag = rd_u32(); rd_f64n(res_ref, 15);
        for (k = 0; k < 4; ++k) { has[k] = rd_u32(); if (has[k]) rd_f64n(jref[k], (size_t)jn[k]); }
        redone = rd_u32();
        if (G_eof) break;
        if (redone) {
            uint64_t sz = rd_u64();
            (void)sz;
            rd_snap(&s, 1);
            if (G_eof) { free(s.meas); break; }
            load_error_from_snap(&e, &s);
        } else {
            ok_imu_params prm;
            ok_imu_meas* dummy = (ok_imu_meas*)calloc(nMeasPre ? (size_t)nMeasPre : 1, sizeof(ok_imu_meas));
            memset(&prm, 0, sizeof prm);
            prm.g = g;
            ok_imu_error_init(&e, dummy, (size_t)nMeasPre, &prm, t0, t1);
            free(dummy);
            e.delta_q.x = dq[0]; e.delta_q.y = dq[1]; e.delta_q.z = dq[2]; e.delta_q.w = dq[3];
            memcpy(e.C_integral, Ci, sizeof Ci); memcpy(e.C_doubleintegral, Cd, sizeof Cd);
            memcpy(e.acc_integral, ai, sizeof ai); memcpy(e.acc_doubleintegral, ad, sizeof ad);
            memcpy(e.dalpha_db_g, da, sizeof da); memcpy(e.dv_db_g, dv, sizeof dv); memcpy(e.dp_db_g, dp, sizeof dp);
            memcpy(e.sqrt_information, sqrtI, sizeof sqrtI);
            memcpy(e.sb_ref, sb_ref, sizeof sb_ref);
            e.redo = (int)redoPre; e.redo_counter = (int)redoCounterPre;
        }
        params[0] = pa; params[1] = pb; params[2] = pc; params[3] = pd;
        for (k = 0; k < 4; ++k) jac[k] = has[k] ? jbuf[k] : NULL;
        {
            int ok = ok_imu_evaluate(&e, params, res, jac);
            bad += cmpi(c, ok, (int64_t)okflag);
        }
        {
            int b = cmpv(c, res, res_ref, 15);
            G_cat[4] += b;
            bad += b;
        }
        for (k = 0; k < 4; ++k) if (has[k]) {
            int b = cmpv(c, jbuf[k], jref[k], (size_t)jn[k]);
            G_cat[k] += b;
            if (b && c->badrecs < 3) fprintf(stderr, "    eval record %ld: jacobian %d differs (%d values)\n", r, k, b);
            bad += b;
        }
        c->recs++; if (bad) { c->badrecs++; if (c->badrecs <= 40) fprintf(stderr, "    eval record %ld: %d mismatches (redone=%u redoPre=%u counterPre=%u nmeas=%llu)\n", r, bad, redone, redoPre, redoCounterPre, (unsigned long long)nMeasPre); }
        ok_imu_error_free(&e); free(s.meas);
    }
    fclose(G_f);
}

static void check_initpose(const char* path, long max, counts* c) {
    long r;
    G_f = fopen(path, "rb");
    if (!G_f) { fprintf(stderr, "note: %s not present, initPose not checked\n", path); return; }
    for (r = 0; max < 0 || r < max; ++r) {
        size_t n;
        ok_imu_meas* m;
        double Tref[7], T[7];
        int bad = 0;
        G_eof = 0;
        m = rd_meas(&n);
        rd_f64n(Tref, 7);
        if (G_eof) { free(m); break; }
        ok_imu_init_pose(m, n, T);
        bad += cmpv(c, T, Tref, 7);
        c->recs++; if (bad) c->badrecs++;
        free(m);
    }
    fclose(G_f);
}

extern int g_eval_temp_parity[2];
int main(int argc, char** argv) {
    if (getenv("OK_EVAL_PARITY")) { g_eval_temp_parity[0] = getenv("OK_EVAL_PARITY")[0] - 48; g_eval_temp_parity[1] = getenv("OK_EVAL_PARITY")[1] - 48; }
    const char* label = argc > 1 ? argv[1] : "seq";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    counts cp = {0, 0, 0, 0}, cpi = {0, 0, 0, 0}, ca = {0, 0, 0, 0}, ce = {0, 0, 0, 0}, ci = {0, 0, 0, 0};
    long bad, tot;
    snprintf(path, sizeof path, "%s/imu_prop.bin", dir);
    check_prop(path, max, &cp);
    report("propagation", &cp);
    snprintf(path, sizeof path, "%s/imu_preint.bin", dir);
    check_preint(path, max, &cpi);
    report("redoPreintegration", &cpi);
    snprintf(path, sizeof path, "%s/imu_append.bin", dir);
    check_append(path, max, &ca);
    report("append", &ca);
    snprintf(path, sizeof path, "%s/imu_eval.bin", dir);
    check_eval(path, max, &ce);
    report("Evaluate", &ce);
    fprintf(stderr, "    Evaluate by output: J0 %ld  J1 %ld  J2 %ld  J3 %ld  residual %ld\n", G_cat[0], G_cat[1], G_cat[2], G_cat[3], G_cat[4]);
    snprintf(path, sizeof path, "%s/imu_initpose.bin", dir);
    check_initpose(path, max, &ci);
    report("initPose", &ci);
    bad = cp.bad + cpi.bad + ca.bad + ce.bad + ci.bad;
    tot = cp.tot + cpi.tot + ca.tot + ce.tot + ci.tot;
    printf("%s: %ld/%ld\n", label, bad, tot);
    return bad == 0 ? 0 : 1;
}
