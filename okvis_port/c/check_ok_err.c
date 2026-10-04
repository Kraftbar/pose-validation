/* OK_PORT_SOURCES: check_ok_err.c ok_err.c ok_param.c ok_cam.c ok_kin.c ok_eigen.c */
/* Bit-exactness harness for okvis_port module 3b (error terms and their information / LLT set-up).
 *
 *   check_ok_err <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_records_per_kind]
 *
 * Replays every record of <dump_dir>/err_{reproj,pose,sab,relpose,hpoint,llt,ctor}.bin (layouts in ok_err.h) through
 * the C module and compares residuals, written Jacobians (full and minimal) and the sqrt-information bitwise (memcmp,
 * signed zeros count); also checks that the Jacobian buffers the C++ code leaves unwritten stay untouched.
 * Evaluate records are replayed from the dumped state (square-root information as stored), so they do not depend on
 * the LLT; the llt / ctor records test the information set-up separately.
 * Prints one line per kind "  <kind>: <mismatching values>/<compared values> (<failing records>/<records> records)"
 * and as the LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_err.h"

static FILE* G_f;
static int G_eof;

static void rd(void* p, size_t n) {
    if (n && fread(p, 1, n, G_f) != n) G_eof = 1;
}
static uint32_t rd_u32(void) { uint32_t v = 0; rd(&v, 4); return v; }
static void rd_f64n(double* p, size_t n) { rd(p, 8 * n); }

typedef struct counts { long bad, tot, recs, badrecs; } counts;
static const char* G_what = "";  /* debugging aid: OK_DEBUG=1 prints the first mismatching entries */
static int G_dbg_printed;
static int cmpv(counts* c, const double* a, const double* b, size_t n) {
    size_t i;
    int bad = 0;
    for (i = 0; i < n; ++i) {
        c->tot++;
        if (memcmp(&a[i], &b[i], 8) != 0) {
            c->bad++; bad++;
            if (getenv("OK_DEBUG") && G_dbg_printed < 60) {
                G_dbg_printed++;
                printf("    MISMATCH %s[%zu]: got %.17g want %.17g\n", G_what, i, a[i], b[i]);
            }
        }
    }
    return bad;
}
static int cmpi(counts* c, int64_t a, int64_t b) {
    c->tot++;
    if (a != b) { c->bad++; return 1; }
    return 0;
}

static const char* G_dir;
static long G_max;
static long G_total_bad, G_total;

#define KIND_BEGIN(name)                                                                      \
    static void check_##name(void) {                                                          \
        counts c;                                                                             \
        char path[1024];                                                                      \
        long rec = 0;                                                                         \
        memset(&c, 0, sizeof c);                                                              \
        snprintf(path, sizeof path, "%s/err_%s.bin", G_dir, #name);                           \
        G_f = fopen(path, "rb");                                                              \
        if (!G_f) { printf("  %s: (no dump file)\n", #name); return; }                       \
        G_eof = 0;                                                                            \
        while (!G_eof && (G_max < 0 || rec < G_max)) {                                        \
            int bad = 0;
#define KIND_END(name)                                                                        \
            if (G_eof) break;                                                                 \
            rec++; c.recs++;                                                                  \
            if (bad) c.badrecs++;                                                             \
        }                                                                                     \
        fclose(G_f);                                                                          \
        printf("  %s: %ld/%ld (%ld/%ld records)\n", #name, c.bad, c.tot, c.badrecs, c.recs);  \
        G_total_bad += c.bad; G_total += c.tot;                                               \
    }

static void rm_to_cm(int R, int C, const double* a, double* out) { /* row-major -> column-major */
    int i, j;
    for (i = 0; i < R; ++i)
        for (j = 0; j < C; ++j) out[i + R * j] = a[i * C + j];
}

static void rd_cam(ok_cam* c) {
    uint32_t tag, w, h, nd;
    double f[4], d[OK_CAM_MAX_DIST];
    memset(d, 0, sizeof d);
    tag = rd_u32(); w = rd_u32(); h = rd_u32();
    rd_f64n(f, 4);
    nd = rd_u32();
    if (nd > OK_CAM_MAX_DIST) { G_eof = 1; nd = 0; }
    rd_f64n(d, nd);
    if (tag != OK_CAM_RADTAN && tag != OK_CAM_EQUIDISTANT && tag != OK_CAM_NODIST) { fprintf(stderr, "unsupported distortion tag %u\n", tag); G_eof = 1; tag = OK_CAM_NODIST; }
    ok_cam_init(c, (int)tag, (int)w, (int)h, f[0], f[1], f[2], f[3], d);
}

/* common Evaluate record tail */
#define MAXB 3
typedef struct evrec {
    int nb, sz[MAXB], have_jac, jn[MAXB], have_min, mn[MAXB], ret;
    double par[MAXB][8];
} evrec;
static void rd_ev_in(evrec* e) {
    int k;
    memset(e, 0, sizeof *e);
    e->nb = (int)rd_u32();
    if (e->nb < 1 || e->nb > MAXB) { G_eof = 1; e->nb = 1; }
    for (k = 0; k < e->nb; ++k) {
        e->sz[k] = (int)rd_u32();
        if (e->sz[k] > 9) { G_eof = 1; e->sz[k] = 0; }
        rd_f64n(e->par[k], (size_t)e->sz[k]);
    }
    e->have_jac = (int)rd_u32();
    for (k = 0; k < e->nb; ++k) { e->jn[k] = (int)rd_u32(); e->have_min = (int)rd_u32(); e->mn[k] = (int)rd_u32(); }
}

typedef struct bufs {
    double* J[MAXB];
    double* M[MAXB];
    double* jp[MAXB];
    double* mp[MAXB];
    double res[16];
    double wJ[MAXB][100], wM[MAXB][100];  /* wanted */
    int jw[MAXB], mw[MAXB];
} bufs;
static const double SENT = -1.2345678901234e-300;
/* run the C evaluate with sentinel-filled buffers per the flags; res_n, jsz[k] = res_n * size, msz[k] = res_n * minsize.
 * minind: minimal Jacobian written independent of jacobians (SpeedAndBiasError) */
static void prep_bufs(bufs* b, const evrec* e, int res_n, const int* jsz, const int* msz) {
    int k, i;
    (void)res_n; (void)msz;
    for (k = 0; k < e->nb; ++k) {
        b->J[k] = (double*)malloc(sizeof(double) * 100);
        b->M[k] = (double*)malloc(sizeof(double) * 100);
        for (i = 0; i < 100; ++i) { b->J[k][i] = SENT; b->M[k][i] = SENT; }
        b->jp[k] = e->jn[k] ? b->J[k] : NULL;
        b->mp[k] = e->mn[k] ? b->M[k] : NULL;
        b->jw[k] = e->jn[k] ? jsz[k] : 0;
    }
}
static void rd_ev_out(evrec* e, bufs* b, int res_n, const int* jsz, const int* msz, int minind) {
    int k;
    e->ret = (int)rd_u32();
    rd_f64n(b->res, (size_t)res_n);
    for (k = 0; k < e->nb; ++k)
        if (e->jn[k]) rd_f64n(b->wJ[k], (size_t)jsz[k]);
    for (k = 0; k < e->nb; ++k) {
        const int w = e->have_min && e->mn[k] && (minind || e->jn[k]);
        b->mw[k] = w ? msz[k] : 0;
        if (w) rd_f64n(b->wM[k], (size_t)msz[k]);
    }
}
static int cmp_ev(counts* c, const evrec* e, bufs* b, const double* res, int res_n, const int* jsz, const int* msz,
                  int minind) {
    int bad = 0, k, i;
    G_what = "res";
    bad += cmpv(c, res, b->res, (size_t)res_n);
    for (k = 0; k < e->nb; ++k) {
        G_what = k == 0 ? "J0" : k == 1 ? "J1" : "J2";
        if (e->jn[k]) bad += cmpv(c, b->J[k], b->wJ[k], (size_t)jsz[k]);
        else for (i = 0; i < 1; ++i) bad += cmpv(c, b->J[k], b->J[k], 0);
        {  /* the unwritten tail of every buffer must still be the sentinel */
            double s[100]; int n = e->jn[k] ? jsz[k] : 0, m = 100 - n;
            for (i = 0; i < m; ++i) s[i] = SENT;
            bad += cmpv(c, b->J[k] + n, s, (size_t)m);
        }
        {
            const int w = e->have_min && e->mn[k] && (minind || e->jn[k]);
            double s[100]; int n = w ? msz[k] : 0, m = 100 - n;
            G_what = k == 0 ? "M0" : k == 1 ? "M1" : "M2";
            if (w) bad += cmpv(c, b->M[k], b->wM[k], (size_t)msz[k]);
            for (i = 0; i < m; ++i) s[i] = SENT;
            bad += cmpv(c, b->M[k] + n, s, (size_t)m);
        }
    }
    return bad;
}
static void free_bufs(bufs* b, int nb) {
    int k;
    for (k = 0; k < nb; ++k) { free(b->J[k]); free(b->M[k]); }
}

KIND_BEGIN(reproj) {
    ok_reproj_err e; ok_cam cam; evrec ev; bufs b;
    double meas[2], info_rm[4], sq_rm[4], res[2];
    const double* params[3];
    double* const* jac = NULL; double* const* jm = NULL;
    double* jarr[3]; double* marr[3];
    static const int jsz[3] = {14, 8, 14}, msz[3] = {12, 6, 12};
    rd_cam(&cam); rd_f64n(meas, 2); rd_f64n(info_rm, 4); rd_f64n(sq_rm, 4);
    rd_ev_in(&ev);
    prep_bufs(&b, &ev, 2, jsz, msz);
    rd_ev_out(&ev, &b, 2, jsz, msz, 0);
    if (G_eof) { free_bufs(&b, ev.nb); break; }
    memset(&e, 0, sizeof e);
    e.cam = cam; e.meas[0] = meas[0]; e.meas[1] = meas[1];
    rm_to_cm(2, 2, info_rm, e.info); rm_to_cm(2, 2, sq_rm, e.sqrt_info);
    params[0] = ev.par[0]; params[1] = ev.par[1]; params[2] = ev.par[2];
    jarr[0] = b.jp[0]; jarr[1] = b.jp[1]; jarr[2] = b.jp[2];
    marr[0] = b.mp[0]; marr[1] = b.mp[1]; marr[2] = b.mp[2];
    if (ev.have_jac) jac = jarr;
    if (ev.have_min) jm = marr;
    bad += cmpi(&c, ok_reproj_err_evaluate(&e, params, res, jac, jm), ev.ret);
    bad += cmp_ev(&c, &ev, &b, res, 2, jsz, msz, 0);
    free_bufs(&b, ev.nb);
} KIND_END(reproj)

KIND_BEGIN(pose) {
    ok_pose_err e; evrec ev; bufs b;
    double mc[7], sq_rm[36], res[6];
    const double* params[1];
    double* const* jac = NULL; double* const* jm = NULL;
    double* jarr[1]; double* marr[1];
    static const int jsz[1] = {42}, msz[1] = {36};
    rd_f64n(mc, 7); rd_f64n(sq_rm, 36);
    rd_ev_in(&ev);
    prep_bufs(&b, &ev, 6, jsz, msz);
    rd_ev_out(&ev, &b, 6, jsz, msz, 0);
    if (G_eof) { free_bufs(&b, ev.nb); break; }
    memset(&e, 0, sizeof e);
    ok_tf_set_coeffs(&e.meas, mc, 1);
    rm_to_cm(6, 6, sq_rm, e.sqrt_info);
    params[0] = ev.par[0];
    jarr[0] = b.jp[0]; marr[0] = b.mp[0];
    if (ev.have_jac) jac = jarr;
    if (ev.have_min) jm = marr;
    bad += cmpi(&c, ok_pose_err_evaluate(&e, params, res, jac, jm), ev.ret);
    bad += cmp_ev(&c, &ev, &b, res, 6, jsz, msz, 0);
    free_bufs(&b, ev.nb);
} KIND_END(pose)

KIND_BEGIN(sab) {
    ok_sab_err e; evrec ev; bufs b;
    double sq_rm[81], res[9];
    const double* params[1];
    double* const* jac = NULL; double* const* jm = NULL;
    double* jarr[1]; double* marr[1];
    static const int jsz[1] = {81}, msz[1] = {81};
    memset(&e, 0, sizeof e);
    rd_f64n(e.meas, 9); rd_f64n(sq_rm, 81);
    rd_ev_in(&ev);
    prep_bufs(&b, &ev, 9, jsz, msz);
    rd_ev_out(&ev, &b, 9, jsz, msz, 1);
    if (G_eof) { free_bufs(&b, ev.nb); break; }
    rm_to_cm(9, 9, sq_rm, e.sqrt_info);
    params[0] = ev.par[0];
    jarr[0] = b.jp[0]; marr[0] = b.mp[0];
    if (ev.have_jac) jac = jarr;
    if (ev.have_min) jm = marr;
    bad += cmpi(&c, ok_sab_err_evaluate(&e, params, res, jac, jm), ev.ret);
    bad += cmp_ev(&c, &ev, &b, res, 9, jsz, msz, 1);
    free_bufs(&b, ev.nb);
} KIND_END(sab)

KIND_BEGIN(relpose) {
    ok_relpose_err e; evrec ev; bufs b;
    double mc[7], sq_rm[36], res[6];
    const double* params[2];
    double* const* jac = NULL; double* const* jm = NULL;
    double* jarr[2]; double* marr[2];
    static const int jsz[2] = {42, 42}, msz[2] = {36, 36};
    rd_f64n(mc, 7); rd_f64n(sq_rm, 36);
    rd_ev_in(&ev);
    prep_bufs(&b, &ev, 6, jsz, msz);
    rd_ev_out(&ev, &b, 6, jsz, msz, 0);
    if (G_eof) { free_bufs(&b, ev.nb); break; }
    memset(&e, 0, sizeof e);
    ok_tf_set_coeffs(&e.T_AB, mc, 1);
    rm_to_cm(6, 6, sq_rm, e.sqrt_info);
    params[0] = ev.par[0]; params[1] = ev.par[1];
    jarr[0] = b.jp[0]; jarr[1] = b.jp[1]; marr[0] = b.mp[0]; marr[1] = b.mp[1];
    if (ev.have_jac) jac = jarr;
    if (ev.have_min) jm = marr;
    bad += cmpi(&c, ok_relpose_err_evaluate(&e, params, res, jac, jm), ev.ret);
    bad += cmp_ev(&c, &ev, &b, res, 6, jsz, msz, 0);
    free_bufs(&b, ev.nb);
} KIND_END(relpose)

KIND_BEGIN(hpoint) {
    ok_hpoint_err e; evrec ev; bufs b;
    double sq_rm[9], res[3];
    const double* params[1];
    double* const* jac = NULL; double* const* jm = NULL;
    double* jarr[1]; double* marr[1];
    static const int jsz[1] = {12}, msz[1] = {9};
    memset(&e, 0, sizeof e);
    rd_f64n(e.meas, 4); rd_f64n(sq_rm, 9);
    rd_ev_in(&ev);
    prep_bufs(&b, &ev, 3, jsz, msz);
    rd_ev_out(&ev, &b, 3, jsz, msz, 0);
    if (G_eof) { free_bufs(&b, ev.nb); break; }
    rm_to_cm(3, 3, sq_rm, e.sqrt_info);
    params[0] = ev.par[0];
    jarr[0] = b.jp[0]; marr[0] = b.mp[0];
    if (ev.have_jac) jac = jarr;
    if (ev.have_min) jm = marr;
    bad += cmpi(&c, ok_hpoint_err_evaluate(&e, params, res, jac, jm), ev.ret);
    bad += cmp_ev(&c, &ev, &b, res, 3, jsz, msz, 0);
    free_bufs(&b, ev.nb);
} KIND_END(hpoint)

KIND_BEGIN(llt) {
    double info_rm[81], sq_rm[81], info[81], got[81], gotrm[81];
    uint32_t n = rd_u32();
    int i, j;
    if (n < 1 || n > 9) { G_eof = 1; break; }
    rd_f64n(info_rm, (size_t)n * n); rd_f64n(sq_rm, (size_t)n * n);
    if (G_eof) break;
    rm_to_cm((int)n, (int)n, info_rm, info);
    ok_llt_sqrt_information((int)n, info, got);
    for (i = 0; i < (int)n; ++i)
        for (j = 0; j < (int)n; ++j) gotrm[i * (int)n + j] = got[i + (int)n * j];
    bad += cmpv(&c, gotrm, sq_rm, (size_t)n * n);
} KIND_END(llt)

KIND_BEGIN(ctor) {
    uint32_t tag = rd_u32(), nin = rd_u32(), n;
    double in[16], info_rm[81], sq_rm[81], gi[81], gs[81], girm[81], gsrm[81];
    int i, j;
    if (nin > 16) { G_eof = 1; break; }
    rd_f64n(in, nin);
    n = rd_u32();
    if (n < 1 || n > 9) { G_eof = 1; break; }
    rd_f64n(info_rm, (size_t)n * n); rd_f64n(sq_rm, (size_t)n * n);
    if (G_eof) break;
    switch (tag) {
        case 0: { ok_pose_err e; ok_tf T; ok_tf_set_coeffs(&T, in, 1); ok_pose_err_init_diag(&e, &T, in + 7);
                  memcpy(gi, e.info, sizeof(double) * 36); memcpy(gs, e.sqrt_info, sizeof(double) * 36); break; }
        case 1: { ok_pose_err e; ok_tf T; ok_tf_identity(&T); ok_pose_err_init_var(&e, &T, in[0], in[1]);
                  memcpy(gi, e.info, sizeof(double) * 36); memcpy(gs, e.sqrt_info, sizeof(double) * 36); break; }
        case 2: { ok_sab_err e; double m[9] = {0}; ok_sab_err_init_var(&e, m, in[9], in[10], in[11]);
                  memcpy(gi, e.info, sizeof(double) * 81); memcpy(gs, e.sqrt_info, sizeof(double) * 81); break; }
        case 3: { ok_relpose_err e; ok_tf T; ok_tf_identity(&T); ok_relpose_err_init_var(&e, in[0], in[1], &T);
                  memcpy(gi, e.info, sizeof(double) * 36); memcpy(gs, e.sqrt_info, sizeof(double) * 36); break; }
        case 4: { ok_hpoint_err e; ok_hpoint_err_init_var(&e, in, in[4]);
                  memcpy(gi, e.info, sizeof(double) * 9); memcpy(gs, e.sqrt_info, sizeof(double) * 9); break; }
        default: fprintf(stderr, "unknown ctor tag %u\n", tag); G_eof = 1; continue;
    }
    for (i = 0; i < (int)n; ++i)
        for (j = 0; j < (int)n; ++j) { girm[i * (int)n + j] = gi[i + (int)n * j]; gsrm[i * (int)n + j] = gs[i + (int)n * j]; }
    bad += cmpv(&c, girm, info_rm, (size_t)n * n);
    bad += cmpv(&c, gsrm, sq_rm, (size_t)n * n);
} KIND_END(ctor)

int main(int argc, char** argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s <seq_label> <fixtures|-> <dump_dir> [max]\n", argv[0]); return 2; }
    G_dir = argv[3];
    G_max = argc > 4 ? atol(argv[4]) : -1;
    check_reproj(); check_pose(); check_sab(); check_relpose(); check_hpoint(); check_llt(); check_ctor();
    printf("%s: %ld/%ld\n", argv[1], G_total_bad, G_total);
    return G_total_bad == 0 ? 0 : 1;
}
