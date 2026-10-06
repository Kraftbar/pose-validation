/* Bit-exactness harness for rdvio_port module M10 (rd_sys_swt.c: SlidingWindowTracker::track).
 *
 *   check_rd_swt <dump dir>        (swt.bin of patch 0012: RDVIO_PORT_SWT_DIR [RDVIO_PORT_SWT_ALL / _EVERY / _FULL])
 *
 * swt.bin (framed: u32 tag, u64 bytes, payload), per logged track() call:
 *    1 IN    u64 call, u32 parsac, [window full], [marg], [ft]
 *    5 PNP   u64 npts, u64 n, n mask bytes          (after find_pnp_matrix_parsac_imu in judge_track_status)
 *    6 ESS   u64 npts, u64 n, n mask bytes          (after find_essential_matrix_parsac in filter_parsac_2d2d)
 *   11 JUDGE, 12 UPDATE, 13 LOCALIZE, 14 MANAGE, 15 LANDMARK, 16 REFINE, 17 SLIDE, 18 SUBWINDOW:
 *          u64 FNV-1a 64 of the stage payload [the payload itself with RDVIO_PORT_SWT_FULL=1];
 *          stage payload = u32 value (judge result / keyframe), [f64 m_th: judge true], [window], [marg: 16, 17], [ft: 12]
 *   window = u64 nframes, nframes x {frame, u64 nsub, nsub x frame}, u64 ntracks,
 *            ntracks x {u64 id, u32 tags, f64 inv_depth, u64 m_life, u64 nrefs, nrefs x {u64 frame id, u64 kp}}
 *   frame  = u64 id, u32 tags, pose q[4] p[3], motion v bg ba [9],
 *            full: t, K[9], sqrt_inv_cov[4], camera q[4] p[3], imu q[4] p[3], pre (preintegration), pre (keyframe_preintegration),
 *                  u64 nkp, nkp x {bearing[3], u64 track id (0: none)}
 *   pre    = cov_w cov_a cov_bg cov_ba [36], delta t q[4] p[3] v[3] cov[225] sqrt_inv_cov[225], jacobian dq_dbg dp_dbg dp_dba
 *            dv_dbg dv_dba [45], u64 n, n x {t, w[3], a[3]}
 *   marg   = u32 has, [u64 nf, nf x {u64 frame id, lin pose q[4] p[3], lin motion v bg ba [9]}, sqrt_inv_cov[N*N], infovec[N]]
 *   ft     = u32 found, [u64 nkp, nkp x u32 (0 no track, 1 track, 3 TT_STATIC track)]   (the feature-tracking frame of the
 *            current frame's id: update_track_status's old_frame)
 * Every IN is rebuilt (keyframe map with subframes and tracks, marginalization factor, a one-frame feature-tracking map) and
 * run through rd_swt_track with the logged PARSAC masks; each stage payload is serialized and compared by hash (and byte for
 * byte when present). Native Ceres solves (module M4), marginalization (M5). Last line: "swt: <mismatches>/<compared>".
 */
#include "rd_sys_swt.h"
#include "rd_sys_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct rec { uint32_t tag; uint64_t len; unsigned char* p; } rec;
typedef struct cur { const unsigned char* p; size_t off, len; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); c->off += 4; return v; }
static uint64_t cu64(cur* c) { uint64_t v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); c->off += 8; return v; }
static double cf64(cur* c) { double v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); c->off += 8; return v; }
static void cf64n(cur* c, double* v, size_t n) { size_t i; for (i = 0; i < n; ++i) v[i] = cf64(c); }
static ok_quat cq(cur* c) { ok_quat q; q.x = cf64(c); q.y = cf64(c); q.z = cf64(c); q.w = cf64(c); return q; }
typedef struct buf { unsigned char* p; size_t n, cap; } buf;
static void put(buf* b, const void* d, size_t n) {
    if (b->n + n > b->cap) { b->cap = (b->n + n) * 2 + 256; b->p = (unsigned char*)realloc(b->p, b->cap); }
    if (n) memcpy(b->p + b->n, d, n);
    b->n += n;
}
static void pu32(buf* b, uint32_t v) { put(b, &v, 4); }
static void pu64(buf* b, uint64_t v) { put(b, &v, 8); }
static void pf64(buf* b, double v) { put(b, &v, 8); }
static void pq(buf* b, const ok_quat* q) { pf64(b, q->x); pf64(b, q->y); pf64(b, q->z); pf64(b, q->w); }

static FILE* G_f;
static int G_debug;
static rec G_la;                     /* look-ahead record */
static int G_has_la;
static long G_bad, G_tot, G_calls, G_call_bad, G_n[20], G_nbad[20], G_shown;
static uint64_t G_call;
static int G_call_failed;
static rd_swt* G_swt;

static int read_rec(rec* r) {
    if (fread(&r->tag, 4, 1, G_f) != 1 || fread(&r->len, 8, 1, G_f) != 1) return 0;
    r->p = (unsigned char*)realloc(r->p, r->len ? (size_t)r->len : 1);
    return r->len == 0 || fread(r->p, 1, (size_t)r->len, G_f) == r->len;
}
static rec* peek(void) { if (!G_has_la) G_has_la = read_rec(&G_la); return G_has_la ? &G_la : NULL; }
static void consume(void) { G_has_la = 0; }
static void fail(int tag, const char* what, size_t at) {
    G_bad++; if (tag >= 0 && tag < 20) G_nbad[tag]++;
    if (!G_call_failed) G_call_bad++;
    G_call_failed = 1;
    if (G_debug && G_shown++ < 40) fprintf(stderr, "    call %llu: record %d: %s (byte %zu)\n", (unsigned long long)G_call, tag, what, at);
}

/* ---- serialization of the C state, as the patch writes it ---- */
static void ser_frame(buf* b, const rd_frame* f) {
    pu64(b, f->id); pu32(b, f->tags);
    pq(b, &f->pose_q); put(b, f->pose_p, 24);
    put(b, f->motion.v, 24); put(b, f->motion.bg, 24); put(b, f->motion.ba, 24);
}
static void ser_window(buf* b, const rd_map* m) {
    size_t i, k;
    pu64(b, rd_map_frame_num(m));
    for (i = 0; i < rd_map_frame_num(m); ++i) {
        const rd_frame* f = rd_map_get_frame(m, i);
        ser_frame(b, f);
        pu64(b, f->nsub);
        for (k = 0; k < f->nsub; ++k) ser_frame(b, f->sub[k]);
    }
    pu64(b, rd_map_track_num(m));
    for (i = 0; i < rd_map_track_num(m); ++i) {
        const rd_track* t = rd_map_get_track(m, i);
        pu64(b, t->id); pu32(b, t->tags); pf64(b, t->inv_depth); pu64(b, t->life); pu64(b, t->nref);
        for (k = 0; k < t->nref; ++k) { pu64(b, t->ref[k].frame->id); pu64(b, t->ref[k].kp); }
    }
}
static void ser_marg(buf* b, const rd_marg* m) {
    int i;
    size_t N;
    pu32(b, m ? 1u : 0u);
    if (!m) return;
    pu64(b, (uint64_t)m->nf);
    for (i = 0; i < m->nf; ++i) {
        pu64(b, m->ids[i]);
        pq(b, &m->lin_pose[i].q); put(b, m->lin_pose[i].p, 24);
        put(b, m->lin_motion[i].v, 24); put(b, m->lin_motion[i].bg, 24); put(b, m->lin_motion[i].ba, 24);
    }
    N = (size_t)m->nf * 15;
    put(b, m->sqrt_inv_cov, N * N * 8); put(b, m->infovec, N * 8);
}
static void ser_ft(buf* b, const rd_map* ft, uint64_t id) {
    const size_t i = rd_map_frame_num(ft) ? 0 : RD_NIL;
    const rd_frame* of = (i == 0 && rd_map_get_frame(ft, 0)->id == id) ? rd_map_get_frame(ft, 0) : NULL;
    size_t k;
    pu32(b, of ? 1u : 0u);
    if (!of) return;
    pu64(b, of->nkp);
    for (k = 0; k < of->nkp; ++k) pu32(b, of->track[k] ? ((of->track[k]->tags & RD_TAG(RD_TT_STATIC)) ? 3u : 1u) : 0u);
}
static uint64_t fnv(const unsigned char* p, size_t n) {
    uint64_t h = 1469598103934665603ull;
    size_t i;
    for (i = 0; i < n; ++i) { h ^= p[i]; h *= 1099511628211ull; }
    return h;
}

/* ---- hooks ---- */
static void on_stage(void* ctx, rd_swt* s, int st, int v) {
    buf b; rec* r;
    (void)ctx;
    memset(&b, 0, sizeof b);
    pu32(&b, (uint32_t)v);
    if (st == 11 && v) pf64(&b, s->m_th);
    ser_window(&b, s->map);
    if (st == 16 || st == 17) ser_marg(&b, s->marg);
    if (st == 12) ser_ft(&b, s->ft, rd_map_get_frame(s->map, rd_map_frame_num(s->map) - 1)->id);
    G_n[st]++; G_tot++;
    r = peek();
    if (!r || (int)r->tag != st) { fail(st, r ? "stage order differs (C took another branch)" : "log ended", 0); free(b.p); return; }
    {
        uint64_t h;
        memcpy(&h, r->p, 8);
        if (h != fnv(b.p, b.n)) {
            size_t i = 0;
            if (r->len > 8) {             /* full payload present: first differing byte */
                const unsigned char* q = r->p + 8;
                const size_t n = (size_t)r->len - 8;
                while (i < b.n && i < n && b.p[i] == q[i]) ++i;
            }
            fail(st, "stage payload differs", i);
        }
    }
    consume();
    free(b.p);
}
static void mask_hook(int tag, size_t n, char* mask) {
    rec* r = peek();
    cur c;
    uint64_t npts, nm;
    G_n[tag]++; G_tot++;
    memset(mask, 1, n);
    if (!r || (int)r->tag != tag) { fail(tag, "no mask record here", 0); return; }
    c.p = r->p; c.len = (size_t)r->len; c.off = 0;
    npts = cu64(&c); nm = cu64(&c);
    if (npts != n || nm != n) fail(tag, "point count differs", 0);
    else memcpy(mask, r->p + 16, n);
    consume();
}
static void pnp_mask(void* ctx, size_t n, const double* p3d, const double* p2d, const size_t* lens, const double Rcw[9],
                     const double tcw[3], double inv_f, char* mask) {
    (void)ctx; (void)p3d; (void)p2d; (void)lens; (void)Rcw; (void)tcw; (void)inv_f;
    mask_hook(5, n, mask);
}
static void ess_mask(void* ctx, size_t n, const double* pts1, const double* pts2, double threshold, char* mask) {
    (void)ctx; (void)pts1; (void)pts2; (void)threshold;
    mask_hook(6, n, mask);
}
static void marginalize(void* ctx, rd_map* m, size_t index) { (void)ctx; (void)m; rd_swt_marginalize(G_swt, index); }

/* ---- the IN record ---- */
static void read_pre(cur* c, rd_preint* p, rd_imu_sample** data, size_t* n, size_t* cap) {
    uint64_t k, nd;
    cf64n(c, p->cov_w, 9); cf64n(c, p->cov_a, 9); cf64n(c, p->cov_bg, 9); cf64n(c, p->cov_ba, 9);
    p->delta.t = cf64(c); p->delta.q = cq(c); cf64n(c, p->delta.p, 3); cf64n(c, p->delta.v, 3);
    cf64n(c, p->delta.cov, 225); cf64n(c, p->delta.sqrt_inv_cov, 225);
    cf64n(c, p->jac.dq_dbg, 9); cf64n(c, p->jac.dp_dbg, 9); cf64n(c, p->jac.dp_dba, 9); cf64n(c, p->jac.dv_dbg, 9); cf64n(c, p->jac.dv_dba, 9);
    nd = cu64(c);
    *n = 0;
    for (k = 0; k < nd; ++k) { rd_imu_sample s; s.t = cf64(c); cf64n(c, s.w, 3); cf64n(c, s.a, 3); rd_imu_list_insert(data, n, cap, *n, &s, 1); }
}
typedef struct fent { rd_frame* f; uint64_t* tid; } fent;
static rd_frame* read_frame(cur* c, fent* e) {
    rd_frame* f = rd_frame_new();
    uint64_t n, k;
    f->id = cu64(c); f->tags = cu32(c);
    f->pose_q = cq(c); cf64n(c, f->pose_p, 3);
    cf64n(c, f->motion.v, 3); cf64n(c, f->motion.bg, 3); cf64n(c, f->motion.ba, 3);
    f->t = cf64(c); cf64n(c, f->K, 9); cf64n(c, f->sqrt_inv_cov, 4);
    f->cam_q = cq(c); cf64n(c, f->cam_p, 3); f->imu_q = cq(c); cf64n(c, f->imu_p, 3);
    read_pre(c, &f->preint, &f->data, &f->ndata, &f->cdata);
    read_pre(c, &f->kpreint, &f->kdata, &f->nkdata, &f->ckdata);
    n = cu64(c);
    e->tid = (uint64_t*)calloc((size_t)n + 1, sizeof(uint64_t));
    for (k = 0; k < n; ++k) { double b[3]; cf64n(c, b, 3); rd_frame_append_keypoint(f, b); e->tid[k] = cu64(c); }
    e->f = f;
    return f;
}
static rd_frame* find(fent* e, size_t ne, uint64_t id) { size_t i; for (i = 0; i < ne; ++i) if (e[i].f->id == id) return e[i].f; return NULL; }
static rd_map* build(cur* c) {
    rd_map* m = rd_map_new();
    const uint64_t nf = cu64(c);
    fent* e = NULL;
    size_t ne = 0, cap = 0, i;
    uint64_t k, nt;
    for (i = 0; i < nf; ++i) {
        rd_frame* f;
        uint64_t ns, j;
        if (ne + 1 > cap) { cap = cap * 2 + 16; e = (fent*)realloc(e, cap * sizeof(fent)); }
        f = read_frame(c, &e[ne++]);
        rd_map_attach_frame(m, f, RD_NIL);
        ns = cu64(c);
        for (j = 0; j < ns; ++j) {
            if (ne + 1 > cap) { cap = cap * 2 + 16; e = (fent*)realloc(e, cap * sizeof(fent)); }
            rd_frame_sub_push(f, read_frame(c, &e[ne++]));
        }
    }
    nt = cu64(c);
    for (k = 0; k < nt; ++k) {
        rd_track* t = rd_map_create_track(m);
        uint64_t nr, j;
        rd_map_set_track_id(m, t, cu64(c));
        t->tags = cu32(c); t->inv_depth = cf64(c); t->life = cu64(c);
        nr = cu64(c);
        t->ref = (rd_kref*)malloc(sizeof(rd_kref) * (size_t)(nr + 1)); t->cap = (size_t)nr + 1; t->nref = (size_t)nr;
        for (j = 0; j < nr; ++j) {
            const uint64_t fid = cu64(c), kp = cu64(c);
            rd_frame* f = find(e, ne, fid);
            if (!f) { fail(1, "track references an unknown frame", 0); t->nref = (size_t)j; break; }
            t->ref[j].frame = f; t->ref[j].kp = (size_t)kp;
            f->track[kp] = t;
        }
    }
    for (i = 0; i < ne; ++i) {                        /* the keypoint -> track links must agree with the logged ones */
        for (k = 0; k < e[i].f->nkp; ++k) {
            G_tot++;
            if ((e[i].f->track[k] ? e[i].f->track[k]->id : 0u) != e[i].tid[k]) fail(1, "keypoint-track link differs", 0);
        }
        free(e[i].tid);
    }
    free(e);
    return m;
}
static rd_marg* read_marg(cur* c) {
    rd_marg* m;
    int i;
    size_t N;
    if (!cu32(c)) return NULL;
    m = (rd_marg*)calloc(1, sizeof(rd_marg));
    m->nf = (int)cu64(c);
    N = (size_t)m->nf * 15;
    m->ids = (uint64_t*)calloc((size_t)m->nf + 1, 8);
    m->lin_pose = (rd_pose*)calloc((size_t)m->nf + 1, sizeof(rd_pose));
    m->lin_motion = (rd_motion*)calloc((size_t)m->nf + 1, sizeof(rd_motion));
    m->sqrt_inv_cov = (double*)calloc(N * N + 1, 8);
    m->infovec = (double*)calloc(N + 1, 8);
    for (i = 0; i < m->nf; ++i) {
        m->ids[i] = cu64(c);
        m->lin_pose[i].q = cq(c); cf64n(c, m->lin_pose[i].p, 3);
        cf64n(c, m->lin_motion[i].v, 3); cf64n(c, m->lin_motion[i].bg, 3); cf64n(c, m->lin_motion[i].ba, 3);
    }
    cf64n(c, m->sqrt_inv_cov, N * N); cf64n(c, m->infovec, N);
    return m;
}
/* the feature-tracking map: the old frame alone (same id, its track links and TT_STATIC) */
static rd_map* read_ft(cur* c, uint64_t id) {
    rd_map* ft = rd_map_new();
    rd_frame* f;
    uint64_t n, k;
    if (!cu32(c)) return ft;
    f = rd_frame_new();
    f->id = id;
    n = cu64(c);
    for (k = 0; k < n; ++k) { const double b[3] = {0, 0, 1}; rd_frame_append_keypoint(f, b); }
    rd_map_attach_frame(ft, f, RD_NIL);
    for (k = 0; k < n; ++k) {
        const uint32_t v = cu32(c);
        rd_track* t;
        if (!v) continue;
        t = rd_map_create_track(ft);
        t->tags = (v & 2) ? RD_TAG(RD_TT_STATIC) : 0;
        t->ref = (rd_kref*)malloc(sizeof(rd_kref)); t->cap = 1; t->nref = 1;
        t->ref[0].frame = f; t->ref[0].kp = (size_t)k;
        f->track[k] = t;
    }
    return ft;
}

int main(int argc, char** argv) {
    char path[4096], err[256];
    rd_cfg cfg;
    rd_map_hooks mh;
    rec r;
    int st;
    memset(&r, 0, sizeof r);
    G_debug = getenv("RD_DEBUG") != NULL;
    snprintf(path, sizeof path, "%s/swt.bin", argc > 1 ? argv[1] : ".");
    G_f = fopen(path, "rb");
    if (!G_f) { printf("swt: 0/0\n"); return 1; }
    {
        const char* s = getenv("RD_SETTING") ? getenv("RD_SETTING") : "rdvio_port/reference/configs/setting.yaml";
        const char* d = getenv("RD_SENSOR") ? getenv("RD_SENSOR") : "rdvio_port/reference/configs/euroc_sensor.yaml";
        if (rd_cfg_load(s, d, &cfg, err, sizeof err)) { fprintf(stderr, "config: %s\n", err); printf("swt: 0/0\n"); return 1; }
    }
    memset(&mh, 0, sizeof mh);
    mh.marginalize = marginalize;
    rd_map_set_hooks(&mh);
    for (;;) {
        rec* in = peek();
        rd_swt s;
        cur c;
        uint64_t curr_id;
        if (!in) break;
        if (in->tag != 1) { fail((int)in->tag < 20 ? (int)in->tag : 0, "unexpected record between calls", 0); consume(); continue; }
        c.p = in->p; c.len = (size_t)in->len; c.off = 0;
        G_call = cu64(&c);
        (void)cu32(&c);                               /* parsac flag (the config's) */
        G_calls++; G_call_failed = 0;
        memset(&s, 0, sizeof s);
        s.cfg = &cfg;
        s.map = build(&c);
        s.marg = read_marg(&c);
        curr_id = rd_map_get_frame(s.map, rd_map_frame_num(s.map) - 1)->id;
        s.ft = read_ft(&c, curr_id);
        consume();
        s.hooks.pnp_mask = pnp_mask; s.hooks.ess_mask = ess_mask; s.hooks.stage = on_stage;
        G_swt = &s;
        rd_swt_track(&s);
        rd_swt_destroy(&s);
        rd_map_free(s.ft);
        while ((in = peek()) && in->tag != 1) { fail((int)in->tag < 20 ? (int)in->tag : 0, "record not consumed by C", 0); consume(); }
    }
    free(r.p); free(G_la.p);
    printf("  track() calls %ld (%ld with a mismatch); records:", G_calls, G_call_bad);
    for (st = 0; st < 20; ++st) if (G_n[st]) printf(" %d: %ld/%ld", st, G_nbad[st], G_n[st]);
    printf("\nswt: %ld/%ld\n", G_bad, G_tot);
    return G_bad == 0 && G_tot > 0 ? 0 : 1;
}
