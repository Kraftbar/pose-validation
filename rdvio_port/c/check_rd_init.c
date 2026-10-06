/* Bit-exactness harness for rdvio_port module M9 (rd_sys_init.c: Initializer::initialize).
 *
 *   check_rd_init <dump dir>        (init.bin of patch 0011: RDVIO_PORT_INIT_DIR)
 *
 * init.bin (framed: u32 tag, u64 bytes, payload):
 *   1 INIT_IN u32 has_map, [snapshot full]     2 SFM u32 ok, [state]     3 IMU u32 ok, bg[3] ba[3] gravity[3] scale, u64 n, n x v[3], [state]
 *   4 OUT [state]
 *   snapshot = u64 nframes, nframes x {u64 id, u32 tags, pose q[4] p[3], motion v bg ba [9], (full: t, K[9], sqrt_inv_cov[4],
 *              camera q[4] p[3], imu q[4] p[3], cov_w cov_a cov_bg cov_ba [36], u64 ndata, ndata x {t, w[3], a[3]}, u64 nkp,
 *              nkp x {bearing[3], u64 track id})}, u64 ntracks, ntracks x {u64 id, u32 tags, f64 inv_depth, u64 m_life, u64 nrefs,
 *              nrefs x {u64 frame index, u64 kp}}
 * Every INIT_IN with a map is rebuilt as a C map and run through rd_init_initialize (native Ceres solves through module M4); the
 * state after init_sfm (incl. the prune), the init_imu results and the final state are compared byte for byte.
 * Last line: "init: <mismatches>/<compared>".
 */
#include "rd_sys_init.h"
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
static long G_bad, G_tot, G_calls, G_maps, G_stage_n[5], G_stage_bad[5];
static int read_rec(rec* r) {
    if (fread(&r->tag, 4, 1, G_f) != 1 || fread(&r->len, 8, 1, G_f) != 1) return 0;
    r->p = (unsigned char*)realloc(r->p, r->len ? (size_t)r->len : 1);
    return r->len == 0 || fread(r->p, 1, (size_t)r->len, G_f) == r->len;
}
static void state(buf* b, rd_map* m) {
    size_t i, k;
    pu64(b, rd_map_frame_num(m));
    for (i = 0; i < rd_map_frame_num(m); ++i) {
        const rd_frame* f = rd_map_get_frame(m, i);
        pu64(b, f->id); pu32(b, f->tags);
        pq(b, &f->pose_q); put(b, f->pose_p, 24);
        put(b, f->motion.v, 24); put(b, f->motion.bg, 24); put(b, f->motion.ba, 24);
    }
    pu64(b, rd_map_track_num(m));
    for (i = 0; i < rd_map_track_num(m); ++i) {
        const rd_track* t = rd_map_get_track(m, i);
        pu64(b, t->id); pu32(b, t->tags); pf64(b, t->inv_depth); pu64(b, t->life); pu64(b, t->nref);
        for (k = 0; k < t->nref; ++k) { pu64(b, rd_map_frame_index_by_id(m, t->ref[k].frame->id)); pu64(b, t->ref[k].kp); }
    }
}
/* compare mine with the next record (expected tag); counts values per 8 bytes */
static void expect(int tag, const buf* mine) {
    rec r; memset(&r, 0, sizeof r);
    G_stage_n[tag]++;
    if (!read_rec(&r) || (int)r.tag != tag) {
        G_bad++; G_tot++; G_stage_bad[tag]++;
        if (G_debug) fprintf(stderr, "    call %ld: expected record %d, log has %u\n", G_calls, tag, r.tag);
        free(r.p); return;
    }
    G_tot += (long)(r.len / 8 + 1);
    if (r.len != mine->n || memcmp(r.p, mine->p, mine->n)) {
        size_t i = 0;
        while (i < mine->n && i < r.len && mine->p[i] == r.p[i]) ++i;
        G_bad++; G_stage_bad[tag]++;
        if (G_debug) fprintf(stderr, "    call %ld: record %d differs (C %zu bytes, log %llu), first difference at byte %zu\n", G_calls, tag, mine->n, (unsigned long long)r.len, i);
    }
    free(r.p);
}
static void on_stage(void* ctx, const rd_init* in, int stage, int ok) {
    buf b; size_t i;
    (void)ctx;
    memset(&b, 0, sizeof b);
    pu32(&b, (uint32_t)ok);
    if (stage == 1) { if (ok) state(&b, in->map); expect(2, &b); }
    else {
        put(&b, in->bg, 24); put(&b, in->ba, 24); put(&b, in->gravity, 24); pf64(&b, in->scale);
        pu64(&b, in->nvel);
        for (i = 0; i < in->nvel; ++i) put(&b, in->vel[i], 24);
        if (ok) state(&b, in->map);
        expect(3, &b);
    }
    free(b.p);
}
static rd_map* build(cur* c) {
    rd_map* m = rd_map_new();
    const uint64_t nf = cu64(c);
    uint64_t i, k, nt;
    uint64_t** tid = (uint64_t**)calloc((size_t)nf + 1, sizeof(uint64_t*));
    for (i = 0; i < nf; ++i) {
        rd_frame* f = rd_frame_new();
        uint64_t n;
        f->id = cu64(c); f->tags = cu32(c);
        f->pose_q = cq(c); cf64n(c, f->pose_p, 3);
        cf64n(c, f->motion.v, 3); cf64n(c, f->motion.bg, 3); cf64n(c, f->motion.ba, 3);
        f->t = cf64(c); cf64n(c, f->K, 9); cf64n(c, f->sqrt_inv_cov, 4);
        f->cam_q = cq(c); cf64n(c, f->cam_p, 3); f->imu_q = cq(c); cf64n(c, f->imu_p, 3);
        cf64n(c, f->preint.cov_w, 9); cf64n(c, f->preint.cov_a, 9); cf64n(c, f->preint.cov_bg, 9); cf64n(c, f->preint.cov_ba, 9);
        n = cu64(c);
        for (k = 0; k < n; ++k) { rd_imu_sample s; s.t = cf64(c); cf64n(c, s.w, 3); cf64n(c, s.a, 3); rd_imu_list_insert(&f->data, &f->ndata, &f->cdata, f->ndata, &s, 1); }
        n = cu64(c);
        tid[i] = (uint64_t*)calloc((size_t)n + 1, sizeof(uint64_t));
        for (k = 0; k < n; ++k) { double b[3]; cf64n(c, b, 3); rd_frame_append_keypoint(f, b); tid[i][k] = cu64(c); }
        rd_map_attach_frame(m, f, RD_NIL);
    }
    nt = cu64(c);
    for (i = 0; i < nt; ++i) {
        rd_track* t = rd_map_create_track(m);
        uint64_t nr, j;
        rd_map_set_track_id(m, t, cu64(c));
        t->tags = cu32(c); t->inv_depth = cf64(c); t->life = cu64(c);
        nr = cu64(c);
        t->ref = (rd_kref*)malloc(sizeof(rd_kref) * (size_t)(nr + 1)); t->cap = (size_t)nr + 1; t->nref = (size_t)nr;
        for (j = 0; j < nr; ++j) {
            const uint64_t fi = cu64(c), kp = cu64(c);
            rd_frame* f = rd_map_get_frame(m, (size_t)fi);
            t->ref[j].frame = f; t->ref[j].kp = (size_t)kp;
            f->track[kp] = t;
        }
    }
    for (i = 0; i < nf; ++i) {                        /* the keypoint -> track links must agree with the logged ones */
        const rd_frame* f = rd_map_get_frame(m, (size_t)i);
        for (k = 0; k < f->nkp; ++k) { G_tot++; if ((f->track[k] ? f->track[k]->id : 0u) != tid[i][k]) G_bad++; }
        free(tid[i]);
    }
    free(tid);
    return m;
}

int main(int argc, char** argv) {
    char path[4096];
    rd_cfg cfg;
    char err[256];
    rec r; memset(&r, 0, sizeof r);
    G_debug = getenv("RD_DEBUG") != NULL;
    snprintf(path, sizeof path, "%s/init.bin", argc > 1 ? argv[1] : ".");
    G_f = fopen(path, "rb");
    if (!G_f) { printf("init: 0/0\n"); return 1; }
    {   /* the reference run's configs */
        const char* s = getenv("RD_SETTING") ? getenv("RD_SETTING") : "rdvio_port/reference/configs/setting.yaml";
        const char* d = getenv("RD_SENSOR") ? getenv("RD_SENSOR") : "rdvio_port/reference/configs/euroc_sensor.yaml";
        if (rd_cfg_load(s, d, &cfg, err, sizeof err)) { fprintf(stderr, "config: %s\n", err); printf("init: 0/0\n"); return 1; }
    }
    while (read_rec(&r)) {
        cur c;
        rd_init in;
        if (r.tag != 1) { G_bad++; G_tot++; if (G_debug) fprintf(stderr, "    unexpected record %u at top level\n", r.tag); continue; }
        G_calls++;
        c.p = r.p; c.len = (size_t)r.len; c.off = 0;
        if (!cu32(&c)) continue;                      /* no map: initialize() returns at once */
        G_maps++;
        rd_init_create(&in, &cfg);
        in.map = build(&c);
        in.on_stage = on_stage;
        if (rd_init_initialize(&in)) {
            buf b; memset(&b, 0, sizeof b);
            state(&b, in.map);
            expect(4, &b);
            free(b.p);
        }
        rd_init_destroy(&in);
    }
    free(r.p);
    printf("  initialize() calls %ld, with a map %ld; records compared: SFM %ld (%ld differ), IMU %ld (%ld differ), OUT %ld (%ld differ)\n",
           G_calls, G_maps, G_stage_n[2], G_stage_bad[2], G_stage_n[3], G_stage_bad[3], G_stage_n[4], G_stage_bad[4]);
    printf("init: %ld/%ld\n", G_bad, G_tot);
    return G_bad == 0 && G_tot > 0 ? 0 : 1;
}
