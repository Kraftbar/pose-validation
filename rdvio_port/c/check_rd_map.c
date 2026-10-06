/* Bit-exactness harness for rdvio_port module M6 (rd_map.c: Frame / Track / Map bookkeeping, re-anchoring, the keypoint logic of
 * Frame::detect_keypoints and Frame::track_keypoints).
 *
 *   check_rd_map <dump dir>   |   check_rd_map <label> <map.bin>     (patch 0009: RDVIO_PORT_MAP_DIR=<dump dir>, see the runner)
 *
 * The replay executes only the TOP-LEVEL operations of the log on the C map layer (map construction / destruction, frame
 * construction / clone / destruction, attach, detach, untrack, erase, marginalize, frame_index_by_id, create / erase / prune
 * tracks, add / remove keypoint, append keypoint, detect, track keypoints); every event the C code reports, including the nested
 * ones (the removals of an untrack, the recycles, the appends of track_keypoints, ...), is compared with the next logged record,
 * byte for byte. What the map layer does not own is taken from the record it is about to produce: poses / extrinsics and the
 * inverse depth before a re-anchoring, the TT_TRIANGULATED tag before add_keypoint, K / extrinsics / the preintegrated rotation /
 * the config / the LK result / the TT_TRASH tags before track_keypoints, the detector's pixel list. The DIGEST records (every
 * live map: frames with their keypoint -> track ids, tracks in vector order with their references and m_life) are compared with
 * the C maps. Last line: "<label>: <mismatches>/<compared>".
 */
#include "rd_map.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct rec { uint32_t tag, len; unsigned char* p; } rec;
static FILE* G_f;
static rec G_pend, G_look;           /* the record being dispatched (consumed by its own event) and one record of look-ahead */
static int G_pend_live, G_look_live;
static long G_bad, G_tot, G_ops[21], G_events[21], G_ev_bad[21];
static int G_debug;
#define DBG(...) do { if (G_debug && G_bad < 30) { fprintf(stderr, "    "); fprintf(stderr, __VA_ARGS__); fputc('\n', stderr); } } while (0)

static int read_rec(rec* r) {
    if (fread(&r->tag, 4, 1, G_f) != 1 || fread(&r->len, 4, 1, G_f) != 1) return 0;
    r->p = (unsigned char*)realloc(r->p, r->len ? r->len : 1);
    return r->len == 0 || fread(r->p, 1, r->len, G_f) == r->len;
}
/* the record the next event of `tag` will be compared with */
static rec* expected(int tag) {
    if (G_pend_live && (int)G_pend.tag == tag) return &G_pend;
    if (!G_look_live) { if (!read_rec(&G_look)) return NULL; G_look_live = 1; }
    return &G_look;
}
static void consume(rec* r) { if (r == &G_pend) G_pend_live = 0; else G_look_live = 0; }

/* ---- object tables ---- */
#define MAXMAP 64
static rd_map* G_map[MAXMAP + 1];                  /* by serial */
static rd_frame** G_fr; static size_t G_frcap;     /* by frame serial */
static uint32_t map_serial(const rd_map* m) { uint32_t s; for (s = 1; s <= MAXMAP; ++s) if (G_map[s] == m) return s; return 0; }
static uint32_t frame_serial(const rd_frame* f) { return f ? (uint32_t)(uintptr_t)f->user : 0u; }
static rd_frame* frame_of(uint32_t s) { return s < G_frcap ? G_fr[s] : NULL; }
static void set_frame(uint32_t s, rd_frame* f) {
    if (s >= G_frcap) { size_t n = G_frcap ? G_frcap : 1024; while (n <= s) n *= 2; G_fr = (rd_frame**)realloc(G_fr, n * sizeof *G_fr); memset(G_fr + G_frcap, 0, (n - G_frcap) * sizeof *G_fr); G_frcap = n; }
    G_fr[s] = f;
    if (f) f->user = (void*)(uintptr_t)s;
}
static rd_track* track_by_id(uint64_t id) {
    uint32_t s;
    for (s = 1; s <= MAXMAP; ++s) if (G_map[s]) { rd_track* t = rd_map_get_track_by_id(G_map[s], id); if (t) return t; }
    return NULL;
}

/* ---- byte buffer of what the C side would have logged ---- */
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
typedef struct cur { const unsigned char* p; size_t off, len; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); c->off += 4; return v; }
static uint64_t cu64(cur* c) { uint64_t v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); c->off += 8; return v; }
static double cf64(cur* c) { double v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); c->off += 8; return v; }
static ok_quat cq(cur* c) { ok_quat q; q.x = cf64(c); q.y = cf64(c); q.z = cf64(c); q.w = cf64(c); return q; }

static void compare(int tag, const buf* mine, rec* r) {
    const int same = r && r->len == mine->n && (mine->n == 0 || memcmp(r->p, mine->p, mine->n) == 0);
    G_tot++; G_events[tag]++;
    if (!same) {
        size_t i = 0;
        G_bad++; G_ev_bad[tag]++;
        if (r) while (i < mine->n && i < r->len && mine->p[i] == r->p[i]) ++i;
        DBG("event %d differs (C %zu bytes, log %u bytes, first difference at byte %zu)", tag, mine->n, r ? r->len : 0, i);
    }
}

/* ---- prepare hooks: inputs the map layer does not own ---- */
static void prep_reanchor(void* ctx, rd_track* t, rd_frame* old_first, rd_frame* new_first) {
    rec* r = expected(16);
    cur c;
    (void)ctx;
    if (!r || r->tag != 16) return;
    c.p = r->p; c.len = r->len; c.off = 8 + 4 + 8 + 4 + 4 + 8 + 4;
    old_first->pose_q = cq(&c); old_first->pose_p[0] = cf64(&c); old_first->pose_p[1] = cf64(&c); old_first->pose_p[2] = cf64(&c);
    old_first->cam_q = cq(&c); old_first->cam_p[0] = cf64(&c); old_first->cam_p[1] = cf64(&c); old_first->cam_p[2] = cf64(&c);
    t->inv_depth = cf64(&c);
    cu32(&c); cu64(&c);
    new_first->pose_q = cq(&c); new_first->pose_p[0] = cf64(&c); new_first->pose_p[1] = cf64(&c); new_first->pose_p[2] = cf64(&c);
    new_first->cam_q = cq(&c); new_first->cam_p[0] = cf64(&c); new_first->cam_p[1] = cf64(&c); new_first->cam_p[2] = cf64(&c);
}
static void prep_add(void* ctx, rd_track* t) {
    rec* r = expected(15);
    cur c;
    (void)ctx;
    if (!r || r->tag != 15) return;
    c.p = r->p; c.len = r->len; c.off = 8 + 4 + 8;
    if (cu32(&c)) t->tags |= RD_TAG(RD_TT_TRIANGULATED); else t->tags &= ~RD_TAG(RD_TT_TRIANGULATED);
}

/* the TRACKKP record of the call in flight (inputs for the callback, outputs for the event) */
static rec* G_tk;
static int lk_cb(void* ctx, rd_frame* f, rd_frame* next, const double* curr, double* next_px, char* status, size_t n) {
    cur c;
    size_t i, np;
    int pred;
    buf mine;
    (void)ctx; (void)f; (void)next; (void)curr;
    c.p = G_tk->p; c.len = G_tk->len; c.off = 4 + 4 + 8 + 72 + 72 + 5 * 32;
    pred = (int)cu32(&c); c.off += 24;
    np = (size_t)cu64(&c);
    memset(&mine, 0, sizeof mine);                 /* the predicted initial flow, if any, must match */
    if (pred && np == n) {
        rec tmp; tmp.tag = 19; tmp.len = (uint32_t)(16 * np); tmp.p = (unsigned char*)G_tk->p + c.off;
        put(&mine, next_px, 16 * np);
        compare(19, &mine, &tmp);
        G_events[19]--;                             /* counted as part of the TRACKKP event */
    }
    free(mine.p);
    c.off += 16 * np;
    for (i = 0; i < 2 * n; ++i) next_px[i] = cf64(&c);
    for (i = 0; i < n; ++i) status[i] = (char)cu32(&c);
    for (i = 0; i < n; ++i) {
        const uint32_t tr = cu32(&c);
        rd_track* t = f->track[i];
        if ((tr == 2) != (t == NULL)) { G_bad++; G_tot++; DBG("track_keypoints: keypoint %zu track presence differs", i); continue; }
        if (t) { if (tr) t->tags |= RD_TAG(RD_TT_TRASH); else t->tags &= ~RD_TAG(RD_TT_TRASH); }
    }
    return 1;
}
static rec* G_det;
static int detect_cb(void* ctx, rd_frame* f, const double* px, size_t n, double** out, size_t* nout) {
    cur c;
    size_t i, oldn, newn;
    buf mine;
    rec tmp;
    (void)ctx; (void)f;
    c.p = G_det->p; c.len = G_det->len; c.off = 4 + 72;
    oldn = (size_t)cu64(&c); newn = (size_t)cu64(&c);
    memset(&mine, 0, sizeof mine);                 /* apply_k of the existing keypoints */
    put(&mine, px, 16 * n);
    tmp.tag = 18; tmp.len = (uint32_t)(16 * (oldn < n ? oldn : n)); tmp.p = (unsigned char*)G_det->p + c.off;
    if (oldn != n) { G_bad++; G_tot++; DBG("detect: %zu existing keypoints, log %zu", n, oldn); }
    else { compare(18, &mine, &tmp); G_events[18]--; }
    free(mine.p);
    *nout = newn;
    *out = (double*)malloc(sizeof(double) * 2 * (newn ? newn : 1));
    for (i = 0; i < 2 * newn; ++i) (*out)[i] = cf64(&c);
    return 1;
}

/* ---- the event hook ---- */
static void on_event(void* ctx, const rd_map_event* e) {
    rec* r = expected(e->tag);
    buf b;
    size_t i;
    (void)ctx;
    memset(&b, 0, sizeof b);
    if (!r) { G_bad++; G_tot++; DBG("event %d beyond the end of the log", e->tag); return; }
    if ((int)r->tag != e->tag) {
        G_bad++; G_tot++; G_ev_bad[e->tag]++;
        DBG("C event %d, the log has record %u", e->tag, r->tag);
        return;                                     /* not consumed: the dispatcher will see it */
    }
    switch (e->tag) {
        case 1: {                                   /* MAPNEW: take the serial */
            cur c; uint32_t s; c.p = r->p; c.len = r->len; c.off = 0; s = cu32(&c);
            if (s >= 1 && s <= MAXMAP) G_map[s] = (rd_map*)e->map;
            pu32(&b, s);
            break;
        }
        case 2: pu32(&b, map_serial(e->map)); break;
        case 3: {                                   /* FRAMENEW: take the object serial */
            cur c; uint32_t s; c.p = r->p; c.len = r->len; c.off = 0; s = cu32(&c);
            set_frame(s, (rd_frame*)e->frame);
            pu32(&b, s); pu64(&b, e->a); pu32(&b, e->b ? frame_serial((const rd_frame*)(uintptr_t)e->c) : 0u);
            break;
        }
        case 4: pu32(&b, frame_serial(e->frame)); break;
        case 5: pu32(&b, map_serial(e->map)); pu32(&b, frame_serial(e->frame)); pu64(&b, e->a); pu64(&b, e->b); break;
        case 6: case 8: case 9: pu32(&b, map_serial(e->map)); pu64(&b, e->a); pu32(&b, frame_serial(e->frame)); break;
        case 7: pu32(&b, map_serial(e->map)); pu32(&b, frame_serial(e->frame)); break;
        case 10: pu32(&b, map_serial(e->map)); pu64(&b, e->a); pu64(&b, e->b); break;
        case 11: pu32(&b, map_serial(e->map)); pu64(&b, e->track->id); pu64(&b, e->a); break;
        case 12: pu32(&b, map_serial(e->map)); pu64(&b, e->track->id); break;
        case 13: {
            rd_track* const* sel = (rd_track* const*)(uintptr_t)e->c;
            pu32(&b, map_serial(e->map)); pu32(&b, (uint32_t)e->a);
            for (i = 0; i < e->a; ++i) pu64(&b, sel[i]->id);
            break;
        }
        case 14: pu32(&b, map_serial(e->map)); pu64(&b, e->track->id); pu64(&b, e->a); pu64(&b, e->b); break;
        case 15:
            pu64(&b, e->track->id); pu32(&b, frame_serial(e->frame)); pu64(&b, e->a);
            pu32(&b, (e->track->tags & RD_TAG(RD_TT_TRIANGULATED)) ? 1u : 0u); pu64(&b, e->track->life);
            break;
        case 16: {
            cur c; uint32_t valid_log; c.p = r->p; c.len = r->len; c.off = 8 + 4 + 8 + 4 + 4 + 8; valid_log = cu32(&c);
            pu64(&b, e->track->id); pu32(&b, frame_serial(e->frame)); pu64(&b, e->a); pu32(&b, (uint32_t)e->flag1); pu32(&b, (uint32_t)e->flag2);
            pu64(&b, e->b);
            pu32(&b, e->b == 0 ? 0u : valid_log);    /* TT_VALID is owned by other modules unless the track became empty */
            if (e->reanchor) {
                const rd_frame* o = e->frame; const rd_frame* nf = e->new_first;
                pq(&b, &o->pose_q); put(&b, o->pose_p, 24); pq(&b, &o->cam_q); put(&b, o->cam_p, 24); pf64(&b, e->inv_before);
                pu32(&b, frame_serial(nf)); pu64(&b, e->new_kp);
                pq(&b, &nf->pose_q); put(&b, nf->pose_p, 24); pq(&b, &nf->cam_q); put(&b, nf->cam_p, 24); pf64(&b, e->inv_after);
            }
            break;
        }
        case 17: pu32(&b, frame_serial(e->frame)); put(&b, e->frame->bearing + 3 * (e->frame->nkp - 1), 24); break;
        case 18: {
            const rd_frame* f = e->frame;
            cur c; c.p = r->p; c.len = r->len; c.off = 4 + 72 + 16;
            pu32(&b, frame_serial(f)); put(&b, f->K, 72); pu64(&b, e->a); pu64(&b, e->b);
            put(&b, r->p + c.off, (size_t)(16 * e->b));                       /* the pixels (detector output, input here) */
            put(&b, f->bearing + 3 * e->a, (size_t)(24 * (e->b - e->a)));   /* remove_k */
            break;
        }
        case 19: {                                  /* everything but the no-translation flag and the final status is input */
            const char* st = (const char*)(uintptr_t)e->c;
            const size_t n = (size_t)e->a;
            put(&b, r->p, r->len - 4 * (n + 1));
            pu32(&b, (uint32_t)e->flag1);
            for (i = 0; i < n; ++i) pu32(&b, (uint32_t)(unsigned char)st[i]);
            break;
        }
        default: break;
    }
    compare(e->tag, &b, r);
    consume(r);
    free(b.p);
}

/* ---- DIGEST: the C maps vs the log ---- */
static void digest(const rec* r) {
    cur c;
    uint32_t nm, k;
    long bad0 = G_bad;
    c.p = r->p; c.len = r->len; c.off = 0;
    nm = cu32(&c);
    { uint32_t s, live = 0; for (s = 1; s <= MAXMAP; ++s) if (G_map[s]) live++; G_tot++; if (live != nm) { G_bad++; DBG("digest: %u live C maps, log %u", live, nm); } }
    for (k = 0; k < nm; ++k) {
        const uint32_t s = cu32(&c);
        const rd_map* m = s <= MAXMAP ? G_map[s] : NULL;
        uint64_t nf = cu64(&c), nt, i, j;
        G_tot++; if (!m || rd_map_frame_num(m) != nf) { G_bad++; DBG("digest: map %u frames %llu", s, (unsigned long long)nf); return; }
        for (i = 0; i < nf; ++i) {
            const rd_frame* f = rd_map_get_frame(m, (size_t)i);
            const uint32_t obj = cu32(&c); const uint64_t id = cu64(&c), nk = cu64(&c);
            G_tot += 3;
            if (frame_serial(f) != obj) G_bad++;
            if (f->id != id) G_bad++;
            if (f->nkp != nk) { G_bad++; return; }
            for (j = 0; j < nk; ++j) { const uint64_t tid = cu64(&c); G_tot++; if ((f->track[j] ? f->track[j]->id : 0u) != tid) G_bad++; }
        }
        nt = cu64(&c);
        G_tot++; if (rd_map_track_num(m) != nt) { G_bad++; DBG("digest: map %u tracks %zu vs %llu", s, rd_map_track_num(m), (unsigned long long)nt); return; }
        for (i = 0; i < nt; ++i) {
            const rd_track* t = rd_map_get_track(m, (size_t)i);
            const uint64_t id = cu64(&c), mi = cu64(&c), nr = cu64(&c);
            G_tot += 3;
            if (t->id != id) G_bad++;
            if (t->map_index != mi) G_bad++;
            if (t->nref != nr) { G_bad++; return; }
            for (j = 0; j < nr; ++j) {
                const uint32_t obj = cu32(&c); const uint64_t kp = cu64(&c);
                G_tot += 2;
                if (frame_serial(t->ref[j].frame) != obj) G_bad++;
                if (t->ref[j].kp != kp) G_bad++;
            }
            G_tot++; if (t->life != cu64(&c)) G_bad++;
            cu32(&c); cf64(&c);                     /* tags, inverse depth: owned by other modules */
        }
    }
    if (G_bad != bad0) DBG("digest differs (%ld values)", G_bad - bad0);
}

static int prune_cond(void* ctx, const rd_track* t) {
    const rec* r = (const rec*)ctx;
    cur c;
    uint32_t n, i;
    c.p = r->p; c.len = r->len; c.off = 4; n = cu32(&c);
    for (i = 0; i < n; ++i) if (cu64(&c) == t->id) return 1;
    return 0;
}

int main(int argc, char** argv) {
    const char* label = argc > 2 ? argv[1] : "m6";
    char path[4096];
    rd_map_hooks h;
    long n = 0;
    G_debug = getenv("RD_DEBUG") != NULL;
    if (argc > 2) snprintf(path, sizeof path, "%s", argv[2]);
    else snprintf(path, sizeof path, "%s/map.bin", argc > 1 ? argv[1] : ".");
    G_f = fopen(path, "rb");
    if (!G_f) { printf("%s: 0/0\n", label); return 1; }
    memset(&h, 0, sizeof h);
    h.event = on_event; h.prepare_reanchor = prep_reanchor; h.prepare_add = prep_add;
    rd_map_set_hooks(&h);
    rd_map_reset_ids();
    for (;;) {
        cur c;
        if (G_look_live) { rec t = G_pend; G_pend = G_look; G_look = t; G_look_live = 0; }
        else if (!read_rec(&G_pend)) break;
        G_pend_live = 1; n++;
        c.p = G_pend.p; c.len = G_pend.len; c.off = 0;
        if (G_pend.tag <= 20) G_ops[G_pend.tag]++;
        switch (G_pend.tag) {
            case 1: rd_map_new(); break;
            case 2: { uint32_t s = cu32(&c); rd_map* m = s <= MAXMAP ? G_map[s] : NULL; size_t i;
                      if (m) { for (i = 0; i < rd_map_frame_num(m); ++i) set_frame(frame_serial(rd_map_get_frame(m, i)), NULL); rd_map_free(m); G_map[s] = NULL; }
                      break; }
            case 3: { uint32_t s = cu32(&c); cu64(&c); uint32_t src = cu32(&c); (void)s;
                      if (src) rd_frame_clone(frame_of(src)); else rd_frame_new(); break; }
            case 4: { uint32_t s = cu32(&c); rd_frame* f = frame_of(s); set_frame(s, NULL); if (f) { f->user = (void*)(uintptr_t)s; rd_frame_free(f); } break; }
            case 5: { uint32_t s = cu32(&c), o = cu32(&c); uint64_t pos = cu64(&c); rd_map_attach_frame(G_map[s], frame_of(o), (size_t)pos); break; }
            case 6: { uint32_t s = cu32(&c); uint64_t idx = cu64(&c); rd_map_detach_frame(G_map[s], (size_t)idx); break; }
            case 7: { uint32_t s = cu32(&c), o = cu32(&c); rd_map_untrack_frame(G_map[s], frame_of(o)); break; }
            case 8: { uint32_t s = cu32(&c); uint64_t idx = cu64(&c); rd_map_erase_frame(G_map[s], (size_t)idx); break; }
            case 9: { uint32_t s = cu32(&c); uint64_t idx = cu64(&c); rd_map_marginalize_frame(G_map[s], (size_t)idx); break; }
            case 10: { uint32_t s = cu32(&c); uint64_t id = cu64(&c); rd_map_frame_index_by_id(G_map[s], id); break; }
            case 11: { uint32_t s = cu32(&c); rd_map_create_track(G_map[s]); break; }
            case 12: { uint32_t s = cu32(&c); uint64_t id = cu64(&c); rd_track* t = track_by_id(id);
                       if (!t) { G_bad++; G_tot++; DBG("erase_track: no C track %llu", (unsigned long long)id); G_pend_live = 0; break; }
                       rd_map_erase_track(G_map[s], t); break; }
            case 13: { uint32_t s = cu32(&c); rec cp = G_pend; cp.p = (unsigned char*)malloc(G_pend.len ? G_pend.len : 1); memcpy(cp.p, G_pend.p, G_pend.len);
                       rd_map_prune_tracks(G_map[s], prune_cond, &cp); free(cp.p); break; }
            case 15: { uint64_t id = cu64(&c); uint32_t o = cu32(&c); uint64_t kp = cu64(&c); rd_track* t = track_by_id(id); rd_frame* f = frame_of(o);
                       if (!t || !f || kp >= f->nkp) { G_bad++; G_tot++; DBG("add_keypoint: no C track %llu / frame %u", (unsigned long long)id, o); G_pend_live = 0; break; }
                       rd_track_add_keypoint(t, f, (size_t)kp); break; }
            case 16: { uint64_t id = cu64(&c); uint32_t o = cu32(&c); cu64(&c); uint32_t su = cu32(&c); rd_track* t = track_by_id(id); rd_frame* f = frame_of(o);
                       if (!t || !f || rd_track_keypoint_index(t, f) == RD_NIL) { G_bad++; G_tot++; DBG("remove_keypoint: no C track %llu / reference", (unsigned long long)id); G_pend_live = 0; break; }
                       rd_track_remove_keypoint(t, f, (int)su); break; }
            case 17: { uint32_t o = cu32(&c); double bb[3]; bb[0] = cf64(&c); bb[1] = cf64(&c); bb[2] = cf64(&c); rd_frame_append_keypoint(frame_of(o), bb); break; }
            case 18: { uint32_t o = cu32(&c); rd_frame* f = frame_of(o); int i;
                       for (i = 0; i < 9; ++i) f->K[i] = cf64(&c);
                       G_det = &G_pend; rd_frame_detect_keypoints(f, detect_cb, NULL); break; }
            case 19: { uint32_t o = cu32(&c), no = cu32(&c); rd_frame* f = frame_of(o); rd_frame* nx = frame_of(no); rd_track_cfg cfg; int i;
                       rec cp = G_pend;
                       cu64(&c);
                       for (i = 0; i < 9; ++i) f->K[i] = cf64(&c);
                       for (i = 0; i < 9; ++i) nx->K[i] = cf64(&c);
                       f->cam_q = cq(&c); f->imu_q = cq(&c); nx->delta_q = cq(&c); nx->imu_q = cq(&c); nx->cam_q = cq(&c);
                       cfg.predict_keypoints = (int)cu32(&c); cfg.rotation_ransac_threshold = cf64(&c);
                       cfg.rotation_misalignment_threshold = cf64(&c); cfg.min_keypoint_distance = cf64(&c);
                       nx->tags &= ~RD_TAG(RD_FT_NO_TRANSLATION);
                       cp.p = (unsigned char*)malloc(G_pend.len ? G_pend.len : 1); memcpy(cp.p, G_pend.p, G_pend.len);
                       G_tk = &cp;
                       rd_frame_track_keypoints(f, nx, &cfg, lk_cb, NULL);
                       free(cp.p); break; }
            case 20: digest(&G_pend); G_pend_live = 0; break;
            default: G_bad++; G_tot++; DBG("record %u at top level", G_pend.tag); G_pend_live = 0; break;
        }
        if (G_pend_live) { G_bad++; G_tot++; DBG("record %u (op %ld) was not reproduced by the C map layer", G_pend.tag, n); G_pend_live = 0; }
        if (G_bad > 0 && !getenv("RD_CONTINUE")) {   /* the C state has diverged: later records would refer to objects it lacks */
            printf("  replay stopped at top-level record %ld (tag %u): the C map layer diverged (RD_CONTINUE=1 to go on)\n", n, G_pend.tag);
            break;
        }
    }
    {
        static const char* nm[21] = {"", "map new", "map delete", "frame new", "frame delete", "attach", "detach", "untrack", "erase frame",
                                     "marginalize frame", "frame_index_by_id", "create track", "erase track", "prune tracks", "recycle track",
                                     "add keypoint", "remove keypoint", "append keypoint", "detect keypoints", "track keypoints", "digest"};
        int t;
        printf("  records: %ld\n", n);
        for (t = 1; t <= 20; ++t) printf("  %-20s top-level %9ld, events compared %9ld, mismatches %ld\n", nm[t], G_ops[t], G_events[t], G_ev_bad[t]);
    }
    printf("%s: %ld/%ld\n", label, G_bad, G_tot);
    return G_bad == 0 && G_tot > 0 ? 0 : 1;
}
