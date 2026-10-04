/* OK_PORT_SOURCES: check_ok_vigraph.c ok_vigraph.c ok_problem.c ok_graph.c ok_twopose.c ok_err.c ok_param.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_dense.c ok_blas.c */
/* Bit-exactness harness for okvis_port module 5d (ViGraph / ViGraphEstimator state and every graph mutation).
 *
 *   check_ok_vigraph <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_mutations]
 *
 * Replays <dump_dir>/problem.bin (patch 0009 Problem log + patch 0010 mutation log, layouts in ok_problem.h and
 * ok_vigraph.h) through the C graph (ok_vigraph.c): every logged ViGraph / ViGraphEstimator mutation (and every direct
 * access of ViSlamBackend to a graph) is executed on the C state; the Problem calls the C code makes must equal, call
 * by call and with the block / residual-block identities matched through a bijection, the ones ceres::Problem logged
 * during the mutation (so the PROGRAM ORDER the solver depends on is reproduced); the results of the mutation (new
 * ids, poses, speed and biases, anyState arithmetic, covisibilities, MST edges, converted observations) must be
 * bitwise equal. At every optimise(): the C graph's states, parameter blocks (values, flags, landmark quality) and
 * IMU terms must equal the reference's (hashes of every block, at every optimise), the sampled solve.bin PROBLEM
 * snapshot (every parameter value, constant flag, residual block with its type, loss, parameter blocks and cost
 * function payload) must equal the C graph, the solver's changes (values written back, IMU re-integration) are applied
 * and the hash of all parameter blocks after the solve must equal the END record of solve.bin: graph state -> program
 * order -> solve -> updated state. Prints per-kind lines and as the LAST line "<seq_label>: <mismatches>/<total>".
 * OK_DEBUG=1 prints the first mismatches.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_vigraph.h"

/* ---------------------------------------------------------------- readers / helpers */
typedef struct cur { const unsigned char* p; size_t off, len; int bad; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); else c->bad = 1; c->off += 4; return v; }
static uint64_t cu64(cur* c) { uint64_t v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static double cf64(cur* c) { double v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static void cf64n(cur* c, double* out, size_t n) { size_t i; for (i = 0; i < n; ++i) out[i] = cf64(c); }

typedef struct cnt { long tot, bad; } cnt;
static cnt C_events, C_results, C_opt_state, C_opt_blocks, C_opt_imu, C_snap_param, C_snap_resid, C_end, C_program, C_struct;
static long C_mut[160];
static int G_debug, G_printed;
static long G_mut_no;

#define DBG(...) do { if (G_debug && G_printed < 80) { G_printed++; printf("    [mut %ld] ", G_mut_no); printf(__VA_ARGS__); printf("\n"); } } while (0)
static int chk(cnt* c, int ok) { c->tot++; if (!ok) c->bad++; return ok; }
static int chk_d(cnt* c, const char* what, double got, double want) {
    c->tot++;
    if (memcmp(&got, &want, 8) != 0) { c->bad++; DBG("%s: got %.17g want %.17g", what, got, want); return 0; }
    return 1;
}
static int chk_u(cnt* c, const char* what, uint64_t got, uint64_t want) {
    c->tot++;
    if (got != want) { c->bad++; DBG("%s: got %llu want %llu", what, (unsigned long long)got, (unsigned long long)want); return 0; }
    return 1;
}
static int chk_vec(cnt* c, const char* what, const double* got, const double* want, int n) {
    int i, ok = 1;
    for (i = 0; i < n; ++i) { c->tot++; if (memcmp(&got[i], &want[i], 8) != 0) { c->bad++; ok = 0; DBG("%s[%d]: got %.17g want %.17g", what, i, got[i], want[i]); } }
    return ok;
}

/* uint64 -> uint64 map (open addressing, tombstones) */
typedef struct u2u { uint64_t* k; uint64_t* v; unsigned char* st; int cap, n, used; } u2u;
static uint64_t hh(uint64_t x) { x ^= x >> 33; x *= 0xff51afd7ed558ccdULL; x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL; x ^= x >> 33; return x; }
static void u2u_grow(u2u* m) {
    u2u o = *m; int i;
    m->cap = o.cap ? (o.n * 4 > o.cap ? o.cap * 2 : o.cap) : 1024;
    m->k = (uint64_t*)calloc((size_t)m->cap, 8); m->v = (uint64_t*)calloc((size_t)m->cap, 8); m->st = (unsigned char*)calloc((size_t)m->cap, 1);
    m->n = m->used = 0;
    for (i = 0; i < o.cap; ++i) if (o.st[i] == 1) {
        uint64_t h = hh(o.k[i]) & (uint64_t)(m->cap - 1);
        while (m->st[h]) h = (h + 1) & (uint64_t)(m->cap - 1);
        m->k[h] = o.k[i]; m->v[h] = o.v[i]; m->st[h] = 1; m->n++; m->used++;
    }
    free(o.k); free(o.v); free(o.st);
}
static int u2u_get(const u2u* m, uint64_t k, uint64_t* v) {
    uint64_t h;
    if (!m->cap) return 0;
    h = hh(k) & (uint64_t)(m->cap - 1);
    while (m->st[h]) { if (m->st[h] == 1 && m->k[h] == k) { *v = m->v[h]; return 1; } h = (h + 1) & (uint64_t)(m->cap - 1); }
    return 0;
}
static void u2u_put(u2u* m, uint64_t k, uint64_t v) {
    uint64_t h;
    if (!m->cap || (m->used + 1) * 2 > m->cap) u2u_grow(m);
    h = hh(k) & (uint64_t)(m->cap - 1);
    while (m->st[h]) { if (m->st[h] == 1 && m->k[h] == k) { m->v[h] = v; return; } h = (h + 1) & (uint64_t)(m->cap - 1); }
    m->k[h] = k; m->v[h] = v; m->st[h] = 1; m->n++; m->used++;
}
static void u2u_del(u2u* m, uint64_t k) {
    uint64_t h;
    if (!m->cap) return;
    h = hh(k) & (uint64_t)(m->cap - 1);
    while (m->st[h]) { if (m->st[h] == 1 && m->k[h] == k) { m->st[h] = 2; m->n--; return; } h = (h + 1) & (uint64_t)(m->cap - 1); }
}

/* ---------------------------------------------------------------- the replayed graphs */
typedef struct rev {                 /* a Problem call of the reference */
    int kind; uint64_t a, b, c; uint32_t nb; uint64_t v[OK_PB_MAXB]; uint64_t cost, loss;
    uint32_t ndeps; uint64_t sid; uint32_t np, nr, full; uint64_t hp, hr;
} rev;
typedef struct revs { rev* a; int n, cap; } revs;
static void revs_push(revs* r, const rev* e) {
    if (r->n == r->cap) { r->cap = r->cap ? 2 * r->cap : 64; r->a = (rev*)realloc(r->a, sizeof(rev) * (size_t)r->cap); }
    r->a[r->n++] = *e;
}

typedef struct gslot {
    uint64_t gptr, problem;
    ok_vg* g;
    u2u p_r2c, p_c2r, r_r2c, r_c2r;      /* parameter-block / residual-block identity: reference pointer <-> C handle */
    u2u man;                              /* reference manifold pointer -> kind */
    u2u convterm;                         /* reference term pointer (CONVOBS) -> malloc'd ok_reproj_err */
    u2u mstterm;                          /* reference TwoPose term pointer -> unused marker */
} gslot;
static gslot G_g[4];
static uint64_t G_last_new_problem;
static int G_ng;

typedef struct pslot { uint64_t problem; revs ev; int alive; } pslot;   /* pending reference events per Problem */
static pslot* G_ps; static int G_nps, G_capps;
static pslot* ps_find(uint64_t problem, int create) {
    int i;
    for (i = G_nps - 1; i >= 0; --i) if (G_ps[i].alive && G_ps[i].problem == problem) return &G_ps[i];
    if (!create) return NULL;
    for (i = 0; i < G_nps; ++i) if (!G_ps[i].alive) { G_ps[i].alive = 1; G_ps[i].problem = problem; G_ps[i].ev.n = 0; return &G_ps[i]; }
    if (G_nps == G_capps) { G_capps = G_capps ? 2 * G_capps : 64; G_ps = (pslot*)realloc(G_ps, sizeof(pslot) * (size_t)G_capps); }
    memset(&G_ps[G_nps], 0, sizeof(pslot)); G_ps[G_nps].alive = 1; G_ps[G_nps].problem = problem;
    return &G_ps[G_nps++];
}
static gslot* gs_find(uint64_t gptr) { int i; for (i = 0; i < G_ng; ++i) if (G_g[i].gptr == gptr) return &G_g[i]; return NULL; }
static gslot* gs_by_problem(uint64_t pr) { int i; for (i = 0; i < G_ng; ++i) if (G_g[i].problem == pr) return &G_g[i]; return NULL; }

/* ---------------------------------------------------------------- event comparison */
static uint64_t map_real_to_c(u2u* r2c, uint64_t real, int* ok) { uint64_t c = 0; if (!u2u_get(r2c, real, &c)) *ok = 0; return c; }

static void bind(gslot* s, u2u* r2c, u2u* c2r, uint64_t real, uint64_t c, const char* what, int* ok) {
    uint64_t x;
    int have_r = u2u_get(r2c, real, &x), have_c;
    if (have_r) { if (x != c) { *ok = 0; DBG("%s: reference %llx already bound to %llx, C has %llx", what, (unsigned long long)real, (unsigned long long)x, (unsigned long long)c); } return; }
    have_c = u2u_get(c2r, c, &x);
    if (have_c) { *ok = 0; DBG("%s: C handle %llx already bound to reference %llx, event has %llx", what, (unsigned long long)c, (unsigned long long)x, (unsigned long long)real); return; }
    u2u_put(r2c, real, c); u2u_put(c2r, c, real);
    (void)s;
}
static void unbind(u2u* r2c, u2u* c2r, uint64_t real) { uint64_t c; if (u2u_get(r2c, real, &c)) { u2u_del(r2c, real); u2u_del(c2r, c); } }

static int cmp_events(gslot* s, const rev* r, int nr, const ok_vg_event* cev, int nc) {
    int i, all = 1;
    if (nr != nc) { chk(&C_events, 0); all = 0; DBG("event count: reference %d, C %d", nr, nc); }
    for (i = 0; i < nr && i < nc; ++i) {
        const rev* e = &r[i];
        const ok_vg_event* c = &cev[i];
        int ok = (e->kind == c->kind);
        if (!ok) DBG("event %d kind: reference %d, C %d", i, e->kind, c->kind);
        else switch (e->kind) {
            case OK_P_ADDPARAM: bind(s, &s->p_r2c, &s->p_c2r, e->a, c->a, "addparam", &ok); if (e->b != c->b) { ok = 0; DBG("addparam size"); } break;
            case OK_P_SETMANIFOLD: {
                uint64_t k;
                { uint64_t m = map_real_to_c(&s->p_r2c, e->a, &ok); if (m != c->a) ok = 0; }
                if (u2u_get(&s->man, e->b, &k)) { if (k != c->b) { ok = 0; DBG("manifold kind"); } } else u2u_put(&s->man, e->b, c->b);
                break;
            }
            case OK_P_ADDRESID: {
                uint32_t k;
                bind(s, &s->r_r2c, &s->r_c2r, e->a, c->a, "addresid", &ok);
                if ((e->loss != 0) != (c->loss != 0)) { ok = 0; DBG("addresid loss: reference %llx, C %d", (unsigned long long)e->loss, c->loss); }
                if (e->nb != (uint32_t)c->nb) { ok = 0; DBG("addresid nb"); }
                else for (k = 0; k < e->nb; ++k) { uint64_t m = map_real_to_c(&s->p_r2c, e->v[k], &ok); if (m != c->v[k]) { ok = 0; DBG("addresid block %u", k); } }
                break;
            }
            case OK_P_RMRESID: { uint64_t m = map_real_to_c(&s->r_r2c, e->a, &ok); if (m != c->a) ok = 0; if (!ok) DBG("rmresid identity"); unbind(&s->r_r2c, &s->r_c2r, e->a); break; }
            case OK_P_RMPARAM: { uint64_t m = map_real_to_c(&s->p_r2c, e->a, &ok); if (m != c->a) ok = 0; if (e->ndeps != 0) { ok = 0; DBG("rmparam with dependents"); } if (!ok) DBG("rmparam identity"); unbind(&s->p_r2c, &s->p_c2r, e->a); break; }
            case OK_P_SETCONST: case OK_P_SETVAR: { uint64_t m = map_real_to_c(&s->p_r2c, e->a, &ok); if (m != c->a) ok = 0; if (!ok) DBG("setconst identity"); break; }
            default: ok = 0; break;
        }
        if (!chk(&C_events, ok)) all = 0;
    }
    return all;
}

/* ---------------------------------------------------------------- dump readers */
typedef struct rec { uint32_t tag; uint64_t len; unsigned char* p; } rec;
static int rd_rec(FILE* f, rec* r, unsigned char** buf, size_t* cap) {
    if (fread(&r->tag, 4, 1, f) != 1 || fread(&r->len, 8, 1, f) != 1) return 0;
    if (r->len > (1ull << 31)) return 0;
    if (r->len > *cap) { *cap = (size_t)r->len * 2 + 4096; *buf = (unsigned char*)realloc(*buf, *cap); }
    if (r->len && fread(*buf, 1, (size_t)r->len, f) != (size_t)r->len) return 0;
    r->p = *buf;
    return 1;
}

static ok_time rd_time(cur* c) { ok_time t; t.sec = cu32(c); t.nsec = cu32(c); return t; }
static ok_imu_meas* rd_meas(cur* c, size_t* n) {
    uint64_t k, i; ok_imu_meas* m;
    k = cu64(c);
    if (k > (1u << 24)) { c->bad = 1; k = 0; }
    m = (ok_imu_meas*)malloc(sizeof(ok_imu_meas) * (size_t)(k ? k : 1));
    for (i = 0; i < k; ++i) { m[i].t = rd_time(c); cf64n(c, m[i].gyr, 3); cf64n(c, m[i].acc, 3); }
    *n = (size_t)k;
    return m;
}
static int rd_cam(cur* c, ok_cam* cam) {
    uint32_t tag, w, h, nd, i; double f[4], d[OK_CAM_MAX_DIST];
    memset(d, 0, sizeof d);
    tag = cu32(c); w = cu32(c); h = cu32(c);
    for (i = 0; i < 4; ++i) f[i] = cf64(c);
    nd = cu32(c);
    if (nd > OK_CAM_MAX_DIST) return 0;
    for (i = 0; i < nd; ++i) d[i] = cf64(c);
    if (tag != OK_CAM_RADTAN && tag != OK_CAM_EQUIDISTANT && tag != OK_CAM_NODIST) return 0;
    ok_cam_init(cam, (int)tag, (int)w, (int)h, f[0], f[1], f[2], f[3], d);
    return 1;
}
static ok_vg_kid rd_kid(cur* c) { ok_vg_kid k; k.frame = cu64(c); k.cam = cu32(c); k.kp = cu32(c); return k; }

/* ---------------------------------------------------------------- solve.bin: one solve at a time */
enum { R_SOLVE = 1, R_PROBLEM = 2, R_END = 11 };
typedef struct solve_rec { int have; uint32_t level; unsigned char* problem; size_t plen; unsigned char* end; size_t elen; } solve_rec;
static int next_solve(FILE* f, solve_rec* s) {
    uint32_t tag; uint64_t len; int got_solve = 0;
    free(s->problem); free(s->end); memset(s, 0, sizeof *s);
    if (!f) return 0;
    while (fread(&tag, 4, 1, f) == 1 && fread(&len, 8, 1, f) == 1) {
        if (tag == R_SOLVE && !got_solve) {
            unsigned char b[16];
            if (len < 16 || fread(b, 1, 16, f) != 16) return 0;
            memcpy(&s->level, b + 8, 4);
            fseek(f, (long)(len - 16), SEEK_CUR);
            got_solve = 1; s->have = 1;
        } else if (tag == R_PROBLEM || tag == R_END) {
            unsigned char* p = (unsigned char*)malloc((size_t)len ? (size_t)len : 1);
            if (len && fread(p, 1, (size_t)len, f) != (size_t)len) { free(p); return 0; }
            if (tag == R_PROBLEM) { s->problem = p; s->plen = (size_t)len; } else { s->end = p; s->elen = (size_t)len; return 1; }
        } else fseek(f, (long)len, SEEK_CUR);
    }
    return 0;
}

/* snapshot comparison: every parameter block and residual block of the Problem against the C graph */
static void cmp_snapshot(gslot* s, const solve_rec* sr) {
    cur c; uint32_t np, nr, i, k;
    uint64_t* real_ptr;
    const ok_problem* pb = ok_vg_problem(s->g);
    c.p = sr->problem; c.off = 0; c.len = sr->plen; c.bad = 0;
    np = cu32(&c);
    chk_u(&C_snap_param, "snapshot np", (uint64_t)pb->np, np);
    real_ptr = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(np ? np : 1));
    for (i = 0; i < np && !c.bad; ++i) {
        uint64_t ptr = cu64(&c), cptr = 0; uint32_t size = cu32(&c), tangent = cu32(&c), kind = cu32(&c), cst = cu32(&c);
        double x[9]; int ok = 1;
        (void)tangent;
        real_ptr[i] = ptr;
        if (size > 9) { c.bad = 1; break; }
        cf64n(&c, x, size);
        if (!u2u_get(&s->p_r2c, ptr, &cptr)) { chk(&C_snap_param, 0); DBG("snapshot param %llx unknown", (unsigned long long)ptr); continue; }
        { const ok_vg_blk* b = (const ok_vg_blk*)(uintptr_t)cptr;
          ok &= chk_u(&C_snap_param, "snap size", (uint64_t)b->size, size);
          if (b->size == (int)size) ok &= chk_vec(&C_snap_param, "snap x", b->x, x, (int)size); }
        ok &= chk_u(&C_snap_param, "snap constant", (uint64_t)ok_vg_is_constant(s->g, cptr), cst);
        ok &= chk_u(&C_snap_param, "snap kind", size == 7 ? 1u : (size == 4 ? 2u : 0u), kind);
        (void)ok;
    }
    nr = cu32(&c);
    chk_u(&C_snap_resid, "snapshot nr", (uint64_t)pb->nr, nr);
    for (i = 0; i < nr && !c.bad; ++i) {
        uint64_t rb = cu64(&c), crb = 0, plen; uint32_t type = cu32(&c), loss = cu32(&c), nb = cu32(&c), nres, bi[OK_PB_MAXB];
        ok_vg_resid d; size_t pend; int ok = 1;
        if (nb < 1 || nb > OK_PB_MAXB) { c.bad = 1; break; }
        for (k = 0; k < nb; ++k) bi[k] = cu32(&c);
        nres = cu32(&c); (void)nres;
        plen = cu64(&c);
        pend = c.off + (size_t)plen;
        if (!u2u_get(&s->r_r2c, rb, &crb) || !ok_vg_find_resid(s->g, crb, &d)) { chk(&C_snap_resid, 0); DBG("snapshot residual %llx unknown (type %u)", (unsigned long long)rb, type); c.off = pend; continue; }
        ok &= chk_u(&C_snap_resid, "snap type", (uint64_t)d.type, type);
        ok &= chk_u(&C_snap_resid, "snap loss", (uint64_t)d.loss, loss);
        ok &= chk_u(&C_snap_resid, "snap nb", (uint64_t)d.nb, nb);
        if ((uint32_t)d.nb == nb)
            for (k = 0; k < nb; ++k) {
                uint64_t m = 0;
                if (bi[k] >= np || !u2u_get(&s->p_r2c, real_ptr[bi[k]], &m)) { m = 0; }
                ok &= chk_u(&C_snap_resid, "snap block", d.blk[k], m);
            }
        if ((uint32_t)d.type == type) {
            unsigned char* mine = NULL; size_t n = 0;
            switch (type) {
                case 1: n = ok_vg_reproj_payload((const ok_reproj_err*)d.term, &mine); break;
                case 2: n = ok_vg_imu_snapshot((const ok_imu_error*)d.term, 1, 1, &mine); break;
                case 3: n = ok_vg_pose_payload((const ok_pose_err*)d.term, &mine); break;
                case 4: n = ok_vg_sab_payload((const ok_sab_err*)d.term, &mine); break;
                case 5: n = ok_vg_relpose_payload((const ok_relpose_err*)d.term, &mine); break;
                case 7: n = ok_vg_tp_payload(&((const ok_twopose*)d.term)->term, &mine); break;
                case 8: n = ok_vg_tp_payload((const ok_tp_std*)d.term, &mine); break;
                default: break;
            }
            if (mine) {
                int eq = (n == (size_t)plen) && memcmp(mine, c.p + c.off, n) == 0;
                chk(&C_snap_resid, eq);
                if (!eq) {
                    size_t j = 0;
                    if (n == (size_t)plen) while (j < n && mine[j] == c.p[c.off + j]) ++j;
                    { double g1 = 0, w1 = 0; size_t o = (j >= 4) ? 4 + ((j - 4) & ~(size_t)7) : 0; if (o + 8 <= n && n == (size_t)plen) { memcpy(&g1, mine + o, 8); memcpy(&w1, c.p + c.off + o, 8); }
                      DBG("snapshot payload type %u differs: size C %zu ref %llu, first difference at byte %zu (double %.17g vs ref %.17g)", type, n, (unsigned long long)plen, j, g1, w1); }
                }
                free(mine);
            }
        }
        c.off = pend;
    }
    free(real_ptr);
    chk(&C_snap_resid, !c.bad);
}

/* hash of all parameter blocks in the pointer order of the reference (Problem::GetParameterBlocks) */
typedef struct pr { uint64_t real; const ok_vg_blk* b; } pr;
static int cmp_pr(const void* a, const void* b) { const pr* x = (const pr*)a; const pr* y = (const pr*)b; return x->real < y->real ? -1 : (x->real > y->real); }
static uint64_t param_hash(gslot* s, int* np_out) {
    const ok_problem* pb = ok_vg_problem(s->g);
    pr* a = (pr*)malloc(sizeof(pr) * (size_t)(pb->np ? pb->np : 1));
    int i, n = 0, miss = 0;
    uint64_t h = 1469598103934665603ULL;
    for (i = 0; i < pb->np; ++i) {
        const uint64_t cptr = pb->params[pb->porder[i]].ptr;
        uint64_t real = 0;
        if (!u2u_get(&s->p_c2r, cptr, &real)) { miss++; continue; }
        a[n].real = real; a[n].b = (const ok_vg_blk*)(uintptr_t)cptr; n++;
    }
    qsort(a, (size_t)n, sizeof(pr), cmp_pr);
    for (i = 0; i < n; ++i) { h ^= ok_vg_fnv(a[i].b->x, 8 * (size_t)a[i].b->size); h *= 1099511628211ULL; }
    free(a);
    if (np_out) *np_out = n + miss;
    if (miss) h ^= 0xdeadbeefULL;
    return h;
}

/* ---------------------------------------------------------------- the main loop */
static solve_rec G_solve;
static FILE* G_solvef;
static long G_solves, G_snaps;

static void do_opt(gslot* s, cur* a, cur* r) {
    ok_vg* g = s->g;
    uint32_t i, k, ns, nb, nimu;
    ok_vg_state_info* si; ok_vg_blkref* bl; ok_vg_imuref* il;
    int nsi, nbl, nil, np_after = 0;
    uint64_t h;
    (void)cu32(a); (void)cu32(a); (void)cf64(a);
    ns = cu32(a);
    nsi = ok_vg_state_infos(g, &si);
    chk_u(&C_opt_state, "nstates", (uint64_t)nsi, ns);
    for (i = 0; i < ns && !a->bad; ++i) {
        uint64_t id = cu64(a); uint32_t kf = cu32(a); ok_time t = rd_time(a); uint32_t no = cu32(a), ntp = cu32(a), ntpc = cu32(a), nrel = cu32(a);
        if ((int)i < nsi) {
            chk_u(&C_opt_state, "state id", si[i].id, id); chk_u(&C_opt_state, "isKeyframe", (uint64_t)si[i].is_kf, kf);
            chk_u(&C_opt_state, "ts sec", si[i].ts.sec, t.sec); chk_u(&C_opt_state, "ts nsec", si[i].ts.nsec, t.nsec);
            chk_u(&C_opt_state, "nobs", (uint64_t)si[i].nobs, no); chk_u(&C_opt_state, "ntp", (uint64_t)si[i].ntp, ntp);
            chk_u(&C_opt_state, "ntpc", (uint64_t)si[i].ntpc, ntpc); chk_u(&C_opt_state, "nrel", (uint64_t)si[i].nrel, nrel);
        }
    }
    free(si);
    nb = cu32(a);
    nbl = ok_vg_blocks(g, &bl);
    chk_u(&C_opt_blocks, "nblocks", (uint64_t)nbl, nb);
    for (i = 0; i < nb && !a->bad; ++i) {
        uint64_t ptr = cu64(a), hv; uint32_t size = cu32(a), flags = cu32(a);
        uint64_t m = 0;
        hv = cu64(a);
        if (size == 4) { double q = cf64(a); uint32_t cl = cu32(a); uint64_t lid = cu64(a);
            if ((int)i < nbl) { chk_d(&C_opt_blocks, "quality", bl[i].quality, q); chk_u(&C_opt_blocks, "classification", (uint64_t)(uint32_t)bl[i].classification, cl); chk_u(&C_opt_blocks, "landmark id", bl[i].lm_id, lid); } }
        if ((int)i >= nbl) continue;
        { int ok = 1;
          bind(s, &s->p_r2c, &s->p_c2r, ptr, (uint64_t)(uintptr_t)bl[i].b, "opt block", &ok);
          if (!ok) chk(&C_opt_blocks, 0);
          (void)m; }
        chk_u(&C_opt_blocks, "block size", (uint64_t)bl[i].b->size, size);
        chk_u(&C_opt_blocks, "block flags", (uint64_t)(bl[i].b->fixed ? 1u : 0u) | (bl[i].b->initialised ? 2u : 0u), flags);
        chk_u(&C_opt_blocks, "block values (fnv)", ok_vg_fnv(bl[i].b->x, 8 * (size_t)bl[i].b->size), hv);
    }
    nimu = cu32(a);
    nil = ok_vg_imu_links(g, &il);
    chk_u(&C_opt_imu, "nimu", (uint64_t)nil, nimu);
    for (i = 0; i < nimu && !a->bad; ++i) {
        uint64_t id = cu64(a), hv = cu64(a);
        if ((int)i < nil) {
            unsigned char* snap; size_t n = ok_vg_imu_snapshot(il[i].e, 0, 1, &snap);
            chk_u(&C_opt_imu, "imu state id", il[i].state_id, id);
            if (!chk_u(&C_opt_imu, "imu snapshot (fnv)", ok_vg_fnv(snap, n), hv))
                DBG("  graph %d state %llu: n_meas %zu t0 %u.%u t1 %u.%u redo %d counter %d", (int)(s - G_g), (unsigned long long)id, il[i].e->n_meas, il[i].e->t0.sec, il[i].e->t0.nsec, il[i].e->t1.sec, il[i].e->t1.nsec, il[i].e->redo, il[i].e->redo_counter);
            free(snap);
        }
    }
    /* the PROBLEM snapshot of this solve (sampled) */
    G_solves++;
    if (G_solvef && next_solve(G_solvef, &G_solve)) {
        if (G_solve.problem) { G_snaps++; cmp_snapshot(s, &G_solve); }
    }
    /* program order check happens at the P_SOLVE record; apply what the solver did */
    {
        uint32_t nch = cu32(r);
        for (i = 0; i < nch && !r->bad; ++i) {
            uint32_t idx = cu32(r); double x[9];
            if ((int)idx >= nbl) { r->bad = 1; break; }
            cf64n(r, x, (size_t)bl[idx].b->size);
            ok_vg_blk_set(bl[idx].b, x);
        }
        nch = cu32(r);
        for (i = 0; i < nch && !r->bad; ++i) {
            uint64_t id = cu64(r); uint32_t redo = cu32(r), counter = cu32(r); double sb[9];
            cf64n(r, sb, 9);
            for (k = 0; k < (uint32_t)nil; ++k) if (il[k].state_id == id) { ok_vg_imu_apply_redo(il[k].e, (int)redo, (int)counter, sb); break; }
        }
    }
    free(bl); free(il);
    if (G_solve.end) {
        cur e; e.p = G_solve.end; e.off = 0; e.len = G_solve.elen; e.bad = 0;
        (void)cu32(&e); (void)cu32(&e); (void)cf64(&e); (void)cf64(&e); (void)cf64(&e);
        h = cu64(&e);
        chk_u(&C_end, "param hash after the solve", param_hash(s, &np_after), h);
        chk_u(&C_end, "np", (uint64_t)np_after, cu32(&e));
    }
}

static int mismatches(void) {
    return (int)(C_events.bad + C_results.bad + C_opt_state.bad + C_opt_blocks.bad + C_opt_imu.bad + C_snap_param.bad + C_snap_resid.bad + C_end.bad + C_program.bad + C_struct.bad);
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "vigraph";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max_mut = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    FILE* f;
    unsigned char* buf = NULL; size_t cap = 0;
    rec r;
    G_debug = getenv("OK_DEBUG") != NULL;
    snprintf(path, sizeof path, "%s/problem.bin", dir);
    f = fopen(path, "rb");
    if (!f) { printf("%s: 0/0\n", label); fprintf(stderr, "cannot open %s\n", path); return 1; }
    snprintf(path, sizeof path, "%s/solve.bin", dir);
    G_solvef = fopen(path, "rb");
    while (rd_rec(f, &r, &buf, &cap)) {
        cur c; c.p = r.p; c.off = 0; c.len = (size_t)r.len; c.bad = 0;
        if (r.tag >= 1 && r.tag <= 10) {                       /* ceres::Problem log */
            rev e; uint64_t pr;
            memset(&e, 0, sizeof e);
            e.kind = (int)r.tag;
            pr = cu64(&c);
            if (r.tag == OK_P_NEW) { pslot* s = ps_find(pr, 1); G_last_new_problem = pr; s->ev.n = 0; e.a = pr; revs_push(&s->ev, &e); continue; }
            if (r.tag == OK_P_DELETE) { pslot* s = ps_find(pr, 0); if (s) s->alive = 0; continue; }
            if (r.tag == OK_P_SOLVE) {
                gslot* gs = gs_by_problem(pr);
                if (gs) {                                       /* the program order at Solve(): compare right now */
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
                continue;
            }
            { pslot* s = ps_find(pr, 1);
              switch (r.tag) {
                  case OK_P_ADDPARAM: e.a = cu64(&c); e.b = cu32(&c); break;
                  case OK_P_SETMANIFOLD: e.a = cu64(&c); e.b = cu64(&c); break;
                  case OK_P_ADDRESID: { uint32_t k; e.a = cu64(&c); e.cost = cu64(&c); e.loss = cu64(&c); e.nb = cu32(&c); if (e.nb > OK_PB_MAXB) { c.bad = 1; break; } for (k = 0; k < e.nb; ++k) e.v[k] = cu64(&c); break; }
                  case OK_P_RMRESID: e.a = cu64(&c); break;
                  case OK_P_RMPARAM: e.a = cu64(&c); e.ndeps = cu32(&c); break;
                  case OK_P_SETCONST: case OK_P_SETVAR: e.a = cu64(&c); break;
                  default: break;
              }
              revs_push(&s->ev, &e); }
            continue;
        }
        if (r.tag < 32) continue;
        /* ------------------------------------------------ a graph mutation */
        {
            uint64_t gptr = cu64(&c);
            uint32_t alen = cu32(&c);
            cur a, rs;
            gslot* s;
            pslot* ps;
            const ok_vg_event* cev = NULL; int ncev = 0;
            a.p = c.p + c.off; a.off = 0; a.len = alen; a.bad = 0;
            rs.p = c.p + c.off + alen; rs.off = 0; rs.len = (size_t)r.len - c.off - alen; rs.bad = 0;
            G_mut_no++;
            if (getenv("OK_TRACE") && G_mut_no >= atol(getenv("OK_TRACE")) && G_mut_no < atol(getenv("OK_TRACE")) + 3000 && r.tag != 45 && r.tag != 41 && r.tag != 42 && r.tag != 37 && r.tag != 38 && r.tag != 46 && r.tag != 47) { uint64_t a0 = 0; if (alen >= 8) memcpy(&a0, c.p + c.off, 8); printf("TRACE %ld tag %u graph %d a0 %llu\n", G_mut_no, r.tag, gs_find(gptr) ? (int)(gs_find(gptr) - G_g) : -1, (unsigned long long)a0); }
            if (max_mut > 0 && G_mut_no > max_mut) break;
            if (r.tag < 160) C_mut[r.tag]++;
            if (r.tag == OK_M_NEW) {
                uint64_t problem = G_last_new_problem;    /* the log holds the ProblemImpl*, the graph the Problem*: claim the Problem constructed last */
                (void)cu64(&a);
                if (G_ng >= 4) continue;
                s = &G_g[G_ng++]; memset(s, 0, sizeof *s);
                s->gptr = gptr; s->problem = problem; s->g = ok_vg_new();
                ps = ps_find(problem, 0);
                chk(&C_events, ps && ps->ev.n == 1 && ps->ev.a[0].kind == OK_P_NEW);
                if (ps) ps->ev.n = 0;
                continue;
            }
            s = gs_find(gptr);
            if (!s) { chk(&C_struct, 0); DBG("mutation %u on an unknown graph", r.tag); continue; }
            ps = ps_find(s->problem, 1);
            switch (r.tag) {
                case OK_M_ADDCAM: { int d = (int)cu32(&a); double sr = cf64(&a), sa = cf64(&a); ok_vg_add_camera(s->g, d, sr, sa); break; }
                case OK_M_ADDIMU: {
                    ok_vg_imu_cfg ic; memset(&ic, 0, sizeof ic);
                    ic.use = (int)cu32(&a); cf64n(&a, ic.T_BS, 7);
                    ic.a_max = cf64(&a); ic.g_max = cf64(&a); ic.sigma_g_c = cf64(&a); ic.sigma_bg = cf64(&a); ic.sigma_a_c = cf64(&a); ic.sigma_ba = cf64(&a);
                    ic.sigma_gw_c = cf64(&a); ic.sigma_aw_c = cf64(&a); cf64n(&a, ic.g0, 3); cf64n(&a, ic.a0, 3); ic.g = cf64(&a);
                    ok_vg_add_imu(s->g, &ic);
                    break;
                }
                case OK_M_INIT: case OK_M_PROP: {
                    ok_time t = rd_time(&a); size_t n; ok_imu_meas* m = rd_meas(&a, &n);
                    uint64_t id, rid; double pose[7], sb[9], rp[7], rsb[9];
                    if (r.tag == OK_M_INIT) {
                        uint32_t nc = cu32(&a), k; double T[OK_TP_MAXEXTR][7];
                        for (k = 0; k < nc && k < OK_TP_MAXEXTR; ++k) cf64n(&a, T[k], 7);
                        id = ok_vg_add_states_initialise(s->g, t, m, n, (int)nc, (const double(*)[7])T);
                    } else {
                        uint32_t kf = cu32(&a);
                        id = ok_vg_add_states_propagate(s->g, t, m, n, (int)kf);
                    }
                    free(m);
                    rid = cu64(&rs); cf64n(&rs, rp, 7); cf64n(&rs, rsb, 9);
                    chk_u(&C_results, "new state id", id, rid);
                    ok_vg_pose_values(s->g, id, pose); ok_vg_sb_values(s->g, id, sb);
                    chk_vec(&C_results, "new pose", pose, rp, 7); chk_vec(&C_results, "new speed and bias", sb, rsb, 9);
                    break;
                }
                case OK_M_ADDLM_ID: { uint64_t id = cu64(&a); double hp[4]; int in; cf64n(&a, hp, 4); in = (int)cu32(&a); ok_vg_add_landmark_id(s->g, id, hp, in); break; }
                case OK_M_ADDLM_NEW: { double hp[4]; int in; uint64_t id, rid; cf64n(&a, hp, 4); in = (int)cu32(&a); id = ok_vg_add_landmark(s->g, hp, in); rid = cu64(&rs); chk_u(&C_results, "new landmark id", id, rid); break; }
                case OK_M_RMLM: ok_vg_remove_landmark(s->g, cu64(&a)); break;
                case OK_M_SETLM_INIT: { uint64_t id = cu64(&a); ok_vg_set_landmark_initialised(s->g, id, (int)cu32(&a)); break; }
                case OK_M_SETLMQ: { uint64_t id = cu64(&a); ok_vg_set_landmark_quality(s->g, id, cf64(&a)); break; }
                case OK_M_SETLM_FULL: { uint64_t id = cu64(&a); double hp[4]; cf64n(&a, hp, 4); ok_vg_set_landmark(s->g, id, hp, 1, (int)cu32(&a)); break; }
                case OK_M_SETLM: { uint64_t id = cu64(&a); double hp[4]; cf64n(&a, hp, 4); ok_vg_set_landmark(s->g, id, hp, 0, 0); break; }
                case OK_M_SETCLASS: { uint64_t id = cu64(&a); ok_vg_set_landmark_classification(s->g, id, (int)cu32(&a)); break; }
                case OK_M_ADDOBS: {
                    uint64_t lm = cu64(&a); ok_vg_kid kid = rd_kid(&a); int uc = (int)cu32(&a); double meas[2], size; ok_cam cam;
                    cf64n(&a, meas, 2); size = cf64(&a);
                    if (!rd_cam(&a, &cam)) { chk(&C_struct, 0); break; }
                    ok_vg_add_observation(s->g, lm, kid, uc, &cam, meas, size);
                    break;
                }
                case OK_M_ADDEXTOBS: {
                    uint64_t lm = cu64(&a); ok_vg_kid kid = rd_kid(&a); int uc = (int)cu32(&a); uint64_t src = cu64(&a);
                    ok_cam cam; double meas[2], info[4]; ok_reproj_err e, *use = &e; uint64_t cp = 0;
                    if (!rd_cam(&a, &cam)) { chk(&C_struct, 0); break; }
                    cf64n(&a, meas, 2); cf64n(&a, info, 4);
                    ok_reproj_err_init(&e, &cam, meas, info);
                    if (u2u_get(&s->convterm, src, &cp)) {              /* the source is a term the port converted back itself: use and compare it */
                        unsigned char *m1, *m2; size_t n1 = ok_vg_reproj_payload((const ok_reproj_err*)(uintptr_t)cp, &m1), n2 = ok_vg_reproj_payload(&e, &m2);
                        chk(&C_results, n1 == n2 && memcmp(m1, m2, n1) == 0);
                        free(m1); free(m2);
                        use = (ok_reproj_err*)(uintptr_t)cp;
                    }
                    ok_vg_add_external_observation(s->g, lm, kid, uc, use);
                    break;
                }
                case OK_M_RMOBS: ok_vg_remove_observation(s->g, rd_kid(&a)); break;
                case OK_M_COVIS: {
                    int was_dirty = ok_vg_covisibilities_dirty(s->g);
                    ok_vg_compute_covisibilities(s->g);
                    if (rs.len == 0) chk(&C_results, !was_dirty);
                    else {
                        uint32_t one = cu32(&rs), n = cu32(&rs), i, j, m; const uint64_t (*ab)[2]; const int* cn; int np = ok_vg_covis_pairs(s->g, &ab, &cn), pi = 0;
                        chk(&C_results, was_dirty && one == 1);
                        chk_u(&C_results, "covis outer size", (uint64_t)ok_vg_covis_size(s->g), n);
                        for (i = 0; i < n && !rs.bad; ++i) {
                            uint64_t a0 = cu64(&rs); m = cu32(&rs);
                            for (j = 0; j < m && !rs.bad; ++j) {
                                uint64_t b0 = cu64(&rs); uint32_t cc = cu32(&rs);
                                if (pi < np) { chk_u(&C_results, "covis a", ab[pi][0], a0); chk_u(&C_results, "covis b", ab[pi][1], b0); chk_u(&C_results, "covis count", (uint64_t)cn[pi], cc); }
                                pi++;
                            }
                        }
                        chk_u(&C_results, "covis pairs", (uint64_t)np, (uint64_t)pi);
                        { const uint64_t* vis; int nv = ok_vg_visible_frames(s->g, &vis); uint32_t rv = cu32(&rs);
                          chk_u(&C_results, "visible frames", (uint64_t)nv, rv);
                          for (i = 0; i < rv && !rs.bad; ++i) { uint64_t v = cu64(&rs); if ((int)i < nv) chk_u(&C_results, "visible frame", vis[i], v); } }
                    }
                    break;
                }
                case OK_M_ADDRELPOSE: { uint64_t i0 = cu64(&a), i1 = cu64(&a); double T[7], info[36]; cf64n(&a, T, 7); cf64n(&a, info, 36); ok_vg_add_relative_pose_constraint(s->g, i0, i1, T, info); break; }
                case OK_M_RMRELPOSE: { uint64_t i0 = cu64(&a), i1 = cu64(&a); ok_vg_remove_relative_pose_constraint(s->g, i0, i1); break; }
                case OK_M_SETKF: { uint64_t id = cu64(&a); ok_vg_set_keyframe(s->g, id, (int)cu32(&a)); break; }
                case OK_M_SETPOSE: { uint64_t id = cu64(&a); double T[7]; cf64n(&a, T, 7); ok_vg_set_pose(s->g, id, T); break; }
                case OK_M_SETSB: { uint64_t id = cu64(&a); double sb[9]; cf64n(&a, sb, 9); ok_vg_set_speed_and_bias(s->g, id, sb); break; }
                case OK_M_SETEXTR: { uint64_t id = cu64(&a); int cam = (int)cu32(&a); double T[7]; cf64n(&a, T, 7); ok_vg_set_extrinsics(s->g, id, cam, T); break; }
                case OK_M_UPDLM: ok_vg_update_landmarks(s->g); break;
                case OK_M_CLEANLM: { int n = ok_vg_clean_unobserved_landmarks(s->g); chk_u(&C_results, "cleaned landmarks", (uint64_t)n, cu32(&rs)); break; }
                case OK_M_RMALLOBS: ok_vg_remove_all_observations(s->g, cu64(&a)); break;
                case OK_M_ELIM: {
                    uint64_t id = cu64(&a), ref = cu64(&a), kf = 0, rkf; double T[7], v[3], rT[7], rv[3]; uint64_t rh;
                    ok_vg_eliminate_state_by_imu_merge(s->g, id, ref, &kf, T, v);
                    rkf = cu64(&rs); cf64n(&rs, rT, 7); cf64n(&rs, rv, 3); rh = cu64(&rs);
                    chk_u(&C_results, "anyState keyframeId", kf, rkf); chk_vec(&C_results, "anyState T_Sk_S", T, rT, 7); chk_vec(&C_results, "anyState v_Sk", v, rv, 3);
                    (void)rh;           /* the merged IMU term is compared through the OPT digests / snapshots that follow */
                    break;
                }
                case OK_M_MERGELM: { uint64_t from = cu64(&a), into = cu64(&a); ok_vg_merge_landmark(s->g, from, into); break; }
                case OK_M_FREEZE_POSES: { uint64_t id = cu64(&a); ok_vg_freeze_poses_until(s->g, id, (int)cu32(&a)); break; }
                case OK_M_UNFREEZE_POSES: ok_vg_unfreeze_poses_from(s->g, cu64(&a)); break;
                case OK_M_FREEZE_SB: { uint64_t id = cu64(&a); ok_vg_freeze_sb_until(s->g, id, (int)cu32(&a)); break; }
                case OK_M_UNFREEZE_SB: ok_vg_unfreeze_sb_from(s->g, cu64(&a)); break;
                case OK_M_MST: {
                    uint32_t n = cu32(&a), m, i; uint64_t *st, *co; ok_vg_mst_result res;
                    st = (uint64_t*)malloc(8 * (size_t)(n + 1));
                    for (i = 0; i < n; ++i) st[i] = cu64(&a);
                    m = cu32(&a); co = (uint64_t*)malloc(8 * (size_t)(m + 1));
                    for (i = 0; i < m; ++i) co[i] = cu64(&a);
                    ok_vg_convert_to_pose_graph_mst(s->g, st, (int)n, co, (int)m, &res);
                    free(st); free(co);
                    { uint32_t ret = cu32(&rs);
                      chk_u(&C_results, "MST return", (uint64_t)res.ret, ret);
                      if (ret == 1) {
                          uint32_t ne = cu32(&rs), nc, nt, no;
                          chk_u(&C_results, "MST edges", (uint64_t)res.nmst, ne);
                          for (i = 0; i < ne && !rs.bad; ++i) { uint64_t x = cu64(&rs), y = cu64(&rs); if ((int)i < res.nmst) { chk_u(&C_results, "MST edge a", res.mst[i][0], x); chk_u(&C_results, "MST edge b", res.mst[i][1], y); } }
                          nc = cu32(&rs); chk_u(&C_results, "created pose-graph edges", (uint64_t)res.ncreated, nc);
                          for (i = 0; i < nc && !rs.bad; ++i) { uint64_t x = cu64(&rs), y = cu64(&rs); (void)cu64(&rs); if ((int)i < res.ncreated) { chk_u(&C_results, "created ref", res.created[i][0], x); chk_u(&C_results, "created other", res.created[i][1], y); } }
                          nt = cu32(&rs); chk_u(&C_results, "removed two-pose errors", (uint64_t)res.nremoved_tp, nt);
                          for (i = 0; i < nt && !rs.bad; ++i) { uint64_t x = cu64(&rs), y = cu64(&rs); if ((int)i < res.nremoved_tp) { chk_u(&C_results, "removed tp a", res.removed_tp[i][0], x); chk_u(&C_results, "removed tp b", res.removed_tp[i][1], y); } }
                          no = cu32(&rs); chk_u(&C_results, "removed observations", (uint64_t)res.nremoved_obs, no);
                          for (i = 0; i < no && !rs.bad; ++i) { ok_vg_kid k = rd_kid(&rs); if ((int)i < res.nremoved_obs) chk(&C_results, res.removed_obs[i].frame == k.frame && res.removed_obs[i].cam == k.cam && res.removed_obs[i].kp == k.kp); }
                      } }
                    ok_vg_mst_result_free(&res);
                    break;
                }
                case OK_M_ADDEXTTP: {
                    uint64_t ref = cu64(&a), oth = cu64(&a); uint32_t present = cu32(&a); ok_tp_std src; int have = 0, k;
                    memset(&src, 0, sizeof src);
                    for (k = 0; k < G_ng && !have; ++k) if (&G_g[k] != s) have = ok_vg_clone_two_pose_const(G_g[k].g, ref, oth, &src);
                    if (present && a.len - a.off >= 4) {
                        unsigned char* mine = NULL; size_t n = ok_vg_tp_payload(&src, &mine);
                        chk(&C_results, have && n == a.len - a.off && memcmp(mine, a.p + a.off, n) == 0);
                        free(mine);
                    }
                    if (!have) { ok_tp_std dummy; memset(&dummy, 0, sizeof dummy); ok_tp_payload_read(a.p + a.off, a.len - a.off, 8, &dummy, NULL); src = dummy; }
                    ok_vg_add_external_two_pose_link(s->g, ref, oth, &src);
                    break;
                }
                case OK_M_RMTPC_ALL: ok_vg_remove_two_pose_const_links(s->g, cu64(&a)); break;
                case OK_M_RMTPC: { uint64_t i0 = cu64(&a), j0 = cu64(&a); ok_vg_remove_two_pose_const_link(s->g, i0, j0); break; }
                case OK_M_CONVOBS: {
                    uint64_t id = cu64(&a); ok_vg_conv_result res; uint32_t ctr, no, i;
                    ok_vg_convert_to_observations(s->g, id, &res);
                    ctr = cu32(&rs); chk_u(&C_results, "converted observations", (uint64_t)res.ctr, ctr);
                    no = cu32(&rs); chk_u(&C_results, "returned observations", (uint64_t)res.nobs, no);
                    for (i = 0; i < no && !rs.bad; ++i) {
                        ok_vg_kid k = rd_kid(&rs); uint64_t ptr = cu64(&rs), lm = cu64(&rs);
                        if ((int)i < res.nobs) {
                            ok_reproj_err* copy = (ok_reproj_err*)malloc(sizeof(ok_reproj_err));
                            chk(&C_results, res.kid[i].frame == k.frame && res.kid[i].cam == k.cam && res.kid[i].kp == k.kp);
                            chk_u(&C_results, "converted landmark id", res.lm[i], lm);
                            *copy = *res.err[i];
                            u2u_put(&s->convterm, ptr, (uint64_t)(uintptr_t)copy);
                        }
                    }
                    { uint32_t nl = cu32(&rs), nc2; chk_u(&C_results, "created landmarks", (uint64_t)res.nlm, nl);
                      for (i = 0; i < nl && !rs.bad; ++i) { uint64_t l = cu64(&rs); if ((int)i < res.nlm) chk_u(&C_results, "created landmark", res.lms[i], l); }
                      nc2 = cu32(&rs); chk_u(&C_results, "connected states", (uint64_t)res.nconnected, nc2);
                      for (i = 0; i < nc2 && !rs.bad; ++i) { uint64_t l = cu64(&rs); if ((int)i < res.nconnected) chk_u(&C_results, "connected state", res.connected[i], l); } }
                    ok_vg_conv_result_free(&res);
                    break;
                }
                case OK_M_RMSBPRIOR: { int ret = ok_vg_remove_speed_and_bias_prior(s->g, cu64(&a)); chk_u(&C_results, "removeSpeedAndBiasPrior", (uint64_t)ret, cu32(&rs)); break; }
                case OK_M_OPT: do_opt(s, &a, &rs); break;
                case OK_M_POKE_FIX_ADD: ok_vg_poke_fixation_add(s->g); break;
                case OK_M_POKE_FIX_REMOVE: ok_vg_poke_fixation_remove(s->g); break;
                case OK_M_POKE_LM_CONST: ok_vg_poke_landmarks_constant(s->g, (int)cu32(&a)); break;
                case OK_M_POKE_SETINFO: { ok_vg_kid k = rd_kid(&a); double info[4]; cf64n(&a, info, 4); ok_vg_poke_set_observation_information(s->g, k, info); break; }
                case OK_M_POKE_COPYSTATE: { uint64_t id = cu64(&a); double T[7], sb[9]; cf64n(&a, T, 7); cf64n(&a, sb, 9); ok_vg_poke_copy_state(s->g, id, T, sb); break; }
                case OK_M_POKE_SYNCIMU: { uint64_t src = cu64(&a), id = cu64(&a); gslot* ss = gs_find(src); if (ss) ok_vg_poke_sync_imu(s->g, ss->g, id); else chk(&C_struct, 0); break; }
                default: chk(&C_struct, 0); DBG("unhandled mutation tag %u", r.tag); break;
            }
            if (a.bad || rs.bad) { chk(&C_struct, 0); DBG("short record tag %u", r.tag); }
            ok_vg_problem_events(s->g, &cev, &ncev);
            cmp_events(s, ps->ev.a, ps->ev.n, cev, ncev);
            ps->ev.n = 0;
            ok_vg_events_clear(s->g);
        }
    }
    fclose(f);
    if (G_solvef) fclose(G_solvef);
#define PK(name, c) printf("  %-22s %ld/%ld\n", name, (c).bad, (c).tot)
    printf("  mutations replayed: %ld (solves %ld, %ld with a PROBLEM snapshot)\n", G_mut_no, G_solves, G_snaps);
    {
        int k; printf("  records per op:");
        for (k = 32; k < 160; ++k) if (C_mut[k]) printf(" %d:%ld", k, C_mut[k]);
        printf("\n");
    }
    PK("problem events", C_events); PK("mutation results", C_results); PK("optimise: states", C_opt_state); PK("optimise: blocks", C_opt_blocks);
    PK("optimise: IMU terms", C_opt_imu); PK("snapshot: parameters", C_snap_param); PK("snapshot: residuals", C_snap_resid);
    PK("program order", C_program); PK("hash after solve", C_end); PK("structure", C_struct);
    {
        long tot = C_events.tot + C_results.tot + C_opt_state.tot + C_opt_blocks.tot + C_opt_imu.tot + C_snap_param.tot + C_snap_resid.tot + C_end.tot + C_program.tot + C_struct.tot;
        printf("%s: %d/%ld\n", label, mismatches(), tot);
        free(buf);
        return mismatches() == 0 && tot > 0 ? 0 : 1;
    }
}
