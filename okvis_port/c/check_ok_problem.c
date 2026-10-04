/* OK_PORT_SOURCES: check_ok_problem.c ok_problem.c */
/* Bit-exactness harness for okvis_port module 5c (ceres::Problem bookkeeping / program order).
 *
 *   check_ok_problem <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_solves]
 *
 * Replays <dump_dir>/problem.bin (patch 0009, layout in ok_problem.h): every Problem of the run (realtime graph,
 * full graph, the frontend's quick solvers) is rebuilt mutation by mutation with ok_problem (the dependent residual
 * blocks of a RemoveParameterBlock are removed in the recorded order, so the port never has to guess the iteration
 * order of Ceres' pointer-keyed set), and at every Solve() the program order of the parameter and residual blocks is
 * compared with the recorded one (hash of the pointer sequences, the full sequences every Nth solve).
 * Prints per-kind lines and as the LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0.
 * OK_DEBUG=1 prints the first mismatches.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_problem.h"

typedef struct cur { const unsigned char* p; size_t off, len; int bad; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); else c->bad = 1; c->off += 4; return v; }
static uint64_t cu64(cur* c) { uint64_t v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }

static uint64_t fnv(const void* p, size_t n) {
    const unsigned char* c = (const unsigned char*)p;
    uint64_t h = 1469598103934665603ULL;
    size_t i;
    for (i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
    return h;
}

typedef struct slot { uint64_t ptr; ok_problem pb; int alive; } slot;
static slot G_pb[64];
static int G_npb;
static int G_debug, G_printed;
static long C_recs[16], C_bad_struct, C_solves, C_cmp, C_bad, C_deps_multi, C_deps_total, C_full;

static slot* find_pb(uint64_t ptr) {
    int i;
    for (i = 0; i < G_npb; ++i)
        if (G_pb[i].alive && G_pb[i].ptr == ptr) return &G_pb[i];
    return NULL;
}
static void structural(const char* what, uint64_t a, uint64_t b) {
    C_bad_struct++;
    if (G_debug && G_printed < 60) { G_printed++; printf("    STRUCT %s %llx %llx\n", what, (unsigned long long)a, (unsigned long long)b); }
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "problem";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max_solves = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    FILE* f;
    uint32_t tag; uint64_t len;
    unsigned char* buf = NULL; size_t cap = 0;
    uint64_t *pp = NULL, *rr = NULL; size_t capp = 0, capr = 0;
    G_debug = getenv("OK_DEBUG") != NULL;
    snprintf(path, sizeof path, "%s/problem.bin", dir);
    f = fopen(path, "rb");
    if (!f) { printf("%s: 0/0\n", label); fprintf(stderr, "cannot open %s\n", path); return 1; }
    while (fread(&tag, 4, 1, f) == 1 && fread(&len, 8, 1, f) == 1) {
        cur c;
        uint64_t ptr;
        slot* s;
        if (len > (1ull << 31)) break;
        if (len > cap) { cap = (size_t)len * 2 + 1024; buf = (unsigned char*)realloc(buf, cap); }
        if (len && fread(buf, 1, (size_t)len, f) != (size_t)len) break;
        c.p = buf; c.off = 0; c.len = (size_t)len; c.bad = 0;
        if (tag < 16) C_recs[tag]++;
        ptr = cu64(&c);
        if (tag == OK_P_NEW) {
            if (find_pb(ptr)) structural("new: already alive", ptr, 0);
            if (G_npb < 64) { s = &G_pb[G_npb++]; s->ptr = ptr; s->alive = 1; ok_problem_init(&s->pb); }
            continue;
        }
        s = find_pb(ptr);
        if (!s) { structural("unknown problem", ptr, tag); continue; }
        switch (tag) {
            case OK_P_DELETE: ok_problem_free(&s->pb); s->alive = 0; G_npb -= (s == &G_pb[G_npb - 1]) ? 1 : 0; break;
            case OK_P_ADDPARAM: {
                const uint64_t v = cu64(&c); const uint32_t size = cu32(&c);
                if (ok_problem_add_parameter_block(&s->pb, v, (int)size) < 0) structural("addparam size", v, size);
                break;
            }
            case OK_P_SETMANIFOLD: { const uint64_t v = cu64(&c), m = cu64(&c); if (!ok_problem_set_manifold(&s->pb, v, m)) structural("setmanifold unknown", v, m); break; }
            case OK_P_ADDRESID: { const uint64_t rb = cu64(&c), cost = cu64(&c), loss = cu64(&c); const uint32_t nb = cu32(&c); uint64_t vals[OK_PB_MAXB]; uint32_t k;
                if (nb > OK_PB_MAXB) { structural("addresid nb", rb, nb); break; }
                for (k = 0; k < nb; ++k) vals[k] = cu64(&c);
                if (ok_problem_add_residual_block(&s->pb, rb, cost, loss, (int)nb, vals) < 0) structural("addresid", rb, nb);
                break;
            }
            case OK_P_RMRESID: { const uint64_t rb = cu64(&c); if (!ok_problem_remove_residual_block(&s->pb, rb)) structural("rmresid unknown", rb, 0); break; }
            case OK_P_RMPARAM: { const uint64_t v = cu64(&c); const uint32_t ndeps = cu32(&c); const int left = ok_problem_remove_parameter_block(&s->pb, v);
                if (left < 0) structural("rmparam unknown", v, 0);
                else if (left != 0) structural("rmparam: dependents not logged before", v, (uint64_t)left);
                C_deps_total += ndeps; if (ndeps > 1) C_deps_multi++; break; }
            case OK_P_SETCONST: { const uint64_t v = cu64(&c); if (!ok_problem_set_constant(&s->pb, v, 1)) structural("setconst unknown", v, 0); break; }
            case OK_P_SETVAR: { const uint64_t v = cu64(&c); if (!ok_problem_set_constant(&s->pb, v, 0)) structural("setvar unknown", v, 0); break; }
            case OK_P_SOLVE: {
                const uint64_t sid = cu64(&c); const uint32_t np = cu32(&c), nr = cu32(&c); const uint64_t hp = cu64(&c), hr = cu64(&c); const uint32_t full = cu32(&c);
                int bad = 0;
                (void)sid;
                if (max_solves > 0 && C_solves >= max_solves) break;
                C_solves++;
                C_cmp += 2;
                if ((uint32_t)s->pb.np != np) { bad++; if (G_debug && G_printed < 60) { G_printed++; printf("    MISMATCH solve %llu np %d want %u\n", (unsigned long long)sid, s->pb.np, np); } }
                if ((uint32_t)s->pb.nr != nr) { bad++; if (G_debug && G_printed < 60) { G_printed++; printf("    MISMATCH solve %llu nr %d want %u\n", (unsigned long long)sid, s->pb.nr, nr); } }
                if (!bad) {
                    uint64_t gp, gr;
                    uint32_t i;
                    if (capp < np + 1) { capp = np + 1024; pp = (uint64_t*)realloc(pp, 8 * capp); }
                    if (capr < nr + 1) { capr = nr + 1024; rr = (uint64_t*)realloc(rr, 8 * capr); }
                    ok_problem_program(&s->pb, pp, rr);
                    gp = fnv(pp, 8 * (size_t)np); gr = fnv(rr, 8 * (size_t)nr);
                    C_cmp += 2;
                    if (gp != hp) { bad++; if (G_debug && G_printed < 60) { G_printed++; printf("    MISMATCH solve %llu param-order hash\n", (unsigned long long)sid); } }
                    if (gr != hr) { bad++; if (G_debug && G_printed < 60) { G_printed++; printf("    MISMATCH solve %llu residual-order hash\n", (unsigned long long)sid); } }
                    if (full) {
                        C_full++;
                        for (i = 0; i < np; ++i) { const uint64_t w = cu64(&c); C_cmp++; if (pp[i] != w) { bad++; if (G_debug && G_printed < 60) { G_printed++; printf("    MISMATCH solve %llu param[%u] %llx want %llx\n", (unsigned long long)sid, i, (unsigned long long)pp[i], (unsigned long long)w); } } }
                        for (i = 0; i < nr; ++i) { const uint64_t w = cu64(&c); C_cmp++; if (rr[i] != w) { bad++; if (G_debug && G_printed < 60) { G_printed++; printf("    MISMATCH solve %llu resid[%u] %llx want %llx\n", (unsigned long long)sid, i, (unsigned long long)rr[i], (unsigned long long)w); } } }
                    }
                }
                C_bad += bad;
                break;
            }
            default: structural("unknown tag", tag, 0); break;
        }
        if (c.bad) structural("short record", tag, len);
    }
    fclose(f);
    printf("  records: new %ld delete %ld addparam %ld setmanifold %ld addresid %ld rmresid %ld rmparam %ld (dependents removed implicitly %ld, in %ld calls with >1) setconst %ld setvar %ld\n",
           C_recs[OK_P_NEW], C_recs[OK_P_DELETE], C_recs[OK_P_ADDPARAM], C_recs[OK_P_SETMANIFOLD], C_recs[OK_P_ADDRESID], C_recs[OK_P_RMRESID],
           C_recs[OK_P_RMPARAM], C_deps_total, C_deps_multi, C_recs[OK_P_SETCONST], C_recs[OK_P_SETVAR]);
    printf("  solves: %ld compared (%ld with the full program order), %ld structural errors\n", C_solves, C_full, C_bad_struct);
    printf("%s: %ld/%ld\n", label, C_bad + C_bad_struct, C_cmp + C_bad_struct);
    free(buf); free(pp); free(rr);
    return (C_bad + C_bad_struct) == 0 && C_cmp > 0 ? 0 : 1;
}
