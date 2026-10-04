/* OK_PORT_SOURCES: check_ok_param.c ok_param.c ok_kin.c ok_eigen.c */
/* Bit-exactness harness for okvis_port module 3a (pose / homogeneous-point manifolds).
 *
 *   check_ok_param <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_records_per_kind]
 *
 * Replays <dump_dir>/err_{pplus,pplusj,pminus,pminusj,hplus,hplusj,hminus,hminusj}.bin (layouts in ok_param.h) and
 * compares bitwise. Output format as check_ok_kin.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_param.h"

static FILE* G_f;
static int G_eof;
static void rd(void* p, size_t n) { if (n && fread(p, 1, n, G_f) != n) G_eof = 1; }
static uint32_t rd_u32(void) { uint32_t v = 0; rd(&v, 4); return v; }
static void rd_f64n(double* p, size_t n) { rd(p, 8 * n); }

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
static int cmpi(counts* c, int64_t a, int64_t b) { c->tot++; if (a != b) { c->bad++; return 1; } return 0; }

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

KIND_BEGIN(pplus) {
    double x[7], d[6], w[7], g[7]; uint32_t r;
    rd_f64n(x, 7); rd_f64n(d, 6); r = rd_u32(); rd_f64n(w, 7);
    if (G_eof) break;
    bad += cmpi(&c, ok_pose_plus(x, d, g), r); bad += cmpv(&c, g, w, 7);
} KIND_END(pplus)
KIND_BEGIN(pplusj) {
    double x[7], w[42], g[42]; uint32_t r;
    rd_f64n(x, 7); r = rd_u32(); rd_f64n(w, 42);
    if (G_eof) break;
    bad += cmpi(&c, ok_pose_plus_jacobian(x, g), r); bad += cmpv(&c, g, w, 42);
} KIND_END(pplusj)
KIND_BEGIN(pminus) {
    double y[7], x[7], w[6], g[6]; uint32_t r;
    rd_f64n(y, 7); rd_f64n(x, 7); r = rd_u32(); rd_f64n(w, 6);
    if (G_eof) break;
    bad += cmpi(&c, ok_pose_minus(y, x, g), r); bad += cmpv(&c, g, w, 6);
} KIND_END(pminus)
KIND_BEGIN(pminusj) {
    double x[7], w[42], g[42]; uint32_t r;
    rd_f64n(x, 7); r = rd_u32(); rd_f64n(w, 42);
    if (G_eof) break;
    bad += cmpi(&c, ok_pose_minus_jacobian(x, g), r); bad += cmpv(&c, g, w, 42);
} KIND_END(pminusj)
KIND_BEGIN(hplus) {
    double x[4], d[3], w[4], g[4]; uint32_t r;
    rd_f64n(x, 4); rd_f64n(d, 3); r = rd_u32(); rd_f64n(w, 4);
    if (G_eof) break;
    bad += cmpi(&c, ok_hpoint_plus(x, d, g), r); bad += cmpv(&c, g, w, 4);
} KIND_END(hplus)
KIND_BEGIN(hplusj) {
    double x[4], w[12], g[12]; uint32_t r;
    rd_f64n(x, 4); r = rd_u32(); rd_f64n(w, 12);
    if (G_eof) break;
    bad += cmpi(&c, ok_hpoint_plus_jacobian(x, g), r); bad += cmpv(&c, g, w, 12);
} KIND_END(hplusj)
KIND_BEGIN(hminus) {
    double y[4], x[4], w[3], g[3]; uint32_t r;
    rd_f64n(y, 4); rd_f64n(x, 4); r = rd_u32(); rd_f64n(w, 3);
    if (G_eof) break;
    bad += cmpi(&c, ok_hpoint_minus(y, x, g), r); bad += cmpv(&c, g, w, 3);
} KIND_END(hminus)
KIND_BEGIN(hminusj) {
    double x[4], w[12], g[12]; uint32_t r;
    rd_f64n(x, 4); r = rd_u32(); rd_f64n(w, 12);
    if (G_eof) break;
    bad += cmpi(&c, ok_hpoint_minus_jacobian(x, g), r); bad += cmpv(&c, g, w, 12);
} KIND_END(hminusj)

int main(int argc, char** argv) {
    if (argc < 4) { fprintf(stderr, "usage: %s <seq_label> <fixtures|-> <dump_dir> [max]\n", argv[0]); return 2; }
    G_dir = argv[3];
    G_max = argc > 4 ? atol(argv[4]) : -1;
    check_pplus(); check_pplusj(); check_pminus(); check_pminusj();
    check_hplus(); check_hplusj(); check_hminus(); check_hminusj();
    printf("%s: %ld/%ld\n", argv[1], G_total_bad, G_total);
    return G_total_bad == 0 ? 0 : 1;
}
