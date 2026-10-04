/* OK_PORT_SOURCES: check_ok_cam.c ok_cam.c ok_kin.c ok_eigen.c */
/* Bit-exactness harness for okvis_port module 2c (pinhole camera models, NCameraSystem overlaps).
 *
 *   check_ok_cam <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_records_per_kind]
 *
 * Replays every record of <dump_dir>/kin_{proj,projj,projx,projh,projhj,projhx,back,backj,backh,backhj,overlap}.bin
 * (layouts in ok_cam.h) through the C module and compares status, success flags and all outputs bitwise.
 * Prints one line per kind "  <kind>: <mismatching values>/<compared values> (<failing records>/<records> records)"
 * and as the LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_cam.h"

static FILE* G_f;
static int G_eof;

static void rd(void* p, size_t n) {
    if (n && fread(p, 1, n, G_f) != n) G_eof = 1;
}
static uint32_t rd_u32(void) { uint32_t v = 0; rd(&v, 4); return v; }
static int32_t rd_i32(void) { int32_t v = 0; rd(&v, 4); return v; }
static void rd_f64n(double* p, size_t n) { rd(p, 8 * n); }

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

static const char* G_dir;
static long G_max;
static long G_total_bad, G_total;

#define KIND_BEGIN(name)                                                                      \
    static void check_##name(void) {                                                          \
        counts c;                                                                             \
        char path[1024];                                                                      \
        long rec = 0;                                                                         \
        memset(&c, 0, sizeof c);                                                              \
        snprintf(path, sizeof path, "%s/kin_%s.bin", G_dir, #name);                           \
        G_f = fopen(path, "rb");                                                              \
        if (!G_f) { printf("  %s: (no dump file)\n", #name); return; }                       \
        G_eof = 0;                                                                            \
        while (!G_eof && (G_max < 0 || rec < G_max)) {                                        \
            int bad = 0; ok_cam cam;
#define KIND_END(name)                                                                        \
            if (G_eof) break;                                                                 \
            rec++; c.recs++;                                                                  \
            if (bad) c.badrecs++;                                                             \
        }                                                                                     \
        fclose(G_f);                                                                          \
        printf("  %s: %ld/%ld (%ld/%ld records)\n", #name, c.bad, c.tot, c.badrecs, c.recs);  \
        G_total_bad += c.bad; G_total += c.tot;                                               \
    }

/* intrinsics Jacobian record part: u32 cols, f64[2*cols]; compare against C result computed with Ji buffer */
static int read_ji(double** want, uint32_t* cols) {
    *cols = rd_u32();
    if (*cols > 8) { G_eof = 1; *cols = 0; }
    *want = (double*)malloc(sizeof(double) * (2 * (*cols) + 1));
    rd_f64n(*want, 2 * (size_t)*cols);
    return 0;
}

KIND_BEGIN(proj) {
    double p[3], img[2] = {0, 0}, want[2] = {0, 0};
    int32_t st; uint32_t w; ok_proj_status gs;
    rd_cam(&cam); rd_f64n(p, 3); st = rd_i32(); w = rd_u32();
    if (w) rd_f64n(want, 2);
    if (G_eof) break;
    gs = ok_cam_project(&cam, p, img);
    bad += cmpi(&c, gs, st);
    bad += cmpi(&c, gs != OK_PROJ_INVALID, w);
    if (w) bad += cmpv(&c, img, want, 2);
} KIND_END(proj)

KIND_BEGIN(projj) {
    double p[3], img[2] = {0, 0}, J[6] = {0}, wi[2] = {0, 0}, wJ[6] = {0}, *Ji = NULL, *wJi = NULL;
    uint32_t has_intr, w, cols = 0; int32_t st; ok_proj_status gs;
    rd_cam(&cam); rd_f64n(p, 3); has_intr = rd_u32(); st = rd_i32(); w = rd_u32();
    if (w) { rd_f64n(wi, 2); rd_f64n(wJ, 6); if (has_intr) read_ji(&wJi, &cols); }
    if (G_eof) { free(wJi); break; }
    if (has_intr) Ji = (double*)calloc((size_t)2 * (size_t)ok_cam_num_intrinsics(&cam) + 1, sizeof(double));
    gs = ok_cam_project_j(&cam, p, img, J, Ji);
    bad += cmpi(&c, gs, st);
    if (w) {
        bad += cmpv(&c, img, wi, 2);
        bad += cmpv(&c, J, wJ, 6);
        if (has_intr) {
            bad += cmpi(&c, ok_cam_num_intrinsics(&cam), cols);
            bad += cmpv(&c, Ji, wJi, 2 * (size_t)cols);
        }
    }
    free(Ji); free(wJi);
} KIND_END(projj)

KIND_BEGIN(projx) {
    double p[3], img[2] = {0, 0}, J[6] = {0}, wi[2] = {0, 0}, wJ[6] = {0}, *Ji = NULL, *wJi = NULL, params[16];
    uint32_t np, has_pj, has_intr, w, cols = 0; int32_t st; ok_proj_status gs;
    rd_cam(&cam); rd_f64n(p, 3); np = rd_u32();
    if (np > 16) { G_eof = 1; np = 0; }
    rd_f64n(params, np);
    has_pj = rd_u32(); has_intr = rd_u32(); st = rd_i32(); w = rd_u32();
    if (w) { rd_f64n(wi, 2); if (has_pj) rd_f64n(wJ, 6); if (has_intr) read_ji(&wJi, &cols); }
    if (G_eof) { free(wJi); break; }
    if (has_intr) Ji = (double*)calloc((size_t)2 * (size_t)ok_cam_num_intrinsics(&cam) + 1, sizeof(double));
    gs = ok_cam_project_ext(&cam, p, params, img, has_pj ? J : NULL, Ji);
    bad += cmpi(&c, gs, st);
    if (w) {
        bad += cmpv(&c, img, wi, 2);
        if (has_pj) bad += cmpv(&c, J, wJ, 6);
        if (has_intr) { bad += cmpi(&c, ok_cam_num_intrinsics(&cam), cols); bad += cmpv(&c, Ji, wJi, 2 * (size_t)cols); }
    }
    free(Ji); free(wJi);
} KIND_END(projx)

KIND_BEGIN(projh) {
    double p[4], img[2] = {0, 0}, want[2] = {0, 0};
    int32_t st; uint32_t w; ok_proj_status gs;
    rd_cam(&cam); rd_f64n(p, 4); st = rd_i32(); w = rd_u32();
    if (w) rd_f64n(want, 2);
    if (G_eof) break;
    gs = ok_cam_project_h(&cam, p, img);
    bad += cmpi(&c, gs, st);
    bad += cmpi(&c, gs != OK_PROJ_INVALID, w);
    if (w) bad += cmpv(&c, img, want, 2);
} KIND_END(projh)

KIND_BEGIN(projhj) {
    double p[4], img[2] = {0, 0}, J[8] = {0}, wi[2] = {0, 0}, wJ[8] = {0}, *Ji = NULL, *wJi = NULL;
    uint32_t has_intr, w, cols = 0; int32_t st; ok_proj_status gs;
    rd_cam(&cam); rd_f64n(p, 4); has_intr = rd_u32(); st = rd_i32(); w = rd_u32();
    if (w) { rd_f64n(wi, 2); rd_f64n(wJ, 8); if (has_intr) read_ji(&wJi, &cols); }
    if (G_eof) { free(wJi); break; }
    if (has_intr) Ji = (double*)calloc((size_t)2 * (size_t)ok_cam_num_intrinsics(&cam) + 1, sizeof(double));
    gs = ok_cam_project_h_j(&cam, p, img, J, Ji);
    bad += cmpi(&c, gs, st);
    if (w) {
        bad += cmpv(&c, img, wi, 2);
        bad += cmpv(&c, J, wJ, 8);
        if (has_intr) { bad += cmpi(&c, ok_cam_num_intrinsics(&cam), cols); bad += cmpv(&c, Ji, wJi, 2 * (size_t)cols); }
    }
    free(Ji); free(wJi);
} KIND_END(projhj)

KIND_BEGIN(projhx) {
    double p[4], img[2] = {0, 0}, J[8] = {0}, wi[2] = {0, 0}, wJ[8] = {0}, *Ji = NULL, *wJi = NULL, params[16];
    uint32_t np, has_pj, has_intr, w, cols = 0; int32_t st; ok_proj_status gs;
    rd_cam(&cam); rd_f64n(p, 4); np = rd_u32();
    if (np > 16) { G_eof = 1; np = 0; }
    rd_f64n(params, np);
    has_pj = rd_u32(); has_intr = rd_u32(); st = rd_i32(); w = rd_u32();
    if (w) { rd_f64n(wi, 2); if (has_pj) rd_f64n(wJ, 8); if (has_intr) read_ji(&wJi, &cols); }
    if (G_eof) { free(wJi); break; }
    if (has_intr) Ji = (double*)calloc((size_t)2 * (size_t)ok_cam_num_intrinsics(&cam) + 1, sizeof(double));
    gs = ok_cam_project_h_ext(&cam, p, params, img, J, Ji);
    bad += cmpi(&c, gs, st);
    if (w) {
        bad += cmpv(&c, img, wi, 2);
        if (has_pj) bad += cmpv(&c, J, wJ, 8);
        if (has_intr) { bad += cmpi(&c, ok_cam_num_intrinsics(&cam), cols); bad += cmpv(&c, Ji, wJi, 2 * (size_t)cols); }
    }
    free(Ji); free(wJi);
} KIND_END(projhx)

KIND_BEGIN(back) {
    double ip[2], want[3], got[3]; uint32_t ok; int gok;
    rd_cam(&cam); rd_f64n(ip, 2); ok = rd_u32(); rd_f64n(want, 3);
    if (G_eof) break;
    gok = ok_cam_back_project(&cam, ip, got);
    bad += cmpi(&c, gok, ok);
    bad += cmpv(&c, got, want, 3);
} KIND_END(back)

KIND_BEGIN(backj) {
    double ip[2], want[3], got[3], wJ[6], J[6]; uint32_t ok; int gok;
    rd_cam(&cam); rd_f64n(ip, 2); ok = rd_u32(); rd_f64n(want, 3); rd_f64n(wJ, 6);
    if (G_eof) break;
    gok = ok_cam_back_project_j(&cam, ip, got, J);
    bad += cmpi(&c, gok, ok);
    bad += cmpv(&c, got, want, 3);
    bad += cmpv(&c, J, wJ, 6);
} KIND_END(backj)

KIND_BEGIN(backh) {
    double ip[2], want[4], got[4]; uint32_t ok; int gok;
    rd_cam(&cam); rd_f64n(ip, 2); ok = rd_u32(); rd_f64n(want, 4);
    if (G_eof) break;
    gok = ok_cam_back_project_h(&cam, ip, got);
    bad += cmpi(&c, gok, ok);
    bad += cmpv(&c, got, want, 4);
} KIND_END(backh)

KIND_BEGIN(backhj) {
    double ip[2], want[4], got[4], wJ[8], J[8]; uint32_t ok; int gok;
    rd_cam(&cam); rd_f64n(ip, 2); ok = rd_u32(); rd_f64n(want, 4); rd_f64n(wJ, 8);
    if (G_eof) break;
    gok = ok_cam_back_project_h_j(&cam, ip, got, J);
    bad += cmpi(&c, gok, ok);
    bad += cmpv(&c, got, want, 4);
    bad += cmpv(&c, J, wJ, 8);
} KIND_END(backhj)

/* NCameraSystem::computeOverlaps: every overlap mask compared byte by byte */
static void check_overlap(void) {
    counts c;
    char path[1024];
    long rec = 0;
    memset(&c, 0, sizeof c);
    snprintf(path, sizeof path, "%s/kin_overlap.bin", G_dir);
    G_f = fopen(path, "rb");
    if (!G_f) { printf("  overlap: (no dump file)\n"); return; }
    G_eof = 0;
    while (!G_eof && (G_max < 0 || rec < G_max)) {
        ok_ncam s;
        uint32_t n, a, b, i;
        int bad = 0;
        n = rd_u32();
        if (G_eof) break;
        if (n == 0 || n > OK_NCAM_MAX) { fprintf(stderr, "bad camera count %u\n", n); G_eof = 1; break; }
        ok_ncam_init(&s);
        for (i = 0; i < n; ++i) {
            ok_cam cam; ok_tf t; (void)t;
            rd_cam(&cam);
            s.cam[i] = cam;
        }
        for (i = 0; i < n; ++i) {
            double co[7], C[9];
            rd_f64n(co, 7); rd_f64n(C, 9);
            s.T_SC[i].r[0] = co[0]; s.T_SC[i].r[1] = co[1]; s.T_SC[i].r[2] = co[2];
            s.T_SC[i].q.x = co[3]; s.T_SC[i].q.y = co[4]; s.T_SC[i].q.z = co[5]; s.T_SC[i].q.w = co[6];
            memcpy(s.T_SC[i].C, C, sizeof C);
        }
        s.n = (int)n;
        if (G_eof) break;
        if (ok_ncam_compute_overlaps(&s)) { fprintf(stderr, "overlap alloc failed\n"); G_eof = 1; break; }
        for (a = 0; a < n; ++a)
            for (b = 0; b < n; ++b) {
                uint32_t has = rd_u32(), w = rd_u32(), h = rd_u32();
                size_t sz = (size_t)w * (size_t)h, k;
                uint8_t* want;
                if (G_eof || w > 4096 || h > 4096) { G_eof = 1; break; }
                want = (uint8_t*)malloc(sz ? sz : 1);
                rd(want, sz);
                if (G_eof) { free(want); break; }
                bad += cmpi(&c, s.overlaps[a][b], has);
                bad += cmpi(&c, s.cam[b].w, w);
                bad += cmpi(&c, s.cam[b].h, h);
                for (k = 0; k < sz; ++k) bad += cmpi(&c, s.mask[a][b][k], want[k]);
                free(want);
            }
        ok_ncam_free(&s);
        if (G_eof) break;
        rec++; c.recs++;
        if (bad) c.badrecs++;
    }
    fclose(G_f);
    printf("  overlap: %ld/%ld (%ld/%ld records)\n", c.bad, c.tot, c.badrecs, c.recs);
    G_total_bad += c.bad; G_total += c.tot;
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "cam";
    G_dir = argc > 3 ? argv[3] : ".";
    G_max = argc > 4 ? atol(argv[4]) : -1;
    check_proj(); check_projj(); check_projx(); check_projh(); check_projhj(); check_projhx();
    check_back(); check_backj(); check_backh(); check_backhj(); check_overlap();
    printf("%s: %ld/%ld\n", label, G_total_bad, G_total);
    return G_total_bad == 0 ? 0 : 1;
}
