/* SPDX-License-Identifier: Apache-2.0 */
/* Harness for module M3 (rd_rand.c, rd_ransac.c, rd_poisson.c): replays the dump records of the reference (patch 0005) or of the oracle program
 * (rdvio_port/reference_tools/rd_m3_oracle.cc, same layouts) through the C port and compares BITWISE.
 * usage: check_rd_m3 <dir> [rnguni|rnglot|rngglibc|essgeo|homgeo|roterr|fess|frot|fhom|pess|phom|pois ...]   exit 0 iff no mismatch. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rd_rand.h"
#include "rd_ransac.h"
#include "rd_poisson.h"

static long g_cmp, g_bad;
static int g_report;
static void bad_msg(const char* what, long rec, const char* fmt, double a, double b) {
    g_bad++;
    if (g_report < 16) { g_report++; printf("  MISMATCH rec %ld %s: ", rec, what); printf(fmt, a, b); printf("\n"); }
}
static void cmp(const char* what, const double* got, const double* exp, int n, long rec) {
    int i;
    for (i = 0; i < n; ++i) {
        g_cmp++;
        if (memcmp(&got[i], &exp[i], 8)) bad_msg(what, rec, "got %.17g exp %.17g", got[i], exp[i]);
    }
}
static void cmp_u64(const char* what, uint64_t got, uint64_t exp, long rec) {
    g_cmp++;
    if (got != exp) bad_msg(what, rec, "got %.0f exp %.0f", (double)got, (double)exp);
}
static int rd(FILE* f, void* p, size_t n) { return fread(p, 1, n, f) == n; }
#define RD(p, n) do { if (!rd(f, (p), (n))) goto done; } while (0)
static FILE* open_kind(const char* dir, const char* name) { char path[512]; snprintf(path, sizeof path, "%s/%s.bin", dir, name); return fopen(path, "rb"); }
#define REPORT(name) printf("%s: %ld records, %ld values compared, %ld mismatches\n", name, rec, g_cmp - c0, g_bad - b0)

static void check_rnguni(const char* dir) {
    FILE* f = open_kind(dir, "rnguni"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t seed, n, i; rd_minstd e;
        RD(&seed, 4); RD(&n, 4);
        rd_minstd_seed(&e, seed);
        for (i = 0; i < n; ++i) {
            uint64_t a, b, r;
            if (!rd(f, &a, 8) || !rd(f, &b, 8) || !rd(f, &r, 8)) goto done;
            cmp_u64("uniform", rd_uniform_int(&e, a, b), r, rec);
        }
        rec++;
    }
done:
    REPORT("rnguni");
    fclose(f);
}

static void check_rnglot(const char* dir) {
    FILE* f = open_kind(dir, "rnglot"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t size, seed, n, i; rd_lotbox lb;
        RD(&size, 4); RD(&seed, 4); RD(&n, 4);
        rd_lotbox_init(&lb, size); rd_lotbox_seed(&lb, seed);
        for (i = 0; i < n; ++i) {
            uint32_t op; uint64_t r;
            if (!rd(f, &op, 4) || !rd(f, &r, 8)) { rd_lotbox_free(&lb); goto done; }
            if (op) rd_lotbox_refill_all(&lb);
            else cmp_u64("lotbox", (uint64_t)rd_lotbox_draw_without_replacement(&lb), r, rec);
        }
        rd_lotbox_free(&lb);
        rec++;
    }
done:
    REPORT("rnglot");
    fclose(f);
}

static void check_glibc(const char* dir) {
    FILE* f = open_kind(dir, "rngglibc"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    if (!f) return;
    for (;;) {
        uint32_t seed; int i;
        RD(&seed, 4);
        rd_glibc_srand(seed);
        for (i = 0; i < 2000; ++i) {
            int32_t v;
            if (!rd(f, &v, 4)) goto done;
            cmp_u64("rand", (uint64_t)rd_glibc_rand(), (uint64_t)v, rec);
        }
        rec++;
    }
done:
    REPORT("rngglibc");
    fclose(f);
}

static void check_err(const char* dir, const char* kind) {
    FILE* f = open_kind(dir, kind); long rec = 0, b0 = g_bad, c0 = g_cmp;
    int is_rot = !strcmp(kind, "roterr");
    if (!f) return;
    for (;;) {
        double M[9], a[3], b[3], e, g;
        RD(M, 72);
        if (is_rot) { RD(a, 24); RD(b, 24); } else { RD(a, 16); RD(b, 16); }
        RD(&e, 8);
        g = !strcmp(kind, "essgeo") ? rd_essential_geometric_error(M, a, b) : is_rot ? rd_rotation_error(M, a, b) : rd_homography_geometric_error(M, a, b);
        cmp(kind, &g, &e, 1, rec); rec++;
    }
done:
    REPORT(kind);
    fclose(f);
}

typedef enum { K_ESS, K_ROT, K_HOM, K_PESS, K_PHOM } find_kind;
static void check_find(const char* dir, const char* name, find_kind kind) {
    FILE* f = open_kind(dir, name); long rec = 0, b0 = g_bad, c0 = g_cmp, nmodel = 0;
    rd_parsac_state st;
    const int d = (kind == K_ROT) ? 3 : 2;
    if (!f) return;
    rd_parsac_state_init(&st);
    for (;;) {
        uint32_t n, mlen; double thr, conf, M[9], gM[9]; uint64_t maxit; int32_t seed; double *p1, *p2; char *emask, *gmask; size_t cnt, k, nexp = 0;
        RD(&n, 4); RD(&thr, 8); RD(&conf, 8); RD(&maxit, 8); RD(&seed, 4);
        if (n > 100000) goto done;
        p1 = (double*)malloc(sizeof(double) * d * (n ? n : 1)); p2 = (double*)malloc(sizeof(double) * d * (n ? n : 1));
        emask = (char*)malloc(n + 1); gmask = (char*)malloc(n + 1);
        if (!rd(f, p1, 8 * d * (size_t)n) || !rd(f, p2, 8 * d * (size_t)n) || !rd(f, M, 72) || !rd(f, &mlen, 4) || mlen > n || !rd(f, emask, mlen)) { free(p1); free(p2); free(emask); free(gmask); goto done; }
        switch (kind) {
        case K_ESS: cnt = rd_find_essential_matrix(n, p1, p2, gmask, thr, conf, maxit, seed, gM); break;
        case K_ROT: cnt = rd_find_rotation_matrix(n, p1, p2, gmask, thr, conf, maxit, seed, gM); break;
        case K_HOM: cnt = rd_find_homography_matrix(n, p1, p2, gmask, thr, conf, maxit, seed, gM); break;
        case K_PESS: cnt = rd_find_essential_matrix_parsac(&st, n, p1, p2, gmask, thr, conf, maxit, seed, gM); break;
        default: cnt = rd_find_homography_matrix_parsac(&st, n, p1, p2, gmask, thr, conf, maxit, seed, gM); break;
        }
        for (k = 0; k < mlen; ++k) nexp += emask[k] != 0;
        if (cnt == (size_t)-1) { bad_msg("out of range point", rec, "(%g %g)", 0, 0); }
        else {
            cmp_u64("inlier count", cnt, nexp, rec);
            if (mlen == n) { g_cmp += n; for (k = 0; k < n; ++k) if (emask[k] != gmask[k]) { bad_msg("mask", rec, "index %.0f (%.0f)", (double)k, (double)gmask[k]); break; } }
            else if (cnt != 0) bad_msg("mask length", rec, "exp %.0f", (double)mlen, 0);
            if (cnt) { nmodel++; cmp("model", gM, M, 9, rec); }
        }
        free(p1); free(p2); free(emask); free(gmask);
        rec++;
    }
done:
    REPORT(name); printf("  (records with a model: %ld)\n", nmodel);
    fclose(f);
}

static void check_pois(const char* dir) {
    FILE* f = open_kind(dir, "pois"); long rec = 0, b0 = g_bad, c0 = g_cmp;
    rd_poisson live[64]; int alive[64]; int i;
    if (!f) return;
    memset(alive, 0, sizeof alive);
    for (;;) {
        uint32_t op, res, n, kept, k; uint64_t id; double p[2], radius;
        RD(&op, 4); RD(&id, 8);
        i = (int)(id % 64);
        switch (op) {
        case 0: RD(&radius, 8); if (alive[i]) rd_poisson_free(&live[i]); rd_poisson_init(&live[i], radius); alive[i] = 1; break;
        case 1: RD(p, 16); rd_poisson_preset_point(&live[i], p); break;
        case 2: RD(p, 16); RD(&res, 4); cmp_u64("permit", (uint64_t)rd_poisson_permit_point(&live[i], p), res, rec); break;
        case 3: RD(p, 16); RD(&res, 4); cmp_u64("insert", (uint64_t)rd_poisson_insert_point(&live[i], p), res, rec); break;
        case 4: {
            double *cand, *ex; size_t got;
            RD(&n, 4);
            cand = (double*)malloc(16 * (size_t)(n + 1));
            if (!rd(f, cand, 16 * (size_t)n) || !rd(f, &kept, 4)) { free(cand); goto done; }
            ex = (double*)malloc(16 * (size_t)(kept + 1));
            if (!rd(f, ex, 16 * (size_t)kept)) { free(cand); free(ex); goto done; }
            got = rd_poisson_insert_points(&live[i], cand, n);
            cmp_u64("kept", got, kept, rec);
            if (got == kept) for (k = 0; k < kept; ++k) cmp("kept point", cand + 2 * k, ex + 2 * k, 2, rec);
            free(cand); free(ex);
            break; }
        case 5: rd_poisson_clear(&live[i]); break;
        case 6: if (alive[i]) { rd_poisson_free(&live[i]); alive[i] = 0; } rec++; break;
        default: goto done;
        }
    }
done:
    REPORT("pois");
    fclose(f);
}

int main(int argc, char** argv) {
    int i; const char* dir;
    if (argc < 2) { fprintf(stderr, "usage: check_rd_m3 <dir> [kinds]\n"); return 2; }
    dir = argv[1];
    {
        int all = argc == 2;
        for (i = 2; i < argc || all; ++i) {
            const char* k = all ? "all" : argv[i];
            if (all || !strcmp(k, "rnguni")) check_rnguni(dir);
            if (all || !strcmp(k, "rnglot")) check_rnglot(dir);
            if (all || !strcmp(k, "rngglibc")) check_glibc(dir);
            if (all || !strcmp(k, "essgeo")) check_err(dir, "essgeo");
            if (all || !strcmp(k, "homgeo")) check_err(dir, "homgeo");
            if (all || !strcmp(k, "roterr")) check_err(dir, "roterr");
            if (all || !strcmp(k, "fess")) check_find(dir, "fess", K_ESS);
            if (all || !strcmp(k, "frot")) check_find(dir, "frot", K_ROT);
            if (all || !strcmp(k, "fhom")) check_find(dir, "fhom", K_HOM);
            if (all || !strcmp(k, "pess")) check_find(dir, "pess", K_PESS);
            if (all || !strcmp(k, "phom")) check_find(dir, "phom", K_PHOM);
            if (all || !strcmp(k, "pois")) check_pois(dir);
            if (all) break;
        }
    }
    printf("TOTAL: %ld values compared, %ld mismatches\n", g_cmp, g_bad);
    return g_bad ? 1 : 0;
}
