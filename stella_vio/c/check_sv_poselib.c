/* SPDX-License-Identifier: MIT (project-authored test harness for sv_poselib.c)
 * check_sv_poselib <pairs.txt> [min_matches=50]  (pairs: tools/blocks/make_pairs.py, runs/blocks/poselib/pairs.txt)
 * 1. synthetic P3P (exact data), 2. relative pose / init variants (sv_init_try_monocular: seeds 4; +refine; lo; lo+refine) on real ORB pairs,
 * 3. PnP vs injected outlier fraction (sv_pnp_ransac 30 / 100 iterations vs sv_pnp_lo_ransac). Same success rules as tools/blocks/blocks_eval.cpp.
 * Build: gcc -std=c99 -O2 -ffp-contract=off -o check_sv_poselib check_sv_poselib.c <sources of sv_run.c without sv_run.c sv_system... > -lm  (see Makefile target `check`) */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "sv_init.h"
#include "sv_pnp.h"
#include "sv_poselib.h"
#include "sv_linalg.h"

typedef struct { double u1, v1, u2, v2, z1; int o1, o2; } match_t;
typedef struct { char seq[64]; int i, j, gap; double fx, fy, cx, cy, R[9] /* row-major */, t[3]; int n; match_t* m; } pair_t;

static double now_ms(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6; }
static int read_pairs(const char* path, pair_t** out) {
    FILE* f = fopen(path, "r");
    char tok[64];
    int np = 0, cap = 0;
    pair_t* a = NULL;
    if (!f) return -1;
    while (fscanf(f, "%63s", tok) == 1) {
        pair_t p;
        double g[12];
        int k;
        if (strcmp(tok, "PAIR")) break;
        if (fscanf(f, "%63s %d %d %d", p.seq, &p.i, &p.j, &p.gap) != 4) break;
        if (fscanf(f, "%*s %lf %lf %lf %lf", &p.fx, &p.fy, &p.cx, &p.cy) != 4) break;
        if (fscanf(f, "%*s") < 0) break;
        for (k = 0; k < 9; ++k) if (fscanf(f, "%lf", p.R + k) != 1) return -1;
        for (k = 0; k < 3; ++k) if (fscanf(f, "%lf", p.t + k) != 1) return -1;
        if (fscanf(f, "%*s") < 0) break;
        for (k = 0; k < 12; ++k) if (fscanf(f, "%lf", g + k) != 1) return -1;
        if (fscanf(f, "%*s %d", &p.n) != 1) return -1;
        p.m = (match_t*)malloc(sizeof(match_t) * (p.n ? p.n : 1));
        for (k = 0; k < p.n; ++k) if (fscanf(f, "%lf %lf %lf %lf %lf %d %d", &p.m[k].u1, &p.m[k].v1, &p.m[k].u2, &p.m[k].v2, &p.m[k].z1, &p.m[k].o1, &p.m[k].o2) != 7) return -1;
        if (np == cap) { cap = cap ? 2 * cap : 256; a = (pair_t*)realloc(a, sizeof(pair_t) * cap); }
        a[np++] = p;
    }
    fclose(f);
    *out = a;
    return np;
}
static double rot_err_deg(const double Re[9] /* col-major est */, const double Rg[9] /* row-major gt */) {
    double tr = 0;
    int r, c;
    for (r = 0; r < 3; ++r) for (c = 0; c < 3; ++c) tr += Re[c * 3 + r] * Rg[r * 3 + c]; /* trace(Re^T Rg) */
    tr = (tr - 1) * 0.5;
    return acos(tr > 1 ? 1 : (tr < -1 ? -1 : tr)) * 180.0 / M_PI;
}
static double ang_deg(const double a[3], const double b[3]) {
    double d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) * sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]));
    return acos(d > 1 ? 1 : (d < -1 ? -1 : d)) * 180.0 / M_PI;
}
static unsigned rs = 12345;
static double urand(void) { rs = rs * 1664525u + 1013904223u; return (rs >> 8) / 16777216.0; }

/* ---- 1. synthetic P3P ---- */
static int test_p3p(void) {
    int trial, fail = 0, nsol_tot = 0;
    for (trial = 0; trial < 2000; ++trial) {
        double w[3] = {urand() - .5, urand() - .5, urand() - .5}, Rt[9], tt[3] = {urand() - .5, urand() - .5, urand() * 2 + 1}, P[9], b[9], R[4][9], t[4][3];
        int i, k, found = 0, ns;
        double th = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) + 1e-12, K[9] = {0, w[2] / th, -w[1] / th, -w[2] / th, 0, w[0] / th, w[1] / th, -w[0] / th, 0}, K2[9];
        /* Rodrigues */
        sv_mat3_mul(K, K, K2);
        for (i = 0; i < 9; ++i) Rt[i] = (i % 4 == 0 ? 1.0 : 0.0) + sin(th) * K[i] + (1 - cos(th)) * K2[i];
        for (i = 0; i < 3; ++i) {
            double X[3] = {urand() * 4 - 2, urand() * 4 - 2, urand() * 4 + 2}, Y[3], n;
            memcpy(P + 3 * i, X, 24);
            sv_mat3_mulv(Rt, X, Y);
            Y[0] += tt[0]; Y[1] += tt[1]; Y[2] += tt[2];
            n = sqrt(Y[0] * Y[0] + Y[1] * Y[1] + Y[2] * Y[2]);
            b[3 * i] = Y[0] / n; b[3 * i + 1] = Y[1] / n; b[3 * i + 2] = Y[2] / n;
        }
        ns = sv_p3p(b, P, R, t);
        nsol_tot += ns;
        for (k = 0; k < ns; ++k) {
            double e = 0;
            for (i = 0; i < 9; ++i) e += fabs(R[k][i] - Rt[i]);
            for (i = 0; i < 3; ++i) e += fabs(t[k][i] - tt[i]);
            if (e < 1e-6) found = 1;
        }
        if (!found) ++fail;
    }
    printf("P3P synthetic: 2000 exact instances, true pose among the solutions in %d, mean #solutions %.2f\n", 2000 - fail, nsol_tot / 2000.0);
    return fail;
}

/* ---- 2. init variants ---- */
typedef struct { const char* name; unsigned seeds; int refine, lo; } init_cfg;
typedef struct { int n, acc, ok; double ms, rot[1024]; int nrot; } init_stat;
static int cmpd(const void* a, const void* b) { double x = *(const double*)a, y = *(const double*)b; return (x > y) - (x < y); }
static void run_init(const pair_t* p, const init_cfg* c, init_stat* st) {
    int n = p->n, k;
    sv_keypoint *k1 = (sv_keypoint*)malloc(sizeof(sv_keypoint) * n), *k2 = (sv_keypoint*)malloc(sizeof(sv_keypoint) * n);
    double *b1 = (double*)malloc(24 * n), *b2 = (double*)malloc(24 * n), *tp = (double*)malloc(24 * n);
    int *matched = (int*)malloc(4 * n), *m2 = (int*)malloc(4 * n);
    unsigned char *ih = (unsigned char*)malloc(n), *jf = (unsigned char*)malloc(n), *ist = (unsigned char*)calloc(n, 1);
    sv_camera_perspective cam;
    double camm[9] = {p->fx, 0, 0, 0, p->fy, 0, p->cx, p->cy, 1};
    sv_init_params ip;
    sv_init_attempt_result ar;
    double t0, tdir[3], te;
    cam.fx = p->fx; cam.fy = p->fy; cam.cx = p->cx; cam.cy = p->cy; cam.focal_x_baseline = 0; cam.min_x = 0; cam.max_x = 640; cam.min_y = 0; cam.max_y = 480;
    for (k = 0; k < n; ++k) {
        k1[k].x = p->m[k].u1; k1[k].y = p->m[k].v1; k1[k].size = 1; k1[k].angle = 0; k1[k].response = 0; k1[k].octave = p->m[k].o1;
        k2[k].x = p->m[k].u2; k2[k].y = p->m[k].v2; k2[k].size = 1; k2[k].angle = 0; k2[k].response = 0; k2[k].octave = p->m[k].o2;
        sv_camera_convert_point_to_bearing(&cam, k1[k].x, k1[k].y, b1 + 3 * k);
        sv_camera_convert_point_to_bearing(&cam, k2[k].x, k2[k].y, b2 + 3 * k);
        matched[k] = k;
    }
    memset(&ip, 0, sizeof ip);
    ip.num_ransac_iters = 100; ip.min_num_valid_pts = 50; ip.min_num_triangulated_pts = 50; ip.parallax_deg_thr = 1.0f; ip.reproj_err_thr = 4.0f;
    ip.num_seeds = c->seeds; ip.refine = c->refine; ip.lo = c->lo;
    memset(&ar, 0, sizeof ar);
    ar.matched_2_in_1 = m2; ar.inlier_h = ih; ar.inlier_f = jf; ar.triangulated_pts = tp; ar.is_triangulated = ist;
    t0 = now_ms();
    sv_init_try_monocular(k1, n, b1, k2, n, b2, matched, &cam, &cam, camm, camm, &ip, &ar);
    st->ms += now_ms() - t0;
    st->n++;
    if (ar.verdict == SV_INIT_SUCCESS) {
        double re = rot_err_deg(ar.rot_ref_to_cur, p->R), de;
        tdir[0] = p->t[0]; tdir[1] = p->t[1]; tdir[2] = p->t[2];
        te = sqrt(tdir[0] * tdir[0] + tdir[1] * tdir[1] + tdir[2] * tdir[2]);
        de = te > 1e-6 ? ang_deg(ar.trans_ref_to_cur, tdir) : 0;
        st->acc++;
        if (re < 2.0 && de < 20.0) st->ok++;
        if (st->nrot < 1024) st->rot[st->nrot++] = re;
    }
    free(k1); free(k2); free(b1); free(b2); free(tp); free(matched); free(m2); free(ih); free(jf); free(ist);
}

/* ---- 3. PnP ---- */
typedef struct { int n, ok; double ms; } pnp_stat;
static void run_pnp(const pair_t* p, double frac, int variant, unsigned idx, pnp_stat* st) {
    int k, n = 0;
    double *b = (double*)malloc(24 * p->n), *P = (double*)malloc(24 * p->n);
    int* oct = (int*)malloc(4 * p->n);
    unsigned char* mask = (unsigned char*)malloc(p->n);
    float scales[8];
    sv_pnp_result res;
    double t0;
    int rc = 0;
    sv_mt19937 rng;
    rs = 5000 + idx * 7 + (unsigned)(frac * 100);
    for (k = 0; k < p->n; ++k) {
        double u = p->m[k].u2, v = p->m[k].v2, bx, by, x1, y1, nn;
        const match_t* m = &p->m[k];
        if (!(m->z1 > 0.2 && m->z1 < 6.0)) continue;
        x1 = (m->u1 - p->cx) / p->fx; y1 = (m->v1 - p->cy) / p->fy;
        if (urand() < frac) { u = urand() * 640; v = urand() * 480; }
        bx = (u - p->cx) / p->fx; by = (v - p->cy) / p->fy; nn = sqrt(bx * bx + by * by + 1);
        b[3 * n] = bx / nn; b[3 * n + 1] = by / nn; b[3 * n + 2] = 1 / nn;
        P[3 * n] = x1 * m->z1; P[3 * n + 1] = y1 * m->z1; P[3 * n + 2] = m->z1;
        oct[n] = m->o2 < 0 ? 0 : (m->o2 > 7 ? 7 : m->o2);
        ++n;
    }
    if (n >= 20) {
        for (k = 0; k < 8; ++k) scales[k] = (float)pow(1.2, k);
        sv_mt19937_init_default(&rng);
        t0 = now_ms();
        memset(&res, 0, sizeof res);
        if (variant == 2) rc = sv_pnp_lo_ransac(b, P, oct, n, scales, 8, 10, 1000, &rng, &res, mask);
        else rc = sv_pnp_ransac(b, P, oct, n, scales, 8, 10, variant == 0 ? 30 : 100, 10, 0, NULL, &res, mask, NULL, NULL);
        st->ms += now_ms() - t0;
        st->n++;
        if (rc == 0 && res.valid) {
            double Rg_row_t[9], ce[3], cg[3], d, re = rot_err_deg(res.rotation, p->R);
            int r, c;
            for (r = 0; r < 3; ++r) for (c = 0; c < 3; ++c) Rg_row_t[c * 3 + r] = p->R[r * 3 + c]; /* gt R col-major */
            for (r = 0; r < 3; ++r) {
                ce[r] = -(res.rotation[r * 3] * res.translation[0] + res.rotation[r * 3 + 1] * res.translation[1] + res.rotation[r * 3 + 2] * res.translation[2]);
                cg[r] = -(Rg_row_t[r * 3] * p->t[0] + Rg_row_t[r * 3 + 1] * p->t[1] + Rg_row_t[r * 3 + 2] * p->t[2]);
            }
            d = sqrt((ce[0] - cg[0]) * (ce[0] - cg[0]) + (ce[1] - cg[1]) * (ce[1] - cg[1]) + (ce[2] - cg[2]) * (ce[2] - cg[2]));
            if (re < 2.0 && d < 0.05) st->ok++;
        }
    }
    free(b); free(P); free(oct); free(mask);
}

int main(int argc, char** argv) {
    pair_t* pairs = NULL;
    int np, i, v, f;
    unsigned minm = argc > 2 ? (unsigned)atoi(argv[2]) : 50;
    static init_stat ist[6];
    const init_cfg cfgs[6] = {{"stella seeds=1", 1, 0, 0}, {"stella seeds=4", 4, 0, 0}, {"seeds=4 + init_refine", 4, 1, 0}, {"seeds=1 + init_refine", 1, 1, 0}, {"init_lo", 1, 0, 1}, {"init_lo + init_refine", 1, 1, 1}};
    const double fracs[5] = {0.0, 0.3, 0.5, 0.7, 0.85};
    static pnp_stat pst[3][5];
    const char* pnames[3] = {"sv_pnp_ransac 30 it", "sv_pnp_ransac 100 it", "pnp_lo (P3P LO-RANSAC, <=1000 it)"};
    int used = 0;
    if (argc < 2) { fprintf(stderr, "usage\n"); return 2; }
    printf("P3P failures: %d\n", test_p3p());
    np = read_pairs(argv[1], &pairs);
    if (np < 0) { fprintf(stderr, "bad pairs\n"); return 2; }
    for (i = 0; i < np; ++i) {
        if ((unsigned)pairs[i].n < minm) continue;
        ++used;
        for (v = 0; v < 6; ++v) run_init(&pairs[i], &cfgs[v], &ist[v]);
        for (f = 0; f < 5; ++f) for (v = 0; v < 3; ++v) run_pnp(&pairs[i], fracs[f], v, (unsigned)i, &pst[v][f]);
    }
    printf("pairs with >= %u matches: %d\n\nINIT (accepted / success = accepted and rot<2deg and dir<20deg, of all pairs; median rot err of accepted; mean ms)\n", minm, used);
    for (v = 0; v < 6; ++v) {
        double med = -1;
        if (ist[v].nrot) { qsort(ist[v].rot, ist[v].nrot, sizeof(double), cmpd); med = ist[v].rot[ist[v].nrot / 2]; }
        printf("  %-26s accepted %5.1f%%  success %5.1f%%  median rot %.2f  %.1f ms\n", cfgs[v].name, 100.0 * ist[v].acc / ist[v].n, 100.0 * ist[v].ok / ist[v].n, med, ist[v].ms / ist[v].n);
    }
    printf("\nPNP success (rot<2deg and centre<5cm) vs injected outlier fraction 0/.3/.5/.7/.85, mean ms at 0 / .85\n");
    for (v = 0; v < 3; ++v) {
        printf("  %-34s", pnames[v]);
        for (f = 0; f < 5; ++f) printf(" %5.1f%%", 100.0 * pst[v][f].ok / (pst[v][f].n ? pst[v][f].n : 1));
        printf("   %.2f / %.2f ms\n", pst[v][0].ms / (pst[v][0].n ? pst[v][0].n : 1), pst[v][4].ms / (pst[v][4].n ? pst[v][4].n : 1));
    }
    return 0;
}
