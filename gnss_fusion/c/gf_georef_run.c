/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code.
 * gf_georef_run : driver for gf_georef (stdio allowed here).
 *
 *   gf_georef_run --stream live.txt --fix fixes.txt --out out.txt [--mode causal|batch] [key=value ...]
 *   stream : lines "t x y z qx qy qz qw [status [sigma_h]]" (the output of gf_run --out-live / --out of a fix-free run); only samples with status bit 1 (aligned) are used when a status column exists
 *   fixes  : "t E N U sigma_h ..." (gf_run format)
 *   out    : "t x y z qx qy qz qw status sigma_h" ; status 1 = geo-referenced, causal: with the fit of the moment (what a live consumer sees); batch: one fit over all fixes
 *   keys   : min_fixes min_extent scale_sigma corr_s forget_s huber_k sigma_floor use_sigma max_pairs max_gap_s   (see gf_georef.h)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gf_georef.h"

typedef struct { double t, p[3], q[4]; } samp;
typedef struct { double t, z[3], sg; } fixr;

int main(int argc, char **argv)
{
    const char *fs = NULL, *ff = NULL, *fo = NULL; int causal = 1, verbose = 0;
    gf_georef_config c; gf_georef_config_default(&c);
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--stream") && i + 1 < argc) fs = argv[++i];
        else if (!strcmp(argv[i], "--fix") && i + 1 < argc) ff = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) fo = argv[++i];
        else if (!strcmp(argv[i], "--mode") && i + 1 < argc) causal = !strcmp(argv[++i], "causal");
        else if (!strcmp(argv[i], "-v")) verbose = 1;
        else if (strchr(argv[i], '=')) {
            char key[64]; const char *eq = strchr(argv[i], '='); if (eq - argv[i] > 60) return 2;
            memcpy(key, argv[i], (size_t)(eq - argv[i])); key[eq - argv[i]] = 0; double d = atof(eq + 1);
            if (!strcmp(key, "min_fixes")) c.min_fixes = (int)d; else if (!strcmp(key, "min_extent")) c.min_extent_m = d;
            else if (!strcmp(key, "scale_sigma")) c.scale_sigma = d; else if (!strcmp(key, "corr_s")) c.corr_s = d;
            else if (!strcmp(key, "forget_s")) c.forget_s = d; else if (!strcmp(key, "huber_k")) c.huber_k = d;
            else if (!strcmp(key, "sigma_floor")) c.sigma_floor = d; else if (!strcmp(key, "use_sigma")) c.use_sigma = (int)d;
            else if (!strcmp(key, "max_pairs")) c.max_pairs = (int)d; else if (!strcmp(key, "max_gap_s")) c.max_gap_s = d;
            else { fprintf(stderr, "bad option %s\n", argv[i]); return 2; }
        } else { fprintf(stderr, "usage: see header of gf_georef_run.c\n"); return 2; }
    }
    if (!fs || !ff || !fo) { fprintf(stderr, "need --stream --fix --out\n"); return 2; }
    FILE *f = fopen(fs, "r"); if (!f) return 1;
    int cap = 1024, n = 0; samp *s = (samp *)malloc(sizeof(samp) * (size_t)cap); char line[512];
    while (fgets(line, sizeof line, f)) {
        double v[11]; int k = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6, v + 7, v + 8, v + 9);
        if (k < 8 || line[0] == '#') continue;
        if (k >= 9 && (((int)v[8]) & 1) == 0) continue;
        if (n == cap) { cap *= 2; s = (samp *)realloc(s, sizeof(samp) * (size_t)cap); }
        s[n].t = v[0]; memcpy(s[n].p, v + 1, 3 * sizeof(double)); memcpy(s[n].q, v + 4, 4 * sizeof(double)); ++n;
    }
    fclose(f);
    f = fopen(ff, "r"); if (!f) return 1;
    int fcap = 1024, nf = 0; fixr *x = (fixr *)malloc(sizeof(fixr) * (size_t)fcap);
    while (fgets(line, sizeof line, f)) {
        double v[10]; int k = sscanf(line, "%lf %lf %lf %lf %lf %lf", v, v + 1, v + 2, v + 3, v + 4, v + 5);
        if (k < 5 || line[0] == '#') continue;
        if (nf == fcap) { fcap *= 2; x = (fixr *)realloc(x, sizeof(fixr) * (size_t)fcap); }
        x[nf].t = v[0] > 1e12 ? v[0] * 1e-9 : v[0]; memcpy(x[nf].z, v + 1, 3 * sizeof(double)); x[nf].sg = v[4]; ++nf;
    }
    fclose(f);
    gf_georef *g = gf_georef_create(&c);
    FILE *o = fopen(fo, "w"); if (!g || !o) return 1;
    int fi = 0, nout = 0;
    /* causal order: sample i is added, then the fixes up to its time are paired and the fit refreshed, then sample i is mapped.
     * batch: the same pairing, but the fit is made once at the end and every sample is mapped with it. */
    if (!causal) {
        for (int i = 0; i < n; ++i) {
            gf_georef_add_pose(g, s[i].t, s[i].p);
            while (fi < nf && x[fi].t <= s[i].t) { gf_georef_add_fix(g, x[fi].t, x[fi].z, x[fi].sg, 0); ++fi; }
        }
        gf_georef_solve(g);
    }
    fi = causal ? 0 : nf;
    for (int i = 0; i < n; ++i) {
        if (causal) {
            gf_georef_add_pose(g, s[i].t, s[i].p);
            while (fi < nf && x[fi].t <= s[i].t) { gf_georef_add_fix(g, x[fi].t, x[fi].z, x[fi].sg, 1); ++fi; }
        }
        double pm[3]; double cs, sn, psi, sc;
        if (!gf_georef_map(g, s[i].p, pm)) continue;
        gf_georef_fit(g, &psi, &sc, NULL, NULL);
        cs = cos(0.5 * psi); sn = sin(0.5 * psi);   /* q_out = Rz(psi) * q */
        double qx = s[i].q[0], qy = s[i].q[1], qz = s[i].q[2], qw = s[i].q[3];
        double ox = cs * qx - sn * qy, oy = cs * qy + sn * qx, oz = cs * qz + sn * qw, ow = cs * qw - sn * qz;
        fprintf(o, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f 1 0.0\n", s[i].t, pm[0], pm[1], pm[2], ox, oy, oz, ow); ++nout;
    }
    fclose(o);
    double psi = 0, sc = 1, sr = 0; int np = 0; gf_georef_fit(g, &psi, &sc, &sr, &np);
    printf("gf_georef_run: %s, %d stream samples, %d fixes -> %d poses; final psi %.2f deg scale %.3f rms %.2f m pairs %d\n", causal ? "causal" : "batch", n, nf, nout, psi * 57.29577951308232, sc, sr, np);
    (void)verbose;
    gf_georef_destroy(g); free(s); free(x);
    return 0;
}
