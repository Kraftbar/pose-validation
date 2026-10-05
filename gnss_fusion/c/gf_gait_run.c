/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code.
 * gf_gait_run : command line driver for gf_gait (stdio allowed here, not in the library).
 *   gf_gait_run --imu imu.csv --epochs ep.txt [--steps st.txt] [--epoch-dt 3] [--model c] [--online --fix fixes.txt] [--window 6]
 *   imu.csv : "t_ns, wx, wy, wz, ax, ay, az" (EuRoC order, '#' comments)    fixes: "t E N sigma_h"
 *   epochs  : "t n_steps cadence state speed sigma window k odometer heading regular" every epoch-dt s of IMU time (first at t0 + epoch-dt)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "gf_gait.h"

int main(int argc, char **argv)
{
    const char *fi = NULL, *fe = NULL, *fs = NULL, *ffx = NULL; double edt = 3.0, win = 0.0, model = -1.0; int online = 0; const char *cfgs[16]; int ncfg = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--imu") && i + 1 < argc) fi = argv[++i];
        else if (!strcmp(argv[i], "--epochs") && i + 1 < argc) fe = argv[++i];
        else if (!strcmp(argv[i], "--steps") && i + 1 < argc) fs = argv[++i];
        else if (!strcmp(argv[i], "--fix") && i + 1 < argc) ffx = argv[++i];
        else if (!strcmp(argv[i], "--epoch-dt") && i + 1 < argc) edt = atof(argv[++i]);
        else if (!strcmp(argv[i], "--window") && i + 1 < argc) win = atof(argv[++i]);
        else if (!strcmp(argv[i], "--model") && i + 1 < argc) model = atof(argv[++i]);
        else if (!strcmp(argv[i], "--online")) online = 1;
        else if (!strcmp(argv[i], "--cfg") && i + 1 < argc) { if (ncfg < 16) cfgs[ncfg++] = argv[++i]; else ++i; }
        else { fprintf(stderr, "usage: see header of gf_gait_run.c\n"); return 2; }
    }
    if (!fi || !fe) { fprintf(stderr, "need --imu --epochs\n"); return 2; }
    gf_gait_config cfg; gf_gait_config_default(&cfg); cfg.online = online;
    for (int k = 0; k < ncfg; ++k) { char key[64]; double v; const char *e = strchr(cfgs[k], '='); if (!e || e - cfgs[k] > 60) { fprintf(stderr, "bad --cfg %s\n", cfgs[k]); return 2; } memcpy(key, cfgs[k], e - cfgs[k]); key[e - cfgs[k]] = 0; v = atof(e + 1); if (!gf_gait_config_set(&cfg, key, v)) { fprintf(stderr, "unknown gait key %s\n", key); return 2; } }
    gf_gait *g = gf_gait_create(&cfg); if (!g) return 1;
    if (model > 0) gf_gait_set_model(g, model);
    FILE *f = fopen(fi, "r"); if (!f) return 1;
    FILE *o = fopen(fe, "w"); FILE *os = fs ? fopen(fs, "w") : NULL; FILE *fx = ffx ? fopen(ffx, "r") : NULL;
    double fxt = 0, fxe = 0, fxn = 0, fxs = 0; int have_fx = 0;
    if (fx) { have_fx = fscanf(fx, "%lf %lf %lf %lf", &fxt, &fxe, &fxn, &fxs) == 4; }
    char line[512]; double t0 = -1, nxt = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        double v[7]; for (char *c = line; *c; ++c) if (*c == ',') *c = ' ';
        if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6) < 7) continue;
        double t = v[0] * 1e-9, w[3] = { v[1], v[2], v[3] }, a[3] = { v[4], v[5], v[6] };
        if (t0 < 0) { t0 = t; nxt = t0 + edt; }
        while (have_fx && fxt <= t) { gf_gait_gnss_fix(g, fxt, fxe, fxn, fxs); have_fx = fscanf(fx, "%lf %lf %lf %lf", &fxt, &fxe, &fxn, &fxs) == 4; }
        int st = gf_gait_push(g, t, a, w);
        if (st && os) fprintf(os, "%.9f\n", t);
        if (t >= nxt) {
            gf_gait_est e; gf_gait_estimate(g, t, win, &e);
            fprintf(o, "%.9f %d %.9f %d %.9f %.9f %.6f %.9f %.9f %.9f %d\n", t, e.n_steps, e.cadence, e.state, e.speed, e.sigma, e.window_s, gf_gait_scale(g), gf_gait_odometer(g), gf_gait_heading(g), e.regular);
            nxt += edt;
        }
    }
    fclose(f); fclose(o); if (os) fclose(os); if (fx) fclose(fx);
    gf_gait_destroy(g);
    return 0;
}
