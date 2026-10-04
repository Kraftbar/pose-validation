/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code.
 * gf_auto_run : driver for gf_auto (stdio / clock allowed here): the live (causal) fusion with the automatic smoother <-> georef switch, from files.
 *
 *   gf_auto_run --odom odom.txt --fix fixes.txt [--speed speed.txt] --out auto.live [--out-sm sm.live] [--out-geo geo.live] [--sig sig.txt] [--timing] [key=value ...]
 *   files as gf_run (odom "t p q [flags]", fixes "t E N U sh sv", speed "t v sigma window [flags]"); --out lines "t x y z qx qy qz qw status w_geo"
 *   keys   : plain gf_run keys apply to BOTH smoothers; a.KEY only the smoother with fixes, b.KEY only the fix-free stream smoother (stream=1);
 *            g.KEY georef (min_fixes min_extent scale_sigma corr_s forget_s huber_k sigma_floor use_sigma max_pairs max_gap_s);
 *            stream=0|1  policy=0|1|2 (smoother | georef | auto)  and the switch keys of gf_auto_cfg.h
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "gf_auto.h"
#include "gf_geo.h"
#include "gf_auto_cfg.h"

typedef struct { double t, p[3], q[4]; unsigned flags; int has_up; double up[3]; } osamp;
typedef struct { double t, v, sg, w; unsigned flags; } sspd;

static int read_speed(const char *path, sspd **out, int *n)
{
    FILE *f = fopen(path, "r"); if (!f) return -1;
    int cap = 256, m = 0; sspd *a = (sspd *)malloc(sizeof(sspd) * (size_t)cap);
    char line[256];
    while (fgets(line, sizeof line, f)) {
        char *s = line; while (*s == ' ' || *s == '\t') ++s;
        if (*s == '#' || *s == '\n' || *s == 0) continue;
        for (char *c = s; *c; ++c) if (*c == ',') *c = ' ';
        double v[5] = { 0, 0, 0, 0, 0 };
        if (sscanf(s, "%lf %lf %lf %lf %lf", v, v + 1, v + 2, v + 3, v + 4) < 4) continue;
        if (m == cap) { cap *= 2; a = (sspd *)realloc(a, sizeof(sspd) * (size_t)cap); }
        a[m].t = v[0]; a[m].v = v[1]; a[m].sg = v[2]; a[m].w = v[3]; a[m].flags = (unsigned)v[4]; ++m;
    }
    fclose(f); *out = a; *n = m; return 0;
}

static int read_odom(const char *path, osamp **out, int *n)
{
    FILE *f = fopen(path, "r"); if (!f) return -1;
    int cap = 1024, m = 0; osamp *a = (osamp *)malloc(sizeof(osamp) * (size_t)cap);
    char line[512]; int pend_up = 0; double up[3] = { 0, 0, 1 };
    while (fgets(line, sizeof line, f)) {
        char *s = line; while (*s == ' ' || *s == '\t' || *s == ',') ++s;
        if (*s == '#' || *s == '\n' || *s == 0) continue;
        if (!strncmp(s, "up", 2)) { sscanf(s + 2, "%lf %lf %lf", &up[0], &up[1], &up[2]); pend_up = 1; continue; }
        for (char *c = s; *c; ++c) if (*c == ',') *c = ' ';
        osamp o; memset(&o, 0, sizeof o); double fl = 0;
        int k = sscanf(s, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &o.t, &o.p[0], &o.p[1], &o.p[2], &o.q[0], &o.q[1], &o.q[2], &o.q[3], &fl);
        if (k < 8) continue;
        if (o.t > 1e12) o.t *= 1e-9;
        o.flags = (unsigned)fl;
        if (pend_up) { o.has_up = 1; memcpy(o.up, up, sizeof up); pend_up = 0; }
        if (m == cap) { cap *= 2; a = (osamp *)realloc(a, sizeof(osamp) * (size_t)cap); }
        a[m++] = o;
    }
    fclose(f); *out = a; *n = m; return 0;
}

static int read_fix(const char *path, int lla, gf_fix **out, int *n)
{
    FILE *f = fopen(path, "r"); if (!f) return -1;
    int cap = 1024, m = 0; gf_fix *a = (gf_fix *)malloc(sizeof(gf_fix) * (size_t)cap);
    char line[512]; gf_enu_frame fr; int have_fr = 0;
    while (fgets(line, sizeof line, f)) {
        char *s = line; while (*s == ' ' || *s == '\t') ++s;
        if (*s == '#' || *s == '\n' || *s == 0) continue;
        for (char *c = s; *c; ++c) if (*c == ',') *c = ' ';
        double v[10]; int k = sscanf(s, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6, v + 7, v + 8, v + 9);
        if (k < 6) continue;
        gf_fix g; memset(&g, 0, sizeof g);
        g.t = v[0] > 1e12 ? v[0] * 1e-9 : v[0];
        if (lla) {
            if (!have_fr) { gf_enu_frame_init(&fr, v[1], v[2], v[3]); have_fr = 1; }
            gf_lla_to_enu(&fr, v[1], v[2], v[3], g.p);
        } else { g.p[0] = v[1]; g.p[1] = v[2]; g.p[2] = v[3]; }
        g.sigma_h = v[4]; g.sigma_v = v[5];
        if (k >= 10) { g.has_vel = 1; g.v[0] = v[6]; g.v[1] = v[7]; g.v[2] = v[8]; g.sigma_vel = v[9]; }
        if (m == cap) { cap *= 2; a = (gf_fix *)realloc(a, sizeof(gf_fix) * (size_t)cap); }
        a[m++] = g;
    }
    fclose(f); *out = a; *n = m; return 0;
}

static void put(FILE *f, const gf_auto_out *o)
{
    fprintf(f, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u %.3f\n", o->t, o->p[0], o->p[1], o->p[2], o->q[0], o->q[1], o->q[2], o->q[3], o->status, o->w_geo);
}

int main(int argc, char **argv)
{
    const char *fo = NULL, *ff = NULL, *fsp = NULL, *out = NULL, *osm = NULL, *ogeo = NULL, *osig = NULL; int timing = 0;
    gf_auto_config cfg; gf_auto_config_default(&cfg);
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--odom") && i + 1 < argc) fo = argv[++i];
        else if (!strcmp(argv[i], "--fix") && i + 1 < argc) ff = argv[++i];
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc) fsp = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--out-sm") && i + 1 < argc) osm = argv[++i];
        else if (!strcmp(argv[i], "--out-geo") && i + 1 < argc) ogeo = argv[++i];
        else if (!strcmp(argv[i], "--sig") && i + 1 < argc) osig = argv[++i];
        else if (!strcmp(argv[i], "--timing")) timing = 1;
        else if (strchr(argv[i], '=')) { if (gf_auto_set(&cfg, argv[i])) { fprintf(stderr, "bad option %s\n", argv[i]); return 2; }
        } else { fprintf(stderr, "usage: see header of gf_auto_run.c\n"); return 2; }
    }
    if (!fo || !ff || !out) { fprintf(stderr, "need --odom --fix --out\n"); return 2; }
    osamp *od; int no; gf_fix *fx; int nf;
    if (read_odom(fo, &od, &no) || read_fix(ff, 0, &fx, &nf)) { fprintf(stderr, "read error\n"); return 1; }
    sspd *sp = NULL; int nsp = 0, si = 0;
    if (fsp && read_speed(fsp, &sp, &nsp)) { fprintf(stderr, "read error (speed)\n"); return 1; }
    gf_auto *a = gf_auto_create(&cfg); if (!a) return 1;
    FILE *fl = fopen(out, "w"), *fsm = osm ? fopen(osm, "w") : NULL, *fg = ogeo ? fopen(ogeo, "w") : NULL, *fs = osig ? fopen(osig, "w") : NULL;
    if (!fl) return 1;
    double *tt = (double *)malloc(sizeof(double) * (size_t)no); int ntt = 0; double tsum = 0;
    int fi = 0;
    for (int i = 0; i < no; ++i) {
        while (fi < nf && fx[fi].t <= od[i].t) {
            if (gf_auto_add_fix(a, &fx[fi])) { gf_auto_out g; if (!gf_auto_get_gnss_only(a, &g)) put(fl, &g); }
            ++fi;
        }
        while (si < nsp && sp[si].t <= od[i].t) { gf_auto_add_speed(a, sp[si].t, sp[si].v, sp[si].sg, sp[si].w, sp[si].flags); ++si; }
        struct timespec t0, t1; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t0);
        int rc = gf_auto_add_odom(a, od[i].t, od[i].p, od[i].q, od[i].flags);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t1);
        double dt = (t1.tv_sec - t0.tv_sec) * 1e6 + (t1.tv_nsec - t0.tv_nsec) * 1e-3; tt[ntt++] = dt; tsum += dt;
        if (rc) { if (rc == GF_ERR_ORDER) continue; fprintf(stderr, "gf_auto_add_odom -> %d\n", rc); return 1; }
        gf_auto_out o;
        if (gf_auto_get(a, &o) == 0) {
            put(fl, &o);
            if (fsm && o.have_sm) fprintf(fsm, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u 0\n", o.t, o.p_sm[0], o.p_sm[1], o.p_sm[2], o.q_sm[0], o.q_sm[1], o.q_sm[2], o.q_sm[3], o.status_sm);
            if (fg && o.have_geo) fprintf(fg, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f 1 0\n", o.t, o.p_geo[0], o.p_geo[1], o.p_geo[2], o.q_geo[0], o.q_geo[1], o.q_geo[2], o.q_geo[3]);
        }
        if (fs) {
            gf_auto_sig g; gf_auto_signals(a, &g);
            fprintf(fs, "%.6f %d %d %.4f %.5f %.5f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %.4f %.4f %.3f %.4f\n", g.t, g.n_pairs, g.have_fit, g.sres, g.scale, g.psi, sqrt(g.e_fast), sqrt(g.e_slow), sqrt(g.e_all),
                    g.sig_rep, g.sig_white, g.dis_fast, g.n_new_frame, g.n_gap, g.distrust, g.rho, g.w_target, sqrt(g.e_last));
        }
    }
    while (fi < nf) { gf_auto_add_fix(a, &fx[fi]); ++fi; }
    fclose(fl); if (fsm) fclose(fsm); if (fg) fclose(fg); if (fs) fclose(fs);
    printf("gf_auto_run: %d odom, %d fixes, policy %d stream %d\n", no, nf, cfg.policy, cfg.stream_mode);
    if (timing) printf("timing gf_auto_add_odom mean %.2f us (cpu), total %.3f s\n", tsum / (no ? no : 1), tsum * 1e-6);
    gf_auto_destroy(a); free(od); free(fx); free(tt);
    return 0;
}
