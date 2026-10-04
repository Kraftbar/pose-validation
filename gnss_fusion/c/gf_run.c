/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code.
 * gf_run : command line driver for the gf_* fusion library (stdio / clock allowed here, not in the library).
 *
 *   gf_run --odom odom.txt --fix fixes.txt --out fused.tum [--mode batch|causal] [key=value ...]
 *
 *   odom.txt : lines "t px py pz qx qy qz qw [flags]" (flags: 1 new frame, 2 gap, 4 loose link; loose_k=); a line "up x y z" sets the gravity direction
 *              of the next frame; '#' comments; t in s (or ns if > 1e12)
 *   fixes    : lines "t E N U sigma_h sigma_v [vx vy vz sigma_vel]"   (--fix-lla: "t lat lon h sigma_h sigma_v", first fix = ENU origin)
 *   --speed F      gait speed measurements "t v sigma window [flags]" (mean speed over [t-window, t]; flags 1 = stationary); needs speed=1
 *   --out F        per-odometry-sample poses "t x y z qx qy qz qw" from gf_query() after the run (batch: full smoother; causal:
 *                  retroactive estimate of every node at the time it was created, identical to the python causal output)
 *   --out-live F   (causal) pose returned by gf_get_pose() right after each sample, as an online consumer sees it: "t x y z qx qy qz qw status sigma_h"
 *   --lookahead L  feed fixes up to L s before they are due (python-equivalence test; real use: 0)
 *   --geo lat0,lon0,h0   filter mode: stdin lines "lat lon h" -> "E N U lat' lon' h'" (LLA -> ENU -> LLA round trip, for the validation script)
 *   --no-history   causal: bounded memory (nodes older than the window are dropped; --out then only holds the retained tail)
 *   --timing       print per-call latency statistics
 *   --nodes F      dump nodes "t psi px py pz s distrusted fix_used"
 *   config keys    node_dt window_s batch_iters causal_iters init_iters settle_nodes init_wait_s odom_sp odom_kp yaw_rw_deg scale_rw scale_prior
 *                  metric gravity_aligned rsa=x,y,z loss(0/1/2) loss_k min_sigma gate_chi2 robust_init gap_s max_speed link_speed
 *                  seg_min_extent trust trust_window_s trust_min_fixes trust_scale_k trust_rho_k trust_rho_min trust_spread_k trust_q_scale
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "gf_fusion.h"
#include "gf_geo.h"

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

static double now_us(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e6 + ts.tv_nsec * 1e-3;
}

static int cmpd(const void *a, const void *b) { double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y); }

static void report(const char *name, double *v, int n)
{
    if (n == 0) { printf("timing %-22s n=0\n", name); return; }
    qsort(v, (size_t)n, sizeof(double), cmpd);
    double s = 0; for (int i = 0; i < n; ++i) s += v[i];
    printf("timing %-22s n=%d mean=%.2f us p50=%.2f p99=%.2f max=%.2f\n", name, n, s / n, v[n / 2], v[(int)(0.99 * (n - 1))], v[n - 1]);
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

static int cfg_kv(gf_config *c, const char *kv)
{
    char key[64]; const char *eq = strchr(kv, '='); if (!eq || eq - kv > 60) return -1;
    memcpy(key, kv, (size_t)(eq - kv)); key[eq - kv] = 0;
    const char *v = eq + 1; double d = atof(v);
#define D(name, field) if (!strcmp(key, name)) { c->field = d; return 0; }
    D("node_dt", node_dt) D("window_s", window_s) D("batch_iters", batch_iters) D("causal_iters", causal_iters) D("init_iters", init_iters)
    D("settle_nodes", settle_nodes) D("init_wait_s", init_wait_s) D("odom_sp", odom_sp) D("odom_kp", odom_kp) D("scale_rw", scale_rw)
    D("scale_prior", scale_prior) D("scale_rw_mono", scale_rw_mono) D("scale_prior_mono", scale_prior_mono) D("metric", metric_scale) D("gravity_aligned", gravity_aligned) D("loss", loss) D("loss_k", loss_k)
    D("min_sigma", min_sigma) D("gate_chi2", gate_chi2) D("gate_floor", gate_floor) D("gate_min_fixes", gate_min_fixes) D("robust_init", robust_init) D("gap_s", gap_s) D("max_speed", max_speed)
    D("link_speed", link_speed) D("seg_min_extent", seg_min_extent) D("trust", trust) D("trust_window_s", trust_window_s)
    D("trust_min_fixes", trust_min_fixes) D("trust_long_s", trust_long_s) D("trust_long_and", trust_long_and) D("trust_rho_long", trust_rho_long) D("trust_scale_k", trust_scale_k) D("trust_scale_k_mono", trust_scale_k_mono) D("trust_rho_k", trust_rho_k) D("trust_rho_min", trust_rho_min)
    D("trust_spread_k", trust_spread_k) D("trust_q_scale", trust_q_scale) D("drift_rate", drift_rate) D("blackout_s", blackout_s)
    D("speed", speed_on) D("speed_k", speed_k) D("speed_sigma_scale", speed_sigma_scale) D("speed_link_sigma", speed_link_sigma) D("zupt_sigma", zupt_sigma) D("speed_align", speed_align) D("speed_scale_rw_rel", speed_scale_rw_rel) D("speed_scale_lim", speed_scale_lim) D("speed_scale_rw", speed_scale_rw) D("loose_k", loose_k) D("speed_align_metric", speed_align_metric)
    D("keep_history", keep_history) D("trust_start_m", trust_start_m) D("gnss_only_nodes", gnss_only_nodes) D("grow_s", grow_s) D("grow_s_mono", grow_s_mono) D("grow_ratio", grow_ratio) D("trust_state_k", trust_state_k) D("scale_min", scale_min) D("scale_max", scale_max)
#undef D
    if (!strcmp(key, "preset")) {
        if (!strcmp(v, "robust")) { gf_config_robust(c); return 0; }
        if (!strcmp(v, "robust1")) {   /* the first version of the robust preset (section 10 of the study), without the section-11 features */
            gf_config_robust(c);
            c->trust_start_m = 0.0; c->trust_state_k = 0.0; c->grow_s = 0.0; c->scale_min = 0.0; c->scale_max = 0.0; c->gnss_only_nodes = 0;
            return 0;
        }
        return -1;
    }
    if (!strcmp(key, "yaw_rw_deg")) { c->yaw_rw = d * 3.14159265358979323846 / 180.0; return 0; }
    if (!strcmp(key, "rsa")) { sscanf(v, "%lf,%lf,%lf", &c->rsa[0], &c->rsa[1], &c->rsa[2]); return 0; }
    return -1;
}

int main(int argc, char **argv)
{
    const char *fsp = NULL; const char *fo = NULL, *ff = NULL, *out = NULL, *outlive = NULL, *nodes = NULL; int lla = 0, causal = 0, timing = 0;
    double lookahead = 0.0; const char *geo = NULL; int nohist = 0;
    gf_config cfg; gf_config_default(&cfg);
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--odom") && i + 1 < argc) fo = argv[++i];
        else if (!strcmp(argv[i], "--fix") && i + 1 < argc) ff = argv[++i];
        else if (!strcmp(argv[i], "--fix-lla") && i + 1 < argc) { ff = argv[++i]; lla = 1; }
        else if (!strcmp(argv[i], "--speed") && i + 1 < argc) fsp = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--out-live") && i + 1 < argc) outlive = argv[++i];
        else if (!strcmp(argv[i], "--nodes") && i + 1 < argc) nodes = argv[++i];
        else if (!strcmp(argv[i], "--mode") && i + 1 < argc) causal = !strcmp(argv[++i], "causal");
        else if (!strcmp(argv[i], "--lookahead") && i + 1 < argc) lookahead = atof(argv[++i]);
        else if (!strcmp(argv[i], "--timing")) timing = 1;
        else if (!strcmp(argv[i], "--no-history")) nohist = 1;
        else if (!strcmp(argv[i], "--geo") && i + 1 < argc) geo = argv[++i];
        else if (strchr(argv[i], '=')) { if (cfg_kv(&cfg, argv[i])) { fprintf(stderr, "bad option %s\n", argv[i]); return 2; } }
        else { fprintf(stderr, "usage: see header of gf_run.c\n"); return 2; }
    }
    if (geo) {
        double la, lo, h; gf_enu_frame fr; sscanf(geo, "%lf,%lf,%lf", &la, &lo, &h); gf_enu_frame_init(&fr, la, lo, h);
        while (scanf("%lf %lf %lf", &la, &lo, &h) == 3) {
            double e[3], a, b, c, ec[3]; gf_lla_to_enu(&fr, la, lo, h, e); gf_enu_to_lla(&fr, e, &a, &b, &c); gf_lla_to_ecef(la, lo, h, ec);
            printf("%.6f %.6f %.6f %.12f %.12f %.6f %.4f %.4f %.4f\n", e[0], e[1], e[2], a, b, c, ec[0], ec[1], ec[2]);
        }
        return 0;
    }
    if (!fo || !ff || !out) { fprintf(stderr, "need --odom --fix --out\n"); return 2; }
    osamp *od; int no; gf_fix *fx; int nf;
    if (read_odom(fo, &od, &no) || read_fix(ff, lla, &fx, &nf)) { fprintf(stderr, "read error\n"); return 1; }
    if (no < 2) { fprintf(stderr, "no odometry\n"); return 1; }
    sspd *sp = NULL; int nsp = 0, si = 0;
    if (fsp && read_speed(fsp, &sp, &nsp)) { fprintf(stderr, "read error (speed)\n"); return 1; }
    if (causal && !cfg.keep_history && !nohist) cfg.keep_history = 1;   /* needed for the retroactive --out; a product would not */
    gf_t *g = gf_create(&cfg, causal);
    if (!g) return 1;
    double *tn = (double *)malloc(sizeof(double) * (size_t)no), *tc = (double *)malloc(sizeof(double) * (size_t)no), *tf = (double *)malloc(sizeof(double) * (size_t)(nf + 1));
    int ntn = 0, ntc = 0, ntf = 0; int fi = 0;
    FILE *fl = outlive ? fopen(outlive, "w") : NULL;
    for (int i = 0; i < no; ++i) {
        while (fi < nf && fx[fi].t <= od[i].t + lookahead) {
            gf_stats sg0; gf_get_stats(g, &sg0);
            double a = now_us(); gf_add_fix(g, &fx[fi]); tf[ntf++] = now_us() - a; ++fi;
            gf_stats sg1; gf_get_stats(g, &sg1);
            if (fl && sg1.n_gnss_nodes != sg0.n_gnss_nodes) {   /* odometry lost: the fix made a GNSS-only node, report the live pose */
                gf_pose p;
                if (gf_get_pose(g, &p) == 0 && (p.status & GF_ST_NO_ODOM))
                    fprintf(fl, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u %.3f\n", p.t, p.p[0], p.p[1], p.p[2], p.q[0], p.q[1], p.q[2], p.q[3], p.status, p.sigma_h);
            }
        }
        while (si < nsp && sp[si].t <= od[i].t + lookahead) { gf_add_speed(g, sp[si].t, sp[si].v, sp[si].sg, sp[si].w, sp[si].flags); ++si; }
        if (od[i].has_up) gf_set_gravity(g, od[i].up);
        gf_stats s0; gf_get_stats(g, &s0);
        double a = now_us();
        int rc = gf_add_odom(g, od[i].t, od[i].p, od[i].q, od[i].flags);
        double dt = now_us() - a;
        if (rc) { fprintf(stderr, "gf_add_odom(%d) -> %d at t=%.3f\n", i, rc, od[i].t); if (rc == GF_ERR_ORDER) continue; return 1; }
        gf_stats s1; gf_get_stats(g, &s1);
        if (s1.n_nodes != s0.n_nodes) tn[ntn++] = dt; else tc[ntc++] = dt;
        if (fl) {
            gf_pose p;
            if (gf_get_pose(g, &p) == 0)
                fprintf(fl, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u %.3f\n", od[i].t, p.p[0], p.p[1], p.p[2], p.q[0], p.q[1], p.q[2], p.q[3], p.status, p.sigma_h);
        }
    }
    while (fi < nf) {
        gf_stats sg0; gf_get_stats(g, &sg0);
        gf_add_fix(g, &fx[fi]); ++fi;
        gf_stats sg1; gf_get_stats(g, &sg1);
        if (fl && sg1.n_gnss_nodes != sg0.n_gnss_nodes) {
            gf_pose p;
            if (gf_get_pose(g, &p) == 0 && (p.status & GF_ST_NO_ODOM))
                fprintf(fl, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u %.3f\n", p.t, p.p[0], p.p[1], p.p[2], p.q[0], p.q[1], p.q[2], p.q[3], p.status, p.sigma_h);
        }
    }
    while (si < nsp) { gf_add_speed(g, sp[si].t, sp[si].v, sp[si].sg, sp[si].w, sp[si].flags); ++si; }
    if (fl) fclose(fl);
    if (!causal) {
        double a = now_us(); int rc = gf_solve_batch(g); double dt = now_us() - a;
        if (rc) { fprintf(stderr, "gf_solve_batch -> %d\n", rc); return 1; }
        if (timing) { gf_stats sb; gf_get_stats(g, &sb); printf("timing batch solve (%ld nodes)   %.1f ms\n", sb.n_nodes, dt * 1e-3); }
    }
    FILE *fo2 = fopen(out, "w"); if (!fo2) return 1;
    int nout = 0, ng = 0;
    gf_stats sg; gf_get_stats(g, &sg);
    /* GNSS-only nodes (odometry lost): poses merged into the output by time */
    int gi = 0, gn_total = 0; gf_pose gp;
    while (gf_node_pose(g, gn_total, &gp) >= 0) ++gn_total;
    for (int i = 0; i <= no; ++i) {
        double tnext = i < no ? od[i].t : 1e300;
        while (gi < gn_total) {
            int rc = gf_node_pose(g, gi, &gp);
            if (rc != 0) { ++gi; continue; }
            if (gp.t > tnext) break;
            fprintf(fo2, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f\n", gp.t, gp.p[0], gp.p[1], gp.p[2], gp.q[0], gp.q[1], gp.q[2], gp.q[3]); ++ng; ++gi;
        }
        if (i == no) break;
        gf_pose p;
        if (gf_query(g, od[i].t, od[i].p, od[i].q, &p) == 0) {
            fprintf(fo2, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f\n", od[i].t, p.p[0], p.p[1], p.p[2], p.q[0], p.q[1], p.q[2], p.q[3]); ++nout;
        }
    }
    (void)sg;
    fclose(fo2); nout += ng;
    if (nodes) {
        FILE *fn = fopen(nodes, "w");
        for (int i = 0; ; ++i) {
            double t, x[5], rho, sig; int bad, fu;
            if (gf_node(g, i, &t, x, &bad, &fu)) break;
            gf_node_diag(g, i, &rho, &sig);
            fprintf(fn, "%.6f %.8f %.6f %.6f %.6f %.6f %d %d %.3f %.3f\n", t, x[0], x[1], x[2], x[3], x[4], bad, fu, rho, sig);
        }
        fclose(fn);
    }
    gf_stats st; gf_get_stats(g, &st);
    printf("gf_run: %s, %d odom, %d fixes -> %d poses; nodes %ld (gnss-only %ld), fixes assigned %ld gated %ld, distrusted nodes %ld, segments %ld, solves %ld\n",
           causal ? "causal" : "batch", no, nf, nout, st.n_nodes, st.n_gnss_nodes, st.n_fix_assigned, st.n_fix_gated, st.n_distrusted_nodes, st.n_segments, st.n_solves);
    if (timing) {
        report("add_odom (no node)", tc, ntc); report("add_odom (new node)", tn, ntn); report("add_fix", tf, ntf);
    }
    gf_destroy(g); free(od); free(fx); free(tn); free(tc); free(tf);
    return 0;
}
