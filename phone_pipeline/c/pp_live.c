/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code (links stella_vio, BSD-2, as a library through the sv_run driver).
 *
 * pp_live : the TRUE LIVE phone pipeline in ONE process, sample by sample (section 15 of docs/gnss_vio_benchmark_20261001.md).
 *
 *   images --> stella_vio (sv_system_feed, one frame at a time)  -->  per-frame LIVE pose (the tracking pose of that frame, before any later local BA / loop
 *              correction) + map id / R-frame / segment id / gravity estimate of that moment
 *   IMU    --> gf_gait   (step cadence -> walking speed measurement every 3 s)
 *   fixes  -----------------------------------------------\
 *   live odometry (gravity aligned with the CURRENT up estimate of its map, flags as phone_pipeline/run.py make_odom) --> gf_auto
 *              (smoother with fixes + fix-free gait smoother -> georef -> automatic switch) --> the output pose of that frame
 *
 * Nothing is looked at that a real-time consumer would not have at the moment of the frame: IMU samples and fixes with time <= the frame time, the image of the frame.
 * Events are processed at EVERY frame time (also when the frame is not tracked) unless --pp-lazy (then only at tracked frames, the order of the offline file pipeline).
 *
 * It is the sv_run driver (stella_vio/c/sv_run.c, included here with SV_RUN_NO_MAIN) plus a per-frame hook, so all sv_run options apply
 * (<vocab> <tum_seq_dir> <fixtures_dir> <out_dir> --lean --no-snap --wait-fixtures --size --camera --imu --imu-ext --imu-toff --imu-bg --set ... --live-out F).
 * Own options (removed before sv_run sees them):
 *   --pp-out PREFIX         writes PREFIX.{auto,sm,geo,odom,sig}  (auto/sm/geo: "t x y z qx qy qz qw status w_geo"; odom: the live odometry fed to the fusion,
 *                           replayable with gnss_fusion/c/gf_auto_run) and PREFIX.timing (CPU seconds per stage)
 *   --pp-speed-out F        the gait speed measurements "t v sigma window flags" (for replays)
 *   --pp-fix F              fixes csv of the dataset (ns,E,N,U,hErr1,hErr2,vErr); without it nothing is geo-referenced (gait-scaled, arbitrary frame)
 *   --pp-gait-c C           per-user gait constant (default: generic 0.389)
 *   --pp-set key=value      gf_auto key (gf_auto_cfg.h); defaults = the phone pipeline of section 13/14 with the switch in AUTO
 *   --pp-lazy               process IMU / fixes only at tracked frames (reproduces the order of the offline file pipeline)
 *   --pp-up-freeze N        a map's gravity rotation is frozen once N frames have contributed (default 150; 0 = follow the running estimate)
 *   --pp-up-min N           no odometry sample before N frames contributed to the map's up vector (default 5)
 *   --pp-servo-noise S      log-normal noise (sigma S) on the speeds that the stella_vio gait scale servo sees (robustness test; the fusion keeps the clean speeds)
 *   --pp-jump 0|1           flag frames of an accepted loop closure / scale calibration as GF_ODOM_GAP (position jump of the map), default 0: it cost 0.1-0.7 m on the indoor sequences (section 15)
 */
#define SV_RUN_NO_MAIN 1
#define SV_RUN_HOST 1   /* keep the sequence association (rgb.txt / depth.txt) of the stand-alone driver */
#include "sv_run.c"
#include "gf_gait.h"
#include "gf_auto.h"
#include "gf_auto_cfg.h"

typedef struct { double t, w[3], a[3]; } imus;
typedef struct { double t, v, sg, w; unsigned fl; } spd;

typedef struct {
    gf_auto *au; gf_gait *gait; sv_system *sys;
    imus *imu; size_t n_imu, k_imu;
    gf_fix *fx; int nf, fi;
    double g_t0, g_nxt;
    spd *sp; int nsp, spcap;
    int lazy, up_freeze, up_min, jump;
    double servo_noise; unsigned long long rng;     /* --pp-servo-noise S: log-normal noise of sigma S on the speeds the servo sees (robustness test; the fusion keeps the clean speeds) */
    int have_prev, prev_map, prev_seg; double last_t;
    unsigned char *frozen; double (*Rg)[9]; int fcap;     /* per map id */
    FILE *fa, *fsm, *fgeo, *fodom, *fspd, *fsig;
    /* CPU accounting */
    double last_exit, cpu_sv, cpu_gait, cpu_fuse;
    double *t_sv, *t_gait, *t_fuse; long cap_t, n_hook, n_odom, n_skipped_up;
} pp_t;

static pp_t PP;

static double cpu_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
}

static void pp_out(FILE *f, const gf_auto_out *o, int which)
{
    if (!f) return;
    if (which == 0) fprintf(f, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u %.3f\n", o->t, o->p[0], o->p[1], o->p[2], o->q[0], o->q[1], o->q[2], o->q[3], o->status, o->w_geo);
    else if (which == 1 && o->have_sm) fprintf(f, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f %u 0\n", o->t, o->p_sm[0], o->p_sm[1], o->p_sm[2], o->q_sm[0], o->q_sm[1], o->q_sm[2], o->q_sm[3], o->status_sm);
    else if (which == 2 && o->have_geo) fprintf(f, "%.9f %.6f %.6f %.6f %.8f %.8f %.8f %.8f 1 0\n", o->t, o->p_geo[0], o->p_geo[1], o->p_geo[2], o->q_geo[0], o->q_geo[1], o->q_geo[2], o->q_geo[3]);
}

/* gravity alignment of one pose (the arithmetic of sv_run.c's trajectory_gz.tum writer): up -> +z by a Rodrigues rotation Rg */
static void rg_from_up(const double up[3], double Rg[9])
{
    const double ax = up[1], ay = -up[0], n2 = sqrt(ax * ax + ay * ay), ca = up[2];
    const double ang = acos(ca > 1.0 ? 1.0 : (ca < -1.0 ? -1.0 : ca));
    if (n2 < 1e-12) { Rg[0] = Rg[4] = Rg[8] = 1.0; Rg[1] = Rg[2] = Rg[3] = Rg[5] = Rg[6] = Rg[7] = 0.0; }
    else {
        const double kx = ax / n2, ky = ay / n2, c = cos(ang), sn = sin(ang), v = 1.0 - c;
        Rg[0] = c + kx * kx * v;   Rg[1] = kx * ky * v;       Rg[2] = ky * sn;
        Rg[3] = kx * ky * v;       Rg[4] = c + ky * ky * v;   Rg[5] = -kx * sn;
        Rg[6] = -ky * sn;          Rg[7] = kx * sn;           Rg[8] = c;
    }
}

static void gz_pose(const double Rg[9], const double wc[16], double pz[3], double q[4])
{
    double R[9], B[9], tr, qx, qy, qz, qw;
    int a, b, c2;
    for (a = 0; a < 3; ++a) for (b = 0; b < 3; ++b) R[a * 3 + b] = wc[b * 4 + a];
    for (a = 0; a < 3; ++a) {
        pz[a] = Rg[a * 3 + 0] * wc[12] + Rg[a * 3 + 1] * wc[13] + Rg[a * 3 + 2] * wc[14];
        for (b = 0; b < 3; ++b) { B[a * 3 + b] = 0.0; for (c2 = 0; c2 < 3; ++c2) B[a * 3 + b] += Rg[a * 3 + c2] * R[c2 * 3 + b]; }
    }
    tr = B[0] + B[4] + B[8];
    if (tr > 0.0) { const double sq = sqrt(tr + 1.0) * 2.0; qw = 0.25 * sq; qx = (B[7] - B[5]) / sq; qy = (B[2] - B[6]) / sq; qz = (B[3] - B[1]) / sq; }
    else if (B[0] > B[4] && B[0] > B[8]) { const double sq = sqrt(1.0 + B[0] - B[4] - B[8]) * 2.0; qw = (B[7] - B[5]) / sq; qx = 0.25 * sq; qy = (B[1] + B[3]) / sq; qz = (B[2] + B[6]) / sq; }
    else if (B[4] > B[8]) { const double sq = sqrt(1.0 + B[4] - B[0] - B[8]) * 2.0; qw = (B[2] - B[6]) / sq; qx = (B[1] + B[3]) / sq; qy = 0.25 * sq; qz = (B[5] + B[7]) / sq; }
    else { const double sq = sqrt(1.0 + B[8] - B[0] - B[4]) * 2.0; qw = (B[3] - B[1]) / sq; qx = (B[2] + B[6]) / sq; qy = (B[5] + B[7]) / sq; qz = 0.25 * sq; }
    q[0] = qx; q[1] = qy; q[2] = qz; q[3] = qw;
}

static void push_speed(pp_t *P, double t, double v, double sg, double w, unsigned fl)
{
    if (P->nsp == P->spcap) { P->spcap = P->spcap ? P->spcap * 2 : 256; P->sp = (spd *)realloc(P->sp, sizeof(spd) * (size_t)P->spcap); }
    P->sp[P->nsp].t = t; P->sp[P->nsp].v = v; P->sp[P->nsp].sg = sg; P->sp[P->nsp].w = w; P->sp[P->nsp].fl = fl; ++P->nsp;
    if (P->fspd) fprintf(P->fspd, "%.9f %.6f %.6f %.3f %u\n", t, v, sg, w, fl);
    if (P->sys) {      /* stella_vio gait scale servo (--set servo=G; ignored while off) */
        double vs = v;
        if (P->servo_noise > 0.0) {      /* Box-Muller from an LCG: deterministic */
            double u1, u2, z;
            P->rng = P->rng * 6364136223846793005ULL + 1442695040888963407ULL; u1 = ((double)(P->rng >> 11) + 1.0) / 9007199254740993.0;
            P->rng = P->rng * 6364136223846793005ULL + 1442695040888963407ULL; u2 = (double)(P->rng >> 11) / 9007199254740992.0;
            z = sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
            vs = v * exp(P->servo_noise * z);
        }
        sv_system_push_speed(P->sys, t, vs);
    }
}

static void frame_hook(void *user, unsigned int frame, double ts, const sv_frame_result *r, sv_system *sys)
{
    pp_t *P = (pp_t *)user;
    const double t_in = cpu_now();
    double t1, t2;
    (void)frame;
    P->sys = sys;
    if (P->n_hook) {
        P->cpu_sv += t_in - P->last_exit;
        if (P->n_hook < P->cap_t) P->t_sv[P->n_hook] = t_in - P->last_exit;
    }
    if (P->lazy && !r->live_valid) { P->last_exit = cpu_now(); ++P->n_hook; return; }
    /* 1. IMU up to the frame time -> gait epochs (speed measurements, kept until the fusion step below) */
    t1 = cpu_now();
    while (P->k_imu < P->n_imu && P->imu[P->k_imu].t <= ts) {
        const imus *s = &P->imu[P->k_imu++];
        if (P->g_t0 < 0) { P->g_t0 = s->t; P->g_nxt = s->t + 3.0; }
        gf_gait_push(P->gait, s->t, s->a, s->w);
        if (s->t >= P->g_nxt) {
            gf_gait_est e;
            gf_gait_estimate(P->gait, s->t, 6.0, &e);
            if (e.state == GF_GAIT_WALK) push_speed(P, s->t, e.speed, e.sigma, e.window_s, 0);
            else if (e.state == GF_GAIT_STATIONARY) push_speed(P, s->t, 0.0, e.sigma, e.window_s, GF_SPEED_STATIONARY);
            P->g_nxt += 3.0;
        }
    }
    t2 = cpu_now();
    P->cpu_gait += t2 - t1;
    if (P->n_hook < P->cap_t) P->t_gait[P->n_hook] = t2 - t1;
    /* 2. fixes, speed measurements, the odometry sample */
    t1 = cpu_now();
    while (P->fi < P->nf && P->fx[P->fi].t <= ts) {
        if (gf_auto_add_fix(P->au, &P->fx[P->fi])) { gf_auto_out g; if (!gf_auto_get_gnss_only(P->au, &g)) pp_out(P->fa, &g, 0); }
        ++P->fi;
    }
    for (int i = 0; i < P->nsp; ++i) gf_auto_add_speed(P->au, P->sp[i].t, P->sp[i].v, P->sp[i].sg, P->sp[i].w, P->sp[i].fl);
    P->nsp = 0;
    if (r->live_valid && ts > P->last_t) {
        const int mid = r->live_map_id;
        if (r->live_up_n == 0 || r->live_up_n < (unsigned)P->up_min) ++P->n_skipped_up;
        else {
            double pz[3], q[4];
            unsigned fl = 0;
            if (mid >= P->fcap) {
                const int nc = mid * 2 + 8;
                P->frozen = (unsigned char *)realloc(P->frozen, (size_t)nc);
                P->Rg = (double (*)[9])realloc(P->Rg, sizeof(double[9]) * (size_t)nc);
                memset(P->frozen + P->fcap, 0, (size_t)(nc - P->fcap));
                P->fcap = nc;
            }
            if (!P->frozen[mid]) {
                rg_from_up(r->live_up, P->Rg[mid]);
                if (P->up_freeze > 0 && r->live_up_n >= (unsigned)P->up_freeze) P->frozen[mid] = 1;
            }
            gz_pose(P->Rg[mid], r->pose_wc, pz, q);
            if (P->have_prev && mid != P->prev_map) fl |= GF_ODOM_NEW_FRAME;
            else if (P->have_prev && r->live_seg != P->prev_seg) fl |= GF_ODOM_GAP | GF_ODOM_LOOSE;
            else if (P->jump && (r->loop_accepted || r->cal_f != 0.0)) fl |= GF_ODOM_GAP;
            if (r->live_rframe) fl |= GF_ODOM_LOOSE;
            P->have_prev = 1; P->prev_map = mid; P->prev_seg = r->live_seg; P->last_t = ts;
            if (P->fodom) fprintf(P->fodom, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f %u\n", ts, pz[0], pz[1], pz[2], q[0], q[1], q[2], q[3], fl);
            if (gf_auto_add_odom(P->au, ts, pz, q, fl) == 0) {
                gf_auto_out o;
                if (gf_auto_get(P->au, &o) == 0) { pp_out(P->fa, &o, 0); pp_out(P->fsm, &o, 1); pp_out(P->fgeo, &o, 2); }
                if (P->fsig) {
                    gf_auto_sig g; gf_auto_signals(P->au, &g);
                    fprintf(P->fsig, "%.6f %d %d %.4f %.5f %.5f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %.4f %.4f %.3f %.4f\n", g.t, g.n_pairs, g.have_fit, g.sres, g.scale, g.psi, sqrt(g.e_fast), sqrt(g.e_slow), sqrt(g.e_all),
                            g.sig_rep, g.sig_white, g.dis_fast, g.n_new_frame, g.n_gap, g.distrust, g.rho, g.w_target, sqrt(g.e_last));
                }
                ++P->n_odom;
            }
        }
    }
    t2 = cpu_now();
    P->cpu_fuse += t2 - t1;
    if (P->n_hook < P->cap_t) P->t_fuse[P->n_hook] = t2 - t1;
    P->last_exit = cpu_now();
    ++P->n_hook;
}

static int cmpd(const void *a, const void *b) { const double x = *(const double *)a, y = *(const double *)b; return (x > y) - (x < y); }

static void stat_line(FILE *f, const char *name, double *v, long n, double total)
{
    double *c;
    if (n <= 0) { fprintf(f, "%s n=0\n", name); return; }
    c = (double *)malloc(sizeof(double) * (size_t)n);
    memcpy(c, v, sizeof(double) * (size_t)n);
    qsort(c, (size_t)n, sizeof(double), cmpd);
    fprintf(f, "%s cpu_s %.3f per_frame_us mean %.1f p50 %.1f p99 %.1f max %.1f n %ld\n", name, total, 1e6 * total / (double)n, 1e6 * c[n / 2], 1e6 * c[(long)(0.99 * (double)(n - 1))], 1e6 * c[n - 1], n);
    free(c);
}

int main(int argc, char **argv)
{
    const char *out = NULL, *fixp = NULL, *imup = NULL, *spout = NULL;
    double gait_c = -1.0;
    char *av[256]; int nav = 0;
    gf_auto_config cfg;
    const char *sets[32]; int nsets = 0;
    memset(&PP, 0, sizeof PP);
    PP.up_freeze = 150; PP.up_min = 5; PP.jump = 0; PP.g_t0 = -1.0;
    for (int i = 0; i < argc && nav < 255; ++i) {
        if (!strcmp(argv[i], "--pp-out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--pp-speed-out") && i + 1 < argc) spout = argv[++i];
        else if (!strcmp(argv[i], "--pp-fix") && i + 1 < argc) fixp = argv[++i];
        else if (!strcmp(argv[i], "--pp-gait-c") && i + 1 < argc) gait_c = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pp-set") && i + 1 < argc && nsets < 32) sets[nsets++] = argv[++i];
        else if (!strcmp(argv[i], "--pp-lazy")) PP.lazy = 1;
        else if (!strcmp(argv[i], "--pp-up-freeze") && i + 1 < argc) PP.up_freeze = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pp-up-min") && i + 1 < argc) PP.up_min = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pp-jump") && i + 1 < argc) PP.jump = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pp-servo-noise") && i + 1 < argc) { PP.servo_noise = atof(argv[++i]); PP.rng = 88172645463325252ULL; }
        else {
            if (!strcmp(argv[i], "--imu") && i + 1 < argc) imup = argv[i + 1];
            av[nav++] = argv[i];
        }
    }
    if (!out || !imup) { fprintf(stderr, "pp_live: need --pp-out PREFIX and --imu imu.csv (see the header of pp_live.c)\n"); return 2; }
    /* IMU for the gait stage (the same csv sv_run reads: t_ns,gx,gy,gz,ax,ay,az) */
    {
        FILE *f = fopen(imup, "r"); char line[1024]; size_t cap = 4096;
        if (!f) { fprintf(stderr, "pp_live: cannot read %s\n", imup); return 2; }
        PP.imu = (imus *)malloc(sizeof(imus) * cap);
        while (fgets(line, sizeof line, f)) {
            long long tn; double w[3], a[3];
            if (line[0] == '#') continue;
            if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &tn, &w[0], &w[1], &w[2], &a[0], &a[1], &a[2]) != 7) continue;
            if (PP.n_imu == cap) { cap *= 2; PP.imu = (imus *)realloc(PP.imu, sizeof(imus) * cap); }
            PP.imu[PP.n_imu].t = (double)tn * 1e-9; memcpy(PP.imu[PP.n_imu].w, w, sizeof w); memcpy(PP.imu[PP.n_imu].a, a, sizeof a); ++PP.n_imu;
        }
        fclose(f);
    }
    if (fixp) {
        FILE *f = fopen(fixp, "r"); char line[512]; int cap = 1024;
        if (!f) { fprintf(stderr, "pp_live: cannot read %s\n", fixp); return 2; }
        PP.fx = (gf_fix *)malloc(sizeof(gf_fix) * (size_t)cap);
        while (fgets(line, sizeof line, f)) {
            double v[7]; gf_fix g;
            if (!(line[0] >= '0' && line[0] <= '9')) continue;
            if (sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", v, v + 1, v + 2, v + 3, v + 4, v + 5, v + 6) != 7) continue;
            memset(&g, 0, sizeof g);
            g.t = v[0] * 1e-9; g.p[0] = v[1]; g.p[1] = v[2]; g.p[2] = v[3]; g.sigma_h = v[4] > 0.02 ? v[4] : 0.02; g.sigma_v = v[6] > 0.02 ? v[6] : 0.02;
            if (PP.nf == cap) { cap *= 2; PP.fx = (gf_fix *)realloc(PP.fx, sizeof(gf_fix) * (size_t)cap); }
            PP.fx[PP.nf++] = g;
        }
        fclose(f);
    }
    gf_auto_config_default(&cfg);
    {
        static const char *base[] = { "preset=robust", "metric=0", "rsa=0,0,0", "speed=1", "speed_align=1", "speed_scale_rw_rel=1", "speed_align_metric=1", "loose_k=5",
                                      "stream=1", "g.scale_sigma=0.15", "policy=2" };
        for (size_t i = 0; i < sizeof base / sizeof *base; ++i) if (gf_auto_set(&cfg, base[i])) { fprintf(stderr, "pp_live: internal config error %s\n", base[i]); return 2; }
        if (!PP.nf) cfg.sm.init_wait_s = 12.0;     /* no fixes: alignment from the gait speed alone */
        for (int i = 0; i < nsets; ++i) if (gf_auto_set(&cfg, sets[i])) { fprintf(stderr, "pp_live: bad --pp-set %s\n", sets[i]); return 2; }
    }
    PP.au = gf_auto_create(&cfg);
    PP.gait = gf_gait_create(NULL);
    if (!PP.au || !PP.gait) return 1;
    if (gait_c > 0) gf_gait_set_model(PP.gait, gait_c);
    gf_auto_set_clock(PP.au, cpu_now);
    {
        char path[4096];
#define OPENF(fp, ext) snprintf(path, sizeof path, "%s.%s", out, ext); fp = fopen(path, "w")
        OPENF(PP.fa, "auto"); OPENF(PP.fsm, "sm"); OPENF(PP.fgeo, "geo"); OPENF(PP.fodom, "odom"); OPENF(PP.fsig, "sig");
        if (spout) PP.fspd = fopen(spout, "w");
    }
    PP.cap_t = 200000;
    PP.t_sv = (double *)calloc((size_t)PP.cap_t, sizeof(double)); PP.t_gait = (double *)calloc((size_t)PP.cap_t, sizeof(double)); PP.t_fuse = (double *)calloc((size_t)PP.cap_t, sizeof(double));
    sv_run_frame_hook = frame_hook;
    sv_run_frame_hook_user = &PP;
    {
        const double c0 = cpu_now();
        const int rc = sv_run_main(nav, av);
        const double c1 = cpu_now();
        char path[4096]; FILE *f;
        gf_auto_timing tm; gf_auto_get_timing(PP.au, &tm);
        if (PP.fa) fclose(PP.fa);
        if (PP.fsm) fclose(PP.fsm);
        if (PP.fgeo) fclose(PP.fgeo);
        if (PP.fodom) fclose(PP.fodom);
        if (PP.fspd) fclose(PP.fspd);
        if (PP.fsig) fclose(PP.fsig);
        snprintf(path, sizeof path, "%s.timing", out);
        f = fopen(path, "w");
        if (f) {
            const long n = PP.n_hook < PP.cap_t ? PP.n_hook : PP.cap_t;
            fprintf(f, "frames %ld imu_samples %zu odom_samples %ld skipped_no_up %ld fixes %d process_thread_cpu_s %.3f\n", PP.n_hook, PP.n_imu, PP.n_odom, PP.n_skipped_up, PP.nf, c1 - c0);
            stat_line(f, "stella_vio(frame incl. PGM read)", PP.t_sv + 1, n > 1 ? n - 1 : 0, PP.cpu_sv);
            stat_line(f, "gait(IMU->speed)", PP.t_gait, n, PP.cpu_gait);
            stat_line(f, "fusion(all)", PP.t_fuse, n, PP.cpu_fuse);
            fprintf(f, "fusion detail: smoother_A %.4f s  stream_smoother_B %.4f s  georef+switch %.4f s  fix %.4f s (%ld)  speed %.4f s  odom calls %ld\n", tm.sm, tm.st, tm.geo, tm.fix, tm.n_fix, tm.speed, tm.n_odom);
            fclose(f);
        }
        gf_auto_destroy(PP.au); gf_gait_destroy(PP.gait);
        return rc;
    }
}
