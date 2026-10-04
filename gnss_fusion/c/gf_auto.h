/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 *
 * gf_auto.h : streaming (live) fusion front end that runs BOTH the section-3..13 smoother (gf_fusion, fixes + optional gait speed) and the section-14
 * geo-referencing (gf_georef, one slowly varying similarity from the fixes) side by side, and chooses / blends between them online from signals that
 * are observable at the time (section 15 of docs/gnss_vio_benchmark_20261001.md).
 *
 *   stream_mode 0 : the georef stream is the raw odometry (any VIO / VO that is metric or monocular, gravity aligned)
 *   stream_mode 1 : the georef stream is the live output of a second, FIX-FREE smoother B (gait speed prior; the phone pipeline of section 14)
 *
 * Call order per time step (what gf_run / gf_georef_run do): fixes and speed measurements due at time t first, then gf_auto_add_odom(t). gf_auto_get() then
 * returns what a live consumer sees. Nothing is revised later. policy GF_AUTO_SMOOTHER reproduces gf_run (causal live pose), GF_AUTO_GEOREF reproduces
 * the two-stage gf_run + gf_georef_run live output; GF_AUTO_AUTO decides.
 */
#ifndef GF_AUTO_H
#define GF_AUTO_H
#include "gf_fusion.h"
#include "gf_georef.h"
#ifdef __cplusplus
extern "C" {
#endif

#define GF_AUTO_SMOOTHER 0
#define GF_AUTO_GEOREF   1
#define GF_AUTO_AUTO     2

typedef struct gf_auto_config {
    gf_config sm;              /* smoother A (fixes + optional speed); gf_config_robust() by default */
    gf_config st;              /* fix-free smoother B that makes the stream when stream_mode == 1 (init_wait_s 12) */
    gf_georef_config geo;
    int    stream_mode;        /* 0 raw odometry, 1 fix-free smoother (default 0) */
    int    policy;             /* GF_AUTO_* (default GF_AUTO_AUTO) */
    /* ---- the switch (section 15). The georef (G) is used only while ALL hold, else the smoother (S) is used; the output is S, G or a cross-fade (G weight w):
     *   - G exists, >= min_pairs pairs, the stream is metric (geo.scale_sigma <= 10; a free-scale mono stream needs the smoother's per-window scale) and the fixes are
     *     poor: reported sigma (EW mean) >= sig_min [m]  (better fixes than the stream's window drift: the smoother can use them, a global similarity cannot)
     *   - the stream agrees with the fixes: rho = e_slow / max(sigma_rep, sig_floor) <= rho_on to enter, < rho_off to stay   (e_slow = rms prequential error of the
     *     georef at the fixes, EW tau_slow_s)
     *   - the smoother's own consistency test does not distrust the odometry: its distrusted share (EW 60 s) <= distr_on to enter, < distr_off to stay
     *   - no sudden failure: the latest prequential error is not > fail_k x max(e_slow, 2 sigma_noise); then w drops to 0 at once and G is barred for dwell_s
     * w rises at 1 / blend_s per second and falls at 1 / down_s. ---- */
    double tau_fast_s;         /* time constant of the exponentially weighted prequential-residual statistics [s] */
    double tau_slow_s;
    double blend_s, down_s;
    double rho_on, rho_off;
    double distr_on, distr_off;
    double sig_min, sig_floor;
    double fail_k, dwell_s;
    double min_span_s;         /* G is not used before the stream has been paired with fixes over this long [s] */
    int    min_pairs;
    int    metric_only;        /* 1: free-scale (mono) streams never use G in AUTO (default); 0: allowed */
} gf_auto_config;

typedef struct gf_auto gf_auto;

typedef struct gf_auto_sig {   /* observable signals, valid after each gf_auto_add_odom() */
    double t;
    int    n_pairs;            /* georef pairs */
    int    have_fit;
    double sres;               /* in-sample residual rms of the georef fit [m] */
    double scale, psi;         /* georef scale (1 = stream already metric), yaw */
    double e_fast, e_slow;     /* rms of the one-step-ahead (prequential) georef error at the fixes [m], EW with tau_fast_s / tau_slow_s */
    double e_all;              /* ... over all fixes so far */
    double e_last;             /* prequential error of the most recent fix [m] */
    double sig_rep;            /* reported horizontal fix sigma, EW mean [m] */
    double sig_white;          /* white noise of the fixes from second differences, EW [m] */
    double dis_fast;           /* rms horizontal disagreement smoother vs georef [m], EW */
    double distrust;           /* share of recent odometry samples the smoother marks distrusted (EW) */
    int    n_new_frame, n_gap;/* odometry restarts / gaps flagged so far (raw odometry streams) */
    double rho;                /* the decision statistic */
    double w_target;           /* 0 smoother, 1 georef */
} gf_auto_sig;

typedef struct gf_auto_out {
    double t;
    double p[3], q[4];         /* the output pose */
    unsigned status;           /* GF_ST_* of the smoother, or GF_ST_INIT if only the georef exists */
    double w_geo;              /* weight of the georef in p (0 smoother .. 1 georef) */
    int    have_sm, have_geo;
    double p_sm[3], q_sm[4], p_geo[3], q_geo[4];
    unsigned status_sm;
} gf_auto_out;

void      gf_auto_config_default(gf_auto_config *c);
gf_auto  *gf_auto_create(const gf_auto_config *c);   /* NULL on allocation failure; the smoothers run causal with keep_history */
void      gf_auto_destroy(gf_auto *a);
int       gf_auto_add_odom(gf_auto *a, double t, const double p[3], const double q[4], unsigned flags);   /* 0 or a gf_add_odom error */
/* returns 1 if the fix made a GNSS-only node of the smoother (odometry lost): gf_auto_get_gnss_only() then gives the live pose */
int       gf_auto_add_fix(gf_auto *a, const gf_fix *f);
int       gf_auto_add_speed(gf_auto *a, double t, double speed, double sigma, double window_s, unsigned flags);
int       gf_auto_get(const gf_auto *a, gf_auto_out *out);            /* latest output after gf_auto_add_odom; -1 if none */
int       gf_auto_get_gnss_only(const gf_auto *a, gf_auto_out *out);  /* smoother pose after a fix that returned 1 */
void      gf_auto_signals(const gf_auto *a, gf_auto_sig *s);
const gf_t *gf_auto_smoother(const gf_auto *a);

/* Optional CPU-time accounting: the library has no clock of its own. Set a function returning seconds (e.g. CLOCK_THREAD_CPUTIME_ID); the time spent in
 * smoother A, stream smoother B and georef + switch inside gf_auto_add_odom() / gf_auto_add_fix() is accumulated in gf_auto_timing. */
typedef double (*gf_auto_clock_fn)(void);
typedef struct gf_auto_timing { double sm, st, geo, fix, speed; long n_odom, n_fix; } gf_auto_timing;
void gf_auto_set_clock(gf_auto *a, gf_auto_clock_fn fn);
void gf_auto_get_timing(const gf_auto *a, gf_auto_timing *t);

#ifdef __cplusplus
}
#endif
#endif
