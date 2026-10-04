/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 *
 * gf_gait.h : walking-speed prior from a phone IMU (step detection, cadence, speed = f(cadence)), streaming, no allocation after create.
 *
 *   Per IMU sample (t, accelerometer specific force [m/s^2], gyro [rad/s]):
 *     an = |a|;  hp = an - EMA(an, tau_slow);  sm = EMA(EMA(hp, tau_sm), tau_sm)
 *     a step is a local maximum of sm above max(std_k * running_std(sm), thr_floor), min_step_dt after the previous one
 *   Epoch estimate over the trailing window W (gf_gait_estimate): cadence = (n-1) / (t_last - t_first) of the steps in the window,
 *     speed v = k * c * cadence^p  (population model: c = 0.389, p = 2 -> 0.70 m step at 1.8 Hz),
 *     state WALK (cadence in [1, 2.8] Hz and enough recent steps) / STATIONARY (still IMU, no steps) / OTHER (shuffling, running, ...: no speed).
 *   Calibration: (a) per user: gf_gait_set_model(c) from a held-out recording with a reference, (b) online: gf_gait_gnss_fix() feeds the position
 *     fixes; chords between 8 s means 22 s apart on straight (gyro heading change <= 20 deg) stretches give k, shrunk to 1 with a prior of 150 m,
 *     (c) none: the defaults are the generic model.
 *   Feed the result to the smoother with gf_add_speed() (gf_fusion.h): speed over [t - window, t], sigma, GF_SPEED_STATIONARY for still epochs.
 * Python reference of the same algorithm: tools/gait.py (tools/check_gait.py compares them).
 */
#ifndef GF_GAIT_H
#define GF_GAIT_H

#ifdef __cplusplus
extern "C" {
#endif

#define GF_GAIT_WALK        0
#define GF_GAIT_STATIONARY  1
#define GF_GAIT_OTHER       2

typedef struct gf_gait_config {
    double tau_slow, tau_sm, tau_std;     /* high-pass (0.8 s), smoothing (2 x 0.03 s), running-std (15 s) time constants */
    double std_k, thr_floor;              /* step threshold: max(0.35 * std, 0.3 m/s^2) */
    double min_step_dt;                   /* 0.3 s */
    double window_s;                      /* default trailing window of an estimate (6 s) */
    int    min_steps;                     /* steps needed in the window (4) */
    double max_last_age;                  /* the last step must be this recent (1.2 s) */
    double cad_min, cad_max;              /* walking cadence range [Hz] (1.0 .. 2.8) */
    double tau_stat, stat_hp_rms, stat_gyro; /* stationary detector: rms of the high-passed norm < 0.12 m/s^2 and mean |gyro| < 0.12 rad/s over ~1.5 s */
    double tau_grav;                      /* gravity filter for the heading (1 s) */
    double model_c, model_p;              /* v = k c cad^p  (0.389, 2) */
    double rel_sigma, abs_sigma;          /* 1-sigma of the model speed: sqrt((rel v)^2 + abs^2)  (0.20, 0.10 m/s); gf_gait_set_model() sets rel_sigma to user_rel_sigma */
    double user_rel_sigma;                /* relative sigma after a per-user calibration (0.10) */
    int    online;                        /* 1: online calibration from gf_gait_gnss_fix() */
    double on_win, on_edge, on_min_dist, on_max_turn, on_eval_dt, on_sigma_max, on_prior_m, k_min, k_max;
} gf_gait_config;

typedef struct gf_gait_est {
    double t, window_s;
    int    n_steps;
    double cadence;                       /* [Hz], 0 if unknown */
    int    state;                         /* GF_GAIT_* */
    double speed, sigma;                  /* [m/s]: valid for WALK (model) and STATIONARY (0, 0.05) */
} gf_gait_est;

typedef struct gf_gait gf_gait;

void      gf_gait_config_default(gf_gait_config *c);
gf_gait  *gf_gait_create(const gf_gait_config *c);   /* NULL = defaults; NULL on allocation failure */
void      gf_gait_destroy(gf_gait *g);
/* per-user model constant c (v = c cad^p); also makes the relative sigma user_rel_sigma */
void      gf_gait_set_model(gf_gait *g, double model_c);
/* one IMU sample, times must increase. Returns 1 if a step was detected at the previous sample, 0 otherwise. */
int       gf_gait_push(gf_gait *g, double t, const double acc[3], const double gyro[3]);
/* estimate over the trailing window (window_s <= 0: configured default) ending at t (normally the newest sample time) */
int       gf_gait_estimate(const gf_gait *g, double t, double window_s, gf_gait_est *out);
/* position fix for the online calibration (any time order up to the IMU clock, east/north metres in any local frame) */
void      gf_gait_gnss_fix(gf_gait *g, double t, double east, double north, double sigma_h);
double    gf_gait_heading(const gf_gait *g);    /* integrated gyro heading about gravity [rad] */
double    gf_gait_scale(const gf_gait *g);      /* online calibration factor k */
double    gf_gait_odometer(const gf_gait *g);   /* walking distance with k applied [m] */

#ifdef __cplusplus
}
#endif
#endif
