/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 *
 * gf_fusion.h : loosely-coupled GNSS + visual(-inertial) odometry fusion for drones / phones.
 *
 * Model (a C99 re-implementation of tools/gnss_loose_fusion.py with robustness extensions):
 *   The odometry (VIO / VO) is any pose stream in its own frame. Its gravity direction is either the odometry's +z
 *   (gravity_aligned) or supplied with gf_set_gravity(). The global frame is a local ENU frame (see gf_geo.h).
 *   One node every node_dt seconds, state per node  x = (psi, px, py, pz, s)
 *     psi : yaw of the odometry frame w.r.t. ENU        p : IMU/body position in ENU        s : metric scale of the odometry
 *   factors
 *     odometry link i->j : Rz(-psi_i)(p_j - p_i) - s_i (pL_j - pL_i) ~ N(0, (sp + kp|d|)^2)
 *     yaw / scale random walk, weak scale prior s ~ 1
 *     GNSS fix           : p_i + Rz(psi_i) a_i - z_i ~ N(0, diag(sh, sh, sv)^2), robust loss (Huber / Cauchy) + chi^2 gating
 *     optional GNSS velocity: Rz(psi_i) s_i dL_i/dt - v_i ~ N(0, sigma_vel^2)
 *   Solved with Gauss-Newton on the block-tridiagonal normal equations (5x5 blocks, O(window) per iteration).
 *   Extensions: odometry segments (gaps / new maps), odometry-vs-GNSS consistency test (odometry links that disagree with the
 *   fixes are dropped and the nodes are bridged by the fixes only), chi^2 gating of fixes.
 *
 * Two modes: causal (online; sliding window of window_s seconds, older nodes frozen) and batch (full graph, gf_solve_batch()).
 * Units: seconds, metres, radians; quaternions are (x, y, z, w) (Hamilton), poses are IMU/body poses (the antenna lever arm
 * cfg.rsa, in the body frame, is only used inside the GNSS factor).
 */
#ifndef GF_FUSION_H
#define GF_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { GF_LOSS_NONE = 0, GF_LOSS_HUBER = 1, GF_LOSS_CAUCHY = 2 } gf_loss;

/* gf_add_odom flags */
#define GF_ODOM_NEW_FRAME   1u  /* odometry restarted in a NEW frame (VO new map / re-initialisation): unrelated yaw/scale/origin */
#define GF_ODOM_GAP         2u  /* explicit tracking loss before this sample, SAME frame continues (gaps > cfg.gap_s are detected automatically) */
#define GF_ODOM_LOOSE       4u  /* the odometry link that ends in this sample is weak (e.g. an extrapolated rotation-only stretch): its position sigma is multiplied by cfg.loose_k (opt-in) */

/* gf_add_speed flags */
#define GF_SPEED_STATIONARY 1u  /* the device is still: zero-velocity factor on the links of the window (speed is ignored) */

/* gf_pose.status bits */
#define GF_ST_INIT          1u  /* alignment initialised; pose is geo-referenced (otherwise odometry is passed through in its own frame) */
#define GF_ST_SEG_PROVISIONAL 2u /* current odometry frame not yet aligned by its own fixes (position = continuation) */
#define GF_ST_ODOM_DISTRUSTED 4u /* consistency test failed: the pose is bridged by the GNSS fixes only */
#define GF_ST_NO_ODOM       16u /* odometry lost: GNSS-only node (position from the fixes, orientation = last odometry orientation, held) */
#define GF_ST_NO_RECENT_FIX 8u  /* no fix used for > cfg.blackout_s: dead-reckoning on odometry */

typedef struct gf_config {
    /* graph */
    double node_dt;          /* node spacing [s] (1.0) */
    double window_s;         /* causal sliding window [s] (30); 0 -> use all nodes (grows without bound) */
    int    batch_iters;      /* Gauss-Newton iterations, batch (25) */
    int    causal_iters;     /* ... per window solve, causal (4) */
    int    init_iters;       /* ... first-nodes settle solve, causal (10) */
    int    settle_nodes;     /* nodes solved together right after the initial alignment, causal (10) */
    double init_wait_s;      /* causal: alignment from the fixes of the first init_wait_s seconds (30) */
    int    keep_history;     /* 1: keep all nodes (needed for gf_query() of old times); 0: bounded memory in causal mode */
    /* odometry noise model (set from physical reasoning) */
    double odom_sp, odom_kp; /* position sigma sp + kp*|d| [m] (0.05, 0.02) */
    double yaw_rw;           /* yaw random walk [rad/sqrt(s)] (0.5 deg) */
    double scale_rw;         /* scale random walk [1/sqrt(s)] (0.003) */
    double scale_prior;      /* sigma of the weak prior s ~ 1 (0.2) */
    double scale_rw_mono, scale_prior_mono; /* the same two for metric_scale == 0 (defaults equal the metric values) */
    int    metric_scale;     /* 1: odometry is metric (VIO); 0: monocular VO, scale per frame estimated at alignment */
    int    gravity_aligned;  /* 1: odometry +z is up; 0: call gf_set_gravity() before the first gf_add_odom of every frame */
    double rsa[3];           /* GNSS antenna position in the body frame [m] */
    /* GNSS */
    gf_loss loss;            /* robust loss on the fixes (HUBER) */
    double loss_k;           /* threshold in sigmas (2.5 Huber; 2.385 is the usual Cauchy tuning) */
    double min_sigma;        /* floor on reported sigmas [m] (0.02) */
    double gate_chi2;        /* chi^2 (3 dof) gate on fixes, 0 = off (16.27 = 99.9 %) */
    double gate_floor;       /* metres added in quadrature to the fix sigma in the gate (odometry prediction error, 1.0) */
    int    gate_min_fixes;   /* a segment needs this many fixes before any of them can be gated (5) */
    int    robust_init;     /* 1: trimmed Procrustes in the initial alignment */
    /* segments */
    double gap_s;            /* odometry gap that starts a new segment [s] (2.0) */
    double max_speed;        /* odometry speed above which a sample pair is a jump -> new frame [m/s], 0 = off */
    double link_speed;       /* sigma of the position bridge across a segment break / distrusted stretch [m/s] (3.0) */
    double seg_min_extent;   /* a new frame is aligned by its own fixes once its track spans this much [m] (4.0) */
    /* odometry-vs-GNSS consistency ("trust GNSS when odometry is inconsistent") */
    int    trust;            /* 1: enable */
    double trust_window_s;   /* test window [s] (30) */
    int    trust_min_fixes;  /* (6) */
    double trust_scale_k;    /* metric odometry: window similarity scale outside [1/k, k] -> distrusted (1.5) */
    double trust_scale_k_mono; /* monocular: ratio of the similarity scales left / right of a node outside [1/k, k] -> distrusted; 0 = off (default: it helps batch a little and hurts causal) */
    double trust_long_s;     /* long window for the slow-drift test [s], 0 = off (120 recommended) */
    int    trust_long_and;   /* 1: the long window must confirm the short-window verdict (robust to bursts of bad fixes); 0: either one distrusts */
    double trust_rho_long;   /*  ... residual of its similarity fit above this [m] -> distrusted (8.0) */
    double trust_state_k;    /* window similarity scale vs the scale state outside [1/k, k] -> distrusted (monocular collapse); 0 = off */
    double trust_start_m;    /* consistency fits start from the best contiguous sub-set (inliers within this many metres), robust to bursts of common-offset fixes; 0 = off */
    int    gnss_only_nodes;  /* 1: while the odometry is silent for more than gap_s, fixes make nodes of their own (output continues through tracking loss); 0 = fixes dropped (python) */
    double grow_s;           /* causal, metric odometry: during the first grow_s seconds the window covers the whole trajectory (no frozen nodes); 0 = off (python) */
    double grow_ratio;       /* ... and it ends earlier once the fixes span grow_ratio x their reported sigma (geometry strong enough to fix yaw / scale); 0 = time cap only */
    double grow_s_mono;      /* the same for monocular odometry (0: its scale / yaw drift makes a long memory worse) */
    double trust_rho_k;      /* residual of the similarity fit > k * (GNSS noise) -> distrusted (4.0) */
    double trust_rho_min;    /*  ... and > this many metres (5.0) */
    double trust_spread_k;   /* GNSS spread in the window must exceed k * noise for the scale test to be meaningful (3.0) */
    double trust_q_scale;    /* yaw/scale random walk multiplier across a distrusted stretch (10) */
    double scale_min, scale_max; /* clamp of the scale state after every Gauss-Newton step (relative to the odometry unit); scale_max = 0: off (python) */
    /* gait / speed prior (gf_add_speed); everything off by default */
    int    speed_on;         /* 1: use the speed measurements (0: they are accepted and ignored) */
    double speed_k;          /* Huber threshold of the speed factor in sigmas (2.0) */
    double speed_sigma_scale; /* multiplies the reported speed sigmas (1.0) */
    double speed_link_sigma; /* extra sigma [m/s] of the single-link speed factor on bridged / GNSS-only links (instantaneous vs window-mean speed, 0.3) */
    double zupt_sigma;       /* zero-velocity factor: displacement sigma per second [m/s] (0.05) */
    int    speed_align;      /* 1: an odometry frame with fewer than 3 fixes is aligned by the speed measurements (scale, yaw 0, position continued): indoors */
    double speed_scale_rw;   /* with speed_on, if > 0: scale random walk [1/sqrt(s)] replacing scale_rw / scale_rw_mono (the speed measurements observe the scale, so it may move faster); 0: unchanged */
    double speed_scale_lim;  /* with speed_on the scale state is clamped to [1/lim, lim] instead of [scale_min, scale_max] (1e5: collapsed / diverged metric odometry can be re-scaled) */
    double speed_scale_rw_rel; /* 1: scale random walk relative to |s| during speed use (allows large scale changes of collapsing odometry); 0: absolute (default) */
    int    speed_align_metric; /* 1: monocular speed alignment tests seg_min_extent on the METRIC path (path x speed-derived unit) instead of the odometry units (0, default: old behaviour) */
    double loose_k;          /* GF_ODOM_LOOSE: multiplier of the odometry position sigma of the link into a node flagged loose (1 = flag has no effect) */
    /* output */
    double drift_rate;       /* heuristic drift per metre travelled since the last fix, for sigma_h (0.02) */
    double blackout_s;       /* GF_ST_NO_RECENT_FIX after this long without an accepted fix (10) */
} gf_config;

typedef struct gf_fix {
    double t;                /* seconds, same clock as the odometry */
    double p[3];             /* local ENU position of the antenna [m] (see gf_geo.h to convert LLA) */
    double sigma_h, sigma_v; /* 1-sigma horizontal / vertical [m] */
    int    has_vel;
    double v[3];             /* ENU velocity [m/s] */
    double sigma_vel;        /* [m/s] */
} gf_fix;

typedef struct gf_pose {
    double t;
    double p[3];             /* IMU/body position in ENU */
    double q[4];             /* body orientation in ENU (x y z w) */
    double yaw, scale;       /* current alignment of the odometry frame */
    double sigma_h;          /* heuristic horizontal 1-sigma [m] (grows with distance since the last fix) */
    double dist_since_fix;   /* path length since the last accepted fix [m] */
    unsigned status;         /* GF_ST_* */
} gf_pose;

typedef struct gf_stats {
    long n_odom, n_fix, n_nodes, n_fix_assigned, n_fix_gated, n_solves;
    long n_distrusted_nodes, n_segments, n_gnss_nodes, n_speed_assigned;
} gf_stats;

typedef struct gf_t gf_t;

void   gf_config_default(gf_config *c);
/* Recommended robust settings on top of the defaults: consistency test + long-window drift test, chi^2 gating, trimmed init, free mono scale. */
void   gf_config_robust(gf_config *c);
/* causal != 0: online sliding-window mode; 0: batch (nothing is solved until gf_solve_batch()). NULL on allocation failure. */
gf_t  *gf_create(const gf_config *c, int causal);
void   gf_destroy(gf_t *g);
void   gf_reset(gf_t *g);

/* Direction of "up" in the odometry frame (any non-zero vector). Applies to the NEXT odometry frame (call it before the first
 * gf_add_odom of the stream and again before a sample flagged GF_ODOM_NEW_FRAME). Needed when cfg.gravity_aligned == 0. */
int    gf_set_gravity(gf_t *g, const double up[3]);

/* Odometry sample: pose of the body in the odometry frame. Times must increase. Returns 0, or a negative error. */
int    gf_add_odom(gf_t *g, double t, const double p[3], const double q[4], unsigned flags);
/* Speed measurement (gait prior, see gf_gait.h): MEAN horizontal speed over [t - window_s, t] with 1-sigma `sigma` [m/s], or flags with
 * GF_SPEED_STATIONARY. Acts on the scale state of the node nearest to t (|v_horizontal of the odometry| s = v) and, where the odometry link is
 * bridged / distrusted / GNSS-only, on the horizontal node displacement. Ignored unless cfg.speed_on. Same buffering as gf_add_fix. */
int    gf_add_speed(gf_t *g, double t, double speed, double sigma, double window_s, unsigned flags);
/* GNSS fix. May arrive slightly before / after the odometry of the same time; fixes older than the node window are ignored. */
int    gf_add_fix(gf_t *g, const gf_fix *f);

/* Latest fused pose (causal: after every add; batch: after gf_solve_batch()). Returns 0, or -1 if no odometry yet. */
int    gf_get_pose(const gf_t *g, gf_pose *out);
/* Batch: initialise + optimise the whole graph. Returns 0, -2 if fewer than 3 usable fixes. */
int    gf_solve_batch(gf_t *g);
/* Fused pose for an odometry sample (t, p, q in the odometry frame of the node it belongs to): works for any retained time
 * (batch, or causal with keep_history). Returns 0, -1 if outside the retained range / uninitialised. */
int    gf_query(const gf_t *g, double t, const double p[3], const double q[4], gf_pose *out);

void   gf_get_stats(const gf_t *g, gf_stats *s);
/* Pose of node i if it is a GNSS-only node (made while the odometry was lost): returns 0 and fills `out`; 1 if node i has odometry (use gf_query),
 * -1 if out of range. Retained nodes only (batch, or causal with keep_history). */
int    gf_node_pose(const gf_t *g, int i, gf_pose *out);
/* Debug / diagnostics: node i -> time, state (psi, px, py, pz, s), distrust flag, has accepted fix. Returns -1 if i is out of range. */
int    gf_node(const gf_t *g, int i, double *t, double x[5], int *distrusted, int *fix_used);
/* Diagnostics of the consistency test for node i: window similarity-fit residual [m] and GNSS short-term noise estimate [m]. */
int    gf_node_diag(const gf_t *g, int i, double *rho, double *sig);

#define GF_ERR_ORDER   (-1)
#define GF_ERR_GRAVITY (-3)
#define GF_ERR_ALLOC   (-4)
#define GF_ERR_ARG     (-5)

#ifdef __cplusplus
}
#endif
#endif
