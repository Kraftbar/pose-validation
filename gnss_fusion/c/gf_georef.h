/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 *
 * gf_georef.h : slowly varying geo-referencing of a metric pose stream (section 14 of docs/gnss_vio_benchmark_20261001.md).
 *
 * The input stream is a continuous, locally accurate, (approximately) metric position stream in an arbitrary frame (the fix-free output of gf_fusion with the gait
 * speed prior, or a VIO). The phone GNSS fixes are NOT used to bend it: the only thing they determine is one 2-D similarity (yaw psi, scale s, translation t) plus a
 * vertical offset, estimated from ALL pairs (stream position at the fix time, fix) seen so far (long memory, optional exponential forgetting), so fixes only
 * geo-reference the stream and correct its global scale / drift, never its shape.
 *   z_i ~ s Rz(psi) (p_i - mp) + mz            (stream p_i interpolated at the fix time t_i)
 *   psi    : 2-D Procrustes angle of the (weighted, optionally Huber re-weighted) pairs
 *   s      : ridge-shrunk towards 1 (a gait-scaled stream is metric to ~15 %):  s = (S_cr/k + lam) / (S_aa/k + lam),  lam = sigma_res^2 / sigma_s^2,
 *            sigma_res = rms residual of the rigid fit (online noise calibration), k = max(1, corr_s * fix_rate) = number of fixes that are one independent sample
 *            (phone fixes carry a slowly varying common error with a correlation time of ~40 s, so n fixes are only n / k independent measurements)
 * Causal use: gf_georef_add_pose() for every stream sample, gf_georef_add_fix() for every fix (in time order, a fix right after the stream sample at or after its time),
 * gf_georef_map() maps the CURRENT stream sample with the CURRENT fit (what a live consumer sees). Batch use: add everything, gf_georef_solve(), map.
 */
#ifndef GF_GEOREF_H
#define GF_GEOREF_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct gf_georef_config {
    int    min_fixes;      /* pairs needed before a fit is produced (8) */
    double min_extent_m;   /* extent (bbox dx + dy) of the track needed for the yaw to be defined: of the stream if scale_sigma <= 10 (metres), of the fixes if the stream is monocular / free scale (30 m) */
    double scale_sigma;    /* sigma_s, prior spread of the stream scale; 0 -> scale fixed to 1 (rigid); 100 -> practically free (0.15) */
    double corr_s;         /* correlation time of the fix errors [s]; 1 -> treat fixes as independent (40) */
    double forget_s;       /* exponential forgetting time of old pairs [s], 0 = none (0) */
    double huber_k;        /* Huber threshold in units of the residual rms of the fit (IRLS, 3 passes), 0 = off (2.5) */
    double sigma_floor;    /* floor on reported fix sigmas used for the relative weights [m] (0.5) */
    int    use_sigma;      /* 1: pairs weighted by 1/sigma_h^2 (relative), 0: equal weights (0: the reported phone sigmas carry no information, section 14) */
    int    max_pairs;      /* pairs kept (oldest quarter dropped when full), 8192 */
    double max_gap_s;      /* a fix is only paired when the stream samples around its time are at most this far apart (holes = tracking loss), 2.5 */
} gf_georef_config;

typedef struct gf_georef gf_georef;

void        gf_georef_config_default(gf_georef_config *c);
gf_georef  *gf_georef_create(const gf_georef_config *c);   /* NULL on allocation failure */
void        gf_georef_destroy(gf_georef *g);
/* Stream sample (times increasing, any rate). Keeps a short ring for interpolation at the fix times. */
int         gf_georef_add_pose(gf_georef *g, double t, const double p[3]);
/* Fix (ENU position, reported horizontal sigma). Pairs it with the stream at t (nearest samples in the ring) and, causal mode, refits. Returns 1 if a fit is available. */
int         gf_georef_add_fix(gf_georef *g, double t, const double z[3], double sigma_h, int refit);
/* Fit over all pairs now (batch). Returns 1 on success. */
int         gf_georef_solve(gf_georef *g);
/* Apply the current fit. Returns 0 if there is none yet (p_out untouched). yaw/scale/ok in fit are optional outputs. */
int         gf_georef_map(const gf_georef *g, const double p[3], double p_out[3]);
int         gf_georef_fit(const gf_georef *g, double *psi, double *scale, double *sigma_res, int *n_pairs);

#ifdef __cplusplus
}
#endif
#endif
