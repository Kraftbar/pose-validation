/* SPDX-License-Identifier: MIT */
/* Offline-testable visual-inertial initialisation: from an up-to-scale visual keyframe trajectory and raw IMU, estimate gyro bias,
 * metric scale, gravity direction, per-keyframe velocities and (optionally) accelerometer bias.
 *
 * Method (own formulation from the preintegration relations of Forster et al. 2017, see sv_imu.h):
 *   1. gyro bias: Gauss-Newton on the rotation residuals Log(dR_ij(bg)^T R_i^T R_j) (first-order Jacobian, re-integrate, repeat);
 *   2. closed form: with S = scale, g (3 free), v_i (3 per keyframe) the position and velocity relations over consecutive keyframes are
 *      LINEAR:  S (c_j - c_i) - dt v_i - dt^2/2 g = R_i dp + (R_j - R_i) p_BC ,   v_j - v_i - dt g = R_i dv ;
 *   3. refinement: Gauss-Newton with |g| = gravity fixed (2-dof tangent update, renormalised), log-scale, gyro bias (first-order),
 *      accelerometer bias (optional, weak prior), all velocities; weights from the preintegration covariance plus noise floors for
 *      the visual positions; reports the Gauss-Newton covariance of log-scale and gravity direction.
 * Frames: keyframe pose = camera in the visual world V: R_VC (3x3 row-major), camera centre c (visual units). Body (IMU) = camera
 * through the calibrated extrinsic R_BC, p_BC (camera origin in body frame, metres, NOT scaled). Body position in metric V-axes:
 * P_i = S c_i - R_VB,i p_BC. Gravity g_V is the physical vector (points down) in V axes, norm = noise->gravity.
 * Initial state is NOT assumed static; the window must contain acceleration/rotation excitation for scale to be observable: the
 * returned sigma_log_scale / sigma_grav_deg tell whether it was.
 * C99, libm only. */
#ifndef SV_IMU_INIT_H
#define SV_IMU_INIT_H

#include <stdint.h>
#include "sv_imu.h"

typedef struct sv_vi_kf { int64_t t_ns; double R_VC[9]; double c[3]; } sv_vi_kf;
typedef struct sv_vi_ext { double R_BC[9]; double p_BC[3]; } sv_vi_ext;

typedef struct sv_vi_cfg {
    int estimate_ba;            /* 0: ba fixed to 0 */
    int max_iter;
    double sigma_pos_floor;     /* [m]  visual position noise added to the preintegration covariance */
    double sigma_vel_floor;     /* [m/s] */
    double sigma_rot_floor;     /* [rad] */
    double sigma_bg_prior;      /* [rad/s] zero-mean prior */
    double sigma_ba_prior;      /* [m/s^2] */
    double max_sigma_log_scale; /* acceptance gate */
    double max_sigma_grav_deg;
} sv_vi_cfg;

typedef struct sv_vi_result {
    int ok;                     /* gates passed (finite, scale > 0, sigmas below the cfg limits, plausible biases) */
    int n_kf; double span_s;
    double scale;               /* metres per visual unit */
    double scale_linear;        /* closed-form (step 2) value before refinement */
    double sigma_log_scale;     /* sqrt(Var(log S)) from the final Gauss-Newton system (scaled by sqrt(max(1, chi2/dof))) */
    double g_V[3];              /* gravity vector in visual axes [m/s^2] */
    double sigma_grav_deg;      /* sqrt(sum of the two tangent variances) in degrees */
    double bg[3], ba[3];
    double rms_rot_before, rms_rot_after;   /* rotation residual rms [rad] at bg=0 and at the estimate */
    double rms_pos, rms_vel;                /* final residual rms [m], [m/s] */
    double chi2_dof;
    double* v;                  /* n_kf x 3 velocities in V axes, metric; free with sv_vi_result_free */
} sv_vi_result;

void sv_vi_cfg_default(sv_vi_cfg* c);
/* 0 = estimate computed (look at r->ok), -1 too few keyframes, -2 IMU not bracketed / outage, -3 singular system, -4 non-positive scale
 * in the closed form (r still holds the closed-form numbers where available), -5 out of memory */
int sv_vi_init(const sv_imu_buf* b, const sv_vi_kf* kf, int n, const sv_vi_ext* ext, const sv_imu_noise* nz, const sv_vi_cfg* cfg, sv_vi_result* r);
void sv_vi_result_free(sv_vi_result* r);
/* step 1 alone: refine bg (in/out) from the keyframe rotations; rms of the rotation residual before/after; returns 0 or <0 as above */
int sv_vi_gyro_bias(const sv_imu_buf* b, const sv_vi_kf* kf, int n, const sv_vi_ext* ext, const sv_imu_noise* nz, double bg[3], double* rms_before, double* rms_after);

#endif
