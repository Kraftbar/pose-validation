/* SPDX-License-Identifier: MIT */
/* stella_vio IMU leaf: data model, SO(3) helpers, on-manifold preintegration, gyro rotation prediction.
 *
 * Clean-room implementation of the preintegration of Forster, Carlone, Dellaert, Scaramuzza,
 * "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry", IEEE T-RO 2017 (arXiv:1512.02363):
 * right-perturbation SO(3), first-order bias Jacobians, covariance propagation in [dtheta, dv, dp] order.
 * No third-party code. Midpoint rule (trapezoid-averaged measurement, mid-interval rotation for the acceleration) per sample interval; OKVIS2 (ok_imu.c, BSD-3) is used
 * only as an independent cross-check in check_sv_imu.c, not as source.
 *
 * C99, libm only. Matrices are ROW-major double[9] (3x3) / double[81] (9x9). Time: int64 nanoseconds.
 * Conventions: R_WB maps body coords to world. Accelerometer measures specific force f = R_WB^T (a_W - g_W), with
 * g_W the physical gravity vector (0,0,-9.81 in a gravity-aligned world). Measurements: w = w_true + bg, f = f_true + ba.
 */
#ifndef SV_IMU_H
#define SV_IMU_H

#include <stdint.h>
#include <stddef.h>

typedef struct sv_imu_sample { int64_t t_ns; double gyr[3]; double acc[3]; } sv_imu_sample;

/* continuous-time noise densities: sigma_g [rad/s/sqrt(Hz)], sigma_a [m/s^2/sqrt(Hz)], sigma_bg [rad/s^2/sqrt(Hz)], sigma_ba [m/s^3/sqrt(Hz)] */
typedef struct sv_imu_noise { double sigma_g, sigma_a, sigma_bg, sigma_ba; double gravity; } sv_imu_noise;

/* time-ordered sample buffer; duplicates / backwards stamps are rejected and counted, long gaps are counted and make
 * preintegration across them fail (no silent interpolation over an outage) */
typedef struct sv_imu_buf {
    sv_imu_sample* s; size_t n, cap;
    int64_t max_gap_ns;
    long n_dup, n_back, n_gap;
} sv_imu_buf;

void sv_imu_buf_init(sv_imu_buf* b, int64_t max_gap_ns);
void sv_imu_buf_free(sv_imu_buf* b);
/* 0 stored, 1 duplicate stamp (dropped), 2 backwards stamp (dropped), -1 out of memory */
int sv_imu_buf_push(sv_imu_buf* b, const sv_imu_sample* s);
/* index of the last sample with t <= t_ns, or -1 */
long sv_imu_buf_find(const sv_imu_buf* b, int64_t t_ns);
/* linear interpolation at t_ns; 0 ok, -1 if t_ns is not bracketed by two stored samples */
int sv_imu_buf_interp(const sv_imu_buf* b, int64_t t_ns, sv_imu_sample* out);

/* ---- SO(3) / 3x3 helpers (row-major) ---- */
void sv_so3_exp(const double w[3], double R[9]);
void sv_so3_log(const double R[9], double w[3]);
void sv_so3_jr(const double w[3], double J[9]);       /* right Jacobian */
void sv_so3_jr_inv(const double w[3], double J[9]);   /* its inverse */
void sv_m3_mul(const double A[9], const double B[9], double C[9]);   /* C = A B (C may not alias) */
void sv_m3_tmul(const double A[9], const double B[9], double C[9]);  /* C = A^T B */
void sv_m3_mulv(const double A[9], const double v[3], double o[3]);  /* o = A v */
void sv_m3_tmulv(const double A[9], const double v[3], double o[3]); /* o = A^T v */
void sv_m3_hat(const double v[3], double K[9]);

/* ---- preintegration between two keyframes ---- */
typedef struct sv_imu_preint {
    double dt; int n;
    double dR[9], dv[3], dp[3];            /* at linearisation biases bg, ba */
    double cov[81];                        /* 9x9, order [dtheta, dv, dp] (right-perturbation) */
    double J_Rbg[9], J_vbg[9], J_vba[9], J_pbg[9], J_pba[9];
    double bg[3], ba[3];
} sv_imu_preint;

void sv_imu_preint_init(sv_imu_preint* p, const double bg[3], const double ba[3]);
/* add one interval of length dt [s] with measurement (gyr, acc) */
void sv_imu_preint_add(sv_imu_preint* p, const double gyr[3], const double acc[3], double dt, const sv_imu_noise* nz);
/* integrate buffer samples over [t0_ns, t1_ns] (endpoints interpolated). Returns number of intervals, -1 if not bracketed,
 * -2 if an interval is longer than the buffer's max gap. */
int sv_imu_preint_range(sv_imu_preint* p, const sv_imu_buf* b, int64_t t0_ns, int64_t t1_ns,
                        const double bg[3], const double ba[3], const sv_imu_noise* nz);
/* first-order bias update to (bg, ba): dR*Exp(J_Rbg dbg), dv + J_vbg dbg + J_vba dba, dp + ... */
void sv_imu_preint_corrected(const sv_imu_preint* p, const double bg[3], const double ba[3],
                             double dR[9], double dv[3], double dp[3]);
/* residual [r_theta, r_v, r_p] of states i, j (rotations R, velocities v, positions p, world gravity g) w.r.t. the interval */
void sv_imu_preint_residual(const sv_imu_preint* p, const double Ri[9], const double vi[3], const double pi_[3],
                            const double Rj[9], const double vj[3], const double pj[3], const double g[3],
                            const double bg[3], const double ba[3], double r[9]);
/* propagate state i to j through the interval (bias-corrected) */
void sv_imu_preint_predict(const sv_imu_preint* p, const double Ri[9], const double vi[3], const double pi_[3], const double g[3],
                           const double bg[3], const double ba[3], double Rj[9], double vj[3], double pj[3]);

/* ---- gyro-aided rotation prediction (rotation only, nonmetric) ---- */
/* body rotation R_B0^T R_B1 integrated from gyro over [t0, t1] (bias bg subtracted); returns intervals, <0 as above */
int sv_imu_gyro_rotation(const sv_imu_buf* b, int64_t t0_ns, int64_t t1_ns, const double bg[3], double dR_B[9]);
/* camera relative rotation R_C0^T R_C1 = R_CB dR_B R_BC; so R_WC1 = R_WC0 * result. R_BC maps camera coords into the body frame. */
int sv_imu_gyro_predict_cam(const sv_imu_buf* b, int64_t t0_ns, int64_t t1_ns, const double bg[3], const double R_BC[9], double R_C0C1[9]);
/* mean gyro over [t0, t1] (static-start bias seed); returns sample count */
int sv_imu_gyro_mean(const sv_imu_buf* b, int64_t t0_ns, int64_t t1_ns, double mean[3]);

/* mean accelerometer (specific force) over the samples in (t0, t1]; returns the sample count */
int sv_imu_acc_mean(const sv_imu_buf* b, int64_t t0, int64_t t1, double mean[3]);

#endif
