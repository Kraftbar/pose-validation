/* SPDX-License-Identifier: MIT */
/* Synthetic test of sv_imu_init: known scale, gravity direction, biases and velocities recovered from an up-to-scale keyframe trajectory.
 * Sweeps window length, adds noise (IMU and visual), and a degenerate case (constant-velocity straight line: scale unobservable)
 * where the estimator must report large sigma_log_scale instead of false confidence. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "sv_imu_init.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static const double G_W[3] = {0, 0, -9.81};
static unsigned long long rs = 1234567ULL;
static double urand(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return ((rs >> 11) + 0.5) / 9007199254740992.0; }
static double nrand(void) { return sqrt(-2.0 * log(urand())) * cos(6.283185307179586 * urand()); }

static int g_kind = 1;   /* 1 sinusoidal, 2 straight constant velocity (degenerate), 3 weak (small amplitude) */
static void pose_at(double t, double R[9], double p[3], double v[3], double a[3]) {
    double ph[3]; int k;
    if (g_kind == 2) { ph[0] = 0.05 * sin(0.3 * t); ph[1] = 0.04 * sin(0.2 * t); ph[2] = 0.03 * sin(0.25 * t); }
    else { ph[0] = 0.5 * sin(0.9 * t); ph[1] = 0.35 * sin(1.3 * t + 1.0); ph[2] = 0.7 * sin(0.5 * t + 2.0); }
    sv_so3_exp(ph, R);
    if (g_kind == 2) { p[0] = 1.2 * t; p[1] = 0.3 * t; p[2] = 0; v[0] = 1.2; v[1] = 0.3; v[2] = 0; for (k = 0; k < 3; ++k) a[k] = 0; return; }
    {
        double A = (g_kind == 3) ? 0.15 : 1.0;
        p[0] = A * 1.0 * sin(0.7 * t);       v[0] = A * 0.7 * cos(0.7 * t);        a[0] = -A * 0.49 * sin(0.7 * t);
        p[1] = A * 0.8 * sin(1.1 * t + 0.5); v[1] = A * 0.88 * cos(1.1 * t + 0.5); a[1] = -A * 0.968 * sin(1.1 * t + 0.5);
        p[2] = A * 0.5 * sin(0.9 * t + 1.0); v[2] = A * 0.45 * cos(0.9 * t + 1.0); a[2] = -A * 0.405 * sin(0.9 * t + 1.0);
    }
}
static void gyro_at(double t, double w[3]) {
    double Ra[9], Rb[9], D[9], p[3], v[3], a[3], h = 1e-5, l[3]; int k;
    pose_at(t - 0.5 * h, Ra, p, v, a); pose_at(t + 0.5 * h, Rb, p, v, a);
    sv_m3_tmul(Ra, Rb, D); sv_so3_log(D, l);
    for (k = 0; k < 3; ++k) w[k] = l[k] / h;
}

static double angle_deg(const double a[3], const double b[3]) {
    double d = (a[0]*b[0]+a[1]*b[1]+a[2]*b[2]) / (sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]) * sqrt(b[0]*b[0]+b[1]*b[1]+b[2]*b[2]));
    if (d > 1) d = 1;
    if (d < -1) d = -1;
    return acos(d) * 57.29577951308232;
}

static void run(const char* name, double window, double kf_dt, double noise_scale, int est_ba, int expect_ok) {
    const double S_true = 0.37, bgt[3] = {0.012, -0.008, 0.02}, bat[3] = {0.08, -0.12, 0.05}, p_BC[3] = {0.03, -0.02, 0.05};
    const double R_BC[9] = {0, 0, 1, -1, 0, 0, 0, -1, 0};
    double phiVW[3] = {0.3, -0.5, 0.8}, R_VW[9], hz = 200;
    sv_imu_buf b; sv_vi_kf* kf; sv_vi_ext ext; sv_imu_noise nz = {2e-3, 2e-2, 1e-5, 1e-4, 9.81};
    sv_vi_cfg cfg; sv_vi_result res; long i, ns = (long)((window + 1.0) * hz) + 1; int n = (int)(window / kf_dt) + 1, k, j, rc;
    double gV_true[3], verr = 0, vmag = 0;
    int64_t t0 = 3000000000LL;
    sv_so3_exp(phiVW, R_VW); sv_m3_mulv(R_VW, G_W, gV_true);
    memcpy(ext.R_BC, R_BC, sizeof R_BC); memcpy(ext.p_BC, p_BC, sizeof p_BC);
    sv_imu_buf_init(&b, 0);
    for (i = 0; i < ns; ++i) {
        double t = (double)i / hz, R[9], p[3], v[3], a[3], u[3], f[3]; sv_imu_sample s;
        pose_at(t, R, p, v, a); gyro_at(t, s.gyr);
        for (k = 0; k < 3; ++k) u[k] = a[k] - G_W[k];
        sv_m3_tmulv(R, u, f);
        for (k = 0; k < 3; ++k) {
            s.gyr[k] += bgt[k] + noise_scale * nz.sigma_g * sqrt(hz) * nrand();
            s.acc[k] = f[k] + bat[k] + noise_scale * nz.sigma_a * sqrt(hz) * nrand();
        }
        s.t_ns = t0 + (int64_t)llround(t * 1e9); sv_imu_buf_push(&b, &s);
    }
    kf = (sv_vi_kf*)calloc((size_t)n, sizeof *kf);
    for (j = 0; j < n; ++j) {
        double t = 0.3 + j * kf_dt, R[9], p[3], v[3], a[3], T[9], pc[3], Rp[3], RWC[9], pw[3];
        pose_at(t, R, p, v, a);
        sv_m3_mul(R, R_BC, RWC);                 /* R_WC */
        sv_m3_mulv(R, p_BC, Rp);
        for (k = 0; k < 3; ++k) pw[k] = p[k] + Rp[k];       /* camera centre in W (metres) */
        sv_m3_mulv(R_VW, pw, pc);
        sv_m3_mul(R_VW, RWC, T);
        kf[j].t_ns = t0 + (int64_t)llround(t * 1e9);
        memcpy(kf[j].R_VC, T, sizeof T);
        for (k = 0; k < 3; ++k) kf[j].c[k] = pc[k] / S_true + noise_scale * 0.005 / S_true * nrand();   /* 5 mm visual position noise */
        if (noise_scale > 0) { double w[3] = {noise_scale * 0.002 * nrand(), noise_scale * 0.002 * nrand(), noise_scale * 0.002 * nrand()}, E[9], T2[9];
            sv_so3_exp(w, E); sv_m3_mul(kf[j].R_VC, E, T2); memcpy(kf[j].R_VC, T2, sizeof T2); }
    }
    sv_vi_cfg_default(&cfg); cfg.estimate_ba = est_ba;
    rc = sv_vi_init(&b, kf, n, &ext, &nz, &cfg, &res);
    if (rc == 0 || rc == -4) {
        for (j = 0; j < n; ++j) {
            double t = 0.3 + j * kf_dt, R[9], p[3], v[3], a[3], vv[3];
            pose_at(t, R, p, v, a); sv_m3_mulv(R_VW, v, vv);
            if (res.v) for (k = 0; k < 3; ++k) { verr += (res.v[3 * j + k] - vv[k]) * (res.v[3 * j + k] - vv[k]); vmag += vv[k] * vv[k]; }
        }
        verr = sqrt(verr / n); vmag = sqrt(vmag / n);
        printf("  %-34s win %4.1fs n=%2d rc=%d ok=%d  scale %.4f (true %.2f, lin %.4f, err %+5.1f%%) sigma_logS %.3f  grav err %.2f deg (sigma %.2f)  bg err %.4f  ba err %.3f  v rms err %.3f (|v| %.2f)\n",
               name, window, n, rc, res.ok, res.scale, S_true, res.scale_linear, 100.0 * (res.scale / S_true - 1), res.sigma_log_scale, angle_deg(res.g_V, gV_true), res.sigma_grav_deg,
               sqrt((res.bg[0]-bgt[0])*(res.bg[0]-bgt[0])+(res.bg[1]-bgt[1])*(res.bg[1]-bgt[1])+(res.bg[2]-bgt[2])*(res.bg[2]-bgt[2])),
               sqrt((res.ba[0]-bat[0])*(res.ba[0]-bat[0])+(res.ba[1]-bat[1])*(res.ba[1]-bat[1])+(res.ba[2]-bat[2])*(res.ba[2]-bat[2])), verr, vmag);
        if (expect_ok == 1) {
            CHECK(rc == 0 && res.ok, "expected an accepted estimate");
            CHECK(fabs(res.scale / S_true - 1) < (noise_scale > 0 ? 0.06 : 0.02), "scale error too large");
            CHECK(angle_deg(res.g_V, gV_true) < (noise_scale > 0 ? 2.0 : 1.0), "gravity error too large");
        } else if (expect_ok == 0) {
            CHECK(!res.ok || res.sigma_log_scale > 0.1, "degenerate window must not be accepted with confidence (ok=%d sigma %.3f)", res.ok, res.sigma_log_scale);
        }
    } else { printf("  %-34s win %4.1fs rc=%d\n", name, window, rc); CHECK(expect_ok == 0 || expect_ok == 2, "init failed rc=%d", rc); }
    sv_vi_result_free(&res); free(kf); sv_imu_buf_free(&b);
}

int main(void) {
    double w; 
    printf("synthetic visual-inertial initialisation (scale 0.37, gravity rotated into V, bg/ba nonzero, KF every 0.5 s)\n");
    g_kind = 1;
    printf("[a] noise-free, ba estimated\n");
    for (w = 1.0; w <= 8.0; w += (w < 3 ? 1.0 : 2.5)) run("sinusoidal, noise-free", w, 0.5, 0.0, 1, w >= 3.0 ? 1 : 2);
    printf("[b] noisy (IMU noise densities, 5 mm / 0.002 rad visual noise)\n");
    for (w = 2.0; w <= 12.0; w += (w < 4 ? 1.0 : 2.0)) { rs = 99 + (unsigned long long)w; run("sinusoidal, noisy", w, 0.5, 1.0, 1, w >= 4.0 ? 1 : 2); }
    printf("[c] ba not estimated (ba true nonzero: biased)\n");
    run("sinusoidal, noisy, ba off", 6.0, 0.5, 1.0, 0, 2);
    printf("[d] degenerate: constant-velocity straight line with tiny rotation (scale unobservable)\n");
    g_kind = 2; run("straight constant velocity", 6.0, 0.5, 1.0, 1, 0);
    printf("[e] weak excitation (amplitude 0.15 of case a)\n");
    g_kind = 3; run("weak sinusoid", 6.0, 0.5, 1.0, 1, 2);
    printf(g_fail ? "\nFAILED (%d)\n" : "\nALL PASS\n", g_fail);
    return g_fail != 0;
}
