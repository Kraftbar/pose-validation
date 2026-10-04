/* SPDX-License-Identifier: MIT */
/* Unit tests for sv_imu.{h,c}: synthetic trajectories vs analytic deltas, bias correction, Jacobians (finite differences),
 * covariance (symmetry, PSD, Monte Carlo consistency), gyro prediction, and an independent cross-check against the OKVIS2 port
 * (okvis_port/c/ok_imu.c, BSD-3 + MPL-2.0 helpers; used only as a reference oracle here).
 * Build: see sv_imu.mk.  Exit code 0 = all pass.  Prints a table of the measured errors. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "sv_imu.h"
#include "ok_imu.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const double G_W[3] = {0, 0, -9.81};

/* ---- synthetic trajectory: R(t) = Exp(phi(t)), p(t) analytic ---- */
typedef struct { int kind; } traj_t;   /* 0 = constant rotation, static; 1 = sinusoidal rotation + translation */
static void phi_at(int kind, double t, double ph[3]) {
    if (kind == 0) { ph[0] = 0.30 * t; ph[1] = -0.20 * t; ph[2] = 0.45 * t; }
    else { ph[0] = 0.6 * sin(0.9 * t); ph[1] = 0.4 * sin(1.3 * t + 1.0); ph[2] = 0.8 * sin(0.5 * t + 2.0); }
}
static void pose_at(int kind, double t, double R[9], double p[3], double v[3], double a[3]) {
    double ph[3];
    phi_at(kind, t, ph); sv_so3_exp(ph, R);
    if (kind == 0) { memset(p, 0, 3 * sizeof(double)); memset(v, 0, 3 * sizeof(double)); memset(a, 0, 3 * sizeof(double)); return; }
    p[0] = 1.0 * sin(0.7 * t);       v[0] = 0.7 * cos(0.7 * t);        a[0] = -0.49 * sin(0.7 * t);
    p[1] = 0.8 * sin(1.1 * t + 0.5); v[1] = 0.88 * cos(1.1 * t + 0.5); a[1] = -0.968 * sin(1.1 * t + 0.5);
    p[2] = 0.5 * sin(0.9 * t + 1.0); v[2] = 0.45 * cos(0.9 * t + 1.0); a[2] = -0.405 * sin(0.9 * t + 1.0);
}
static void gyro_at(int kind, double t, double w[3]) { /* body angular velocity by symmetric differencing of R(t) */
    double Ra[9], Rb[9], D[9], p[3], v[3], a[3], h = 1e-5, l[3]; int k;
    pose_at(kind, t - 0.5 * h, Ra, p, v, a); pose_at(kind, t + 0.5 * h, Rb, p, v, a);
    sv_m3_tmul(Ra, Rb, D); sv_so3_log(D, l);
    for (k = 0; k < 3; ++k) w[k] = l[k] / h;
}
static sv_imu_sample sample_at(int kind, double t, int64_t t_ns, const double bg[3], const double ba[3]) {
    sv_imu_sample s; double R[9], p[3], v[3], a[3], u[3], f[3]; int k;
    pose_at(kind, t, R, p, v, a); gyro_at(kind, t, s.gyr);
    for (k = 0; k < 3; ++k) u[k] = a[k] - G_W[k];
    sv_m3_tmulv(R, u, f);
    for (k = 0; k < 3; ++k) { s.gyr[k] += bg[k]; s.acc[k] = f[k] + ba[k]; }
    s.t_ns = t_ns;
    return s;
}
static void fill(sv_imu_buf* b, int kind, double hz, double T, const double bg[3], const double ba[3], int64_t base) {
    long i, n = (long)(T * hz) + 1;
    sv_imu_buf_init(b, 0);
    for (i = 0; i < n; ++i) {
        int64_t tn = (int64_t)llround((double)i / hz * 1e9);
        sv_imu_sample s = sample_at(kind, (double)tn * 1e-9, tn + base, bg, ba);
        sv_imu_buf_push(b, &s);
    }
}
static void truth_delta(int kind, double t0, double t1, double dR[9], double dv[3], double dp[3]) {
    double Ri[9], Rj[9], pi[3], pj[3], vi[3], vj[3], a[3], u[3]; int k; double dt = t1 - t0;
    pose_at(kind, t0, Ri, pi, vi, a); pose_at(kind, t1, Rj, pj, vj, a);
    sv_m3_tmul(Ri, Rj, dR);
    for (k = 0; k < 3; ++k) u[k] = vj[k] - vi[k] - G_W[k] * dt;
    sv_m3_tmulv(Ri, u, dv);
    for (k = 0; k < 3; ++k) u[k] = pj[k] - pi[k] - vi[k] * dt - 0.5 * G_W[k] * dt * dt;
    sv_m3_tmulv(Ri, u, dp);
}
static double rot_err(const double A[9], const double B[9]) { double D[9], w[3]; sv_m3_tmul(A, B, D); sv_so3_log(D, w); return sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]); }
static double vdist(const double a[3], const double b[3]) { return sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2])); }

/* ---- 1. analytic comparison, rate sweep ---- */
static void test_analytic(void) {
    static const double z[3] = {0, 0, 0};
    int kind, ri, ti;
    double rates[] = {100, 200, 400, 1000}, spans[] = {0.5, 1.0, 5.0};
    printf("[1] preintegration vs analytic (no bias, no noise)\n");
    printf("  %-18s %6s %5s  %12s %12s %12s\n", "trajectory", "Hz", "T[s]", "rot err[rad]", "dv err[m/s]", "dp err[m]");
    for (kind = 0; kind < 2; ++kind) for (ti = 0; ti < 3; ++ti) {
        double prev_r = 0;
        for (ri = 0; ri < 4; ++ri) {
            sv_imu_buf b; sv_imu_preint p; sv_imu_noise nz = {0, 0, 0, 0, 9.81};
            double er, ev, ep, t0 = 0.3;
            int64_t base = 100000000000LL;
            fill(&b, kind, rates[ri], t0 + spans[ti] + 0.3, z, z, base);
            CHECK(sv_imu_preint_range(&p, &b, base + (int64_t)llround(t0 * 1e9), base + (int64_t)llround((t0 + spans[ti]) * 1e9), z, z, &nz) > 0, "range failed");
            { double tR[9], tv[3], tp[3]; truth_delta(kind, t0, t0 + spans[ti], tR, tv, tp);
              er = rot_err(p.dR, tR); ev = vdist(p.dv, tv); ep = vdist(p.dp, tp); }
            printf("  %-18s %6.0f %5.1f  %12.3e %12.3e %12.3e\n", kind ? "sinusoidal" : "const rotation", rates[ri], spans[ti], er, ev, ep);
            if (kind == 0) CHECK(er < 1e-9, "constant rotation must be exact (%g)", er);
            if (ri == 1 && spans[ti] == 1.0) { CHECK(er < 5e-4 && ev < 5e-3 && ep < 2e-3, "200 Hz 1 s too coarse"); }
            if (ri == 3) { CHECK(er < 2e-5 && ev < 3e-4 && ep < 1e-4, "1 kHz error too large"); }
            if (kind == 1 && ri > 0 && prev_r > 1e-9) CHECK(er < prev_r, "rotation error must shrink with rate");
            prev_r = er;
            sv_imu_buf_free(&b);
        }
    }
}

/* ---- 2/3. bias correction and Jacobians ---- */
static void test_bias(void) {
    static const double z[3] = {0, 0, 0};
    const double bgt[3] = {0.02, -0.01, 0.03}, bat[3] = {0.10, -0.20, 0.05};
    sv_imu_buf b; sv_imu_preint p0, p1; sv_imu_noise nz = {0, 0, 0, 0, 9.81};
    double dR[9], dv[3], dp[3], tR[9], tv[3], tp[3], eR, ev, ep, e0R, e0v, e0p;
    int64_t base = 5000000000LL, t0 = base + 300000000LL, t1 = base + 2300000000LL;
    int k, m;
    printf("[2] bias: preintegrate at 0, correct to true bias (bg=%.2f,%.2f,%.2f ba=%.2f,%.2f,%.2f) vs truth / re-integration\n", bgt[0], bgt[1], bgt[2], bat[0], bat[1], bat[2]);
    fill(&b, 1, 200, 2.7, bgt, bat, base);
    sv_imu_preint_range(&p0, &b, t0, t1, z, z, &nz);
    sv_imu_preint_range(&p1, &b, t0, t1, bgt, bat, &nz);
    truth_delta(1, 0.3, 2.3, tR, tv, tp);
    sv_imu_preint_corrected(&p0, bgt, bat, dR, dv, dp);
    eR = rot_err(dR, tR); ev = vdist(dv, tv); ep = vdist(dp, tp);
    e0R = rot_err(p0.dR, tR); e0v = vdist(p0.dv, tv); e0p = vdist(p0.dp, tp);
    printf("  uncorrected          rot %.3e  dv %.3e  dp %.3e\n", e0R, e0v, e0p);
    printf("  first-order corrected rot %.3e  dv %.3e  dp %.3e\n", eR, ev, ep);
    printf("  re-integrated        rot %.3e  dv %.3e  dp %.3e\n", rot_err(p1.dR, tR), vdist(p1.dv, tv), vdist(p1.dp, tp));
    CHECK(eR < 0.05 * e0R && ev < 0.15 * e0v && ep < 0.15 * e0p, "first-order bias correction should remove >85%% of the bias error");
    printf("  first-order vs re-integrated: rot %.2e  dv %.2e  dp %.2e (second-order in bias step: |dbg| %.3f rad/s over 2 s)\n", rot_err(dR, p1.dR), vdist(dv, p1.dv), vdist(dp, p1.dp), sqrt(bgt[0]*bgt[0]+bgt[1]*bgt[1]+bgt[2]*bgt[2]));
    CHECK(rot_err(dR, p1.dR) < 1e-3 && vdist(dv, p1.dv) < 5e-2 && vdist(dp, p1.dp) < 3e-2, "corrected deltas far from re-integration");
    /* Jacobians vs finite differences of the discrete model */
    printf("[3] bias Jacobians vs central finite differences (eps 1e-5), max abs error over all columns\n");
    {
        double maxR = 0, maxvg = 0, maxpg = 0, maxva = 0, maxpa = 0, eps = 1e-5;
        for (m = 0; m < 3; ++m) {
            double bp[3] = {0, 0, 0}, bm[3] = {0, 0, 0}, ap[3] = {0, 0, 0}, am[3] = {0, 0, 0}, w[3], D[9];
            sv_imu_preint a, c; double col[3];
            bp[m] = eps; bm[m] = -eps; ap[m] = eps; am[m] = -eps;
            sv_imu_preint_range(&a, &b, t0, t1, bp, z, &nz); sv_imu_preint_range(&c, &b, t0, t1, bm, z, &nz);
            sv_m3_tmul(c.dR, a.dR, D); sv_so3_log(D, w);   /* = Jr-type difference of dR at +/-eps; compare J_Rbg e_m * 2 eps */
            for (k = 0; k < 3; ++k) col[k] = w[k] / (2 * eps);
            { double dRm[9], E[9], t[3], w2[3], X[9];
              for (k = 0; k < 3; ++k) t[k] = p0.J_Rbg[3 * k + m] * (-eps);
              sv_so3_exp(t, E); sv_m3_mul(p0.dR, E, dRm);
              for (k = 0; k < 3; ++k) t[k] = p0.J_Rbg[3 * k + m] * eps;
              sv_so3_exp(t, E); sv_m3_mul(p0.dR, E, X);
              sv_m3_tmul(dRm, X, E); sv_so3_log(E, w2);
              for (k = 0; k < 3; ++k) { double e = fabs(w2[k] / (2 * eps) - col[k]); if (e > maxR) maxR = e; } }
            for (k = 0; k < 3; ++k) {
                double e;
                e = fabs((a.dv[k] - c.dv[k]) / (2 * eps) - p0.J_vbg[3 * k + m]); if (e > maxvg) maxvg = e;
                e = fabs((a.dp[k] - c.dp[k]) / (2 * eps) - p0.J_pbg[3 * k + m]); if (e > maxpg) maxpg = e;
            }
            sv_imu_preint_range(&a, &b, t0, t1, z, ap, &nz); sv_imu_preint_range(&c, &b, t0, t1, z, am, &nz);
            for (k = 0; k < 3; ++k) {
                double e;
                e = fabs((a.dv[k] - c.dv[k]) / (2 * eps) - p0.J_vba[3 * k + m]); if (e > maxva) maxva = e;
                e = fabs((a.dp[k] - c.dp[k]) / (2 * eps) - p0.J_pba[3 * k + m]); if (e > maxpa) maxpa = e;
            }
        }
        printf("  J_Rbg %.2e  J_vbg %.2e  J_pbg %.2e  J_vba %.2e  J_pba %.2e (magnitudes: dv/dbg up to %.2f, dp/dbg up to %.2f)\n", maxR, maxvg, maxpg, maxva, maxpa,
               fabs(p0.J_vbg[0]) + fabs(p0.J_vbg[4]) + fabs(p0.J_vbg[8]), fabs(p0.J_pbg[0]) + fabs(p0.J_pbg[4]) + fabs(p0.J_pbg[8]));
        CHECK(maxR < 1e-4 && maxvg < 1e-4 && maxpg < 1e-4 && maxva < 1e-6 && maxpa < 1e-6, "Jacobian mismatch");
    }
    sv_imu_buf_free(&b);
}

/* ---- 4. covariance ---- */
static int chol9(const double* A, double* L) {
    int i, j, k;
    for (i = 0; i < 9; ++i) for (j = 0; j <= i; ++j) {
        double s = A[9 * i + j];
        for (k = 0; k < j; ++k) s -= L[9 * i + k] * L[9 * j + k];
        if (i == j) { if (s <= 0) return 0; L[9 * i + i] = sqrt(s); } else L[9 * i + j] = s / L[9 * j + j];
    }
    return 1;
}
static double maha9(const double* L, const double r[9]) {
    double y[9], s = 0; int i, k;
    for (i = 0; i < 9; ++i) { double t = r[i]; for (k = 0; k < i; ++k) t -= L[9 * i + k] * y[k]; y[i] = t / L[9 * i + i]; s += y[i] * y[i]; }
    return s;
}
static unsigned long long rs = 88172645463325252ULL;
static double urand(void) { rs ^= rs << 13; rs ^= rs >> 7; rs ^= rs << 17; return ((rs >> 11) + 0.5) / 9007199254740992.0; }
static double nrand(void) { return sqrt(-2.0 * log(urand())) * cos(6.283185307179586 * urand()); }

static void test_cov(void) {
    static const double z[3] = {0, 0, 0};
    sv_imu_buf b0; sv_imu_noise nz = {2e-3, 2e-2, 0, 0, 9.81};
    double hz = 100, T = 1.0, dt = 1.0 / hz;
    int trials = 4000, tr, i, j, k;
    double L[81], sum = 0, var[9] = {0}, mean[9] = {0}, Pd[9];
    sv_imu_preint pn; int64_t base = 1000000000LL;
    printf("[4] covariance: symmetry / PSD and Monte Carlo (%d trials, %.0f Hz, %.1f s, sigma_g %.0e sigma_a %.0e)\n", trials, hz, T, nz.sigma_g, nz.sigma_a);
    fill(&b0, 1, hz, T + 0.2, z, z, base);
    sv_imu_preint_range(&pn, &b0, base + 100000000LL, base + 100000000LL + (int64_t)(T * 1e9), z, z, &nz);
    { double mx = 0; for (i = 0; i < 9; ++i) for (j = 0; j < 9; ++j) mx = fmax(mx, fabs(pn.cov[9 * i + j] - pn.cov[9 * j + i]));
      CHECK(mx < 1e-18, "covariance not symmetric (%g)", mx); }
    CHECK(chol9(pn.cov, L), "covariance not positive definite");
    for (tr = 0; tr < trials; ++tr) {
        sv_imu_preint p; double Ri[9], Rj[9], pi_[3], pj[3], vi[3], vj[3], a[3], r[9], t0 = 0.1, t1 = 0.1 + T;
        /* independent noise on the interval-mean measurement (what the 'add' model assumes) */
        sv_imu_preint_init(&p, z, z);
        for (i = 10; i < 110; ++i) {
            double g[3], ac[3];
            for (k = 0; k < 3; ++k) {
                g[k] = 0.5 * (b0.s[i].gyr[k] + b0.s[i + 1].gyr[k]) + nz.sigma_g / sqrt(dt) * nrand();
                ac[k] = 0.5 * (b0.s[i].acc[k] + b0.s[i + 1].acc[k]) + nz.sigma_a / sqrt(dt) * nrand();
            }
            sv_imu_preint_add(&p, g, ac, dt, &nz);
        }
        pose_at(1, t0, Ri, pi_, vi, a); pose_at(1, t1, Rj, pj, vj, a);
        sv_imu_preint_residual(&p, Ri, vi, pi_, Rj, vj, pj, G_W, z, z, r);
        sum += maha9(L, r);
        for (k = 0; k < 9; ++k) { mean[k] += r[k]; var[k] += r[k] * r[k]; }
    }
    for (k = 0; k < 9; ++k) { mean[k] /= trials; var[k] = var[k] / trials - mean[k] * mean[k]; Pd[k] = pn.cov[10 * k]; }
    printf("  mean Mahalanobis^2 = %.3f (expect 9)\n  component  sample-sigma / predicted-sigma:", sum / trials);
    for (k = 0; k < 9; ++k) printf(" %.3f", sqrt(var[k] / Pd[k]));
    printf("\n");
    CHECK(fabs(sum / trials - 9.0) < 0.6, "Monte Carlo chi2 mean %g", sum / trials);
    for (k = 0; k < 9; ++k) CHECK(sqrt(var[k] / Pd[k]) > 0.9 && sqrt(var[k] / Pd[k]) < 1.1, "component %d sigma ratio %g", k, sqrt(var[k] / Pd[k]));
    sv_imu_buf_free(&b0);
}

/* ---- 5. OKVIS2 cross-check ---- */
static void test_okvis(void) {
    static const double z[3] = {0, 0, 0};
    sv_imu_buf b; sv_imu_preint p; sv_imu_noise nz = {1e-3, 1e-2, 1e-4, 1e-3, 9.81};
    ok_imu_params prm = {1e-3, 1e-2, 1e-4, 1e-3, 9.81, 1e9, 1e9};
    ok_imu_meas m[1024]; ok_imu_error e; double sb[9] = {0};
    int64_t base = 200000000000LL; long i, n; int k, c, r;
    double dRok[9], Rtmp[9], maxdR, maxdv, maxdp, maxJ[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
    ok_quat q;
    printf("[5] cross-check vs OKVIS2 port ok_imu (midpoint integration), sinusoidal, 200 Hz, 1 s\n");
    fill(&b, 1, 200, 1.5, z, z, base);
    sv_imu_preint_range(&p, &b, base + 200000000LL, base + 1200000000LL, z, z, &nz);
    n = 0;
    for (i = 0; i < (long)b.n && n < 1024; ++i) {
        int64_t t = b.s[i].t_ns; if (t < base + 200000000LL || t > base + 1200000000LL) continue;
        m[n].t.sec = (uint32_t)(t / 1000000000LL); m[n].t.nsec = (uint32_t)(t % 1000000000LL);
        for (k = 0; k < 3; ++k) { m[n].gyr[k] = b.s[i].gyr[k]; m[n].acc[k] = b.s[i].acc[k]; }
        n++;
    }
    { ok_time t0 = {(uint32_t)((base + 200000000LL) / 1000000000LL), (uint32_t)((base + 200000000LL) % 1000000000LL)};
      ok_time t1 = {(uint32_t)((base + 1200000000LL) / 1000000000LL), (uint32_t)((base + 1200000000LL) % 1000000000LL)};
      ok_imu_error_init(&e, m, (size_t)n, &prm, t0, t1);
      r = ok_imu_redo_preintegration(&e, sb); }
    CHECK(r > 0, "ok_imu_redo_preintegration returned %d", r);
    q = e.delta_q; ok_quat_to_mat3(&q, dRok);   /* column-major */
    for (k = 0; k < 3; ++k) for (c = 0; c < 3; ++c) Rtmp[3 * k + c] = dRok[k + 3 * c];
    maxdR = rot_err(p.dR, Rtmp); maxdv = vdist(p.dv, e.acc_integral); maxdp = vdist(p.dp, e.acc_doubleintegral);
    /* Jacobians (OKVIS stores column-major, its dalpha_db_g is the negative of ours (opposite perturbation convention)) */
    for (k = 0; k < 3; ++k) for (c = 0; c < 3; ++c) {
        double a1 = p.J_Rbg[3 * k + c], a2 = a1, b1 = p.J_vbg[3 * k + c], b2 = e.dv_db_g[k + 3 * c], c1 = p.J_pbg[3 * k + c], c2 = e.dp_db_g[k + 3 * c];
        maxJ[0] = fmax(maxJ[0], fabs(a1 - a2)); maxJ[1] = fmax(maxJ[1], fabs(b1 - b2)); maxJ[2] = fmax(maxJ[2], fabs(c1 - c2));
        scale[0] = fmax(scale[0], fabs(a1)); scale[1] = fmax(scale[1], fabs(b1)); scale[2] = fmax(scale[2], fabs(c1));
    }
    printf("  dR angle diff %.2e rad, dv diff %.2e m/s, dp diff %.2e m (|dv| %.2f |dp| %.2f)\n", maxdR, maxdv, maxdp, sqrt(p.dv[0]*p.dv[0]+p.dv[1]*p.dv[1]+p.dv[2]*p.dv[2]), sqrt(p.dp[0]*p.dp[0]+p.dp[1]*p.dp[1]+p.dp[2]*p.dp[2]));
    { double mn = 0; for (k = 0; k < 3; ++k) for (c = 0; c < 3; ++c) { double a1 = 0, a2 = e.dalpha_db_g[k + 3 * c]; int m2; for (m2 = 0; m2 < 3; ++m2) a1 += p.dR[3 * k + m2] * p.J_Rbg[3 * m2 + c]; mn = fmax(mn, fabs(a1 + a2)); }
      printf("  J_Rbg: OKVIS dalpha_db_g vs -(dR J_Rbg) (left-perturbation form) max diff %.2e\n", mn); maxJ[0] = mn; }
    printf("  Jacobian max abs diff (ours - okvis): J_Rbg %.2e (max %.2f), J_vbg %.2e (max %.2f), J_pbg %.2e (max %.2f)\n", maxJ[0], scale[0], maxJ[1], scale[1], maxJ[2], scale[2]);
    CHECK(maxdR < 1e-5 && maxdv < 1e-5 && maxdp < 1e-5, "delta mismatch vs OKVIS");
    CHECK(maxJ[0] < 1e-3 * (scale[0] + 1) && maxJ[1] < 1e-3 * (scale[1] + 1) && maxJ[2] < 1e-3 * (scale[2] + 1), "Jacobian mismatch vs OKVIS");
    ok_imu_error_free(&e); sv_imu_buf_free(&b);
}

/* ---- 6. gyro prediction + buffer behaviour ---- */
static void test_gyro_buf(void) {
    static const double z[3] = {0, 0, 0};
    const double bgt[3] = {0.02, -0.01, 0.03};
    const double R_BC[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    sv_imu_buf b; double R[9], R1[9], p[3], v[3], a[3], pred[9], T1[9], T2[9], truth[9], dB[9], worst = 0, worst_nb = 0;
    int64_t base = 7000000000LL; int i, n;
    sv_imu_sample s; int r;
    printf("[6] gyro rotation prediction (sinusoidal rotation, 200 Hz, frames every 50 ms, gyro bias %.2f,%.2f,%.2f)\n", bgt[0], bgt[1], bgt[2]);
    fill(&b, 1, 200, 10.0, bgt, z, base);
    for (i = 0, n = 0; i < 180; ++i, ++n) {
        double t0 = 0.5 + 0.05 * i, t1 = t0 + 0.05;
        pose_at(1, t0, R, p, v, a); pose_at(1, t1, R1, p, v, a);
        sv_m3_tmul(R, R1, dB); sv_m3_tmul(R_BC, dB, T1); sv_m3_mul(T1, R_BC, truth);
        sv_imu_gyro_predict_cam(&b, base + (int64_t)llround(t0 * 1e9), base + (int64_t)llround(t1 * 1e9), bgt, R_BC, pred);
        worst = fmax(worst, rot_err(pred, truth));
        sv_imu_gyro_predict_cam(&b, base + (int64_t)llround(t0 * 1e9), base + (int64_t)llround(t1 * 1e9), z, R_BC, pred);
        worst_nb = fmax(worst_nb, rot_err(pred, truth));
    }
    printf("  worst per-frame error with bias removed %.2e rad, with bias ignored %.2e rad (expected ~|bg| dt = %.2e)\n", worst, worst_nb, 0.05 * sqrt(0.02*0.02+0.01*0.01+0.03*0.03));
    CHECK(worst < 5e-5, "gyro prediction error %g", worst);
    CHECK(worst_nb > 10 * worst, "bias must matter");
    (void)T2;
    /* buffer rules */
    sv_imu_buf_free(&b); sv_imu_buf_init(&b, 100000000LL);
    for (i = 0; i < 20; ++i) { s = sample_at(1, i * 0.01, base + i * 10000000LL, z, z); sv_imu_buf_push(&b, &s); }
    s = sample_at(1, 0.19, base + 190000000LL, z, z);
    CHECK(sv_imu_buf_push(&b, &s) == 1, "duplicate not flagged");
    s = sample_at(1, 0.05, base + 50000000LL, z, z);
    CHECK(sv_imu_buf_push(&b, &s) == 2, "backwards not flagged");
    s = sample_at(1, 0.80, base + 800000000LL, z, z);   /* 0.61 s outage */
    sv_imu_buf_push(&b, &s);
    for (i = 1; i < 10; ++i) { s = sample_at(1, 0.8 + i * 0.01, base + 800000000LL + i * 10000000LL, z, z); sv_imu_buf_push(&b, &s); }
    r = sv_imu_gyro_rotation(&b, base + 20000000LL, base + 150000000LL, z, R);
    CHECK(r > 0, "in-range integration failed (%d)", r);
    r = sv_imu_gyro_rotation(&b, base + 100000000LL, base + 850000000LL, z, R);
    CHECK(r == -2, "outage must be refused, got %d", r);
    r = sv_imu_gyro_rotation(&b, base + 100000000LL, base + 5000000000LL, z, R);
    CHECK(r == -1, "unbracketed end must be refused, got %d", r);
    CHECK(b.n_dup == 1 && b.n_back == 1 && b.n_gap == 1, "buffer counters dup %ld back %ld gap %ld", b.n_dup, b.n_back, b.n_gap);
    printf("  buffer: duplicates=%ld backwards=%ld gaps=%ld; outage / unbracketed integration refused\n", b.n_dup, b.n_back, b.n_gap);
    sv_imu_buf_free(&b);
}

int main(void) {
    test_analytic(); test_bias(); test_cov(); test_okvis(); test_gyro_buf();
    printf(g_fail ? "\nFAILED (%d checks)\n" : "\nALL PASS\n", g_fail);
    return g_fail != 0;
}
