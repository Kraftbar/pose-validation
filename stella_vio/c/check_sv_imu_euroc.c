/* SPDX-License-Identifier: MIT */
/* Real-data check of sv_imu preintegration: EuRoC-format IMU csv (ns, wx wy wz ax ay az) against ground-truth body poses (TUM: t x y z qx qy qz qw,
 * body = IMU frame, world gravity (0,0,-9.81)). For interval lengths L the relative rotation / velocity / position delta predicted by the IMU is
 * compared with the same quantities from GT (velocity = central difference of GT position over +-25 ms).
 * usage: check_sv_imu_euroc imu.csv gt_body.tum [static_seconds=1.5]
 * Variants: (a) no bias correction, (b) gyro bias from the initial static seconds, (c) oracle constant bg/ba fitted by linear least squares to the
 * 1 s intervals (checks the Jacobians on real data), re-integrated at the fit. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "sv_imu.h"

typedef struct { double t, p[3], R[9]; } gtp;
static gtp* gt; static long ngt;
static void q2R(double x, double y, double z, double w, double R[9]) {
    double n = sqrt(x*x+y*y+z*z+w*w); x/=n; y/=n; z/=n; w/=n;
    R[0]=1-2*(y*y+z*z); R[1]=2*(x*y-z*w);   R[2]=2*(x*z+y*w);
    R[3]=2*(x*y+z*w);   R[4]=1-2*(x*x+z*z); R[5]=2*(y*z-x*w);
    R[6]=2*(x*z-y*w);   R[7]=2*(y*z+x*w);   R[8]=1-2*(x*x+y*y);
}
static int gt_at(double t, double R[9], double p[3]) {
    long lo = 0, hi = ngt - 1; int k;
    if (t < gt[0].t || t > gt[ngt - 1].t) return -1;
    while (hi - lo > 1) { long m = (lo + hi) / 2; if (gt[m].t <= t) lo = m; else hi = m; }
    { double f = (t - gt[lo].t) / (gt[hi].t - gt[lo].t), w[3], D[9], E[9], A[9];
      for (k = 0; k < 3; ++k) p[k] = gt[lo].p[k] + f * (gt[hi].p[k] - gt[lo].p[k]);
      sv_m3_tmul(gt[lo].R, gt[hi].R, D); sv_so3_log(D, w); for (k = 0; k < 3; ++k) w[k] *= f;
      sv_so3_exp(w, E); sv_m3_mul(gt[lo].R, E, A); memcpy(R, A, sizeof A); }
    return 0;
}
static int cmpd(const void* a, const void* b) { double x = *(const double*)a, y = *(const double*)b; return (x > y) - (x < y); }
static void stats(double* v, int n, double* med, double* p95) {
    if (!n) { *med = *p95 = NAN; return; }
    qsort(v, (size_t)n, sizeof *v, cmpd); *med = v[n / 2]; *p95 = v[(int)(0.95 * (n - 1))];
}
static const double G_W[3] = {0, 0, -9.81};

typedef struct { double rot, dv, dp; } err3;
/* evaluate one interval: returns 0 and errors vs GT; also exports GT deltas */
static int gt_delta(double ta, double tb, double dR[9], double dv[3], double dp[3]) {
    double Ri[9], Rj[9], pi_[3], pj[3], pa[3], pb[3], vi[3], vj[3], dummy[9], u[3]; int k; double dt = tb - ta, h = 0.025;
    if (gt_at(ta, Ri, pi_) || gt_at(tb, Rj, pj)) return -1;
    if (gt_at(ta - h, dummy, pa) || gt_at(ta + h, dummy, pb)) return -1;
    for (k = 0; k < 3; ++k) vi[k] = (pb[k] - pa[k]) / (2 * h);
    if (gt_at(tb - h, dummy, pa) || gt_at(tb + h, dummy, pb)) return -1;
    for (k = 0; k < 3; ++k) vj[k] = (pb[k] - pa[k]) / (2 * h);
    sv_m3_tmul(Ri, Rj, dR);
    for (k = 0; k < 3; ++k) u[k] = vj[k] - vi[k] - G_W[k] * dt;
    sv_m3_tmulv(Ri, u, dv);
    for (k = 0; k < 3; ++k) u[k] = pj[k] - pi_[k] - vi[k] * dt - 0.5 * G_W[k] * dt * dt;
    sv_m3_tmulv(Ri, u, dp);
    return 0;
}

int main(int argc, char** argv) {
    FILE* f; char line[512]; sv_imu_buf b; double t0imu = 0, static_s = argc > 3 ? atof(argv[3]) : 1.5;
    double Ls[] = {0.1, 0.25, 0.5, 1.0, 2.0}; int li, k, v;
    double bg_static[3], z[3] = {0, 0, 0}, bgfit[3] = {0, 0, 0}, bafit[3] = {0, 0, 0};
    long cap = 0; sv_imu_noise nz = {1.7e-4, 2e-3, 1.9e-5, 3e-3, 9.81};
    if (argc < 3) { fprintf(stderr, "usage: %s imu.csv gt_body.tum [static_s]\n", argv[0]); return 2; }
    sv_imu_buf_init(&b, 100000000LL);
    f = fopen(argv[1], "r"); if (!f) { perror(argv[1]); return 2; }
    while (fgets(line, sizeof line, f)) {
        sv_imu_sample s; long long tn;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &tn, &s.gyr[0], &s.gyr[1], &s.gyr[2], &s.acc[0], &s.acc[1], &s.acc[2]) == 7) { s.t_ns = tn; sv_imu_buf_push(&b, &s); }
    }
    fclose(f);
    f = fopen(argv[2], "r"); if (!f) { perror(argv[2]); return 2; }
    while (fgets(line, sizeof line, f)) {
        double t, x, y, z_, qx, qy, qz, qw;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf", &t, &x, &y, &z_, &qx, &qy, &qz, &qw) == 8) {
            if (ngt == cap) { cap = cap ? 2 * cap : 4096; gt = (gtp*)realloc(gt, (size_t)cap * sizeof *gt); }
            gt[ngt].t = t; gt[ngt].p[0] = x; gt[ngt].p[1] = y; gt[ngt].p[2] = z_; q2R(qx, qy, qz, qw, gt[ngt].R); ngt++;
        }
    }
    fclose(f);
    printf("imu samples %zu (dup %ld back %ld gaps>0.1s %ld), gt poses %ld\n", b.n, b.n_dup, b.n_back, b.n_gap, ngt);
    t0imu = (double)b.s[0].t_ns * 1e-9;
    { double ts = gt[0].t > t0imu ? gt[0].t + 0.2 : t0imu;   /* static window starts once GT exists (the IMU stream starts earlier) */
      sv_imu_gyro_mean(&b, (int64_t)llround(ts * 1e9), (int64_t)llround((ts + static_s) * 1e9), bg_static); }
    printf("static-start gyro mean over %.1f s: %.5f %.5f %.5f rad/s\n", static_s, bg_static[0], bg_static[1], bg_static[2]);

    /* oracle LS fit of constant bg/ba on non-overlapping 1 s intervals */
    {
        double H[36] = {0}, g[6] = {0}; int nint = 0;
        double ta;
        for (ta = t0imu + 1.0; ta + 1.0 < (double)b.s[b.n - 1].t_ns * 1e-9 - 1.0; ta += 1.0) {
            sv_imu_preint p; double dRg[9], dvg[3], dpg[3];
            int64_t a_ns = (int64_t)llround(ta * 1e9), b_ns = (int64_t)llround((ta + 1.0) * 1e9);
            if (sv_imu_preint_range(&p, &b, a_ns, b_ns, z, z, &nz) <= 0) continue;
            if (gt_delta(ta, ta + 1.0, dRg, dvg, dpg)) continue;
            {
                double A[9], B[9], r[3], Ji[9], J[9], nr[3], rows[9][7]; int i, j;
                sv_m3_tmul(p.dR, dRg, A);          /* dR^T dR_gt */
                sv_so3_log(A, r);
                nr[0] = -r[0]; nr[1] = -r[1]; nr[2] = -r[2]; sv_so3_jr_inv(nr, Ji); sv_m3_mul(Ji, p.J_Rbg, J);
                (void)B;
                /* rotation: r(bg) = r - J bg  ->  J bg = r */
                for (i = 0; i < 3; ++i) { for (j = 0; j < 3; ++j) { rows[i][j] = J[3 * i + j]; rows[i][3 + j] = 0; } rows[i][6] = r[i] / 0.01; for (j = 0; j < 6; ++j) rows[i][j] /= 0.01; }
                /* velocity: dv_gt = dv + Jvbg bg + Jvba ba */
                for (i = 0; i < 3; ++i) { for (j = 0; j < 3; ++j) { rows[3 + i][j] = p.J_vbg[3 * i + j] / 0.05; rows[3 + i][3 + j] = p.J_vba[3 * i + j] / 0.05; } rows[3 + i][6] = (dvg[i] - p.dv[i]) / 0.05; }
                for (i = 0; i < 3; ++i) { for (j = 0; j < 3; ++j) { rows[6 + i][j] = p.J_pbg[3 * i + j] / 0.02; rows[6 + i][3 + j] = p.J_pba[3 * i + j] / 0.02; } rows[6 + i][6] = (dpg[i] - p.dp[i]) / 0.02; }
                for (i = 0; i < 9; ++i) for (j = 0; j < 6; ++j) { g[j] += rows[i][j] * rows[i][6]; for (k = 0; k < 6; ++k) H[6 * j + k] += rows[i][j] * rows[i][k]; }
            }
            nint++;
        }
        /* Gaussian elimination 6x6 */
        {
            double M[6][7]; int i, j, c;
            for (i = 0; i < 6; ++i) { for (j = 0; j < 6; ++j) M[i][j] = H[6 * i + j]; M[i][6] = g[i]; }
            for (c = 0; c < 6; ++c) {
                int piv = c; for (i = c + 1; i < 6; ++i) if (fabs(M[i][c]) > fabs(M[piv][c])) piv = i;
                for (j = 0; j < 7; ++j) { double t = M[c][j]; M[c][j] = M[piv][j]; M[piv][j] = t; }
                for (i = 0; i < 6; ++i) if (i != c) { double fct = M[i][c] / M[c][c]; for (j = c; j < 7; ++j) M[i][j] -= fct * M[c][j]; }
            }
            for (i = 0; i < 3; ++i) { bgfit[i] = M[i][6] / M[i][i]; bafit[i] = M[3 + i][6] / M[3 + i][3 + i]; }
        }
        printf("oracle constant-bias fit over %d x 1 s intervals: bg %.5f %.5f %.5f rad/s, ba %.4f %.4f %.4f m/s^2\n", nint, bgfit[0], bgfit[1], bgfit[2], bafit[0], bafit[1], bafit[2]);
    }

    printf("\n%-5s %-28s %5s | %9s %9s | %9s %9s | %9s %9s\n", "L[s]", "variant", "n", "rot med", "rot p95", "dv med", "dv p95", "dp med", "dp p95");
    printf("%-5s %-28s %5s | %9s %9s | %9s %9s | %9s %9s\n", "", "", "", "[deg]", "[deg]", "[m/s]", "[m/s]", "[m]", "[m]");
    for (li = 0; li < 5; ++li) {
        double L = Ls[li];
        static const char* names[4] = {"(a) bias 0", "(b) static-start gyro bias", "(c) oracle fit, 1st-order", "(c') oracle fit, re-integrated"};
        for (v = 0; v < 4; ++v) {
            double *er = malloc(100000 * sizeof(double)), *ev = malloc(100000 * sizeof(double)), *ep = malloc(100000 * sizeof(double));
            int n = 0; double ta;
            for (ta = t0imu + 0.5; ta + L < (double)b.s[b.n - 1].t_ns * 1e-9 - 0.5 && n < 100000; ta += L) {
                sv_imu_preint p; double dR[9], dv[3], dp[3], dRg[9], dvg[3], dpg[3], A[9], w[3];
                const double *bg = z, *ba = z;
                int64_t a_ns = (int64_t)llround(ta * 1e9), b_ns = (int64_t)llround((ta + L) * 1e9);
                if (v == 1) bg = bg_static;
                if (v >= 2) { bg = bgfit; ba = bafit; }
                if (sv_imu_preint_range(&p, &b, a_ns, b_ns, v == 3 ? bg : z, v == 3 ? ba : z, &nz) <= 0) continue;
                if (gt_delta(ta, ta + L, dRg, dvg, dpg)) continue;
                sv_imu_preint_corrected(&p, bg, ba, dR, dv, dp);
                sv_m3_tmul(dR, dRg, A); sv_so3_log(A, w);
                er[n] = sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) * 57.29577951308232;
                ev[n] = sqrt((dv[0]-dvg[0])*(dv[0]-dvg[0])+(dv[1]-dvg[1])*(dv[1]-dvg[1])+(dv[2]-dvg[2])*(dv[2]-dvg[2]));
                ep[n] = sqrt((dp[0]-dpg[0])*(dp[0]-dpg[0])+(dp[1]-dpg[1])*(dp[1]-dpg[1])+(dp[2]-dpg[2])*(dp[2]-dpg[2]));
                n++;
            }
            { double a1, a2, b1, b2, c1, c2; stats(er, n, &a1, &a2); stats(ev, n, &b1, &b2); stats(ep, n, &c1, &c2);
              printf("%-5.2f %-28s %5d | %9.4f %9.4f | %9.4f %9.4f | %9.4f %9.4f\n", L, names[v], n, a1, a2, b1, b2, c1, c2); }
            free(er); free(ev); free(ep);
        }
    }
    sv_imu_buf_free(&b); free(gt);
    return 0;
}
