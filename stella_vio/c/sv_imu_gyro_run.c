/* SPDX-License-Identifier: MIT */
/* Driver: gyro-predicted vs visual (or GT) camera rotation between consecutive trajectory poses.
 * usage: sv_imu_gyro_run imu.csv traj.tum ext.txt [--bg x,y,z | --fit-bg] [--max-gap s] [--toff s]
 *   --toff: IMU clock = camera clock + toff (seconds)
 *   traj.tum: t x y z qx qy qz qw (camera pose; only the rotation is used), ext.txt: 12 numbers (R_BC row-major, p_BC) as for check_sv_imu_init.
 *   --fit-bg: gyro bias from the first continuous <= 60 s of the trajectory (sv_vi_gyro_bias, keyframes 0.5 s apart).
 * Output (stdout): "t0 t1 w_gyro_norm[rad/s] ang_vis[deg] err_gyro[deg] err_cv[deg]" per consecutive pair; err_gyro = angle(R_vis^T R_pred),
 * err_cv = same for a constant-angular-velocity predictor (previous pair's visual rotation rescaled to dt). The identity predictor error is ang_vis. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "sv_imu_init.h"

typedef struct { double t, R[9]; } tp_t;
static void q2R(double x, double y, double z, double w, double R[9]) {
    double n = sqrt(x*x+y*y+z*z+w*w); x/=n; y/=n; z/=n; w/=n;
    R[0]=1-2*(y*y+z*z); R[1]=2*(x*y-z*w);   R[2]=2*(x*z+y*w);
    R[3]=2*(x*y+z*w);   R[4]=1-2*(x*x+z*z); R[5]=2*(y*z-x*w);
    R[6]=2*(x*z-y*w);   R[7]=2*(y*z+x*w);   R[8]=1-2*(x*x+y*y);
}
static double ang(const double A[9], const double B[9]) { double D[9], w[3]; sv_m3_tmul(A, B, D); sv_so3_log(D, w); return sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) * 57.29577951308232; }

int main(int argc, char** argv) {
    sv_imu_buf b; tp_t* tr = NULL; long ntr = 0, cap = 0, i; char line[1024]; FILE* f;
    sv_vi_ext ext; double bg[3] = {0, 0, 0}, maxgap = 0.3, toff = 0.0; int fit = 0, a;
    double prevR[9], prevdt = 0; int have_prev = 0; double prevt = -1;
    if (argc < 4) { fprintf(stderr, "usage: %s imu.csv traj.tum ext.txt [--bg x,y,z | --fit-bg] [--max-gap s]\n", argv[0]); return 2; }
    for (a = 4; a < argc; ++a) {
        if (!strcmp(argv[a], "--fit-bg")) fit = 1;
        else if (!strcmp(argv[a], "--bg") && a + 1 < argc) sscanf(argv[++a], "%lf,%lf,%lf", &bg[0], &bg[1], &bg[2]);
        else if (!strcmp(argv[a], "--max-gap") && a + 1 < argc) maxgap = atof(argv[++a]);
        else if (!strcmp(argv[a], "--toff") && a + 1 < argc) toff = atof(argv[++a]);
    }
    f = fopen(argv[3], "r"); if (!f) { perror(argv[3]); return 2; }
    { double e[12]; for (a = 0; a < 12; ++a) if (fscanf(f, "%lf", &e[a]) != 1) return 2; memcpy(ext.R_BC, e, 9 * sizeof(double)); memcpy(ext.p_BC, e + 9, 3 * sizeof(double)); }
    fclose(f);
    sv_imu_buf_init(&b, 250000000LL);
    f = fopen(argv[1], "r"); if (!f) { perror(argv[1]); return 2; }
    while (fgets(line, sizeof line, f)) {
        sv_imu_sample s; long long tn;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &tn, &s.gyr[0], &s.gyr[1], &s.gyr[2], &s.acc[0], &s.acc[1], &s.acc[2]) == 7) { s.t_ns = tn; sv_imu_buf_push(&b, &s); }
    }
    fclose(f);
    f = fopen(argv[2], "r"); if (!f) { perror(argv[2]); return 2; }
    while (fgets(line, sizeof line, f)) {
        double t, x, y, z, qx, qy, qz, qw;
        if (line[0] == '#') continue;
        if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf", &t, &x, &y, &z, &qx, &qy, &qz, &qw) == 8) {
            if (ntr == cap) { cap = cap ? 2 * cap : 4096; tr = (tp_t*)realloc(tr, (size_t)cap * sizeof *tr); }
            if (t > 1e12) t *= 1e-9;
            tr[ntr].t = t; q2R(qx, qy, qz, qw, tr[ntr].R); ntr++;
        }
    }
    fclose(f);
    if (fit) {
        sv_vi_kf* kf = (sv_vi_kf*)malloc(200 * sizeof *kf); int nk = 0; double last = -1e30, t00 = tr[0].t; sv_imu_noise nz = {1e-3, 1e-2, 1e-4, 1e-3, 9.81}; double rb, ra;
        for (i = 0; i < ntr && nk < 120; ++i) {
            if (tr[i].t - t00 > 60) break;
            if (nk && tr[i].t - last > 1.0) break;   /* first continuous stretch only */
            if (tr[i].t - last >= 0.5 - 1e-6) {
                double iR[9];
                kf[nk].t_ns = (int64_t)llround((tr[i].t + toff) * 1e9); memcpy(kf[nk].R_VC, tr[i].R, sizeof iR); kf[nk].c[0] = kf[nk].c[1] = kf[nk].c[2] = 0; nk++; last = tr[i].t;
            }
        }
        if (nk >= 4 && sv_vi_gyro_bias(&b, kf, nk, &ext, &nz, bg, &rb, &ra) == 0) fprintf(stderr, "fit bg = %.5f %.5f %.5f rad/s from %d keyframes (rot rms %.4f -> %.4f rad)\n", bg[0], bg[1], bg[2], nk, rb, ra);
        else fprintf(stderr, "bg fit failed (%d keyframes); using zero\n", nk);
        free(kf);
    }
    for (i = 1; i < ntr; ++i) {
        double dt = tr[i].t - tr[i - 1].t, Rv[9], Rp[9], Rcv[9], w[3], wn, t0 = tr[i - 1].t, t1 = tr[i].t, e_cv;
        sv_imu_sample s0;
        if (dt <= 0 || dt > maxgap) { have_prev = 0; continue; }
        sv_m3_tmul(tr[i - 1].R, tr[i].R, Rv);
        if (sv_imu_gyro_predict_cam(&b, (int64_t)llround((t0 + toff) * 1e9), (int64_t)llround((t1 + toff) * 1e9), bg, ext.R_BC, Rp) <= 0) { have_prev = 0; continue; }
        if (sv_imu_buf_interp(&b, (int64_t)llround((0.5 * (t0 + t1) + toff) * 1e9), &s0)) continue;
        wn = sqrt((s0.gyr[0]-bg[0])*(s0.gyr[0]-bg[0])+(s0.gyr[1]-bg[1])*(s0.gyr[1]-bg[1])+(s0.gyr[2]-bg[2])*(s0.gyr[2]-bg[2]));
        e_cv = NAN;
        if (have_prev && fabs(t0 - prevt) < 1e-9) {
            double lw[3]; int k; sv_so3_log(prevR, lw); for (k = 0; k < 3; ++k) lw[k] *= dt / prevdt;
            sv_so3_exp(lw, Rcv); e_cv = ang(Rv, Rcv);
        }
        sv_so3_log(Rv, w);
        printf("%.4f %.4f %.4f %.5f %.5f %.5f\n", t0, t1, wn, sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]) * 57.29577951308232, ang(Rv, Rp), e_cv);
        memcpy(prevR, Rv, sizeof Rv); prevdt = dt; prevt = t1; have_prev = 1;
    }
    free(tr); sv_imu_buf_free(&b);
    return 0;
}
