/* SPDX-License-Identifier: MIT */
/* Command-line driver for sv_vi_init on real data (EuRoC-style IMU csv + a mono visual trajectory), many windows per call.
 * usage: check_sv_imu_init imu.csv traj.tum ext.txt kf_dt sigma_g sigma_a --starts s1,s2,.. --lens l1,l2,.. [--no-ba] [--vout file]
 *   imu.csv  : "#..." header, then ns, wx, wy, wz, ax, ay, az (rad/s, m/s^2 specific force)
 *   traj.tum : t x y z qx qy qz qw  = camera pose T_VC in the (arbitrary-scale) visual world; t in s (or ns)
 *   ext.txt  : 12 numbers: R_BC row-major (camera->body) then p_BC (camera origin in body frame, metres)
 *   starts are absolute times in the trajectory's clock [s]. Per window the keyframes are the trajectory poses at least kf_dt apart;
 *   a window is skipped ("gap") if consecutive keyframes are > max(3 kf_dt, 1 s) apart or the trajectory/IMU do not cover it.
 * Output: one line per (start, len): key=value pairs (consumed by stella_vio/tools/imu_init_eval.py). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "sv_imu_init.h"

typedef struct { double t, p[3], R[9]; } tp_t;
static void q2R(double x, double y, double z, double w, double R[9]) {
    double n = sqrt(x*x+y*y+z*z+w*w); x/=n; y/=n; z/=n; w/=n;
    R[0]=1-2*(y*y+z*z); R[1]=2*(x*y-z*w);   R[2]=2*(x*z+y*w);
    R[3]=2*(x*y+z*w);   R[4]=1-2*(x*x+z*z); R[5]=2*(y*z-x*w);
    R[6]=2*(x*z-y*w);   R[7]=2*(y*z+x*w);   R[8]=1-2*(x*x+y*y);
}
static int parse_list(const char* s, double* out, int max) {
    int n = 0; char* e;
    while (*s && n < max) { out[n++] = strtod(s, &e); if (e == s) break; s = (*e == ',') ? e + 1 : e; if (!*e) break; }
    return n;
}

int main(int argc, char** argv) {
    sv_imu_buf b; tp_t* tr = NULL; long ntr = 0, cap = 0; char line[1024]; FILE* f;
    double starts[512], lens[64]; int ns = 0, nl = 0, a, si, li, est_ba = 1;
    double fl_p = -1, fl_v = -1, fl_r = -1, sba = -1, smax = -1;
    sv_vi_ext ext; sv_imu_noise nz; double kf_dt; const char* vout = NULL; FILE* fv = NULL;
    if (argc < 9) { fprintf(stderr, "usage: %s imu.csv traj.tum ext.txt kf_dt sigma_g sigma_a --starts a,b --lens x,y [--no-ba] [--floors pos,vel,rot[,sigma_ba[,max_sigma_logS]]] [--vout f]\n", argv[0]); return 2; }
    kf_dt = atof(argv[4]); nz.sigma_g = atof(argv[5]); nz.sigma_a = atof(argv[6]); nz.sigma_bg = 1e-4; nz.sigma_ba = 1e-3; nz.gravity = 9.81;
    for (a = 7; a < argc; ++a) {
        if (!strcmp(argv[a], "--starts") && a + 1 < argc) ns = parse_list(argv[++a], starts, 512);
        else if (!strcmp(argv[a], "--lens") && a + 1 < argc) nl = parse_list(argv[++a], lens, 64);
        else if (!strcmp(argv[a], "--no-ba")) est_ba = 0;
        else if (!strcmp(argv[a], "--floors") && a + 1 < argc) { double q[5] = {0}; int nq = parse_list(argv[++a], q, 5); if (nq >= 3) { fl_p = q[0]; fl_v = q[1]; fl_r = q[2]; } if (nq >= 4) sba = q[3]; if (nq >= 5) smax = q[4]; }
        else if (!strcmp(argv[a], "--vout") && a + 1 < argc) vout = argv[++a];
    }
    f = fopen(argv[3], "r"); if (!f) { perror(argv[3]); return 2; }
    { double e[12]; for (a = 0; a < 12; ++a) if (fscanf(f, "%lf", &e[a]) != 1) { fprintf(stderr, "bad ext\n"); return 2; }
      memcpy(ext.R_BC, e, 9 * sizeof(double)); memcpy(ext.p_BC, e + 9, 3 * sizeof(double)); }
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
            tr[ntr].t = t; tr[ntr].p[0] = x; tr[ntr].p[1] = y; tr[ntr].p[2] = z; q2R(qx, qy, qz, qw, tr[ntr].R); ntr++;
        }
    }
    fclose(f);
    if (vout) fv = fopen(vout, "w");
    fprintf(stderr, "imu %zu samples (dup %ld back %ld gap %ld), traj %ld poses\n", b.n, b.n_dup, b.n_back, b.n_gap, ntr);
    for (si = 0; si < ns; ++si) for (li = 0; li < nl; ++li) {
        double t0 = starts[si], len = lens[li]; sv_vi_kf* kf = (sv_vi_kf*)malloc((size_t)(len / kf_dt + 3) * sizeof *kf);
        int nk = 0, bad = 0; long j; double last = -1e30;
        for (j = 0; j < ntr && !bad; ++j) {
            if (tr[j].t < t0) continue;
            if (tr[j].t > t0 + len + 1e-6) break;
            if (tr[j].t - last >= kf_dt - 1e-6) {
                if (nk > 0 && tr[j].t - last > fmax(3 * kf_dt, 1.0)) { bad = 1; break; }
                kf[nk].t_ns = (int64_t)llround(tr[j].t * 1e9); memcpy(kf[nk].R_VC, tr[j].R, sizeof tr[j].R); memcpy(kf[nk].c, tr[j].p, sizeof tr[j].p);
                nk++; last = tr[j].t;
            }
        }
        if (!bad && (nk < 3 || last - (kf[0].t_ns * 1e-9) < 0.8 * len)) bad = 1;
        if (bad) { printf("start=%.3f len=%.2f status=gap\n", t0, len); free(kf); continue; }
        {
            sv_vi_cfg cfg; sv_vi_result r; int rc;
            sv_vi_cfg_default(&cfg); cfg.estimate_ba = est_ba;
            if (fl_p > 0) { cfg.sigma_pos_floor = fl_p; cfg.sigma_vel_floor = fl_v; cfg.sigma_rot_floor = fl_r; }
            if (sba > 0) cfg.sigma_ba_prior = sba;
            if (smax > 0) cfg.max_sigma_log_scale = smax;
            rc = sv_vi_init(&b, kf, nk, &ext, &nz, &cfg, &r);
            if (rc != 0 && rc != -4) { printf("start=%.3f len=%.2f status=fail rc=%d n_kf=%d\n", t0, len, rc, nk); free(kf); continue; }
            printf("start=%.3f len=%.2f status=%s rc=%d n_kf=%d span=%.2f scale=%.6f scale_lin=%.6f sigma_logS=%.4f gx=%.5f gy=%.5f gz=%.5f sigma_g_deg=%.3f "
                   "bgx=%.5f bgy=%.5f bgz=%.5f bax=%.4f bay=%.4f baz=%.4f rot_before=%.5f rot_after=%.5f rms_pos=%.4f rms_vel=%.4f chi2dof=%.3f ok=%d\n",
                   t0, len, r.ok ? "ok" : "rejected", rc, nk, r.span_s, r.scale, r.scale_linear, r.sigma_log_scale, r.g_V[0], r.g_V[1], r.g_V[2], r.sigma_grav_deg,
                   r.bg[0], r.bg[1], r.bg[2], r.ba[0], r.ba[1], r.ba[2], r.rms_rot_before, r.rms_rot_after, r.rms_pos, r.rms_vel, r.chi2_dof, r.ok);
            if (fv && r.v) { int q; for (q = 0; q < nk; ++q) fprintf(fv, "%.3f %.2f %.6f %.6f %.6f %.6f\n", t0, len, kf[q].t_ns * 1e-9, r.v[3*q], r.v[3*q+1], r.v[3*q+2]); }
            sv_vi_result_free(&r);
        }
        free(kf);
    }
    if (fv) fclose(fv);
    free(tr); sv_imu_buf_free(&b);
    return 0;
}
