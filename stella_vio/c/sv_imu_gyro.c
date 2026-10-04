/* SPDX-License-Identifier: MIT */
/* Gyro-aided rotation prediction (see sv_imu.h). Own implementation: trapezoid gyro samples, exact SO(3) exponential. */
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "sv_imu.h"

int sv_imu_gyro_rotation(const sv_imu_buf* b, int64_t t0, int64_t t1, const double bg[3], double dR_B[9]) {
    sv_imu_sample cur, e1;
    long i, last;
    int cnt = 0, k;
    memset(dR_B, 0, 9 * sizeof(double)); dR_B[0] = dR_B[4] = dR_B[8] = 1.0;
    if (t1 <= t0) return 0;
    if (sv_imu_buf_interp(b, t0, &cur) || sv_imu_buf_interp(b, t1, &e1)) return -1;
    last = sv_imu_buf_find(b, t1);
    for (i = sv_imu_buf_find(b, t0) + 1; i <= last + 1; ++i) {
        sv_imu_sample nxt; double w[3], E[9], T[9], dt;
        if (i <= last && b->s[i].t_ns < t1) nxt = b->s[i];
        else { nxt = e1; i = last + 2; }
        if (nxt.t_ns <= cur.t_ns) continue;
        if (b->max_gap_ns > 0) {
            long j0 = sv_imu_buf_find(b, cur.t_ns);
            if (j0 + 1 < (long)b->n && b->s[j0 + 1].t_ns - b->s[j0].t_ns > b->max_gap_ns) return -2;
        }
        dt = (double)(nxt.t_ns - cur.t_ns) * 1e-9;
        for (k = 0; k < 3; ++k) w[k] = (0.5 * (cur.gyr[k] + nxt.gyr[k]) - bg[k]) * dt;
        sv_so3_exp(w, E); sv_m3_mul(dR_B, E, T); memcpy(dR_B, T, sizeof T);
        cnt++; cur = nxt;
    }
    return cnt;
}

int sv_imu_gyro_predict_cam(const sv_imu_buf* b, int64_t t0, int64_t t1, const double bg[3], const double R_BC[9], double R_C0C1[9]) {
    double dR[9], T[9];
    int n = sv_imu_gyro_rotation(b, t0, t1, bg, dR);
    if (n < 0) return n;
    sv_m3_tmul(R_BC, dR, T);          /* R_CB dR */
    sv_m3_mul(T, R_BC, R_C0C1);       /* R_CB dR R_BC */
    return n;
}

int sv_imu_gyro_mean(const sv_imu_buf* b, int64_t t0, int64_t t1, double mean[3]) {
    long i, a = sv_imu_buf_find(b, t0), c = sv_imu_buf_find(b, t1);
    int n = 0, k;
    mean[0] = mean[1] = mean[2] = 0;
    for (i = (a < 0 ? 0 : (b->s[a].t_ns < t0 ? a + 1 : a)); i <= c; ++i) {
        for (k = 0; k < 3; ++k) mean[k] += b->s[i].gyr[k];
        n++;
    }
    if (n) for (k = 0; k < 3; ++k) mean[k] /= n;
    return n;
}

int sv_imu_acc_mean(const sv_imu_buf* b, int64_t t0, int64_t t1, double mean[3]) {
    long i, a = sv_imu_buf_find(b, t0), c = sv_imu_buf_find(b, t1);
    int n = 0, k;
    mean[0] = mean[1] = mean[2] = 0;
    for (i = (a < 0 ? 0 : a + 1); i <= c; ++i) {
        for (k = 0; k < 3; ++k) mean[k] += b->s[i].acc[k];
        n++;
    }
    if (n) for (k = 0; k < 3; ++k) mean[k] /= n;
    return n;
}
