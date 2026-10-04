/* SPDX-License-Identifier: MIT */
/* See sv_imu.h. Forster et al. 2017 on-manifold preintegration, own implementation. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include "sv_imu.h"

/* ---------------- buffer ---------------- */
void sv_imu_buf_init(sv_imu_buf* b, int64_t max_gap_ns) {
    memset(b, 0, sizeof *b);
    b->max_gap_ns = max_gap_ns;
}
void sv_imu_buf_free(sv_imu_buf* b) { free(b->s); memset(b, 0, sizeof *b); }
int sv_imu_buf_push(sv_imu_buf* b, const sv_imu_sample* s) {
    if (b->n) {
        int64_t last = b->s[b->n - 1].t_ns;
        if (s->t_ns == last) { b->n_dup++; return 1; }
        if (s->t_ns < last) { b->n_back++; return 2; }
        if (b->max_gap_ns > 0 && s->t_ns - last > b->max_gap_ns) b->n_gap++;
    }
    if (b->n == b->cap) {
        size_t c = b->cap ? 2 * b->cap : 4096;
        sv_imu_sample* t = (sv_imu_sample*)realloc(b->s, c * sizeof *t);
        if (!t) return -1;
        b->s = t; b->cap = c;
    }
    b->s[b->n++] = *s;
    return 0;
}
long sv_imu_buf_find(const sv_imu_buf* b, int64_t t_ns) {
    long lo = 0, hi = (long)b->n - 1, r = -1;
    while (lo <= hi) {
        long m = lo + (hi - lo) / 2;
        if (b->s[m].t_ns <= t_ns) { r = m; lo = m + 1; } else hi = m - 1;
    }
    return r;
}
int sv_imu_buf_interp(const sv_imu_buf* b, int64_t t_ns, sv_imu_sample* out) {
    long i = sv_imu_buf_find(b, t_ns);
    int k;
    if (i < 0) return -1;
    if (b->s[i].t_ns == t_ns) { *out = b->s[i]; return 0; }
    if ((size_t)i + 1 >= b->n) return -1;
    {
        const sv_imu_sample *a = &b->s[i], *c = &b->s[i + 1];
        double f = (double)(t_ns - a->t_ns) / (double)(c->t_ns - a->t_ns);
        out->t_ns = t_ns;
        for (k = 0; k < 3; ++k) {
            out->gyr[k] = a->gyr[k] + f * (c->gyr[k] - a->gyr[k]);
            out->acc[k] = a->acc[k] + f * (c->acc[k] - a->acc[k]);
        }
    }
    return 0;
}

/* ---------------- 3x3 helpers ---------------- */
void sv_m3_mul(const double A[9], const double B[9], double C[9]) {
    int i, j, k;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {
        double s = 0; for (k = 0; k < 3; ++k) s += A[3 * i + k] * B[3 * k + j];
        C[3 * i + j] = s;
    }
}
void sv_m3_tmul(const double A[9], const double B[9], double C[9]) {
    int i, j, k;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {
        double s = 0; for (k = 0; k < 3; ++k) s += A[3 * k + i] * B[3 * k + j];
        C[3 * i + j] = s;
    }
}
void sv_m3_mulv(const double A[9], const double v[3], double o[3]) {
    int i; for (i = 0; i < 3; ++i) o[i] = A[3 * i] * v[0] + A[3 * i + 1] * v[1] + A[3 * i + 2] * v[2];
}
void sv_m3_tmulv(const double A[9], const double v[3], double o[3]) {
    int i; for (i = 0; i < 3; ++i) o[i] = A[i] * v[0] + A[3 + i] * v[1] + A[6 + i] * v[2];
}
void sv_m3_hat(const double v[3], double K[9]) {
    K[0] = 0;     K[1] = -v[2]; K[2] = v[1];
    K[3] = v[2];  K[4] = 0;     K[5] = -v[0];
    K[6] = -v[1]; K[7] = v[0];  K[8] = 0;
}

/* R = I + a K + b K^2 with K = [w]x */
static void so3_series(const double w[3], double a, double b, double R[9]) {
    double K[9], K2[9]; int i;
    sv_m3_hat(w, K); sv_m3_mul(K, K, K2);
    for (i = 0; i < 9; ++i) R[i] = ((i % 4 == 0) ? 1.0 : 0.0) + a * K[i] + b * K2[i];
}
void sv_so3_exp(const double w[3], double R[9]) {
    double th2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2], th = sqrt(th2), a, b;
    if (th < 1e-5) { a = 1.0 - th2 / 6.0; b = 0.5 - th2 / 24.0; }
    else { a = sin(th) / th; b = (1.0 - cos(th)) / th2; }
    so3_series(w, a, b, R);
}
void sv_so3_log(const double R[9], double w[3]) {
    double c = (R[0] + R[4] + R[8] - 1.0) * 0.5, th, v[3], s;
    if (c > 1.0) c = 1.0;
    if (c < -1.0) c = -1.0;
    th = acos(c);
    v[0] = R[7] - R[5]; v[1] = R[2] - R[6]; v[2] = R[3] - R[1];
    s = sin(th);
    if (th < 1e-6) { w[0] = 0.5 * v[0]; w[1] = 0.5 * v[1]; w[2] = 0.5 * v[2]; return; }
    if (3.14159265358979323846 - th > 1e-3) {
        double f = th / (2.0 * s);
        w[0] = f * v[0]; w[1] = f * v[1]; w[2] = f * v[2]; return;
    } else { /* near pi: axis from R + I = 2 n n^T (sign from the antisymmetric part) */
        int k = 0, i; double n[3], d;
        for (i = 1; i < 3; ++i) if (R[4 * i] > R[4 * k]) k = i;
        d = sqrt(fmax(R[4 * k] - c, 1e-12) / (1.0 - c));
        n[k] = d;
        for (i = 0; i < 3; ++i) if (i != k) n[i] = (R[3 * k + i] + R[3 * i + k]) / (2.0 * (1.0 - c) * d);
        if (n[0] * v[0] + n[1] * v[1] + n[2] * v[2] < 0) { n[0] = -n[0]; n[1] = -n[1]; n[2] = -n[2]; }
        w[0] = th * n[0]; w[1] = th * n[1]; w[2] = th * n[2];
    }
}
void sv_so3_jr(const double w[3], double J[9]) {
    double th2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2], th = sqrt(th2), a, b;
    if (th < 1e-4) { a = -0.5 + th2 / 24.0; b = 1.0 / 6.0 - th2 / 120.0; }
    else { a = -(1.0 - cos(th)) / th2; b = (th - sin(th)) / (th2 * th); }
    so3_series(w, a, b, J);
}
void sv_so3_jr_inv(const double w[3], double J[9]) {
    double th2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2], th = sqrt(th2), b;
    if (th < 1e-3) b = 1.0 / 12.0 + th2 / 720.0;
    else b = 1.0 / th2 - (1.0 + cos(th)) / (2.0 * th * sin(th));
    so3_series(w, 0.5, b, J);
}

/* ---------------- preintegration ---------------- */
void sv_imu_preint_init(sv_imu_preint* p, const double bg[3], const double ba[3]) {
    memset(p, 0, sizeof *p);
    p->dR[0] = p->dR[4] = p->dR[8] = 1.0;
    memcpy(p->bg, bg, sizeof p->bg); memcpy(p->ba, ba, sizeof p->ba);
}

static void mm9(const double* A, const double* B, double* C) { /* C = A B, 9x9 row-major */
    int i, j, k;
    for (i = 0; i < 9; ++i) for (j = 0; j < 9; ++j) {
        double s = 0; for (k = 0; k < 9; ++k) s += A[9 * i + k] * B[9 * k + j];
        C[9 * i + j] = s;
    }
}

/* One interval with constant measurement (gyr, acc) over dt. Rotation: exact exponential. Acceleration is rotated with the
 * mid-interval rotation C_mid = dR Exp(w dt/2) (second-order accurate, like the trapezoid rule); covariance and bias Jacobians are the
 * exact first-order linearisation of THIS discrete update (Forster et al. 2017 structure, midpoint variant). */
void sv_imu_preint_add(sv_imu_preint* p, const double gyr[3], const double acc[3], double dt, const sv_imu_noise* nz) {
    double w[3], a[3], wdt[3], whalf[3], dRinc[9], Em[9], Jr[9], Jrh[9], K[9], Cm[9], CK[9], CKE[9], CKJ[9], Cma[3], tmp[9], dRn[9];
    double A[81], AP[81], Pn[81], B[54], BS[54];
    double JR[9], Jmid[9], Jvbgn[9], Jpbgn[9], Jvban[9], Jpban[9], JRn[9];
    int i, j, k;
    for (i = 0; i < 3; ++i) { w[i] = gyr[i] - p->bg[i]; a[i] = acc[i] - p->ba[i]; wdt[i] = w[i] * dt; whalf[i] = 0.5 * wdt[i]; }
    sv_so3_exp(wdt, dRinc); sv_so3_exp(whalf, Em);
    sv_so3_jr(wdt, Jr); sv_so3_jr(whalf, Jrh);
    sv_m3_mul(p->dR, Em, Cm);
    sv_m3_hat(a, K);
    sv_m3_mul(Cm, K, CK);                    /* C_mid [a]x */
    sv_m3_mulv(Cm, a, Cma);

    /* covariance: P' = A P A^T + B S B^T,  state [dtheta dv dp], noise [eta_g (sg^2/dt), eta_a (sa^2/dt)] */
    memset(A, 0, sizeof A); memset(B, 0, sizeof B);
    for (i = 0; i < 9; ++i) A[10 * i] = 1.0;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) A[9 * i + j] = dRinc[3 * j + i];
    { double EmT[9]; for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) EmT[3 * i + j] = Em[3 * j + i];
      sv_m3_mul(CK, EmT, CKE);
      for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {
          A[9 * (3 + i) + j] = -dt * CKE[3 * i + j];
          A[9 * (6 + i) + j] = -0.5 * dt * dt * CKE[3 * i + j];
      } }
    for (i = 0; i < 3; ++i) A[9 * (6 + i) + 3 + i] = dt;
    /* B is 9x6 row-major: columns 0-2 gyro noise, 3-5 accel noise */
    sv_m3_mul(CK, Jrh, CKJ);
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) {
        B[6 * i + j] = Jr[3 * i + j] * dt;
        B[6 * (3 + i) + j] = -0.5 * dt * dt * CKJ[3 * i + j];
        B[6 * (3 + i) + 3 + j] = Cm[3 * i + j] * dt;
        B[6 * (6 + i) + j] = -0.25 * dt * dt * dt * CKJ[3 * i + j];
        B[6 * (6 + i) + 3 + j] = 0.5 * dt * dt * Cm[3 * i + j];
    }
    mm9(A, p->cov, AP);
    for (i = 0; i < 9; ++i) for (j = 0; j < 9; ++j) {
        double s = 0; for (k = 0; k < 9; ++k) s += AP[9 * i + k] * A[9 * j + k];
        Pn[9 * i + j] = s;
    }
    if (nz) {
        double sg2 = nz->sigma_g * nz->sigma_g / dt, sa2 = nz->sigma_a * nz->sigma_a / dt;
        for (i = 0; i < 9; ++i) for (k = 0; k < 6; ++k) BS[6 * i + k] = B[6 * i + k] * (k < 3 ? sg2 : sa2);
        for (i = 0; i < 9; ++i) for (j = 0; j < 9; ++j) {
            double s = 0; for (k = 0; k < 6; ++k) s += BS[6 * i + k] * B[6 * j + k];
            Pn[9 * i + j] += s;
        }
    }
    memcpy(p->cov, Pn, sizeof Pn);

    /* bias Jacobians of the discrete update (values before this step on the right) */
    for (i = 0; i < 9; ++i) JR[i] = p->J_Rbg[i];
    { double EmT[9], T[9]; for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) EmT[3 * i + j] = Em[3 * j + i];
      sv_m3_mul(EmT, JR, T);
      for (i = 0; i < 9; ++i) Jmid[i] = T[i] - 0.5 * dt * Jrh[i];
      for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) tmp[3 * i + j] = dRinc[3 * j + i];
      sv_m3_mul(tmp, JR, T);
      for (i = 0; i < 9; ++i) JRn[i] = T[i] - Jr[i] * dt; }
    sv_m3_mul(CK, Jmid, CKJ);                /* C_mid [a]x J_mid */
    for (i = 0; i < 9; ++i) {
        Jvban[i] = p->J_vba[i] - Cm[i] * dt;
        Jpban[i] = p->J_pba[i] + p->J_vba[i] * dt - 0.5 * Cm[i] * dt * dt;
        Jvbgn[i] = p->J_vbg[i] - CKJ[i] * dt;
        Jpbgn[i] = p->J_pbg[i] + p->J_vbg[i] * dt - 0.5 * CKJ[i] * dt * dt;
    }
    memcpy(p->J_Rbg, JRn, sizeof JRn); memcpy(p->J_vba, Jvban, sizeof Jvban); memcpy(p->J_pba, Jpban, sizeof Jpban);
    memcpy(p->J_vbg, Jvbgn, sizeof Jvbgn); memcpy(p->J_pbg, Jpbgn, sizeof Jpbgn);

    /* deltas */
    for (i = 0; i < 3; ++i) {
        p->dp[i] += p->dv[i] * dt + 0.5 * Cma[i] * dt * dt;
        p->dv[i] += Cma[i] * dt;
    }
    sv_m3_mul(p->dR, dRinc, dRn); memcpy(p->dR, dRn, sizeof dRn);
    p->dt += dt; p->n++;
}

int sv_imu_preint_range(sv_imu_preint* p, const sv_imu_buf* b, int64_t t0, int64_t t1,
                        const double bg[3], const double ba[3], const sv_imu_noise* nz) {
    sv_imu_sample s0, s1;
    long i, last;
    int k, cnt = 0;
    sv_imu_preint_init(p, bg, ba);
    if (t1 <= t0) return 0;
    if (sv_imu_buf_interp(b, t0, &s0) || sv_imu_buf_interp(b, t1, &s1)) return -1;
    last = sv_imu_buf_find(b, t1);
    {
        sv_imu_sample cur = s0;
        for (i = sv_imu_buf_find(b, t0) + 1; i <= last + 1; ++i) {
            sv_imu_sample nxt;
            double g[3], a[3], dt;
            if (i <= last && b->s[i].t_ns < t1) nxt = b->s[i];
            else if (i <= last + 1) { nxt = s1; i = last + 2; }
            else break;
            if (nxt.t_ns <= cur.t_ns) continue;
            if (b->max_gap_ns > 0) {
                /* the gap is measured between the stored samples bracketing this interval */
                long j0 = sv_imu_buf_find(b, cur.t_ns), j1 = j0 + 1;
                if (j1 < (long)b->n && b->s[j1].t_ns - b->s[j0].t_ns > b->max_gap_ns) return -2;
            }
            dt = (double)(nxt.t_ns - cur.t_ns) * 1e-9;
            for (k = 0; k < 3; ++k) { g[k] = 0.5 * (cur.gyr[k] + nxt.gyr[k]); a[k] = 0.5 * (cur.acc[k] + nxt.acc[k]); }
            sv_imu_preint_add(p, g, a, dt, nz);
            cnt++;
            cur = nxt;
        }
    }
    return cnt;
}

void sv_imu_preint_corrected(const sv_imu_preint* p, const double bg[3], const double ba[3],
                             double dR[9], double dv[3], double dp[3]) {
    double dbg[3], dba[3], E[9], t[3];
    int k;
    for (k = 0; k < 3; ++k) { dbg[k] = bg[k] - p->bg[k]; dba[k] = ba[k] - p->ba[k]; }
    sv_m3_mulv(p->J_Rbg, dbg, t); sv_so3_exp(t, E); sv_m3_mul(p->dR, E, dR);
    {
        double a[3], b2[3], c[3], d[3];
        sv_m3_mulv(p->J_vbg, dbg, a); sv_m3_mulv(p->J_vba, dba, b2);
        sv_m3_mulv(p->J_pbg, dbg, c); sv_m3_mulv(p->J_pba, dba, d);
        for (k = 0; k < 3; ++k) { dv[k] = p->dv[k] + a[k] + b2[k]; dp[k] = p->dp[k] + c[k] + d[k]; }
    }
}

void sv_imu_preint_residual(const sv_imu_preint* p, const double Ri[9], const double vi[3], const double pi_[3],
                            const double Rj[9], const double vj[3], const double pj[3], const double g[3],
                            const double bg[3], const double ba[3], double r[9]) {
    double dR[9], dv[3], dp[3], A[9], B[9], u[3], x[3];
    int k; double dt = p->dt;
    sv_imu_preint_corrected(p, bg, ba, dR, dv, dp);
    sv_m3_tmul(Ri, Rj, A);                    /* Ri^T Rj */
    sv_m3_tmul(dR, A, B);                     /* dR^T Ri^T Rj */
    sv_so3_log(B, r);
    for (k = 0; k < 3; ++k) u[k] = vj[k] - vi[k] - g[k] * dt;
    sv_m3_tmulv(Ri, u, x);
    for (k = 0; k < 3; ++k) r[3 + k] = x[k] - dv[k];
    for (k = 0; k < 3; ++k) u[k] = pj[k] - pi_[k] - vi[k] * dt - 0.5 * g[k] * dt * dt;
    sv_m3_tmulv(Ri, u, x);
    for (k = 0; k < 3; ++k) r[6 + k] = x[k] - dp[k];
}

void sv_imu_preint_predict(const sv_imu_preint* p, const double Ri[9], const double vi[3], const double pi_[3], const double g[3],
                           const double bg[3], const double ba[3], double Rj[9], double vj[3], double pj[3]) {
    double dR[9], dv[3], dp[3], a[3], b[3]; int k; double dt = p->dt;
    sv_imu_preint_corrected(p, bg, ba, dR, dv, dp);
    sv_m3_mul(Ri, dR, Rj);
    sv_m3_mulv(Ri, dv, a); sv_m3_mulv(Ri, dp, b);
    for (k = 0; k < 3; ++k) {
        vj[k] = vi[k] + g[k] * dt + a[k];
        pj[k] = pi_[k] + vi[k] * dt + 0.5 * g[k] * dt * dt + b[k];
    }
}
