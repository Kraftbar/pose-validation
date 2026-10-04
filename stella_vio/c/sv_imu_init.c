/* SPDX-License-Identifier: MIT */
/* See sv_imu_init.h. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include "sv_imu_init.h"

/* ---------------- dense helpers ---------------- */
static int chol(int n, double* A) { /* in-place lower Cholesky of the n x n row-major matrix; 0 on failure */
    int i, j, k;
    for (i = 0; i < n; ++i) for (j = 0; j <= i; ++j) {
        double s = A[(size_t)i * n + j];
        for (k = 0; k < j; ++k) s -= A[(size_t)i * n + k] * A[(size_t)j * n + k];
        if (i == j) { if (!(s > 0)) return 0; A[(size_t)i * n + i] = sqrt(s); }
        else A[(size_t)i * n + j] = s / A[(size_t)j * n + j];
    }
    return 1;
}
static void chol_solve(int n, const double* L, double* x) { /* x = (L L^T)^-1 x */
    int i, k;
    for (i = 0; i < n; ++i) { double s = x[i]; for (k = 0; k < i; ++k) s -= L[(size_t)i * n + k] * x[k]; x[i] = s / L[(size_t)i * n + i]; }
    for (i = n - 1; i >= 0; --i) { double s = x[i]; for (k = i + 1; k < n; ++k) s -= L[(size_t)k * n + i] * x[k]; x[i] = s / L[(size_t)i * n + i]; }
}
static void row_add(double* H, double* g, int n, int nnz, const int* idx, const double* val, double rhs, double w) {
    int a, c; double w2 = w * w;
    for (a = 0; a < nnz; ++a) {
        g[idx[a]] += w2 * val[a] * rhs;
        for (c = 0; c < nnz; ++c) H[(size_t)idx[a] * n + idx[c]] += w2 * val[a] * val[c];
    }
}
static double norm3(const double a[3]) { return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
static void cross3(const double a[3], const double b[3], double c[3]) {
    c[0] = a[1] * b[2] - a[2] * b[1]; c[1] = a[2] * b[0] - a[0] * b[2]; c[2] = a[0] * b[1] - a[1] * b[0];
}

void sv_vi_cfg_default(sv_vi_cfg* c) {
    c->estimate_ba = 1; c->max_iter = 12;
    c->sigma_pos_floor = 0.03; c->sigma_vel_floor = 0.1; c->sigma_rot_floor = 0.01;
    c->sigma_bg_prior = 0.1; c->sigma_ba_prior = 0.3;
    c->max_sigma_log_scale = 0.25; c->max_sigma_grav_deg = 5.0;
}
void sv_vi_result_free(sv_vi_result* r) { free(r->v); r->v = NULL; }

/* body rotations R_VB,i = R_VC,i R_CB */
static void body_rot(const sv_vi_kf* kf, const sv_vi_ext* e, double R[9]) {
    double T[9]; int i, j;
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) T[3 * i + j] = e->R_BC[3 * j + i];   /* R_CB */
    sv_m3_mul(kf->R_VC, T, R);
}

static int integrate_all(const sv_imu_buf* b, const sv_vi_kf* kf, int n, const sv_imu_noise* nz, const double bg[3], const double ba[3], sv_imu_preint* pre) {
    int i;
    for (i = 0; i + 1 < n; ++i) {
        int r = sv_imu_preint_range(&pre[i], b, kf[i].t_ns, kf[i + 1].t_ns, bg, ba, nz);
        if (r < 0) return -2;
        if (r == 0) return -2;
    }
    return 0;
}

static double rot_rms(const sv_imu_preint* pre, const double* Rb, int n, const double bg[3]) {
    double s = 0; int i;
    static const double z[3] = {0, 0, 0};
    for (i = 0; i + 1 < n; ++i) {
        double dR[9], dv[3], dp[3], A[9], B[9], r[3];
        sv_imu_preint_corrected(&pre[i], bg, z, dR, dv, dp);
        sv_m3_tmul(&Rb[9 * i], &Rb[9 * (i + 1)], A);
        sv_m3_tmul(dR, A, B); sv_so3_log(B, r);
        s += r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
    }
    return sqrt(s / (n - 1));
}

int sv_vi_gyro_bias(const sv_imu_buf* b, const sv_vi_kf* kf, int n, const sv_vi_ext* ext, const sv_imu_noise* nz, double bg[3], double* rms_before, double* rms_after) {
    static const double z[3] = {0, 0, 0};
    sv_imu_preint* pre; double* Rb; int i, it, rc = 0;
    if (n < 2) return -1;
    pre = (sv_imu_preint*)malloc((size_t)(n - 1) * sizeof *pre); Rb = (double*)malloc((size_t)n * 9 * sizeof(double));
    if (!pre || !Rb) { free(pre); free(Rb); return -5; }
    for (i = 0; i < n; ++i) body_rot(&kf[i], ext, &Rb[9 * i]);
    if (rms_before) {
        if ((rc = integrate_all(b, kf, n, nz, z, z, pre))) goto done;
        *rms_before = rot_rms(pre, Rb, n, z);
    }
    for (it = 0; it < 4; ++it) {
        double H[9] = {0}, g[3] = {0}, d[3], dd; int a, c;
        if ((rc = integrate_all(b, kf, n, nz, bg, z, pre))) goto done;
        for (i = 0; i + 1 < n; ++i) {
            double A[9], B[9], r[3], Ji[9], J[9];
            sv_m3_tmul(&Rb[9 * i], &Rb[9 * (i + 1)], A); sv_m3_tmul(pre[i].dR, A, B); sv_so3_log(B, r);
            { double nr[3] = {-r[0], -r[1], -r[2]}; sv_so3_jr_inv(nr, Ji); }  /* left Jacobian inverse */
            sv_m3_mul(Ji, pre[i].J_Rbg, J);
            for (a = 0; a < 3; ++a) for (c = 0; c < 3; ++c) { H[3 * a + c] += J[3 * 0 + a] * J[c] + J[3 + a] * J[3 + c] + J[6 + a] * J[6 + c]; }
            for (a = 0; a < 3; ++a) g[a] -= (J[a] * r[0] + J[3 + a] * r[1] + J[6 + a] * r[2]);   /* -J^T r */
        }
        /* r(d) = r - Jl^-1(r) J_Rbg d = r - J d: normal equations J^T J d = J^T r */
        for (a = 0; a < 3; ++a) { g[a] = -g[a]; H[4 * a] += 1e-12; }
        { double L[9]; memcpy(L, H, sizeof L); if (!chol(3, L)) { rc = -3; goto done; } memcpy(d, g, sizeof d); chol_solve(3, L, d); }
        for (a = 0; a < 3; ++a) bg[a] += d[a];
        dd = norm3(d);
        if (dd < 1e-7) break;
    }
    if ((rc = integrate_all(b, kf, n, nz, bg, z, pre))) goto done;
    if (rms_after) *rms_after = rot_rms(pre, Rb, n, bg);
done:
    free(pre); free(Rb);
    return rc;
}

/* ---------------- main initialiser ---------------- */
typedef struct {
    int n; const sv_vi_kf* kf; const sv_vi_ext* ext; const double* Rb; const sv_imu_preint* pre;
    const sv_vi_cfg* cfg; double gravity;
    double *sp, *sv, *sr;   /* per-pair sigmas */
} vi_ctx;

static void pair_sigmas(const sv_imu_preint* p, const sv_vi_cfg* c, double* sr, double* sp, double* sv) {
    double a = 0, b = 0, d = 0; int k;
    for (k = 0; k < 3; ++k) { a += p->cov[10 * k]; b += p->cov[10 * (3 + k)]; d += p->cov[10 * (6 + k)]; }
    *sr = sqrt(a / 3 + c->sigma_rot_floor * c->sigma_rot_floor);
    *sv = sqrt(b / 3 + c->sigma_vel_floor * c->sigma_vel_floor);
    *sp = sqrt(d / 3 + c->sigma_pos_floor * c->sigma_pos_floor);
}

/* closed-form linear step; x = [S, g(3), v_0 ... v_{N-1}] */
static int linear_step(const vi_ctx* cx, double* S, double g[3], double* v) {
    int N = cx->n, nu = 4 + 3 * N, i, k, rc = 0;
    double* H = (double*)calloc((size_t)nu * nu, sizeof(double)); double* rhs = (double*)calloc((size_t)nu, sizeof(double));
    if (!H || !rhs) { free(H); free(rhs); return -5; }
    for (i = 0; i + 1 < N; ++i) {
        const sv_imu_preint* p = &cx->pre[i];
        const double *Ri = &cx->Rb[9 * i], *Rj = &cx->Rb[9 * (i + 1)];
        double dt = p->dt, Rdp[3], Rdv[3], pb[3], ci[3], cj[3];
        static const double z[3] = {0, 0, 0};
        double dR9[9], dvc[3], dpc[3];
        sv_imu_preint_corrected(p, p->bg, z, dR9, dvc, dpc);
        sv_m3_mulv(Ri, dpc, Rdp); sv_m3_mulv(Ri, dvc, Rdv);
        { double T[9]; for (k = 0; k < 9; ++k) T[k] = Rj[k] - Ri[k]; sv_m3_mulv(T, cx->ext->p_BC, pb); }
        for (k = 0; k < 3; ++k) { ci[k] = cx->kf[i].c[k]; cj[k] = cx->kf[i + 1].c[k]; }
        {
            double sr, sp, sv_; pair_sigmas(p, cx->cfg, &sr, &sp, &sv_);
            for (k = 0; k < 3; ++k) {
                int idx[4]; double val[4];
                idx[0] = 0; val[0] = cj[k] - ci[k];
                idx[1] = 1 + k; val[1] = -0.5 * dt * dt;
                idx[2] = 4 + 3 * i + k; val[2] = -dt;
                row_add(H, rhs, nu, 3, idx, val, Rdp[k] + pb[k], 1.0 / sp);
                idx[0] = 1 + k; val[0] = -dt;
                idx[1] = 4 + 3 * (i + 1) + k; val[1] = 1.0;
                idx[2] = 4 + 3 * i + k; val[2] = -1.0;
                row_add(H, rhs, nu, 3, idx, val, Rdv[k], 1.0 / sv_);
            }
        }
    }
    for (i = 0; i < nu; ++i) H[(size_t)i * nu + i] += 1e-9 * (H[(size_t)i * nu + i] + 1.0);
    if (!chol(nu, H)) rc = -3;
    else {
        chol_solve(nu, H, rhs);
        *S = rhs[0]; for (k = 0; k < 3; ++k) g[k] = rhs[1 + k];
        for (i = 0; i < 3 * N; ++i) v[i] = rhs[4 + i];
    }
    free(H); free(rhs);
    return rc;
}

int sv_vi_init(const sv_imu_buf* b, const sv_vi_kf* kf, int n, const sv_vi_ext* ext, const sv_imu_noise* nz, const sv_vi_cfg* cfg, sv_vi_result* r) {
    static const double z[3] = {0, 0, 0};
    sv_imu_preint* pre = NULL; double *Rb = NULL, *sr = NULL, *sp = NULL, *svv = NULL, *H = NULL, *gv = NULL, *Hc = NULL;
    double bg[3] = {0, 0, 0}, ba[3] = {0, 0, 0}, S, gl[3], gu[3], rb0, rb1, G = nz->gravity;
    int i, k, it, rc = 0, nu;
    vi_ctx cx;
    memset(r, 0, sizeof *r);
    if (n < 3) return -1;
    r->n_kf = n; r->span_s = (double)(kf[n - 1].t_ns - kf[0].t_ns) * 1e-9;
    r->v = (double*)calloc((size_t)n * 3, sizeof(double));
    pre = (sv_imu_preint*)malloc((size_t)(n - 1) * sizeof *pre); Rb = (double*)malloc((size_t)n * 9 * sizeof(double));
    sr = (double*)malloc((size_t)(n - 1) * sizeof(double)); sp = (double*)malloc((size_t)(n - 1) * sizeof(double)); svv = (double*)malloc((size_t)(n - 1) * sizeof(double));
    if (!r->v || !pre || !Rb || !sr || !sp || !svv) { rc = -5; goto done; }
    for (i = 0; i < n; ++i) body_rot(&kf[i], ext, &Rb[9 * i]);

    /* 1. gyro bias */
    if ((rc = sv_vi_gyro_bias(b, kf, n, ext, nz, bg, &rb0, &rb1))) goto done;
    r->rms_rot_before = rb0; r->rms_rot_after = rb1;
    if ((rc = integrate_all(b, kf, n, nz, bg, z, pre))) goto done;
    cx.n = n; cx.kf = kf; cx.ext = ext; cx.Rb = Rb; cx.pre = pre; cx.cfg = cfg; cx.gravity = G; cx.sp = sp; cx.sv = svv; cx.sr = sr;

    /* 2. closed-form scale + gravity + velocities */
    if ((rc = linear_step(&cx, &S, gl, r->v))) goto done;
    r->scale_linear = S;
    if (!(S > 0) || !isfinite(S) || norm3(gl) < 1e-6) { rc = -4; r->scale = S; memcpy(r->g_V, gl, sizeof gl); memcpy(r->bg, bg, sizeof bg); goto done; }
    for (k = 0; k < 3; ++k) gu[k] = gl[k] / norm3(gl);

    /* 3. refinement: params [u=dlogS, dg(2), dba(3), dbg(3), dv_i(3N)] */
    nu = 9 + 3 * n;
    H = (double*)malloc((size_t)nu * nu * sizeof(double)); gv = (double*)malloc((size_t)nu * sizeof(double)); Hc = (double*)malloc((size_t)nu * nu * sizeof(double));
    if (!H || !gv || !Hc) { rc = -5; goto done; }
    for (i = 0; i + 1 < n; ++i) pair_sigmas(&pre[i], cfg, &sr[i], &sp[i], &svv[i]);
    {
        double chi2 = 0, rms_p = 0, rms_v = 0; int final_pass = 0;
        for (it = 0; it <= cfg->max_iter; ++it) {
            double b1[3], b2[3], ax[3] = {0, 0, 0}, gcur[3], step = 0;
            int m;
            memset(H, 0, (size_t)nu * nu * sizeof(double)); memset(gv, 0, (size_t)nu * sizeof(double));
            chi2 = 0; rms_p = 0; rms_v = 0;
            { int am = (fabs(gu[0]) < fabs(gu[1]) && fabs(gu[0]) < fabs(gu[2])) ? 0 : (fabs(gu[1]) < fabs(gu[2]) ? 1 : 2); ax[am] = 1.0; }
            cross3(gu, ax, b1); { double nn = norm3(b1); for (k = 0; k < 3; ++k) b1[k] /= nn; }
            cross3(gu, b1, b2);
            for (k = 0; k < 3; ++k) gcur[k] = G * gu[k];
            for (i = 0; i + 1 < n; ++i) {
                const sv_imu_preint* p = &pre[i];
                const double *Ri = &Rb[9 * i], *Rj = &Rb[9 * (i + 1)];
                double dt = p->dt, dRc[9], dvc[3], dpc[3], T[9], pb[3], Rdp[3], Rdv[3], rp[3], rv[3], rr[3], A[9], B[9], Ji[9], JR[9];
                double RJpba[9], RJpbg[9], RJvba[9], RJvbg[9], Gb1[3], Gb2[3];
                const double* vi = &r->v[3 * i]; const double* vj = &r->v[3 * (i + 1)];
                for (k = 0; k < 9; ++k) T[k] = Rj[k] - Ri[k];
                sv_m3_mulv(T, ext->p_BC, pb);
                sv_imu_preint_corrected(p, bg, ba, dRc, dvc, dpc);
                sv_m3_mulv(Ri, dpc, Rdp); sv_m3_mulv(Ri, dvc, Rdv);
                for (k = 0; k < 3; ++k) {
                    rp[k] = S * (kf[i + 1].c[k] - kf[i].c[k]) - pb[k] - dt * vi[k] - 0.5 * dt * dt * gcur[k] - Rdp[k];
                    rv[k] = vj[k] - vi[k] - dt * gcur[k] - Rdv[k];
                    Gb1[k] = G * b1[k]; Gb2[k] = G * b2[k];
                }
                sv_m3_tmul(Ri, Rj, A); sv_m3_tmul(dRc, A, B); sv_so3_log(B, rr);
                { double nr[3] = {-rr[0], -rr[1], -rr[2]}; sv_so3_jr_inv(nr, Ji); }
                sv_m3_mul(Ji, p->J_Rbg, JR);
                sv_m3_mul(Ri, p->J_pba, RJpba); sv_m3_mul(Ri, p->J_pbg, RJpbg); sv_m3_mul(Ri, p->J_vba, RJvba); sv_m3_mul(Ri, p->J_vbg, RJvbg);
                for (k = 0; k < 3; ++k) {
                    int idx[11]; double val[11]; int c;
                    /* position row k */
                    idx[0] = 0; val[0] = S * (kf[i + 1].c[k] - kf[i].c[k]);
                    idx[1] = 1; val[1] = -0.5 * dt * dt * Gb1[k]; idx[2] = 2; val[2] = -0.5 * dt * dt * Gb2[k];
                    for (c = 0; c < 3; ++c) { idx[3 + c] = 3 + c; val[3 + c] = -RJpba[3 * k + c]; idx[6 + c] = 6 + c; val[6 + c] = -RJpbg[3 * k + c]; }
                    idx[9] = 9 + 3 * i + k; val[9] = -dt;
                    row_add(H, gv, nu, 10, idx, val, -rp[k], 1.0 / sp[i]);
                    /* velocity row k */
                    idx[0] = 1; val[0] = -dt * Gb1[k]; idx[1] = 2; val[1] = -dt * Gb2[k];
                    for (c = 0; c < 3; ++c) { idx[2 + c] = 3 + c; val[2 + c] = -RJvba[3 * k + c]; idx[5 + c] = 6 + c; val[5 + c] = -RJvbg[3 * k + c]; }
                    idx[8] = 9 + 3 * i + k; val[8] = -1.0; idx[9] = 9 + 3 * (i + 1) + k; val[9] = 1.0;
                    row_add(H, gv, nu, 10, idx, val, -rv[k], 1.0 / svv[i]);
                    /* rotation row k (bg only) */
                    for (c = 0; c < 3; ++c) { idx[c] = 6 + c; val[c] = -JR[3 * k + c]; }
                    row_add(H, gv, nu, 3, idx, val, -rr[k], 1.0 / sr[i]);
                    chi2 += (rp[k] / sp[i]) * (rp[k] / sp[i]) + (rv[k] / svv[i]) * (rv[k] / svv[i]) + (rr[k] / sr[i]) * (rr[k] / sr[i]);
                    rms_p += rp[k] * rp[k]; rms_v += rv[k] * rv[k];
                }
            }
            { double sba = cfg->estimate_ba ? cfg->sigma_ba_prior : 1e-6;
              for (m = 0; m < 3; ++m) {
                  int idx[1]; double val[1] = {1.0};
                  idx[0] = 3 + m; row_add(H, gv, nu, 1, idx, val, -ba[m], 1.0 / sba); chi2 += (ba[m] / sba) * (ba[m] / sba);
                  idx[0] = 6 + m; row_add(H, gv, nu, 1, idx, val, -bg[m], 1.0 / cfg->sigma_bg_prior); chi2 += (bg[m] / cfg->sigma_bg_prior) * (bg[m] / cfg->sigma_bg_prior);
              } }
            r->rms_pos = sqrt(rms_p / (3.0 * (n - 1))); r->rms_vel = sqrt(rms_v / (3.0 * (n - 1)));
            memcpy(Hc, H, (size_t)nu * nu * sizeof(double));
            for (i = 0; i < nu; ++i) Hc[(size_t)i * nu + i] += 1e-10 * (Hc[(size_t)i * nu + i] + 1.0);
            if (!chol(nu, Hc)) { rc = -3; goto done; }
            if (final_pass || it == cfg->max_iter) {
                /* covariance of log-scale and gravity tangent from the last assembled system */
                double* e = (double*)calloc((size_t)nu, sizeof(double)); double var_u, var_g = 0; int q;
                if (!e) { rc = -5; goto done; }
                e[0] = 1.0; chol_solve(nu, Hc, e); var_u = e[0];
                for (q = 1; q <= 2; ++q) { memset(e, 0, (size_t)nu * sizeof(double)); e[q] = 1.0; chol_solve(nu, Hc, e); var_g += e[q]; }
                free(e);
                {
                    int dof = 3 * (n - 1) * 3 - (nu - 3); double c2 = dof > 0 ? chi2 / dof : 1.0; double inf = sqrt(c2 > 1.0 ? c2 : 1.0);
                    r->chi2_dof = c2; r->sigma_log_scale = sqrt(var_u) * inf; r->sigma_grav_deg = sqrt(var_g) * inf * 57.29577951308232;
                }
                break;
            }
            chol_solve(nu, Hc, gv);
            /* apply */
            S *= exp(gv[0]);
            for (k = 0; k < 3; ++k) gu[k] += gv[1] * b1[k] + gv[2] * b2[k];
            { double nn = norm3(gu); for (k = 0; k < 3; ++k) gu[k] /= nn; }
            for (k = 0; k < 3; ++k) { ba[k] += gv[3 + k]; bg[k] += gv[6 + k]; }
            for (i = 0; i < 3 * n; ++i) r->v[i] += gv[9 + i];
            for (i = 0; i < 9; ++i) step += gv[i] * gv[i];
            if (sqrt(step) < 1e-9) final_pass = 1;
        }
    }
    r->scale = S;
    for (k = 0; k < 3; ++k) { r->g_V[k] = G * gu[k]; r->bg[k] = bg[k]; r->ba[k] = ba[k]; }
    {   /* rotation rms at the final bg (first-order) */
        r->rms_rot_after = rot_rms(pre, Rb, n, bg);
        r->ok = isfinite(S) && S > 0 && r->sigma_log_scale <= cfg->max_sigma_log_scale && r->sigma_grav_deg <= cfg->max_sigma_grav_deg
                && norm3(ba) < 1.5 && norm3(bg) < 0.5;
    }
done:
    free(pre); free(Rb); free(sr); free(sp); free(svv); free(H); free(gv); free(Hc);
    if (rc < 0 && rc != -4) { free(r->v); r->v = NULL; }
    return rc;
}
