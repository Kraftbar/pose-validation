/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/* See rd_imu.h. Every statement notes the Eigen 3.4.0 expression form it reproduces (lazy 3x3 product = rule A of
 * okvis_port/HANDOVER.md, GEMM product = ok_gemm, temporaries for nested products). */
#include "rd_imu.h"
#include "rd_eigen.h"
#include "../../okvis_port/c/ok_dense.h"
#include <math.h>
#include <string.h>

enum { ES_Q = 0, ES_P = 3, ES_V = 6, ES_BG = 9, ES_BA = 12 };
static const double GRAV[3] = {0.0, 0.0, -RD_GRAVITY_NOMINAL};

/* ---- small helpers: 3x3 column-major ---- */
static void m3_scale(double s, const double a[9], double out[9]) { int i; for (i = 0; i < 9; ++i) out[i] = s * a[i]; }
static void m3_ident(double out[9]) { int i; for (i = 0; i < 9; ++i) out[i] = (i % 4 == 0) ? 1.0 : 0.0; }
static void m3_rot(const ok_quat* q, double out[9]) { ok_quat_to_mat3(q, out); }  /* QuaternionBase::matrix() */
static void m3_rot_conj(const ok_quat* q, double out[9]) { ok_quat qc = rd_quat_conj(*q); ok_quat_to_mat3(&qc, out); }

/* write a 3x3 block into a column-major matrix with leading dimension ld */
static void put3(double* m, int ld, int r0, int c0, const double b[9]) {
    int i, j;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) m[(r0 + i) + ld * (c0 + j)] = b[i + 3 * j];
}

void rd_pi_reset(rd_preint* pi) {
    memset(&pi->delta, 0, sizeof pi->delta);
    pi->delta.q.x = pi->delta.q.y = pi->delta.q.z = 0.0; pi->delta.q.w = 1.0;
    memset(&pi->jac, 0, sizeof pi->jac);
}

void rd_pi_increment(rd_preint* pi, double dt, const rd_imu_sample* d, const double bg[3], const double ba[3], int compute_jacobian,
                     int compute_covariance) {
    double w[3], a[3], wdt[3], R[9], Ec[9], Ha[9], rj[9];
    int i, j;
    ok_quat e;
    for (i = 0; i < 3; ++i) { w[i] = d->w[i] - bg[i]; a[i] = d->a[i] - ba[i]; }
    for (i = 0; i < 3; ++i) wdt[i] = w[i] * dt;
    e = rd_expmap(wdt);
    m3_rot(&pi->delta.q, R);
    m3_rot_conj(&e, Ec);        /* expmap(w*dt).conjugate().matrix() */
    rd_hat(a, Ha);
    rd_right_jacobian(wdt, rj);

    if (compute_covariance) {
        double A[81], B[54], W[36], T1[81], T2[81], T3[54], T4[81], C0[81], At[81], Bt[54], blk[9], t9[9], t9b[9], inv_dt;
        memset(A, 0, sizeof A); memset(B, 0, sizeof B); memset(W, 0, sizeof W);
        for (i = 0; i < 9; ++i) A[i + 9 * i] = 1.0;
        put3(A, 9, ES_Q, ES_Q, Ec);
        /* A.block<3,3>(ES_V,ES_Q) = -dt * R * hat(a)  == ((-dt)*R)*hat(a) */
        m3_scale(-dt, R, t9); ok_m3_mul(t9, Ha, blk); put3(A, 9, ES_V, ES_Q, blk);
        /* A.block<3,3>(ES_P,ES_Q) = -0.5*dt*dt*R*hat(a) == ((((-0.5*dt)*dt)*R)*hat(a) */
        m3_scale((-0.5 * dt) * dt, R, t9); ok_m3_mul(t9, Ha, blk); put3(A, 9, ES_P, ES_Q, blk);
        /* A.block<3,3>(ES_P,ES_V) = dt * Identity */
        m3_ident(t9b); m3_scale(dt, t9b, blk); put3(A, 9, ES_P, ES_V, blk);
        /* B */
        m3_scale(dt, rj, blk); put3(B, 9, ES_Q, 0, blk);
        m3_scale(dt, R, blk); put3(B, 9, ES_V, 3, blk);
        m3_scale((0.5 * dt) * dt, R, blk); put3(B, 9, ES_P, 3, blk);
        inv_dt = 1.0 / (dt > 1.0e-7 ? dt : 1.0e-7);
        m3_scale(inv_dt, pi->cov_w, blk); put3(W, 6, 0, 0, blk);   /* cov_w * inv_dt (matrix * scalar: same product) */
        m3_scale(inv_dt, pi->cov_a, blk); put3(W, 6, 3, 3, blk);
        for (j = 0; j < 9; ++j) for (i = 0; i < 9; ++i) C0[i + 9 * j] = pi->delta.cov[i + 15 * j];
        for (j = 0; j < 9; ++j) for (i = 0; i < 9; ++i) At[i + 9 * j] = A[j + 9 * i];
        for (j = 0; j < 9; ++j) for (i = 0; i < 6; ++i) Bt[i + 6 * j] = B[j + 9 * i];
        /* A * C * A^T + B * W * B^T : four GEMM products into temporaries, then the coefficientwise sum */
        ok_gemm(9, 9, 9, A, C0, T1);
        ok_gemm(9, 9, 9, T1, At, T2);
        ok_gemm(9, 6, 6, B, W, T3);
        ok_gemm(9, 9, 6, T3, Bt, T4);
        for (j = 0; j < 9; ++j) for (i = 0; i < 9; ++i) pi->delta.cov[i + 15 * j] = T2[i + 9 * j] + T4[i + 9 * j];
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) {
            pi->delta.cov[(ES_BG + i) + 15 * (ES_BG + j)] += pi->cov_bg[i + 3 * j] * dt;
            pi->delta.cov[(ES_BA + i) + 15 * (ES_BA + j)] += pi->cov_ba[i + 3 * j] * dt;
        }
    }

    if (compute_jacobian) {
        double X[9], Y[9], t9[9];
        rd_pre_jac* J = &pi->jac;
        /* dp_dbg += dt * dv_dbg - 0.5*dt*dt*R*hat(a)*dq_dbg */
        m3_scale((0.5 * dt) * dt, R, t9); ok_m3_mul(t9, Ha, Y); ok_m3_mul(Y, J->dq_dbg, X);
        for (i = 0; i < 9; ++i) J->dp_dbg[i] = J->dp_dbg[i] + (dt * J->dv_dbg[i] - X[i]);
        /* dp_dba += dt * dv_dba - 0.5*dt*dt*R */
        for (i = 0; i < 9; ++i) J->dp_dba[i] = J->dp_dba[i] + (dt * J->dv_dba[i] - ((0.5 * dt) * dt) * R[i]);
        /* dv_dbg -= dt * R * hat(a) * dq_dbg */
        m3_scale(dt, R, t9); ok_m3_mul(t9, Ha, Y); ok_m3_mul(Y, J->dq_dbg, X);
        for (i = 0; i < 9; ++i) J->dv_dbg[i] = J->dv_dbg[i] - X[i];
        /* dv_dba -= dt * R */
        for (i = 0; i < 9; ++i) J->dv_dba[i] = J->dv_dba[i] - dt * R[i];
        /* dq_dbg = expmap(w*dt)^* * dq_dbg - dt * right_jacobian(w*dt) */
        ok_m3_mul(Ec, J->dq_dbg, X);
        for (i = 0; i < 9; ++i) J->dq_dbg[i] = X[i] - dt * rj[i];
    }

    {
        double qa[3], p_new[3], v_new[3];
        rd_quat_rotate(&pi->delta.q, a, qa);
        pi->delta.t = pi->delta.t + dt;
        for (i = 0; i < 3; ++i) {
            p_new[i] = (pi->delta.p[i] + dt * pi->delta.v[i]) + ((0.5 * dt) * dt) * qa[i];
            v_new[i] = pi->delta.v[i] + dt * qa[i];
        }
        for (i = 0; i < 3; ++i) { pi->delta.p[i] = p_new[i]; pi->delta.v[i] = v_new[i]; }
        {
            ok_quat q;
            ok_quat_mul(&pi->delta.q, &e, &q);
            pi->delta.q = ok_quat_normalized(q);
        }
    }
}

void rd_pi_compute_sqrt_inv_cov(rd_preint* pi) {
    double inv[225];
    rd_inverse_ppl(15, pi->delta.cov, inv);
    rd_llt_sqrt_info(15, inv, pi->delta.sqrt_inv_cov);
}

int rd_pi_integrate(rd_preint* pi, const rd_imu_sample* data, int n, double t, const double bg[3], const double ba[3],
                    int compute_jacobian, int compute_covariance) {
    int i;
    if (n == 0) return 0;
    rd_pi_reset(pi);
    for (i = 0; i + 1 < n; ++i)
        rd_pi_increment(pi, data[i + 1].t - data[i].t, &data[i], bg, ba, compute_jacobian, compute_covariance);
    rd_pi_increment(pi, t - data[n - 1].t, &data[n - 1], bg, ba, compute_jacobian, compute_covariance);
    if (compute_covariance) rd_pi_compute_sqrt_inv_cov(pi);
    return 1;
}

void rd_pi_predict(const rd_preint* pi, const rd_pose* op, const rd_motion* om, rd_pose* np, rd_motion* nm) {
    double dv_r[3], dp_r[3], g_half[3], g_t[3];
    int i;
    const double t = pi->delta.t;
    memcpy(nm->bg, om->bg, sizeof nm->bg);
    memcpy(nm->ba, om->ba, sizeof nm->ba);
    rd_quat_rotate(&op->q, pi->delta.v, dv_r);
    rd_quat_rotate(&op->q, pi->delta.p, dp_r);
    for (i = 0; i < 3; ++i) { g_t[i] = GRAV[i] * t; g_half[i] = ((0.5 * GRAV[i]) * t) * t; }
    for (i = 0; i < 3; ++i) {
        const double v = (om->v[i] + g_t[i]) + dv_r[i];
        const double p = ((op->p[i] + g_half[i]) + om->v[i] * t) + dp_r[i];
        nm->v[i] = v; np->p[i] = p;
    }
    ok_quat_mul(&op->q, &pi->delta.q, &np->q);
}

void rd_quat_plus(const double q[4], const double dq[3], double out[4]) {
    ok_quat a, e, r;
    a.x = q[0]; a.y = q[1]; a.z = q[2]; a.w = q[3];
    e = rd_expmap(dq);
    ok_quat_mul(&a, &e, &r);
    r = ok_quat_normalized(r);
    out[0] = r.x; out[1] = r.y; out[2] = r.z; out[3] = r.w;
}

/* ---- Evaluate ---- */
static void matvec3(const double m[9], const double v[3], double out[3]) { ok_m3_mulv(m, v, out); }

void rd_pie_eval(const rd_preint* pre, const ok_quat* imu_i_q, const double imu_i_p[3], const ok_quat* imu_j_q, const double imu_j_p[3],
                 const double bg_i_0[3], const double ba_i_0[3], const double* const params[10], double residual[15], double* jac[10]) {
    static const int SZ[10] = {4, 3, 3, 3, 3, 4, 3, 3, 3, 3};
    ok_quat qci, qcj, qi, qj, qic, qcic;
    const double *pci = params[1], *vi = params[2], *bgi = params[3], *bai = params[4];
    const double *pcj = params[6], *vj = params[7], *bgj = params[8], *baj = params[9];
    double pi_[3], pj[3], rot[3], dbg[3], dba[3], r[15], t[3], x[3], y[3], z[3];
    const double dt = pre->delta.t;
    const rd_pre_jac* J = &pre->jac;
    int i, k;
    qci.x = params[0][0]; qci.y = params[0][1]; qci.z = params[0][2]; qci.w = params[0][3];
    qcj.x = params[5][0]; qcj.y = params[5][1]; qcj.z = params[5][2]; qcj.w = params[5][3];
    ok_quat_mul(&qci, imu_i_q, &qi);
    rd_quat_rotate(&qci, imu_i_p, rot); for (i = 0; i < 3; ++i) pi_[i] = pci[i] + rot[i];
    ok_quat_mul(&qcj, imu_j_q, &qj);
    rd_quat_rotate(&qcj, imu_j_p, rot); for (i = 0; i < 3; ++i) pj[i] = pcj[i] + rot[i];
    for (i = 0; i < 3; ++i) { dbg[i] = bgi[i] - bg_i_0[i]; dba[i] = bai[i] - ba_i_0[i]; }
    qic = rd_quat_conj(qi);

    /* r.Q = logmap((dq * expmap(dq_dbg*dbg))^* * q_i^* * q_j) */
    {
        ok_quat e, a, b, c;
        matvec3(J->dq_dbg, dbg, t);
        e = rd_expmap(t);
        ok_quat_mul(&pre->delta.q, &e, &a);
        a = rd_quat_conj(a);
        ok_quat_mul(&a, &qic, &b);
        ok_quat_mul(&b, &qj, &c);
        rd_logmap(&c, &r[ES_Q]);
    }
    /* r.P = q_i^* (p_j - p_i - dt v_i - 0.5 dt dt g) - (dp + dp_dbg dbg + dp_dba dba) */
    {
        double w1[3], m1[3], m2[3];
        for (i = 0; i < 3; ++i) x[i] = ((pj[i] - pi_[i]) - dt * vi[i]) - (((0.5 * dt) * dt) * GRAV[i]);
        rd_quat_rotate(&qic, x, y);
        matvec3(J->dp_dbg, dbg, m1); matvec3(J->dp_dba, dba, m2);
        for (i = 0; i < 3; ++i) { w1[i] = (pre->delta.p[i] + m1[i]) + m2[i]; r[ES_P + i] = y[i] - w1[i]; }
    }
    /* r.V = q_i^* (v_j - v_i - dt g) - (dv + dv_dbg dbg + dv_dba dba) */
    {
        double w1[3], m1[3], m2[3];
        for (i = 0; i < 3; ++i) z[i] = (vj[i] - vi[i]) - dt * GRAV[i];
        rd_quat_rotate(&qic, z, t);
        matvec3(J->dv_dbg, dbg, m1); matvec3(J->dv_dba, dba, m2);
        for (i = 0; i < 3; ++i) { w1[i] = (pre->delta.v[i] + m1[i]) + m2[i]; r[ES_V + i] = t[i] - w1[i]; }
    }
    for (i = 0; i < 3; ++i) { r[ES_BG + i] = bgj[i] - bgi[i]; r[ES_BA + i] = baj[i] - bai[i]; }

    if (jac) {
        double Jr_inv[9], negJr_inv[9], tmp[9], tmp2[9], blk[9], Hh[9], Rqjc[9], Rci[9], Ric[9], Rcj_c[9], Rcsi_c[9], Qcj[9], Qci[9];
        double Jm[15 * 4];
        double rq[3] = {r[ES_Q], r[ES_Q + 1], r[ES_Q + 2]};
        double rj_[9];
        qcic = rd_quat_conj(qci);
        rd_right_jacobian(rq, rj_);
        rd_inverse3(rj_, Jr_inv);
        for (i = 0; i < 9; ++i) negJr_inv[i] = -Jr_inv[i];
        m3_rot_conj(&qj, Rqjc);
        m3_rot(&qci, Qci);
        m3_rot(&qcj, Qcj);
        m3_rot_conj(&qi, Ric);
        m3_rot_conj(imu_i_q, Rcsi_c);
        m3_rot_conj(imu_j_q, Rcj_c);
        (void)Rci; (void)tmp2;
        for (k = 0; k < 10; ++k) {
            const int n = SZ[k];
            if (!jac[k]) continue;
            memset(Jm, 0, sizeof(double) * 15 * n);
            switch (k) {
            case 0:  /* q_i */
                ok_m3_mul(negJr_inv, Rqjc, tmp); ok_m3_mul(tmp, Qci, blk); put3(Jm, 15, ES_Q, 0, blk);
                for (i = 0; i < 3; ++i) x[i] = ((pj[i] - pci[i]) - dt * vi[i]) - (((0.5 * dt) * dt) * GRAV[i]);
                rd_quat_rotate(&qcic, x, y); rd_hat(y, Hh); ok_m3_mul(Rcsi_c, Hh, blk); put3(Jm, 15, ES_P, 0, blk);
                for (i = 0; i < 3; ++i) z[i] = (vj[i] - vi[i]) - dt * GRAV[i];
                rd_quat_rotate(&qcic, z, y); rd_hat(y, Hh); ok_m3_mul(Rcsi_c, Hh, blk); put3(Jm, 15, ES_V, 0, blk);
                break;
            case 1:  /* p_i */
                for (i = 0; i < 9; ++i) blk[i] = -Ric[i];
                put3(Jm, 15, ES_P, 0, blk);
                break;
            case 2:  /* v_i */
                m3_scale(-dt, Ric, blk); put3(Jm, 15, ES_P, 0, blk);
                for (i = 0; i < 9; ++i) blk[i] = -Ric[i];
                put3(Jm, 15, ES_V, 0, blk);
                break;
            case 3: { /* bg_i */
                double dbgv[3], rjb[9], E[9];
                ok_quat ex = rd_expmap(rq);
                matvec3(J->dq_dbg, dbg, dbgv);
                rd_right_jacobian(dbgv, rjb);
                m3_rot_conj(&ex, E);
                ok_m3_mul(negJr_inv, E, tmp); ok_m3_mul(tmp, rjb, tmp2); ok_m3_mul(tmp2, J->dq_dbg, blk); put3(Jm, 15, ES_Q, 0, blk);
                for (i = 0; i < 9; ++i) blk[i] = -J->dp_dbg[i];
                put3(Jm, 15, ES_P, 0, blk);
                for (i = 0; i < 9; ++i) blk[i] = -J->dv_dbg[i];
                put3(Jm, 15, ES_V, 0, blk);
                m3_ident(tmp); for (i = 0; i < 9; ++i) blk[i] = -tmp[i];
                put3(Jm, 15, ES_BG, 0, blk);
                break; }
            case 4:  /* ba_i */
                for (i = 0; i < 9; ++i) blk[i] = -J->dp_dba[i];
                put3(Jm, 15, ES_P, 0, blk);
                for (i = 0; i < 9; ++i) blk[i] = -J->dv_dba[i];
                put3(Jm, 15, ES_V, 0, blk);
                m3_ident(tmp); for (i = 0; i < 9; ++i) blk[i] = -tmp[i];
                put3(Jm, 15, ES_BA, 0, blk);
                break;
            case 5:  /* q_j */
                ok_m3_mul(Jr_inv, Rcj_c, blk); put3(Jm, 15, ES_Q, 0, blk);
                for (i = 0; i < 9; ++i) tmp[i] = -Ric[i];
                ok_m3_mul(tmp, Qcj, tmp2); rd_hat(imu_j_p, Hh); ok_m3_mul(tmp2, Hh, blk); put3(Jm, 15, ES_P, 0, blk);
                break;
            case 6:  put3(Jm, 15, ES_P, 0, Ric); break;
            case 7:  put3(Jm, 15, ES_V, 0, Ric); break;
            case 8:  m3_ident(blk); put3(Jm, 15, ES_BG, 0, blk); break;
            default: m3_ident(blk); put3(Jm, 15, ES_BA, 0, blk); break;
            }
            {   /* dr = sqrt_inv_cov * dr  (aliasing -> GEMM into a column-major temporary), stored row-major */
                double out[15 * 4];
                int jj;
                ok_gemm(15, n, 15, pre->delta.sqrt_inv_cov, Jm, out);
                for (jj = 0; jj < n; ++jj) for (i = 0; i < 15; ++i) jac[k][i * n + jj] = out[i + 15 * jj];
            }
        }
    }
    {   /* r = sqrt_inv_cov * r : GEMV into a temporary */
        double out[15];
        for (i = 0; i < 15; ++i) out[i] = 0.0;
        ok_gemv_col(15, 15, pre->delta.sqrt_inv_cov, 15, r, 1, out, 1.0);
        memcpy(residual, out, sizeof out);
    }
}
