/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 1: IMU propagation / preintegration. See ok_imu.h for notices.
 * "Math on paper": every OKVIS/Eigen expression is written out one equation per line in the exact
 * evaluation order of the reference build (-O2 -ffp-contract=off, Eigen 3.4.0 SSE2). */
#include "ok_imu.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------------------------------------------ */
/* small fixed-size helpers (column-major)                                                           */
/* ------------------------------------------------------------------------------------------------ */

#define M3(a, i, j) (a)[(i) + 3 * (j)]
#define F15(a, i, j) (a)[(i) + 15 * (j)]

static void m3_zero(double a[9]) { memset(a, 0, 9 * sizeof(double)); }

/* okvis::kinematics::crossMx */
static void cross_mx(const double v[3], double out[9]) {
    const double x = v[0], y = v[1], z = v[2];
    M3(out, 0, 0) = 0.0; M3(out, 0, 1) = -z;  M3(out, 0, 2) = y;
    M3(out, 1, 0) = z;   M3(out, 1, 1) = 0.0; M3(out, 1, 2) = -x;
    M3(out, 2, 0) = -y;  M3(out, 2, 1) = x;   M3(out, 2, 2) = 0.0;
}

/* ode::sinc / kinematics::sinc */
static double sinc_(double x) {
    if (fabs(x) > 1e-6) {
        return sin(x) / x;
    } else {
        const double c_2 = 1.0 / 6.0;
        const double c_4 = 1.0 / 120.0;
        const double c_6 = 1.0 / 5040.0;
        const double x_2 = x * x;
        const double x_4 = x_2 * x_2;
        const double x_6 = x_2 * x_2 * x_2;
        return 1.0 - c_2 * x_2 + c_4 * x_4 - c_6 * x_6;
    }
}

/* kinematics::rightJacobian(PhiVec) */
static void right_jacobian(const double phi_vec[3], double out[9]) {
    const double Phi = ok_v3_norm(phi_vec);
    double Phi_x[9], Phi_x2[9];
    int k;
    cross_mx(phi_vec, Phi_x);
    ok_m3_mul(Phi_x, Phi_x, Phi_x2);
    for (k = 0; k < 9; ++k) out[k] = 0.0;
    out[0] = out[4] = out[8] = 1.0;
    if (Phi < 1.0e-4) {
        for (k = 0; k < 9; ++k) out[k] += (-0.5 * Phi_x[k]) + (1.0 / 6.0 * Phi_x2[k]);
    } else {
        const double Phi2 = Phi * Phi;
        const double Phi3 = Phi2 * Phi;
        const double a = -(1.0 - cos(Phi)) / (Phi2);
        const double b = (Phi - sin(Phi)) / Phi3;
        for (k = 0; k < 9; ++k) out[k] += (a * Phi_x[k]) + (b * Phi_x2[k]);
    }
}

/* out = P * Q * P^T (+ nothing) as evaluated by Eigen for  `dst = P*Q*P.transpose()`  (Dense assignment):
 * inner product N, outer product on the transposed problem (measured, okvis_port/reference_tools). */
static void gemm_pqpt(const double P[225], const double Q[225], double out[225]) {
    double t[225], tT[225], r[225];
    int i, j;
    ok_gemm(15, 15, 15, P, Q, t);
    for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) tT[j + 15 * i] = t[i + 15 * j];
    ok_gemm(15, 15, 15, P, tT, r);
    for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) out[i + 15 * j] = r[j + 15 * i];
}

/* PseudoInverse::symmSqrtU (SelfAdjointEigenSolver based) */
static void symm_sqrt_u(const double a[225], double out[225]) {
    double ev[15], V[225], tol, maxc, d[15];
    const double epsilon = 2.220446049250313e-16; /* numeric_limits<double>::epsilon() */
    int i, j;
    ok_selfadjoint_eig15(a, ev, V);
    maxc = ev[0];
    for (i = 1; i < 15; ++i) if (ev[i] > maxc) maxc = ev[i];
    tol = epsilon * 15.0 * maxc;
    if (epsilon > tol) tol = epsilon;                  /* std::max(epsilon, ...) */
    for (i = 0; i < 15; ++i) d[i] = sqrt(ev[i] > tol ? 1.0 / ev[i] : 1.0 / tol);
    for (j = 0; j < 15; ++j)
        for (i = 0; i < 15; ++i) out[i + 15 * j] = d[i] * V[j + 15 * i]; /* diag(d) * V^T */
}

/* ------------------------------------------------------------------------------------------------ */
/* static ImuError::propagation                                                                      */
/* ------------------------------------------------------------------------------------------------ */

int ok_imu_propagation(const ok_imu_meas* meas, size_t n, const ok_imu_params* p, double T_WS[7], double sb[9],
                       ok_time t_start, ok_time t_end, double* cov, double* jac) {
    ok_time time = t_start;
    const ok_time end = t_end;
    ok_quat q_WS_0 = {T_WS[3], T_WS[4], T_WS[5], T_WS[6]};
    double r_0[3] = {T_WS[0], T_WS[1], T_WS[2]};
    double C_WS_0[9];
    ok_quat Delta_q = {0.0, 0.0, 0.0, 1.0};
    double C_integral[9] = {0}, C_doubleintegral[9] = {0}, acc_integral[3] = {0, 0, 0}, acc_doubleintegral[3] = {0, 0, 0};
    double cross[9] = {0}, dalpha_db_g[9] = {0}, dv_db_g[9] = {0}, dp_db_g[9] = {0};
    double P_delta[225];
    double Delta_t = 0.0;
    int hasStarted = 0, steps = 0, k;
    size_t it;

    if (n == 0 || ok_time_lt(meas[n - 1].t, end)) return -1;    /* !(back.timeStamp >= end): nothing to do */
    ok_quat_to_mat3(&q_WS_0, C_WS_0);
    memset(P_delta, 0, sizeof P_delta);

    for (it = 0; it < n; ++it) {
        double omega_S_0[3], acc_S_0[3], omega_S_1[3], acc_S_1[3];
        ok_time nexttime;
        double dt;
        double sigma_g_c = p->sigma_g_c, sigma_a_c = p->sigma_a_c;
        double omega_true[3], acc_true[3], theta_half, sinc_theta_half, cos_theta_half;
        ok_quat dq, Delta_q_1, dq_inv;
        double C[9], C_1[9], C_integral_1[9], acc_integral_1[3], cross_1[9], dv_db_g_1[9];
        double Csum[9], M05[9], M25[9], tmp3[3], accx[9], P1[9], P2[9], rj[9], rjdt[9], Rinv[9];

        for (k = 0; k < 3; ++k) {
            omega_S_0[k] = meas[it].gyr[k];
            acc_S_0[k] = meas[it].acc[k];
        }
        /* (it + 1)->measurement is read before the end check; at the last element it is never used */
        if (it + 1 < n) {
            for (k = 0; k < 3; ++k) { omega_S_1[k] = meas[it + 1].gyr[k]; acc_S_1[k] = meas[it + 1].acc[k]; }
        } else {
            for (k = 0; k < 3; ++k) { omega_S_1[k] = 0.0; acc_S_1[k] = 0.0; }
        }
        if (it + 1 == n) nexttime = t_end; else nexttime = meas[it + 1].t;
        dt = ok_time_diff_sec(nexttime, time);

        if (ok_time_lt(end, nexttime)) {
            const double interval = ok_time_diff_sec(nexttime, meas[it].t);
            double r;
            nexttime = t_end;
            dt = ok_time_diff_sec(nexttime, time);
            r = dt / interval;
            for (k = 0; k < 3; ++k) {
                omega_S_1[k] = (1.0 - r) * omega_S_0[k] + r * omega_S_1[k];
                acc_S_1[k] = (1.0 - r) * acc_S_0[k] + r * acc_S_1[k];
            }
        }
        if (dt <= 0.0) continue;
        Delta_t += dt;

        if (!hasStarted) {
            const double r = dt / ok_time_diff_sec(nexttime, meas[it].t);
            hasStarted = 1;
            for (k = 0; k < 3; ++k) {
                omega_S_0[k] = r * omega_S_0[k] + (1.0 - r) * omega_S_1[k];
                acc_S_0[k] = r * acc_S_0[k] + (1.0 - r) * acc_S_1[k];
            }
        }

        if (fabs(omega_S_0[0]) > p->g_max || fabs(omega_S_0[1]) > p->g_max || fabs(omega_S_0[2]) > p->g_max ||
            fabs(omega_S_1[0]) > p->g_max || fabs(omega_S_1[1]) > p->g_max || fabs(omega_S_1[2]) > p->g_max) {
            sigma_g_c *= 100;
        }
        if (fabs(acc_S_0[0]) > p->a_max || fabs(acc_S_0[1]) > p->a_max || fabs(acc_S_0[2]) > p->a_max ||
            fabs(acc_S_1[0]) > p->a_max || fabs(acc_S_1[1]) > p->a_max || fabs(acc_S_1[2]) > p->a_max) {
            sigma_a_c *= 100;
        }

        /* orientation */
        for (k = 0; k < 3; ++k) omega_true[k] = (0.5 * (omega_S_0[k] + omega_S_1[k])) - sb[3 + k];
        theta_half = ok_v3_norm(omega_true) * 0.5 * dt;
        sinc_theta_half = sinc_(theta_half);
        cos_theta_half = cos(theta_half);
        dq.x = sinc_theta_half * omega_true[0] * 0.5 * dt;
        dq.y = sinc_theta_half * omega_true[1] * 0.5 * dt;
        dq.z = sinc_theta_half * omega_true[2] * 0.5 * dt;
        dq.w = cos_theta_half;
        ok_quat_mul(&Delta_q, &dq, &Delta_q_1);
        ok_quat_to_mat3(&Delta_q, C);
        ok_quat_to_mat3(&Delta_q_1, C_1);
        for (k = 0; k < 3; ++k) acc_true[k] = (0.5 * (acc_S_0[k] + acc_S_1[k])) - sb[6 + k];
        for (k = 0; k < 9; ++k) { Csum[k] = C[k] + C_1[k]; M05[k] = 0.5 * Csum[k]; M25[k] = 0.25 * Csum[k]; }

        /* C_integral_1 = C_integral + 0.5*(C + C_1)*dt */
        for (k = 0; k < 9; ++k) C_integral_1[k] = C_integral[k] + M05[k] * dt;
        /* acc_integral_1 = acc_integral + 0.5*(C + C_1)*acc_S_true*dt */
        ok_m3_mulv(M05, acc_true, tmp3);
        for (k = 0; k < 3; ++k) acc_integral_1[k] = acc_integral[k] + tmp3[k] * dt;
        /* C_doubleintegral += C_integral*dt + 0.25*(C + C_1)*dt*dt */
        for (k = 0; k < 9; ++k) C_doubleintegral[k] += C_integral[k] * dt + M25[k] * dt * dt;
        /* acc_doubleintegral += acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt */
        ok_m3_mulv(M25, acc_true, tmp3);
        for (k = 0; k < 3; ++k) acc_doubleintegral[k] += acc_integral[k] * dt + tmp3[k] * dt * dt;

        /* Jacobian parts: dalpha_db_g += dt*C_1 */
        for (k = 0; k < 9; ++k) dalpha_db_g[k] += dt * C_1[k];
        /* cross_1 = dq.inverse().toRotationMatrix()*cross + rightJacobian(omega_S_true*dt)*dt */
        dq_inv = ok_quat_inverse(dq);
        ok_quat_to_mat3(&dq_inv, Rinv);
        {
            double wdt[3] = {omega_true[0] * dt, omega_true[1] * dt, omega_true[2] * dt};
            right_jacobian(wdt, rj);
        }
        {
            double Rc[9];
            ok_m3_mul(Rinv, cross, Rc);
            for (k = 0; k < 9; ++k) rjdt[k] = rj[k] * dt;
            for (k = 0; k < 9; ++k) cross_1[k] = Rc[k] + rjdt[k];
        }
        cross_mx(acc_true, accx);
        /* P1 = C*acc_S_x*cross, P2 = C_1*acc_S_x*cross_1 */
        {
            double t[9];
            ok_m3_mul(C, accx, t);
            ok_m3_mul(t, cross, P1);
            ok_m3_mul(C_1, accx, t);
            ok_m3_mul(t, cross_1, P2);
        }
        /* dv_db_g_1 = dv_db_g + 0.5*dt*(P1 + P2) ; dp_db_g += dt*dv_db_g + 0.25*dt*dt*(P1 + P2) */
        for (k = 0; k < 9; ++k) dv_db_g_1[k] = dv_db_g[k] + 0.5 * dt * (P1[k] + P2[k]);
        for (k = 0; k < 9; ++k) dp_db_g[k] += dt * dv_db_g[k] + 0.25 * dt * dt * (P1[k] + P2[k]);

        if (cov) {
            double F[225], Pn[225], v3[3], vv[3], blk[9], neg[9];
            int i, j;
            /* F_delta = Identity; blocks as OKVIS */
            for (k = 0; k < 225; ++k) F[k] = 0.0;
            for (k = 0; k < 15; ++k) F15(F, k, k) = 1.0;
            /* (0,3) = -crossMx(acc_integral*dt + 0.25*(C + C_1)*acc_S_true*dt*dt) */
            ok_m3_mulv(M25, acc_true, tmp3);
            for (k = 0; k < 3; ++k) v3[k] = acc_integral[k] * dt + tmp3[k] * dt * dt;
            cross_mx(v3, blk);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 3 + j) = -M3(blk, i, j);
            /* (0,6) = Identity*dt */
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 6 + j) = (i == j ? 1.0 : 0.0) * dt;
            /* (0,9) = dt*dv_db_g + 0.25*dt*dt*(P1 + P2) */
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
                F15(F, i, 9 + j) = dt * M3(dv_db_g, i, j) + 0.25 * dt * dt * (M3(P1, i, j) + M3(P2, i, j));
            /* (0,12) = -C_integral*dt + 0.25*(C + C_1)*dt*dt */
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
                F15(F, i, 12 + j) = (-M3(C_integral, i, j)) * dt + M3(M25, i, j) * dt * dt;
            /* (3,9) = -dt*C_1 */
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 3 + i, 9 + j) = (-dt) * M3(C_1, i, j);
            /* (6,3) = -crossMx(0.5*(C + C_1)*acc_S_true*dt) */
            ok_m3_mulv(M05, acc_true, tmp3);
            for (k = 0; k < 3; ++k) vv[k] = tmp3[k] * dt;
            cross_mx(vv, neg);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 6 + i, 3 + j) = -M3(neg, i, j);
            /* (6,9) = 0.5*dt*(P1 + P2) */
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
                F15(F, 6 + i, 9 + j) = 0.5 * dt * (M3(P1, i, j) + M3(P2, i, j));
            /* (6,12) = -0.5*(C + C_1)*dt */
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
                F15(F, 6 + i, 12 + j) = (-0.5 * M3(Csum, i, j)) * dt;
            gemm_pqpt(F, P_delta, Pn);
            memcpy(P_delta, Pn, sizeof Pn);
            {
                const double sigma2_dalpha = dt * sigma_g_c * sigma_g_c;
                const double sigma2_v = dt * sigma_a_c * p->sigma_a_c;
                const double sigma2_p = 0.5 * dt * dt * sigma2_v;
                const double sigma2_b_g = dt * p->sigma_gw_c * p->sigma_gw_c;
                const double sigma2_b_a = dt * p->sigma_aw_c * p->sigma_aw_c;
                for (k = 0; k < 3; ++k) {
                    F15(P_delta, 3 + k, 3 + k) += sigma2_dalpha;
                    F15(P_delta, 6 + k, 6 + k) += sigma2_v;
                    F15(P_delta, k, k) += sigma2_p;
                    F15(P_delta, 9 + k, 9 + k) += sigma2_b_g;
                    F15(P_delta, 12 + k, 12 + k) += sigma2_b_a;
                }
            }
        }

        /* memory shift */
        Delta_q = Delta_q_1;
        memcpy(C_integral, C_integral_1, sizeof C_integral);
        memcpy(acc_integral, acc_integral_1, sizeof acc_integral);
        memcpy(cross, cross_1, sizeof cross);
        memcpy(dv_db_g, dv_db_g_1, sizeof dv_db_g);
        time = nexttime;
        ++steps;
        if (ok_time_eq(nexttime, t_end)) break;
    }

    /* actual propagation output: g_W = g*Vector3d(0,0,6371009).normalized() = (0,0,g) */
    {
        const double g_W[3] = {p->g * 0.0, p->g * 0.0, p->g * 1.0};
        double Ca[3], Cb[3], rn[3];
        int i;
        ok_m3_mulv(C_WS_0, acc_doubleintegral, Ca);
        for (i = 0; i < 3; ++i) rn[i] = r_0[i] + sb[i] * Delta_t + Ca[i] - 0.5 * g_W[i] * Delta_t * Delta_t;
        {
            ok_quat qn, qq;
            ok_quat_mul(&q_WS_0, &Delta_q, &qq);
            qn = ok_quat_normalized(qq);
            T_WS[0] = rn[0]; T_WS[1] = rn[1]; T_WS[2] = rn[2];
            T_WS[3] = qn.x; T_WS[4] = qn.y; T_WS[5] = qn.z; T_WS[6] = qn.w;
        }
        ok_m3_mulv(C_WS_0, acc_integral, Cb);
        for (i = 0; i < 3; ++i) sb[i] += Cb[i] - g_W[i] * Delta_t;
    }

    if (jac) {
        double v3[3], t[9], blk[9];
        int i, j;
        double* F = jac;
        for (k = 0; k < 225; ++k) F[k] = 0.0;
        for (k = 0; k < 15; ++k) F15(F, k, k) = 1.0;
        ok_m3_mulv(C_WS_0, acc_doubleintegral, v3);
        cross_mx(v3, blk);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 3 + j) = -M3(blk, i, j);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 6 + j) = (i == j ? 1.0 : 0.0) * Delta_t;
        ok_m3_mul(C_WS_0, dp_db_g, t);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 9 + j) = M3(t, i, j);
        {
            double nC[9];
            for (k = 0; k < 9; ++k) nC[k] = -C_WS_0[k];
            ok_m3_mul(nC, C_doubleintegral, t);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 12 + j) = M3(t, i, j);
            ok_m3_mul(nC, dalpha_db_g, t);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 3 + i, 9 + j) = M3(t, i, j);
            ok_m3_mulv(C_WS_0, acc_integral, v3);
            cross_mx(v3, blk);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 6 + i, 3 + j) = -M3(blk, i, j);
            ok_m3_mul(C_WS_0, dv_db_g, t);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 6 + i, 9 + j) = M3(t, i, j);
            ok_m3_mul(nC, C_integral, t);
            for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 6 + i, 12 + j) = M3(t, i, j);
        }
    }
    if (cov) {
        double T[225];
        int i, j;
        for (k = 0; k < 225; ++k) T[k] = 0.0;
        for (k = 0; k < 15; ++k) F15(T, k, k) = 1.0;
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) {
            F15(T, i, j) = M3(C_WS_0, i, j);
            F15(T, 3 + i, 3 + j) = M3(C_WS_0, i, j);
            F15(T, 6 + i, 6 + j) = M3(C_WS_0, i, j);
        }
        gemm_pqpt(T, P_delta, cov);
    }
    return steps;
}

/* ------------------------------------------------------------------------------------------------ */
/* ImuError: preintegration state                                                                    */
/* ------------------------------------------------------------------------------------------------ */

void ok_imu_error_init(ok_imu_error* e, const ok_imu_meas* meas, size_t n, const ok_imu_params* p, ok_time t0, ok_time t1) {
    memset(e, 0, sizeof *e);
    e->params = *p;
    e->t0 = t0;
    e->t1 = t1;
    e->cap_meas = n ? n : 1;
    e->meas = (ok_imu_meas*)malloc(e->cap_meas * sizeof(ok_imu_meas));
    if (n) memcpy(e->meas, meas, n * sizeof(ok_imu_meas));
    e->n_meas = n;
    e->delta_q.w = 1.0;
    e->redo = 1;
}

void ok_imu_error_free(ok_imu_error* e) {
    free(e->meas);
    e->meas = NULL;
    e->n_meas = e->cap_meas = 0;
}

/* one integration step shared by redoPreintegration and append (identical bodies in OKVIS2) */
static void preint_step(ok_imu_error* e, const double sb[9], const double omega_S_0[3], const double acc_S_0[3],
                        const double omega_S_1[3], const double acc_S_1[3], double dt, double gyr_sat_mult,
                        double acc_sat_mult) {
    double omega_true[3], acc_true[3], theta_half, sinc_theta_half, cos_theta_half;
    ok_quat dq, Delta_q_1, dq_inv;
    double C[9], C_1[9], Csum[9], M05[9], M25[9], C_integral_1[9], acc_integral_1[3], tmp3[3];
    double rj[9], wdt[3], Rinv[9], Rc[9], cross_1[9], accx[9], t[9], P1[9], P2[9], dv_db_g_1[9], T1[9];
    double F[225], K[4][225], Pn[225];
    double v3[3], blk[9], vv[3], neg[9];
    int k, i, j, jj;

    for (k = 0; k < 3; ++k) omega_true[k] = (0.5 * (omega_S_0[k] + omega_S_1[k])) - sb[3 + k];
    theta_half = ok_v3_norm(omega_true) * 0.5 * dt;
    sinc_theta_half = sinc_(theta_half);
    cos_theta_half = cos(theta_half);
    dq.x = sinc_theta_half * omega_true[0] * 0.5 * dt;
    dq.y = sinc_theta_half * omega_true[1] * 0.5 * dt;
    dq.z = sinc_theta_half * omega_true[2] * 0.5 * dt;
    dq.w = cos_theta_half;
    ok_quat_mul(&e->delta_q, &dq, &Delta_q_1);
    ok_quat_to_mat3(&e->delta_q, C);
    ok_quat_to_mat3(&Delta_q_1, C_1);
    for (k = 0; k < 3; ++k) acc_true[k] = (0.5 * (acc_S_0[k] + acc_S_1[k])) - sb[6 + k];
    for (k = 0; k < 9; ++k) { Csum[k] = C[k] + C_1[k]; M05[k] = 0.5 * Csum[k]; M25[k] = 0.25 * Csum[k]; }

    for (k = 0; k < 9; ++k) C_integral_1[k] = e->C_integral[k] + M05[k] * dt;
    ok_m3_mulv(M05, acc_true, tmp3);
    for (k = 0; k < 3; ++k) acc_integral_1[k] = e->acc_integral[k] + tmp3[k] * dt;
    for (k = 0; k < 9; ++k) e->C_doubleintegral[k] += e->C_integral[k] * dt + M25[k] * dt * dt;
    ok_m3_mulv(M25, acc_true, tmp3);
    for (k = 0; k < 3; ++k) e->acc_doubleintegral[k] += e->acc_integral[k] * dt + tmp3[k] * dt * dt;

    /* dalpha_db_g += C_1 * rightJacobian(omega_S_true*dt) * dt */
    for (k = 0; k < 3; ++k) wdt[k] = omega_true[k] * dt;
    right_jacobian(wdt, rj);
    ok_m3_mul(C_1, rj, t);
    for (k = 0; k < 9; ++k) e->dalpha_db_g[k] += t[k] * dt;
    /* cross_1 = dq.inverse().toRotationMatrix()*cross + rightJacobian(omega_S_true*dt)*dt */
    dq_inv = ok_quat_inverse(dq);
    ok_quat_to_mat3(&dq_inv, Rinv);
    ok_m3_mul(Rinv, e->cross, Rc);
    for (k = 0; k < 9; ++k) cross_1[k] = Rc[k] + rj[k] * dt;
    cross_mx(acc_true, accx);
    ok_m3_mul(C, accx, T1);
    ok_m3_mul(T1, e->cross, P1);
    ok_m3_mul(C_1, accx, T1);
    ok_m3_mul(T1, cross_1, P2);
    for (k = 0; k < 9; ++k) dv_db_g_1[k] = e->dv_db_g[k] + 0.5 * dt * (P1[k] + P2[k]);
    for (k = 0; k < 9; ++k) e->dp_db_g[k] += dt * e->dv_db_g[k] + 0.25 * dt * dt * (P1[k] + P2[k]);

    /* covariance propagation: F_delta */
    for (k = 0; k < 225; ++k) F[k] = 0.0;
    for (k = 0; k < 15; ++k) F15(F, k, k) = 1.0;
    ok_m3_mulv(M25, acc_true, tmp3);
    for (k = 0; k < 3; ++k) v3[k] = e->acc_integral[k] * dt + tmp3[k] * dt * dt;
    cross_mx(v3, blk);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 3 + j) = -M3(blk, i, j);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, i, 6 + j) = (i == j ? 1.0 : 0.0) * dt;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
        F15(F, i, 9 + j) = dt * M3(e->dv_db_g, i, j) + 0.25 * dt * dt * (M3(P1, i, j) + M3(P2, i, j));
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
        F15(F, i, 12 + j) = (-M3(e->C_integral, i, j)) * dt + M3(M25, i, j) * dt * dt;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 3 + i, 9 + j) = (-dt) * M3(C_1, i, j);
    ok_m3_mulv(M05, acc_true, tmp3);
    for (k = 0; k < 3; ++k) vv[k] = tmp3[k] * dt;
    cross_mx(vv, neg);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 6 + i, 3 + j) = -M3(neg, i, j);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i)
        F15(F, 6 + i, 9 + j) = 0.5 * dt * (M3(P1, i, j) + M3(P2, i, j));
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F, 6 + i, 12 + j) = (-0.5 * M3(Csum, i, j)) * dt;

    /* Q = K * sigma_sq */
    memset(K, 0, sizeof K);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) {
        const double id = (i == j ? 1.0 : 0.0);
        F15(K[0], 3 + i, 3 + j) = gyr_sat_mult * dt * id;
        F15(K[1], i, j) = 0.5 * dt * dt * dt * acc_sat_mult * acc_sat_mult * acc_sat_mult * id;
        F15(K[1], 6 + i, 6 + j) = acc_sat_mult * dt * id;
        F15(K[2], 9 + i, 9 + j) = dt * id;
        F15(K[3], 12 + i, 12 + j) = dt * id;
    }
    for (jj = 0; jj < 4; ++jj) {
        gemm_pqpt(F, e->dPdsigma[jj], Pn);
        for (k = 0; k < 225; ++k) e->dPdsigma[jj][k] = Pn[k] + K[jj][k];
    }

    /* memory shift */
    e->delta_q = Delta_q_1;
    memcpy(e->C_integral, C_integral_1, sizeof C_integral_1);
    memcpy(e->acc_integral, acc_integral_1, sizeof acc_integral_1);
    memcpy(e->cross, cross_1, sizeof cross_1);
    memcpy(e->dv_db_g, dv_db_g_1, sizeof dv_db_g_1);
}

/* symmetrise dPdsigma, P_delta = sum dPdsigma_j*sigma_j^2, sqrt information, information */
static void preint_finish(ok_imu_error* e) {
    int jj, i, j, k;
    double At[225], A[225], rt[225];
    const double* sg[4] = {&e->params.sigma_g_c, &e->params.sigma_a_c, &e->params.sigma_gw_c, &e->params.sigma_aw_c};
    for (jj = 0; jj < 4; ++jj) {
        for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) At[i + 15 * j] = e->dPdsigma[jj][j + 15 * i];
        for (k = 0; k < 225; ++k) e->dPdsigma[jj][k] = 0.5 * e->dPdsigma[jj][k] + 0.5 * At[k];
    }
    for (k = 0; k < 225; ++k) e->P_delta[k] = e->dPdsigma[0][k] * (*sg[0]) * (*sg[0]);
    for (jj = 1; jj < 4; ++jj)
        for (k = 0; k < 225; ++k) e->P_delta[k] += e->dPdsigma[jj][k] * (*sg[jj]) * (*sg[jj]);
    symm_sqrt_u(e->P_delta, e->sqrt_information);
    /* information = sqrt^T * sqrt (lhs transposed GEMM, plain problem) */
    for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) rt[i + 15 * j] = e->sqrt_information[j + 15 * i];
    ok_gemm(15, 15, 15, rt, e->sqrt_information, A);
    memcpy(e->information, A, sizeof A);
}

/* common saturation check; returns multipliers */
static void sat_mults(const ok_imu_params* p, const double w0[3], const double a0[3], const double w1[3],
                      const double a1[3], double* gm, double* am) {
    *gm = 1.0;
    *am = 1.0;
    if (fabs(w0[0]) > p->g_max || fabs(w0[1]) > p->g_max || fabs(w0[2]) > p->g_max || fabs(w1[0]) > p->g_max ||
        fabs(w1[1]) > p->g_max || fabs(w1[2]) > p->g_max)
        *gm *= 100;
    if (fabs(a0[0]) > p->a_max || fabs(a0[1]) > p->a_max || fabs(a0[2]) > p->a_max || fabs(a1[0]) > p->a_max ||
        fabs(a1[1]) > p->a_max || fabs(a1[2]) > p->a_max)
        *am *= 100;
}

int ok_imu_redo_preintegration(ok_imu_error* e, const double sb[9]) {
    ok_time time = e->t0;
    const ok_time end = e->t1;
    int hasStarted = 0, steps = 0, k, jj;
    size_t it;
    const size_t n = e->n_meas;

    if (n == 0 || ok_time_lt(e->meas[n - 1].t, end)) return -1;
    memset(&e->delta_q, 0, sizeof e->delta_q);
    e->delta_q.w = 1.0;
    m3_zero(e->C_integral); m3_zero(e->C_doubleintegral);
    for (k = 0; k < 3; ++k) { e->acc_integral[k] = 0.0; e->acc_doubleintegral[k] = 0.0; }
    m3_zero(e->cross); m3_zero(e->dalpha_db_g); m3_zero(e->dv_db_g); m3_zero(e->dp_db_g);
    memset(e->P_delta, 0, sizeof e->P_delta);
    for (jj = 0; jj < 4; ++jj) memset(e->dPdsigma[jj], 0, sizeof e->dPdsigma[jj]);

    for (it = 0; it < n; ++it) {
        double omega_S_0[3], acc_S_0[3], omega_S_1[3], acc_S_1[3], gm, am;
        ok_time nexttime;
        double dt;
        for (k = 0; k < 3; ++k) { omega_S_0[k] = e->meas[it].gyr[k]; acc_S_0[k] = e->meas[it].acc[k]; }
        if (it + 1 < n) {
            for (k = 0; k < 3; ++k) { omega_S_1[k] = e->meas[it + 1].gyr[k]; acc_S_1[k] = e->meas[it + 1].acc[k]; }
        } else {
            for (k = 0; k < 3; ++k) { omega_S_1[k] = 0.0; acc_S_1[k] = 0.0; }
        }
        if (it + 1 == n) nexttime = e->t1; else nexttime = e->meas[it + 1].t;
        dt = ok_time_diff_sec(nexttime, time);
        if (ok_time_lt(end, nexttime)) {
            const double interval = ok_time_diff_sec(nexttime, e->meas[it].t);
            double r;
            nexttime = e->t1;
            dt = ok_time_diff_sec(nexttime, time);
            r = dt / interval;
            for (k = 0; k < 3; ++k) {
                omega_S_1[k] = (1.0 - r) * omega_S_0[k] + r * omega_S_1[k];
                acc_S_1[k] = (1.0 - r) * acc_S_0[k] + r * acc_S_1[k];
            }
        }
        if (dt <= 0.0) continue;
        if (!hasStarted) {
            const double r = dt / ok_time_diff_sec(nexttime, e->meas[it].t);
            hasStarted = 1;
            for (k = 0; k < 3; ++k) {
                omega_S_0[k] = r * omega_S_0[k] + (1.0 - r) * omega_S_1[k];
                acc_S_0[k] = r * acc_S_0[k] + (1.0 - r) * acc_S_1[k];
            }
        }
        sat_mults(&e->params, omega_S_0, acc_S_0, omega_S_1, acc_S_1, &gm, &am);
        preint_step(e, sb, omega_S_0, acc_S_0, omega_S_1, acc_S_1, dt, gm, am);
        time = nexttime;
        ++steps;
        if (ok_time_eq(nexttime, e->t1)) break;
    }
    memcpy(e->sb_ref, sb, 9 * sizeof(double));
    preint_finish(e);
    return steps;
}

int ok_imu_append(ok_imu_error* e, const double sb[9], const ok_imu_meas* meas, size_t n, ok_time t_1) {
    ok_time time = e->t1;
    const ok_time end = t_1;
    int hasStarted = 0, steps = 0, k;
    size_t it, first = 0;

    /* merge IMU deque: skip measurements not newer than our last one, append the rest */
    while (first < n && !ok_time_lt(e->meas[e->n_meas - 1].t, meas[first].t)) ++first;
    if (e->n_meas + (n - first) > e->cap_meas) {
        e->cap_meas = (e->n_meas + (n - first)) * 2;
        e->meas = (ok_imu_meas*)realloc(e->meas, e->cap_meas * sizeof(ok_imu_meas));
    }
    memcpy(e->meas + e->n_meas, meas + first, (n - first) * sizeof(ok_imu_meas));
    e->n_meas += n - first;
    e->t1 = t_1;
    if (n == 0 || ok_time_lt(meas[n - 1].t, end)) return -1;

    for (it = 0; it < n; ++it) {
        double omega_S_0[3], acc_S_0[3], omega_S_1[3], acc_S_1[3], gm, am;
        ok_time nexttime;
        double dt;
        for (k = 0; k < 3; ++k) { omega_S_0[k] = meas[it].gyr[k]; acc_S_0[k] = meas[it].acc[k]; }
        if (it + 1 < n) {
            for (k = 0; k < 3; ++k) { omega_S_1[k] = meas[it + 1].gyr[k]; acc_S_1[k] = meas[it + 1].acc[k]; }
        } else {
            for (k = 0; k < 3; ++k) { omega_S_1[k] = 0.0; acc_S_1[k] = 0.0; }
        }
        nexttime = (it + 1 < n) ? meas[it + 1].t : e->t1; /* (it+1)==imuMeasurements_.end() is never true in OKVIS2 */
        dt = ok_time_diff_sec(nexttime, time);
        if (ok_time_lt(end, nexttime)) {
            const double interval = ok_time_diff_sec(nexttime, meas[it].t);
            double r;
            nexttime = e->t1;
            dt = ok_time_diff_sec(nexttime, time);
            r = dt / interval;
            for (k = 0; k < 3; ++k) {
                omega_S_1[k] = (1.0 - r) * omega_S_0[k] + r * omega_S_1[k];
                acc_S_1[k] = (1.0 - r) * acc_S_0[k] + r * acc_S_1[k];
            }
        }
        if (dt <= 0.0) continue;
        if (!hasStarted) {
            const double r = dt / ok_time_diff_sec(nexttime, meas[it].t);
            hasStarted = 1;
            for (k = 0; k < 3; ++k) {
                omega_S_0[k] = r * omega_S_0[k] + (1.0 - r) * omega_S_1[k];
                acc_S_0[k] = r * acc_S_0[k] + (1.0 - r) * acc_S_1[k];
            }
        }
        sat_mults(&e->params, omega_S_0, acc_S_0, omega_S_1, acc_S_1, &gm, &am);
        preint_step(e, sb, omega_S_0, acc_S_0, omega_S_1, acc_S_1, dt, gm, am);
        time = nexttime;
        ++steps;
        if (ok_time_eq(nexttime, e->t1)) break;
    }
    preint_finish(e);
    return steps;
}

/* ------------------------------------------------------------------------------------------------ */
/* ImuError::Evaluate (EvaluateWithMinimalJacobians without minimal Jacobians)                       */
/* ------------------------------------------------------------------------------------------------ */

#define M4(a, i, j) (a)[(i) + 4 * (j)]

/* kinematics::plus / oplus (operators.hpp); q = [x,y,z,w] */
static void quat_plus(const ok_quat* q_, double Q[16]) {
    const double q[4] = {q_->x, q_->y, q_->z, q_->w};
    M4(Q, 0, 0) = q[3];  M4(Q, 0, 1) = -q[2]; M4(Q, 0, 2) = q[1];  M4(Q, 0, 3) = q[0];
    M4(Q, 1, 0) = q[2];  M4(Q, 1, 1) = q[3];  M4(Q, 1, 2) = -q[0]; M4(Q, 1, 3) = q[1];
    M4(Q, 2, 0) = -q[1]; M4(Q, 2, 1) = q[0];  M4(Q, 2, 2) = q[3];  M4(Q, 2, 3) = q[2];
    M4(Q, 3, 0) = -q[0]; M4(Q, 3, 1) = -q[1]; M4(Q, 3, 2) = -q[2]; M4(Q, 3, 3) = q[3];
}
static void quat_oplus(const ok_quat* q_, double Q[16]) {
    const double q[4] = {q_->x, q_->y, q_->z, q_->w};
    M4(Q, 0, 0) = q[3];  M4(Q, 0, 1) = q[2];  M4(Q, 0, 2) = -q[1]; M4(Q, 0, 3) = q[0];
    M4(Q, 1, 0) = -q[2]; M4(Q, 1, 1) = q[3];  M4(Q, 1, 2) = q[0];  M4(Q, 1, 3) = q[1];
    M4(Q, 2, 0) = q[1];  M4(Q, 2, 1) = -q[0]; M4(Q, 2, 2) = q[3];  M4(Q, 2, 3) = q[2];
    M4(Q, 3, 0) = -q[0]; M4(Q, 3, 1) = -q[1]; M4(Q, 3, 2) = -q[2]; M4(Q, 3, 3) = q[3];
}
/* Matrix4d * Matrix4d: every row left-associative (stella_port/HANDOVER.md) */
static void m4_mul(const double a[16], const double b[16], double out[16]) {
    double res[16];
    int i, j;
    for (j = 0; j < 4; ++j)
        for (i = 0; i < 4; ++i) {
            double s = M4(a, i, 0) * M4(b, 0, j);
            s = s + M4(a, i, 1) * M4(b, 1, j);
            s = s + M4(a, i, 2) * M4(b, 2, j);
            s = s + M4(a, i, 3) * M4(b, 3, j);
            res[i + 4 * j] = s;
        }
    memcpy(out, res, sizeof res);
}
/* kinematics::deltaQ */
static ok_quat delta_q(const double dAlpha[3]) {
    const double halfnorm = 0.5 * ok_v3_norm(dAlpha);
    const double s = sinc_(halfnorm) * 0.5;
    ok_quat q;
    q.x = s * dAlpha[0]; q.y = s * dAlpha[1]; q.z = s * dAlpha[2];
    q.w = cos(halfnorm);
    return q;
}
/* 3x6 block times 6-vector (Block<3,6> * Vector6): rows 0-1 left fold, row 2 pairwise (a0+(a1+a2))+(a3+(a4+a5)) */
static void m3x6_mulv6(const double* a, int lda, const double v[6], double out[3]) {
    int i;
    double res[3];
    for (i = 0; i < 2; ++i) {
        double s = a[i + lda * 0] * v[0];
        s = s + a[i + lda * 1] * v[1];
        s = s + a[i + lda * 2] * v[2];
        s = s + a[i + lda * 3] * v[3];
        s = s + a[i + lda * 4] * v[4];
        s = s + a[i + lda * 5] * v[5];
        res[i] = s;
    }
    res[2] = (a[2 + lda * 0] * v[0] + (a[2 + lda * 1] * v[1] + a[2 + lda * 2] * v[2])) +
             (a[2 + lda * 3] * v[3] + (a[2 + lda * 4] * v[4] + a[2 + lda * 5] * v[5]));
    memcpy(out, res, sizeof res);
}
/* PoseManifold::minusJacobian -> J_lift (6x7 row-major) */
static void minus_jacobian(const double x[7], double J[42]) {
    const ok_quat q_inv = {-x[3], -x[4], -x[5], x[6]};
    double Qp[16], Jp[12], t[12];
    int i, j, k;
#define JL(r, c) J[(r) * 7 + (c)]
    for (k = 0; k < 42; ++k) J[k] = 0.0;
    JL(0, 0) = JL(1, 1) = JL(2, 2) = 1.0;
    quat_oplus(&q_inv, Qp);
    /* Jq_pinv (3x4): [2*I3 | 0] ; Identity * 2.0 */
    for (k = 0; k < 12; ++k) Jp[k] = 0.0;
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) Jp[i + 3 * j] = (i == j ? 1.0 : 0.0) * 2.0;
    /* Jq_pinv * Qplus (3x4 * 4x4): rows 0-1 left fold, row 2 pairwise (a0+a1)+(a2+a3) */
    for (j = 0; j < 4; ++j) {
        for (i = 0; i < 2; ++i) {
            double s = Jp[i + 3 * 0] * M4(Qp, 0, j);
            s = s + Jp[i + 3 * 1] * M4(Qp, 1, j);
            s = s + Jp[i + 3 * 2] * M4(Qp, 2, j);
            s = s + Jp[i + 3 * 3] * M4(Qp, 3, j);
            t[i + 3 * j] = s;
        }
        t[2 + 3 * j] = (Jp[2 + 3 * 0] * M4(Qp, 0, j) + Jp[2 + 3 * 1] * M4(Qp, 1, j)) +
                       (Jp[2 + 3 * 2] * M4(Qp, 2, j) + Jp[2 + 3 * 3] * M4(Qp, 3, j));
    }
    for (j = 0; j < 4; ++j) for (i = 0; i < 3; ++i) JL(3 + i, 3 + j) = t[i + 3 * j];
#undef JL
}

/* alignment parity of the stack temp holding J0 / J2 (840-byte Matrix<double,15,7> has no static alignment,
 * so Eigen peels by the runtime address); measured against the reference binary, see HANDOVER.md. */
int g_eval_temp_parity[2] = {0, 0};

int ok_imu_evaluate(ok_imu_error* e, const double* const params[4], double residuals[15], double* const jacobians[4]) {
    const double *p0 = params[0], *sb0p = params[1], *p1 = params[2], *sb1p = params[3];
    ok_quat q0 = {p0[3], p0[4], p0[5], p0[6]}, q1 = {p1[3], p1[4], p1[5], p1[6]};
    double sb0[9], sb1[9], C_WS_0[9], C_S0_W[9], Delta_t, Delta_b[6];
    int success = 1, i, j, k;
    double g_W[3];
    double F0[225], F1[225];
    double dp_est[3], dv_est[3];
    ok_quat Dq, q1inv, qa, qb;
    double mdb[3], dqin[3];
    double error[15];

    /* Quaterniond(...).normalized() passed to Transformation(r, q), whose ctor normalises again */
    q0 = ok_quat_normalized(ok_quat_normalized(q0));
    q1 = ok_quat_normalized(ok_quat_normalized(q1));
    memcpy(sb0, sb0p, sizeof sb0);
    memcpy(sb1, sb1p, sizeof sb1);
    ok_quat_to_mat3(&q0, C_WS_0);
    for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) M3(C_S0_W, i, j) = M3(C_WS_0, j, i);

    Delta_t = ok_time_diff_sec(e->t1, e->t0);
    for (k = 0; k < 6; ++k) Delta_b[k] = sb0[3 + k] - e->sb_ref[3 + k];
    if (!e->redo) e->redo = ok_v3_norm(Delta_b) > 0.0003;
    if ((e->redo && e->n_meas < 50) || e->redo_counter == 0) {
        const int steps = ok_imu_redo_preintegration(e, sb0);
        e->redo_counter++;
        for (k = 0; k < 6; ++k) Delta_b[k] = 0.0;
        e->redo = 0;
        if (steps == 0) success = 0;
    }

    g_W[0] = e->params.g * 0.0; g_W[1] = e->params.g * 0.0; g_W[2] = e->params.g * 1.0;
    for (k = 0; k < 15 * 15; ++k) { F0[k] = 0.0; F1[k] = -0.0; }
    for (k = 0; k < 15; ++k) { F0[k + 15 * k] = 1.0; F1[k + 15 * k] = -1.0; }

    for (k = 0; k < 3; ++k) dp_est[k] = p0[k] - p1[k] + sb0[k] * Delta_t - 0.5 * g_W[k] * Delta_t * Delta_t;
    for (k = 0; k < 3; ++k) dv_est[k] = sb0[k] - sb1[k] - g_W[k] * Delta_t;
    /* Dq = deltaQ(-dalpha_db_g_*Delta_b.head<3>()) * Delta_q_ */
    ok_m3_mulv(e->dalpha_db_g, Delta_b, mdb);
    for (k = 0; k < 3; ++k) dqin[k] = -mdb[k];
    qa = delta_q(dqin);
    ok_quat_mul(&qa, &e->delta_q, &Dq);
    q1inv = ok_quat_inverse(q1);

    /* F0 */
    {
        double blk[9], t[9], a4[16], b4[16], p4[16], I3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, nd[9];
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, i, j) = M3(C_S0_W, i, j);
        cross_mx(dp_est, blk);
        ok_m3_mul(C_S0_W, blk, t);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, i, 3 + j) = M3(t, i, j);
        ok_m3_mul(C_S0_W, I3, t);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, i, 6 + j) = M3(t, i, j) * Delta_t;
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, i, 9 + j) = M3(e->dp_db_g, i, j);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, i, 12 + j) = -M3(e->C_doubleintegral, i, j);
        ok_quat_mul(&Dq, &q1inv, &qa);
        quat_plus(&qa, a4);
        quat_oplus(&q0, b4);
        m4_mul(a4, b4, p4);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, 3 + i, 3 + j) = M4(p4, i, j);
        ok_quat_mul(&q1inv, &q0, &qb);
        quat_oplus(&qb, a4);
        quat_oplus(&Dq, b4);
        m4_mul(a4, b4, p4);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) { blk[i + 3 * j] = M4(p4, i, j); nd[i + 3 * j] = -M3(e->dalpha_db_g, i, j); }
        ok_m3_mul(blk, nd, t);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, 3 + i, 9 + j) = M3(t, i, j);
        cross_mx(dv_est, blk);
        ok_m3_mul(C_S0_W, blk, t);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, 6 + i, 3 + j) = M3(t, i, j);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, 6 + i, 6 + j) = M3(C_S0_W, i, j);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, 6 + i, 9 + j) = M3(e->dv_db_g, i, j);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F0, 6 + i, 12 + j) = -M3(e->C_integral, i, j);

        /* F1 */
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F1, i, j) = -M3(C_S0_W, i, j);
        quat_plus(&Dq, a4);
        quat_oplus(&q0, b4);
        m4_mul(a4, b4, p4);
        quat_plus(&q1inv, a4);
        m4_mul(p4, a4, b4);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F1, 3 + i, 3 + j) = -M4(b4, i, j);
        for (j = 0; j < 3; ++j) for (i = 0; i < 3; ++i) F15(F1, 6 + i, 6 + j) = -M3(C_S0_W, i, j);
    }

    /* the overall error vector */
    {
        double a[3], b[3], qq[4];
        ok_quat qm, qr;
        ok_m3_mulv(C_S0_W, dp_est, a);
        m3x6_mulv6(&F15(F0, 0, 9), 15, Delta_b, b);
        for (k = 0; k < 3; ++k) error[k] = a[k] + e->acc_doubleintegral[k] + b[k];
        ok_quat_mul(&q1inv, &q0, &qm);
        ok_quat_mul(&Dq, &qm, &qr);
        qq[0] = qr.x; qq[1] = qr.y; qq[2] = qr.z;
        for (k = 0; k < 3; ++k) error[3 + k] = 2 * qq[k];
        ok_m3_mulv(C_S0_W, dv_est, a);
        m3x6_mulv6(&F15(F0, 6, 9), 15, Delta_b, b);
        for (k = 0; k < 3; ++k) error[6 + k] = a[k] + e->acc_integral[k] + b[k];
        for (k = 0; k < 6; ++k) error[9 + k] = sb0[3 + k] - sb1[3 + k];
        if (!success) for (k = 0; k < 15; ++k) error[k] = 0.0;
    }

    /* error weighting: GEMV, column-major kernel => one left fold per row */
    for (i = 0; i < 15; ++i) {
        double acc = 0.0;
        for (j = 0; j < 15; ++j) acc = acc + F15(e->sqrt_information, i, j) * error[j];
        residuals[i] = 0.0 + 1.0 * acc;
    }

    if (jacobians != NULL) {
        double Jmin[90], Jb[135], Jl[42];
        int blk;
        const double* sqrtI = e->sqrt_information;
        for (blk = 0; blk < 4; ++blk) {
            double* J = jacobians[blk];
            const double* F = (blk < 2) ? F0 : F1;
            if (J == NULL) continue;
            if (blk == 0 || blk == 2) {
                /* J_minimal (15x6) = sqrtI * F.block<15,6>(0,0);  J (15x7, row-major) = J_minimal * J_lift */
                ok_gemm(15, 6, 15, sqrtI, F, Jmin);
                minus_jacobian(params[blk], Jl);
                /* J = Jm * J_lift evaluates (aliasing) into a column-major 15x7 temp with SliceVectorized
                 * traversal: per column the packet (2 rows) start alternates (15 is odd), so one end row of each
                 * column is a scalar coefficient: pairwise (a0+(a1+a2))+(a3+(a4+a5)); packet rows are left folds. */
                for (j = 0; j < 7; ++j)
                    for (i = 0; i < 15; ++i) {
                        const int col_misaligned = ((j + g_eval_temp_parity[blk >> 1]) & 1);
                        const int scalar_row = col_misaligned ? (i == 0) : (i == 14);
                        double s;
                        if (scalar_row) {
                            double a[6];
                            for (k = 0; k < 6; ++k) a[k] = Jmin[i + 15 * k] * Jl[k * 7 + j];
                            s = (a[0] + (a[1] + a[2])) + (a[3] + (a[4] + a[5]));
                        } else {
                            s = Jmin[i + 15 * 0] * Jl[0 * 7 + j];
                            for (k = 1; k < 6; ++k) s = s + Jmin[i + 15 * k] * Jl[k * 7 + j];
                        }
                        J[i * 7 + j] = s;
                    }
            } else {
                /* J (15x9, row-major) = sqrtI * F.block<15,9>(0,6) */
                ok_gemm(15, 9, 15, sqrtI, F + 15 * 6, Jb);
                for (j = 0; j < 9; ++j) for (i = 0; i < 15; ++i) J[i * 9 + j] = Jb[i + 15 * j];
            }
        }
    }
    return 1;
}

/* ------------------------------------------------------------------------------------------------ */
/* ImuError::initPose                                                                                */
/* ------------------------------------------------------------------------------------------------ */

int ok_imu_init_pose(const ok_imu_meas* meas, size_t n, double T_WS[7]) {
    double acc_B[3] = {0.0, 0.0, 0.0}, e_acc[3], cr[3], cr_n[3], inc[3], dalpha[3], halfnorm, angle, dot;
    const double ez_W[3] = {0.0, 0.0, 1.0};
    ok_quat q = {0.0, 0.0, 0.0, 1.0}, dq, qn;
    size_t it;
    int k;
    for (k = 0; k < 7; ++k) T_WS[k] = 0.0;
    T_WS[6] = 1.0;
    if (n == 0) return 0;
    for (it = 0; it < n; ++it)
        for (k = 0; k < 3; ++k) acc_B[k] += meas[it].acc[k];
    for (k = 0; k < 3; ++k) acc_B[k] /= (double)n;
    ok_v3_normalized(acc_B, e_acc);
    if (!((acc_B[0] * acc_B[0] + acc_B[1] * acc_B[1]) + acc_B[2] * acc_B[2] > 0.0)) memcpy(e_acc, acc_B, sizeof e_acc);
    /* poseIncrement.tail<3>() = ez_W.cross(e_acc).normalized() */
    cr[0] = ez_W[1] * e_acc[2] - ez_W[2] * e_acc[1];
    cr[1] = ez_W[2] * e_acc[0] - ez_W[0] * e_acc[2];
    cr[2] = ez_W[0] * e_acc[1] - ez_W[1] * e_acc[0];
    memcpy(cr_n, cr, sizeof cr);
    if ((cr[0] * cr[0] + cr[1] * cr[1]) + cr[2] * cr[2] > 0.0) ok_v3_normalized(cr, cr_n);
    /* angle = acos(ez_W^T * e_acc) (inner product: left fold) */
    dot = (ez_W[0] * e_acc[0] + ez_W[1] * e_acc[1]) + ez_W[2] * e_acc[2];
    angle = acos(dot);
    for (k = 0; k < 3; ++k) inc[k] = cr_n[k] * angle;
    /* T_WS.oplus(-poseIncrement): r += -0, dq = deltaQ-like, q = normalize(dq * q) */
    for (k = 0; k < 3; ++k) T_WS[k] = 0.0 + (-0.0);
    for (k = 0; k < 3; ++k) dalpha[k] = -inc[k];
    halfnorm = 0.5 * ok_v3_norm(dalpha);
    {
        const double s = sinc_(halfnorm);
        dq.x = s * 0.5 * dalpha[0]; dq.y = s * 0.5 * dalpha[1]; dq.z = s * 0.5 * dalpha[2];
        dq.w = cos(halfnorm);
    }
    ok_quat_mul(&dq, &q, &qn);
    ok_quat_normalize(&qn);
    T_WS[3] = qn.x; T_WS[4] = qn.y; T_WS[5] = qn.z; T_WS[6] = qn.w;
    return 1;
}
