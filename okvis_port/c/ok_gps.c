/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2-X GNSS error terms + PoseManifold4d. See ok_gps.h for provenance and notices.
 * "Math on paper": one equation per line, in the evaluation order of the reference build (Eigen 3.4.0, SSE2, no FMA).
 *
 * Eigen small-product models (shared with ok_err.c, all measured against real Eigen by okvis_gps_test.cc):
 *   - fixed-size lazy products (all dimensions <= 8) are evaluated with inner-unrolled slice vectorisation: rows [0, 2*(R/2))
 *     are packets = left fold of the D products, an odd last row is a coefficient = halving redux tree; independent of the
 *     alignment of the temporary (no runtime peeling when the inner loop is unrolled)  => ok_lazy_a
 *   - Matrix3d * Matrix3d / Vector3d: ok_m3_mul / ok_m3_mulv (the same rule); (-A) * B == -(A * B) bit for bit
 *   - J = sqrtI * J_minimal * J_lift assigned into a row-major Map: the inner product (sqrtI * J_minimal) is a column-major
 *     3x6 temporary evaluated like the rule above (ok_lazy_a; also the minimal Jacobian); the outer product has a column-major
 *     lhs and a row-major destination, so the storage orders disagree and every coefficient is scalar: the redux tree
 *     (p0 + (p1 + p2)) + (p3 + (p4 + p5)) in ALL rows (ok_lazy_tree)  [new Eigen fact, found by this oracle]
 *   - Matrix<15,15> P = T * Pd * T^T (assignment): gemm_pqpt below (ok_imu.c's model of the same statement) */
#include "ok_gps.h"

#include <math.h>
#include <string.h>

#include "ok_err.h"
#include "ok_gps_init.h"

#define M3(a, i, j) ((a)[(i) + 3 * (j)])
#define F15(a, i, j) ((a)[(i) + 15 * (j)])

#ifndef OK_GPS_MUTATE
#define OK_GPS_MUTATE 0 /* sensitivity builds only: a deliberately wrong evaluation order, see okvis_gps_test.cc */
#endif

#if OK_GPS_MUTATE == 2
#define LIFT_PRODUCT ok_lazy_a /* wrong: packets for rows 0-1 */
#else
#define LIFT_PRODUCT ok_lazy_tree
#endif

static void transpose(int R, int C, const double* A, double* out) { /* A R x C col-major -> out C x R col-major */
    int i, j;
    for (j = 0; j < C; ++j)
        for (i = 0; i < R; ++i) out[j + C * i] = A[i + R * j];
}

/* ------------------------------------------------ shared pieces ------------------------------------------------ */
static void pose_rotation(const double* p, double C[9]) { /* Quaterniond(p[6], p[3], p[4], p[5]).toRotationMatrix() */
    const ok_quat q = {p[3], p[4], p[5], p[6]};
    ok_quat_to_mat3(&q, C);
}

/* J0_minimal (3x6): [-C_GW | C_GW * crossMx(arm)], arm = C_S * r_SA at the pose the antenna is evaluated at */
static void jac_pose_minimal(const double C_GW[9], const double arm[3], double J[18]) {
    double X[9], R[9];
    int k;
    ok_kin_cross_mx(arm, X);
    ok_m3_mul(C_GW, X, R);
    for (k = 0; k < 9; ++k) { J[k] = -C_GW[k]; J[9 + k] = R[k]; }
}
/* J_minimal of T_GW (3x6): [-I | crossMx(v)], v = C_GW * (r + C_S * r_SA); -Identity has -0.0 off the diagonal */
static void jac_world_minimal(const double v[3], double J[18]) {
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) J[i + 3 * j] = -(i == j ? 1.0 : 0.0);
    ok_kin_cross_mx(v, J + 9);
}
/* jac = (sqrtI * Jm) * J_lift(pose) as the row-major 3 x 7; jacmin = sqrtI * Jm as the row-major 3 x 6 (may be NULL) */
static void weight_lift(const double S[9], const double Jm[18], const double pose[7], double* jac, double* jacmin) {
    double W[18], Lrm[42], L[42], F[21];
    ok_lazy_a(3, 6, 3, S, Jm, W);
    ok_pose_minus_jacobian(pose, Lrm); /* 6x7 row-major */
    transpose(7, 6, Lrm, L);           /* = the same matrix, column-major */
    LIFT_PRODUCT(3, 7, 6, W, L, F);
    transpose(3, 7, F, jac);
    if (jacmin) transpose(3, 6, W, jacmin);
}
static void inverse3(const double a[9], double b[9]) { ok_gps_inverse3(a, b); } /* Matrix3d::inverse() */
/* LLT<Matrix3d>(info).matrixL().transpose() */
static void sqrt_information(const double info[9], double out[9]) { ok_llt_sqrt_information(3, info, out); }

/* --------------------------------------------- GpsErrorSynchronous --------------------------------------------- */
void ok_gps_sync_set_information(ok_gps_sync* e, const double info[9]) {
    memcpy(e->info, info, 9 * sizeof(double));
    inverse3(info, e->covariance);
    sqrt_information(info, e->sqrt_info);
}
void ok_gps_sync_init(ok_gps_sync* e, const double meas[3], const double info[9], const double lever[3]) {
    memset(e, 0, sizeof *e);
    memcpy(e->meas, meas, 3 * sizeof(double));
    memcpy(e->lever, lever, 3 * sizeof(double));
    ok_gps_sync_set_information(e, info);
}

int ok_gps_sync_evaluate(const ok_gps_sync* e, const double* const p[2], double res[3], double* const* jac,
                         double* const* jacmin) {
    double C_WS[9], C_GW[9], arm[3], v[3], gv[3], err[3], Jm[18];
    int k;
    pose_rotation(p[0], C_WS);
    pose_rotation(p[1], C_GW);
    ok_m3_mulv(C_WS, e->lever, arm);                           /* C_WS * r_SA */
    for (k = 0; k < 3; ++k) v[k] = p[0][k] + arm[k];           /* t_WS_W + C_WS * r_SA */
    ok_m3_mulv(C_GW, v, gv);                                   /* C_GW * (...) */
    for (k = 0; k < 3; ++k) err[k] = e->meas[k] - (gv[k] + p[1][k]);
    ok_m3_mulv(e->sqrt_info, err, res);
#if OK_GPS_MUTATE == 1
    res[0] = e->sqrt_info[0] * err[0] + (e->sqrt_info[3] * err[1] + e->sqrt_info[6] * err[2]); /* row 0 as a tree: wrong */
#endif
    if (jac != NULL) {
        if (jac[0] != NULL) {
            jac_pose_minimal(C_GW, arm, Jm);
            weight_lift(e->sqrt_info, Jm, p[0], jac[0], jacmin ? jacmin[0] : NULL);
        }
        if (jac[1] != NULL) {
            jac_world_minimal(gv, Jm);
            weight_lift(e->sqrt_info, Jm, p[1], jac[1], jacmin ? jacmin[1] : NULL);
        }
    }
    return 1;
}

/* ------------------------------------------- GpsErrorAsynchronous ------------------------------------------- */
/* P = T * Pd * T^T  (assignment form, 15x15): ok_imu.c models `F * dPdsigma * F^T` as GEBP of the inner product and
 * of the transposed outer product */
static void gemm_pqpt(const double P[225], const double Q[225], double out[225]) {
    double t[225], tT[225], r[225];
    int i, j;
    ok_gemm(15, 15, 15, P, Q, t);
    for (j = 0; j < 15; ++j)
        for (i = 0; i < 15; ++i) tT[j + 15 * i] = t[i + 15 * j];
#if OK_GPS_MUTATE == 3 /* wrong: a plain left fold per entry instead of the two GEBP products */
    { int k; for (j = 0; j < 15; ++j) for (i = 0; i < 15; ++i) { double s = t[i] * P[j]; for (k = 1; k < 15; ++k) s = s + t[i + 15 * k] * P[j + 15 * k]; out[i + 15 * j] = s; } (void)tT; (void)r; return; }
#endif
    ok_gemm(15, 15, 15, P, tT, r);
    for (j = 0; j < 15; ++j)
        for (i = 0; i < 15; ++i) out[i + 15 * j] = r[j + 15 * i];
}

void ok_gps_async_set_information(ok_gps_async* e, const double info[9]) {
    memcpy(e->info, info, 9 * sizeof(double));
    inverse3(info, e->covariance);
}
void ok_gps_async_init(ok_gps_async* e, const double meas[3], const double info[9], const double lever[3],
                       const ok_imu_meas* imu, size_t n, const ok_imu_params* p, ok_time tk, ok_time tg) {
    memset(e, 0, sizeof *e);
    memcpy(e->meas, meas, 3 * sizeof(double));
    memcpy(e->lever, lever, 3 * sizeof(double));
    ok_gps_async_set_information(e, info);
    ok_imu_error_init(&e->imu, imu, n, p, tk, tg); /* Delta_q_ = identity, redo_ = true, redoCounter_ = 0, sb_ref = 0 */
    e->use_imu_covariance = 1;
    e->redo_always = 0;
}
void ok_gps_async_init_sigma(ok_gps_async* e, const double meas[3], const double sigma[3], const double lever[3],
                             const ok_imu_meas* imu, size_t n, const ok_imu_params* p, ok_time tk, ok_time tg) {
    double info[9];
    int k;
    for (k = 0; k < 9; ++k) info[k] = 0.0; /* Matrix3d::Identity() with the diagonal overwritten */
    info[0] = 1 / (sigma[0] * sigma[0]);
    info[4] = 1 / (sigma[1] * sigma[1]);
    info[8] = 1 / (sigma[2] * sigma[2]);
    ok_gps_async_init(e, meas, info, lever, imu, n, p, tk, tg);
}
void ok_gps_async_free(ok_gps_async* e) { ok_imu_error_free(&e->imu); }

/* T_WS_tg = (r + v*dt - 0.5*g*dt*dt + C * (acc_dd + dp_db_g*db_g - C_dd*db_a), q * Delta_q * deltaQ(-dalpha_db_g*db_g)) */
static void propagate(const ok_imu_error* u, const double r[3], const double C[9], const ok_quat* q, const double sb[9],
                      const double db[6], double dt, ok_tf* T_tg) {
    double g_W[3], v3[3], a[3], b[3], inner[3], pos[3], negD[9], phi[3], rr[3];
    ok_quat dq, qa, qb;
    int k;
    v3[0] = 0; v3[1] = 0; v3[2] = 6371009;
    ok_v3_normalized(v3, a);
    for (k = 0; k < 3; ++k) g_W[k] = u->params.g * a[k];
    ok_m3_mulv(u->dp_db_g, db, a);          /* dp_db_g_ * Delta_b.head<3>() */
    ok_m3_mulv(u->C_doubleintegral, db + 3, b); /* C_doubleintegral_ * Delta_b.tail<3>() */
    for (k = 0; k < 3; ++k) inner[k] = (u->acc_doubleintegral[k] + a[k]) - b[k];
    ok_m3_mulv(C, inner, a);                /* C_WS_tk * (...) */
    for (k = 0; k < 3; ++k) {
#if OK_GPS_MUTATE == 5
        pos[k] = ((r[k] + sb[k] * dt) + a[k]) - 0.5 * g_W[k] * dt * dt;
#else
        pos[k] = ((r[k] + sb[k] * dt) - 0.5 * g_W[k] * dt * dt) + a[k];
#endif
    }
    for (k = 0; k < 9; ++k) negD[k] = -u->dalpha_db_g[k]; /* (-dalpha_db_g_) * Delta_b.head<3>() */
    ok_m3_mulv(negD, db, phi);
    dq = ok_kin_delta_q(phi);
    ok_quat_mul(q, &u->delta_q, &qa);
    ok_quat_mul(&qa, &dq, &qb);
    for (k = 0; k < 3; ++k) rr[k] = pos[k];
    ok_tf_from_rq(T_tg, rr, &qb, 1); /* Transformation::set(r, q): q normalised, C cached */
}

int ok_gps_async_evaluate(ok_gps_async* e, const double* const p[3], double res[3], double* const* jac,
                          double* const* jacmin) {
    ok_imu_error* u = &e->imu;
    ok_tf T_tk, T_tg;
    ok_quat q;
    double C_GW[9], sb[9], db[6], dt, arm[3], v[3], gv[3], Jprop[225], blk[9], negC[9], Jm[18], Jtk[18];
    int i, j, k;

    if (u->n_meas == 0) return 0; /* upstream: back() of an empty deque, undefined */
    q.x = p[0][3]; q.y = p[0][4]; q.z = p[0][5]; q.w = p[0][6];
    ok_tf_from_rq(&T_tk, p[0], &q, 1);    /* Transformation T_WS_tk(r_WS, q_WS): q normalised, C cached */
    memcpy(sb, p[1], 9 * sizeof(double));
    pose_rotation(p[2], C_GW);
    dt = ok_time_diff_sec(u->t1, u->t0);  /* (tg_ - tk_).toSec() */

    for (k = 0; k < 6; ++k) db[k] = sb[3 + k] - u->sb_ref[3 + k];
    if (!u->redo) u->redo = ok_v3_norm(db) > 0.0003; /* redo_ || (Delta_b.head<3>().norm() > 0.0003) */
    if (e->redo_always || (u->redo && u->n_meas < (OK_GPS_MUTATE == 9 ? 51 : 50)) || u->redo_counter == 0) {
        ok_imu_redo_preintegration(u, sb); /* the return value (-1: not covered) is ignored upstream too */
        u->redo_counter++;
        for (k = 0; k < 6; ++k) db[k] = 0.0;
        u->redo = 0;
    }

    propagate(u, p[0], T_tk.C, &T_tk.q, sb, db, dt, &T_tg);

    /* Jprop (15x15), only rows 0-5 are read: [I, -[C acc_dd]x, I*dt, C dp_db_g, -C C_dd; 0, I, 0, -C dalpha_db_g, 0] */
    for (k = 0; k < 225; ++k) Jprop[k] = 0.0;
    for (k = 0; k < 15; ++k) F15(Jprop, k, k) = 1.0;
    ok_m3_mulv(T_tk.C, u->acc_doubleintegral, v);
    ok_kin_cross_mx(v, blk);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) F15(Jprop, i, 3 + j) = -M3(blk, i, j);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) F15(Jprop, i, 6 + j) = (i == j ? 1.0 : 0.0) * dt; /* Identity() * Delta_t */
    ok_m3_mul(T_tk.C, u->dp_db_g, blk);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) F15(Jprop, i, 9 + j) = M3(blk, i, j);
    for (k = 0; k < 9; ++k) negC[k] = -T_tk.C[k];
#if OK_GPS_MUTATE == 4
    ok_m3_mul(u->C_doubleintegral, negC, blk);
#else
    ok_m3_mul(negC, u->C_doubleintegral, blk);
#endif
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) F15(Jprop, i, 12 + j) = M3(blk, i, j);
    ok_m3_mul(negC, u->dalpha_db_g, blk);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) F15(Jprop, 3 + i, 9 + j) = M3(blk, i, j);

    if (e->use_imu_covariance) {
        double T[225], P[225], P6[36], Jws[18], JwsT[18], t1[18], cov[9], info[9], nG[9], X[9];
        for (k = 0; k < 225; ++k) T[k] = 0.0;
        for (k = 0; k < 15; ++k) F15(T, k, k) = 1.0;
        for (k = 0; k < 3; ++k) /* T.topLeftCorner<3,3>(), T.block<3,3>(3,3), T.block<3,3>(6,6) = C_WS_tk */
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) F15(T, 3 * k + i, 3 * k + j) = M3(T_tk.C, i, j);
        gemm_pqpt(T, u->P_delta, P);                       /* P = T * P_delta_ * T.transpose() */
        ok_m3_mulv(T_tk.C, e->lever, v);                   /* T_WS_tk.C() * r_SA */
        ok_kin_cross_mx(v, X);
        for (k = 0; k < 9; ++k) nG[k] = -C_GW[k];
        ok_m3_mul(nG, X, Jws + 9);                         /* Jws.block<3,3>(0,3) = -C_GW * crossMx(...) */
        memcpy(Jws, C_GW, 9 * sizeof(double));             /* Jws.block<3,3>(0,0) = C_GW */
        for (j = 0; j < 6; ++j)
            for (i = 0; i < 6; ++i) P6[i + 6 * j] = F15(P, i, j); /* P.block<6,6>(0,0) */
        transpose(3, 6, Jws, JwsT);                        /* Jws.transpose() */
        ok_lazy_a(3, 6, 6, Jws, P6, t1);      /* Jws * P6   (3x6) */
#if OK_GPS_MUTATE == 7
        ok_lazy_tree(3, 3, 6, t1, JwsT, cov);
#else
        ok_lazy_a(3, 3, 6, t1, JwsT, cov);                 /* (Jws * P6) * Jws^T (3x3) */
#endif
        for (k = 0; k < 9; ++k) cov[k] = e->covariance[k] + cov[k];
        inverse3(cov, info);                               /* covOverall.inverse() */
        sqrt_information(info, e->sqrt_info);
    } else {
        double info[9];
        inverse3(e->covariance, info);
        sqrt_information(info, e->sqrt_info);
    }

    ok_m3_mulv(T_tg.C, e->lever, arm);                     /* T_WS_tg.C() * r_SA */
    for (k = 0; k < 3; ++k) v[k] = T_tg.r[k] + arm[k];
    ok_m3_mulv(C_GW, v, gv);
#if OK_GPS_MUTATE == 8
    for (k = 0; k < 3; ++k) e->error[k] = (e->meas[k] - gv[k]) - p[2][k];
#else
    for (k = 0; k < 3; ++k) e->error[k] = e->meas[k] - (gv[k] + p[2][k]);
#endif
    ok_m3_mulv(e->sqrt_info, e->error, res);

    if (jac != NULL) {
        if (jac[0] != NULL) { /* w.r.t. the pose at tk */
            double Jg[18], Jd[36], W[18], L[42], Lrm[42], F[21];
            jac_pose_minimal(C_GW, arm, Jg);                /* at T_WS_tg */
            for (j = 0; j < 6; ++j)
                for (i = 0; i < 6; ++i) Jd[i + 6 * j] = F15(Jprop, i, j); /* Jprop.topLeftCorner<6,6>() */
            ok_lazy_a(3, 6, 6, Jg, Jd, Jtk);                /* J0_minimal_tk = J0_minimal_tg * J_dxg_dxt */
            ok_lazy_a(3, 6, 3, e->sqrt_info, Jtk, W);       /* sqrtI * J0_minimal_tk */
            ok_pose_minus_jacobian(p[0], Lrm);
            transpose(7, 6, Lrm, L);
            LIFT_PRODUCT(3, 7, 6, W, L, F);
            transpose(3, 7, F, jac[0]);
            if (jacmin != NULL && jacmin[0] != NULL) transpose(3, 6, W, jacmin[0]);
        }
        if (jac[1] != NULL) { /* w.r.t. speed and biases at tk, no lift */
            double Jg[18], Jb[54], J1tk[27], W[27];
            jac_pose_minimal(C_GW, arm, Jg);
            for (j = 0; j < 9; ++j)
                for (i = 0; i < 6; ++i) Jb[i + 6 * j] = F15(Jprop, i, 6 + j); /* Jprop.block<6,9>(0,6) */
            ok_lazy_a(3, 9, 6, Jg, Jb, J1tk);               /* J1_tk = J1_minimal_tg * J_dxg_dsbk */
            ok_lazy_a(3, 9, 3, e->sqrt_info, J1tk, W);      /* J1 = sqrtI * J1_tk */
            transpose(3, 9, W, jac[1]);
            if (jacmin != NULL && jacmin[1] != NULL) memcpy(jacmin[1], jac[1], 27 * sizeof(double));
        }
        if (jac[2] != NULL) { /* w.r.t. T_GW */
            jac_world_minimal(gv, Jm);
            weight_lift(e->sqrt_info, Jm, p[2], jac[2], jacmin ? jacmin[2] : NULL);
        }
    }
    return 1;
}

int ok_gps_async_apply_preint(ok_gps_async* e, const ok_tf* T_in, const double sb_in[9], ok_tf* T_prop) {
    ok_imu_error* u = &e->imu;
    double db[6], dt;
    int k;
    dt = ok_time_diff_sec(u->t1, u->t0);
    for (k = 0; k < 6; ++k) db[k] = sb_in[3 + k] - u->sb_ref[3 + k];
    if (!u->redo) u->redo = ok_v3_norm(db) > 0.0003;
    propagate(u, T_in->r, T_in->C, &T_in->q, sb_in, db, dt, T_prop);
    return 1;
}

/* ---------------------------------------------- PoseManifold4d ---------------------------------------------- */
int ok_pose4_plus(const double x[7], const double d[4], double out[7]) {
    const ok_quat q = {x[3], x[4], x[5], x[6]};
    double d6[6] = {0, 0, 0, 0, 0, 0};
    ok_tf t;
    d6[0] = d[0]; d6[1] = d[1]; d6[2] = d[2]; d6[5] = d[3];
    ok_tf_from_rq(&t, x, &q, 1);
    ok_tf_oplus(&t, d6, 1);
    ok_pose_block_set_estimate(out, &t);
    return 1;
}
int ok_pose4_minus(const double y[7], const double x[7], double out[4]) {
    double d[6];
    ok_pose_minus(y, x, d);
    out[0] = y[0] - x[0]; out[1] = y[1] - x[1]; out[2] = y[2] - x[2];
    out[3] = d[5];
    return 1;
}
int ok_pose4_plus_jacobian(const double x[7], double J[28]) {
    const ok_quat q = {x[3], x[4], x[5], x[6]};
    ok_tf t;
    double F[42];
    int i, j;
    ok_tf_from_rq(&t, x, &q, 1);
    ok_tf_oplus_jacobian(&t, F); /* 7x6 column-major (ok_kin) */
    for (i = 0; i < 7; ++i) { /* Jp.topLeftCorner<7,3>() and Jp.bottomRightCorner<7,1>() of the 7x6 Jacobian */
        for (j = 0; j < 3; ++j) J[4 * i + j] = F[i + 7 * j];
        J[4 * i + 3] = F[i + 7 * 5];
    }
    return 1;
}
int ok_pose4_minus_jacobian(const double x[7], double J[28]) {
    double F[42];
    int k;
    ok_pose_minus_jacobian(x, F); /* 6x7: rows 0-2 = [I 0], rows 3-5 = [0 Jq_pinv * Qplus] */
    for (k = 0; k < 21; ++k) J[k] = F[k];
    for (k = 0; k < 7; ++k) J[21 + k] = F[35 + k]; /* the z row of the product */
    return 1;
}
int ok_pose4_right_multiply(const double x[7], int num_rows, const double* A, double* out) {
    double PJ[28];
    int i, j, k;
    ok_pose4_plus_jacobian(x, PJ);
    if (num_rows + 7 + 4 < 20) { /* coefficient-based lazy product: every entry a left fold */
        for (i = 0; i < num_rows; ++i)
            for (j = 0; j < 4; ++j) {
                double s = A[7 * i] * PJ[j];
                for (k = 1; k < 7; ++k) s = s + A[7 * i + k] * PJ[4 * k + j];
#if OK_GPS_MUTATE == 6 /* wrong: redux tree instead of the left fold */
                { double pr[7]; for (k = 0; k < 7; ++k) pr[k] = A[7 * i + k] * PJ[4 * k + j]; s = ok_red_tree(pr, 7); }
#endif
                out[4 * i + j] = s;
            }
    } else { /* GEBP on the transposed problem (row-major result): out^T(4 x rows) = PJ^T(4x7) * A^T(7 x rows) */
        ok_gemm(4, num_rows, 7, PJ, A, out);
    }
    return 1;
}
