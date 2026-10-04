/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 3b: error terms. See ok_err.h for notices.
 * "Math on paper": one equation per line, in the evaluation order of the reference build (Eigen 3.4.0, SSE2, no FMA).
 *
 * Eigen small-product models used below (all measured against real Eigen by okvis_err_test.cc):
 *   lazy products (rows+cols+depth small, CoeffBasedProductMode); the result's storage order decides the loop:
 *   A) lhs column-major (any rhs): packets along rows. Rows [0, 2*(R/2)): left fold of the D products
 *      (((p0+p1)+p2)+...), zero products included; an odd last row is a coefficient: redux tree (halving).
 *   B) lhs row-major AND rhs row-major, assigned into a Map<RowMajor> (all J = J_minimal * J_lift statements): every
 *      coefficient is scalar, redux tree (a plain row-major local would get packets for the even columns instead).
 *   C) lhs row-major, rhs column-major: every coefficient is (lhs.row.transpose().cwiseProduct(rhs.col)).sum() with
 *      both operands contiguous => vectorised redux (packet lanes, then horizontal add, scalar remainder).
 *   Matrix * Vector with a Large dimension (9x9 * 9): GEMV column kernel, per row a left fold from 0, res = 0 + 1*c.
 *   Large*Large (9x9 * 9x9): GEBP (ok_gemm), -A * B extracts alpha = -1: res = 0 + (-1 * acc). */
#include "ok_err.h"

#include <math.h>
#include <string.h>

#include "ok_param.h"

#define M4(a, i, j) (a)[(i) + 4 * (j)]

/* redux (sum) of n products: Eigen redux_novec_unroller, halving split */
double ok_red_tree(const double* p, int n) {
    if (n == 1) return p[0];
    return ok_red_tree(p, n / 2) + ok_red_tree(p + n / 2, n - n / 2);
}

/* redux of contiguous packet-accessible operands (PacketSize 2): redux_vec_unroller over n/2 packets (halving), then
 * the horizontal add, then the scalar remainder (n odd) */
void ok_red_vec_lanes(const double* p, int np, double lane[2]) {
    if (np == 1) { lane[0] = p[0]; lane[1] = p[1]; return; }
    {
        double a[2], b[2];
        ok_red_vec_lanes(p, np / 2, a);
        ok_red_vec_lanes(p + 2 * (np / 2), np - np / 2, b);
        lane[0] = a[0] + b[0];
        lane[1] = a[1] + b[1];
    }
}
double ok_red_vec(const double* p, int n) {
    if (n < 2) return p[0];
    {
        double lane[2], r;
        ok_red_vec_lanes(p, n / 2, lane);
        r = lane[0] + lane[1];
        if (n & 1) r = r + p[n - 1];
        return r;
    }
}

#define MAXD 8
/* A) out(R x C) = A(R x D) * B(D x C), all column-major, lhs column-major */
void ok_lazy_a(int R, int C, int D, const double* A, const double* B, double* out) {
    int i, j, k;
    for (j = 0; j < C; ++j)
        for (i = 0; i < R; ++i) {
            double p[MAXD] = {0.0};
            for (k = 0; k < D; ++k) p[k] = A[i + R * k] * B[k + D * j];
            if (i < 2 * (R / 2)) {
                double s = p[0];
                for (k = 1; k < D; ++k) s = s + p[k];
                out[i + R * j] = s;
            } else {
                out[i + R * j] = ok_red_tree(p, D);
            }
        }
}

/* B) out(R x C) = A(R x D) * B(D x C), all ROW-major, assigned straight into a row-major Map (or a small row-major
 * local with an odd column count): the assignment loop is scalar, every coefficient the redux tree */
void ok_lazy_tree_rm(int R, int C, int D, const double* A, const double* B, double* out) {
    int i, j, k;
    for (i = 0; i < R; ++i)
        for (j = 0; j < C; ++j) {
            double p[MAXD];
            for (k = 0; k < D; ++k) p[k] = A[i * D + k] * B[k * C + j];
            out[i * C + j] = ok_red_tree(p, D);
        }
}

/* C) out(R x C) = A(R x D) [row-major] * B(D x C) [column-major] -> column-major out */
void ok_lazy_c(int R, int C, int D, const double* A, const double* B, double* out) {
    int i, j, k;
    for (j = 0; j < C; ++j)
        for (i = 0; i < R; ++i) {
            double p[MAXD];
            for (k = 0; k < D; ++k) p[k] = A[i * D + k] * B[k + D * j];
            out[i + R * j] = ok_red_vec(p, D);
        }
}

/* D) out(R x C) = A(R x D) * B(D x C), all column-major, with the product's storage order disagreeing with the
 * destination's (column-major lhs evaluated straight into a ROW-major fixed-size destination, i.e. a construction
 * `const Matrix<RowMajor> X = A * B` without the aliasing temporary): no packets, every coefficient the redux tree */
void ok_lazy_tree(int R, int C, int D, const double* A, const double* B, double* out) {
    int i, j, k;
    for (j = 0; j < C; ++j)
        for (i = 0; i < R; ++i) {
            double p[MAXD];
            for (k = 0; k < D; ++k) p[k] = A[i + R * k] * B[k + D * j];
            out[i + R * j] = ok_red_tree(p, D);
        }
}

static void transpose(int R, int C, const double* A, double* out) { /* A R x C col-major -> out C x R col-major */
    int i, j;
    for (j = 0; j < C; ++j)
        for (i = 0; i < R; ++i) out[j + C * i] = A[i + R * j];
}
static void neg(int n, const double* a, double* out) {
    int i;
    for (i = 0; i < n; ++i) out[i] = -a[i];
}
/* column-major R x C -> row-major buffer */
static void to_rm(int R, int C, const double* A, double* out) {
    int i, j;
    for (i = 0; i < R; ++i)
        for (j = 0; j < C; ++j) out[i * C + j] = A[i + R * j];
}
static void from_rm(int R, int C, const double* A, double* out) {
    int i, j;
    for (i = 0; i < R; ++i)
        for (j = 0; j < C; ++j) out[i + R * j] = A[i * C + j];
}
static void m4_mul(const double* a, const double* b, double* out) { ok_lazy_a(4, 4, 4, a, b, out); }
static void m4_mulv(const double* a, const double* v, double* out) { ok_lazy_a(4, 1, 4, a, v, out); }

/* ---------------------------------------------------- LLT ---------------------------------------------------- */
int ok_llt_sqrt_information(int n, const double* info, double* out) {
    double a[81];
    int i, j, k, ret = -1;
    memcpy(a, info, sizeof(double) * (size_t)(n * n));
#define A_(r, c) a[(r) + n * (c)]
    for (k = 0; k < n; ++k) {
        const int rs = n - k - 1;
        double x = A_(k, k);
        if (k > 0) {  /* A10.squaredNorm(): left fold of squares (strided row block, no vectorisation) */
            double sq = A_(k, 0) * A_(k, 0);
            for (j = 1; j < k; ++j) sq = sq + A_(k, j) * A_(k, j);
            x -= sq;
        }
        if (x <= 0.0) { ret = k; break; }
        x = sqrt(x);
        A_(k, k) = x;
        if (k > 0 && rs > 0) {  /* A21.noalias() -= A20 * A10^T : GEMV column kernel, per row fold from 0, res += (-1)*c */
            for (i = k + 1; i < n; ++i) {
                double c = 0.0;
                for (j = 0; j < k; ++j) c = c + A_(i, j) * A_(k, j);
                A_(i, k) = A_(i, k) + (-1.0) * c;
            }
        }
        if (rs > 0)
            for (i = k + 1; i < n; ++i) A_(i, k) = A_(i, k) / x;
    }
    /* matrixL().transpose() assigned to a dense matrix: upper triangle = L^T, strictly lower = 0 */
    {
        double r[81];
        for (j = 0; j < n; ++j)
            for (i = 0; i < n; ++i) r[i + n * j] = (i <= j) ? A_(j, i) : 0.0;
        memcpy(out, r, sizeof(double) * (size_t)(n * n));
    }
#undef A_
    return ret;
}

/* n x n: Matrix::Identity() * 1.0 / var, elementwise ((I*1.0)/var) */
static void identity_over(int n, double* m, double var) {
    int i, j;
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) m[i + n * j] = ((i == j ? 1.0 : 0.0) * 1.0) / var;
}

/* ---------------------------------------------- ReprojectionError ---------------------------------------------- */
void ok_reproj_err_set_information(ok_reproj_err* e, const double info[4]) {
    memcpy(e->info, info, sizeof e->info);
    ok_llt_sqrt_information(2, e->info, e->sqrt_info);
}

void ok_reproj_err_init(ok_reproj_err* e, const ok_cam* cam, const double meas[2], const double info[4]) {
    e->cam = *cam;
    e->meas[0] = meas[0]; e->meas[1] = meas[1];
    ok_reproj_err_set_information(e, info);
}

/* T = [C, -C*t; 0 0 0 1] with C = rotation matrix of the normalised quaternion of `p` (pose block), as built
 * by ReprojectionError for T_SW / T_CS from (C^T, t): `Cinv` receives C^T */
static void rt_inverse_matrix(const double Cinv[9], const double t[3], double T[16]) {
    double nC[9], v[3];
    int i, j;
    neg(9, Cinv, nC);
    ok_m3_mulv(nC, t, v);  /* -C * t : negated lhs, Matrix3d * Vector3d */
    for (j = 0; j < 4; ++j)
        for (i = 0; i < 4; ++i) M4(T, i, j) = (i == j) ? 1.0 : 0.0;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M4(T, i, j) = Cinv[i + 3 * j];
    M4(T, 0, 3) = v[0]; M4(T, 1, 3) = v[1]; M4(T, 2, 3) = v[2];
}

int ok_reproj_err_evaluate(const ok_reproj_err* e, const double* const params[3], double res[2], double* const* jac,
                           double* const* jacmin) {
    const double* p0 = params[0];
    const double* hp_W = params[1];
    const double* p2 = params[2];
    ok_quat q_WS = {p0[3], p0[4], p0[5], p0[6]};
    ok_quat q_SC = {p2[3], p2[4], p2[5], p2[6]};
    double C_SC[9], C_CS[9], C_WS[9], C_SW[9], T_CS[16], T_SW[16], hp_S[4], hp_C[4];
    double kp[2] = {0.0, 0.0}, Jh[8] = {0, 0, 0, 0, 0, 0, 0, 0}, Jh_w[8], err[2];
    int i, j;
    ok_quat_normalize(&q_WS);
    ok_quat_normalize(&q_SC);
    ok_quat_to_mat3(&q_SC, C_SC);
    transpose(3, 3, C_SC, C_CS);
    ok_quat_to_mat3(&q_WS, C_WS);
    transpose(3, 3, C_WS, C_SW);
    rt_inverse_matrix(C_CS, p2, T_CS);   /* T_CS = [C_CS, -C_CS * t_SC_S] */
    rt_inverse_matrix(C_SW, p0, T_SW);   /* T_SW = [C_SW, -C_SW * t_WS_W] */
    m4_mulv(T_SW, hp_W, hp_S);
    m4_mulv(T_CS, hp_S, hp_C);
    if (jac != NULL) {
        ok_cam_project_h_j(&e->cam, hp_C, kp, Jh, NULL);
        ok_lazy_a(2, 4, 2, e->sqrt_info, Jh, Jh_w);  /* Jh_weighted = squareRootInformation_ * Jh */
    } else {
        ok_cam_project_h(&e->cam, hp_C, kp);
    }
    err[0] = e->meas[0] - kp[0];
    err[1] = e->meas[1] - kp[1];
    ok_lazy_a(2, 1, 2, e->sqrt_info, err, res);  /* weighted_error = squareRootInformation_ * error */

    if (jac != NULL) {
        if (jac[0] != NULL) {
            double p[3], J[24], cr[9], nC[9], blk[9], tmp[8], J0m[12], J0m_rm[12], Jlift[42];
            for (i = 0; i < 3; ++i) p[i] = hp_W[i] - p0[i] * hp_W[3];
            for (i = 0; i < 24; ++i) J[i] = 0.0;
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) J[i + 4 * j] = C_SW[i + 3 * j] * hp_W[3];  /* topLeft = C_SW * hp_W[3] */
            ok_kin_cross_mx(p, cr);
            neg(9, C_SW, nC);
            ok_m3_mul(nC, cr, blk);  /* topRight = -C_SW * crossMx(p) */
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) J[i + 4 * (3 + j)] = blk[i + 3 * j];
            ok_lazy_a(2, 4, 4, Jh_w, T_CS, tmp);   /* (Jh_weighted * T_CS) */
            ok_lazy_a(2, 6, 4, tmp, J, J0m);       /* ... * J */
            to_rm(2, 6, J0m, J0m_rm);
            ok_pose_minus_jacobian(p0, Jlift);
            ok_lazy_tree_rm(2, 7, 6, J0m_rm, Jlift, jac[0]);  /* J0 = J0_minimal * J_lift */
            if (jacmin != NULL && jacmin[0] != NULL) memcpy(jacmin[0], J0m_rm, 12 * sizeof(double));
        }
        if (jac[1] != NULL) {
            double T_CW[16], nJ[8], J1c[8], S[12], J1m[6];
            m4_mul(T_CS, T_SW, T_CW);
            neg(8, Jh_w, nJ);
            ok_lazy_a(2, 4, 4, nJ, T_CW, J1c);  /* J1 = -Jh_weighted * T_CW */
            to_rm(2, 4, J1c, jac[1]);
            if (jacmin != NULL && jacmin[1] != NULL) {
                for (i = 0; i < 12; ++i) S[i] = 0.0;
                S[0 + 4 * 0] = 1.0; S[1 + 4 * 1] = 1.0; S[2 + 4 * 2] = 1.0;
                ok_lazy_c(2, 3, 4, jac[1], S, J1m);  /* J1_minimal = J1 (row-major map) * S */
                to_rm(2, 3, J1m, jacmin[1]);
            }
        }
        if (jac[2] != NULL) {
            double p[3], J[24], cr[9], nC[9], blk[9], J2m[12], J2m_rm[12], Jlift[42];
            for (i = 0; i < 3; ++i) p[i] = hp_S[i] - p2[i] * hp_S[3];
            for (i = 0; i < 24; ++i) J[i] = 0.0;
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) J[i + 4 * j] = C_CS[i + 3 * j] * hp_S[3];
            ok_kin_cross_mx(p, cr);
            neg(9, C_CS, nC);
            ok_m3_mul(nC, cr, blk);
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) J[i + 4 * (3 + j)] = blk[i + 3 * j];
            ok_lazy_a(2, 6, 4, Jh_w, J, J2m);  /* J2_minimal = Jh_weighted * J */
            to_rm(2, 6, J2m, J2m_rm);
            ok_pose_minus_jacobian(p2, Jlift);
            ok_lazy_tree_rm(2, 7, 6, J2m_rm, Jlift, jac[2]);
            if (jacmin != NULL && jacmin[2] != NULL) memcpy(jacmin[2], J2m_rm, 12 * sizeof(double));
        }
    }
    return 1;
}

/* ------------------------------------------------- PoseError --------------------------------------------------- */
void ok_pose_err_set_information(ok_pose_err* e, const double info[36]) {
    memcpy(e->info, info, sizeof e->info);
    ok_llt_sqrt_information(6, e->info, e->sqrt_info);
}
void ok_pose_err_init_info(ok_pose_err* e, const ok_tf* T, const double info[36]) {
    e->meas = *T;
    ok_pose_err_set_information(e, info);
}
void ok_pose_err_init_diag(ok_pose_err* e, const ok_tf* T, const double diag[6]) {
    int i, j;
    e->meas = *T;
    for (j = 0; j < 6; ++j)
        for (i = 0; i < 6; ++i) {
            e->info[i + 6 * j] = (i == j) ? diag[i] : 0.0;
            e->sqrt_info[i + 6 * j] = (i == j) ? sqrt(diag[i]) : 0.0;
        }
}
void ok_pose_err_init_var(ok_pose_err* e, const ok_tf* T, double tv, double rv) {
    double info[36], blk[9];
    int i, j;
    e->meas = *T;
    for (i = 0; i < 36; ++i) info[i] = 0.0;
    identity_over(3, blk, tv);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) info[i + 6 * j] = blk[i + 3 * j];
    identity_over(3, blk, rv);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) info[(3 + i) + 6 * (3 + j)] = blk[i + 3 * j];
    ok_pose_err_set_information(e, info);
}

/* shared tail of PoseError / RelativePoseError Jacobian: J_minimal (6x6, column-major in `Jm`) = sqrtInfo * Jm;
 * jac (6x7 row-major) = Jm_rm * J_lift(of `pblock`) ; jacmin = Jm_rm */
static void pose_jac_tail(const double sqrt_info[36], double Jm[36], const double pblock[7], double* jac,
                          double* jacmin) {
    double t[36], Jlift[42], Jm_rm[36];
    ok_lazy_a(6, 6, 6, sqrt_info, Jm, t);  /* J_minimal = (squareRootInformation_ * J_minimal).eval() */
    to_rm(6, 6, t, Jm_rm);
    ok_pose_minus_jacobian(pblock, Jlift);
    ok_lazy_tree_rm(6, 7, 6, Jm_rm, Jlift, jac);  /* J0 = J0_minimal * J_lift */
    if (jacmin != NULL) memcpy(jacmin, Jm_rm, 36 * sizeof(double));
}

int ok_pose_err_evaluate(const ok_pose_err* e, const double* const params[1], double res[6], double* const* jac,
                         double* const* jacmin) {
    const double* p = params[0];
    ok_tf T_WS, inv, dp;
    ok_quat qn = {p[3], p[4], p[5], p[6]};
    double r3[3] = {p[0], p[1], p[2]}, err[6], Q[16];
    int i, j;
    qn = ok_quat_normalized(qn);
    ok_tf_from_rq(&T_WS, r3, &qn, 1);
    ok_tf_inverse(&T_WS, &inv, 1);
    ok_tf_mul(&e->meas, &inv, &dp, 1);  /* dp = measurement_ * T_WS.inverse() */
    err[0] = e->meas.r[0] - T_WS.r[0];
    err[1] = e->meas.r[1] - T_WS.r[1];
    err[2] = e->meas.r[2] - T_WS.r[2];
    err[3] = 2.0 * dp.q.x;
    err[4] = 2.0 * dp.q.y;
    err[5] = 2.0 * dp.q.z;
    ok_lazy_a(6, 1, 6, e->sqrt_info, err, res);
    if (jac != NULL && jac[0] != NULL) {
        double Jm[36];
        for (j = 0; j < 6; ++j)
            for (i = 0; i < 6; ++i) Jm[i + 6 * j] = ((i == j) ? 1.0 : 0.0);
        for (i = 0; i < 36; ++i) Jm[i] = Jm[i] * -1.0;   /* setIdentity(); *= -1.0 (zeros become -0) */
        ok_kin_plus(&dp.q, Q);
        for (j = 0; j < 3; ++j)
            for (i = 0; i < 3; ++i) Jm[(3 + i) + 6 * (3 + j)] = -M4(Q, i, j);
        pose_jac_tail(e->sqrt_info, Jm, p, jac[0], (jacmin != NULL) ? jacmin[0] : NULL);
    }
    return 1;
}

/* ---------------------------------------------- SpeedAndBiasError ---------------------------------------------- */
void ok_sab_err_set_information(ok_sab_err* e, const double info[81]) {
    memcpy(e->info, info, sizeof e->info);
    ok_llt_sqrt_information(9, e->info, e->sqrt_info);
}
void ok_sab_err_init_info(ok_sab_err* e, const double meas[9], const double info[81]) {
    memcpy(e->meas, meas, sizeof e->meas);
    ok_sab_err_set_information(e, info);
}
void ok_sab_err_init_var(ok_sab_err* e, const double meas[9], double sv, double gv, double av) {
    double info[81], blk[9];
    int i, j, b;
    const double var[3] = {sv, gv, av};
    memcpy(e->meas, meas, sizeof e->meas);
    for (i = 0; i < 81; ++i) info[i] = 0.0;
    for (b = 0; b < 3; ++b) {
        identity_over(3, blk, var[b]);
        for (j = 0; j < 3; ++j)
            for (i = 0; i < 3; ++i) info[(3 * b + i) + 9 * (3 * b + j)] = blk[i + 3 * j];
    }
    ok_sab_err_set_information(e, info);
}

/* -sqrtInfo * Identity (9x9, Large*Large => GEBP with alpha = -1), written row-major */
static void sab_neg_sqrt_rm(const double sqrt_info[81], double* out_rm) {
    double id[81], g[81], r[81];
    int i, j;
    for (j = 0; j < 9; ++j)
        for (i = 0; i < 9; ++i) id[i + 9 * j] = (i == j) ? 1.0 : 0.0;
    ok_gemm(9, 9, 9, sqrt_info, id, g);  /* g = 0 + 1*acc */
    for (i = 0; i < 81; ++i) r[i] = 0.0 + ((-1.0) * g[i]);
    to_rm(9, 9, r, out_rm);
}

int ok_sab_err_evaluate(const ok_sab_err* e, const double* const params[1], double res[9], double* const* jac,
                        double* const* jacmin) {
    double err[9];
    int i, j;
    for (i = 0; i < 9; ++i) err[i] = e->meas[i] - params[0][i];
    for (i = 0; i < 9; ++i) {  /* GEMV column kernel: fold from 0, dst (zeroed) += alpha * c */
        double c = 0.0;
        for (j = 0; j < 9; ++j) c = c + e->sqrt_info[i + 9 * j] * err[j];
        res[i] = 0.0 + 1.0 * c;
    }
    if (jac != NULL && jac[0] != NULL) sab_neg_sqrt_rm(e->sqrt_info, jac[0]);
    if (jacmin != NULL && jacmin[0] != NULL) sab_neg_sqrt_rm(e->sqrt_info, jacmin[0]);
    return 1;
}

/* ---------------------------------------------- RelativePoseError ---------------------------------------------- */
void ok_relpose_err_set_information(ok_relpose_err* e, const double info[36]) {
    memcpy(e->info, info, sizeof e->info);
    ok_llt_sqrt_information(6, e->info, e->sqrt_info);
}
void ok_relpose_err_init_info(ok_relpose_err* e, const double info[36], const ok_tf* T_AB) {
    e->T_AB = *T_AB;
    ok_relpose_err_set_information(e, info);
}
void ok_relpose_err_init_var(ok_relpose_err* e, double tv, double rv, const ok_tf* T_AB) {
    double info[36], blk[9];
    int i, j;
    e->T_AB = *T_AB;
    for (i = 0; i < 36; ++i) info[i] = 0.0;
    identity_over(3, blk, tv);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) info[i + 6 * j] = blk[i + 3 * j];
    identity_over(3, blk, rv);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) info[(3 + i) + 6 * (3 + j)] = blk[i + 3 * j];
    ok_relpose_err_set_information(e, info);
}

int ok_relpose_err_evaluate(const ok_relpose_err* e, const double* const params[2], double res[6], double* const* jac,
                            double* const* jacmin) {
    const double* pa = params[0];
    const double* pb = params[1];
    ok_tf T_WA, T_WB, T_AWi, T_AB, T_AW, T_BW;
    ok_quat qa = {pa[3], pa[4], pa[5], pa[6]}, qb = {pb[3], pb[4], pb[5], pb[6]}, qinv, qd;
    double ra[3] = {pa[0], pa[1], pa[2]}, rb[3] = {pb[0], pb[1], pb[2]}, err[6];
    int i, j;
    qa = ok_quat_normalized(qa);
    qb = ok_quat_normalized(qb);
    ok_tf_from_rq(&T_WA, ra, &qa, 1);
    ok_tf_from_rq(&T_WB, rb, &qb, 1);
    ok_tf_inverse(&T_WA, &T_AWi, 1);
    ok_tf_mul(&T_AWi, &T_WB, &T_AB, 1);  /* T_AB = T_WA.inverse() * T_WB */
    err[0] = e->T_AB.r[0] - T_AB.r[0];
    err[1] = e->T_AB.r[1] - T_AB.r[1];
    err[2] = e->T_AB.r[2] - T_AB.r[2];
    qinv = ok_quat_inverse(T_AB.q);
    ok_quat_mul(&e->T_AB.q, &qinv, &qd);
    err[3] = 2.0 * qd.x;
    err[4] = 2.0 * qd.y;
    err[5] = 2.0 * qd.z;
    ok_lazy_a(6, 1, 6, e->sqrt_info, err, res);
    ok_tf_inverse(&T_WA, &T_AW, 1);
    ok_tf_inverse(&T_WB, &T_BW, 1);
    if (jac != NULL) {
        double qq[16], oo[16], pm[16], blk[9];
        ok_quat q1;
        ok_quat_mul(&e->T_AB.q, &T_BW.q, &q1);
        ok_kin_plus(&q1, qq);
        ok_kin_oplus(&T_WA.q, oo);
        m4_mul(qq, oo, pm);  /* plus(T_AB_.q * T_BW.q) * oplus(T_WA.q) */
        if (jac[0] != NULL) {
            double Jm[36], nC[9], cr[9], d[3];
            for (j = 0; j < 6; ++j)
                for (i = 0; i < 6; ++i) Jm[i + 6 * j] = ((i == j) ? 1.0 : 0.0);
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) Jm[i + 6 * j] = T_AW.C[i + 3 * j];
            d[0] = T_WB.r[0] - T_WA.r[0]; d[1] = T_WB.r[1] - T_WA.r[1]; d[2] = T_WB.r[2] - T_WA.r[2];
            ok_kin_cross_mx(d, cr);
            neg(9, T_AW.C, nC);
            ok_m3_mul(nC, cr, blk);  /* -T_AW.C() * crossMx(T_WB.r() - T_WA.r()) */
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) Jm[i + 6 * (3 + j)] = blk[i + 3 * j];
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) Jm[(3 + i) + 6 * (3 + j)] = M4(pm, i, j);
            pose_jac_tail(e->sqrt_info, Jm, pa, jac[0], (jacmin != NULL) ? jacmin[0] : NULL);
        }
        if (jac[1] != NULL) {
            double Jm[36];
            for (j = 0; j < 6; ++j)
                for (i = 0; i < 6; ++i) Jm[i + 6 * j] = ((i == j) ? 1.0 : 0.0);
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) Jm[i + 6 * j] = -T_AW.C[i + 3 * j];
            for (j = 0; j < 3; ++j)
                for (i = 0; i < 3; ++i) Jm[(3 + i) + 6 * (3 + j)] = -M4(pm, i, j);
            pose_jac_tail(e->sqrt_info, Jm, pb, jac[1], (jacmin != NULL) ? jacmin[1] : NULL);
        }
    }
    return 1;
}

/* -------------------------------------------- HomogeneousPointError -------------------------------------------- */
void ok_hpoint_err_set_information(ok_hpoint_err* e, const double info[9]) {
    memcpy(e->info, info, sizeof e->info);
    ok_llt_sqrt_information(3, e->info, e->sqrt_info);
}
void ok_hpoint_err_init_info(ok_hpoint_err* e, const double meas[4], const double info[9]) {
    memcpy(e->meas, meas, sizeof e->meas);
    ok_hpoint_err_set_information(e, info);
}
void ok_hpoint_err_init_var(ok_hpoint_err* e, const double meas[4], double variance) {
    double info[9];
    memcpy(e->meas, meas, sizeof e->meas);
    identity_over(3, info, variance);
    ok_hpoint_err_set_information(e, info);
}

int ok_hpoint_err_evaluate(const ok_hpoint_err* e, const double* const params[1], double res[3], double* const* jac,
                           double* const* jacmin) {
    double err[3];
    ok_hpoint_minus(params[0], e->meas, err);
    ok_lazy_a(3, 1, 3, e->sqrt_info, err, res);
    if (jac != NULL && jac[0] != NULL) {
        double Jl[12], Jp[12], Jm_rm[9], Jm[9], t[9], Jm2_rm[9];
        ok_hpoint_minus_jacobian(params[0], Jl);   /* 3x4 row-major */
        ok_hpoint_plus_jacobian(params[0], Jp);    /* 4x3 row-major */
        ok_lazy_tree_rm(3, 3, 4, Jl, Jp, Jm_rm);         /* J_lift * J_plus */
        from_rm(3, 3, Jm_rm, Jm);
        ok_lazy_a(3, 3, 3, e->sqrt_info, Jm, t);      /* (squareRootInformation * J0_minimal).eval() */
        to_rm(3, 3, t, Jm2_rm);
        ok_lazy_tree_rm(3, 4, 3, Jm2_rm, Jl, jac[0]);    /* J0 = J0_minimal * J_lift */
        if (jacmin != NULL && jacmin[0] != NULL) memcpy(jacmin[0], Jm2_rm, 9 * sizeof(double));
    }
    return 1;
}
