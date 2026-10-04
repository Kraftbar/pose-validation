/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/* See rd_factor.h for provenance and licences. Each statement notes the Eigen 3.4.0 expression form it reproduces. */
#include "rd_factor.h"
#include <string.h>

static void m3_rot(const ok_quat* q, double out[9]) { ok_quat_to_mat3(q, out); }
static void m3_rot_conj(const ok_quat* q, double out[9]) { ok_quat qc = rd_quat_conj(*q); ok_quat_to_mat3(&qc, out); }

/* Coefficient-based lazy product of a column-major lhs with 2 rows (CanVectorizeLhs, packet = both rows): every entry is the left fold
 * ((a0*b0 + a1*b1) + a2*b2) ... of etor_product_packet_impl<ColMajor>. A is 2 x depth (lda 2), B(k,j) = B[k*bk + j*bj]. out 2 x cols col-major. */
static void mm2(int depth, int cols, const double* A, const double* B, long bk, long bj, double* out) {
    int j, k, i;
    for (j = 0; j < cols; ++j)
        for (i = 0; i < 2; ++i) {
            double acc = A[i] * B[j * bj];
            for (k = 1; k < depth; ++k) acc = A[i + 2 * k] * B[k * bk + j * bj] + acc;  /* pmadd(a, b, res) = a*b + res */
            out[i + 2 * j] = acc;
        }
}

void rd_dproj_dp(const double p[3], double out[6]) {
    /* (matrix<2,3>() << 1/z, 0, -x/(z*z), 0, 1/z, -y/(z*z)).finished(), stored column-major */
    const double a = 1.0 / p[2], b = -p[0] / (p[2] * p[2]), c = -p[1] / (p[2] * p[2]);
    out[0] = a; out[1] = 0.0; out[2] = 0.0; out[3] = a; out[4] = b; out[5] = c;
}

/* local_tangent (3x3): columns = s2_tangential_basis(z) (2) and z; its transpose is applied as T^T * v. */
static void local_tangent(const double z[3], double LT[9]) {
    double b[6];
    int i;
    rd_s2_tangential_basis(z, b);
    for (i = 0; i < 6; ++i) LT[i] = b[i];
    for (i = 0; i < 3; ++i) LT[6 + i] = z[i];
}

/* local_tangent.transpose() * y : lhs row-major (Transpose of a column-major matrix), rhs column vector: the inner product of a
 * contiguous row with the vector is vectorised (packet of 2 + scalar tail) = left fold (see ok_m3_mulv_lhsT). */
static void lt_t_mul(const double LT[9], const double y[3], double out[3]) { ok_m3_mulv_lhsT(LT, y, out); }

static void sandwich(const double LT[9], const double u[3], const double sic[4], double out[6]) {
    /* sqrt_inv_cov * dproj_dp(u) * local_tangent.transpose(): ((2x2 * 2x3) -> temporary) * (3x3)^T */
    double D[6], T[6], LTt[9];
    int i, j;
    rd_dproj_dp(u, D);
    mm2(2, 3, sic, D, 1, 2, T);                      /* B(k,j) = D[k + 2j] */
    for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) LTt[i + 3 * j] = LT[j + 3 * i];
    mm2(3, 3, T, LTt, 1, 3, out);                    /* (T * LT^T)(i,j) = sum_k T(i,k) LT(j,k) */
}

void rd_rpe_eval(const double z[3], const double z_ref[3], const rd_extrinsic* cam_ref, const rd_extrinsic* cam_tgt,
                 const double sqrt_inv_cov[4], const double* const params[5], double residual[2], double* jac[5]) {
    ok_quat q_tgt, q_ref, qcs_tgt_c;
    const double *p_tgt = params[1], *p_ref = params[3];
    const double inv_depth = *params[4];
    double LT[9], y_ref[3], t[3], y_ref_center[3], x[3], d[3], y_tgt_center[3], tc[3], y_tgt[3], u_tgt[3], r[2];
    int i;
    q_tgt.x = params[0][0]; q_tgt.y = params[0][1]; q_tgt.z = params[0][2]; q_tgt.w = params[0][3];
    q_ref.x = params[2][0]; q_ref.y = params[2][1]; q_ref.z = params[2][2]; q_ref.w = params[2][3];
    local_tangent(z, LT);
    for (i = 0; i < 3; ++i) y_ref[i] = z_ref[i] / inv_depth;
    rd_quat_rotate(&cam_ref->q_cs, y_ref, t);
    for (i = 0; i < 3; ++i) y_ref_center[i] = t[i] + cam_ref->p_cs[i];
    rd_quat_rotate(&q_ref, y_ref_center, t);
    for (i = 0; i < 3; ++i) x[i] = t[i] + p_ref[i];
    for (i = 0; i < 3; ++i) d[i] = x[i] - p_tgt[i];
    { ok_quat c = rd_quat_conj(q_tgt); rd_quat_rotate(&c, d, y_tgt_center); }
    qcs_tgt_c = rd_quat_conj(cam_tgt->q_cs);
    for (i = 0; i < 3; ++i) tc[i] = y_tgt_center[i] - cam_tgt->p_cs[i];
    rd_quat_rotate(&qcs_tgt_c, tc, y_tgt);
    lt_t_mul(LT, y_tgt, u_tgt);
    r[0] = u_tgt[0] / u_tgt[2]; r[1] = u_tgt[1] / u_tgt[2];   /* hnormalized() */

    if (jac) {
        double dr_dy_tgt[6], R_cs_tgt_c[9], dr_dy_tgt_center[6], R_tgt_c[9], dr_dx[6], R_ref[9], dr_dy_ref_center[6], H[9];
        m3_rot_conj(&cam_tgt->q_cs, R_cs_tgt_c);
        sandwich(LT, u_tgt, sqrt_inv_cov, dr_dy_tgt);
        mm2(3, 3, dr_dy_tgt, R_cs_tgt_c, 1, 3, dr_dy_tgt_center);
        m3_rot_conj(&q_tgt, R_tgt_c);
        mm2(3, 3, dr_dy_tgt_center, R_tgt_c, 1, 3, dr_dx);
        m3_rot(&q_ref, R_ref);
        mm2(3, 3, dr_dx, R_ref, 1, 3, dr_dy_ref_center);
        if (jac[0]) {   /* row-major 2x4: block<2,3> = dr_dy_tgt_center * hat(y_tgt_center), col 3 = 0 */
            double blk[6];
            rd_hat(y_tgt_center, H);
            mm2(3, 3, dr_dy_tgt_center, H, 1, 3, blk);
            for (i = 0; i < 3; ++i) { jac[0][0 * 4 + i] = blk[0 + 2 * i]; jac[0][1 * 4 + i] = blk[1 + 2 * i]; }
            jac[0][3] = 0.0; jac[0][7] = 0.0;
        }
        if (jac[1]) {   /* -dr_dx, row-major 2x3 */
            for (i = 0; i < 3; ++i) { jac[1][i] = -dr_dx[0 + 2 * i]; jac[1][3 + i] = -dr_dx[1 + 2 * i]; }
        }
        if (jac[2]) {   /* block<2,3> = (-dr_dy_ref_center) * hat(y_ref_center) */
            double neg[6], blk[6];
            for (i = 0; i < 6; ++i) neg[i] = -dr_dy_ref_center[i];
            rd_hat(y_ref_center, H);
            mm2(3, 3, neg, H, 1, 3, blk);
            for (i = 0; i < 3; ++i) { jac[2][0 * 4 + i] = blk[0 + 2 * i]; jac[2][1 * 4 + i] = blk[1 + 2 * i]; }
            jac[2][3] = 0.0; jac[2][7] = 0.0;
        }
        if (jac[3]) {
            for (i = 0; i < 3; ++i) { jac[3][i] = dr_dx[0 + 2 * i]; jac[3][3 + i] = dr_dx[1 + 2 * i]; }
        }
        if (jac[4]) {   /* -dr_dy_ref_center * camera_ref.q_cs.matrix() * y_ref / inv_depth */
            double neg[6], Rcs[9], T1[6], T2[2];
            for (i = 0; i < 6; ++i) neg[i] = -dr_dy_ref_center[i];
            m3_rot(&cam_ref->q_cs, Rcs);
            mm2(3, 3, neg, Rcs, 1, 3, T1);
            mm2(3, 1, T1, y_ref, 1, 3, T2);
            jac[4][0] = T2[0] / inv_depth; jac[4][1] = T2[1] / inv_depth;
        }
    }
    /* r = sqrt_inv_cov * r : 2x2 * 2 (packet, aliasing temporary) */
    {
        double out[2];
        mm2(2, 1, sqrt_inv_cov, r, 1, 2, out);
        residual[0] = out[0]; residual[1] = out[1];
    }
}

void rd_rot_prior_eval(const double z[3], const double z_ref[3], const rd_extrinsic* cam_ref, const rd_extrinsic* cam_tgt,
                       const double sqrt_inv_cov[4], const ok_quat* q_ref_center, const double q_tgt_p[4], double residual[2], double* jac0) {
    ok_quat q_tgt, qtc, qq;
    double LT[9], t[3], z_ref_center[3], z_tgt_center[3], tc[3], z_tgt[3], u_tgt[3], r[2];
    int i;
    q_tgt.x = q_tgt_p[0]; q_tgt.y = q_tgt_p[1]; q_tgt.z = q_tgt_p[2]; q_tgt.w = q_tgt_p[3];
    local_tangent(z, LT);
    rd_quat_rotate(&cam_ref->q_cs, z_ref, t);
    for (i = 0; i < 3; ++i) z_ref_center[i] = t[i] + cam_ref->p_cs[i];
    qtc = rd_quat_conj(q_tgt);
    ok_quat_mul(&qtc, q_ref_center, &qq);                     /* q_tgt.conjugate() * q_ref_center : quaternion product first */
    rd_quat_rotate(&qq, z_ref_center, z_tgt_center);
    { ok_quat c = rd_quat_conj(cam_tgt->q_cs); for (i = 0; i < 3; ++i) tc[i] = z_tgt_center[i] - cam_tgt->p_cs[i]; rd_quat_rotate(&c, tc, z_tgt); }
    lt_t_mul(LT, z_tgt, u_tgt);
    r[0] = u_tgt[0] / u_tgt[2]; r[1] = u_tgt[1] / u_tgt[2];
    if (jac0) {
        double dr_dz_tgt[6], Rc[9], dr_dz_tgt_center[6], H[9], blk[6];
        sandwich(LT, u_tgt, sqrt_inv_cov, dr_dz_tgt);
        m3_rot_conj(&cam_tgt->q_cs, Rc);
        mm2(3, 3, dr_dz_tgt, Rc, 1, 3, dr_dz_tgt_center);
        rd_hat(z_tgt_center, H);
        mm2(3, 3, dr_dz_tgt_center, H, 1, 3, blk);
        for (i = 0; i < 3; ++i) { jac0[i] = blk[0 + 2 * i]; jac0[4 + i] = blk[1 + 2 * i]; }
        jac0[3] = 0.0; jac0[7] = 0.0;
    }
    {
        double out[2];
        mm2(2, 1, sqrt_inv_cov, r, 1, 2, out);
        residual[0] = out[0]; residual[1] = out[1];
    }
}
