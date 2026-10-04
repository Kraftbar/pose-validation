/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 2b: kinematics. See ok_kin.h for notices.
 * "Math on paper": one equation per line in the evaluation order of the reference build. */
#include "ok_kin.h"

#include <math.h>
#include <string.h>

#define M3(a, i, j) (a)[(i) + 3 * (j)]
#define M4(a, i, j) (a)[(i) + 4 * (j)]

void ok_kin_cross_mx(const double v[3], double out[9]) {
    const double x = v[0], y = v[1], z = v[2];
    M3(out, 0, 0) = 0.0; M3(out, 0, 1) = -z;  M3(out, 0, 2) = y;
    M3(out, 1, 0) = z;   M3(out, 1, 1) = 0.0; M3(out, 1, 2) = -x;
    M3(out, 2, 0) = -y;  M3(out, 2, 1) = x;   M3(out, 2, 2) = 0.0;
}

void ok_kin_plus(const ok_quat* q_, double Q[16]) {
    const double q[4] = {q_->x, q_->y, q_->z, q_->w};
    M4(Q, 0, 0) = q[3];  M4(Q, 0, 1) = -q[2]; M4(Q, 0, 2) = q[1];  M4(Q, 0, 3) = q[0];
    M4(Q, 1, 0) = q[2];  M4(Q, 1, 1) = q[3];  M4(Q, 1, 2) = -q[0]; M4(Q, 1, 3) = q[1];
    M4(Q, 2, 0) = -q[1]; M4(Q, 2, 1) = q[0];  M4(Q, 2, 2) = q[3];  M4(Q, 2, 3) = q[2];
    M4(Q, 3, 0) = -q[0]; M4(Q, 3, 1) = -q[1]; M4(Q, 3, 2) = -q[2]; M4(Q, 3, 3) = q[3];
}

void ok_kin_oplus(const ok_quat* q_, double Q[16]) {
    const double q[4] = {q_->x, q_->y, q_->z, q_->w};
    M4(Q, 0, 0) = q[3];  M4(Q, 0, 1) = q[2];  M4(Q, 0, 2) = -q[1]; M4(Q, 0, 3) = q[0];
    M4(Q, 1, 0) = -q[2]; M4(Q, 1, 1) = q[3];  M4(Q, 1, 2) = q[0];  M4(Q, 1, 3) = q[1];
    M4(Q, 2, 0) = q[1];  M4(Q, 2, 1) = -q[0]; M4(Q, 2, 2) = q[3];  M4(Q, 2, 3) = q[2];
    M4(Q, 3, 0) = -q[0]; M4(Q, 3, 1) = -q[1]; M4(Q, 3, 2) = -q[2]; M4(Q, 3, 3) = q[3];
}

double ok_kin_sinc(double x) {
    if (fabs(x) > 1.0e-6) {
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

ok_quat ok_kin_delta_q(const double dAlpha[3]) {
    const double halfnorm = 0.5 * ok_v3_norm(dAlpha);
    const double s = ok_kin_sinc(halfnorm) * 0.5; /* sinc(halfnorm) * 0.5 * dAlpha: scalar product first */
    ok_quat q;
    q.x = s * dAlpha[0];
    q.y = s * dAlpha[1];
    q.z = s * dAlpha[2];
    q.w = cos(halfnorm);
    return q;
}

void ok_kin_right_jacobian(const double phi_vec[3], double out[9]) {
    const double Phi = ok_v3_norm(phi_vec);
    double Phi_x[9], Phi_x2[9];
    int k;
    ok_kin_cross_mx(phi_vec, Phi_x);
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

ok_quat ok_quat_from_mat3(const double m[9]) {
    ok_quat q;
    double c[4]; /* x y z w, written like coeffs().coeffRef(i) */
    double t = M3(m, 0, 0) + (M3(m, 1, 1) + M3(m, 2, 2)); /* trace(): redux d0 + (d1 + d2) */
    c[0] = c[1] = c[2] = c[3] = 0.0;
    if (t > 0.0) {
        t = sqrt(t + 1.0);
        c[3] = 0.5 * t;
        t = 0.5 / t;
        c[0] = (M3(m, 2, 1) - M3(m, 1, 2)) * t;
        c[1] = (M3(m, 0, 2) - M3(m, 2, 0)) * t;
        c[2] = (M3(m, 1, 0) - M3(m, 0, 1)) * t;
    } else {
        int i = 0, j, k;
        if (M3(m, 1, 1) > M3(m, 0, 0)) i = 1;
        if (M3(m, 2, 2) > M3(m, i, i)) i = 2;
        j = (i + 1) % 3;
        k = (j + 1) % 3;
        t = sqrt(M3(m, i, i) - M3(m, j, j) - M3(m, k, k) + 1.0);
        c[i] = 0.5 * t;
        t = 0.5 / t;
        c[3] = (M3(m, k, j) - M3(m, j, k)) * t;
        c[j] = (M3(m, j, i) + M3(m, i, j)) * t;
        c[k] = (M3(m, k, i) + M3(m, i, k)) * t;
    }
    q.x = c[0]; q.y = c[1]; q.z = c[2]; q.w = c[3];
    return q;
}

/* ------------------------------------------ Transformation ------------------------------------------ */

static void update_c(ok_tf* t, int cached) {
    if (cached) ok_quat_to_mat3(&t->q, t->C);
}

void ok_tf_identity(ok_tf* t) {
    t->r[0] = t->r[1] = t->r[2] = 0.0;
    t->q.x = 0.0; t->q.y = 0.0; t->q.z = 0.0; t->q.w = 1.0;
    memset(t->C, 0, sizeof t->C);
    t->C[0] = t->C[4] = t->C[8] = 1.0;
}

void ok_tf_from_rq(ok_tf* t, const double r[3], const ok_quat* q, int cached) {
    t->r[0] = r[0]; t->r[1] = r[1]; t->r[2] = r[2];
    t->q = ok_quat_normalized(*q);
    update_c(t, cached);
}

void ok_tf_from_m4(ok_tf* t, const double m[16], int cached) {
    double R[9];
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) R[i + 3 * j] = M4(m, i, j);
    if (cached) memcpy(t->C, R, sizeof R); /* C_(T_AB.topLeftCorner<3,3>()) verbatim, not from q */
    t->r[0] = M4(m, 0, 3); t->r[1] = M4(m, 1, 3); t->r[2] = M4(m, 2, 3);
    t->q = ok_quat_from_mat3(R);
}

void ok_tf_set_m4(ok_tf* t, const double m[16], int cached) {
    double R[9];
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) R[i + 3 * j] = M4(m, i, j);
    t->r[0] = M4(m, 0, 3); t->r[1] = M4(m, 1, 3); t->r[2] = M4(m, 2, 3);
    t->q = ok_quat_from_mat3(R);
    update_c(t, cached);
}

void ok_tf_set_coeffs(ok_tf* t, const double c[7], int cached) {
    t->r[0] = c[0]; t->r[1] = c[1]; t->r[2] = c[2];
    t->q.x = c[3]; t->q.y = c[4]; t->q.z = c[5]; t->q.w = c[6];
    update_c(t, cached);
}

void ok_tf_convert(ok_tf* t, const double coeffs[7]) { ok_tf_set_coeffs(t, coeffs, 1); }

void ok_tf_inverse(const ok_tf* t, ok_tf* out, int cached) {
    double Ct[9], rot[9], v[3], r[3];
    ok_quat qi;
    const double* C = t->C;
    if (!cached) { ok_quat_to_mat3(&t->q, rot); C = rot; }
    (void)Ct;
    ok_m3_mulv_lhsT(C, t->r, v);                 /* C^T * r, all rows left-assoc */
    r[0] = -v[0]; r[1] = -v[1]; r[2] = -v[2];
    qi = ok_quat_inverse(t->q);
    ok_tf_from_rq(out, r, &qi, cached);          /* the ctor normalises the inverse quaternion again */
}

void ok_tf_mul(const ok_tf* a, const ok_tf* b, ok_tf* out, int cached) {
    double rot[9], v[3], r[3];
    ok_quat q;
    const double* C = a->C;
    if (!cached) { ok_quat_to_mat3(&a->q, rot); C = rot; }
    ok_m3_mulv(C, b->r, v);
    r[0] = v[0] + a->r[0]; r[1] = v[1] + a->r[1]; r[2] = v[2] + a->r[2];
    ok_quat_mul(&a->q, &b->q, &q);
    ok_tf_from_rq(out, r, &q, cached);
}

void ok_tf_mul_v3(const ok_tf* t, const double v[3], double out[3], int cached) {
    double rot[9];
    const double* C = t->C;
    if (!cached) { ok_quat_to_mat3(&t->q, rot); C = rot; }
    ok_m3_mulv(C, v, out);
}

void ok_tf_mul_v4(const ok_tf* t, const double v[4], double out[4], int cached) {
    double rot[9], p[3];
    const double s = v[3];
    const double* C = t->C;
    if (!cached) { ok_quat_to_mat3(&t->q, rot); C = rot; }
    ok_m3_mulv(C, v, p);                          /* v.head<3>() */
    out[0] = p[0] + t->r[0] * s;
    out[1] = p[1] + t->r[1] * s;
    out[2] = p[2] + t->r[2] * s;
    out[3] = s;
}

void ok_tf_oplus(ok_tf* t, const double delta[6], int cached) {
    const double dalpha[3] = {delta[3], delta[4], delta[5]};
    ok_quat dq, qn;
    const double halfnorm = 0.5 * ok_v3_norm(dalpha);
    const double s = ok_kin_sinc(halfnorm) * 0.5;
    t->r[0] += delta[0]; t->r[1] += delta[1]; t->r[2] += delta[2];
    dq.x = s * dalpha[0]; dq.y = s * dalpha[1]; dq.z = s * dalpha[2];
    dq.w = cos(halfnorm);
    ok_quat_mul(&dq, &t->q, &qn);
    ok_quat_normalize(&qn);
    t->q = qn;
    update_c(t, cached);
}

void ok_tf_oplus_jacobian(const ok_tf* t, double J[42]) {
    /* J (7x6) = [I3 0; 0 oplus(q)*S], S = [0.5*I3; 0] (4x3). Matrix4d*Matrix<4,3>: every entry a left fold. */
    double Q[16], S[12], P[12];
    int i, j, k;
#define J76(r, c) J[(r) + 7 * (c)]
    for (k = 0; k < 42; ++k) J[k] = 0.0;
    J76(0, 0) = J76(1, 1) = J76(2, 2) = 1.0;
    for (k = 0; k < 12; ++k) S[k] = 0.0;
    S[0 + 4 * 0] = 0.5; S[1 + 4 * 1] = 0.5; S[2 + 4 * 2] = 0.5;
    ok_kin_oplus(&t->q, Q);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 4; ++i) {
            double s = M4(Q, i, 0) * S[0 + 4 * j];
            s = s + M4(Q, i, 1) * S[1 + 4 * j];
            s = s + M4(Q, i, 2) * S[2 + 4 * j];
            s = s + M4(Q, i, 3) * S[3 + 4 * j];
            P[i + 4 * j] = s;
        }
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 4; ++i) J76(3 + i, 3 + j) = P[i + 4 * j];
#undef J76
}

void ok_tf_lift_jacobian(const ok_tf* t, double J[42]) {
    /* J (6x7) = [I3 0; 0 2*oplus(q^-1).topLeftCorner<3,4>()] */
    double Q[16];
    ok_quat qi = ok_quat_inverse(t->q);
    int i, j, k;
#define J67(r, c) J[(r) + 6 * (c)]
    for (k = 0; k < 42; ++k) J[k] = 0.0;
    J67(0, 0) = 1.0; J67(1, 1) = 1.0; J67(2, 2) = 1.0;
    J67(0, 1) = J67(0, 2) = J67(1, 0) = J67(1, 2) = J67(2, 0) = J67(2, 1) = 0.0;
    ok_kin_oplus(&qi, Q);
    for (j = 0; j < 4; ++j)
        for (i = 0; i < 3; ++i) J67(3 + i, 3 + j) = 2.0 * M4(Q, i, j);
#undef J67
}

void ok_tf_T4(const ok_tf* t, double out[16], int cached) {
    double rot[9];
    const double* C = t->C;
    int i, j;
    if (!cached) { ok_quat_to_mat3(&t->q, rot); C = rot; }
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M4(out, i, j) = M3(C, i, j);
    M4(out, 0, 3) = t->r[0]; M4(out, 1, 3) = t->r[1]; M4(out, 2, 3) = t->r[2];
    M4(out, 3, 0) = 0.0; M4(out, 3, 1) = 0.0; M4(out, 3, 2) = 0.0;
    M4(out, 3, 3) = 1.0;
}

void ok_tf_T3x4(const ok_tf* t, double out[12], int cached) {
    double rot[9];
    const double* C = t->C;
    int i, j;
    if (!cached) { ok_quat_to_mat3(&t->q, rot); C = rot; }
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) out[i + 3 * j] = M3(C, i, j);
    out[9] = t->r[0]; out[10] = t->r[1]; out[11] = t->r[2];
}

void ok_tf_C(const ok_tf* t, double out[9], int cached) {
    if (cached) memcpy(out, t->C, 9 * sizeof(double));
    else ok_quat_to_mat3(&t->q, out);
}
