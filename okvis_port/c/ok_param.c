/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 3a: parameter blocks and manifolds. See ok_param.h for notices.
 * "Math on paper": one equation per line in the evaluation order of the reference build. */
#include "ok_param.h"

#include <string.h>

#define M4(a, i, j) (a)[(i) + 4 * (j)]

int ok_pose_plus(const double x[7], const double delta[6], double out[7]) {
    /* q = (deltaQ(delta.tail<3>()) * Quaterniond(x[6],x[3],x[4],x[5]).normalized()).normalized().coeffs() */
    const double dalpha[3] = {delta[3], delta[4], delta[5]};
    ok_quat q0 = {x[3], x[4], x[5], x[6]}, dq, qp, q;
    dq = ok_kin_delta_q(dalpha);
    q0 = ok_quat_normalized(q0);
    ok_quat_mul(&dq, &q0, &qp);
    q = ok_quat_normalized(qp);
    out[0] = x[0] + delta[0];
    out[1] = x[1] + delta[1];
    out[2] = x[2] + delta[2];
    out[3] = q.x; out[4] = q.y; out[5] = q.z; out[6] = q.w;
    return 1;
}

int ok_pose_plus_jacobian(const double x[7], double J[42]) {
    /* J (7x6, row-major) = [I3 0; 0 oplus(q) * S], S (4x3) = [0.5*I3; 0].
     * Matrix4d * Matrix<4,3> (lazy, rows packetised): every entry a plain left fold of the 4 products, zeros included. */
    const ok_quat q = {x[3], x[4], x[5], x[6]};
    double Q[16], S[12];
    int i, j, k;
    for (k = 0; k < 42; ++k) J[k] = 0.0;
    J[0 * 6 + 0] = 1.0; J[1 * 6 + 1] = 1.0; J[2 * 6 + 2] = 1.0;
    for (k = 0; k < 12; ++k) S[k] = 0.0;
    M4(S, 0, 0) = 0.5; M4(S, 1, 1) = 0.5; M4(S, 2, 2) = 0.5;  /* S is 4x3: index i + 4*j */
    ok_kin_oplus(&q, Q);
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 4; ++i) {
            double s = M4(Q, i, 0) * S[0 + 4 * j];
            s = s + M4(Q, i, 1) * S[1 + 4 * j];
            s = s + M4(Q, i, 2) * S[2 + 4 * j];
            s = s + M4(Q, i, 3) * S[3 + 4 * j];
            J[(3 + i) * 6 + (3 + j)] = s;
        }
    return 1;
}

int ok_pose_minus(const double y[7], const double x[7], double out[6]) {
    const ok_quat qpd = {y[3], y[4], y[5], y[6]};
    const ok_quat q = {x[3], x[4], x[5], x[6]};
    ok_quat qi, p;
    out[0] = y[0] - x[0];
    out[1] = y[1] - x[1];
    out[2] = y[2] - x[2];
    qi = ok_quat_inverse(q);
    ok_quat_mul(&qpd, &qi, &p);
    out[3] = 2.0 * p.x;
    out[4] = 2.0 * p.y;
    out[5] = 2.0 * p.z;
    return 1;
}

int ok_pose_minus_jacobian(const double x[7], double J[42]) {
    /* J_lift (6x7, row-major) = [I3 0; 0 Jq_pinv * oplus(q_inv)], Jq_pinv (3x4) = [2*I3 0], q_inv = (w, -x, -y, -z).
     * Jq_pinv * Qplus (3x4 * 4x4, lazy, lhs column-major): rows 0-1 packetised (left fold of 4 products), row 2 the
     * coefficient path (redux tree (p0+p1)+(p2+p3)). Jq_pinv = Identity*2.0 keeps +0 off-diagonals. */
    const ok_quat q_inv = {-x[3], -x[4], -x[5], x[6]};
    double Q[16], Jp[12]; /* Jp 3x4 col-major */
    int i, j, k;
    for (k = 0; k < 42; ++k) J[k] = 0.0;
    J[0 * 7 + 0] = 1.0; J[1 * 7 + 1] = 1.0; J[2 * 7 + 2] = 1.0;
    ok_kin_oplus(&q_inv, Q);
    for (k = 0; k < 12; ++k) Jp[k] = 0.0;
    Jp[0 + 3 * 0] = 1.0 * 2.0; Jp[1 + 3 * 1] = 1.0 * 2.0; Jp[2 + 3 * 2] = 1.0 * 2.0;
    for (j = 0; j < 4; ++j)
        for (i = 0; i < 3; ++i) {
            const double p0 = Jp[i + 3 * 0] * M4(Q, 0, j), p1 = Jp[i + 3 * 1] * M4(Q, 1, j);
            const double p2 = Jp[i + 3 * 2] * M4(Q, 2, j), p3 = Jp[i + 3 * 3] * M4(Q, 3, j);
            double s;
            if (i < 2) s = ((p0 + p1) + p2) + p3;
            else s = (p0 + p1) + (p2 + p3);
            J[(3 + i) * 7 + (3 + j)] = s;
        }
    return 1;
}

int ok_hpoint_plus(const double x[4], const double delta[3], double out[4]) {
    out[0] = x[0] + delta[0];
    out[1] = x[1] + delta[1];
    out[2] = x[2] + delta[2];
    out[3] = x[3] + 0.0;
    return 1;
}

int ok_hpoint_plus_jacobian(const double x[4], double J[12]) {
    int k;
    (void)x;
    for (k = 0; k < 12; ++k) J[k] = 0.0;
    J[0 * 3 + 0] = 1.0; J[1 * 3 + 1] = 1.0; J[2 * 3 + 2] = 1.0;
    return 1;
}

int ok_hpoint_minus(const double y[4], const double x[4], double out[3]) {
    out[0] = y[0] - x[0];
    out[1] = y[1] - x[1];
    out[2] = y[2] - x[2];
    return 1;
}

int ok_hpoint_minus_jacobian(const double x[4], double J[12]) {
    int k;
    (void)x;
    for (k = 0; k < 12; ++k) J[k] = 0.0;
    J[0 * 4 + 0] = 1.0; J[1 * 4 + 1] = 1.0; J[2 * 4 + 2] = 1.0;
    return 1;
}

void ok_sab_plus(const double x[9], const double delta[9], double out[9]) {
    int i;
    for (i = 0; i < 9; ++i) out[i] = x[i] + delta[i];
}

void ok_sab_plus_jacobian(double J[81]) {
    int i;
    for (i = 0; i < 81; ++i) J[i] = 0.0;
    for (i = 0; i < 9; ++i) J[i * 9 + i] = 1.0;
}

void ok_sab_minus(const double x0[9], const double x0_plus_delta[9], double out[9]) {
    int i;
    for (i = 0; i < 9; ++i) out[i] = x0_plus_delta[i] - x0[i];
}

void ok_sab_minus_jacobian(double J[81]) { ok_sab_plus_jacobian(J); }

void ok_pose_block_set_estimate(double block[7], const ok_tf* T) {
    block[0] = T->r[0]; block[1] = T->r[1]; block[2] = T->r[2];
    block[3] = T->q.x; block[4] = T->q.y; block[5] = T->q.z; block[6] = T->q.w;
}

void ok_pose_block_estimate(const double block[7], ok_tf* T) { ok_tf_convert(T, block); }

void ok_pose_block_set_parameters(double block[7], const double params[7]) { memcpy(block, params, 7 * sizeof(double)); }

void ok_hpoint_block_from_v3(double block[4], const double p[3]) {
    block[0] = p[0]; block[1] = p[1]; block[2] = p[2]; block[3] = 1.0;
}
