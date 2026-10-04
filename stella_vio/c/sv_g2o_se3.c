/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_g2o_se3.h (BSD, g2o/stella_vslam-derived). */
#include "sv_g2o_se3.h"
#include "sv_linalg.h"
#include <math.h>

static void mat3_skew(const double w[3], double out[9]) {
    /* column-major: out(row,col) = out[col*3+row] */
    out[0 * 3 + 0] = 0.0;
    out[0 * 3 + 1] = w[2];
    out[0 * 3 + 2] = -w[1];
    out[1 * 3 + 0] = -w[2];
    out[1 * 3 + 1] = 0.0;
    out[1 * 3 + 2] = w[0];
    out[2 * 3 + 0] = w[1];
    out[2 * 3 + 1] = -w[0];
    out[2 * 3 + 2] = 0.0;
}

/* out = I + s1*A + s2*B, elementwise, left-associative
 * ((I_ij + s1*A_ij) + s2*B_ij) matching the as-written C++ expression
 * `Matrix3::Identity() + s1*Omega + s2*Omega2` (plain CwiseBinaryOp
 * addition chain -- no reduction, so no row-0-1/row-2 packet split). */
static void mat3_id_plus_s1a_plus_s2b(double s1, const double a[9], double s2, const double b[9], double out[9]) {
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            double id = (r == c) ? 1.0 : 0.0;
            out[c * 3 + r] = (id + s1 * a[c * 3 + r]) + s2 * b[c * 3 + r];
        }
    }
}

/* SE3Quat::normalizeRotation(): sign-canonicalize (w>=0) then unit-
 * normalize -- called at the end of EVERY SE3Quat constructor that takes
 * a rotation (matrix or quaternion) and by operator*, so every rotation
 * this port produces must go through it too, not just sv_se3_exp's own
 * matrix->quaternion conversion. Bit-exact against real Eigen/g2o,
 * measured 0/1,000,000 (see stella_port/HANDOVER.md module-4b entry). */
void sv_se3_normalize_rotation(sv_se3* pose) {
    if (pose->q.w < 0.0) {
        pose->q.x = -pose->q.x;
        pose->q.y = -pose->q.y;
        pose->q.z = -pose->q.z;
        pose->q.w = -pose->q.w;
    }
    sv_quat_normalize(&pose->q);
}

void sv_se3_exp(const double update[6], sv_se3* out) {
    const double omega[3] = {update[0], update[1], update[2]};
    const double upsilon[3] = {update[3], update[4], update[5]};

    const double theta = sv_vec3_norm(omega);
    double Omega[9];
    mat3_skew(omega, Omega);
    double Omega2[9];
    sv_mat3_mul(Omega, Omega, Omega2);

    double R[9], V[9];
    if (theta < 0.00001) {
        mat3_id_plus_s1a_plus_s2b(1.0, Omega, 0.5, Omega2, R);
        mat3_id_plus_s1a_plus_s2b(0.5, Omega, 1.0 / 6.0, Omega2, V);
    } else {
        const double s = sin(theta);
        const double c = cos(theta);
        const double theta2 = theta * theta;
        const double a1 = s / theta;
        const double a2 = (1.0 - c) / theta2;
        const double b2 = (1.0 - c) / theta2;
        const double b3 = (theta - s) / (theta * theta * theta);
        mat3_id_plus_s1a_plus_s2b(a1, Omega, a2, Omega2, R);
        mat3_id_plus_s1a_plus_s2b(b2, Omega, b3, Omega2, V);
    }

    sv_quat_from_mat3(R, &out->q);
    sv_mat3_mulv(V, upsilon, out->t);
    /* SE3Quat::exp returns SE3Quat(Quaternion(R), V*upsilon), whose
     * constructor calls normalizeRotation(). */
    sv_se3_normalize_rotation(out);
}

void sv_se3_compose(const sv_se3* a, const sv_se3* b, sv_se3* out) {
    sv_quat q;
    sv_quat_mul(&a->q, &b->q, &q);
    double rotated_bt[3];
    sv_quat_map(&a->q, b->t, rotated_bt);
    out->q = q;
    out->t[0] = a->t[0] + rotated_bt[0];
    out->t[1] = a->t[1] + rotated_bt[1];
    out->t[2] = a->t[2] + rotated_bt[2];
    /* operator*(tr2) computes result._r *= tr2._r then
     * result.normalizeRotation() before returning. */
    sv_se3_normalize_rotation(out);
}

void sv_se3_map(const sv_se3* pose, const double p_w[3], double p_c[3]) {
    double rotated[3];
    sv_quat_map(&pose->q, p_w, rotated);
    p_c[0] = rotated[0] + pose->t[0];
    p_c[1] = rotated[1] + pose->t[1];
    p_c[2] = rotated[2] + pose->t[2];
}

void sv_shot_vertex_oplus(const sv_se3* old_pose, const double update[6], sv_se3* out) {
    sv_se3 delta;
    sv_se3_exp(update, &delta);
    sv_se3_compose(&delta, old_pose, out);
}

void sv_landmark_vertex_oplus(const double old_pos[3], const double update[3], double out[3]) {
    out[0] = old_pos[0] + update[0];
    out[1] = old_pos[1] + update[1];
    out[2] = old_pos[2] + update[2];
}
