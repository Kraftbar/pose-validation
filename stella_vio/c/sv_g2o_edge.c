/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_g2o_edge.h (BSD, g2o/stella_vslam-derived). */
#include "sv_g2o_edge.h"
#include <math.h>

void sv_pose_opt_edge_project(const sv_pose_opt_edge* e, const double pos_c[3], double out[2]) {
    out[0] = e->fx * pos_c[0] / pos_c[2] + e->cx;
    out[1] = e->fy * pos_c[1] / pos_c[2] + e->cy;
}

void sv_pose_opt_edge_error(const sv_pose_opt_edge* e, const sv_se3* pose, double error[2]) {
    double pos_c[3], proj[2];
    sv_se3_map(pose, e->pos_w, pos_c);
    sv_pose_opt_edge_project(e, pos_c, proj);
    error[0] = e->obs[0] - proj[0];
    error[1] = e->obs[1] - proj[1];
}

double sv_pose_opt_edge_chi2(const sv_pose_opt_edge* e, const double error[2]) {
    /* g2o::BaseEdge::chi2(): `_error.dot(information() * _error)` --
     * computes `information*error` first (per-component: isq*e_i, since
     * information is diagonal), THEN dot()s with error -- i.e.
     * (isq*e0)*e0 + (isq*e1)*e1, NOT isq*(e0*e0+e1*e1) (same class of
     * grouping bug as sv_pose_opt_edge_accumulate's H/b terms; see
     * HANDOVER.md). */
    const double ie0 = e->inv_sigma_sq * error[0];
    const double ie1 = e->inv_sigma_sq * error[1];
    return ie0 * error[0] + ie1 * error[1];
}

void sv_pose_opt_edge_jacobian(const sv_pose_opt_edge* e, const sv_se3* pose, double jac[2][6]) {
    double pos_c[3];
    sv_se3_map(pose, e->pos_w, pos_c);
    const double x = pos_c[0];
    const double y = pos_c[1];
    const double z = pos_c[2];
    const double z_sq = z * z;

    jac[0][0] = x * y / z_sq * e->fx;
    jac[0][1] = -(1.0 + (x * x / z_sq)) * e->fx;
    jac[0][2] = y / z * e->fx;
    jac[0][3] = -1.0 / z * e->fx;
    jac[0][4] = 0.0;
    jac[0][5] = x / z_sq * e->fx;

    jac[1][0] = (1.0 + y * y / z_sq) * e->fy;
    jac[1][1] = -x * y / z_sq * e->fy;
    jac[1][2] = -x / z * e->fy;
    jac[1][3] = 0.0;
    jac[1][4] = -1.0 / z * e->fy;
    jac[1][5] = y / z_sq * e->fy;
}

int sv_pose_opt_edge_depth_positive(const sv_pose_opt_edge* e, const sv_se3* pose) {
    double pos_c[3];
    sv_se3_map(pose, e->pos_w, pos_c);
    return pos_c[2] > 0.0;
}

void sv_huber_robustify(double delta, double chi2, double rho[3]) {
    const double dsqr = delta * delta;
    if (chi2 <= dsqr) {
        rho[0] = chi2;
        rho[1] = 1.0;
        rho[2] = 0.0;
    } else {
        const double sqrte = sqrt(chi2);
        rho[0] = 2.0 * sqrte * delta - dsqr;
        rho[1] = delta / sqrte;
        rho[2] = -0.5 * rho[1] / chi2;
    }
}

void sv_pose_opt_edge_accumulate(const sv_pose_opt_edge* e, const sv_se3* pose,
                                  double H[6][6], double b[6]) {
    if (e->level != 0) {
        return;
    }

    double error[2];
    sv_pose_opt_edge_error(e, pose, error);

    /* rho1: g2o's Gauss-Newton downweight (1.0 when the edge has no robust
     * kernel -- multiplying by 1.0 is exact in IEEE754, so reusing the
     * same formula below for both branches is bit-identical to g2o's
     * actual non-robust branch, which omits the rho1 factor entirely --
     * see BaseFixedSizedEdge::constructQuadraticForm's two branches). */
    double rho1 = 1.0;
    if (e->use_robust_kernel) {
        const double chi2 = sv_pose_opt_edge_chi2(e, error);
        double rho[3];
        sv_huber_robustify(e->huber_delta, chi2, rho);
        rho1 = rho[1];
    }

    double jac[2][6];
    sv_pose_opt_edge_jacobian(e, pose, jac);

    /* Replicates g2o's actual operation GROUPING, measured bit-exact
     * against real Eigen (0/500000 random cases,
     * `-O2 -DNDEBUG -ffp-contract=off -fno-fast-math -msse2`, matching
     * BaseFixedSizedEdge::constructQuadraticForm for a 2-dim edge/6-dim
     * unary vertex with isotropic information = inv_sigma_sq*I2):
     *   omega = rho1 * information         (InformationType, D x D)
     *   weightedError = -information*error; weightedError *= rho1
     *   AtO = J^T * omega                  (6x2)
     *   b += J^T * weightedError
     *   H += AtO * J
     * NOT the same as first collapsing to a scalar
     * s=inv_sigma_sq*rho1 and multiplying s into an already-summed
     * (jac0*jac0+jac1*jac1)/(jac0*e0+jac1*e1) -- that grouping is
     * mathematically equal but differs in the last bit or two (this was
     * the actual, measured source of check_sv_g2o_pose.c's ~1e-9
     * residual -- see HANDOVER.md). Because information is diagonal
     * (inv_sigma_sq*I2), omega's off-diagonal terms are exact zeros, so
     * AtO's two-term inner sum reduces to a single nonzero product --
     * still computed as `jac[k][r]*omega_diag` per k, matching the real
     * (6x2)*(2x2) product's per-entry two-term (one exact-zero) sum. */
    const double omega_diag = rho1 * e->inv_sigma_sq;
    double AtO0[6], AtO1[6];
    int r, c;
    for (r = 0; r < 6; ++r) {
        AtO0[r] = jac[0][r] * omega_diag;
        AtO1[r] = jac[1][r] * omega_diag;
    }
    const double we0 = (-(e->inv_sigma_sq * error[0])) * rho1;
    const double we1 = (-(e->inv_sigma_sq * error[1])) * rho1;
    for (r = 0; r < 6; ++r) {
        for (c = 0; c < 6; ++c) {
            H[r][c] += AtO0[r] * jac[0][c] + AtO1[r] * jac[1][c];
        }
        b[r] += jac[0][r] * we0 + jac[1][r] * we1;
    }
}
