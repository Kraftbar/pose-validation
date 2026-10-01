/* SPDX-License-Identifier: MIT */
/* Standalone self-check, deliberately NOT named check_*.c so
 * tools/check_stella_port.py (which globs check_*.c and expects the
 * <seq_label> <fixtures_dir> <dump_dir> [max_frames] reference-dump
 * contract) does not sweep it up -- see the scope note below.
 * Build/run directly:
 *   gcc -std=c99 -O2 -o /tmp/chk sv_eigen_quaternion.c sv_g2o_se3.c \
 *       sv_g2o_edge.c sv_g2o_pose_optimizer.c sv_linalg.c \
 *       selfcheck_sv_g2o_pose_optimizer.c -lm && /tmp/chk
 *
 * Self-consistency checks for the module-4b g2o/pose_optimizer port.
 *
 * IMPORTANT SCOPE NOTE (see stella_port/HANDOVER.md module-4b entry):
 * these are NOT the tiered real-library validation the module asked for
 * (no reference dump tool captures real stella_vslam/g2o problems yet).
 * They check internal consistency only: analytic vs. central-difference
 * Jacobians, SE3 group/composition identities against direct matrix math,
 * Huber weighting behavior, and convergence of the LM+outlier-reclassify
 * loop on synthetic (noise+outlier-contaminated) perspective reprojection
 * problems built directly in this file, independent of stella/g2o headers.
 * A 0-mismatch run here is evidence the port is internally coherent, not
 * evidence it reproduces the real g2o build bit-exactly.
 */
#include "sv_g2o_pose_optimizer.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static int g_failures = 0;

static void check(int cond, const char* msg) {
    if (!cond) {
        printf("FAIL: %s\n", msg);
        g_failures++;
    }
}

static void rodrigues_rotate(const double axis[3], double theta, const double v[3], double out[3]) {
    /* Independent re-derivation of the rotation *value* (vector form,
     * v*cos + (axis x v)*sin + axis*(axis.v)*(1-cos)) for the test oracle
     * -- not required to match Eigen's evaluation order, only the math,
     * to validate sv_se3_exp's rotation against a second formula. */
    double n = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    double u[3] = {axis[0] / n, axis[1] / n, axis[2] / n};
    double c = cos(theta), s = sin(theta);
    double cross[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
    double dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    int k;
    for (k = 0; k < 3; ++k) {
        out[k] = v[k] * c + cross[k] * s + u[k] * dot * (1 - c);
    }
}

static void check_se3_exp_matches_rodrigues(void) {
    double update[6] = {0.1, -0.2, 0.05, 1.0, 2.0, -0.5};
    sv_se3 pose;
    sv_se3_exp(update, &pose);

    double axis[3] = {update[0], update[1], update[2]};
    double theta = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);

    double p[3] = {1.0, 0.0, 0.0};
    double got[3];
    sv_se3_map(&pose, p, got);

    double rotated[3];
    rodrigues_rotate(axis, theta, p, rotated);
    double expected[3] = {rotated[0] + pose.t[0], rotated[1] + pose.t[1], rotated[2] + pose.t[2]};

    double err = fabs(got[0] - expected[0]) + fabs(got[1] - expected[1]) + fabs(got[2] - expected[2]);
    check(err < 1e-9, "sv_se3_exp rotation matches independent Rodrigues formula");
}

static void check_quat_roundtrip(void) {
    /* quat -> matrix -> quat should be a fixed point (up to sign) for a
     * range of rotations, and quat_mul(q, q^-1)=identity. */
    double updates[3][6] = {
        {0.3, 0.1, -0.2, 0, 0, 0},
        {1.5, -0.7, 0.9, 0, 0, 0},
        {0.0001, -0.0002, 0.00005, 0, 0, 0},
    };
    int i;
    for (i = 0; i < 3; ++i) {
        sv_se3 pose;
        sv_se3_exp(updates[i], &pose);
        double m[9];
        sv_quat_to_mat3(&pose.q, m);
        sv_quat q2;
        sv_quat_from_mat3(m, &q2);
        double d1 = fabs(pose.q.x - q2.x) + fabs(pose.q.y - q2.y) + fabs(pose.q.z - q2.z) + fabs(pose.q.w - q2.w);
        double d2 = fabs(pose.q.x + q2.x) + fabs(pose.q.y + q2.y) + fabs(pose.q.z + q2.z) + fabs(pose.q.w + q2.w);
        check(d1 < 1e-9 || d2 < 1e-9, "quat -> matrix -> quat round trip (up to sign)");
    }
}

static void check_jacobian_numeric(void) {
    sv_pose_opt_edge e;
    e.pos_w[0] = 0.5;
    e.pos_w[1] = -0.3;
    e.pos_w[2] = 4.0;
    e.obs[0] = 320.0;
    e.obs[1] = 240.0;
    e.inv_sigma_sq = 1.0;
    e.fx = 517.3;
    e.fy = 516.5;
    e.cx = 318.6;
    e.cy = 255.3;
    e.huber_delta = sqrt(5.99146);
    e.use_robust_kernel = 0;
    e.level = 0;

    double base_update[6] = {0.05, -0.03, 0.02, 0.1, -0.2, 0.05};
    sv_se3 pose;
    sv_se3_exp(base_update, &pose);

    double jac[2][6];
    sv_pose_opt_edge_jacobian(&e, &pose, jac);

    double h = 1e-6;
    int k;
    double max_rel_err = 0.0;
    for (k = 0; k < 6; ++k) {
        double up[6] = {0, 0, 0, 0, 0, 0};
        up[k] = h;
        sv_se3 pose_plus, pose_minus;
        sv_shot_vertex_oplus(&pose, up, &pose_plus);
        up[k] = -h;
        sv_shot_vertex_oplus(&pose, up, &pose_minus);

        double e_plus[2], e_minus[2];
        sv_pose_opt_edge_error(&e, &pose_plus, e_plus);
        sv_pose_opt_edge_error(&e, &pose_minus, e_minus);

        double dcol[2];
        dcol[0] = (e_plus[0] - e_minus[0]) / (2 * h);
        dcol[1] = (e_plus[1] - e_minus[1]) / (2 * h);

        /* computeError() = obs - project(...), so d(error)/d(update) =
         * -jacobianOplus (linearizeOplus computes d(project)/d(update)
         * directly as the Jacobian g2o stores, and g2o's convention is
         * error = obs - h(x), chain rule flips sign relative to h's own
         * derivative -- but stella's linearizeOplus computes the Jacobian
         * of `error` directly per g2o's BaseEdge convention, so compare
         * to `jac` with a sign flip against the central-difference of
         * `error` itself, since jac IS d(error)/d(update) already). */
        double num0 = dcol[0];
        double num1 = dcol[1];
        double a0 = jac[0][k];
        double a1 = jac[1][k];
        double denom = fmax(1.0, fmax(fabs(num0), fabs(num1)));
        double rel = fmax(fabs(num0 - a0), fabs(num1 - a1)) / denom;
        if (rel > max_rel_err) {
            max_rel_err = rel;
        }
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "analytic Jacobian matches central-difference (max rel err %.3e)", max_rel_err);
    check(max_rel_err < 1e-5, buf);
    printf("  jacobian numeric check: max rel err = %.3e\n", max_rel_err);
}

static double frand(unsigned int* seed, double lo, double hi) {
    *seed = (*seed) * 1103515245u + 12345u;
    double u = ((*seed) >> 8) / (double)(1u << 24);
    return lo + u * (hi - lo);
}

static void check_ba_convergence_with_outliers(void) {
    /* Build a synthetic frame: true pose, N landmarks with known world
     * position, project them, perturb the pose, add gaussian-ish pixel
     * noise plus a handful of gross outliers, and check the port recovers
     * a pose close to ground truth and correctly flags the outliers. */
    const double fx = 517.3, fy = 516.5, cx = 318.6, cy = 255.3;
    sv_se3 true_pose;
    double true_update[6] = {0.05, 0.08, -0.03, 0.3, -0.1, 1.5};
    sv_se3_exp(true_update, &true_pose);

    const int N = 60;
    const int num_outliers = 8;
    sv_pose_opt_edge edges[60];
    unsigned int seed = 12345;

    int i;
    for (i = 0; i < N; ++i) {
        double pw[3] = {frand(&seed, -2, 2), frand(&seed, -2, 2), frand(&seed, 3, 8)};
        double pc[3];
        sv_se3_map(&true_pose, pw, pc);
        double proj[2];
        proj[0] = fx * pc[0] / pc[2] + cx;
        proj[1] = fy * pc[1] / pc[2] + cy;

        double noise_x = frand(&seed, -0.3, 0.3);
        double noise_y = frand(&seed, -0.3, 0.3);
        if (i < num_outliers) {
            noise_x += frand(&seed, 30, 60) * (i % 2 ? 1 : -1);
            noise_y += frand(&seed, 30, 60) * (i % 2 ? -1 : 1);
        }

        edges[i].pos_w[0] = pw[0];
        edges[i].pos_w[1] = pw[1];
        edges[i].pos_w[2] = pw[2];
        edges[i].obs[0] = proj[0] + noise_x;
        edges[i].obs[1] = proj[1] + noise_y;
        edges[i].inv_sigma_sq = 1.0;
        edges[i].fx = fx;
        edges[i].fy = fy;
        edges[i].cx = cx;
        edges[i].cy = cy;
        edges[i].level = 0;
    }

    /* Start from a perturbed pose, as stella's pose optimizer does
     * (initial estimate = last frame's pose / motion model, not GT). */
    sv_se3 init_pose;
    double perturb[6] = {-0.02, 0.03, 0.01, 0.15, 0.2, -0.1};
    sv_shot_vertex_oplus(&true_pose, perturb, &init_pose);

    sv_pose_optimizer_params params;
    params.num_trials_robust = 4;
    params.num_trials = 6;
    params.num_each_iter = 10;

    sv_se3 pose = init_pose;
    unsigned int num_inliers = sv_pose_optimizer_optimize(&pose, edges, N, &params);

    check(num_inliers == (unsigned int)(N - num_outliers), "outlier reclassification isolates exactly the injected outliers");
    for (i = 0; i < num_outliers; ++i) {
        check(edges[i].level == 1, "injected outlier flagged as outlier");
    }
    for (i = num_outliers; i < N; ++i) {
        check(edges[i].level == 0, "clean observation flagged as inlier");
    }

    /* Compare recovered pose to ground truth via a few test points. */
    double max_pos_err = 0.0;
    double test_pts[3][3] = {{0, 0, 5}, {1, -1, 4}, {-1, 1, 6}};
    for (i = 0; i < 3; ++i) {
        double got[3], want[3];
        sv_se3_map(&pose, test_pts[i], got);
        sv_se3_map(&true_pose, test_pts[i], want);
        double d = fabs(got[0] - want[0]) + fabs(got[1] - want[1]) + fabs(got[2] - want[2]);
        if (d > max_pos_err) {
            max_pos_err = d;
        }
    }
    printf("  BA convergence: %u/%d inliers, max mapped-point err = %.4e\n", num_inliers, N, max_pos_err);
    check(max_pos_err < 0.05, "recovered pose close to ground truth (mapped test points)");
}

int main(void) {
    check_se3_exp_matches_rodrigues();
    check_quat_roundtrip();
    check_jacobian_numeric();
    check_ba_convergence_with_outliers();

    if (g_failures == 0) {
        printf("OK: all sv_g2o_pose_optimizer self-checks passed\n");
        return 0;
    }
    printf("%d self-check failure(s)\n", g_failures);
    return 1;
}
