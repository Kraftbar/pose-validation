/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_g2o_pose_optimizer.h (BSD, g2o/stella_vslam-derived). */
#include "sv_g2o_pose_optimizer.h"
#include "sv_eigen_llt.h"
#include <math.h>
#include <string.h>

/* pose_optimizer_g2o.cc declares these as FLOAT (`constexpr float
 * chi_sq_2D = 5.99146; const float sqrt_chi_sq_2D = std::sqrt(chi_sq_2D);`),
 * not double -- `5.99146f` rounds differently than the double literal
 * `5.99146` (5.991459846496582... vs 5.99146 exactly), and sqrtf() of
 * that float is correctly-rounded IN FLOAT, not double-sqrt-then-widen.
 * Reproducing this exactly (via `double sqrt_chi_sq_2d = sqrtf(...)`
 * below) was the last piece closing check_sv_g2o_pose.c's residual --
 * every downstream chi2 threshold/Huber-weight computation depends on
 * this constant, so this tiny (~1e-7 relative) mismatch was silently
 * amplified over repeated LM iterations. See HANDOVER.md module-4b. */
#define CHI_SQ_2D ((double)5.99146f)
#define TAU 1e-5
#define GOOD_STEP_LOWER 0.3333333333333333
#define GOOD_STEP_UPPER 0.6666666666666666
#define MAX_TRIALS_AFTER_FAILURE 10
#define GAIN_THRESHOLD 1e-3

/* g2o::LinearSolverEigen<BlockSolver_6_3::PoseMatrixType>'s real solver
 * is Eigen::SimplicialLLT (NOT LDLT) + AMDOrdering, specialized to this
 * leaf's only real input shape (a single dense 6x6 pose block,
 * AMD-ordering == identity -- see sv_eigen_amd.h/sv_eigen_llt.h for the
 * measured justification and the upper-triangle source-value
 * convention). */
static int chol6_solve(const double H[6][6], const double b[6], double x[6]) {
    sv_llt6 f;
    sv_llt6_factorize(H, 6, &f);
    if (!f.ok) {
        return 0;
    }
    sv_llt6_solve(&f, b, x);
    return 1;
}

static double active_robust_chi2(const sv_se3* pose, const sv_pose_opt_edge* edges, int n) {
    double sum = 0.0;
    int i;
    for (i = 0; i < n; ++i) {
        if (edges[i].level != 0) {
            continue;
        }
        double error[2];
        sv_pose_opt_edge_error(&edges[i], pose, error);
        double chi2 = sv_pose_opt_edge_chi2(&edges[i], error);
        if (edges[i].use_robust_kernel) {
            double rho[3];
            sv_huber_robustify(edges[i].huber_delta, chi2, rho);
            sum += rho[0];
        } else {
            sum += chi2;
        }
    }
    return sum;
}

static void build_system(const sv_se3* pose, const sv_pose_opt_edge* edges, int n,
                          double H[6][6], double b[6]) {
    memset(H, 0, sizeof(double) * 36);
    memset(b, 0, sizeof(double) * 6);
    int i;
    for (i = 0; i < n; ++i) {
        sv_pose_opt_edge_accumulate(&edges[i], pose, H, b);
    }
}

/* One call to g2o::SparseOptimizer::optimize(num_each_iter) via
 * OptimizationAlgorithmLevenberg -- mutates *pose in place. */
static void lm_solve(sv_se3* pose, sv_pose_opt_edge* edges, int n, unsigned int num_each_iter) {
    double current_lambda = -1.0;
    double ni = 2.0;
    double last_chi = 0.0;
    unsigned int iter;

    for (iter = 0; iter < num_each_iter; ++iter) {
        double H[6][6], b[6];
        build_system(pose, edges, n, H, b);

        double current_chi = active_robust_chi2(pose, edges, n);

        if (iter == 0) {
            double max_diag = 0.0;
            int i;
            for (i = 0; i < 6; ++i) {
                if (fabs(H[i][i]) > max_diag) {
                    max_diag = fabs(H[i][i]);
                }
            }
            current_lambda = TAU * max_diag;
            ni = 2.0;
        }

        int qmax = 0;
        double rho = 0.0;
        do {
            double Hl[6][6];
            memcpy(Hl, H, sizeof(Hl));
            int i;
            for (i = 0; i < 6; ++i) {
                Hl[i][i] += current_lambda;
            }
            double dx[6];
            int ok = chol6_solve(Hl, b, dx);

            sv_se3 trial_pose;
            double temp_chi;
            if (ok) {
                sv_shot_vertex_oplus(pose, dx, &trial_pose);
                temp_chi = active_robust_chi2(&trial_pose, edges, n);
            } else {
                temp_chi = 1.0 / 0.0; /* +inf, matches numeric_limits::max() intent */
                memset(dx, 0, sizeof(dx));
                trial_pose = *pose;
            }

            rho = current_chi - temp_chi;
            double scale = 0.0;
            for (i = 0; i < 6; ++i) {
                scale += dx[i] * (current_lambda * dx[i] + b[i]);
            }
            scale += 1e-3;
            rho /= scale;

            if (rho > 0 && isfinite(temp_chi)) {
                double alpha = 1.0 - pow(2 * rho - 1, 3);
                if (alpha > GOOD_STEP_UPPER) {
                    alpha = GOOD_STEP_UPPER;
                }
                double scale_factor = (GOOD_STEP_LOWER > alpha) ? GOOD_STEP_LOWER : alpha;
                current_lambda *= scale_factor;
                ni = 2.0;
                current_chi = temp_chi;
                *pose = trial_pose;
            } else {
                current_lambda *= ni;
                ni *= 2.0;
                if (!isfinite(current_lambda)) {
                    break;
                }
            }
            qmax++;
        } while (rho < 0 && qmax < MAX_TRIALS_AFTER_FAILURE);

        /* OptimizationAlgorithmLevenberg::solve()'s own return value:
         *   if (qmax == _maxTrialsAfterFailure->value() || rho == 0 ||
         *       !g2o_isfinite(_currentLambda))
         *     return Terminate;
         *   return OK;
         * SparseOptimizer::optimize(iterations)'s for loop condition is
         * `i < iterations && !terminate() && ok` where
         * `ok = (result == OK)` -- so a Terminate result here stops the
         * OUTER iteration loop after this iteration's postIteration()
         * (gain-check) still runs once more, but no further iterations
         * are attempted. Missing this was a real bug: this port kept
         * iterating past the point real g2o stops (visible as extra
         * "iter=N" H/b/dx activity beyond where the real trace ends for
         * a given trial) -- see HANDOVER.md module-4b. */
        int should_terminate = (qmax == MAX_TRIALS_AFTER_FAILURE) || (rho == 0.0) || !isfinite(current_lambda);

        /* terminate_action: gain threshold 1e-3 on activeRobustChi2 at the
         * pose now in place (recomputed, matching its computeActiveErrors()
         * call at the top of operator()). */
        double chi_now = active_robust_chi2(pose, edges, n);
        if (iter == 0) {
            last_chi = chi_now;
        } else {
            double gain = (last_chi - chi_now) / chi_now;
            last_chi = chi_now;
            if (gain >= 0 && gain < GAIN_THRESHOLD) {
                break;
            }
        }

        if (should_terminate) {
            break;
        }
    }
}

unsigned int sv_pose_optimizer_optimize(sv_se3* pose, sv_pose_opt_edge* edges, int num_edges,
                                         const sv_pose_optimizer_params* params) {
    const double sqrt_chi_sq_2d = (double)sqrtf((float)CHI_SQ_2D);
    int i;

    if (num_edges < 5) {
        return 0;
    }

    const int use_robust_initially = params->num_trials_robust != 0;
    for (i = 0; i < num_edges; ++i) {
        edges[i].huber_delta = sqrt_chi_sq_2d;
        edges[i].use_robust_kernel = use_robust_initially;
        edges[i].level = 0;
    }

    unsigned int num_bad_obs = 0;
    unsigned int total_trials = params->num_trials_robust + params->num_trials;
    unsigned int trial;
    for (trial = 0; trial < total_trials; ++trial) {
        lm_solve(pose, edges, num_edges, params->num_each_iter);

        num_bad_obs = 0;
        for (i = 0; i < num_edges; ++i) {
            sv_pose_opt_edge* e = &edges[i];
            /* stella only recomputes error for edges that were previously
             * flagged outlier (level==1); level==0 edges already hold the
             * error from the last computeActiveErrors() inside lm_solve --
             * this port's error() is a pure function of `pose`, so
             * recomputing it here for every edge yields the same value. */
            double error[2];
            sv_pose_opt_edge_error(e, pose, error);
            double chi2 = sv_pose_opt_edge_chi2(e, error);
            if (chi2 > CHI_SQ_2D) {
                e->level = 1;
                ++num_bad_obs;
            } else {
                e->level = 0;
            }

            if (params->num_trials != 0 && trial + 1 == params->num_trials_robust) {
                e->use_robust_kernel = 0;
            }
        }

        if ((unsigned int)num_edges - num_bad_obs < 5) {
            break;
        }
    }

    return (unsigned int)num_edges - num_bad_obs;
}
