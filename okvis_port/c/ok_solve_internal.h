/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Internal state shared by ok_solve.c (minimizer) and ok_solve_linear.c (linear solvers). See ok_solve.h. */
#ifndef OK_SOLVE_INTERNAL_H
#define OK_SOLVE_INTERNAL_H
#include "ok_solve.h"
#include "ok_sparse.h"

typedef struct ok_sv_cell { int block_id, arg, position; } ok_sv_cell;  /* block_id: reduced parameter index */
typedef struct ok_sv_row { int size, position, ncells; ok_sv_cell* cells; } ok_sv_row;

typedef struct ok_sv_dogleg_state {
    double radius, max_radius, min_diagonal, max_diagonal, mu, min_mu, max_mu, mu_increase_factor;
    double increase_threshold, decrease_threshold, dogleg_step_norm, alpha;
    int reuse, n, mu_increases, branch;
    double decrease_factor;        /* LevenbergMarquardtStrategy (strategy_lm): reuse = reuse_diagonal_, radius shared */
    double *diagonal, *gradient, *gauss_newton_step, *lm_diagonal, *scaled_gradient;
} ok_sv_dogleg_state;

typedef struct ok_sv_step_eval {
    int max_consecutive_nonmonotonic_steps, num_consecutive_nonmonotonic_steps;
    double minimum_cost, current_cost, reference_cost, candidate_cost;
    double accumulated_reference_model_cost_change, accumulated_candidate_model_cost_change;
} ok_sv_step_eval;

typedef struct ok_sv_state {
    ok_sv_problem* pb;
    const ok_sv_hooks* hooks;
    int *porder, *sorder, *rorder;  /* reduced program: original indices in program order */
    int np_red, nr_red;
    double fixed_cost;
    int num_eliminate_blocks;
    int num_parameters, num_effective, num_residuals, nnz;
    ok_sv_row* rows;
    ok_sv_cell* cells;
    double* values;                /* block-sparse Jacobian values */
    const double** state_ptr;
    double *scratch, *scratch_res, *scratch_res2, *scratch_grad;
    double *parameters, *x, *candidate_x, *projected_gradient_step, *residuals, *model_residuals, *Jg;
    double *gradient, *negative_gradient, *jacobian_scaling, *trust_region_step, *delta;
    double x_cost, candidate_cost, minimum_cost, model_cost_change, initial_cost, final_cost;
    int candidate_valid, delta_valid, step_valid, init_failed;
    int num_consecutive_invalid_steps, num_successful_steps, num_unsuccessful_steps, num_iterations, termination;
    ok_sv_iter it, last_iteration;
    ok_sv_dogleg_state dog;
    ok_sv_step_eval se;
    void* lin;                     /* ok_solve_linear.c state */
} ok_sv_state;

int ok_sv_eval_block(ok_sv_state* S, ok_sv_resid* rb, const double* const* params, double* cost, double* residuals,
                     double* const* jacobians);
void ok_sv_jac_right_multiply(const ok_sv_state* S, const double* x, double* y);
void ok_sv_jac_left_multiply(const ok_sv_state* S, const double* x, double* y);

void ok_sv_linear_init(ok_sv_state* S);
/* LinearSolver::Solve(jacobian, residuals, D, x): returns the LinearSolverTerminationType */
int ok_sv_linear_solve(ok_sv_state* S, const double* D, double* x);
void ok_sv_linear_free(ok_sv_state* S);

#endif
