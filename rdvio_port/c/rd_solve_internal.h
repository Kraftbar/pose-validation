/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Internal state shared by rd_solve.c (minimizer) and rd_solve_linear.c (linear solvers). See rd_solve.h. */
#ifndef RD_SOLVE_INTERNAL_H
#define RD_SOLVE_INTERNAL_H
#include "rd_solve.h"
#include "ok_sparse.h"

typedef struct rd_sv_cell { int block_id, arg, position; } rd_sv_cell;  /* block_id: reduced parameter index */
typedef struct rd_sv_row { int size, position, ncells; rd_sv_cell* cells; } rd_sv_row;

typedef struct rd_sv_dogleg_state {
    double radius, max_radius, min_diagonal, max_diagonal, mu, min_mu, max_mu, mu_increase_factor;
    double increase_threshold, decrease_threshold, dogleg_step_norm, alpha;
    int reuse, n, mu_increases, branch;
    double *diagonal, *gradient, *gauss_newton_step, *lm_diagonal, *scaled_gradient;
} rd_sv_dogleg_state;

typedef struct rd_sv_step_eval {
    int max_consecutive_nonmonotonic_steps, num_consecutive_nonmonotonic_steps;
    double minimum_cost, current_cost, reference_cost, candidate_cost;
    double accumulated_reference_model_cost_change, accumulated_candidate_model_cost_change;
} rd_sv_step_eval;

typedef struct rd_sv_state {
    rd_sv_problem* pb;
    const rd_sv_hooks* hooks;
    int *porder, *sorder, *rorder;  /* reduced program: original indices in program order */
    int np_red, nr_red;
    double fixed_cost;
    int num_eliminate_blocks;
    int num_parameters, num_effective, num_residuals, nnz;
    rd_sv_row* rows;
    rd_sv_cell* cells;
    double* values;                /* block-sparse Jacobian values */
    const double** state_ptr;
    double *scratch, *scratch_res, *scratch_res2, *scratch_grad;
    double *parameters, *x, *candidate_x, *projected_gradient_step, *residuals, *model_residuals, *Jg;
    double *gradient, *negative_gradient, *jacobian_scaling, *trust_region_step, *delta;
    double min_iter_cost, x_cost, candidate_cost, minimum_cost, model_cost_change, initial_cost, final_cost;
    int candidate_valid, delta_valid, step_valid, init_failed;
    int num_consecutive_invalid_steps, num_successful_steps, num_unsuccessful_steps, num_iterations, termination;
    rd_sv_iter it, last_iteration;
    rd_sv_dogleg_state dog;
    rd_sv_step_eval se;
    void* lin;                     /* rd_solve_linear.c state */
} rd_sv_state;

int rd_sv_eval_block(rd_sv_state* S, rd_sv_resid* rb, const double* const* params, double* cost, double* residuals,
                     double* const* jacobians);
void rd_sv_jac_right_multiply(const rd_sv_state* S, const double* x, double* y);
void rd_sv_jac_left_multiply(const rd_sv_state* S, const double* x, double* y);

void rd_sv_linear_init(rd_sv_state* S);
/* LinearSolver::Solve(jacobian, residuals, D, x): returns the LinearSolverTerminationType */
int rd_sv_linear_solve(rd_sv_state* S, const double* D, double* x);
void rd_sv_linear_free(rd_sv_state* S);

#endif
