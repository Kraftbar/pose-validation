/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M4: the nonlinear least-squares solver RD-VIO runs through Ceres Solver 2.2.0
 * (rdvio::Solver::solve -> ::ceres::Solve): problem reduction and reordering, block-sparse Jacobian evaluation with the
 * quaternion manifold and the Cauchy loss, Jacobi scaling, the trust-region minimizer with the traditional Dogleg strategy
 * and the SPARSE_SCHUR linear solver (Schur elimination into a BlockRandomAccessSparseMatrix, Eigen SimplicialLDLT with the
 * Schur-complement columns pre-ordered by Eigen's AMD on the block pattern).
 *
 * Derived from (a) rd_solve.c / rd_solve_linear.c / rd_solve.h: modified copies of okvis_port/c/ok_solve.{c,h},
 * ok_solve_linear.c, ok_solve_internal.h (BSD-3-Clause; themselves derived from Ceres Solver 2.2.0, Copyright 2023 Google Inc.
 * All rights reserved, internal/ceres/{solver,program,reorder_program,parameter_block_ordering,graph_algorithms,
 * block_jacobian_writer,program_evaluator,residual_block,corrector,loss_function,trust_region_minimizer,
 * trust_region_step_evaluator,dogleg_strategy,schur_complement_solver,schur_eliminator_impl,detect_structure,
 * block_random_access_sparse_matrix,block_sparse_matrix,compressed_row_sparse_matrix,eigensparse}.{h,cc}, and OKVIS2,
 * Copyright (c) 2015 ETH Zurich ASL, 2020 Imperial College SRL, 2024 TUM SRL); (b) RD-VIO (Jianxff/rd_vio, Apache-2.0,
 * XRSLAM, Copyright 2022 XRSLAM Authors: solver.cpp options, Problem construction, factor classes); (c) Eigen 3.4.0
 * evaluation-order models reused by path from okvis_port/c (ok_blas.c, ok_dense.c, ok_sparse.c, ok_amd.c: BSD-3 / MPL-2.0).
 * See rdvio_port/NOTICE. Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> <limits.h> only. Jacobian blocks are ROW-major (Ceres).
 *
 * ---- Ceres configuration reproduced (rdvio_port/HANDOVER.md, "M4 Ceres configuration") ----
 * linear_solver_type = SPARSE_SCHUR with sparse_linear_algebra_library_type = EIGEN_SPARSE (the library default when only
 * Eigen sparse is compiled in) and linear_solver_ordering_type = AMD, trust_region_strategy_type = DOGLEG
 * (TRADITIONAL_DOGLEG), num_threads = 1, jacobi_scaling, update_state_every_iteration = true, max_num_iterations =
 * solver.iteration_limit (30), all tolerances at the Ceres defaults, no callbacks, no inner iterations.
 *
 * ---- solve.bin (patch 0006 + ceres_patches/0006; same framing and records as okvis_port/c/ok_solve.h) ----
 * Sequence of framed records: u32 tag, u64 payload length, payload (native endian; f64/u32/i32/u64).
 *   SOLVE (1)  : u64 solve_id, u32 level (1 snapshot/hashes, 2 + vectors), u32 linear_solver_type, trust_region_strategy_type,
 *                dogleg_type, max_num_iterations, num_threads, f64 function_tolerance, gradient_tolerance, parameter_tolerance,
 *                initial_trust_region_radius, max_trust_region_radius, min_trust_region_radius, min_relative_decrease,
 *                min_lm_diagonal, max_lm_diagonal, u32 jacobi_scaling, use_nonmonotonic_steps, max_num_consecutive_invalid_steps,
 *                linear_solver_ordering_type, sparse_linear_algebra_library_type, dense_linear_algebra_library_type,
 *                has_linear_solver_ordering, num_callbacks, use_inner_iterations, dynamic_sparsity, minimizer_type,
 *                preconditioner_type, use_explicit_schur_complement, max_num_refinement_iterations, use_mixed_precision_solves,
 *                update_state_every_iteration, f64 max_solver_time_in_seconds, u32 num_threads
 *   PROBLEM (2): u32 np; np x {u64 ptr, u32 size, u32 tangent, u32 kind (0 none, 1 quaternion manifold, 3 other), u32 constant,
 *                f64 x[size]} (pointer-sorted, as Problem::GetParameterBlocks); u32 nr; nr x {u64 ptr, u32 type, u32 loss (0 none,
 *                1 CauchyLoss(1), 2 other), u32 nb, u32 block_index[nb], u32 nres, u64 payload_len, payload}; the residual blocks
 *                are in program order. Payloads (f64):
 *                  vis = z[3] z_ref[3] cam_ref{q[4] p[3]} cam_tgt{q[4] p[3]} sqrt_inv_cov[4]   (this frame's keypoint, the reference
 *                        observation, the extrinsics of the reference and the target frame, Frame::sqrt_inv_cov column-major)
 *                  live = u32 n; n x {u64 ptr, u32 size, f64 v[size]}: memory the factor reads through Frame / Track pointers
 *                        (the USER STATE of a parameter block when ptr is one, else a constant of the snapshot)
 *                  1 ReprojectionError: vis                      blocks q_tgt p_tgt q_ref p_ref inv_depth
 *                  2 ReprojectionPrior: vis, live{q_ref, p_ref, inv_depth}   blocks q_tgt p_tgt
 *                  3 RotationPrior: z z_ref cam_ref cam_tgt sqrt_inv_cov, live{q_ref_center}   block q_tgt
 *                  4 PreIntegrationError: pre{t, q[4], p[3], v[3], dq_dbg dp_dbg dp_dba dv_dbg dv_dba (5x9), sqrt_inv_cov[225]}
 *                        imu_i{q[4] p[3]} imu_j{q[4] p[3]}, live{bg_i, ba_i}   blocks 10 (q p v bg ba) x 2
 *                  5 PreIntegrationPrior: as 4, live{q_i, p_i, v_i, bg_i, ba_i}   blocks 5 (frame j)
 *                  6 Marginalization: no payload (evaluated through the ORACLE records)
 *   PROGRAM (12): u32 np, u64 ptr[np], u32 nr, u64 ptr[nr]: program order at Solve() entry
 *   REDUCED (3) : u32 status, f64 fixed_cost, u32 num_eliminate_blocks, u32 linear_solver_type, u32 np_reduced, u64 ptr[np_reduced]
 *                (order after Schur ordering AND the AMD pre-ordering of the f blocks), u32 nr_reduced, u64 ptr[nr_reduced]
 *                (lexicographic residual order), u32 nremoved, u64 ptr[]
 *   ITER (4), DOGLEG (5), GN (6), SCHUR (7), END (11): as okvis_port/c/ok_solve.h
 *   SPARSE (9)  : the REDUCED SCHUR system handed to EigenSparseCholesky: u32 n, u32 nnz, u32 storage_type, u32 cholesky_storage_type,
 *                u64 fnv(values), fnv(rhs), fnv(rows), fnv(cols); level 2: i32 rows[n+1], i32 cols[nnz], f64 values[nnz], f64 rhs[n];
 *                u32 termination, u64 fnv(x); level 2 && success: f64 x[n]
 *   ORACLE (10) : raw outputs of the marginalization factor evaluations (as okvis_port/c/ok_solve.h)
 */
#ifndef RD_SOLVE_H
#define RD_SOLVE_H
#include <stdint.h>
#include <stdlib.h>
#include "rd_factor.h"
#include "rd_imu.h"

#define RD_SV_MAXB 64   /* the marginalization factor of a 12-frame window has 55 parameter blocks */

uint64_t rd_fnv(const void* p, size_t n);                 /* FNV-1a 64 */
uint64_t rd_fnv_combine(uint64_t h, uint64_t block_hash); /* h ^= block; h *= prime */

enum { RD_SV_T_UNKNOWN = 0, RD_SV_T_RPE = 1, RD_SV_T_RPP = 2, RD_SV_T_ROP = 3, RD_SV_T_PIE = 4, RD_SV_T_PIP = 5, RD_SV_T_MAR = 6 };
enum { RD_SV_KIND_NONE = 0, RD_SV_KIND_QUAT = 1, RD_SV_KIND_OTHER = 3 };
enum { RD_SV_LOSS_NONE = 0, RD_SV_LOSS_CAUCHY = 1, RD_SV_LOSS_OTHER = 2 };
/* ceres::LinearSolverType values */
enum { RD_SV_DENSE_NORMAL_CHOLESKY = 0, RD_SV_DENSE_QR = 1, RD_SV_SPARSE_NORMAL_CHOLESKY = 2, RD_SV_DENSE_SCHUR = 3,
       RD_SV_SPARSE_SCHUR = 4, RD_SV_ITERATIVE_SCHUR = 5, RD_SV_CGNR = 6 };
/* ceres::internal::LinearSolverTerminationType */
enum { RD_SV_LS_SUCCESS = 0, RD_SV_LS_NO_CONVERGENCE = 1, RD_SV_LS_FAILURE = 2, RD_SV_LS_FATAL_ERROR = 3 };
/* ceres::TerminationType */
enum { RD_SV_CONVERGENCE = 0, RD_SV_NO_CONVERGENCE = 1, RD_SV_FAILURE = 2, RD_SV_USER_SUCCESS = 3, RD_SV_USER_FAILURE = 4 };

typedef struct rd_sv_options {
    int linear_solver_type, max_num_iterations;
    double function_tolerance, gradient_tolerance, parameter_tolerance;
    double initial_trust_region_radius, max_trust_region_radius, min_trust_region_radius;
    double min_relative_decrease, min_lm_diagonal, max_lm_diagonal;
    int jacobi_scaling, max_num_consecutive_invalid_steps, update_state_every_iteration;
} rd_sv_options;

typedef struct rd_sv_param {
    uint64_t ptr;
    int size, tangent, kind, constant;
    double* x;                       /* USER state (owned by the problem) */
    int index, state_offset, delta_offset;  /* set by the reduced program */
    double plus_jacobian[12];        /* ambient x tangent, row-major: [I; 0] (4x3) for the quaternion manifold */
} rd_sv_param;

/* memory a factor reads outside its parameter arguments: user state of a parameter block (pidx >= 0) or a constant */
typedef struct rd_sv_live { uint64_t ptr; int size, pidx; double v[4]; } rd_sv_live;

typedef struct rd_sv_vis {
    double z[3], z_ref[3], sqrt_inv_cov[4];
    rd_extrinsic cam_ref, cam_tgt;
} rd_sv_vis;
typedef struct rd_sv_pie {
    rd_preint pre;
    ok_quat imu_i_q, imu_j_q;
    double imu_i_p[3], imu_j_p[3];
} rd_sv_pie;

typedef struct rd_sv_resid {
    uint64_t ptr;
    int type, loss, nb, nres;
    int blk[RD_SV_MAXB];             /* indices into rd_sv_problem.p */
    union { rd_sv_vis vis; rd_sv_pie pie; } term;
    int nlive;
    rd_sv_live live[5];
    int oracle;                      /* evaluated through rd_sv_hooks.oracle (marginalization factor) */
} rd_sv_resid;

typedef struct rd_sv_problem {
    int np;
    rd_sv_param* p;
    int nr;
    rd_sv_resid* r;
    rd_sv_options opt;
} rd_sv_problem;

/* Observation points (the solver calls them exactly where the Ceres patch writes the corresponding record). */
typedef struct rd_sv_iter {
    int iteration;
    double cost, cost_change, gradient_max_norm, gradient_norm, step_norm, relative_decrease, trust_region_radius;
    int step_is_valid, step_is_successful, step_is_nonmonotonic;
    double model_cost_change, candidate_cost, x_cost, minimum_cost;
    int num_parameters, num_effective_parameters, num_residuals;
    int candidate_valid, delta_valid, step_valid;
    const double *x, *candidate_x, *delta, *trust_region_step, *gradient, *jacobian_scaling, *residuals;
} rd_sv_iter;
typedef struct rd_sv_dogleg {
    int reuse, termination, n;
    double radius, mu, alpha, dogleg_step_norm, gradient_norm, gauss_newton_norm;
    const double *gradient, *gauss_newton_step, *diagonal, *step; /* step NULL when the linear solver failed */
} rd_sv_dogleg;
typedef struct rd_sv_gn {
    int termination, mu_increases, n;
    double mu;
    const double *lm_diagonal, *gauss_newton_step; /* gauss_newton_step NULL on failure */
} rd_sv_gn;
typedef struct rd_sv_schur { int row_block_size, e_block_size, f_block_size, num_eliminate_blocks, num_f_blocks,
                             one_f_block, num_col_blocks, num_row_blocks; } rd_sv_schur;
typedef struct rd_sv_end { int termination_type, num_iterations, num_successful_steps, num_unsuccessful_steps;
                           double initial_cost, final_cost, fixed_cost; uint64_t param_hash; int np; } rd_sv_end;

typedef struct rd_sv_hooks {
    void* ctx;
    /* evaluate a not-ported residual block (marginalization factor): fill residuals (nres) and the non-NULL jacobians
     * (nres x size row-major, ambient); return 1 on success */
    int (*oracle)(void* ctx, const rd_sv_resid* rb, const double* const* params, double* residuals,
                  double* const* jacobians);
    void (*on_reduced)(void* ctx, const rd_sv_problem* pb, double fixed_cost, int num_eliminate_blocks,
                       int np_reduced, const int* param_order, int nr_reduced, const int* resid_order);
    void (*on_iter)(void* ctx, const rd_sv_iter* it);
    void (*on_dogleg)(void* ctx, const rd_sv_dogleg* d);
    void (*on_gn)(void* ctx, const rd_sv_gn* g);
    void (*on_schur)(void* ctx, const rd_sv_schur* s);
    void (*on_sparse)(void* ctx, int n, int nnz, const int* rows, const int* cols, const double* values,
                      const double* rhs, int termination, const double* x);
    void (*on_end)(void* ctx, const rd_sv_end* e);
} rd_sv_hooks;

/* ::ceres::Solve(options, problem, summary) as configured by rdvio::Solver::solve. Updates the user state of the parameter
 * blocks in place (after every iteration with update_state_every_iteration, and at the end). Returns the TerminationType. */
int rd_sv_solve(rd_sv_problem* pb, const rd_sv_hooks* hooks);

#endif
