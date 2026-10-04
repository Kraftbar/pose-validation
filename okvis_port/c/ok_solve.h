/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 4: the nonlinear least-squares solver OKVIS2 runs through Ceres Solver 2.2.0
 * (okvis::ViGraph::optimise -> ::ceres::Solve): problem reduction and reordering, block-sparse Jacobian
 * evaluation with the OKVIS manifolds and the Cauchy loss, Jacobi scaling, the trust-region minimizer with the
 * traditional Dogleg strategy, the DENSE_SCHUR linear solver (Schur elimination + Eigen dense LLT, realtime
 * graph) and the SPARSE_NORMAL_CHOLESKY linear solver (J^T J + Eigen SimplicialLDLT, full/pose graph).
 *
 * Derived from Ceres Solver (http://ceres-solver.org), Copyright 2023 Google Inc. All rights reserved,
 * BSD-3-Clause (internal/ceres/{solver,program,reorder_program,parameter_block_ordering,graph_algorithms,
 * block_jacobian_writer,program_evaluator,residual_block,corrector,loss_function,trust_region_minimizer,
 * trust_region_step_evaluator,dogleg_strategy,schur_complement_solver,schur_eliminator_impl,detect_structure,
 * block_random_access_dense_matrix,dense_cholesky,invert_psd_matrix,sparse_normal_cholesky_solver,
 * inner_product_computer,block_sparse_matrix,eigensparse}.{h,cc}), and from OKVIS2 (okvis_ceres ViGraph::optimise,
 * ViSlamBackend::optimise{Realtime,Full}Graph, the manifolds and error terms of modules 1-3), BSD-3-Clause,
 * Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab / Imperial College London,
 * 2024 Smart Robotics Lab / Technical University of Munich. The Eigen 3.4.0 evaluation-order models
 * (ok_dense.c, ok_sparse.c, ok_eigen.c) are MPL-2.0. Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> <limits.h> only. Jacobian blocks are ROW-major (Ceres).
 *
 * ---- Ceres configuration reproduced (okvis_port/HANDOVER.md, "M4 Ceres configuration") ----
 * trust_region_strategy_type = DOGLEG (TRADITIONAL_DOGLEG), jacobi_scaling, num_threads = 1, no callbacks,
 * no bounds, no inner iterations; realtime graph: linear_solver_type = DENSE_SCHUR (max 10 iterations),
 * full graph: SPARSE_NORMAL_CHOLESKY with EIGEN_SPARSE + AMD (block ordering by Ceres, natural ordering inside
 * Eigen), function_tolerance 1e-3 (pre-pass, numIter/3 iterations) then 1e-6 (numIter iterations).
 *
 * ---- solve.bin (patch 0008, okvis_port/reference/patches/0008-solver-dump.patch) ----
 * Sequence of framed records: u32 tag, u64 payload length, payload (native endian; f64/u32/i32/u64).
 *   SOLVE (1)  : u64 solve_id, u32 level (0 summary, 1 +snapshot/hashes, 2 +vectors), u32 linear_solver_type,
 *                u32 trust_region_strategy_type, u32 dogleg_type, u32 max_num_iterations, u32 num_threads,
 *                f64 function_tolerance, gradient_tolerance, parameter_tolerance, initial_trust_region_radius,
 *                max_trust_region_radius, min_trust_region_radius, min_relative_decrease, min_lm_diagonal,
 *                max_lm_diagonal, u32 jacobi_scaling, u32 use_nonmonotonic_steps, u32 max_num_consecutive_invalid_steps,
 *                u32 linear_solver_ordering_type, u32 sparse_linear_algebra_library_type,
 *                u32 dense_linear_algebra_library_type, u32 has_linear_solver_ordering, u32 num_callbacks,
 *                u32 use_inner_iterations, u32 dynamic_sparsity, u32 minimizer_type, u32 preconditioner_type,
 *                u32 use_explicit_schur_complement, u32 max_num_refinement_iterations, u32 use_mixed_precision_solves
 *   PROBLEM (2): u32 np; np x {u64 ptr, u32 size, u32 tangent, u32 kind (0 none, 1 PoseManifold,
 *                2 HomogeneousPointManifold, 3 other), u32 constant, f64 x[size]};
 *                u32 nr; nr x {u64 ptr, u32 type, u32 loss (0 none, 1 CauchyLoss(1), 2 other), u32 nb,
 *                u32 block_index[nb], u32 nres, u64 payload_len, payload}
 *                type/payload: 1 ReprojectionError: cam header (ok_cam.h), f64 meas[2], f64 information[4] (row-major)
 *                              2 ImuError: SNAPSHOT (ok_imu.h, with measurements)
 *                              3 PoseError: f64 T coeffs[7], u32 6, f64 info[36], f64 sqrt_info[36] (row-major)
 *                              4 SpeedAndBiasError: f64 meas[9], u32 9, f64 info[81], f64 sqrt_info[81]
 *                              5 RelativePoseError: f64 T_AB coeffs[7], u32 6, f64 info[36], f64 sqrt_info[36]
 *                              6 HomogeneousPointError: f64 meas[4], u32 3, f64 info[9], f64 sqrt_info[9]
 *                              7/8 TwoPoseStandardGraphError(Const): u32 isComputed, f64 DeltaX[6], f64 J[36]
 *                                (column-major), f64 lin T_S0S1 (r[3], q xyzw) (patch 0009; empty before it)
 *                              9/10 TwoPoseExtrinsicsGraphError(Const): u32 isComputed, u32 n, f64 DeltaX[n],
 *                                f64 J[n*n] (column-major), f64 lin T_S0S1[7], u32 nextr, nextr x {u32 present,
 *                                f64 T_SC[7]} (patch 0009); 0 unknown: no payload.
 *                                Types 7-10 are evaluated natively (ok_twopose) when the payload is present and
 *                                their raw outputs are still recorded as ORACLE records (verification); without a
 *                                payload (dumps before patch 0009) they are replayed from the ORACLE records.
 *   PROGRAM (12): u32 np, u64 ptr[np], u32 nr, u64 ptr[nr]: the Problem's PROGRAM order of the parameter and
 *                residual blocks at Solve() entry (the PROBLEM record lists the parameter blocks in the order of
 *                Ceres' pointer-keyed ParameterMap; the program order is what the Schur ordering's stable sort and
 *                the AMD block pattern depend on)
 *   REDUCED (3): u32 status, f64 fixed_cost, u32 num_eliminate_blocks, u32 linear_solver_type, u32 np_reduced,
 *                u64 ptr[np_reduced] (order after reordering), u32 nr_reduced, u64 ptr[nr_reduced], u32 nremoved, u64 ptr[]
 *   ITER (4)   : u32 iteration, f64 cost, cost_change, gradient_max_norm, gradient_norm, step_norm,
 *                relative_decrease, trust_region_radius, u32 step_is_valid, u32 step_is_successful,
 *                u32 step_is_nonmonotonic, f64 model_cost_change, f64 candidate_cost (0 if never evaluated),
 *                f64 x_cost, f64 minimum_cost, u32 num_parameters, u32 num_effective_parameters, u32 num_residuals,
 *                u32 candidate_valid, u32 delta_valid, u32 step_valid;
 *                level >= 1: u64 fnv(x), fnv(candidate_x), fnv(delta), fnv(trust_region_step), fnv(gradient),
 *                fnv(jacobian_scaling), fnv(residuals) (0 where not valid);
 *                level >= 2: f64 x[np], gradient[ne], jacobian_scaling[ne], residuals[nres],
 *                [trust_region_step[ne]] [delta[ne]] [candidate_x[np]] (if valid)
 *   DOGLEG (5) : u32 reuse, u32 termination, f64 radius, mu, alpha, dogleg_step_norm, gradient_norm,
 *                gauss_newton_norm, u32 n, u64 fnv(gradient), fnv(gauss_newton_step), fnv(diagonal), fnv(step) (0 if
 *                the linear solver failed); level 2: f64 gradient[n], gauss_newton_step[n], diagonal[n], [step[n]]
 *   GN (6)     : u32 termination, u32 mu_increases, f64 mu, u32 n, u64 fnv(gauss_newton_step as solved, 0 on failure),
 *                u64 fnv(lm_diagonal); level 2: f64 lm_diagonal[n], [gauss_newton_step[n]]
 *   SCHUR (7)  : i32 row_block_size, i32 e_block_size, i32 f_block_size (-1 = dynamic), u32 num_eliminate_blocks,
 *                u32 num_f_blocks, u32 one_f_block_eliminator, u32 num_col_blocks, u32 num_row_blocks
 *   DENSE (8)  : u32 n, u64 fnv(lhs n*n row-major as built by the eliminator), u64 fnv(rhs); level 2: f64 lhs[n*n],
 *                f64 rhs[n]; u32 termination, u64 fnv(solution) (0 on failure); level 2 && success: f64 solution[n]
 *   SPARSE (9) : u32 n, u32 nnz, u32 storage_type, u32 cholesky_storage_type, u64 fnv(values), fnv(rhs), fnv(rows),
 *                fnv(cols); level 2: i32 rows[n+1], i32 cols[nnz], f64 values[nnz], f64 rhs[n]; u32 termination,
 *                u64 fnv(x); level 2 && success: f64 x[n]
 *   ORACLE (10): u64 residual_block_ptr, u64 fnv-combined hash of the parameter blocks, u32 have_jacobians,
 *                u32 nb, u32 jac_nonnull[nb], u32 nres, f64 residuals[nres], per non-null block f64 J[nres*size]
 *   END (11)   : u32 termination_type, u32 num_iterations, f64 initial_cost, f64 final_cost, f64 fixed_cost,
 *                u64 fnv-combined hash over all Problem parameter blocks, u32 np, u32 num_successful_steps,
 *                u32 num_unsuccessful_steps
 * fnv = 64-bit FNV-1a over the raw bytes (ok_fnv); the combined hashes are h ^= fnv(block); h *= prime.
 */
#ifndef OK_SOLVE_H
#define OK_SOLVE_H
#include <stdint.h>
#include <stdlib.h>
#include "ok_err.h"
#include "ok_imu.h"
#include "ok_param.h"
#include "ok_twopose.h"

#define OK_SV_MAXB 8

uint64_t ok_fnv(const void* p, size_t n);                 /* FNV-1a 64 */
uint64_t ok_fnv_combine(uint64_t h, uint64_t block_hash); /* h ^= block; h *= prime */

enum { OK_SV_T_UNKNOWN = 0, OK_SV_T_REPROJ = 1, OK_SV_T_IMU = 2, OK_SV_T_POSE = 3, OK_SV_T_SAB = 4,
       OK_SV_T_RELPOSE = 5, OK_SV_T_HPOINT = 6, OK_SV_T_TWOPOSE = 7, OK_SV_T_TWOPOSE_CONST = 8,
       OK_SV_T_TWOPOSE_EXT = 9, OK_SV_T_TWOPOSE_EXT_CONST = 10 };
enum { OK_SV_KIND_NONE = 0, OK_SV_KIND_POSE = 1, OK_SV_KIND_HPOINT = 2, OK_SV_KIND_OTHER = 3 };
enum { OK_SV_LOSS_NONE = 0, OK_SV_LOSS_CAUCHY = 1, OK_SV_LOSS_OTHER = 2 };
/* ceres::LinearSolverType values */
enum { OK_SV_DENSE_NORMAL_CHOLESKY = 0, OK_SV_DENSE_QR = 1, OK_SV_SPARSE_NORMAL_CHOLESKY = 2, OK_SV_DENSE_SCHUR = 3,
       OK_SV_SPARSE_SCHUR = 4, OK_SV_ITERATIVE_SCHUR = 5, OK_SV_CGNR = 6 };
/* ceres::internal::LinearSolverTerminationType (linear_solver.h) */
enum { OK_SV_LS_SUCCESS = 0, OK_SV_LS_NO_CONVERGENCE = 1, OK_SV_LS_FAILURE = 2, OK_SV_LS_FATAL_ERROR = 3 };
/* ceres::TerminationType */
enum { OK_SV_CONVERGENCE = 0, OK_SV_NO_CONVERGENCE = 1, OK_SV_FAILURE = 2, OK_SV_USER_SUCCESS = 3, OK_SV_USER_FAILURE = 4 };

typedef struct ok_sv_options {
    int linear_solver_type, max_num_iterations;
    double function_tolerance, gradient_tolerance, parameter_tolerance;
    double initial_trust_region_radius, max_trust_region_radius, min_trust_region_radius;
    double min_relative_decrease, min_lm_diagonal, max_lm_diagonal;
    int jacobi_scaling, max_num_consecutive_invalid_steps;
} ok_sv_options;

typedef struct ok_sv_param {
    uint64_t ptr;
    int size, tangent, kind, constant;
    double* x;                       /* user state (owned by the problem) */
    int index, state_offset, delta_offset;  /* set by the reduced program */
    double plus_jacobian[63];        /* ambient x tangent, row-major (7x6 or 4x3) */
} ok_sv_param;

typedef struct ok_sv_resid {
    uint64_t ptr;
    int type, loss, nb, nres;
    int blk[OK_SV_MAXB];             /* indices into ok_sv_problem.p */
    union {
        ok_reproj_err reproj;
        ok_imu_error imu;
        ok_pose_err pose;
        ok_sab_err sab;
        ok_relpose_err relpose;
        ok_hpoint_err hpoint;
        ok_tp_std tp;
        ok_tp_ext tpx;
    } term;
    int oracle;                      /* evaluated through ok_sv_hooks.oracle (no native term) */
} ok_sv_resid;

typedef struct ok_sv_problem {
    int np;
    ok_sv_param* p;
    int nr;
    ok_sv_resid* r;
    ok_sv_options opt;
} ok_sv_problem;

/* Observation points (the solver calls them exactly where patch 0008 writes the corresponding record). */
typedef struct ok_sv_iter {
    int iteration;
    double cost, cost_change, gradient_max_norm, gradient_norm, step_norm, relative_decrease, trust_region_radius;
    int step_is_valid, step_is_successful, step_is_nonmonotonic;
    double model_cost_change, candidate_cost, x_cost, minimum_cost;
    int num_parameters, num_effective_parameters, num_residuals;
    int candidate_valid, delta_valid, step_valid;
    const double *x, *candidate_x, *delta, *trust_region_step, *gradient, *jacobian_scaling, *residuals;
} ok_sv_iter;
typedef struct ok_sv_dogleg {
    int reuse, termination, n;
    double radius, mu, alpha, dogleg_step_norm, gradient_norm, gauss_newton_norm;
    const double *gradient, *gauss_newton_step, *diagonal, *step; /* step NULL when the linear solver failed */
} ok_sv_dogleg;
typedef struct ok_sv_gn {
    int termination, mu_increases, n;
    double mu;
    const double *lm_diagonal, *gauss_newton_step; /* gauss_newton_step NULL on failure */
} ok_sv_gn;
typedef struct ok_sv_schur { int row_block_size, e_block_size, f_block_size, num_eliminate_blocks, num_f_blocks,
                             one_f_block, num_col_blocks, num_row_blocks; } ok_sv_schur;
typedef struct ok_sv_end { int termination_type, num_iterations, num_successful_steps, num_unsuccessful_steps;
                           double initial_cost, final_cost, fixed_cost; uint64_t param_hash; int np; } ok_sv_end;

typedef struct ok_sv_hooks {
    void* ctx;
    /* evaluate a not-ported residual block (type 7-10 / 0): fill residuals (nres) and the non-NULL jacobians
     * (nres x size row-major, ambient); return 1 on success */
    int (*oracle)(void* ctx, const ok_sv_resid* rb, const double* const* params, double* residuals,
                  double* const* jacobians);
    /* observation of the raw outputs of a natively evaluated TwoPose* term (type 7-10 with payload): the harness
     * compares them with the ORACLE record of the reference */
    void (*on_term)(void* ctx, const ok_sv_resid* rb, const double* const* params, const double* residuals,
                    double* const* jacobians);
    void (*on_reduced)(void* ctx, const ok_sv_problem* pb, double fixed_cost, int num_eliminate_blocks,
                       int np_reduced, const int* param_order, int nr_reduced, const int* resid_order);
    void (*on_iter)(void* ctx, const ok_sv_iter* it);
    void (*on_dogleg)(void* ctx, const ok_sv_dogleg* d);
    void (*on_gn)(void* ctx, const ok_sv_gn* g);
    void (*on_schur)(void* ctx, const ok_sv_schur* s);
    void (*on_dense)(void* ctx, int n, const double* lhs, const double* rhs, int termination, const double* solution);
    void (*on_sparse)(void* ctx, int n, int nnz, const int* rows, const int* cols, const double* values,
                      const double* rhs, int termination, const double* x);
    void (*on_end)(void* ctx, const ok_sv_end* e);
} ok_sv_hooks;

/* ::ceres::Solve(options, problem, summary) as configured by ViGraph. Updates the parameter blocks in place
 * (CopyParameterBlockStateToUserState) and the ImuError states. Returns the TerminationType. */
int ok_sv_solve(ok_sv_problem* pb, const ok_sv_hooks* hooks);

#endif
