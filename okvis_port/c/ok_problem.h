/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * OKVIS2 pure-C port, module 5c: the bookkeeping of ceres::Problem (Ceres Solver 2.2.0 internal/ceres/problem_impl.cc,
 * program.h, parameter_block.h, residual_block.h) that defines the PROGRAM ORDER of the parameter and residual blocks
 * -- the order the stable Schur ordering, the AMD block pattern and the Jacobian layout of module 4 depend on:
 * AddParameterBlock appends, RemoveParameterBlock / RemoveResidualBlock swap the last block into the hole
 * (DeleteBlockInVector), RemoveParameterBlock first removes every residual block that depends on the block
 * (enable_fast_removal: in the iteration order of a pointer-keyed std::unordered_set, which this port cannot
 * reproduce from the pointer values; it removes them in ascending program order instead and reports how many there
 * were, and the reference log records the real order so the replay can follow it), SetParameterBlockConstant /
 * Variable toggle the flag, SetManifold stores the manifold.
 *
 * Derived from Ceres Solver (http://ceres-solver.org), Copyright 2023 Google Inc. All rights reserved, BSD-3-Clause
 * (okvis_port/LICENSES/ceres-solver-BSD-3-Clause.txt): the names of Google Inc. and its contributors may not be used
 * to endorse derived products.
 *
 * C99, <stdint.h> <stdlib.h> <string.h> only. Blocks are identified by the user's double* (as a u64) and residual
 * blocks by their ResidualBlockId (a u64), so a reference log can be replayed verbatim.
 *
 * ---- problem.bin (patch 0009, OKVIS_PORT_GRAPH_DUMP_DIR; framed records u32 tag, u64 len, payload) ----
 *   P_NEW (1): u64 problem, u32 enable_fast_removal          ProblemImpl constructed
 *   P_DELETE (2): u64 problem                                 destroyed
 *   P_ADDPARAM (3): u64 problem, u64 values, u32 size         InternalAddParameterBlock (only when new)
 *   P_SETMANIFOLD (4): u64 problem, u64 values, u64 manifold  InternalSetManifold
 *   P_ADDRESID (5): u64 problem, u64 rb, u64 cost_function, u64 loss_function, u32 nb, u64 values[nb]
 *   P_RMRESID (6): u64 problem, u64 rb                        InternalRemoveResidualBlock (explicit and implicit)
 *   P_RMPARAM (7): u64 problem, u64 values, u32 ndeps         RemoveParameterBlock, after its dependents' P_RMRESID
 *   P_SETCONST (8) / P_SETVAR (9): u64 problem, u64 values
 *   P_SOLVE (10): u64 problem, u64 solve_index, u32 np, u32 nr, u64 fnv(program parameter pointers), u64 fnv(program
 *                 residual-block ids), u32 full, [u64 params[np], u64 rbs[nr]]   (every Solve(), before preprocessing)
 */
#ifndef OK_PROBLEM_H
#define OK_PROBLEM_H
#include <stdint.h>

enum { OK_P_NEW = 1, OK_P_DELETE = 2, OK_P_ADDPARAM = 3, OK_P_SETMANIFOLD = 4, OK_P_ADDRESID = 5, OK_P_RMRESID = 6,
       OK_P_RMPARAM = 7, OK_P_SETCONST = 8, OK_P_SETVAR = 9, OK_P_SOLVE = 10 };

#define OK_PB_MAXB 8

typedef struct ok_pb_param {
    uint64_t ptr, manifold;
    int size, constant, index, alive;
    int ndep, capdep; int* dep;      /* dependent residual slots (the fast-removal set) */
} ok_pb_param;
typedef struct ok_pb_resid {
    uint64_t ptr, cost, loss;
    int nb; int blk[OK_PB_MAXB];     /* parameter slots */
    int index, alive;
} ok_pb_resid;
typedef struct ok_u64map { uint64_t* keys; int* vals; unsigned char* state; int cap, n, used; } ok_u64map;
typedef struct ok_problem {
    ok_pb_param* params; int nparams, capparams; int* free_p; int nfree_p;
    ok_pb_resid* resids; int nresids, capresids; int* free_r; int nfree_r;
    int* porder; int np, capp;       /* program->parameter_blocks_: slots in program order */
    int* rorder; int nr, capr;       /* program->residual_blocks_ */
    ok_u64map pmap, rmap;            /* parameter_block_map_ (by pointer), residual_block_set_ (by id) */
} ok_problem;

void ok_problem_init(ok_problem* p);
void ok_problem_free(ok_problem* p);
int ok_problem_find_param(const ok_problem* p, uint64_t ptr);   /* slot or -1 */
int ok_problem_find_resid(const ok_problem* p, uint64_t ptr);
/* InternalAddParameterBlock: returns the slot (existing or new); -1 if the pointer exists with another size */
int ok_problem_add_parameter_block(ok_problem* p, uint64_t ptr, int size);
int ok_problem_set_manifold(ok_problem* p, uint64_t ptr, uint64_t manifold);        /* 1 ok, 0 unknown block */
/* AddResidualBlock (all parameter blocks must exist already; the pipeline always adds them first): slot or -1 */
int ok_problem_add_residual_block(ok_problem* p, uint64_t rb, uint64_t cost, uint64_t loss, int nb, const uint64_t* values);
int ok_problem_remove_residual_block(ok_problem* p, uint64_t rb);                   /* 1 ok, 0 unknown */
/* RemoveParameterBlock: removes the dependent residual blocks in ascending program order, then the block;
 * returns the number of dependents removed, -1 if unknown */
int ok_problem_remove_parameter_block(ok_problem* p, uint64_t ptr);
int ok_problem_set_constant(ok_problem* p, uint64_t ptr, int constant);             /* 1 ok, 0 unknown */
/* the program order: params[np] user pointers, rbs[nr] residual-block ids (buffers of at least np / nr) */
void ok_problem_program(const ok_problem* p, uint64_t* params, uint64_t* rbs);

#endif
