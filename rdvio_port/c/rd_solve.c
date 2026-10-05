/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause AND MPL-2.0 */
/* See rd_solve.h (modified copy of okvis_port/c/ok_solve.c; Ceres Solver 2.2.0, BSD-3-Clause, Copyright 2023 Google Inc.;
 * OKVIS2 BSD-3-Clause; RD-VIO Apache-2.0; Eigen evaluation-order models MPL-2.0).
 * Part 1: problem reduction, reordering, evaluator, minimizer, Dogleg. */
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "ok_blas.h"
#include "ok_dense.h"
#include "rd_solve_internal.h"

uint64_t rd_fnv(const void* p, size_t n) {
    const unsigned char* c = (const unsigned char*)p;
    uint64_t h = 1469598103934665603ULL;
    size_t i;
    for (i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
    return h;
}
uint64_t rd_fnv_combine(uint64_t h, uint64_t block_hash) { h ^= block_hash; h *= 1099511628211ULL; return h; }

static void* xcalloc(size_t n, size_t sz) { void* p = calloc(n ? n : 1, sz); return p; }

/* ------------------------------------------------------------------------------------------------------------
 * Program::CreateReducedProgram / RemoveFixedBlocks: residual blocks whose parameter blocks are all constant are
 * evaluated once into fixed_cost and dropped; parameter blocks that are constant or unreferenced are dropped.
 * Order is preserved. */
static int reduce_program(rd_sv_state* S) {
    rd_sv_problem* pb = S->pb;
    int i, k, n_active = 0;
    double fixed = 0.0;
    for (i = 0; i < pb->np; ++i) pb->p[i].index = -1;
    S->nr_red = 0;
    for (i = 0; i < pb->nr; ++i) {
        rd_sv_resid* rb = &pb->r[i];
        int all_constant = 1;
        for (k = 0; k < rb->nb; ++k)
            if (!pb->p[rb->blk[k]].constant) { all_constant = 0; pb->p[rb->blk[k]].index = 1; }
        if (!all_constant) { S->rorder[S->nr_red++] = i; continue; }
        {   /* residual_block->Evaluate(true, &cost, nullptr, nullptr, scratch): cost only, loss applied */
            double cost;
            if (!rd_sv_eval_block(S, rb, NULL, &cost, NULL, NULL)) return 0;
            fixed += cost;
        }
    }
    for (i = 0; i < pb->np; ++i)
        if (pb->p[i].index != -1) S->porder[n_active++] = i;
    S->np_red = n_active;
    S->fixed_cost = fixed;
    return 1;
}

/* Program::SetParameterOffsetsAndIndex on the reduced program */
static void set_offsets(rd_sv_state* S) {
    int i, so = 0, dofs = 0;
    for (i = 0; i < S->pb->np; ++i) S->pb->p[i].index = -1;
    for (i = 0; i < S->np_red; ++i) {
        rd_sv_param* p = &S->pb->p[S->porder[i]];
        p->index = i;
        p->state_offset = so;
        p->delta_offset = dofs;
        so += p->size;
        dofs += p->tangent;
    }
    S->num_parameters = so;
    S->num_effective = dofs;
}

/* ---- ComputeStableSchurOrdering (parameter_block_ordering.cc + graph_algorithms.h) ---- */
/* Hessian graph: vertices = reduced parameter blocks, edge between two blocks sharing a residual block.
 * Degree = number of distinct neighbours. */
static int schur_ordering(rd_sv_state* S) {
    const int n = S->np_red;
    rd_sv_problem* pb = S->pb;
    int* deg = (int*)xcalloc((size_t)n, sizeof(int));
    unsigned char* adj = NULL;  /* n x n adjacency (dense; n is small for the realtime graph) */
    int* queue = (int*)xcalloc((size_t)n, sizeof(int));
    char* color = (char*)xcalloc((size_t)n, 1);
    int i, j, k, m, cnt = 0, independent = 0;
    size_t nn = (size_t)n * (size_t)n;
    adj = (unsigned char*)xcalloc(nn, 1);
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_resid* rb = &pb->r[S->rorder[i]];
        for (j = 0; j < rb->nb; ++j) {
            const int a = pb->p[rb->blk[j]].index;
            if (a < 0) continue;
            for (k = j + 1; k < rb->nb; ++k) {
                const int b = pb->p[rb->blk[k]].index;
                if (b < 0 || a == b) continue;
                if (!adj[(size_t)a * n + b]) {
                    adj[(size_t)a * n + b] = adj[(size_t)b * n + a] = 1;
                    deg[a]++; deg[b]++;
                }
            }
        }
    }
    /* vertex_queue = reduced order, stable_sort by degree (counting sort by degree, stable) */
    {
        int maxd = 0;
        int* head;
        for (i = 0; i < n; ++i) if (deg[i] > maxd) maxd = deg[i];
        head = (int*)xcalloc((size_t)maxd + 2, sizeof(int));
        for (i = 0; i < n; ++i) head[deg[i] + 1]++;
        for (i = 0; i <= maxd; ++i) head[i + 1] += head[i];
        for (i = 0; i < n; ++i) queue[head[deg[i]]++] = i;
        free(head);
    }
    /* greedy independent set in queue order; grey the neighbours */
    for (m = 0; m < n; ++m) {
        const int v = queue[m];
        if (color[v] != 0) continue;
        S->sorder[cnt++] = v;
        color[v] = 2;
        for (j = 0; j < n; ++j) if (adj[(size_t)v * n + j]) color[j] = 1;
    }
    independent = cnt;
    for (m = 0; m < n; ++m) if (color[queue[m]] != 2) S->sorder[cnt++] = queue[m];
    /* swap(program.parameter_blocks, schur_ordering): sorder holds reduced indices -> new porder */
    {
        int* newp = (int*)xcalloc((size_t)n, sizeof(int));
        for (i = 0; i < n; ++i) newp[i] = S->porder[S->sorder[i]];
        memcpy(S->porder, newp, sizeof(int) * (size_t)n);
        free(newp);
    }
    free(deg); free(adj); free(queue); free(color);
    return independent;
}

/* LexicographicallyOrderResidualBlocks: bucket by the smallest parameter-block index (e-blocks first); each
 * bucket is filled from its end while scanning forwards, i.e. the order WITHIN a bucket is reversed. */
static void lexicographic_residual_order(rd_sv_state* S, int size_of_first_group) {
    const int nr = S->nr_red;
    int* bucket_of = (int*)xcalloc((size_t)nr, sizeof(int));
    int* offsets = (int*)xcalloc((size_t)size_of_first_group + 2, sizeof(int));
    int* out = (int*)xcalloc((size_t)nr, sizeof(int));
    int i, k;
    for (i = 0; i < nr; ++i) {
        const rd_sv_resid* rb = &S->pb->r[S->rorder[i]];
        int pos = size_of_first_group;
        for (k = 0; k < rb->nb; ++k) {
            const rd_sv_param* p = &S->pb->p[rb->blk[k]];
            if (!p->constant && p->index < pos) pos = p->index;
        }
        bucket_of[i] = pos;
        offsets[pos + 1]++;  /* counts, shifted: offsets[b+1] = count(b) */
    }
    /* partial_sum of counts -> offsets[b] = end of bucket b */
    for (k = 0; k <= size_of_first_group; ++k) offsets[k + 1] += offsets[k];
    for (k = 0; k <= size_of_first_group; ++k) offsets[k] = offsets[k + 1];  /* offsets[b] = cumulative incl. b */
    for (i = 0; i < nr; ++i) {
        const int b = bucket_of[i];
        offsets[b]--;
        out[offsets[b]] = S->rorder[i];
    }
    memcpy(S->rorder, out, sizeof(int) * (size_t)nr);
    free(bucket_of); free(offsets); free(out);
}

/* ---- ReorderSchurComplementColumnsUsingEigen (reorder_program.cc, SPARSE_SCHUR + EIGEN_SPARSE + AMD) ----
 * block_schur_complement = F^T F - F^T E E^T F on the 0/1 BLOCK Jacobian (E = the first ne block columns, F the rest; rows =
 * residual blocks of the reduced program). Eigen keeps explicit zeros of the difference, so the stored pattern is the union
 * of the patterns of F^T F and F^T E E^T F (all counts are positive): two f blocks are linked when they share a residual
 * block (F^T F) or are both linked, through any residual blocks, to the same e block. AMDOrdering then gives perm; the f
 * parameter blocks are re-ordered as parameter_blocks[ne + i] = old[ne + perm[i]]; the e blocks keep their order. */
static void schur_complement_ordering(rd_sv_state* S, int ne) {
    const int nf = S->np_red - ne, n = S->np_red;
    int i, j, k, e, nnz = 0;
    unsigned char* adj;
    unsigned char* mark;
    int *Ap, *Ai, *perm, *newp;
    if (nf <= 0) return;
    adj = (unsigned char*)xcalloc((size_t)nf * (size_t)nf, 1);
    mark = (unsigned char*)xcalloc((size_t)(ne ? ne : 1) * (size_t)nf, 1);
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_resid* rb = &S->pb->r[S->rorder[i]];
        int ebl = -1;
        for (j = 0; j < rb->nb; ++j) {
            const int a = S->pb->p[rb->blk[j]].index;
            if (a >= 0 && a < ne) ebl = a;
        }
        for (j = 0; j < rb->nb; ++j) {
            const int a = S->pb->p[rb->blk[j]].index - ne;
            if (a < 0 || S->pb->p[rb->blk[j]].index < 0) continue;
            for (k = 0; k < rb->nb; ++k) {
                const int b = S->pb->p[rb->blk[k]].index - ne;
                if (b >= 0 && S->pb->p[rb->blk[k]].index >= 0) adj[(size_t)a * nf + b] = 1;   /* F^T F */
            }
            if (ebl >= 0) mark[(size_t)ebl * nf + a] = 1;
        }
    }
    for (e = 0; e < ne; ++e)   /* F^T E E^T F: f blocks sharing the e block */
        for (j = 0; j < nf; ++j)
            if (mark[(size_t)e * nf + j])
                for (k = 0; k < nf; ++k) if (mark[(size_t)e * nf + k]) adj[(size_t)j * nf + k] = 1;
    for (i = 0; i < nf * nf; ++i) nnz += adj[i];
    Ap = (int*)xcalloc((size_t)nf + 1, sizeof(int));
    Ai = (int*)xcalloc((size_t)nnz + 1, sizeof(int));
    perm = (int*)xcalloc((size_t)nf, sizeof(int));
    nnz = 0;
    for (j = 0; j < nf; ++j) {  /* CSC, sorted row indices */
        Ap[j] = nnz;
        for (i = 0; i < nf; ++i) if (adj[(size_t)i * nf + j]) Ai[nnz++] = i;
    }
    Ap[nf] = nnz;
    ok_amd_order(nf, Ap, Ai, perm);
    newp = (int*)xcalloc((size_t)n, sizeof(int));
    for (i = 0; i < ne; ++i) newp[i] = S->porder[i];
    for (i = 0; i < nf; ++i) newp[ne + i] = S->porder[ne + perm[i]];
    memcpy(S->porder, newp, sizeof(int) * (size_t)n);
    free(adj); free(mark); free(Ap); free(Ai); free(perm); free(newp);
}

/* ---- BlockJacobianWriter::CreateJacobian: cells per row sorted by parameter block index ---- */
static void build_jacobian_structure(rd_sv_state* S) {
    int i, k, nres_total = 0, ncells = 0, nnz = 0;
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_resid* rb = &S->pb->r[S->rorder[i]];
        for (k = 0; k < rb->nb; ++k) if (S->pb->p[rb->blk[k]].index >= 0) ncells++;
    }
    S->rows = (rd_sv_row*)xcalloc((size_t)S->nr_red, sizeof(rd_sv_row));
    S->cells = (rd_sv_cell*)xcalloc((size_t)ncells, sizeof(rd_sv_cell));
    ncells = 0;
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_resid* rb = &S->pb->r[S->rorder[i]];
        rd_sv_row* row = &S->rows[i];
        int c, j;
        row->size = rb->nres;
        row->position = nres_total;
        nres_total += rb->nres;
        row->cells = &S->cells[ncells];
        row->ncells = 0;
        for (k = 0; k < rb->nb; ++k) {
            const int bi = S->pb->p[rb->blk[k]].index;
            if (bi < 0) continue;
            row->cells[row->ncells].block_id = bi;
            row->cells[row->ncells].arg = k;
            row->ncells++;
        }
        /* std::sort by block_id (CellLessThan); block ids within a row are distinct */
        for (c = 1; c < row->ncells; ++c) {
            rd_sv_cell t = row->cells[c];
            for (j = c - 1; j >= 0 && row->cells[j].block_id > t.block_id; --j) row->cells[j + 1] = row->cells[j];
            row->cells[j + 1] = t;
        }
        for (c = 0; c < row->ncells; ++c) {
            row->cells[c].position = nnz;
            nnz += rb->nres * S->pb->p[S->porder[row->cells[c].block_id]].tangent;
        }
        ncells += row->ncells;
    }
    S->num_residuals = nres_total;
    S->nnz = nnz;
    S->values = (double*)xcalloc((size_t)nnz, sizeof(double));
}

/* ---------------------------------------------------- evaluation ------------------------------------------ */
/* the memory a factor reads through Frame / Track pointers: the USER state of a parameter block (it only changes between
 * iterations, see update_user_state) or a constant of the snapshot */
static const double* live_ptr(const rd_sv_state* S, const rd_sv_resid* rb, int k) {
    return rb->live[k].pidx >= 0 ? S->pb->p[rb->live[k].pidx].x : rb->live[k].v;
}

/* CostFunction::Evaluate of one residual block with the RD-VIO factors (ambient Jacobians, row-major). */
static int cost_function_evaluate(rd_sv_state* S, rd_sv_resid* rb, const double* const* params, double* res,
                                  double* const* jac) {
    switch (rb->type) {
        case RD_SV_T_RPE: {
            const rd_sv_vis* v = &rb->term.vis;
            rd_rpe_eval(v->z, v->z_ref, &v->cam_ref, &v->cam_tgt, v->sqrt_inv_cov, params, res, (double**)jac);
            return 1;
        }
        case RD_SV_T_RPP: {   /* CeresReprojectionPriorFactor: (q_tgt, p_tgt) + the live reference pose and inverse depth */
            const rd_sv_vis* v = &rb->term.vis;
            const double* p5[5];
            double* j5[5] = {NULL, NULL, NULL, NULL, NULL};
            p5[0] = params[0]; p5[1] = params[1];
            p5[2] = live_ptr(S, rb, 0); p5[3] = live_ptr(S, rb, 1); p5[4] = live_ptr(S, rb, 2);
            if (jac) { j5[0] = jac[0]; j5[1] = jac[1]; }
            rd_rpe_eval(v->z, v->z_ref, &v->cam_ref, &v->cam_tgt, v->sqrt_inv_cov, p5, res, jac ? j5 : NULL);
            return 1;
        }
        case RD_SV_T_ROP: {
            const rd_sv_vis* v = &rb->term.vis;
            ok_quat q_ref;
            const double* l = live_ptr(S, rb, 0);
            q_ref.x = l[0]; q_ref.y = l[1]; q_ref.z = l[2]; q_ref.w = l[3];
            rd_rot_prior_eval(v->z, v->z_ref, &v->cam_ref, &v->cam_tgt, v->sqrt_inv_cov, &q_ref, params[0], res, jac ? jac[0] : NULL);
            return 1;
        }
        case RD_SV_T_PIE: {   /* bg_i_0 / ba_i_0 are read from the live user state of frame i */
            const rd_sv_pie* e = &rb->term.pie;
            rd_pie_eval(&e->pre, &e->imu_i_q, e->imu_i_p, &e->imu_j_q, e->imu_j_p, live_ptr(S, rb, 0), live_ptr(S, rb, 1), params, res, (double**)jac);
            return 1;
        }
        case RD_SV_T_PIP: {   /* CeresPreIntegrationPriorFactor: frame i from the live user state, frame j = the arguments */
            const rd_sv_pie* e = &rb->term.pie;
            const double* p10[10];
            double* j10[10];
            int k;
            for (k = 0; k < 5; ++k) { p10[k] = live_ptr(S, rb, k); p10[5 + k] = params[k]; j10[k] = NULL; j10[5 + k] = jac ? jac[k] : NULL; }
            rd_pie_eval(&e->pre, &e->imu_i_q, e->imu_i_p, &e->imu_j_q, e->imu_j_p, p10[3], p10[4], p10, res, jac ? j10 : NULL);
            return 1;
        }
        case RD_SV_T_MAR: {   /* CeresMarginalizationFactor::Evaluate (module M5) */
            const int ok = rb->marg ? rd_marg_eval(rb->marg, params, res, jac) : 0;
            if (ok && S->hooks && S->hooks->on_marg) S->hooks->on_marg(S->hooks->ctx, rb, params, res, jac);
            return ok;
        }
        default:
            if (S->hooks && S->hooks->oracle) return S->hooks->oracle(S->hooks->ctx, rb, params, res, jac);
            return 0;
    }
}

/* CauchyLoss(a = 1): b = a^2, c = 1/b */
static void cauchy_evaluate(double s, double rho[3]) {
    const double b = 1.0, c = 1.0;
    const double sum = 1.0 + s * c;
    const double inv = 1.0 / sum;
    rho[0] = b * log(sum);
    rho[1] = inv > 2.2250738585072014e-308 ? inv : 2.2250738585072014e-308;  /* max(DBL_MIN, inv) */
    rho[2] = -c * (inv * inv);
}

/* ResidualBlock::Evaluate(apply_loss_function = true, cost, residuals, jacobians, scratch).
 * params: state pointers per block; residuals may be NULL (cost only; scratch is used internally);
 * jacobians: NULL, or per block NULL / the row-major nres x tangent destination. */
int rd_sv_eval_block(rd_sv_state* S, rd_sv_resid* rb, const double* const* params_in, double* cost,
                     double* residuals, double* const* jacobians) {
    const int nres = rb->nres;
    const double* params[RD_SV_MAXB];
    double* global_jac[RD_SV_MAXB];
    double* scratch = S->scratch;
    double* res = residuals ? residuals : S->scratch_res;
    double sq;
    int k;
    for (k = 0; k < rb->nb; ++k) params[k] = params_in ? params_in[k] : S->pb->p[rb->blk[k]].x;
    for (k = 0; k < rb->nb; ++k) {
        const rd_sv_param* p = &S->pb->p[rb->blk[k]];
        global_jac[k] = NULL;
        if (jacobians && jacobians[k]) {
            if (p->kind != RD_SV_KIND_NONE) { global_jac[k] = scratch; scratch += nres * p->size; }
            else global_jac[k] = jacobians[k];
        }
    }
    if (!cost_function_evaluate(S, rb, params, res, jacobians ? global_jac : NULL)) return 0;
    sq = ok_dyn_sqnorm(res, nres);
    if (jacobians) {
        for (k = 0; k < rb->nb; ++k) {
            const rd_sv_param* p = &S->pb->p[rb->blk[k]];
            if (jacobians[k] && p->kind != RD_SV_KIND_NONE)
                ok_mmm(global_jac[k], nres, p->size, p->plus_jacobian, p->size, p->tangent, jacobians[k], 0, 0,
                       p->tangent, 0);
        }
    }
    if (rb->loss == RD_SV_LOSS_NONE) { *cost = 0.5 * sq; return 1; }
    {
        double rho[3], sqrt_rho1, residual_scaling, alpha_sq_norm;
        int i;
        cauchy_evaluate(sq, rho);
        *cost = 0.5 * rho[0];
        if (!jacobians && !residuals) return 1;
        /* Corrector */
        sqrt_rho1 = sqrt(rho[1]);
        if (sq == 0.0 || rho[2] <= 0.0) { residual_scaling = sqrt_rho1; alpha_sq_norm = 0.0; }
        else {
            const double D = 1.0 + 2.0 * sq * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling = sqrt_rho1 / (1 - alpha);
            alpha_sq_norm = alpha / sq;
        }
        if (jacobians) {
            for (k = 0; k < rb->nb; ++k) {
                const rd_sv_param* p = &S->pb->p[rb->blk[k]];
                double* J = jacobians[k];
                if (!J) continue;
                if (alpha_sq_norm == 0.0) {
                    for (i = 0; i < nres * p->tangent; ++i) J[i] *= sqrt_rho1;
                } else {
                    int c, r;
                    for (c = 0; c < p->tangent; ++c) {
                        double rtj = 0.0;
                        for (r = 0; r < nres; ++r) rtj += J[r * p->tangent + c] * res[r];
                        for (r = 0; r < nres; ++r)
                            J[r * p->tangent + c] = sqrt_rho1 * (J[r * p->tangent + c] - alpha_sq_norm * res[r] * rtj);
                    }
                }
            }
        }
        if (residuals) for (i = 0; i < nres; ++i) res[i] *= residual_scaling;
    }
    return 1;
}

/* ParameterBlock::SetState for the reduced blocks (state = x + offset); the PlusJacobian of the quaternion manifold is the
 * constant [I; 0] */
static int state_to_blocks(rd_sv_state* S, const double* x) {
    int i;
    for (i = 0; i < S->np_red; ++i) {
        rd_sv_param* p = &S->pb->p[S->porder[i]];
        S->state_ptr[i] = x + p->state_offset;
        if (p->kind == RD_SV_KIND_OTHER) return 0;
    }
    return 1;
}

/* ProgramEvaluator::Evaluate (one thread): cost, residuals (may be NULL), gradient (may be NULL), jacobian
 * (into S->values, may be off). Returns 1 on success. */
static int evaluate(rd_sv_state* S, const double* x, double* cost, double* residuals, double* gradient, int jacobian) {
    double scratch_cost = 0.0;
    int i, k;
    if (!state_to_blocks(S, x)) return 0;
    if (residuals) memset(residuals, 0, sizeof(double) * (size_t)S->num_residuals);
    if (jacobian) memset(S->values, 0, sizeof(double) * (size_t)S->nnz);
    if (gradient) memset(S->scratch_grad, 0, sizeof(double) * (size_t)S->num_effective);
    for (i = 0; i < S->nr_red; ++i) {
        rd_sv_resid* rb = &S->pb->r[S->rorder[i]];
        const rd_sv_row* row = &S->rows[i];
        double* block_res = NULL;
        double* jacs[RD_SV_MAXB];
        double* const* block_jacs = NULL;
        const double* params[RD_SV_MAXB];
        double block_cost;
        if (residuals) block_res = residuals + row->position;
        else if (gradient) block_res = S->scratch_res2;
        for (k = 0; k < rb->nb; ++k) {
            const rd_sv_param* p = &S->pb->p[rb->blk[k]];
            params[k] = p->index >= 0 ? S->state_ptr[p->index] : p->x;
            jacs[k] = NULL;
        }
        if (jacobian || gradient) {  /* BlockEvaluatePreparer::Prepare */
            int c;
            for (c = 0; c < row->ncells; ++c) jacs[row->cells[c].arg] = S->values + row->cells[c].position;
            block_jacs = jacs;
        }
        if (!rd_sv_eval_block(S, rb, params, &block_cost, block_res, block_jacs)) return 0;
        scratch_cost += block_cost;
        if (gradient) {
            int c;
            for (c = 0; c < row->ncells; ++c) {
                const rd_sv_param* p = &S->pb->p[S->porder[row->cells[c].block_id]];
                ok_mtv(jacs[row->cells[c].arg], rb->nres, p->tangent, block_res, S->scratch_grad + p->delta_offset, 1);
            }
        }
    }
    *cost = 0.0;
    *cost += scratch_cost;
    if (gradient) for (i = 0; i < S->num_effective; ++i) gradient[i] = 0.0 + S->scratch_grad[i];
    if (!(*cost == *cost) || *cost > 1.7976931348623157e308 || *cost < -1.7976931348623157e308) return 0;
    return 1;
}

/* Program::Plus: per block manifold plus */
static int plus(rd_sv_state* S, const double* x, const double* delta, double* x_plus_delta) {
    int i, j;
    for (i = 0; i < S->np_red; ++i) {
        const rd_sv_param* p = &S->pb->p[S->porder[i]];
        const double* bx = x + p->state_offset;
        const double* bd = delta + p->delta_offset;
        double* out = x_plus_delta + p->state_offset;
        if (p->kind == RD_SV_KIND_QUAT) rd_quat_plus(bx, bd, out);   /* QuaternionParameterization::Plus (always true) */
        else if (p->kind == RD_SV_KIND_NONE) { for (j = 0; j < p->size; ++j) out[j] = bx[j] + bd[j]; }
        else return 0;
    }
    return 1;
}

/* ---- BlockSparseMatrix operations (single thread) ---- */
void rd_sv_jac_right_multiply(const rd_sv_state* S, const double* x, double* y) {  /* y += J x */
    int i, c;
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_row* row = &S->rows[i];
        for (c = 0; c < row->ncells; ++c) {
            const rd_sv_param* p = &S->pb->p[S->porder[row->cells[c].block_id]];
            ok_mv(S->values + row->cells[c].position, row->size, p->tangent, x + p->delta_offset, y + row->position, 1);
        }
    }
}
void rd_sv_jac_left_multiply(const rd_sv_state* S, const double* x, double* y) {  /* y += J^T x */
    int i, c;
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_row* row = &S->rows[i];
        for (c = 0; c < row->ncells; ++c) {
            const rd_sv_param* p = &S->pb->p[S->porder[row->cells[c].block_id]];
            ok_mtv(S->values + row->cells[c].position, row->size, p->tangent, x + row->position, y + p->delta_offset, 1);
        }
    }
}
/* SquaredColumnNorm: x = 0; per cell `VectorRef(x + pos, cols) += m.colwise().squaredNorm()` with m a row-major
 * map: a LinearVectorized assignment peeled by the destination address (x is a 16-byte aligned Vector, so
 * alignedStart = pos & 1); packet columns reduce the rows with Eigen's packetwise_redux_impl tree
 * (p0 + ((p1+p2)+(p3+p4)) + ... then the remaining rows one by one), peeled columns with the scalar left fold. */
static void squared_column_norm(const rd_sv_state* S, double* x) {
    int i, c;
    memset(x, 0, sizeof(double) * (size_t)S->num_effective);
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_row* row = &S->rows[i];
        for (c = 0; c < row->ncells; ++c) {
            const rd_sv_param* p = &S->pb->p[S->porder[row->cells[c].block_id]];
            ok_colwise_sqnorm_add(S->values + row->cells[c].position, row->size, p->tangent, p->delta_offset & 1,
                                  x + p->delta_offset);
        }
    }
}
static void scale_columns(rd_sv_state* S, const double* scale) {
    int i, c, r, j;
    for (i = 0; i < S->nr_red; ++i) {
        const rd_sv_row* row = &S->rows[i];
        for (c = 0; c < row->ncells; ++c) {
            const rd_sv_param* p = &S->pb->p[S->porder[row->cells[c].block_id]];
            double* m = S->values + row->cells[c].position;
            for (r = 0; r < row->size; ++r)
                for (j = 0; j < p->tangent; ++j) m[r * p->tangent + j] = m[r * p->tangent + j] * scale[p->delta_offset + j];
        }
    }
}

/* ----------------------------------------------- Dogleg strategy ------------------------------------------- */
static void dogleg_init(rd_sv_dogleg_state* D, const rd_sv_options* o, int n) {
    D->radius = o->initial_trust_region_radius;
    D->max_radius = o->max_trust_region_radius;
    D->min_diagonal = o->min_lm_diagonal;
    D->max_diagonal = o->max_lm_diagonal;
    D->mu = 1e-8; D->min_mu = 1e-8; D->max_mu = 1.0;
    D->mu_increase_factor = 10.0;
    D->increase_threshold = 0.75; D->decrease_threshold = 0.25;
    D->dogleg_step_norm = 0.0;
    D->reuse = 0;
    D->n = n;
    D->diagonal = (double*)xcalloc((size_t)n, sizeof(double));
    D->gradient = (double*)xcalloc((size_t)n, sizeof(double));
    D->gauss_newton_step = (double*)xcalloc((size_t)n, sizeof(double));
    D->lm_diagonal = (double*)xcalloc((size_t)n, sizeof(double));
    D->scaled_gradient = (double*)xcalloc((size_t)n, sizeof(double));
}

static void dogleg_traditional_step(rd_sv_state* S, double* step) {
    rd_sv_dogleg_state* D = &S->dog;
    const int n = D->n;
    const double gradient_norm = ok_dyn_norm(D->gradient, n);
    const double gauss_newton_norm = ok_dyn_norm(D->gauss_newton_step, n);
    int i;
    if (gauss_newton_norm <= D->radius) {
        for (i = 0; i < n; ++i) step[i] = D->gauss_newton_step[i];
        D->dogleg_step_norm = gauss_newton_norm;
        for (i = 0; i < n; ++i) step[i] /= D->diagonal[i];
        D->branch = 0;
        return;
    }
    if (gradient_norm * D->alpha >= D->radius) {
        const double s = -(D->radius / gradient_norm);
        for (i = 0; i < n; ++i) step[i] = s * D->gradient[i];
        D->dogleg_step_norm = D->radius;
        for (i = 0; i < n; ++i) step[i] /= D->diagonal[i];
        D->branch = 1;
        return;
    }
    {
        const double b_dot_a = -D->alpha * ok_dyn_dot(D->gradient, D->gauss_newton_step, n);
        const double a_squared_norm = pow(D->alpha * gradient_norm, 2.0);
        const double b_minus_a_squared_norm = a_squared_norm - 2 * b_dot_a + pow(gauss_newton_norm, 2.0);
        const double c = b_dot_a - a_squared_norm;
        const double d = sqrt(c * c + b_minus_a_squared_norm * (pow(D->radius, 2.0) - a_squared_norm));
        const double beta = (c <= 0) ? (d - c) / b_minus_a_squared_norm : (D->radius * D->radius - a_squared_norm) / (d + c);
        const double ga = -D->alpha * (1.0 - beta);
        for (i = 0; i < n; ++i) step[i] = ga * D->gradient[i] + beta * D->gauss_newton_step[i];
        D->dogleg_step_norm = ok_dyn_norm(step, n);
        for (i = 0; i < n; ++i) step[i] /= D->diagonal[i];
        D->branch = 2;
    }
}

/* DoglegStrategy::ComputeGaussNewtonStep: the mu loop around the linear solver */
static int dogleg_gauss_newton(rd_sv_state* S) {
    rd_sv_dogleg_state* D = &S->dog;
    const int n = D->n;
    int term = RD_SV_LS_FAILURE, i;
    D->mu_increases = 0;
    while (D->mu < D->max_mu) {
        const double sm = sqrt(D->mu);
        int valid = 1;
        for (i = 0; i < n; ++i) D->lm_diagonal[i] = D->diagonal[i] * sm;
        for (i = 0; i < n; ++i) D->gauss_newton_step[i] = NAN;  /* InvalidateArray */
        term = rd_sv_linear_solve(S, D->lm_diagonal, D->gauss_newton_step);
        if (term == RD_SV_LS_FATAL_ERROR) return term;
        for (i = 0; i < n; ++i) if (!(D->gauss_newton_step[i] == D->gauss_newton_step[i]) || fabs(D->gauss_newton_step[i]) > 1.7976931348623157e308) valid = 0;
        if (term == RD_SV_LS_FAILURE || !valid) {
            D->mu *= D->mu_increase_factor;
            D->mu_increases++;
            term = RD_SV_LS_FAILURE;
            continue;
        }
        break;
    }
    if (S->hooks && S->hooks->on_gn) {
        rd_sv_gn g;
        g.termination = term; g.mu_increases = D->mu_increases; g.mu = D->mu; g.n = n;
        g.lm_diagonal = D->lm_diagonal;
        g.gauss_newton_step = (term != RD_SV_LS_FAILURE && term != RD_SV_LS_FATAL_ERROR) ? D->gauss_newton_step : NULL;
        S->hooks->on_gn(S->hooks->ctx, &g);
    }
    if (term != RD_SV_LS_FAILURE)
        for (i = 0; i < n; ++i) D->gauss_newton_step[i] = D->gauss_newton_step[i] * -D->diagonal[i];
    return term;
}

/* DoglegStrategy::ComputeStep; returns the linear solver termination type */
static int dogleg_compute_step(rd_sv_state* S, const double* residuals, double* step) {
    rd_sv_dogleg_state* D = &S->dog;
    const int n = D->n;
    int term, i;
    const int reuse = D->reuse;
    if (D->reuse) {
        dogleg_traditional_step(S, step);
        term = RD_SV_LS_SUCCESS;
    } else {
        double jg_sq, g_sq;
        D->reuse = 1;
        squared_column_norm(S, D->diagonal);
        for (i = 0; i < n; ++i) {
            double v = D->diagonal[i];
            v = v > D->min_diagonal ? v : D->min_diagonal;   /* std::max(d, min) */
            v = v < D->max_diagonal ? v : D->max_diagonal;   /* std::min(.., max) */
            D->diagonal[i] = v;
        }
        for (i = 0; i < n; ++i) D->diagonal[i] = sqrt(D->diagonal[i]);
        /* ComputeGradient */
        memset(D->gradient, 0, sizeof(double) * (size_t)n);
        rd_sv_jac_left_multiply(S, residuals, D->gradient);
        for (i = 0; i < n; ++i) D->gradient[i] /= D->diagonal[i];
        /* ComputeCauchyPoint */
        memset(S->Jg, 0, sizeof(double) * (size_t)S->num_residuals);
        for (i = 0; i < n; ++i) D->scaled_gradient[i] = D->gradient[i] / D->diagonal[i];
        rd_sv_jac_right_multiply(S, D->scaled_gradient, S->Jg);
        g_sq = ok_dyn_sqnorm(D->gradient, n);
        jg_sq = ok_dyn_sqnorm(S->Jg, S->num_residuals);
        D->alpha = g_sq / jg_sq;
        term = dogleg_gauss_newton(S);
        if (term == RD_SV_LS_FATAL_ERROR) return term;
        if (term != RD_SV_LS_FAILURE) dogleg_traditional_step(S, step);
    }
    if (S->hooks && S->hooks->on_dogleg) {
        rd_sv_dogleg d;
        d.reuse = reuse; d.termination = term; d.n = n;
        d.radius = D->radius; d.mu = D->mu; d.alpha = D->alpha; d.dogleg_step_norm = D->dogleg_step_norm;
        d.gradient_norm = ok_dyn_norm(D->gradient, n);
        d.gauss_newton_norm = ok_dyn_norm(D->gauss_newton_step, n);
        d.gradient = D->gradient; d.gauss_newton_step = D->gauss_newton_step; d.diagonal = D->diagonal;
        d.step = (reuse || term == RD_SV_LS_SUCCESS) ? step : NULL;
        S->hooks->on_dogleg(S->hooks->ctx, &d);
    }
    return term;
}
static void dogleg_step_accepted(rd_sv_dogleg_state* D, double step_quality) {
    double m;
    if (step_quality < D->decrease_threshold) D->radius *= 0.5;
    if (step_quality > D->increase_threshold) {
        const double r = 3.0 * D->dogleg_step_norm;
        D->radius = D->radius > r ? D->radius : r;  /* std::max(radius, 3*norm) */
    }
    m = 2.0 * D->mu / D->mu_increase_factor;
    D->mu = D->min_mu > m ? D->min_mu : m;
    D->reuse = 0;
}
static void dogleg_step_rejected(rd_sv_dogleg_state* D) { D->radius *= 0.5; D->reuse = 1; }
static void dogleg_step_invalid(rd_sv_dogleg_state* D) { D->mu *= D->mu_increase_factor; D->reuse = 0; }

/* -------------------------------------------- TrustRegionMinimizer ---------------------------------------- */
static int evaluate_gradient_and_jacobian(rd_sv_state* S) {
    int i;
    if (!evaluate(S, S->x, &S->x_cost, S->residuals, S->gradient, 1)) return 0;
    S->it.cost = S->x_cost + S->fixed_cost;
    if (S->pb->opt.jacobi_scaling) {
        if (S->it.iteration == 0) {
            squared_column_norm(S, S->jacobian_scaling);
            for (i = 0; i < S->num_effective; ++i) S->jacobian_scaling[i] = 1.0 / (1.0 + sqrt(S->jacobian_scaling[i]));
        }
        scale_columns(S, S->jacobian_scaling);
    }
    for (i = 0; i < S->num_effective; ++i) S->negative_gradient[i] = -S->gradient[i];
    if (!plus(S, S->x, S->negative_gradient, S->projected_gradient_step)) return 0;
    S->it.gradient_max_norm = ok_dyn_maxabs_diff(S->x, S->projected_gradient_step, S->num_parameters);
    S->it.gradient_norm = ok_dyn_norm_diff(S->x, S->projected_gradient_step, S->num_parameters);
    return 1;
}

static void report_iteration(rd_sv_state* S) {
    if (!S->hooks || !S->hooks->on_iter) return;
    S->it.model_cost_change = S->model_cost_change;
    S->it.candidate_cost = S->candidate_valid ? S->candidate_cost : 0.0;
    S->it.x_cost = S->x_cost;
    S->it.minimum_cost = S->minimum_cost;
    S->it.num_parameters = S->num_parameters;
    S->it.num_effective_parameters = S->num_effective;
    S->it.num_residuals = S->num_residuals;
    S->it.candidate_valid = S->candidate_valid; S->it.delta_valid = S->delta_valid; S->it.step_valid = S->step_valid;
    S->it.x = S->x; S->it.candidate_x = S->candidate_x; S->it.delta = S->delta; S->it.trust_region_step = S->trust_region_step;
    S->it.gradient = S->gradient; S->it.jacobian_scaling = S->jacobian_scaling; S->it.residuals = S->residuals;
    S->hooks->on_iter(S->hooks->ctx, &S->it);
}

static void update_user_state(rd_sv_state* S) {
    int i;
    if (!S->pb->opt.update_state_every_iteration) return;
    for (i = 0; i < S->np_red; ++i) {
        rd_sv_param* p = &S->pb->p[S->porder[i]];
        memcpy(p->x, S->parameters + p->state_offset, sizeof(double) * (size_t)p->size);
    }
}

/* FinalizeIterationAndCheckIfMinimizerCanContinue; returns 1 to continue */
static int finalize_iteration(rd_sv_state* S) {
    const rd_sv_options* o = &S->pb->opt;
    if (S->it.step_is_successful) {
        S->num_successful_steps++;
        if (S->x_cost < S->minimum_cost) {
            S->minimum_cost = S->x_cost;
            memcpy(S->parameters, S->x, sizeof(double) * (size_t)S->num_parameters);
            S->it.step_is_nonmonotonic = 0;
        } else S->it.step_is_nonmonotonic = 1;
    } else S->num_unsuccessful_steps++;
    S->it.trust_region_radius = S->dog.radius;
    S->num_iterations++;
    S->last_iteration = S->it;
    if (S->it.cost < S->min_iter_cost) S->min_iter_cost = S->it.cost;
    report_iteration(S);
    update_user_state(S);   /* StateUpdatingCallback (update_state_every_iteration): user state = the best parameters so far */
    if (S->it.iteration >= o->max_num_iterations) { S->termination = RD_SV_NO_CONVERGENCE; return 0; }
    if (S->it.step_is_successful && S->it.gradient_max_norm <= o->gradient_tolerance) { S->termination = RD_SV_CONVERGENCE; return 0; }
    if (S->it.trust_region_radius <= o->min_trust_region_radius) { S->termination = RD_SV_CONVERGENCE; return 0; }
    return 1;
}

static int compute_trust_region_step(rd_sv_state* S) {
    int term, i;
    S->it.step_is_valid = 0;
    term = dogleg_compute_step(S, S->residuals, S->trust_region_step);
    if (term == RD_SV_LS_FATAL_ERROR) { S->termination = RD_SV_FAILURE; return 0; }
    if (term == RD_SV_LS_FAILURE) return 1;
    S->step_valid = 1;
    memset(S->model_residuals, 0, sizeof(double) * (size_t)S->num_residuals);
    rd_sv_jac_right_multiply(S, S->trust_region_step, S->model_residuals);
    S->model_cost_change = -(0.0 + (0.0 + ok_dyn_dot_model(S->model_residuals, S->residuals, S->num_residuals)));
    S->it.step_is_valid = S->model_cost_change > 0.0;
    if (S->it.step_is_valid) {
        for (i = 0; i < S->num_effective; ++i) S->delta[i] = S->trust_region_step[i] * S->jacobian_scaling[i];
        S->delta_valid = 1;
        S->num_consecutive_invalid_steps = 0;
    }
    return 1;
}

static int handle_invalid_step(rd_sv_state* S) {
    if (++S->num_consecutive_invalid_steps >= S->pb->opt.max_num_consecutive_invalid_steps) {
        S->termination = RD_SV_FAILURE;
        return 0;
    }
    dogleg_step_invalid(&S->dog);
    S->it.cost = S->x_cost + S->fixed_cost;
    S->it.cost_change = 0.0;
    S->it.gradient_max_norm = S->last_iteration.gradient_max_norm;
    S->it.gradient_norm = S->last_iteration.gradient_norm;
    S->it.step_norm = 0.0;
    S->it.relative_decrease = 0.0;
    return 1;
}

/* TrustRegionStepEvaluator (max_consecutive_nonmonotonic_steps = 0) */
static double step_quality(const rd_sv_state* S, double cost, double model_cost_change) {
    const rd_sv_step_eval* E = &S->se;
    double rel, hist;
    if (cost >= 1.7976931348623157e308) return -1.7976931348623157e308;
    rel = (E->current_cost - cost) / model_cost_change;
    hist = (E->reference_cost - cost) / (E->accumulated_reference_model_cost_change + model_cost_change);
    return rel > hist ? rel : hist;  /* std::max */
}
static void step_evaluator_accepted(rd_sv_step_eval* E, double cost, double model_cost_change) {
    E->current_cost = cost;
    E->accumulated_candidate_model_cost_change += model_cost_change;
    E->accumulated_reference_model_cost_change += model_cost_change;
    if (E->current_cost < E->minimum_cost) {
        E->minimum_cost = E->current_cost;
        E->num_consecutive_nonmonotonic_steps = 0;
        E->candidate_cost = E->current_cost;
        E->accumulated_candidate_model_cost_change = 0.0;
    } else {
        ++E->num_consecutive_nonmonotonic_steps;
        if (E->current_cost > E->candidate_cost) {
            E->candidate_cost = E->current_cost;
            E->accumulated_candidate_model_cost_change = 0.0;
        }
    }
    if (E->num_consecutive_nonmonotonic_steps == E->max_consecutive_nonmonotonic_steps) {
        E->reference_cost = E->candidate_cost;
        E->accumulated_reference_model_cost_change = E->accumulated_candidate_model_cost_change;
    }
}

static void minimize(rd_sv_state* S) {
    const rd_sv_options* o = &S->pb->opt;
    int atleast_one_successful_step = 0;
    /* IterationZero */
    memset(&S->it, 0, sizeof S->it);
    S->it.iteration = 0;
    if (!evaluate_gradient_and_jacobian(S)) { S->termination = RD_SV_FAILURE; S->init_failed = 1; return; }
    S->initial_cost = S->x_cost + S->fixed_cost;
    S->it.step_is_valid = 1;
    S->it.step_is_successful = 1;
    /* step evaluator */
    S->se.max_consecutive_nonmonotonic_steps = 0;
    S->se.minimum_cost = S->se.current_cost = S->se.reference_cost = S->se.candidate_cost = S->x_cost;
    S->se.accumulated_reference_model_cost_change = S->se.accumulated_candidate_model_cost_change = 0.0;
    S->se.num_consecutive_nonmonotonic_steps = 0;
    while (finalize_iteration(S)) {
        const double previous_gradient_norm = S->it.gradient_norm;
        const double previous_gradient_max_norm = S->it.gradient_max_norm;
        const int previous_iteration = S->it.iteration;
        memset(&S->it, 0, sizeof S->it);
        S->it.iteration = previous_iteration + 1;
        if (!compute_trust_region_step(S)) return;
        if (!S->it.step_is_valid) {
            if (!handle_invalid_step(S)) return;
            continue;
        }
        /* ComputeCandidatePointAndEvaluateCost */
        S->candidate_valid = 1;
        if (!plus(S, S->x, S->delta, S->candidate_x)) S->candidate_cost = 1.7976931348623157e308;
        else if (!evaluate(S, S->candidate_x, &S->candidate_cost, NULL, NULL, 0)) S->candidate_cost = 1.7976931348623157e308;
        /* ParameterToleranceReached (only after a successful step) */
        if (atleast_one_successful_step) {
            const double x_norm = ok_dyn_norm(S->x, S->num_parameters);
            S->it.step_norm = ok_dyn_norm_diff(S->x, S->candidate_x, S->num_parameters);
            if (S->it.step_norm <= o->parameter_tolerance * (x_norm + o->parameter_tolerance)) {
                S->termination = RD_SV_CONVERGENCE;
                return;
            }
        }
        /* FunctionToleranceReached */
        S->it.cost_change = S->x_cost - S->candidate_cost;
        if (fabs(S->it.cost_change) <= o->function_tolerance * S->x_cost) { S->termination = RD_SV_CONVERGENCE; return; }
        /* IsStepSuccessful */
        S->it.relative_decrease = step_quality(S, S->candidate_cost, S->model_cost_change);
        if (S->it.relative_decrease > o->min_relative_decrease) {
            atleast_one_successful_step = 1;
            memcpy(S->x, S->candidate_x, sizeof(double) * (size_t)S->num_parameters);
            if (!evaluate_gradient_and_jacobian(S)) { S->termination = RD_SV_FAILURE; return; }
            S->it.step_is_successful = 1;
            dogleg_step_accepted(&S->dog, S->it.relative_decrease);
            step_evaluator_accepted(&S->se, S->candidate_cost, S->model_cost_change);
        } else {
            S->it.step_is_successful = 0;
            S->it.cost = S->candidate_cost + S->fixed_cost;
            S->it.gradient_norm = previous_gradient_norm;
            S->it.gradient_max_norm = previous_gradient_max_norm;
            dogleg_step_rejected(&S->dog);
        }
    }
}

/* ------------------------------------------------- Solve ------------------------------------------------- */
int rd_sv_solve(rd_sv_problem* pb, const rd_sv_hooks* hooks) {
    rd_sv_state St;
    rd_sv_state* S = &St;
    int i, size_of_first_group = 0, maxres = 0, maxscratch = 0;
    memset(S, 0, sizeof *S);
    S->pb = pb;
    S->hooks = hooks;
    S->termination = RD_SV_NO_CONVERGENCE;
    S->porder = (int*)xcalloc((size_t)pb->np, sizeof(int));
    S->sorder = (int*)xcalloc((size_t)pb->np, sizeof(int));
    S->rorder = (int*)xcalloc((size_t)pb->nr, sizeof(int));
    for (i = 0; i < pb->nr; ++i) {  /* scratch sizes: Program::MaxScratchDoublesNeededForEvaluate etc. */
        const rd_sv_resid* rb = &pb->r[i];
        int k, sd = 1;
        for (k = 0; k < rb->nb; ++k) if (pb->p[rb->blk[k]].kind != RD_SV_KIND_NONE) sd += pb->p[rb->blk[k]].size;
        sd *= rb->nres;
        if (sd > maxscratch) maxscratch = sd;
        if (rb->nres > maxres) maxres = rb->nres;
    }
    S->scratch = (double*)xcalloc((size_t)maxscratch, sizeof(double));
    S->scratch_res = S->scratch;  /* ResidualBlock::Evaluate: residuals = scratch when not outputting them */
    S->scratch_res2 = (double*)xcalloc((size_t)maxres, sizeof(double));
    for (i = 0; i < pb->np; ++i) {   /* quaternion manifold: PlusJacobian = [I; 0] (ambient 4 x tangent 3, row-major) */
        static const double qpj[12] = {1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
        if (pb->p[i].kind == RD_SV_KIND_QUAT) memcpy(pb->p[i].plus_jacobian, qpj, sizeof qpj);
    }
    if (!reduce_program(S)) { S->termination = RD_SV_FAILURE; goto done; }
    if (S->np_red == 0) {   /* "No non-constant parameter blocks found": Ceres still writes the REDUCED record (empty program) */
        if (hooks && hooks->on_reduced) hooks->on_reduced(hooks->ctx, pb, S->fixed_cost, -1, 0, S->porder, 0, S->rorder);
        S->termination = RD_SV_CONVERGENCE; S->initial_cost = S->final_cost = S->fixed_cost; S->num_successful_steps = S->num_unsuccessful_steps = -1; goto done; }
    set_offsets(S);
    if (pb->opt.linear_solver_type == RD_SV_SPARSE_SCHUR) {
        size_of_first_group = schur_ordering(S);          /* ComputeStableSchurOrdering */
        set_offsets(S);
        schur_complement_ordering(S, size_of_first_group); /* block AMD of the Schur complement columns (Eigen) */
        set_offsets(S);
        lexicographic_residual_order(S, size_of_first_group);
        S->num_eliminate_blocks = size_of_first_group;
    } else { S->termination = RD_SV_FAILURE; goto done; }
    if (hooks && hooks->on_reduced)
        hooks->on_reduced(hooks->ctx, pb, S->fixed_cost, S->num_eliminate_blocks, S->np_red, S->porder, S->nr_red, S->rorder);
    build_jacobian_structure(S);
    S->state_ptr = (const double**)xcalloc((size_t)S->np_red, sizeof(double*));
    S->scratch_grad = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    S->parameters = (double*)xcalloc((size_t)S->num_parameters, sizeof(double));
    S->x = (double*)xcalloc((size_t)S->num_parameters, sizeof(double));
    S->candidate_x = (double*)xcalloc((size_t)S->num_parameters, sizeof(double));
    S->projected_gradient_step = (double*)xcalloc((size_t)S->num_parameters, sizeof(double));
    S->residuals = (double*)xcalloc((size_t)S->num_residuals, sizeof(double));
    S->model_residuals = (double*)xcalloc((size_t)S->num_residuals, sizeof(double));
    S->Jg = (double*)xcalloc((size_t)S->num_residuals, sizeof(double));
    S->gradient = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    S->negative_gradient = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    S->jacobian_scaling = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    S->trust_region_step = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    S->delta = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    for (i = 0; i < S->num_effective; ++i) S->jacobian_scaling[i] = 1.0;
    /* ParameterBlocksToStateVector */
    for (i = 0; i < S->np_red; ++i) {
        const rd_sv_param* p = &pb->p[S->porder[i]];
        memcpy(S->parameters + p->state_offset, p->x, sizeof(double) * (size_t)p->size);
    }
    memcpy(S->x, S->parameters, sizeof(double) * (size_t)S->num_parameters);
    S->x_cost = 1.7976931348623157e308;
    S->minimum_cost = S->x_cost;
    S->min_iter_cost = 1.7976931348623157e308;
    S->model_cost_change = 0.0;
    dogleg_init(&S->dog, &pb->opt, S->num_effective);
    rd_sv_linear_init(S);
    minimize(S);
    /* CopyParameterBlockStateToUserState: the minimizer's parameters_ (best x) */
    if (!S->init_failed) {
        for (i = 0; i < S->np_red; ++i) {
            rd_sv_param* p = &pb->p[S->porder[i]];
            memcpy(p->x, S->parameters + p->state_offset, sizeof(double) * (size_t)p->size);
        }
    }
    /* SetSummaryFinalCost: min(initial_cost, cost of every recorded iteration): a rejected step's candidate cost can be lower than
     * the cost of the parameters that were kept */
    S->final_cost = S->min_iter_cost < S->initial_cost ? S->min_iter_cost : S->initial_cost;
done:
    if (hooks && hooks->on_end) {
        rd_sv_end e;
        uint64_t h = 1469598103934665603ULL;
        e.termination_type = S->termination;
        e.num_iterations = S->num_iterations;
        e.num_successful_steps = S->num_successful_steps;
        e.num_unsuccessful_steps = S->num_unsuccessful_steps;
        e.initial_cost = S->initial_cost;
        e.final_cost = S->final_cost;
        e.fixed_cost = S->fixed_cost;
        for (i = 0; i < pb->np; ++i) h = rd_fnv_combine(h, rd_fnv(pb->p[i].x, sizeof(double) * (size_t)pb->p[i].size));
        e.param_hash = h;
        e.np = pb->np;
        hooks->on_end(hooks->ctx, &e);
    }
    rd_sv_linear_free(S);
    free(S->dog.diagonal); free(S->dog.gradient); free(S->dog.gauss_newton_step); free(S->dog.lm_diagonal);
    free(S->dog.scaled_gradient);
    free(S->porder); free(S->sorder); free(S->rorder); free(S->scratch); free(S->scratch_res2);
    free(S->rows); free(S->cells); free(S->values); free((void*)S->state_ptr); free(S->scratch_grad);
    free(S->parameters); free(S->x); free(S->candidate_x); free(S->projected_gradient_step); free(S->residuals);
    free(S->model_residuals); free(S->Jg); free(S->gradient); free(S->negative_gradient); free(S->jacobian_scaling);
    free(S->trust_region_step); free(S->delta);
    return S->termination;
}
