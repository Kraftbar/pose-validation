/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* See ok_solve.h. Part 2: the linear solvers. DENSE_SCHUR = SchurEliminator<Dynamic,Dynamic,Dynamic> (naive
 * small_blas kernels, InvertPSDMatrix<Dynamic>) + BlockRandomAccessDenseMatrix + EigenDenseCholesky
 * (LLT<Ref<MatrixXd>, Lower>); SPARSE_NORMAL_CHOLESKY = J^T J by InnerProductComputer (LOWER_TRIANGULAR block
 * storage) + the appended D rows + EigenSparseCholesky (SimplicialLDLT, natural ordering). Ceres 2.2.0,
 * BSD-3-Clause, Copyright 2023 Google Inc.; Eigen models MPL-2.0. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "ok_blas.h"
#include "ok_dense.h"
#include "ok_align4.h"
#include "ok_solve_internal.h"

typedef struct chunk { int start, size, nf; int* f_blocks; int* f_offsets; } chunk;  /* f_blocks ascending (std::map) */

typedef struct lin {
    int type;
    /* Schur */
    int nchunks; chunk* chunks; int buffer_size; double* buffer; double* b1t_inv; int uneliminated_row_begins;
    int nf; int* lhs_row_layout; int lhs_n; double* lhs; double* lhs_copy; double* rhs; double* sol; int schur_reported;
    double *ete, *inv_ete, *g, *inv_ete_g, *sj; int max_e, max_row;
    /* sparse */
    int sp_init; int nterms; int* t_row; int* t_col; int* t_off; int* row_block_nnz;
    int crs_nnz; int* crs_rows; int* crs_cols; double* crs_vals; double* sp_rhs; double* dcell; int max_blk;
    ok_ldlt ldlt; int analyzed;
} lin;

static void* xcalloc(size_t n, size_t sz) { return calloc(n ? n : 1, sz); }
static int bsize(const ok_sv_state* S, int reduced_index) { return S->pb->p[S->porder[reduced_index]].tangent; }
static int bpos(const ok_sv_state* S, int reduced_index) { return S->pb->p[S->porder[reduced_index]].delta_offset; }

/* ---------------------------------------------- Schur: Init ---------------------------------------------- */
static void schur_init(ok_sv_state* S, lin* L) {
    const int ne = S->num_eliminate_blocks;
    int r = 0, i, c, lhs_rows = 0;
    L->nf = S->np_red - ne;
    L->lhs_row_layout = (int*)xcalloc((size_t)L->nf, sizeof(int));
    for (i = ne; i < S->np_red; ++i) { L->lhs_row_layout[i - ne] = lhs_rows; lhs_rows += bsize(S, i); }
    L->lhs_n = lhs_rows;
    L->chunks = (chunk*)xcalloc((size_t)S->nr_red + 1, sizeof(chunk));
    L->buffer_size = 1;
    L->max_e = 1; L->max_row = 1;
    while (r < S->nr_red) {
        const int chunk_block_id = S->rows[r].cells[0].block_id;
        chunk* ch;
        int buffer_size = 0, e_block_size;
        if (chunk_block_id >= ne) break;
        ch = &L->chunks[L->nchunks++];
        ch->start = r; ch->size = 0;
        ch->f_blocks = (int*)xcalloc((size_t)S->np_red, sizeof(int));
        ch->f_offsets = (int*)xcalloc((size_t)S->np_red, sizeof(int));
        e_block_size = bsize(S, chunk_block_id);
        if (e_block_size > L->max_e) L->max_e = e_block_size;
        while (r + ch->size < S->nr_red) {
            const ok_sv_row* row = &S->rows[r + ch->size];
            if (row->cells[0].block_id != chunk_block_id) break;
            if (row->size > L->max_row) L->max_row = row->size;
            for (c = 1; c < row->ncells; ++c) {
                const int fb = row->cells[c].block_id;
                int k, present = 0;
                for (k = 0; k < ch->nf; ++k) if (ch->f_blocks[k] == fb) { present = 1; break; }
                if (!present) {  /* InsertIfNotPresent: offset in first-encounter order */
                    ch->f_blocks[ch->nf] = fb; ch->f_offsets[ch->nf] = buffer_size; ch->nf++;
                    buffer_size += e_block_size * bsize(S, fb);
                }
            }
            if (buffer_size > L->buffer_size) L->buffer_size = buffer_size;
            ++ch->size;
        }
        /* std::map iteration order: ascending f block id (insertion sort, keeping the offsets) */
        for (i = 1; i < ch->nf; ++i) {
            int fb = ch->f_blocks[i], fo = ch->f_offsets[i], j;
            for (j = i - 1; j >= 0 && ch->f_blocks[j] > fb; --j) { ch->f_blocks[j + 1] = ch->f_blocks[j]; ch->f_offsets[j + 1] = ch->f_offsets[j]; }
            ch->f_blocks[j + 1] = fb; ch->f_offsets[j + 1] = fo;
        }
        r += ch->size;
    }
    for (i = 0; i < S->nr_red; ++i) if (S->rows[i].size > L->max_row) L->max_row = S->rows[i].size;
    L->uneliminated_row_begins = L->nchunks ? L->chunks[L->nchunks - 1].start + L->chunks[L->nchunks - 1].size : 0;
    L->buffer = (double*)xcalloc((size_t)L->buffer_size, sizeof(double));
    L->b1t_inv = (double*)xcalloc((size_t)L->buffer_size, sizeof(double));
    L->lhs = (double*)xcalloc((size_t)L->lhs_n * (size_t)L->lhs_n, sizeof(double));
    L->lhs_copy = (double*)xcalloc((size_t)L->lhs_n * (size_t)L->lhs_n, sizeof(double));
    L->rhs = (double*)xcalloc((size_t)L->lhs_n, sizeof(double));
    L->sol = (double*)xcalloc((size_t)L->lhs_n, sizeof(double));
    L->ete = (double*)xcalloc((size_t)L->max_e * (size_t)L->max_e, sizeof(double));
    L->inv_ete = (double*)xcalloc((size_t)L->max_e * (size_t)L->max_e, sizeof(double));
    L->g = (double*)xcalloc((size_t)L->max_e, sizeof(double));
    L->inv_ete_g = (double*)xcalloc((size_t)L->max_e, sizeof(double));
    L->sj = (double*)xcalloc((size_t)L->max_row, sizeof(double));
}

/* DetectStructure (for the SCHUR record only) */
static void detect_structure(const ok_sv_state* S, ok_sv_schur* out) {
    const int ne = S->num_eliminate_blocks;
    int r, rbs = 0, ebs = 0, fbs = 0;
    for (r = 0; r < S->nr_red; ++r) {
        const ok_sv_row* row = &S->rows[r];
        int c;
        if (row->cells[0].block_id >= ne) break;
        if (rbs == 0) rbs = row->size; else if (rbs != -1 && rbs != row->size) rbs = -1;
        if (ebs == 0) ebs = bsize(S, row->cells[0].block_id); else if (ebs != -1 && ebs != bsize(S, row->cells[0].block_id)) ebs = -1;
        if (row->ncells > 1) {
            if (fbs == 0) fbs = bsize(S, row->cells[1].block_id);
            for (c = 1; c < row->ncells && fbs != -1; ++c) if (fbs != bsize(S, row->cells[c].block_id)) fbs = -1;
        }
        if (rbs == -1 && ebs == -1 && fbs == -1) break;
    }
    out->row_block_size = rbs; out->e_block_size = ebs; out->f_block_size = fbs;
    out->num_eliminate_blocks = ne; out->num_f_blocks = S->np_red - ne;
    out->one_f_block = (rbs == 2 && ebs == 3 && fbs == 6 && S->np_red - ne == 1);
    out->num_col_blocks = S->np_red; out->num_row_blocks = S->nr_red;
}

/* lhs cell (block1, block2): row-major dense, start (r, c), column stride n */
#define LHS_R(L, S, b) ((L)->lhs_row_layout[(b)])

static void schur_eliminate(ok_sv_state* S, lin* L, const double* b, const double* D) {
    const int ne = S->num_eliminate_blocks;
    const int n = L->lhs_n;
    const double* values = S->values;
    int i, c, j, k;
    if (n > 0) { memset(L->lhs, 0, sizeof(double) * (size_t)n * (size_t)n); memset(L->rhs, 0, sizeof(double) * (size_t)n); }
    if (D) {
        for (i = ne; i < S->np_red; ++i) {
            const int bs = bsize(S, i), r = LHS_R(L, S, i - ne);
            const double* diag = D + bpos(S, i);
            for (k = 0; k < bs; ++k) L->lhs[(r + k) * n + r + k] += diag[k] * diag[k];
        }
    }
    for (i = 0; i < L->nchunks; ++i) {
        const chunk* ch = &L->chunks[i];
        const int e_block_id = S->rows[ch->start].cells[0].block_id;
        const int e = bsize(S, e_block_id);
        double* ete = L->ete;
        memset(L->buffer, 0, sizeof(double) * (size_t)L->buffer_size);
        memset(ete, 0, sizeof(double) * (size_t)e * (size_t)e);
        if (D) { const double* diag = D + bpos(S, e_block_id); for (k = 0; k < e; ++k) ete[k * e + k] = diag[k] * diag[k]; }
        memset(L->g, 0, sizeof(double) * (size_t)e);
        /* ChunkDiagonalBlockAndGradient */
        for (j = 0; j < ch->size; ++j) {
            const ok_sv_row* row = &S->rows[ch->start + j];
            const double* E = values + row->cells[0].position;
            if (row->ncells > 1) {  /* EBlockRowOuterProduct */
                int ci, cj;
                for (ci = 1; ci < row->ncells; ++ci) {
                    const int b1 = row->cells[ci].block_id - ne, s1 = bsize(S, row->cells[ci].block_id);
                    const double* F1 = values + row->cells[ci].position;
                    ok_mtm(F1, row->size, s1, F1, row->size, s1, L->lhs, LHS_R(L, S, b1), LHS_R(L, S, b1), n, 1);
                    for (cj = ci + 1; cj < row->ncells; ++cj) {
                        const int b2 = row->cells[cj].block_id - ne, s2 = bsize(S, row->cells[cj].block_id);
                        ok_mtm(F1, row->size, s1, values + row->cells[cj].position, row->size, s2, L->lhs,
                               LHS_R(L, S, b1), LHS_R(L, S, b2), n, 1);
                    }
                }
            }
            ok_mtm(E, row->size, e, E, row->size, e, ete, 0, 0, e, 1);
            if (b) ok_mtv(E, row->size, e, b + row->position, L->g, 1);
            for (c = 1; c < row->ncells; ++c) {
                const int fb = row->cells[c].block_id, fs = bsize(S, fb);
                int off = -1;
                for (k = 0; k < ch->nf; ++k) if (ch->f_blocks[k] == fb) { off = ch->f_offsets[k]; break; }
                ok_mtm(E, row->size, e, values + row->cells[c].position, row->size, fs, L->buffer + off, 0, 0, fs, 1);
            }
        }
        ok_invert_psd_dyn(e, ete, L->inv_ete);
        if (b) {  /* UpdateRhs */
            ok_mv(L->inv_ete, e, e, L->g, L->inv_ete_g, 0);
            for (j = 0; j < ch->size; ++j) {
                const ok_sv_row* row = &S->rows[ch->start + j];
                const double* E = values + row->cells[0].position;
                memcpy(L->sj, b + row->position, sizeof(double) * (size_t)row->size);
                ok_mv(E, row->size, e, L->inv_ete_g, L->sj, -1);
                for (c = 1; c < row->ncells; ++c) {
                    const int fb = row->cells[c].block_id, fs = bsize(S, fb);
                    ok_mtv(values + row->cells[c].position, row->size, fs, L->sj, L->rhs + LHS_R(L, S, fb - ne), 1);
                }
            }
        }
        /* ChunkOuterProduct (buffer_layout iterated in ascending f block id) */
        for (j = 0; j < ch->nf; ++j) {
            const int fb1 = ch->f_blocks[j], s1 = bsize(S, fb1), b1 = fb1 - ne;
            ok_mtm(L->buffer + ch->f_offsets[j], e, s1, L->inv_ete, e, e, L->b1t_inv, 0, 0, e, 0);
            for (k = j; k < ch->nf; ++k) {
                const int fb2 = ch->f_blocks[k], s2 = bsize(S, fb2), b2 = fb2 - ne;
                ok_mmm(L->b1t_inv, s1, e, L->buffer + ch->f_offsets[k], e, s2, L->lhs, LHS_R(L, S, b1), LHS_R(L, S, b2), n, -1);
            }
        }
    }
    /* NoEBlockRowsUpdate */
    for (i = L->uneliminated_row_begins; i < S->nr_red; ++i) {
        const ok_sv_row* row = &S->rows[i];
        int ci, cj;
        for (ci = 0; ci < row->ncells; ++ci) {
            const int b1 = row->cells[ci].block_id - ne, s1 = bsize(S, row->cells[ci].block_id);
            const double* F1 = values + row->cells[ci].position;
            ok_mtm(F1, row->size, s1, F1, row->size, s1, L->lhs, LHS_R(L, S, b1), LHS_R(L, S, b1), n, 1);
            for (cj = ci + 1; cj < row->ncells; ++cj) {
                const int b2 = row->cells[cj].block_id - ne, s2 = bsize(S, row->cells[cj].block_id);
                ok_mtm(F1, row->size, s1, values + row->cells[cj].position, row->size, s2, L->lhs, LHS_R(L, S, b1),
                       LHS_R(L, S, b2), n, 1);
            }
        }
        if (!b) continue;
        for (c = 0; c < row->ncells; ++c) {
            const int fb = row->cells[c].block_id, fs = bsize(S, fb);
            ok_mtv(values + row->cells[c].position, row->size, fs, b + row->position, L->rhs + LHS_R(L, S, fb - ne), 1);
        }
    }
}

static void schur_back_substitute(ok_sv_state* S, lin* L, const double* b, const double* D, const double* z, double* y) {
    const int ne = S->num_eliminate_blocks;
    const double* values = S->values;
    int i, j, c, k;
    for (i = 0; i < L->nchunks; ++i) {
        const chunk* ch = &L->chunks[i];
        const int e_block_id = S->rows[ch->start].cells[0].block_id;
        const int e = bsize(S, e_block_id);
        double* y_block = y + bpos(S, e_block_id);
        double* ete = L->ete;
        double tmp[64];
        memset(ete, 0, sizeof(double) * (size_t)e * (size_t)e);
        if (D) { const double* diag = D + bpos(S, e_block_id); for (k = 0; k < e; ++k) ete[k * e + k] = diag[k] * diag[k]; }
        for (j = 0; j < ch->size; ++j) {
            const ok_sv_row* row = &S->rows[ch->start + j];
            const double* E = values + row->cells[0].position;
            memcpy(L->sj, b + row->position, sizeof(double) * (size_t)row->size);
            for (c = 1; c < row->ncells; ++c) {
                const int fb = row->cells[c].block_id, fs = bsize(S, fb);
                ok_mv(values + row->cells[c].position, row->size, fs, z + LHS_R(L, S, fb - ne), L->sj, -1);
            }
            ok_mtv(E, row->size, e, L->sj, y_block, 1);
            ok_mtm(E, row->size, e, E, row->size, e, ete, 0, 0, e, 1);
        }
        ok_invert_psd_dyn(e, ete, L->inv_ete);
        /* y_block = inverse_ete * y_block: RowMajor dynamic GEMV into a temporary (0 + 1*acc) */
        for (k = 0; k < e; ++k) {
            double l0 = 0.0, l1 = 0.0, cc;
            int q;
            for (q = 0; q + 2 <= e; q += 2) { l0 = l0 + L->inv_ete[k * e + q] * y_block[q]; l1 = l1 + L->inv_ete[k * e + q + 1] * y_block[q + 1]; }
            cc = l0 + l1;
            for (; q < e; ++q) cc += L->inv_ete[k * e + q] * y_block[q];
            tmp[k] = 0.0 + 1.0 * cc;
        }
        memcpy(y_block, tmp, sizeof(double) * (size_t)e);
    }
}

static int schur_solve(ok_sv_state* S, lin* L, const double* D, double* x) {
    const int n = L->lhs_n;
    double* reduced = x + S->num_effective - n;
    int term = OK_SV_LS_SUCCESS;
    if (!L->schur_reported) {
        L->schur_reported = 1;
        if (S->hooks && S->hooks->on_schur) { ok_sv_schur sc; detect_structure(S, &sc); S->hooks->on_schur(S->hooks->ctx, &sc); }
    }
    memset(x, 0, sizeof(double) * (size_t)S->num_effective);
    schur_eliminate(S, L, S->residuals, D);
    if (n > 0) {
        int ret;
        memcpy(L->lhs_copy, L->lhs, sizeof(double) * (size_t)n * (size_t)n);
        /* EigenDenseCholesky: LLT<Ref<MatrixXd>, Lower> on the column-major view of the row-major buffer */
        ret = ok_llt_lower(n, L->lhs, n);
        if (ret >= 0) term = OK_SV_LS_FAILURE;
        else ok_llt_lower_solve(n, L->lhs, n, L->rhs, L->sol);
        if (S->hooks && S->hooks->on_dense)
            S->hooks->on_dense(S->hooks->ctx, n, L->lhs_copy, L->rhs, term, term == OK_SV_LS_SUCCESS ? L->sol : NULL);
        if (term == OK_SV_LS_SUCCESS) memcpy(reduced, L->sol, sizeof(double) * (size_t)n);
    }
    if (term == OK_SV_LS_SUCCESS) schur_back_substitute(S, L, S->residuals, D, reduced, x);
    return term;
}

/* ------------------------------------------- SPARSE_NORMAL_CHOLESKY -------------------------------------- */
typedef struct term { int row, col, index; } term;
static int term_cmp(const void* a, const void* b) {
    const term *x = (const term*)a, *y = (const term*)b;
    if (x->row != y->row) return x->row < y->row ? -1 : 1;
    if (x->col != y->col) return x->col < y->col ? -1 : 1;
    return x->index < y->index ? -1 : (x->index > y->index);
}

/* InnerProductComputer::Init for LOWER_TRIANGULAR storage over the Jacobian rows plus one appended diagonal row
 * block per column block (CreateDiagonalMatrix) */
static void sparse_init(ok_sv_state* S, lin* L) {
    const int nb = S->np_red;
    int i, c1, c2, nt = 0, nnz = 0, col_nnz = 0, nnzpos = 0;
    term* terms;
    int* crs_rows;
    for (i = 0; i < S->nr_red; ++i) nt += S->rows[i].ncells * (S->rows[i].ncells + 1) / 2;
    nt += nb;
    terms = (term*)xcalloc((size_t)nt, sizeof(term));
    nt = 0;
    for (i = 0; i < S->nr_red; ++i) {
        const ok_sv_row* row = &S->rows[i];
        for (c1 = 0; c1 < row->ncells; ++c1)
            for (c2 = 0; c2 <= c1; ++c2) { terms[nt].row = row->cells[c1].block_id; terms[nt].col = row->cells[c2].block_id; terms[nt].index = nt; nt++; }
    }
    for (i = 0; i < nb; ++i) { terms[nt].row = i; terms[nt].col = i; terms[nt].index = nt; nt++; }
    L->nterms = nt;
    L->t_row = (int*)xcalloc((size_t)nt, sizeof(int));
    L->t_col = (int*)xcalloc((size_t)nt, sizeof(int));
    L->t_off = (int*)xcalloc((size_t)nt, sizeof(int));
    for (i = 0; i < nt; ++i) { L->t_row[i] = terms[i].row; L->t_col[i] = terms[i].col; }
    qsort(terms, (size_t)nt, sizeof(term), term_cmp);
    /* ComputeNonzeros */
    L->row_block_nnz = (int*)xcalloc((size_t)nb, sizeof(int));
    L->row_block_nnz[terms[0].row] = bsize(S, terms[0].col);
    nnz = bsize(S, terms[0].row) * bsize(S, terms[0].col);
    for (i = 1; i < nt; ++i) {
        if (terms[i].row != terms[i - 1].row || terms[i].col != terms[i - 1].col) {
            L->row_block_nnz[terms[i].row] += bsize(S, terms[i].col);
            nnz += bsize(S, terms[i].row) * bsize(S, terms[i].col);
        }
    }
    L->crs_nnz = nnz;
    L->crs_rows = (int*)xcalloc((size_t)S->num_effective + 1, sizeof(int));
    L->crs_cols = (int*)xcalloc((size_t)nnz, sizeof(int));
    L->crs_vals = (double*)xcalloc((size_t)nnz, sizeof(double));
    crs_rows = L->crs_rows;
    crs_rows[0] = 0;
    {
        int* p = crs_rows;
        for (i = 0; i < nb; ++i) { int j; for (j = 0; j < bsize(S, i); ++j, ++p) p[1] = p[0] + L->row_block_nnz[i]; }
    }
    /* ComputeOffsetsAndCreateResultMatrix */
#define FILL_BLOCK(cur)                                                                        \
    do {                                                                                       \
        const int rb_ = terms[cur].row, cb_ = terms[cur].col, nir_ = L->row_block_nnz[rb_];     \
        int j_, k_;                                                                            \
        L->t_off[terms[cur].index] = nnzpos + col_nnz;                                         \
        for (j_ = 0; j_ < bsize(S, rb_); ++j_)                                                 \
            for (k_ = 0; k_ < bsize(S, cb_); ++k_) L->crs_cols[nnzpos + j_ * nir_ + col_nnz + k_] = bpos(S, cb_) + k_; \
    } while (0)
    col_nnz = 0; nnzpos = 0;
    FILL_BLOCK(0);
    for (i = 1; i < nt; ++i) {
        if (terms[i - 1].row == terms[i].row && terms[i - 1].col == terms[i].col) { L->t_off[terms[i].index] = L->t_off[terms[i - 1].index]; continue; }
        if (terms[i - 1].row == terms[i].row) col_nnz += bsize(S, terms[i - 1].col);
        else { col_nnz = 0; nnzpos += L->row_block_nnz[terms[i - 1].row] * bsize(S, terms[i - 1].row); }
        FILL_BLOCK(i);
    }
#undef FILL_BLOCK
    free(terms);
    L->sp_rhs = (double*)xcalloc((size_t)S->num_effective, sizeof(double));
    L->max_blk = 1;
    for (i = 0; i < nb; ++i) if (bsize(S, i) > L->max_blk) L->max_blk = bsize(S, i);
    L->dcell = (double*)xcalloc((size_t)L->max_blk * (size_t)L->max_blk, sizeof(double));
    L->sp_init = 1;
}

static int sparse_solve(ok_sv_state* S, lin* L, const double* D, double* x) {
    const int n = S->num_effective, nb = S->np_red;
    int i, c1, c2, cursor = 0, term_type = OK_SV_LS_SUCCESS;
    if (!L->sp_init) sparse_init(S, L);
    memset(x, 0, sizeof(double) * (size_t)n);
    memset(L->sp_rhs, 0, sizeof(double) * (size_t)n);
    ok_sv_jac_left_multiply(S, S->residuals, L->sp_rhs);
    /* InnerProductComputer::Compute */
    memset(L->crs_vals, 0, sizeof(double) * (size_t)L->crs_nnz);
    for (i = 0; i < S->nr_red; ++i) {
        const ok_sv_row* row = &S->rows[i];
        for (c1 = 0; c1 < row->ncells; ++c1) {
            const int b1 = row->cells[c1].block_id, s1 = bsize(S, b1);
            const int row_nnz = L->crs_rows[bpos(S, b1) + 1] - L->crs_rows[bpos(S, b1)];
            for (c2 = 0; c2 <= c1; ++c2, ++cursor) {
                const int b2 = row->cells[c2].block_id, s2 = bsize(S, b2);
                ok_mtm(S->values + row->cells[c1].position, row->size, s1, S->values + row->cells[c2].position, row->size,
                       s2, L->crs_vals + L->t_off[cursor], 0, 0, row_nnz, 1);
            }
        }
    }
    for (i = 0; i < nb; ++i, ++cursor) {  /* appended diagonal row blocks: one cell diag(D) per column block */
        const int s = bsize(S, i), row_nnz = L->crs_rows[bpos(S, i) + 1] - L->crs_rows[bpos(S, i)];
        int k;
        memset(L->dcell, 0, sizeof(double) * (size_t)s * (size_t)s);
        if (D) for (k = 0; k < s; ++k) L->dcell[k * s + k] = D[bpos(S, i) + k];
        ok_mtm(L->dcell, s, s, L->dcell, s, s, L->crs_vals + L->t_off[cursor], 0, 0, row_nnz, 1);
    }
    /* EigenSparseCholesky: the CRS (lower block storage) mapped column-major = upper */
    if (!L->analyzed) { ok_ldlt_analyze(&L->ldlt, n, L->crs_rows, L->crs_cols); L->analyzed = 1; }
    if (!ok_ldlt_factorize(&L->ldlt, L->crs_rows, L->crs_cols, L->crs_vals)) term_type = OK_SV_LS_FAILURE;
    else ok_ldlt_solve(&L->ldlt, L->sp_rhs, x);
    if (S->hooks && S->hooks->on_sparse)
        S->hooks->on_sparse(S->hooks->ctx, n, L->crs_nnz, L->crs_rows, L->crs_cols, L->crs_vals, L->sp_rhs, term_type,
                            term_type == OK_SV_LS_SUCCESS ? x : NULL);
    return term_type;
}

/* ---------------------------------------------- DENSE_QR ----------------------------------------------------- */
/* DenseQRSolver::SolveImpl (EIGEN): lhs_ (ColMajor, rows + cols x cols) = [J; diag(D)], rhs_ = [b; 0], then EigenDenseQR
 * (HouseholderQR::solve). Single parameter block: S->values is the RowMajor DenseSparseMatrix. */
static int dense_qr_solve(ok_sv_state* S, const double* D, double* x) {
    const int rows = S->num_residuals, cols = S->num_effective, aug = rows + (D ? cols : 0);
    double* lhs = (double*)xcalloc((size_t)aug * (size_t)cols, sizeof(double));
    double* rhs = (double*)xcalloc((size_t)aug, sizeof(double));
    int i, j;
    for (i = 0; i < rows; ++i)
        for (j = 0; j < cols; ++j) lhs[(size_t)j * aug + i] = S->values[(size_t)i * cols + j];
    memcpy(rhs, S->residuals, sizeof(double) * (size_t)rows);
    if (D) for (j = 0; j < cols; ++j) lhs[(size_t)j * aug + rows + j] = D[j];
    ok_eigen_hqr_solve(aug, cols, lhs, rhs, x);
    free(lhs); free(rhs);
    return OK_SV_LS_SUCCESS;
}

/* ------------------------------------------------ dispatch ---------------------------------------------- */
void ok_sv_linear_init(ok_sv_state* S) {
    lin* L = (lin*)xcalloc(1, sizeof(lin));
    L->type = S->pb->opt.linear_solver_type;
    S->lin = L;
    if (L->type == OK_SV_DENSE_SCHUR) schur_init(S, L);
}
int ok_sv_linear_solve(ok_sv_state* S, const double* D, double* x) {
    lin* L = (lin*)S->lin;
    if (L->type == OK_SV_DENSE_SCHUR) return schur_solve(S, L, D, x);
    if (L->type == OK_SV_SPARSE_NORMAL_CHOLESKY) return sparse_solve(S, L, D, x);
    if (L->type == OK_SV_DENSE_QR) return dense_qr_solve(S, D, x);
    return OK_SV_LS_FATAL_ERROR;
}
void ok_sv_linear_free(ok_sv_state* S) {
    lin* L = (lin*)S->lin;
    int i;
    if (!L) return;
    for (i = 0; i < L->nchunks; ++i) { free(L->chunks[i].f_blocks); free(L->chunks[i].f_offsets); }
    free(L->chunks); free(L->buffer); free(L->b1t_inv); free(L->lhs_row_layout); free(L->lhs); free(L->lhs_copy);
    free(L->rhs); free(L->sol); free(L->ete); free(L->inv_ete); free(L->g); free(L->inv_ete_g); free(L->sj);
    free(L->t_row); free(L->t_col); free(L->t_off); free(L->row_block_nnz); free(L->crs_rows); free(L->crs_cols);
    free(L->crs_vals); free(L->sp_rhs); free(L->dcell);
    if (L->analyzed) ok_ldlt_free(&L->ldlt);
    free(L);
    S->lin = NULL;
}
