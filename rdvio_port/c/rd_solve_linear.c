/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause AND MPL-2.0 */
/* See rd_solve.h. Part 2: the linear solver. SPARSE_SCHUR = SchurEliminator<Dynamic,Dynamic,Dynamic> (naive small_blas kernels,
 * InvertPSDMatrix<Dynamic>; copy of okvis_port/c/ok_solve_linear.c) into a BlockRandomAccessSparseMatrix (cells = the f-block
 * pairs (i <= j) in std::set order, each a row-major dense block), converted with ToCompressedRowSparseMatrixTranspose (LOWER
 * block storage) and factorised by EigenSparseCholesky (SimplicialLDLT<Upper, NaturalOrdering>, ok_sparse.c; the AMD ordering
 * of the Schur columns was already applied to the parameter blocks). Ceres 2.2.0, BSD-3-Clause, Copyright 2023 Google Inc.;
 * Eigen models MPL-2.0. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "ok_blas.h"
#include "ok_dense.h"
#include "rd_solve_internal.h"
#include "rd_static.h"

typedef struct chunk { int start, size, nf; int* f_blocks; int* f_offsets; } chunk;  /* f_blocks ascending (std::map) */

typedef struct lin {
    int type;
    /* Schur eliminator */
    int nchunks; chunk* chunks; int buffer_size; double* buffer; double* b1t_inv; int uneliminated_row_begins;
    int e3, f3;   /* SchurEliminator<2,3,*> (e3) / <2,3,3> (f3): the kernels whose template dimensions are all static (rd_static.c) */
    int nf; int* lhs_row_layout; int lhs_n; double* rhs; double* sol; int schur_reported;
    double *ete, *inv_ete, *g, *inv_ete_g, *sj; int max_e, max_row;
    /* BlockRandomAccessSparseMatrix: the f-block pairs (i <= j) in std::set order, each a row-major dense cell */
    int* cell_off;                       /* nf x nf: offset of cell (i, j) into cell_vals, -1 if absent */
    double* cell_vals; int ncell_vals;
    /* ToCompressedRowSparseMatrixTranspose: LOWER block CRS, entry k takes cell_vals[crs_src[k]] */
    int crs_nnz; int* crs_rows; int* crs_cols; int* crs_src; double* crs_vals;
    ok_ldlt ldlt; int analyzed;
} lin;

static void* xcalloc(size_t n, size_t sz) { return calloc(n ? n : 1, sz); }
static int bsize(const rd_sv_state* S, int reduced_index) { return S->pb->p[S->porder[reduced_index]].tangent; }
static int bpos(const rd_sv_state* S, int reduced_index) { return S->pb->p[S->porder[reduced_index]].delta_offset; }

/* ------------------------- BlockRandomAccessSparseMatrix: layout of the reduced system -------------------------- */
static int cmp_pair_key(const void* a, const void* b) { const long x = *(const long*)a, y = *(const long*)b; return (x > y) - (x < y); }

/* SparseSchurComplementSolver::InitStorage: block_pairs = {(i,i)} + every pair of f blocks sharing an e block (per chunk of rows
 * with the same e block) + the pairs (i <= j) of the cells of the rows that do not contain an e block. */
static void sparse_lhs_init(rd_sv_state* S, lin* L) {
    const int ne = S->num_eliminate_blocks, nf = L->nf;
    long* keys;
    size_t nkeys = 0, cap = (size_t)nf, i, j;
    int r = 0, k, c;
    size_t total_cells = 0;
    for (i = 0; i < (size_t)S->nr_red; ++i) { const size_t nc = (size_t)S->rows[i].ncells; cap += nc * nc; total_cells += nc; }
    keys = (long*)xcalloc(cap + 1, sizeof(long));
#define PUSH_KEY(v) do { if (nkeys == cap) { cap = cap * 2 + 16; keys = (long*)realloc(keys, cap * sizeof(long)); } keys[nkeys++] = (v); } while (0)
    for (k = 0; k < nf; ++k) PUSH_KEY((long)k * nf + k);
    while (r < S->nr_red) {
        const int e_block_id = S->rows[r].cells[0].block_id;
        size_t start, n;
        int* fb;
        if (e_block_id >= ne) break;
        fb = (int*)xcalloc(total_cells + 1, sizeof(int));
        n = 0;
        for (; r < S->nr_red; ++r) {
            const rd_sv_row* row = &S->rows[r];
            if (row->cells[0].block_id != e_block_id) break;
            for (c = 1; c < row->ncells; ++c) fb[n++] = row->cells[c].block_id - ne;
        }
        /* sort + unique (insertion sort: short lists) */
        for (i = 1; i < n; ++i) { int t = fb[i]; long q; for (q = (long)i - 1; q >= 0 && fb[q] > t; --q) fb[q + 1] = fb[q]; fb[q + 1] = t; }
        for (start = 0, i = 0; i < n; ++i) if (i == 0 || fb[i] != fb[i - 1]) fb[start++] = fb[i];
        for (i = 0; i < start; ++i) for (j = i + 1; j < start; ++j) PUSH_KEY((long)fb[i] * nf + fb[j]);
        free(fb);
    }
    for (; r < S->nr_red; ++r) {
        const rd_sv_row* row = &S->rows[r];
        int c1, c2;
        for (c1 = 0; c1 < row->ncells; ++c1)
            for (c2 = 0; c2 < row->ncells; ++c2) {
                const int b1 = row->cells[c1].block_id - ne, b2 = row->cells[c2].block_id - ne;
                if (b1 <= b2) PUSH_KEY((long)b1 * nf + b2);
            }
    }
    qsort(keys, nkeys, sizeof(long), cmp_pair_key);
    L->cell_off = (int*)xcalloc((size_t)nf * (size_t)nf, sizeof(int));
    for (i = 0; i < (size_t)nf * (size_t)nf; ++i) L->cell_off[i] = -1;
    {
        int off = 0;
        for (i = 0; i < nkeys; ++i) {
            if (i > 0 && keys[i] == keys[i - 1]) continue;
            L->cell_off[keys[i]] = off;
            off += bsize(S, ne + (int)(keys[i] / nf)) * bsize(S, ne + (int)(keys[i] % nf));
        }
        L->ncell_vals = off;
    }
    free(keys);
    L->cell_vals = (double*)xcalloc((size_t)L->ncell_vals, sizeof(double));
    /* ToCompressedRowSparseMatrixTranspose: CRS row = scalar column (c, a) of the upper block matrix; the entries of that row
     * are the blocks (r, c), r <= c ascending, with the columns of block r; value = cell(r, c)[b][a] */
    {
        int nnz = 0, row_i = 0, cc, rr, a, bb;
        for (cc = 0; cc < nf; ++cc)
            for (rr = 0; rr <= cc; ++rr) if (L->cell_off[(size_t)rr * nf + cc] >= 0) nnz += bsize(S, ne + rr) * bsize(S, ne + cc);
        L->crs_nnz = nnz;
        L->crs_rows = (int*)xcalloc((size_t)L->lhs_n + 1, sizeof(int));
        L->crs_cols = (int*)xcalloc((size_t)nnz, sizeof(int));
        L->crs_src = (int*)xcalloc((size_t)nnz, sizeof(int));
        L->crs_vals = (double*)xcalloc((size_t)nnz, sizeof(double));
        nnz = 0;
        for (cc = 0; cc < nf; ++cc)
            for (a = 0; a < bsize(S, ne + cc); ++a, ++row_i) {
                for (rr = 0; rr <= cc; ++rr) {
                    const int off = L->cell_off[(size_t)rr * nf + cc];
                    if (off < 0) continue;
                    for (bb = 0; bb < bsize(S, ne + rr); ++bb) {
                        L->crs_cols[nnz] = L->lhs_row_layout[rr] + bb;
                        L->crs_src[nnz] = off + bb * bsize(S, ne + cc) + a;
                        ++nnz;
                    }
                }
                L->crs_rows[row_i + 1] = nnz;
            }
    }
}
/* BlockRandomAccessSparseMatrix::GetCell: NULL when the pair is not in the layout (the eliminator then skips it) */
static double* lhs_cell(const lin* L, int b1, int b2, int* col_stride, const rd_sv_state* S) {
    const int off = L->cell_off[(size_t)b1 * L->nf + b2];
    if (off < 0) return NULL;
    *col_stride = bsize(S, S->num_eliminate_blocks + b2);
    return L->cell_vals + off;
}
#define LHS_OP(fn, A, ra, ca, B, rb, cb, b1, b2, op)                                   \
    do {                                                                               \
        int st_;                                                                       \
        double* C_ = lhs_cell(L, (b1), (b2), &st_, S);                                 \
        if (C_) (fn)(A, ra, ca, B, rb, cb, C_, 0, 0, st_, op);                           \
    } while (0)

/* the ok_mtm / ok_mmm signature, for the all-static kernels of SchurEliminator<2,3,3> (small_blas.h: the Eigen path) */
static void mtm_st(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int sr, int sc, int cs, int op) {
    (void)ra; (void)ca; (void)rb; (void)cb;
    rd_st_mtm_2_3(A, B, C + (long)sr * cs + sc, cs, op);
}
static void mtm33_st(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int sr, int sc, int cs, int op) {
    (void)ra; (void)ca; (void)rb; (void)cb;
    rd_st_mtm_3_3(A, B, C + (long)sr * cs + sc, cs, op);
}
static void mmm33_st(const double* A, int ra, int ca, const double* B, int rb, int cb, double* C, int sr, int sc, int cs, int op) {
    (void)ra; (void)ca; (void)rb; (void)cb;
    rd_st_mmm_3_3(A, B, C + (long)sr * cs + sc, cs, op);
}

static void detect_structure(const rd_sv_state* S, rd_sv_schur* out);

/* ---------------------------------------------- Schur: Init ---------------------------------------------- */
static void schur_init(rd_sv_state* S, lin* L) {
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
            const rd_sv_row* row = &S->rows[r + ch->size];
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
    sparse_lhs_init(S, L);
    {
        rd_sv_schur sc;
        detect_structure(S, &sc);
        /* SchurEliminatorBase::Create: row 2 + e 3 -> <2,3,3> / <2,3,4> / <2,3,6> / <2,3,9> / <2,3,Dynamic> (only f = 3 or anything else
         * occurs here); every other structure is the fully dynamic eliminator (<2,Dynamic,Dynamic> included: no all-static kernel) */
        L->e3 = sc.row_block_size == 2 && sc.e_block_size == 3;
        L->f3 = L->e3 && sc.f_block_size == 3;
    }
    L->rhs = (double*)xcalloc((size_t)L->lhs_n, sizeof(double));
    L->sol = (double*)xcalloc((size_t)L->lhs_n, sizeof(double));
    L->ete = (double*)xcalloc((size_t)L->max_e * (size_t)L->max_e, sizeof(double));
    L->inv_ete = (double*)xcalloc((size_t)L->max_e * (size_t)L->max_e, sizeof(double));
    L->g = (double*)xcalloc((size_t)L->max_e, sizeof(double));
    L->inv_ete_g = (double*)xcalloc((size_t)L->max_e, sizeof(double));
    L->sj = (double*)xcalloc((size_t)L->max_row, sizeof(double));
}

/* DetectStructure (for the SCHUR record only) */
static void detect_structure(const rd_sv_state* S, rd_sv_schur* out) {
    const int ne = S->num_eliminate_blocks;
    int r, rbs = 0, ebs = 0, fbs = 0;
    for (r = 0; r < S->nr_red; ++r) {
        const rd_sv_row* row = &S->rows[r];
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

static void schur_eliminate(rd_sv_state* S, lin* L, const double* b, const double* D) {
    const int ne = S->num_eliminate_blocks;
    const int n = L->lhs_n;
    const double* values = S->values;
    int i, c, j, k;
    memset(L->cell_vals, 0, sizeof(double) * (size_t)L->ncell_vals);   /* BlockRandomAccessSparseMatrix::SetZero */
    memset(L->rhs, 0, sizeof(double) * (size_t)n);
    if (D) {   /* D^T D on the diagonal blocks of the f blocks */
        for (i = ne; i < S->np_red; ++i) {
            const int bs = bsize(S, i);
            const double* diag = D + bpos(S, i);
            int st;
            double* cell = lhs_cell(L, i - ne, i - ne, &st, S);
            if (cell) for (k = 0; k < bs; ++k) cell[k * st + k] += diag[k] * diag[k];
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
            const rd_sv_row* row = &S->rows[ch->start + j];
            const double* E = values + row->cells[0].position;
            if (row->ncells > 1) {  /* EBlockRowOuterProduct */
                int ci, cj;
                for (ci = 1; ci < row->ncells; ++ci) {
                    const int b1 = row->cells[ci].block_id - ne, s1 = bsize(S, row->cells[ci].block_id);
                    const double* F1 = values + row->cells[ci].position;
                    LHS_OP(L->f3 ? mtm_st : ok_mtm, F1, row->size, s1, F1, row->size, s1, b1, b1, 1);
                    for (cj = ci + 1; cj < row->ncells; ++cj) {
                        const int b2 = row->cells[cj].block_id - ne, s2 = bsize(S, row->cells[cj].block_id);
                        LHS_OP(L->f3 ? mtm_st : ok_mtm, F1, row->size, s1, values + row->cells[cj].position, row->size, s2, b1, b2, 1);
                    }
                }
            }
            (L->e3 ? mtm_st : ok_mtm)(E, row->size, e, E, row->size, e, ete, 0, 0, e, 1);
            if (b) ok_mtv(E, row->size, e, b + row->position, L->g, 1);
            for (c = 1; c < row->ncells; ++c) {
                const int fb = row->cells[c].block_id, fs = bsize(S, fb);
                int off = -1;
                for (k = 0; k < ch->nf; ++k) if (ch->f_blocks[k] == fb) { off = ch->f_offsets[k]; break; }
                (L->f3 ? mtm_st : ok_mtm)(E, row->size, e, values + row->cells[c].position, row->size, fs, L->buffer + off, 0, 0, fs, 1);
            }
        }
        if (L->e3) rd_st_invert_psd3(ete, L->inv_ete); else ok_invert_psd_dyn(e, ete, L->inv_ete);
        if (b) {  /* UpdateRhs */
            ok_mv(L->inv_ete, e, e, L->g, L->inv_ete_g, 0);
            for (j = 0; j < ch->size; ++j) {
                const rd_sv_row* row = &S->rows[ch->start + j];
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
            (L->f3 ? mtm33_st : ok_mtm)(L->buffer + ch->f_offsets[j], e, s1, L->inv_ete, e, e, L->b1t_inv, 0, 0, e, 0);
            for (k = j; k < ch->nf; ++k) {
                const int fb2 = ch->f_blocks[k], s2 = bsize(S, fb2), b2 = fb2 - ne;
                LHS_OP(L->f3 ? mmm33_st : ok_mmm, L->b1t_inv, s1, e, L->buffer + ch->f_offsets[k], e, s2, b1, b2, -1);
            }
        }
    }
    /* NoEBlockRowsUpdate */
    for (i = L->uneliminated_row_begins; i < S->nr_red; ++i) {
        const rd_sv_row* row = &S->rows[i];
        int ci, cj;
        for (ci = 0; ci < row->ncells; ++ci) {
            const int b1 = row->cells[ci].block_id - ne, s1 = bsize(S, row->cells[ci].block_id);
            const double* F1 = values + row->cells[ci].position;
            LHS_OP(ok_mtm, F1, row->size, s1, F1, row->size, s1, b1, b1, 1);
            for (cj = ci + 1; cj < row->ncells; ++cj) {
                const int b2 = row->cells[cj].block_id - ne, s2 = bsize(S, row->cells[cj].block_id);
                LHS_OP(ok_mtm, F1, row->size, s1, values + row->cells[cj].position, row->size, s2, b1, b2, 1);
            }
        }
        if (!b) continue;
        for (c = 0; c < row->ncells; ++c) {
            const int fb = row->cells[c].block_id, fs = bsize(S, fb);
            ok_mtv(values + row->cells[c].position, row->size, fs, b + row->position, L->rhs + LHS_R(L, S, fb - ne), 1);
        }
    }
}

static void schur_back_substitute(rd_sv_state* S, lin* L, const double* b, const double* D, const double* z, double* y) {
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
            const rd_sv_row* row = &S->rows[ch->start + j];
            const double* E = values + row->cells[0].position;
            memcpy(L->sj, b + row->position, sizeof(double) * (size_t)row->size);
            for (c = 1; c < row->ncells; ++c) {
                const int fb = row->cells[c].block_id, fs = bsize(S, fb);
                ok_mv(values + row->cells[c].position, row->size, fs, z + LHS_R(L, S, fb - ne), L->sj, -1);
            }
            ok_mtv(E, row->size, e, L->sj, y_block, 1);
            (L->e3 ? mtm_st : ok_mtm)(E, row->size, e, E, row->size, e, ete, 0, 0, e, 1);
        }
        if (L->e3) rd_st_invert_psd3(ete, L->inv_ete); else ok_invert_psd_dyn(e, ete, L->inv_ete);
        /* y_block = inverse_ete * y_block: RowMajor dynamic GEMV into a temporary (0 + 1*acc) */
        if (L->e3) {   /* fixed-size `InvertPSDMatrix<3>(..) * y_block`: left fold */
            rd_st_inv_times_y(L->inv_ete, y_block, tmp);
            memcpy(y_block, tmp, sizeof(double) * 3);
            continue;
        }
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

static int schur_solve(rd_sv_state* S, lin* L, const double* D, double* x) {
    const int n = L->lhs_n;
    double* reduced = x + S->num_effective - n;
    int term = RD_SV_LS_SUCCESS;
    if (!L->schur_reported) {
        L->schur_reported = 1;
        if (S->hooks && S->hooks->on_schur) { rd_sv_schur sc; detect_structure(S, &sc); S->hooks->on_schur(S->hooks->ctx, &sc); }
    }
    memset(x, 0, sizeof(double) * (size_t)S->num_effective);
    schur_eliminate(S, L, S->residuals, D);
    if (n > 0) {   /* SparseSchurComplementSolver::SolveReducedLinearSystem (no hook record when there are no f blocks) */
        int k;
        for (k = 0; k < L->crs_nnz; ++k) L->crs_vals[k] = L->cell_vals[L->crs_src[k]];
        if (!L->analyzed) { ok_ldlt_analyze(&L->ldlt, n, L->crs_rows, L->crs_cols); L->analyzed = 1; }
        if (!ok_ldlt_factorize(&L->ldlt, L->crs_rows, L->crs_cols, L->crs_vals)) term = RD_SV_LS_FAILURE;
        else ok_ldlt_solve(&L->ldlt, L->rhs, L->sol);
        if (S->hooks && S->hooks->on_sparse)
            S->hooks->on_sparse(S->hooks->ctx, n, L->crs_nnz, L->crs_rows, L->crs_cols, L->crs_vals, L->rhs, term,
                                term == RD_SV_LS_SUCCESS ? L->sol : NULL);
        if (term == RD_SV_LS_SUCCESS) memcpy(reduced, L->sol, sizeof(double) * (size_t)n);
    }
    if (term == RD_SV_LS_SUCCESS) schur_back_substitute(S, L, S->residuals, D, reduced, x);
    return term;
}

/* ------------------------------------------------ dispatch ---------------------------------------------- */
void rd_sv_linear_init(rd_sv_state* S) {
    lin* L = (lin*)xcalloc(1, sizeof(lin));
    L->type = S->pb->opt.linear_solver_type;
    S->lin = L;
    if (L->type == RD_SV_SPARSE_SCHUR) schur_init(S, L);
}
int rd_sv_linear_solve(rd_sv_state* S, const double* D, double* x) {
    lin* L = (lin*)S->lin;
    if (L->type == RD_SV_SPARSE_SCHUR) return schur_solve(S, L, D, x);
    return RD_SV_LS_FATAL_ERROR;
}
void rd_sv_linear_free(rd_sv_state* S) {
    lin* L = (lin*)S->lin;
    int i;
    if (!L) return;
    for (i = 0; i < L->nchunks; ++i) { free(L->chunks[i].f_blocks); free(L->chunks[i].f_offsets); }
    free(L->chunks); free(L->buffer); free(L->b1t_inv); free(L->lhs_row_layout); free(L->rhs); free(L->sol); free(L->ete);
    free(L->inv_ete); free(L->g); free(L->inv_ete_g); free(L->sj); free(L->cell_off); free(L->cell_vals); free(L->crs_rows);
    free(L->crs_cols); free(L->crs_src); free(L->crs_vals);
    if (L->analyzed) ok_ldlt_free(&L->ldlt);
    free(L);
    S->lin = NULL;
}
