/* OK_PORT_SOURCES: check_ok_solve.c ok_solve.c ok_solve_linear.c ok_blas.c ok_dense.c ok_sparse.c ok_amd.c ok_err.c ok_param.c ok_cam.c ok_kin.c ok_imu.c ok_time.c ok_eigen.c ok_twopose.c ok_graph.c */
/* Bit-exactness harness for okvis_port module 4 (the Ceres solver as driven by ViGraph::optimise).
 *
 *   check_ok_solve <seq_label> <fixtures_dir (unused, "-")> <dump_dir> [max_solves]
 *
 * Replays every snapshotted Solve() of <dump_dir>/solve.bin (patch 0008, layout in ok_solve.h): the Problem is
 * rebuilt from the PROBLEM record (modules 1-3 error terms; the TwoPose* terms of module 5 are evaluated natively
 * from their payload (patch 0009) and every evaluation is verified against the ORACLE record the reference still
 * writes for them -- "twopose" below; dumps without the payload (patch 0008 only) replay the ORACLE records
 * instead, which then only verify the parameters handed to the terms), ok_sv_solve runs, and at every
 * observation point the C state is compared bitwise with the dumped record: the reduced/reordered program,
 * every iteration summary and the hashes (or, at level 2, the full vectors) of x, delta, step, gradient,
 * scaling, residuals; the Dogleg and Gauss-Newton internals; the Schur structure, the reduced dense system and
 * its solution; the sparse J^T J, rhs and solution; the END record (termination, costs, hash of all parameter
 * blocks after the solve). Level-0 solves (no snapshot) are counted only.
 * Prints one line per kind "  <kind>: <mismatching values>/<compared values> (<failing records>/<records>)" and
 * as the LAST line "<seq_label>: <mismatches>/<total>"; exit 0 iff mismatches == 0. OK_DEBUG=1 prints the first
 * mismatches.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ok_graph.h"
#include "ok_solve.h"

enum { R_SOLVE = 1, R_PROBLEM = 2, R_REDUCED = 3, R_ITER = 4, R_DOGLEG = 5, R_GN = 6, R_SCHUR = 7, R_DENSE = 8,
       R_SPARSE = 9, R_ORACLE = 10, R_END = 11, R_PROGRAM = 12 };

typedef struct rec { uint32_t tag; uint64_t len; unsigned char* p; } rec;
typedef struct counts { long bad, tot, recs, badrecs; } counts;

static counts C_reduced, C_iter, C_dogleg, C_gn, C_schur, C_dense, C_sparse, C_oracle, C_end, C_term;
static long G_solves_total, G_solves_replayed, G_level0, G_struct_bad;
static int G_debug, G_dbg_printed;
static const char* G_kind = "";
static uint64_t G_solve_id;

/* ---- byte-buffer readers ---- */
typedef struct cur { const unsigned char* p; size_t off, len; int bad; } cur;
static uint32_t cu32(cur* c) { uint32_t v = 0; if (c->off + 4 <= c->len) memcpy(&v, c->p + c->off, 4); else c->bad = 1; c->off += 4; return v; }
static int32_t ci32(cur* c) { return (int32_t)cu32(c); }
static uint64_t cu64(cur* c) { uint64_t v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static double cf64(cur* c) { double v = 0; if (c->off + 8 <= c->len) memcpy(&v, c->p + c->off, 8); else c->bad = 1; c->off += 8; return v; }
static const double* cf64n(cur* c, size_t n) { const double* r = (const double*)(c->p + c->off); if (c->off + 8 * n > c->len) { c->bad = 1; r = NULL; } c->off += 8 * n; return r; }
static const int32_t* ci32n(cur* c, size_t n) { const int32_t* r = (const int32_t*)(c->p + c->off); if (c->off + 4 * n > c->len) { c->bad = 1; r = NULL; } c->off += 4 * n; return r; }

static void mism(counts* c, const char* what, long idx, double got, double want) {
    c->bad++;
    if (G_debug && G_dbg_printed < 80) {
        G_dbg_printed++;
        printf("    MISMATCH solve %llu %s[%ld]: got %.17g want %.17g\n", (unsigned long long)G_solve_id, what, idx, got, want);
    }
}
static int cmp_d(counts* c, const char* what, double got, double want) {
    c->tot++;
    if (memcmp(&got, &want, 8) != 0) { mism(c, what, 0, got, want); return 1; }
    return 0;
}
static int cmp_i(counts* c, const char* what, long got, long want) {
    c->tot++;
    if (got != want) { mism(c, what, 0, (double)got, (double)want); return 1; }
    return 0;
}
static int cmp_u64(counts* c, const char* what, uint64_t got, uint64_t want) {
    c->tot++;
    if (got != want) { c->bad++; if (G_debug && G_dbg_printed < 80) { G_dbg_printed++; printf("    MISMATCH solve %llu %s: hash %016llx want %016llx\n", (unsigned long long)G_solve_id, what, (unsigned long long)got, (unsigned long long)want); } return 1; }
    return 0;
}
static int cmp_vec(counts* c, const char* what, const double* got, const double* want, long n) {
    long i; int bad = 0;
    if (!got || !want) { c->tot += n; c->bad += n; return 1; }
    for (i = 0; i < n; ++i) { c->tot++; if (memcmp(&got[i], &want[i], 8) != 0) { bad++; mism(c, what, i, got[i], want[i]); } }
    return bad;
}
static uint64_t hashv(const double* v, long n) { return v ? ok_fnv(v, 8 * (size_t)n) : 0ULL; }

/* ---- the per-solve expectation queue ---- */
typedef struct solve_ctx {
    rec* recs; int nrec, next; int level, bad;
    ok_sv_problem* pb;
} solve_ctx;

static rec* next_rec(solve_ctx* X, uint32_t tag, const char* what) {
    while (X->next < X->nrec) {
        rec* r = &X->recs[X->next++];
        if (r->tag == tag) return r;
        /* unexpected record kind: structural mismatch */
        G_struct_bad++;
        if (G_debug && G_dbg_printed < 80) { G_dbg_printed++; printf("    STRUCT solve %llu: expected %s (tag %u) got tag %u\n", (unsigned long long)G_solve_id, what, tag, r->tag); }
    }
    G_struct_bad++;
    if (G_debug && G_dbg_printed < 80) { G_dbg_printed++; printf("    STRUCT solve %llu: expected %s (tag %u) but no records left\n", (unsigned long long)G_solve_id, what, tag); }
    return NULL;
}

/* ---- hooks ---- */
static int h_oracle(void* ctx, const ok_sv_resid* rb, const double* const* params, double* residuals, double* const* jacobians) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_ORACLE, "ORACLE");
    cur c;
    uint64_t ptr, hp, h = 1469598103934665603ULL;
    uint32_t have_jac, nb, nres, nonnull[OK_SV_MAXB];
    uint32_t k;
    int bad = 0;
    C_oracle.recs++;
    memset(residuals, 0, sizeof(double) * (size_t)rb->nres);
    if (!r) { C_oracle.badrecs++; return 1; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    ptr = cu64(&c); hp = cu64(&c); have_jac = cu32(&c); nb = cu32(&c);
    for (k = 0; k < nb && k < OK_SV_MAXB; ++k) nonnull[k] = cu32(&c);
    nres = cu32(&c);
    G_kind = "oracle";
    bad += cmp_u64(&C_oracle, "oracle.ptr", rb->ptr, ptr);
    for (k = 0; k < (uint32_t)rb->nb; ++k) h = ok_fnv_combine(h, ok_fnv(params[k], 8 * (size_t)X->pb->p[rb->blk[k]].size));
    bad += cmp_u64(&C_oracle, "oracle.params", h, hp);
    bad += cmp_i(&C_oracle, "oracle.nres", rb->nres, nres);
    bad += cmp_i(&C_oracle, "oracle.have_jac", jacobians != NULL, have_jac);
    if ((uint32_t)rb->nb != nb) { C_oracle.badrecs++; return 1; }
    {
        const double* res = cf64n(&c, nres);
        if (res) memcpy(residuals, res, 8 * (size_t)(nres < (uint32_t)rb->nres ? nres : (uint32_t)rb->nres));
        for (k = 0; k < nb; ++k) {
            const uint32_t size = (uint32_t)X->pb->p[rb->blk[k]].size;
            bad += cmp_i(&C_oracle, "oracle.jac_nonnull", jacobians && jacobians[k] ? 1 : 0, nonnull[k]);
            if (nonnull[k]) {
                const double* J = cf64n(&c, (size_t)nres * size);
                if (J && jacobians && jacobians[k]) memcpy(jacobians[k], J, 8 * (size_t)nres * size);
            }
        }
    }
    if (bad) C_oracle.badrecs++;
    return 1;
}

/* a natively evaluated TwoPose* term: compare its raw outputs with the ORACLE record of the reference */
static void h_term(void* ctx, const ok_sv_resid* rb, const double* const* params, const double* residuals, double* const* jacobians) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_ORACLE, "ORACLE");
    cur c;
    uint64_t ptr, hp, h = 1469598103934665603ULL;
    uint32_t have_jac, nb, nres, nonnull[OK_SV_MAXB];
    uint32_t k;
    int bad = 0;
    C_term.recs++;
    if (!r) { C_term.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    ptr = cu64(&c); hp = cu64(&c); have_jac = cu32(&c); nb = cu32(&c);
    for (k = 0; k < nb && k < OK_SV_MAXB; ++k) nonnull[k] = cu32(&c);
    nres = cu32(&c);
    G_kind = "twopose";
    bad += cmp_u64(&C_term, "twopose.ptr", rb->ptr, ptr);
    for (k = 0; k < (uint32_t)rb->nb; ++k) h = ok_fnv_combine(h, ok_fnv(params[k], 8 * (size_t)X->pb->p[rb->blk[k]].size));
    bad += cmp_u64(&C_term, "twopose.params", h, hp);
    bad += cmp_i(&C_term, "twopose.nres", rb->nres, nres);
    bad += cmp_i(&C_term, "twopose.have_jac", jacobians != NULL, have_jac);
    if ((uint32_t)rb->nb != nb || nres != (uint32_t)rb->nres) { C_term.badrecs++; X->bad = 1; return; }
    bad += cmp_vec(&C_term, "twopose.residuals", residuals, cf64n(&c, nres), nres);
    for (k = 0; k < nb; ++k) {
        const uint32_t size = (uint32_t)X->pb->p[rb->blk[k]].size;
        bad += cmp_i(&C_term, "twopose.jac_nonnull", jacobians && jacobians[k] ? 1 : 0, nonnull[k]);
        if (nonnull[k]) {
            const double* J = cf64n(&c, (size_t)nres * size);
            if (jacobians && jacobians[k]) bad += cmp_vec(&C_term, "twopose.J", jacobians[k], J, (long)nres * size);
        }
    }
    if (bad) { C_term.badrecs++; X->bad = 1; }
}

static void h_reduced(void* ctx, const ok_sv_problem* pb, double fixed_cost, int num_eliminate_blocks, int np_red,
                      const int* param_order, int nr_red, const int* resid_order) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_REDUCED, "REDUCED");
    cur c;
    uint32_t status, ne, np, nr, i;
    int bad = 0;
    C_reduced.recs++;
    if (!r) { C_reduced.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    status = cu32(&c);
    bad += cmp_d(&C_reduced, "fixed_cost", fixed_cost, cf64(&c));
    ne = cu32(&c); cu32(&c);
    bad += cmp_i(&C_reduced, "num_eliminate_blocks", num_eliminate_blocks, ne);
    np = cu32(&c);
    bad += cmp_i(&C_reduced, "np_reduced", np_red, np);
    for (i = 0; i < np; ++i) { const uint64_t ptr = cu64(&c); if ((int)i < np_red) bad += cmp_u64(&C_reduced, "param_order", pb->p[param_order[i]].ptr, ptr); }
    nr = cu32(&c);
    bad += cmp_i(&C_reduced, "nr_reduced", nr_red, nr);
    for (i = 0; i < nr; ++i) { const uint64_t ptr = cu64(&c); if ((int)i < nr_red) bad += cmp_u64(&C_reduced, "resid_order", pb->r[resid_order[i]].ptr, ptr); }
    (void)status;
    if (bad) { C_reduced.badrecs++; X->bad = 1; }
}

static void h_iter(void* ctx, const ok_sv_iter* it) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_ITER, "ITER");
    cur c;
    int bad = 0;
    uint32_t np, ne, nres, cand, dval, sval;
    C_iter.recs++;
    if (!r) { C_iter.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    bad += cmp_i(&C_iter, "iteration", it->iteration, cu32(&c));
    bad += cmp_d(&C_iter, "cost", it->cost, cf64(&c));
    bad += cmp_d(&C_iter, "cost_change", it->cost_change, cf64(&c));
    bad += cmp_d(&C_iter, "gradient_max_norm", it->gradient_max_norm, cf64(&c));
    bad += cmp_d(&C_iter, "gradient_norm", it->gradient_norm, cf64(&c));
    bad += cmp_d(&C_iter, "step_norm", it->step_norm, cf64(&c));
    bad += cmp_d(&C_iter, "relative_decrease", it->relative_decrease, cf64(&c));
    bad += cmp_d(&C_iter, "trust_region_radius", it->trust_region_radius, cf64(&c));
    bad += cmp_i(&C_iter, "step_is_valid", it->step_is_valid, cu32(&c));
    bad += cmp_i(&C_iter, "step_is_successful", it->step_is_successful, cu32(&c));
    bad += cmp_i(&C_iter, "step_is_nonmonotonic", it->step_is_nonmonotonic, cu32(&c));
    bad += cmp_d(&C_iter, "model_cost_change", it->model_cost_change, cf64(&c));
    bad += cmp_d(&C_iter, "candidate_cost", it->candidate_cost, cf64(&c));
    bad += cmp_d(&C_iter, "x_cost", it->x_cost, cf64(&c));
    bad += cmp_d(&C_iter, "minimum_cost", it->minimum_cost, cf64(&c));
    np = cu32(&c); ne = cu32(&c); nres = cu32(&c);
    bad += cmp_i(&C_iter, "num_parameters", it->num_parameters, np);
    bad += cmp_i(&C_iter, "num_effective_parameters", it->num_effective_parameters, ne);
    bad += cmp_i(&C_iter, "num_residuals", it->num_residuals, nres);
    cand = cu32(&c); dval = cu32(&c); sval = cu32(&c);
    bad += cmp_i(&C_iter, "candidate_valid", it->candidate_valid, cand);
    bad += cmp_i(&C_iter, "delta_valid", it->delta_valid, dval);
    bad += cmp_i(&C_iter, "step_valid", it->step_valid, sval);
    if (X->level >= 1 && np == (uint32_t)it->num_parameters && ne == (uint32_t)it->num_effective_parameters && nres == (uint32_t)it->num_residuals) {
        bad += cmp_u64(&C_iter, "h_x", hashv(it->x, np), cu64(&c));
        bad += cmp_u64(&C_iter, "h_candidate_x", cand ? hashv(it->candidate_x, np) : 0, cu64(&c));
        bad += cmp_u64(&C_iter, "h_delta", dval ? hashv(it->delta, ne) : 0, cu64(&c));
        bad += cmp_u64(&C_iter, "h_step", sval ? hashv(it->trust_region_step, ne) : 0, cu64(&c));
        bad += cmp_u64(&C_iter, "h_gradient", hashv(it->gradient, ne), cu64(&c));
        bad += cmp_u64(&C_iter, "h_scaling", hashv(it->jacobian_scaling, ne), cu64(&c));
        bad += cmp_u64(&C_iter, "h_residuals", hashv(it->residuals, nres), cu64(&c));
        if (X->level >= 2) {
            bad += cmp_vec(&C_iter, "x", it->x, cf64n(&c, np), np);
            bad += cmp_vec(&C_iter, "gradient", it->gradient, cf64n(&c, ne), ne);
            bad += cmp_vec(&C_iter, "scaling", it->jacobian_scaling, cf64n(&c, ne), ne);
            bad += cmp_vec(&C_iter, "residuals", it->residuals, cf64n(&c, nres), nres);
            if (sval) bad += cmp_vec(&C_iter, "step", it->trust_region_step, cf64n(&c, ne), ne);
            if (dval) bad += cmp_vec(&C_iter, "delta", it->delta, cf64n(&c, ne), ne);
            if (cand) bad += cmp_vec(&C_iter, "candidate_x", it->candidate_x, cf64n(&c, np), np);
        }
    }
    if (bad) { C_iter.badrecs++; X->bad = 1; }
}

static void h_dogleg(void* ctx, const ok_sv_dogleg* d) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_DOGLEG, "DOGLEG");
    cur c;
    int bad = 0;
    uint32_t n;
    C_dogleg.recs++;
    if (!r) { C_dogleg.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    bad += cmp_i(&C_dogleg, "reuse", d->reuse, cu32(&c));
    bad += cmp_i(&C_dogleg, "termination", d->termination, cu32(&c));
    bad += cmp_d(&C_dogleg, "radius", d->radius, cf64(&c));
    bad += cmp_d(&C_dogleg, "mu", d->mu, cf64(&c));
    bad += cmp_d(&C_dogleg, "alpha", d->alpha, cf64(&c));
    bad += cmp_d(&C_dogleg, "dogleg_step_norm", d->dogleg_step_norm, cf64(&c));
    bad += cmp_d(&C_dogleg, "gradient_norm", d->gradient_norm, cf64(&c));
    bad += cmp_d(&C_dogleg, "gauss_newton_norm", d->gauss_newton_norm, cf64(&c));
    n = cu32(&c);
    bad += cmp_i(&C_dogleg, "n", d->n, n);
    if (n == (uint32_t)d->n) {
        bad += cmp_u64(&C_dogleg, "h_gradient", hashv(d->gradient, n), cu64(&c));
        bad += cmp_u64(&C_dogleg, "h_gn", hashv(d->gauss_newton_step, n), cu64(&c));
        bad += cmp_u64(&C_dogleg, "h_diagonal", hashv(d->diagonal, n), cu64(&c));
        bad += cmp_u64(&C_dogleg, "h_step", d->step ? hashv(d->step, n) : 0, cu64(&c));
        if (X->level >= 2) {
            bad += cmp_vec(&C_dogleg, "gradient", d->gradient, cf64n(&c, n), n);
            bad += cmp_vec(&C_dogleg, "gn", d->gauss_newton_step, cf64n(&c, n), n);
            bad += cmp_vec(&C_dogleg, "diagonal", d->diagonal, cf64n(&c, n), n);
            if (d->step) bad += cmp_vec(&C_dogleg, "step", d->step, cf64n(&c, n), n);
        }
    }
    if (bad) { C_dogleg.badrecs++; X->bad = 1; }
}

static void h_gn(void* ctx, const ok_sv_gn* g) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_GN, "GN");
    cur c;
    int bad = 0;
    uint32_t n;
    C_gn.recs++;
    if (!r) { C_gn.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    bad += cmp_i(&C_gn, "termination", g->termination, cu32(&c));
    bad += cmp_i(&C_gn, "mu_increases", g->mu_increases, cu32(&c));
    bad += cmp_d(&C_gn, "mu", g->mu, cf64(&c));
    n = cu32(&c);
    bad += cmp_i(&C_gn, "n", g->n, n);
    if (n == (uint32_t)g->n) {
        bad += cmp_u64(&C_gn, "h_gn", g->gauss_newton_step ? hashv(g->gauss_newton_step, n) : 0, cu64(&c));
        bad += cmp_u64(&C_gn, "h_lm_diagonal", hashv(g->lm_diagonal, n), cu64(&c));
        if (X->level >= 2) {
            bad += cmp_vec(&C_gn, "lm_diagonal", g->lm_diagonal, cf64n(&c, n), n);
            if (g->gauss_newton_step) bad += cmp_vec(&C_gn, "gn", g->gauss_newton_step, cf64n(&c, n), n);
        }
    }
    if (bad) { C_gn.badrecs++; X->bad = 1; }
}

static void h_schur(void* ctx, const ok_sv_schur* s) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_SCHUR, "SCHUR");
    cur c;
    int bad = 0;
    C_schur.recs++;
    if (!r) { C_schur.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    bad += cmp_i(&C_schur, "row_block_size", s->row_block_size, ci32(&c));
    bad += cmp_i(&C_schur, "e_block_size", s->e_block_size, ci32(&c));
    bad += cmp_i(&C_schur, "f_block_size", s->f_block_size, ci32(&c));
    bad += cmp_i(&C_schur, "num_eliminate_blocks", s->num_eliminate_blocks, cu32(&c));
    bad += cmp_i(&C_schur, "num_f_blocks", s->num_f_blocks, cu32(&c));
    bad += cmp_i(&C_schur, "one_f_block", s->one_f_block, cu32(&c));
    bad += cmp_i(&C_schur, "num_col_blocks", s->num_col_blocks, cu32(&c));
    bad += cmp_i(&C_schur, "num_row_blocks", s->num_row_blocks, cu32(&c));
    if (bad) { C_schur.badrecs++; X->bad = 1; }
}

static void h_dense(void* ctx, int n, const double* lhs, const double* rhs, int termination, const double* solution) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_DENSE, "DENSE");
    cur c;
    int bad = 0;
    uint32_t nn;
    C_dense.recs++;
    if (!r) { C_dense.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    nn = cu32(&c);
    bad += cmp_i(&C_dense, "n", n, nn);
    if (nn == (uint32_t)n) {
        bad += cmp_u64(&C_dense, "h_lhs", hashv(lhs, (long)n * n), cu64(&c));
        bad += cmp_u64(&C_dense, "h_rhs", hashv(rhs, n), cu64(&c));
        if (X->level >= 2) {
            bad += cmp_vec(&C_dense, "lhs", lhs, cf64n(&c, (size_t)n * n), (long)n * n);
            bad += cmp_vec(&C_dense, "rhs", rhs, cf64n(&c, n), n);
        }
        bad += cmp_i(&C_dense, "termination", termination, cu32(&c));
        bad += cmp_u64(&C_dense, "h_solution", solution ? hashv(solution, n) : 0, cu64(&c));
        if (X->level >= 2 && solution) bad += cmp_vec(&C_dense, "solution", solution, cf64n(&c, n), n);
    }
    if (bad) { C_dense.badrecs++; X->bad = 1; }
}

static void h_sparse(void* ctx, int n, int nnz, const int* rows, const int* cols, const double* values, const double* rhs,
                     int termination, const double* x) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_SPARSE, "SPARSE");
    cur c;
    int bad = 0;
    uint32_t nn, nz;
    C_sparse.recs++;
    if (!r) { C_sparse.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    nn = cu32(&c); nz = cu32(&c); cu32(&c); cu32(&c);
    bad += cmp_i(&C_sparse, "n", n, nn);
    bad += cmp_i(&C_sparse, "nnz", nnz, nz);
    if (nn == (uint32_t)n && nz == (uint32_t)nnz) {
        bad += cmp_u64(&C_sparse, "h_values", hashv(values, nnz), cu64(&c));
        bad += cmp_u64(&C_sparse, "h_rhs", hashv(rhs, n), cu64(&c));
        bad += cmp_u64(&C_sparse, "h_rows", ok_fnv(rows, 4 * (size_t)(n + 1)), cu64(&c));
        bad += cmp_u64(&C_sparse, "h_cols", ok_fnv(cols, 4 * (size_t)nnz), cu64(&c));
        if (X->level >= 2) {
            const int32_t* rr = ci32n(&c, (size_t)n + 1);
            const int32_t* cc = ci32n(&c, (size_t)nnz);
            long i;
            if (rr) for (i = 0; i <= n; ++i) if (cmp_i(&C_sparse, "rows", rows[i], rr[i])) { bad++; break; }
            if (cc) for (i = 0; i < nnz; ++i) if (cmp_i(&C_sparse, "cols", cols[i], cc[i])) { bad++; break; }
            bad += cmp_vec(&C_sparse, "values", values, cf64n(&c, nnz), nnz);
            bad += cmp_vec(&C_sparse, "rhs", rhs, cf64n(&c, n), n);
        }
        bad += cmp_i(&C_sparse, "termination", termination, cu32(&c));
        bad += cmp_u64(&C_sparse, "h_x", x ? hashv(x, n) : 0, cu64(&c));
        if (X->level >= 2 && x) bad += cmp_vec(&C_sparse, "x", x, cf64n(&c, n), n);
    }
    if (bad) { C_sparse.badrecs++; X->bad = 1; }
}

static void h_end(void* ctx, const ok_sv_end* e) {
    solve_ctx* X = (solve_ctx*)ctx;
    rec* r = next_rec(X, R_END, "END");
    cur c;
    int bad = 0;
    C_end.recs++;
    if (!r) { C_end.badrecs++; return; }
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    bad += cmp_i(&C_end, "termination_type", e->termination_type, cu32(&c));
    bad += cmp_i(&C_end, "num_iterations", e->num_iterations, cu32(&c));
    bad += cmp_d(&C_end, "initial_cost", e->initial_cost, cf64(&c));
    bad += cmp_d(&C_end, "final_cost", e->final_cost, cf64(&c));
    bad += cmp_d(&C_end, "fixed_cost", e->fixed_cost, cf64(&c));
    {   /* Ceres hashes Problem::GetParameterBlocks(), i.e. the pointer-sorted ParameterMap order */
        uint64_t h = 1469598103934665603ULL;
        int i, j, n = X->pb->np;
        int* order = (int*)malloc(sizeof(int) * (size_t)(n ? n : 1));
        for (i = 0; i < n; ++i) order[i] = i;
        for (i = 1; i < n; ++i) { int t = order[i]; for (j = i - 1; j >= 0 && X->pb->p[order[j]].ptr > X->pb->p[t].ptr; --j) order[j + 1] = order[j]; order[j + 1] = t; }
        for (i = 0; i < n; ++i) h = ok_fnv_combine(h, ok_fnv(X->pb->p[order[i]].x, 8 * (size_t)X->pb->p[order[i]].size));
        free(order);
        (void)e->param_hash;
        bad += cmp_u64(&C_end, "param_hash", h, cu64(&c));
    }
    bad += cmp_i(&C_end, "np", e->np, cu32(&c));
    bad += cmp_i(&C_end, "num_successful_steps", e->num_successful_steps, cu32(&c));
    bad += cmp_i(&C_end, "num_unsuccessful_steps", e->num_unsuccessful_steps, cu32(&c));
    if (bad) { C_end.badrecs++; X->bad = 1; }
}

/* ---- Problem construction from the PROBLEM record ---- */
static int rd_cam(cur* c, ok_cam* cam) {
    uint32_t tag, w, h, nd, i;
    double f[4], d[OK_CAM_MAX_DIST];
    memset(d, 0, sizeof d);
    tag = cu32(c); w = cu32(c); h = cu32(c);
    for (i = 0; i < 4; ++i) f[i] = cf64(c);
    nd = cu32(c);
    if (nd > OK_CAM_MAX_DIST) return 0;
    for (i = 0; i < nd; ++i) d[i] = cf64(c);
    if (tag != OK_CAM_RADTAN && tag != OK_CAM_EQUIDISTANT && tag != OK_CAM_NODIST) return 0;
    ok_cam_init(cam, (int)tag, (int)w, (int)h, f[0], f[1], f[2], f[3], d);
    return 1;
}
static void rd_imu_snapshot(cur* c, ok_imu_error* e) {
    uint64_t n = cu64(c), i;
    ok_imu_meas* m;
    ok_imu_params p;
    ok_time t0, t1;
    double dq[4];
    uint32_t j, ndps;
    if (n > (1u << 24)) { c->bad = 1; n = 0; }
    m = (ok_imu_meas*)malloc((size_t)(n ? n : 1) * sizeof(ok_imu_meas));
    for (i = 0; i < n; ++i) {
        m[i].t.sec = cu32(c); m[i].t.nsec = cu32(c);
        for (j = 0; j < 3; ++j) m[i].gyr[j] = cf64(c);
        for (j = 0; j < 3; ++j) m[i].acc[j] = cf64(c);
    }
    p.sigma_g_c = cf64(c); p.sigma_a_c = cf64(c); p.sigma_gw_c = cf64(c); p.sigma_aw_c = cf64(c);
    p.g = cf64(c); p.g_max = cf64(c); p.a_max = cf64(c);
    t0.sec = cu32(c); t0.nsec = cu32(c); t1.sec = cu32(c); t1.nsec = cu32(c);
    ok_imu_error_init(e, m, (size_t)n, &p, t0, t1);
    free(m);
    for (j = 0; j < 4; ++j) dq[j] = cf64(c);
    e->delta_q.x = dq[0]; e->delta_q.y = dq[1]; e->delta_q.z = dq[2]; e->delta_q.w = dq[3];
#define RDN(arr, k) do { uint32_t q_; for (q_ = 0; q_ < (k); ++q_) (arr)[q_] = cf64(c); } while (0)
    RDN(e->C_integral, 9); RDN(e->C_doubleintegral, 9); RDN(e->acc_integral, 3); RDN(e->acc_doubleintegral, 3);
    RDN(e->cross, 9); RDN(e->dalpha_db_g, 9); RDN(e->dv_db_g, 9); RDN(e->dp_db_g, 9); RDN(e->P_delta, 225); RDN(e->sb_ref, 9);
    e->redo = (int)cu32(c); e->redo_counter = (int)cu32(c);
    RDN(e->information, 225); RDN(e->sqrt_information, 225);
    ndps = cu32(c);
    if (ndps > 4) { c->bad = 1; ndps = 0; }
    for (j = 0; j < ndps; ++j) RDN(e->dPdsigma[j], 225);
#undef RDN
}

static ok_sv_problem* build_problem(const rec* r, const ok_sv_options* opt, int* ok) {
    cur c;
    ok_sv_problem* pb = (ok_sv_problem*)calloc(1, sizeof(ok_sv_problem));
    uint32_t np, nr, i, k;
    *ok = 1;
    c.p = r->p; c.off = 0; c.len = r->len; c.bad = 0;
    pb->opt = *opt;
    np = cu32(&c);
    pb->np = (int)np;
    pb->p = (ok_sv_param*)calloc(np ? np : 1, sizeof(ok_sv_param));
    for (i = 0; i < np && !c.bad; ++i) {
        ok_sv_param* p = &pb->p[i];
        const double* x;
        p->ptr = cu64(&c); p->size = (int)cu32(&c); p->tangent = (int)cu32(&c); p->kind = (int)cu32(&c); p->constant = (int)cu32(&c);
        if (p->size < 1 || p->size > 9) { c.bad = 1; break; }
        x = cf64n(&c, (size_t)p->size);
        p->x = (double*)calloc((size_t)p->size, sizeof(double));
        if (x) memcpy(p->x, x, 8 * (size_t)p->size);
        p->index = -1;
    }
    nr = cu32(&c);
    pb->nr = (int)nr;
    pb->r = (ok_sv_resid*)calloc(nr ? nr : 1, sizeof(ok_sv_resid));
    for (i = 0; i < nr && !c.bad; ++i) {
        ok_sv_resid* rb = &pb->r[i];
        uint64_t plen;
        size_t pend;
        rb->ptr = cu64(&c); rb->type = (int)cu32(&c); rb->loss = (int)cu32(&c); rb->nb = (int)cu32(&c);
        if (rb->nb < 1 || rb->nb > OK_SV_MAXB) { c.bad = 1; break; }
        for (k = 0; k < (uint32_t)rb->nb; ++k) { rb->blk[k] = (int)cu32(&c); if (rb->blk[k] < 0 || rb->blk[k] >= pb->np) c.bad = 1; }
        rb->nres = (int)cu32(&c);
        plen = cu64(&c);
        pend = c.off + (size_t)plen;
        switch (rb->type) {
            case OK_SV_T_REPROJ: {
                ok_cam cam; double meas[2], info[4];
                if (!rd_cam(&c, &cam)) { c.bad = 1; break; }
                meas[0] = cf64(&c); meas[1] = cf64(&c);
                for (k = 0; k < 4; ++k) info[k] = cf64(&c);  /* row-major 2x2 == column-major (symmetric) */
                ok_reproj_err_init(&rb->term.reproj, &cam, meas, info);
                break;
            }
            case OK_SV_T_IMU: rd_imu_snapshot(&c, &rb->term.imu); break;
            case OK_SV_T_POSE: case OK_SV_T_RELPOSE: {
                double co[7], info_rm[36], sq_rm[36], info[36], sq[36];
                ok_tf T;
                uint32_t n;
                int rr, cc;
                for (k = 0; k < 7; ++k) co[k] = cf64(&c);
                n = cu32(&c);
                if (n != 6) { c.bad = 1; break; }
                for (k = 0; k < 36; ++k) info_rm[k] = cf64(&c);
                for (k = 0; k < 36; ++k) sq_rm[k] = cf64(&c);
                for (rr = 0; rr < 6; ++rr) for (cc = 0; cc < 6; ++cc) { info[rr + 6 * cc] = info_rm[rr * 6 + cc]; sq[rr + 6 * cc] = sq_rm[rr * 6 + cc]; }
                ok_tf_set_coeffs(&T, co, 1);
                if (rb->type == OK_SV_T_POSE) { ok_pose_err_init_info(&rb->term.pose, &T, info); memcpy(rb->term.pose.sqrt_info, sq, sizeof sq); }
                else { ok_relpose_err_init_info(&rb->term.relpose, info, &T); memcpy(rb->term.relpose.sqrt_info, sq, sizeof sq); }
                break;
            }
            case OK_SV_T_SAB: {
                double meas[9], info_rm[81], sq_rm[81], info[81], sq[81];
                uint32_t n; int rr, cc;
                for (k = 0; k < 9; ++k) meas[k] = cf64(&c);
                n = cu32(&c);
                if (n != 9) { c.bad = 1; break; }
                for (k = 0; k < 81; ++k) info_rm[k] = cf64(&c);
                for (k = 0; k < 81; ++k) sq_rm[k] = cf64(&c);
                for (rr = 0; rr < 9; ++rr) for (cc = 0; cc < 9; ++cc) { info[rr + 9 * cc] = info_rm[rr * 9 + cc]; sq[rr + 9 * cc] = sq_rm[rr * 9 + cc]; }
                ok_sab_err_init_info(&rb->term.sab, meas, info); memcpy(rb->term.sab.sqrt_info, sq, sizeof sq);
                break;
            }
            case OK_SV_T_HPOINT: {
                double meas[4], info_rm[9], sq_rm[9], info[9], sq[9];
                uint32_t n; int rr, cc;
                for (k = 0; k < 4; ++k) meas[k] = cf64(&c);
                n = cu32(&c);
                if (n != 3) { c.bad = 1; break; }
                for (k = 0; k < 9; ++k) info_rm[k] = cf64(&c);
                for (k = 0; k < 9; ++k) sq_rm[k] = cf64(&c);
                for (rr = 0; rr < 3; ++rr) for (cc = 0; cc < 3; ++cc) { info[rr + 3 * cc] = info_rm[rr * 3 + cc]; sq[rr + 3 * cc] = sq_rm[rr * 3 + cc]; }
                ok_hpoint_err_init_info(&rb->term.hpoint, meas, info); memcpy(rb->term.hpoint.sqrt_info, sq, sizeof sq);
                break;
            }
            case OK_SV_T_TWOPOSE: case OK_SV_T_TWOPOSE_CONST: case OK_SV_T_TWOPOSE_EXT: case OK_SV_T_TWOPOSE_EXT_CONST:
                /* patch 0009 payload: evaluated natively (verified by h_term); none: replay the ORACLE records */
                if (plen > 0 && ok_tp_payload_read(c.p + c.off, (size_t)plen, rb->type, &rb->term.tp, &rb->term.tpx) > 0) {
                    if (rb->type == OK_SV_T_TWOPOSE_CONST) rb->term.tp.is_computed = 1;
                    if (rb->type == OK_SV_T_TWOPOSE_EXT_CONST) rb->term.tpx.is_computed = 1;
                    rb->oracle = 0;
                } else rb->oracle = 1;
                break;
            default: rb->oracle = 1; break;
        }
        c.off = pend;
        if (rb->loss == OK_SV_LOSS_OTHER) c.bad = 1;
    }
    if (c.bad) *ok = 0;
    return pb;
}
static void free_problem(ok_sv_problem* pb) {
    int i;
    if (!pb) return;
    for (i = 0; i < pb->np; ++i) free(pb->p[i].x);
    for (i = 0; i < pb->nr; ++i) if (pb->r[i].type == OK_SV_T_IMU) ok_imu_error_free(&pb->r[i].term.imu);
    free(pb->p); free(pb->r); free(pb);
}

static int rd_record(FILE* f, rec* r) {
    uint32_t tag; uint64_t len;
    if (fread(&tag, 4, 1, f) != 1 || fread(&len, 8, 1, f) != 1) return 0;
    if (len > (1ull << 32)) return 0;
    r->tag = tag; r->len = len;
    r->p = (unsigned char*)malloc(len ? (size_t)len : 1);
    if (len && fread(r->p, 1, (size_t)len, f) != (size_t)len) { free(r->p); return 0; }
    return 1;
}

static void print_kind(const char* name, const counts* c) {
    printf("  %s: %ld/%ld (%ld/%ld records)\n", name, c->bad, c->tot, c->badrecs, c->recs);
}

int main(int argc, char** argv) {
    const char* label = argc > 1 ? argv[1] : "solve";
    const char* dir = argc > 3 ? argv[3] : ".";
    long max_solves = argc > 4 ? atol(argv[4]) : -1;
    char path[1024];
    FILE* f;
    rec r;
    long solves_bad = 0, tot, bad;
    G_debug = getenv("OK_DEBUG") != NULL;
    snprintf(path, sizeof path, "%s/solve.bin", dir);
    f = fopen(path, "rb");
    if (!f) { printf("%s: 0/0\n", label); fprintf(stderr, "cannot open %s\n", path); return 1; }
    while (rd_record(f, &r)) {
        ok_sv_options opt;
        cur c;
        uint64_t sid;
        uint32_t level;
        if (r.tag != R_SOLVE) { free(r.p); G_struct_bad++; continue; }
        c.p = r.p; c.off = 0; c.len = r.len; c.bad = 0;
        sid = cu64(&c); level = cu32(&c);
        memset(&opt, 0, sizeof opt);
        opt.linear_solver_type = (int)cu32(&c); cu32(&c); cu32(&c);
        opt.max_num_iterations = (int)cu32(&c); cu32(&c);
        opt.function_tolerance = cf64(&c); opt.gradient_tolerance = cf64(&c); opt.parameter_tolerance = cf64(&c);
        opt.initial_trust_region_radius = cf64(&c); opt.max_trust_region_radius = cf64(&c); opt.min_trust_region_radius = cf64(&c);
        opt.min_relative_decrease = cf64(&c); opt.min_lm_diagonal = cf64(&c); opt.max_lm_diagonal = cf64(&c);
        opt.jacobi_scaling = (int)cu32(&c); cu32(&c); opt.max_num_consecutive_invalid_steps = (int)cu32(&c);
        free(r.p);
        G_solves_total++;
        G_solve_id = sid;
        if (max_solves > 0 && G_solves_replayed >= max_solves) level = 0;
        if (level == 0) {  /* summary-only solve: skip to END */
            G_level0++;
            while (rd_record(f, &r)) { const uint32_t t = r.tag; free(r.p); if (t == R_END) break; }
            continue;
        }
        {
            solve_ctx X;
            ok_sv_hooks hooks;
            int cap = 64, ok = 1;
            memset(&X, 0, sizeof X);
            X.recs = (rec*)malloc(sizeof(rec) * (size_t)cap);
            X.level = (int)level;
            while (rd_record(f, &r)) {
                if (X.nrec == cap) { cap *= 2; X.recs = (rec*)realloc(X.recs, sizeof(rec) * (size_t)cap); }
                X.recs[X.nrec++] = r;
                if (r.tag == R_END) break;
            }
            if (X.nrec == 0 || X.recs[0].tag != R_PROBLEM) { G_struct_bad++; ok = 0; }
            if (ok) {
                X.pb = build_problem(&X.recs[0], &opt, &ok);
                X.next = 1;
                /* PROGRAM record: the Problem's program order (the PROBLEM record lists the parameter blocks in
                 * pointer order); reorder the parameter array and check the residual order */
                if (ok && X.nrec > 1 && X.recs[1].tag == R_PROGRAM) {
                    cur c;
                    uint32_t np, nr, i, j;
                    c.p = X.recs[1].p; c.off = 0; c.len = X.recs[1].len; c.bad = 0;
                    np = cu32(&c);
                    if (np != (uint32_t)X.pb->np) ok = 0;
                    else {
                        ok_sv_param* newp = (ok_sv_param*)calloc(np ? np : 1, sizeof(ok_sv_param));
                        int* newidx = (int*)calloc(np ? np : 1, sizeof(int));
                        for (i = 0; i < np && ok; ++i) {
                            const uint64_t ptr = cu64(&c);
                            int found = -1;
                            for (j = 0; j < np; ++j) if (X.pb->p[j].ptr == ptr) { found = (int)j; break; }
                            if (found < 0) { ok = 0; break; }
                            newp[i] = X.pb->p[found];
                            newidx[found] = (int)i;
                        }
                        if (ok) {
                            int r, k;
                            for (r = 0; r < X.pb->nr; ++r)
                                for (k = 0; k < X.pb->r[r].nb; ++k) X.pb->r[r].blk[k] = newidx[X.pb->r[r].blk[k]];
                            memcpy(X.pb->p, newp, sizeof(ok_sv_param) * np);
                        }
                        free(newp); free(newidx);
                    }
                    nr = cu32(&c);
                    if (ok && nr != (uint32_t)X.pb->nr) ok = 0;
                    for (i = 0; ok && i < nr; ++i) { const uint64_t ptr = cu64(&c); if (X.pb->r[i].ptr != ptr) ok = 0; }
                    if (!ok) { G_struct_bad++; fprintf(stderr, "solve %llu: PROGRAM record inconsistent with PROBLEM\n", (unsigned long long)sid); }
                    X.next = 2;
                } else if (ok) {
                    G_struct_bad++;
                    fprintf(stderr, "solve %llu: no PROGRAM record (dump predates the program-order record)\n", (unsigned long long)sid);
                    ok = 0;
                }
                if (ok) {
                    memset(&hooks, 0, sizeof hooks);
                    hooks.ctx = &X;
                    hooks.oracle = h_oracle; hooks.on_reduced = h_reduced; hooks.on_iter = h_iter; hooks.on_dogleg = h_dogleg;
                    hooks.on_gn = h_gn; hooks.on_schur = h_schur; hooks.on_dense = h_dense; hooks.on_sparse = h_sparse;
                    hooks.on_end = h_end; hooks.on_term = h_term;
                    ok_sv_solve(X.pb, &hooks);
                    G_solves_replayed++;
                    if (X.next < X.nrec) {  /* records the C solve never consumed */
                        G_struct_bad += X.nrec - X.next;
                        if (G_debug && G_dbg_printed < 80) { G_dbg_printed++; printf("    STRUCT solve %llu: %d unconsumed records (next tag %u)\n", (unsigned long long)sid, X.nrec - X.next, X.recs[X.next].tag); }
                        X.bad = 1;
                    }
                    if (X.bad) solves_bad++;
                } else { G_struct_bad++; solves_bad++; fprintf(stderr, "solve %llu: cannot rebuild the problem\n", (unsigned long long)sid); }
                free_problem(X.pb);
            }
            {
                int i;
                for (i = 0; i < X.nrec; ++i) free(X.recs[i].p);
                free(X.recs);
            }
        }
    }
    fclose(f);
    print_kind("reduced", &C_reduced); print_kind("iter", &C_iter); print_kind("dogleg", &C_dogleg); print_kind("gn", &C_gn);
    print_kind("schur", &C_schur); print_kind("dense", &C_dense); print_kind("sparse", &C_sparse); print_kind("oracle", &C_oracle);
    print_kind("twopose", &C_term); print_kind("end", &C_end);
    printf("  solves: %ld replayed of %ld (%ld summary-only), %ld with mismatches, %ld structural errors\n", G_solves_replayed,
           G_solves_total, G_level0, solves_bad, G_struct_bad);
    bad = C_reduced.bad + C_iter.bad + C_dogleg.bad + C_gn.bad + C_schur.bad + C_dense.bad + C_sparse.bad + C_oracle.bad + C_term.bad + C_end.bad + G_struct_bad;
    tot = C_reduced.tot + C_iter.tot + C_dogleg.tot + C_gn.tot + C_schur.tot + C_dense.tot + C_sparse.tot + C_oracle.tot + C_term.tot + C_end.tot + G_struct_bad;
    printf("%s: %ld/%ld\n", label, bad, tot);
    return bad == 0 && tot > 0 ? 0 : 1;
}
