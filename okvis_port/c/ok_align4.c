/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* See ok_align4.h. */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "ok_align4.h"
#include "ok_gps.h"

/* Sensitivity checks of the oracle (runs/okvis2x_port/align4/build_oracle.sh mutN): -DOK_ALIGN4_MUTATE=N builds a deliberately
 * wrong evaluation order that okvis_align4_test MUST detect. 1: fused multiply-add in the Jet product; 2: v + (w * uv + cross);
 * 3: single-accumulator redux; 4: the GEMV kernel also for one remaining column; 5: no 0.0 + (signed zero) in the dot;
 * 6: no skip of a zero right-hand side entry in the triangular solve; 7: Q^T applied in reverse order; 8: (tau * ess) * tmp
 * replaced by tau * (ess * tmp). */
#ifndef OK_ALIGN4_MUTATE
#define OK_ALIGN4_MUTATE 0
#endif

/* ------------------------------------------------ Jet<double, 7> ------------------------------------------------ */
typedef struct jet { double a; double v[7]; } jet;
static jet jet_const(double a) { jet j; int i; j.a = a; for (i = 0; i < 7; ++i) j.v[i] = 0.0; return j; }
static jet jet_var(double a, int k) { jet j = jet_const(a); j.v[k] = 1.0; return j; }
static jet jet_add(const jet* f, const jet* g) { jet r; int i; r.a = f->a + g->a; for (i = 0; i < 7; ++i) r.v[i] = f->v[i] + g->v[i]; return r; }
static jet jet_sub(const jet* f, const jet* g) { jet r; int i; r.a = f->a - g->a; for (i = 0; i < 7; ++i) r.v[i] = f->v[i] - g->v[i]; return r; }
static jet jet_mul(const jet* f, const jet* g) {  /* Jet(f.a * g.a, f.a * g.v + f.v * g.a) */
    jet r; int i;
    r.a = f->a * g->a;
#if OK_ALIGN4_MUTATE == 1
    for (i = 0; i < 7; ++i) r.v[i] = fma(f->a, g->v[i], f->v[i] * g->a);
#else
    for (i = 0; i < 7; ++i) r.v[i] = f->a * g->v[i] + f->v[i] * g->a;
#endif
    return r;
}
/* Eigen cross(): (l1 r2 - l2 r1, l2 r0 - l0 r2, l0 r1 - l1 r0) with Jet scalars */
static void jet_cross(const jet l[3], const jet r[3], jet out[3]) {
    jet a, b;
    a = jet_mul(&l[1], &r[2]); b = jet_mul(&l[2], &r[1]); out[0] = jet_sub(&a, &b);
    a = jet_mul(&l[2], &r[0]); b = jet_mul(&l[0], &r[2]); out[1] = jet_sub(&a, &b);
    a = jet_mul(&l[0], &r[1]); b = jet_mul(&l[1], &r[0]); out[2] = jet_sub(&a, &b);
}

void ok_align4_residual(const ok_align4_term* t, const double x[7], double res[3], double* jac) {
    if (!jac) {                                   /* T = double: Quaternion<double>::_transformVector */
        const double w = x[6], vec[3] = {x[3], x[4], x[5]};
        double uv[3], c[3], out[3];
        int i;
        uv[0] = vec[1] * t->pW[2] - vec[2] * t->pW[1];
        uv[1] = vec[2] * t->pW[0] - vec[0] * t->pW[2];
        uv[2] = vec[0] * t->pW[1] - vec[1] * t->pW[0];
        for (i = 0; i < 3; ++i) uv[i] += uv[i];
        c[0] = vec[1] * uv[2] - vec[2] * uv[1];
        c[1] = vec[2] * uv[0] - vec[0] * uv[2];
        c[2] = vec[0] * uv[1] - vec[1] * uv[0];
        for (i = 0; i < 3; ++i) out[i] = (t->pW[i] + w * uv[i]) + c[i];
        for (i = 0; i < 3; ++i) res[i] = t->pG[i] - (out[i] + x[i]);
        return;
    } else {
        jet r[3], vec[3], w, pW[3], uv[3], c[3], pin[3], err;
        int i, k;
        for (i = 0; i < 3; ++i) { r[i] = jet_var(x[i], i); vec[i] = jet_var(x[3 + i], 3 + i); pW[i] = jet_const(t->pW[i]); }
        w = jet_var(x[6], 6);
        jet_cross(vec, pW, uv);
        for (i = 0; i < 3; ++i) uv[i] = jet_add(&uv[i], &uv[i]);
        jet_cross(vec, uv, c);
        for (i = 0; i < 3; ++i) {
            jet wu = jet_mul(&w, &uv[i]), s;
#if OK_ALIGN4_MUTATE == 2
            wu = jet_add(&wu, &c[i]);
            s = jet_add(&pW[i], &wu);
#else
            s = jet_add(&pW[i], &wu);
            s = jet_add(&s, &c[i]);
#endif
            pin[i] = jet_add(&s, &r[i]);
        }
        for (i = 0; i < 3; ++i) {
            jet g = jet_const(t->pG[i]);
            err = jet_sub(&g, &pin[i]);
            res[i] = err.a;
            for (k = 0; k < 7; ++k) jac[i * 7 + k] = err.v[k];
        }
    }
}

/* ------------------------------------------- the Ceres problem ------------------------------------------------- */
int ok_align4dof_ceres(int n, const double* pG, const double* pW, const ok_tf* T_init, ok_tf* T_out, const ok_sv_hooks* hooks) {
    ok_sv_problem pb;
    ok_align4_term* terms;
    double x[7];
    int i, term;
    memset(&pb, 0, sizeof pb);
    terms = (ok_align4_term*)calloc((size_t)(n ? n : 1), sizeof(ok_align4_term));
    pb.p = (ok_sv_param*)calloc(1, sizeof(ok_sv_param));
    pb.r = (ok_sv_resid*)calloc((size_t)(n ? n : 1), sizeof(ok_sv_resid));
    /* PoseParameterBlock(T_GW_init): the coefficients [r, q.coeffs()] */
    memcpy(x, T_init->r, sizeof(double) * 3);
    x[3] = T_init->q.x; x[4] = T_init->q.y; x[5] = T_init->q.z; x[6] = T_init->q.w;
    pb.np = 1;
    pb.p[0].ptr = 1; pb.p[0].size = 7; pb.p[0].tangent = 4; pb.p[0].kind = OK_SV_KIND_POSE4; pb.p[0].constant = 0; pb.p[0].x = x;
    pb.p[0].index = -1;
    for (i = 0; i < n; ++i) {
        ok_sv_resid* rb = &pb.r[i];
        memcpy(terms[i].pG, pG + 3 * i, sizeof(double) * 3);
        memcpy(terms[i].pW, pW + 3 * i, sizeof(double) * 3);
        rb->ptr = (uint64_t)(100 + i); rb->type = OK_SV_T_ALIGN4; rb->loss = OK_SV_LOSS_CAUCHY3; rb->nb = 1; rb->nres = 3;
        rb->blk[0] = 0;
        rb->term.align4 = &terms[i];
    }
    pb.nr = n;
    pb.opt.linear_solver_type = OK_SV_DENSE_QR;                 /* Solver::Options defaults + DENSE_QR + 100 iterations */
    pb.opt.max_num_iterations = 100;
    pb.opt.function_tolerance = 1e-6; pb.opt.gradient_tolerance = 1e-10; pb.opt.parameter_tolerance = 1e-8;
    pb.opt.initial_trust_region_radius = 1e4; pb.opt.max_trust_region_radius = 1e16; pb.opt.min_trust_region_radius = 1e-32;
    pb.opt.min_relative_decrease = 1e-3; pb.opt.min_lm_diagonal = 1e-6; pb.opt.max_lm_diagonal = 1e32;
    pb.opt.jacobi_scaling = 1; pb.opt.max_num_consecutive_invalid_steps = 5;
    pb.opt.strategy_lm = 1;
    term = ok_sv_solve(&pb, hooks);
    ok_tf_convert(T_out, x);                                    /* estimate(): Transformation(r, q) of the block */
    free(terms); free(pb.p); free(pb.r);
    return term;
}

/* ------------------------------------------- Eigen 3.4 HouseholderQR::solve -------------------------------------
 * HouseholderQR(Map<ColMajor>) copies the matrix into its own MatrixXd and runs householder_qr_inplace_blocked (maxBlockSize
 * 48: one block for cols <= 48, i.e. householder_qr_inplace_unblocked on the whole matrix). The redux over the cwiseAbs2 /
 * cwiseProduct expressions has no DirectAccess, so alignedStart = 0 (Redux.h LinearVectorizedTraversal): two Packet2d
 * accumulators over groups of four, merge, optional last pair, predux (lane0 + lane1), scalar tail. The `essential^T * bottom`
 * of applyHouseholderOnTheLeft is a runtime-vector dot product when bottom has one column (lane pairs from 0 like the redux of
 * the product expression) and the RowMajor GEMV kernel (per column of bottom: packet pairs, predux, tail, then 0 + 1 * acc)
 * otherwise. solve(): c = Q^T rhs by H_0 .. H_{rank-1} applied to the vector (dot path), then the upper triangular
 * solveInPlace (TriangularSolverVector.h, ColMajor, one panel for size <= 8). */
static double redux_sq(const double* v, long n) {                 /* (v.cwiseAbs2()).sum() */
    long a2, a1, e2, e, idx;
    double p0a, p0b, res;
    if (n <= 0) return 0.0;
#if OK_ALIGN4_MUTATE == 3
    res = 0.0; for (idx = 0; idx < n; ++idx) res += v[idx] * v[idx]; return res;
#endif
    a2 = (n / 4) * 4; a1 = (n / 2) * 2; e2 = a2; e = a1;
    if (a1) {
        p0a = v[0] * v[0]; p0b = v[1] * v[1];
        if (a1 > 2) {
            double p1a = v[2] * v[2], p1b = v[3] * v[3];
            for (idx = 4; idx < e2; idx += 4) {
                p0a += v[idx] * v[idx]; p0b += v[idx + 1] * v[idx + 1];
                p1a += v[idx + 2] * v[idx + 2]; p1b += v[idx + 3] * v[idx + 3];
            }
            p0a += p1a; p0b += p1b;
            if (e > e2) { p0a += v[e2] * v[e2]; p0b += v[e2 + 1] * v[e2 + 1]; }
        }
        res = p0a + p0b;
        for (idx = e; idx < n; ++idx) res += v[idx] * v[idx];
    } else {
        res = v[0] * v[0];
        for (idx = 1; idx < n; ++idx) res += v[idx] * v[idx];
    }
    return res;
}
static double redux_dot(const double* a, const double* b, long n) {  /* a.transpose().cwiseProduct(b).sum() */
    long a2, a1, idx;
    double p0a, p0b, res;
    if (n <= 0) return 0.0;
    a2 = (n / 4) * 4; a1 = (n / 2) * 2;
    if (a1) {
        p0a = a[0] * b[0]; p0b = a[1] * b[1];
        if (a1 > 2) {
            double p1a = a[2] * b[2], p1b = a[3] * b[3];
            for (idx = 4; idx < a2; idx += 4) {
                p0a += a[idx] * b[idx]; p0b += a[idx + 1] * b[idx + 1];
                p1a += a[idx + 2] * b[idx + 2]; p1b += a[idx + 3] * b[idx + 3];
            }
            p0a += p1a; p0b += p1b;
            if (a1 > a2) { p0a += a[a2] * b[a2]; p0b += a[a2 + 1] * b[a2 + 1]; }
        }
        res = p0a + p0b;
        for (idx = a1; idx < n; ++idx) res += a[idx] * b[idx];
    } else {
        res = a[0] * b[0];
        for (idx = 1; idx < n; ++idx) res += a[idx] * b[idx];
    }
    return res;
}
/* makeHouseholderInPlace on x[0..len): returns beta, x[1..] = essential, *tau */
static double make_householder(double* x, int len, double* tau) {
    const double c0 = x[0];
    double tail_sq, beta, denom;
    int i;
    tail_sq = len == 1 ? 0.0 : redux_sq(x + 1, len - 1);
    if (tail_sq <= 2.2250738585072014e-308) {          /* abs2(imag(c0)) = 0 <= tol */
        *tau = 0.0; beta = c0;
        for (i = 1; i < len; ++i) x[i] = 0.0;
        return beta;
    }
    beta = sqrt(c0 * c0 + tail_sq);
    if (c0 >= 0.0) beta = -beta;
    denom = c0 - beta;
    for (i = 1; i < len; ++i) x[i] = x[i] / denom;
    *tau = (beta - c0) / beta;
    return beta;
}
/* applyHouseholderOnTheLeft on the block of nrows x ncols at `blk` (column stride ld); essential has nrows-1 entries.
 * The dot of one column with the essential vector: gemv=1 -> RowMajor GEMV kernel (+0 + 1*acc), gemv=0 -> dot product. */
static void apply_left(double* blk, int ld, int nrows, int ncols, const double* ess, double tau, double* tmp, double* tess, int gemv) {
    const int n = nrows - 1;
    int i, j;
    if (nrows == 1) {
        const double f = 1.0 - tau;
        for (j = 0; j < ncols; ++j) blk[j * ld] *= f;
        return;
    }
    if (tau == 0.0) return;
    if (!gemv) {
        for (j = 0; j < ncols; ++j) 
#if OK_ALIGN4_MUTATE == 5
            tmp[j] = redux_dot(ess, blk + (size_t)j * ld + 1, n);
#else
            tmp[j] = 0.0 + 1.0 * redux_dot(ess, blk + (size_t)j * ld + 1, n);
#endif
           /* dst.coeffRef(0,0) += alpha * dot */
    } else {
        for (j = 0; j < ncols; ++j) {
            const double* col = blk + (size_t)j * ld + 1;
            double l0 = 0.0, l1 = 0.0, cc;
            for (i = 0; i + 2 <= n; i += 2) { l0 += col[i] * ess[i]; l1 += col[i + 1] * ess[i + 1]; }
            cc = l0 + l1;
            for (; i < n; ++i) cc += col[i] * ess[i];
            tmp[j] = 0.0 + 1.0 * cc;
        }
    }
    for (j = 0; j < ncols; ++j) tmp[j] += blk[(size_t)j * ld];
    for (j = 0; j < ncols; ++j) blk[(size_t)j * ld] -= tau * tmp[j];
    for (i = 0; i < n; ++i) tess[i] = tau * ess[i];
    for (j = 0; j < ncols; ++j) {
        double* col = blk + (size_t)j * ld + 1;
#if OK_ALIGN4_MUTATE == 8
        for (i = 0; i < n; ++i) col[i] -= tau * (ess[i] * tmp[j]);
#else
        for (i = 0; i < n; ++i) col[i] -= tess[i] * tmp[j];
#endif
    }
}

void ok_eigen_hqr_solve(int rows, int cols, const double* lhs, const double* rhs, double* x) {
    const int rank = rows < cols ? rows : cols;
    double* qr = (double*)malloc(sizeof(double) * (size_t)rows * (size_t)cols);
    double* h = (double*)calloc((size_t)rank + 1, sizeof(double));
    double* c = (double*)malloc(sizeof(double) * (size_t)rows);
    double* tmp = (double*)calloc((size_t)cols + 1, sizeof(double));
    double* tess = (double*)malloc(sizeof(double) * (size_t)rows);
    int k, i;
    memcpy(qr, lhs, sizeof(double) * (size_t)rows * (size_t)cols);
    memcpy(c, rhs, sizeof(double) * (size_t)rows);
    for (k = 0; k < rank; ++k) {             /* householder_qr_inplace_unblocked */
        const int rr = rows - k, rc = cols - k - 1;
        double beta;
        beta = make_householder(qr + (size_t)k * rows + k, rr, &h[k]);
        qr[(size_t)k * rows + k] = beta;
        apply_left(qr + (size_t)(k + 1) * rows + k, rows, rr, rc, qr + (size_t)k * rows + k + 1, h[k], tmp, tess, rc > (OK_ALIGN4_MUTATE == 4 ? 0 : 1));
    }
    for (k = 0; k < rank; ++k) {             /* c = Q^T c: reverse flag set, actual_k = k, dst cols = 1 */
        const int kk = OK_ALIGN4_MUTATE == 7 ? rank - 1 - k : k;
        apply_left(c + kk, rows, rows - kk, 1, qr + (size_t)kk * rows + kk + 1, h[kk], tmp, tess, 0);
    }
    for (k = 0; k < rank; ++k) {             /* upper solveInPlace on c[0..rank), one panel */
        const int j = rank - k - 1;
        if (OK_ALIGN4_MUTATE == 6 || c[j] != 0.0) {
            const int r = rank - k - 1;
            c[j] /= qr[(size_t)j * rows + j];
            for (i = 0; i < r; ++i) c[j - r + i] -= c[j] * qr[(size_t)j * rows + (j - r + i)];
        }
    }
    for (i = 0; i < rank; ++i) x[i] = c[i];
    for (i = rank; i < cols; ++i) x[i] = 0.0;
    free(qr); free(h); free(c); free(tmp); free(tess);
}
