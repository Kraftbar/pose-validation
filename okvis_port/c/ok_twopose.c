/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 5a: TwoPose*GraphError terms and PseudoInverse. See ok_twopose.h for notices.
 * "Math on paper": one equation per line, in the evaluation order of the reference build (Eigen 3.4.0, SSE2, no FMA).
 *
 * Eigen statement forms used here (letters as in ok_err.c; all measured by okvis_twopose_test.cc):
 *   `Jmin(RowMajor) = J_.block<6,6>(0,0) * Jerr` : assignment of a product to an existing matrix goes through the
 *      aliasing temporary of the PRODUCT's storage order (column-major): case A, then a transposing copy.
 *   `const Matrix<RowMajor> Jmin = J_ * JerrRef` : a construction has no temporary; the column-major product is
 *      evaluated straight into the row-major object: no packets, every coefficient the redux tree (case D).
 *   `JerrRef.block<3,3>(0,3) = C * crossMx(d)` : 3x3 product into a block of a row-major matrix: aliasing temporary
 *      (column-major 3x3, the stella 3x3 rules) then copy.
 *   `error_weighted = J_ * error` (6x6 * 6 into a Map): case A. `J = Jmin * J_lift` into a Map<RowMajor>: case B.
 *   `DeltaX_ = -M * (M.transpose() * b0_)` : the inner product is evaluated first (lhs row-major view: case C with the
 *      vectorised redux), then (-M) * t with the negation inside the lazy product (case A).
 *   `M = W * V_inv_sqrt`, `mH += M * M.transpose()`, `mb += M * (V^T b1)` : case A (left folds; the 3x3 transposed
 *      product V^T b1 is the all-left-fold A^T v rule of ok_eigen).
 *   Depth-2 products (J^T J, J^T r of the 2-row reprojection Jacobians) are order-free; `+=` / `-=` add the
 *      coefficient of the lazy product to the destination.
 *   Diagonal products (`D.asDiagonal() * V^T`, `V * D.asDiagonal()`) are plain coefficient scalings.
 *   The n >= 12 TwoPoseExtrinsics term: `J_ * error` is the GEMV column kernel (ok_gemv_col into a zeroed vector);
 *      `Jmin(MatrixXd) = J_.block(0,0,n,6) * Jerr` the dynamic column-major aliasing temporary (even n: every row a
 *      packet, left folds) and its `J = Jmin * J_lift` a column-major temporary too (left folds); the constructions
 *      `const Matrix<Dynamic,6,RowMajor> Jmin = J_.block(0,j,n,6) * JerrRef / Jerrex` read a DYNAMIC-size block, whose
 *      coefficient redux is the NoUnrolling left fold (not the fixed-size tree of the standard term). */
#include "ok_twopose.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ok_dense.h"
#include "ok_param.h"

#define M4(a, i, j) (a)[(i) + 4 * (j)]
#define M6(a, i, j) (a)[(i) + 6 * (j)]

/* ------------------------------------------------ PseudoInverse ------------------------------------------------ */
static void pinv_common(int n, const double* a, double epsilon, int* rank, double* V, double* sel) {
    double ev[OK_EIG_MAX], mx, tol;
    int i;
    ok_selfadjoint_eig(n, a, ev, V, 0);
    mx = ev[0];
    for (i = 1; i < n; ++i) mx = (mx < ev[i]) ? ev[i] : mx; /* maxCoeff */
    tol = epsilon * (double)n * mx;                         /* epsilon * a.cols() * max */
    tol = (epsilon < tol) ? tol : epsilon;                  /* std::max(epsilon, ...) */
    for (i = 0; i < n; ++i) sel[i] = (ev[i] > tol) ? 1.0 / ev[i] : 1.0 / tol;
    if (rank) {
        *rank = 0;
        for (i = 0; i < n; ++i)
            if (ev[i] > tol) (*rank)++;
    }
}
void ok_pinv_symm(int n, const double* a, double* result, double epsilon, int* rank) {
    double V[OK_EIG_MAX * OK_EIG_MAX], sel[OK_EIG_MAX], VD[OK_EIG_MAX * OK_EIG_MAX], Vt[OK_EIG_MAX * OK_EIG_MAX];
    int i, j;
    pinv_common(n, a, epsilon, rank, V, sel);
    /* result = (V * diag(sel)) * V^T : the left product is a diagonal scaling, the outer one a lazy product with a
     * column-major lhs (case A) and the transposed eigenvector matrix as rhs */
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) VD[i + n * j] = V[i + n * j] * sel[j];
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) Vt[j + n * i] = V[i + n * j];
    /* the DiagonalWrapper of a VectorXd makes the temporary a Matrix<n, Dynamic>: the outer product has a dynamic
     * depth, whose coefficient loops (etor_product_*_impl<..., Dynamic>) are plain left folds for every entry */
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) {
            double s = VD[i + n * 0] * Vt[0 + n * j];
            int k;
            for (k = 1; k < n; ++k) s = s + VD[i + n * k] * Vt[k + n * j];
            result[i + n * j] = s;
        }
}
void ok_pinv_symm_sqrt(int n, const double* a, double* result, double epsilon, int* rank) {
    double V[OK_EIG_MAX * OK_EIG_MAX], sel[OK_EIG_MAX];
    int i, j;
    pinv_common(n, a, epsilon, rank, V, sel);
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) result[i + n * j] = V[i + n * j] * sqrt(sel[j]);
}
void ok_pinv_symm_sqrt_u(int n, const double* a, double* result, double epsilon, int* rank) {
    double V[OK_EIG_MAX * OK_EIG_MAX], sel[OK_EIG_MAX];
    int i, j;
    pinv_common(n, a, epsilon, rank, V, sel);
    for (j = 0; j < n; ++j)
        for (i = 0; i < n; ++i) result[i + n * j] = sqrt(sel[i]) * V[j + n * i];
}

/* ------------------------------------------- shared evaluate pieces -------------------------------------------- */
/* T_WS0 = Transformation(r, q.normalized()); T_S0W = T_WS0.inverse(); T_WSi likewise; T_S0Si = T_S0W * T_WSi;
 * dx[0..2] = T_S0Si.r - lin.r ; dx[3..5] = 2 * (T_S0Si.q * lin.q^-1).xyz */
static void relpose_delta(const double* p0, const double* p1, const ok_tf* lin, ok_tf* T_WS0, ok_tf* T_S0W,
                          double dx[6]) {
    ok_quat q0 = {p0[3], p0[4], p0[5], p0[6]}, q1 = {p1[3], p1[4], p1[5], p1[6]}, qinv, qd;
    double r0[3] = {p0[0], p0[1], p0[2]}, r1[3] = {p1[0], p1[1], p1[2]};
    ok_tf T_WSi, T_S0Si;
    q0 = ok_quat_normalized(q0);
    ok_tf_from_rq(T_WS0, r0, &q0, 1);
    ok_tf_inverse(T_WS0, T_S0W, 1);
    q1 = ok_quat_normalized(q1);
    ok_tf_from_rq(&T_WSi, r1, &q1, 1);
    ok_tf_mul(T_S0W, &T_WSi, &T_S0Si, 1);
    dx[0] = T_S0Si.r[0] - lin->r[0];
    dx[1] = T_S0Si.r[1] - lin->r[1];
    dx[2] = T_S0Si.r[2] - lin->r[2];
    qinv = ok_quat_inverse(lin->q);
    ok_quat_mul(&T_S0Si.q, &qinv, &qd);
    dx[3] = 2.0 * qd.x;
    dx[4] = 2.0 * qd.y;
    dx[5] = 2.0 * qd.z;
}

/* Jerr (6x6 col-major) = [T_S0W.C 0; 0 (plus(T_WS0.q^-1) * oplus(T_WS.q * lin.q^-1)).topLeft3] with T_WS from p1
 * (constructor normalisation only); JerrRef (6x6, logically row-major but held column-major here) = [-C  C*crossMx(T_WS.r - T_WS0.r); 0 -Jerr_br] */
static void relpose_jacobian_cores(const double* p1, const ok_tf* T_WS0, const ok_tf* T_S0W, const ok_tf* lin,
                                   double Jerr[36], double JerrRef[36]) {
    ok_quat q1 = {p1[3], p1[4], p1[5], p1[6]}, q0inv, qlinv, qm;
    double r1[3] = {p1[0], p1[1], p1[2]}, P[16], O[16], PO[16], d[3], cr[9], blk[9];
    ok_tf T_WS;
    int i, j;
    ok_tf_from_rq(&T_WS, r1, &q1, 1);
    for (i = 0; i < 36; ++i) Jerr[i] = 0.0;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M6(Jerr, i, j) = T_S0W->C[i + 3 * j];
    q0inv = ok_quat_inverse(T_WS0->q);
    ok_kin_plus(&q0inv, P);
    qlinv = ok_quat_inverse(lin->q);
    ok_quat_mul(&T_WS.q, &qlinv, &qm);
    ok_kin_oplus(&qm, O);
    ok_lazy_a(4, 4, 4, P, O, PO); /* plus(q0^-1) * oplus(q * qlin^-1), a 4x4 lazy product evaluated into a temporary */
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M6(Jerr, 3 + i, 3 + j) = M4(PO, i, j);
    for (i = 0; i < 36; ++i) JerrRef[i] = 0.0;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M6(JerrRef, i, j) = -T_S0W->C[i + 3 * j];
    d[0] = T_WS.r[0] - T_WS0->r[0];
    d[1] = T_WS.r[1] - T_WS0->r[1];
    d[2] = T_WS.r[2] - T_WS0->r[2];
    ok_kin_cross_mx(d, cr);
    ok_m3_mul(T_S0W->C, cr, blk); /* T_S0W.C() * crossMx(...): 3x3 lazy product into a temporary, stella rules */
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M6(JerrRef, i, 3 + j) = blk[i + 3 * j];
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) M6(JerrRef, 3 + i, 3 + j) = -M6(Jerr, 3 + i, 3 + j);
}

/* out(R x C) = A(R x D, column-major, leading dimension lda) * B(D x C, column-major) with a DYNAMIC-size lhs block
 * (`J_.block(0, j, n, 6)` of the extrinsics term): the coefficient redux of a dynamic-size expression is
 * redux_impl<DefaultTraversal, NoUnrolling>, a plain left fold, for every coefficient */
static void lazy_fold_dyn(int R, int C, int D, const double* A, long lda, const double* B, double* out) {
    int i, j, k;
    for (j = 0; j < C; ++j)
        for (i = 0; i < R; ++i) {
            double s = A[i + lda * 0] * B[0 + D * j];
            for (k = 1; k < D; ++k) s = s + A[i + lda * k] * B[k + D * j];
            out[i + R * j] = s;
        }
}

static void to_rm(int R, int C, const double* A, double* out) {
    int i, j;
    for (i = 0; i < R; ++i)
        for (j = 0; j < C; ++j) out[i * C + j] = A[i + R * j];
}

/* `J = Jmin * J_lift` into the row-major Map (case B: the row-major Jmin of the standard term) and the optional
 * minimal Jacobian copy */
static void lift_and_store(int nres, const double* Jmin_rm, const double* pblock, double* jac, double* jacmin) {
    double Jlift[42];
    ok_pose_minus_jacobian(pblock, Jlift);
    ok_lazy_tree_rm(nres, 7, 6, Jmin_rm, Jlift, jac);
    if (jacmin != NULL) memcpy(jacmin, Jmin_rm, sizeof(double) * (size_t)nres * 6);
}
/* the same for a COLUMN-major Jmin (the MatrixXd / Matrix<Dynamic,6,RowMajor> mix of the extrinsics term): the
 * assignment to the Map goes through a column-major n x 7 temporary of the product (packets along the columns:
 * left folds, case A), which is then copied into the row-major map; Jmin_rm is still what the minimal map receives */
static void lift_and_store_cm(int nres, const double* Jmin_cm, const double* Jmin_rm, const double* pblock, double* jac,
                              double* jacmin) {
    double Jlift[42], Jlift_cm[42], t[(6 + 6 * OK_TP_MAXEXTR) * 7];
    int i, j;
    ok_pose_minus_jacobian(pblock, Jlift);
    for (i = 0; i < 6; ++i)
        for (j = 0; j < 7; ++j) Jlift_cm[i + 6 * j] = Jlift[i * 7 + j];
    ok_lazy_a(nres, 7, 6, Jmin_cm, Jlift_cm, t);
    to_rm(nres, 7, t, jac);
    if (jacmin != NULL) memcpy(jacmin, Jmin_rm, sizeof(double) * (size_t)nres * 6);
}

/* ---------------------------------------- TwoPoseStandardGraphError(Const) --------------------------------------- */
int ok_tp_std_evaluate(const ok_tp_std* e, const double* const params[2], double res[6], double* const* jac,
                       double* const* jacmin) {
    ok_tf T_WS0, T_S0W;
    double dx[6], error[6];
    int i;
    if (!e->is_computed) return 0;
    relpose_delta(params[0], params[1], &e->lin_T_S0S1, &T_WS0, &T_S0W, dx);
    for (i = 0; i < 6; ++i) error[i] = e->DeltaX[i] + dx[i];
    ok_lazy_a(6, 1, 6, e->J, error, res); /* error_weighted = J_ * error */
    if (jac != NULL) {
        double Jerr[36], JerrRef[36];
        for (i = 0; i < 36; ++i) JerrRef[i] = 0.0;
        if (jac[1] != NULL) {
            double t[36], Jmin_rm[36];
            relpose_jacobian_cores(params[1], &T_WS0, &T_S0W, &e->lin_T_S0S1, Jerr, jac[0] != NULL ? JerrRef : t);
            if (jac[0] == NULL)
                for (i = 0; i < 36; ++i) JerrRef[i] = 0.0; /* JerrRef is only filled when jacobians[refIdx] */
            ok_lazy_a(6, 6, 6, e->J, Jerr, t); /* Jmin = J_.block<6,6>(0,0) * Jerr (aliasing temporary, case A) */
            to_rm(6, 6, t, Jmin_rm);
            lift_and_store(6, Jmin_rm, params[1], jac[1], (jacmin != NULL) ? jacmin[1] : NULL);
        }
        if (jac[0] != NULL) {
            double t[36], Jmin_rm[36];
            ok_lazy_tree(6, 6, 6, e->J, JerrRef, t); /* const Matrix<RowMajor> Jmin = J_ * JerrRef (case D) */
            to_rm(6, 6, t, Jmin_rm);
            lift_and_store(6, Jmin_rm, params[0], jac[0], (jacmin != NULL) ? jacmin[0] : NULL);
        }
    }
    return 1;
}

/* --------------------------------------- TwoPoseExtrinsicsGraphError(Const) -------------------------------------- */
int ok_tp_ext_evaluate(const ok_tp_ext* e, const double* const* params, double* res, double* const* jac,
                       double* const* jacmin) {
    const int n = e->n;
    ok_tf T_WS0, T_S0W;
    double dx[6 + 6 * OK_TP_MAXEXTR], error[6 + 6 * OK_TP_MAXEXTR];
    int i, c;
    if (!e->is_computed) return 0;
    for (i = 0; i < n; ++i) dx[i] = 0.0;
    relpose_delta(params[0], params[1], &e->lin_T_S0S1, &T_WS0, &T_S0W, dx);
    for (c = 0; c < e->nextr; ++c) {
        const double* p = params[2 + c];
        ok_quat q = {p[3], p[4], p[5], p[6]}, qinv, qd;
        double r[3] = {p[0], p[1], p[2]};
        ok_tf T_SC;
        q = ok_quat_normalized(q);
        ok_tf_from_rq(&T_SC, r, &q, 1);
        if (e->extr_present[c]) {
            const ok_tf* L = &e->lin_T_SC[c];
            dx[6 + 6 * c + 0] = T_SC.r[0] - L->r[0];
            dx[6 + 6 * c + 1] = T_SC.r[1] - L->r[1];
            dx[6 + 6 * c + 2] = T_SC.r[2] - L->r[2];
            qinv = ok_quat_inverse(L->q);
            ok_quat_mul(&T_SC.q, &qinv, &qd);
            dx[6 + 6 * c + 3] = 2.0 * qd.x;
            dx[6 + 6 * c + 4] = 2.0 * qd.y;
            dx[6 + 6 * c + 5] = 2.0 * qd.z;
        }
    }
    for (i = 0; i < n; ++i) error[i] = e->DeltaX[i] + dx[i];
    for (i = 0; i < n; ++i) res[i] = 0.0;
    ok_gemv_col(n, n, e->J, n, error, 1, res, 1.0); /* error_weighted = J_ * error (GEMV column kernel) */
    if (jac != NULL) {
        double Jerr[36], JerrRef[36];
        for (i = 0; i < 36; ++i) JerrRef[i] = 0.0;
        if (jac[1] != NULL) {
            double t[(6 + 6 * OK_TP_MAXEXTR) * 6], Jmin_rm[(6 + 6 * OK_TP_MAXEXTR) * 6];
            relpose_jacobian_cores(params[1], &T_WS0, &T_S0W, &e->lin_T_S0S1, Jerr, jac[0] != NULL ? JerrRef : t);
            if (jac[0] == NULL)
                for (i = 0; i < 36; ++i) JerrRef[i] = 0.0;
            ok_lazy_a(n, 6, 6, e->J, Jerr, t); /* Jmin(MatrixXd) = J_.block(0,0,n,6) * Jerr (dynamic aliasing temporary) */
            to_rm(n, 6, t, Jmin_rm);
            lift_and_store_cm(n, t, Jmin_rm, params[1], jac[1], (jacmin != NULL) ? jacmin[1] : NULL);
        }
        if (jac[0] != NULL) {
            double t[(6 + 6 * OK_TP_MAXEXTR) * 6], Jmin_rm[(6 + 6 * OK_TP_MAXEXTR) * 6];
            lazy_fold_dyn(n, 6, 6, e->J, n, JerrRef, t); /* const Matrix<Dynamic,6,RowMajor> Jmin = J_.block(0,0,n,6) * JerrRef: dynamic block, left folds */
            to_rm(n, 6, t, Jmin_rm);
            lift_and_store(n, Jmin_rm, params[0], jac[0], (jacmin != NULL) ? jacmin[0] : NULL);
        }
        for (c = 0; c < e->nextr; ++c) {
            if (jac[2 + c] != NULL) {
                const double* p = params[2 + c];
                double Jerrex[36], t[(6 + 6 * OK_TP_MAXEXTR) * 6], Jmin_rm[(6 + 6 * OK_TP_MAXEXTR) * 6];
                for (i = 0; i < 36; ++i) Jerrex[i] = 0.0;
                if (e->extr_present[c]) {
                    ok_quat q = {p[3], p[4], p[5], p[6]}, qinv, qm;
                    double r[3] = {p[0], p[1], p[2]}, O[16];
                    ok_tf T_SC;
                    int j;
                    q = ok_quat_normalized(q);
                    ok_tf_from_rq(&T_SC, r, &q, 1);
                    M6(Jerrex, 0, 0) = 1.0; M6(Jerrex, 1, 1) = 1.0; M6(Jerrex, 2, 2) = 1.0;
                    qinv = ok_quat_inverse(e->lin_T_SC[c].q);
                    ok_quat_mul(&T_SC.q, &qinv, &qm);
                    ok_kin_oplus(&qm, O);
                    for (j = 0; j < 3; ++j)
                        for (i = 0; i < 3; ++i) M6(Jerrex, 3 + i, 3 + j) = M4(O, i, j);
                }
                lazy_fold_dyn(n, 6, 6, e->J + (long)n * (6 + 6 * c), n, Jerrex, t); /* const Matrix<Dynamic,6,RowMajor> Jmin = J_.block(0, 6+6c, n, 6) * Jerrex: dynamic block, left folds */
                to_rm(n, 6, t, Jmin_rm);
                lift_and_store(n, Jmin_rm, p, jac[2 + c], (jacmin != NULL) ? jacmin[2 + c] : NULL);
            }
        }
    }
    return 1;
}

/* ----------------------------------------- TwoPoseGraphError bookkeeping ----------------------------------------- */
void ok_twopose_init(ok_twopose* t, uint64_t ref_id, uint64_t other_id, int num_cams, int stay_const) {
    int i;
    memset(t, 0, sizeof *t);
    t->ref_id = ref_id;
    t->other_id = other_id;
    t->num_cams = num_cams;
    t->stay_const = stay_const;
    t->nextr = stay_const ? num_cams : 2 * num_cams;
    for (i = 0; i < 36; ++i) { t->H00[i] = 0.0; t->term.J[i] = 0.0; }
    for (i = 0; i < 6; ++i) { t->b0[i] = 0.0; t->term.DeltaX[i] = 0.0; }
    ok_tf_identity(&t->term.lin_T_S0S1);
}

void ok_twopose_free(ok_twopose* t) {
    int g;
    for (g = 0; g < t->ngroups; ++g) free(t->groups[g].obs);
    free(t->groups);
    free(t->lm_id); free(t->lm_vec_idx); free(t->lm_idx); free(t->lm_snapshot);
    free(t->lm_S0);
    memset(t, 0, sizeof *t);
}

static ok_tp_group* find_or_add_group(ok_twopose* t, uint64_t lm_id) {
    int g, pos;
    for (g = 0; g < t->ngroups; ++g)
        if (t->groups[g].lm_id == lm_id) return &t->groups[g];
    if (t->ngroups == t->cap_groups) {
        t->cap_groups = t->cap_groups ? 2 * t->cap_groups : 16;
        t->groups = (ok_tp_group*)realloc(t->groups, sizeof(ok_tp_group) * (size_t)t->cap_groups);
    }
    /* std::map: keep the groups sorted by landmark id */
    pos = t->ngroups;
    while (pos > 0 && t->groups[pos - 1].lm_id > lm_id) { t->groups[pos] = t->groups[pos - 1]; pos--; }
    t->groups[pos].lm_id = lm_id;
    t->groups[pos].nobs = 0;
    t->groups[pos].cap = 0;
    t->groups[pos].obs = NULL;
    t->ngroups++;
    return &t->groups[pos];
}

int ok_twopose_add_observation(ok_twopose* t, uint64_t frame_id, int cam, int kp, const ok_reproj_err* err, int loss,
                               uint64_t pose_id, const double pose[7], uint64_t hpoint_id, const double hpoint[4],
                               int hpoint_initialised, uint64_t extr_id, const double extr[7], int is_duplication) {
    const int ci = cam;
    ok_tp_group* grp = find_or_add_group(t, hpoint_id);
    ok_tp_obs* o;
    int pose_idx = 0, offset, found = 0, i;
    if (grp->nobs == grp->cap) {
        grp->cap = grp->cap ? 2 * grp->cap : 4;
        grp->obs = (ok_tp_obs*)realloc(grp->obs, sizeof(ok_tp_obs) * (size_t)grp->cap);
    }
    o = &grp->obs[grp->nobs++];
    o->frame_id = frame_id; o->cam = cam; o->kp = kp;
    o->loss = loss;
    o->is_marginalised = 0;
    o->is_duplication = is_duplication;
    o->pose_id = pose_id; o->hpoint_id = hpoint_id; o->extr_id = extr_id;
    o->hpoint_initialised = hpoint_initialised;
    o->hp_live_init = NULL;
    memcpy(o->hpoint, hpoint, sizeof o->hpoint);
    o->err = *err;                                               /* reprojectionError->clone() */
    if (is_duplication) ok_reproj_err_set_information(&o->err, o->err.info); /* setInformation(information()) */
    /* first parameter: pose (poseIdx is only set when the info is created, as upstream) */
    if (pose_id == t->ref_id) {
        if (!t->pose_present[0]) { t->pose_present[0] = 1; t->pose_id[0] = pose_id; memcpy(t->pose_snapshot[0], pose, sizeof(double) * 7); memcpy(t->pose_live[0], pose, sizeof(double) * 7); pose_idx = 0; }
    } else if (pose_id == t->other_id) {
        if (!t->pose_present[1]) { t->pose_present[1] = 1; t->pose_id[1] = pose_id; memcpy(t->pose_snapshot[1], pose, sizeof(double) * 7); memcpy(t->pose_live[1], pose, sizeof(double) * 7); pose_idx = 1; }
    } else {
        return 0; /* "pose not registered" */
    }
    /* second parameter: landmark */
    for (i = 0; i < t->nlm; ++i)
        if (t->lm_id[i] == hpoint_id) { found = 1; break; }
    if (!found) {
        int pos;
        if (t->nlm == t->cap_lm) {
            t->cap_lm = t->cap_lm ? 2 * t->cap_lm : 16;
            t->lm_id = (uint64_t*)realloc(t->lm_id, sizeof(uint64_t) * (size_t)t->cap_lm);
            t->lm_vec_idx = (int*)realloc(t->lm_vec_idx, sizeof(int) * (size_t)t->cap_lm);
            t->lm_idx = (int*)realloc(t->lm_idx, sizeof(int) * (size_t)t->cap_lm);
            t->lm_snapshot = (double(*)[4])realloc(t->lm_snapshot, sizeof(double[4]) * (size_t)t->cap_lm);
        }
        /* the id2idx map is ordered by id; the vector index is the insertion order */
        pos = t->nlm;
        while (pos > 0 && t->lm_id[pos - 1] > hpoint_id) {
            t->lm_id[pos] = t->lm_id[pos - 1]; t->lm_vec_idx[pos] = t->lm_vec_idx[pos - 1]; t->lm_idx[pos] = t->lm_idx[pos - 1];
            memcpy(t->lm_snapshot[pos], t->lm_snapshot[pos - 1], sizeof(double) * 4);
            pos--;
        }
        t->lm_id[pos] = hpoint_id;
        t->lm_vec_idx[pos] = t->nlm;
        t->lm_idx[pos] = t->sparse_size;
        memcpy(t->lm_snapshot[pos], hpoint, sizeof(double) * 4);
        t->nlm++;
        t->sparse_size += 3;
    }
    /* third parameter: extrinsics */
    offset = t->stay_const ? 0 : pose_idx * t->num_cams;
    if (!t->extr_present[offset + ci]) {
        t->extr_present[offset + ci] = 1;
        t->extr_id[offset + ci] = extr_id;
        t->extr_idx[offset + ci] = 6 + offset + ci * 6;
        memcpy(t->extr_snapshot[offset + ci], extr, sizeof(double) * 7);
    }
    return 1;
}

/* CauchyLoss(1.0)::Evaluate (Ceres loss_function.cc) */
static void cauchy1_evaluate(double s, double rho[3]) {
    const double sum = 1.0 + s * 1.0;
    const double inv = 1.0 / sum;
    rho[0] = 1.0 * log(sum);
    rho[1] = inv > 2.2250738585072014e-308 ? inv : 2.2250738585072014e-308;
    rho[2] = -1.0 * (inv * inv);
}

/* mJ (2 x C row-major) = sqrt_rho1 * (mJ - alpha_sq_norm * residual * (residual^T * mJ)) */
static void robustify_jac(int C, double* mJ, const double r[2], double sqrt_rho1, double alpha_sq_norm) {
    double tcol[8];
    int i, j;
    for (j = 0; j < C; ++j) tcol[j] = r[0] * mJ[0 * C + j] + r[1] * mJ[1 * C + j]; /* residual^T * mJ (depth 2) */
    for (i = 0; i < 2; ++i)
        for (j = 0; j < C; ++j) {
            const double o = (alpha_sq_norm * r[i]) * tcol[j];
            mJ[i * C + j] = sqrt_rho1 * (mJ[i * C + j] - o);
        }
}

/* H (A x B col-major) += J1^T * J2 with J1 (2 x A row-major), J2 (2 x B row-major): depth-2 products */
static void add_jtj(int A, int B, double* H, const double* J1, const double* J2) {
    int a, b;
    for (b = 0; b < B; ++b)
        for (a = 0; a < A; ++a) {
            const double p0 = J1[0 * A + a] * J2[0 * B + b];
            const double p1 = J1[1 * A + a] * J2[1 * B + b];
            H[a + A * b] = H[a + A * b] + (p0 + p1);
        }
}
/* b (A) -= J^T * r with J (2 x A row-major) */
static void sub_jtr(int A, double* b, const double* J, const double r[2]) {
    int a;
    for (a = 0; a < A; ++a) {
        const double p0 = J[0 * A + a] * r[0];
        const double p1 = J[1 * A + a] * r[1];
        b[a] = b[a] - (p0 + p1);
    }
}

int ok_twopose_compute(ok_twopose* t) {
    ok_tf T_WS0, T_S0W;
    double mH[36], mb[6], ev[6], V[36], D_sqrt[6], D_inv_sqrt[6], M[36], negM[36], tmp[6], mx, tol;
    int g, i, j;
    if (t->term.is_computed) return 1;
    if (!t->pose_present[0] || !t->pose_present[1]) return 0;
    ok_tf_convert(&T_WS0, t->pose_live[0]); /* parameterBlock->estimate() (cacheless) -> const Transformation */
    ok_tf_inverse(&T_WS0, &T_S0W, 1);
    t->rel_pose_set = 0;
    for (i = 0; i < 36; ++i) mH[i] = 0.0;
    for (i = 0; i < 6; ++i) mb[i] = 0.0;
    t->nlm_S0 = 0;
    for (g = 0; g < t->ngroups; ++g) {
        ok_tp_group* grp = &t->groups[g];
        double H00[36], b0[6], H01[18], H11[9], b1[3], hp_W[4], hp_S0[4], minDist, V_inv_sqrt[9];
        int o, k, rank;
        for (i = 0; i < 36; ++i) H00[i] = 0.0;
        for (i = 0; i < 6; ++i) b0[i] = 0.0;
        for (i = 0; i < 18; ++i) H01[i] = 0.0;
        for (i = 0; i < 9; ++i) H11[i] = 0.0;
        for (i = 0; i < 3; ++i) b1[i] = 0.0;
        /* landmark processed: the snapshot taken by addObservation */
        for (k = 0; k < t->nlm; ++k)
            if (t->lm_id[k] == grp->lm_id) break;
        if (k == t->nlm) return 0;
        memcpy(hp_W, t->lm_snapshot[k], sizeof hp_W);
        ok_tf_mul_v4(&T_S0W, hp_W, hp_S0, 1); /* hp_S0 = T_S0W * hp_W */
        if (t->nlm_S0 == t->cap_lm_S0) {
            t->cap_lm_S0 = t->cap_lm_S0 ? 2 * t->cap_lm_S0 : 16;
            t->lm_S0 = (ok_tp_lm_S0*)realloc(t->lm_S0, sizeof(ok_tp_lm_S0) * (size_t)t->cap_lm_S0);
        }
        t->lm_S0[t->nlm_S0].id = grp->lm_id;
        memcpy(t->lm_S0[t->nlm_S0].hp_S0, hp_S0, sizeof hp_S0);
        t->nlm_S0++;
        minDist = hp_S0[2] / hp_S0[3];
        for (o = 0; o < grp->nobs; ++o) {
            ok_tp_obs* ob = &grp->obs[o];
            const int is_ref = ob->frame_id == t->ref_id;
            const int pose_idx = is_ref ? 0 : 1;
            const int offset = t->stay_const ? 0 : pose_idx * t->num_cams;
            const double* info0 = t->pose_snapshot[pose_idx];
            const double* info2 = t->extr_snapshot[offset + ob->cam];
            double residual[2], J0[14], mJ0[12], J1[8], mJ1[6], pblock[7], rnorm;
            double* jacs[3];
            double* mjacs[3];
            const double* params[3];
            ok_tf T_S0S;
            ok_tf_identity(&T_S0S);
            if (!is_ref) {
                ok_quat q = {info0[3], info0[4], info0[5], info0[6]};
                double r[3] = {info0[0], info0[1], info0[2]};
                ok_tf T_WS;
                ok_tf_from_rq(&T_WS, r, &q, 1);
                ok_tf_mul(&T_S0W, &T_WS, &T_S0S, 1); /* T_S0S = T_S0W * T_WS */
            }
            if (!is_ref && !t->rel_pose_set) {
                t->term.lin_T_S0S1 = T_S0S;
                t->rel_pose_set = 1;
            }
            ok_pose_block_set_estimate(pblock, &T_S0S); /* PoseParameterBlock pose(T_S0S, id0, Time(0)) */
            params[0] = pblock;
            params[1] = hp_S0;
            params[2] = info2;
            jacs[0] = is_ref ? NULL : J0;
            jacs[1] = J1;
            jacs[2] = NULL;
            mjacs[0] = is_ref ? NULL : mJ0;
            mjacs[1] = mJ1;
            mjacs[2] = NULL;
            for (i = 0; i < 12; ++i) mJ0[i] = 0.0; /* uninitialised upstream when isReference (unused then) */
            ok_reproj_err_evaluate(&ob->err, params, residual, jacs, mjacs);
            rnorm = sqrt(residual[0] * residual[0] + residual[1] * residual[1]);
            if (rnorm > 3.0) continue; /* ignore obvious outliers */
            if (ob->loss) {
                const double sq_norm = residual[0] * residual[0] + residual[1] * residual[1];
                double rho[3], sqrt_rho1, residual_scaling, alpha_sq_norm;
                cauchy1_evaluate(sq_norm, rho);
                sqrt_rho1 = sqrt(rho[1]);
                if (sq_norm == 0.0 || rho[2] <= 0.0) {
                    residual_scaling = sqrt_rho1;
                    alpha_sq_norm = 0.0;
                } else {
                    const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
                    const double alpha = 1.0 - sqrt(D);
                    residual_scaling = sqrt_rho1 / (1 - alpha);
                    alpha_sq_norm = alpha / sq_norm;
                }
                robustify_jac(6, mJ0, residual, sqrt_rho1, alpha_sq_norm);
                robustify_jac(3, mJ1, residual, sqrt_rho1, alpha_sq_norm);
                residual[0] *= residual_scaling;
                residual[1] *= residual_scaling;
            }
            if (!is_ref) {
                add_jtj(6, 6, H00, mJ0, mJ0);
                sub_jtr(6, b0, mJ0, residual);
                add_jtj(6, 3, H01, mJ0, mJ1);
            }
            add_jtj(3, 3, H11, mJ1, mJ1);
            sub_jtr(3, b1, mJ1, residual);
            ob->is_marginalised = 1;
        }
        /* now marginalise out */
        ok_pinv_symm_sqrt(3, H11, V_inv_sqrt, 1.0e-7, &rank);
        if (rank < 3 && minDist < 2.99) {
            /* don't do anything */
        } else {
            double Mw[18], Mt[18], prod[36], vb[3], mv[6];
            for (i = 0; i < 36; ++i) t->H00[i] += H00[i];
            for (i = 0; i < 6; ++i) t->b0[i] += b0[i];
            ok_lazy_a(6, 3, 3, H01, V_inv_sqrt, Mw); /* M = W * V_inv_sqrt */
            for (j = 0; j < 6; ++j)
                for (i = 0; i < 3; ++i) Mt[i + 3 * j] = Mw[j + 6 * i];
            ok_lazy_a(6, 6, 3, Mw, Mt, prod); /* mH += M * M.transpose() */
            for (i = 0; i < 36; ++i) mH[i] += prod[i];
            ok_m3_mulv_lhsT(V_inv_sqrt, b1, vb); /* V_inv_sqrt.transpose() * b1 */
            ok_lazy_a(6, 1, 3, Mw, vb, mv);       /* M * (...) */
            for (i = 0; i < 6; ++i) mb[i] += mv[i];
        }
    }
    for (i = 0; i < 36; ++i) t->H00[i] -= mH[i];
    for (i = 0; i < 6; ++i) t->b0[i] -= mb[i];
    /* eigendecomposition of the (possibly singular) H00_ */
    ok_selfadjoint_eig(6, t->H00, ev, V, 0);
    mx = ev[0];
    for (i = 1; i < 6; ++i) mx = (mx < ev[i]) ? ev[i] : mx;
    tol = 1.0e-8 * 6.0 * mx;
    for (i = 0; i < 6; ++i) D_sqrt[i] = (ev[i] > tol) ? sqrt(ev[i]) : 0.0;
    for (i = 0; i < 6; ++i) D_inv_sqrt[i] = (ev[i] > tol) ? sqrt(1.0 / ev[i]) : 0.0;
    for (j = 0; j < 6; ++j)
        for (i = 0; i < 6; ++i) M6(t->term.J, i, j) = D_sqrt[i] * M6(V, j, i); /* J_ = D_sqrt.asDiagonal() * V^T */
    for (j = 0; j < 6; ++j)
        for (i = 0; i < 6; ++i) M6(M, i, j) = M6(V, i, j) * D_inv_sqrt[j];        /* M = V * D_inv_sqrt.asDiagonal() */
    ok_lazy_c(6, 1, 6, M, t->b0, tmp); /* M.transpose() * b0_ : the row-major view of M against b0 (case C) */
    for (i = 0; i < 36; ++i) negM[i] = -M[i];
    ok_lazy_a(6, 1, 6, negM, tmp, t->term.DeltaX); /* DeltaX_ = -M * (...) */
    t->sparse_size = 0;
    t->nlm = 0; /* landmarkParameterBlockId2idx_.clear() (the infos vector stays) */
    t->term.is_computed = 1;
    return 1;
}

int ok_twopose_convert(ok_twopose* t, const double T_WS0_live[7], double (*out_hp_W)[4], int max_out, int* out_dup) {
    ok_tf T_WS0;
    int g, o, n = 0, dup = 0, i;
    ok_tf_convert(&T_WS0, T_WS0_live);
    for (g = 0; g < t->ngroups; ++g) {
        ok_tp_group* grp = &t->groups[g];
        for (o = 0; o < grp->nobs; ++o) {
            ok_tp_obs* ob = &grp->obs[o];
            int k;
            if (!ob->is_marginalised) continue;
            if (ob->is_duplication) dup++;
            for (k = 0; k < t->nlm_S0; ++k)
                if (t->lm_S0[k].id == ob->hpoint_id) break;
            if (k == t->nlm_S0) return -1;
            if (n < max_out) ok_tf_mul_v4(&T_WS0, t->lm_S0[k].hp_S0, out_hp_W[n], 1); /* hp_W = T_WS0 * landmarks_.at(id) */
            n++;
        }
    }
    /* clear all */
    t->nlm = 0;
    for (g = 0; g < t->ngroups; ++g) free(t->groups[g].obs);
    t->ngroups = 0;
    t->nlm_S0 = 0;
    for (i = 0; i < 2 * OK_TP_MAXEXTR; ++i) t->extr_present[i] = 0;
    for (i = 0; i < 36; ++i) { t->term.J[i] = 0.0; t->H00[i] = 0.0; }
    for (i = 0; i < 6; ++i) t->b0[i] = 0.0;
    t->sparse_size = 0;
    if (out_dup) *out_dup = dup;
    return n;
}
