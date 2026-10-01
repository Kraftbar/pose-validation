/* SPDX-License-Identifier: MPL-2.0 */
#ifndef SV_EIGEN_LLT_H
#define SV_EIGEN_LLT_H

/* g2o's actual pose-optimizer linear solver: NOT SimplicialLDLT --
 * `g2o::LinearSolverEigen<BlockSolver_6_3::PoseMatrixType>` (see
 * external/candidates/g2o/g2o/solvers/eigen/linear_solver_eigen.h) wraps
 * `Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>` (plain Cholesky,
 * L*L^T, sqrt on the diagonal), with `AMDOrdering<int>` for the
 * permutation. (This header/file used to be named sv_eigen_ldlt.{h,c} and
 * implemented the LDLT numeric algorithm -- wrong solver entirely; see
 * stella_port/HANDOVER.md module-4b "bit-exact closure" note for how that
 * was found: comparing dx after the linear solve against a real g2o
 * replay showed dx differing while H/b matched bit-exact, which meant the
 * solve step itself was wrong, not just its evaluation order.) -- MPL-2.0
 * (Eigen-derived):
 *   external/eigen/Eigen/src/SparseCholesky/SimplicialCholesky_impl.h
 *     (SimplicialCholeskyBase::factorize_preordered, DoLDLT=false path:
 *     divides by the already-computed L(i,i)=sqrt(d_i) instead of a
 *     separate D array, and stores the diagonal inline in L itself)
 *   external/eigen/Eigen/src/SparseCholesky/SimplicialCholesky.h
 *     (_solve_impl: forward/back substitution against the REAL (non-unit)
 *     triangular L/L^T -- no separate diagonal-inverse-multiply step,
 *     unlike LDLT)
 *
 * Scope, AMD, and the identity-permutation justification: same as the
 * old file -- see sv_eigen_amd.h. Source-value convention: g2o's
 * `SparseBlockMatrix::fillCCS(Cx, upperTriangle=true)` (sparse_block_
 * matrix.hpp) copies, for each column c of the (single, 6x6) pose block,
 * rows 0..c -- i.e. entries H(row,col) with row<=col, the UPPER triangle
 * -- into the sparse matrix fed to SimplicialLLT. Because this port's own
 * H is a full (not-quite-symmetric-to-the-last-bit, like g2o's own) 6x6
 * array, reading the WRONG triangle (row>=col) silently gives a
 * mathematically-close but not bit-identical input to the factorization
 * -- this was the other half of the original ~1e-9 residual.
 */

#define SV_LLT_MAXN 6

typedef struct sv_llt6 {
    double L[SV_LLT_MAXN][SV_LLT_MAXN]; /* real (non-unit) lower triangular, incl. diagonal */
    int n;
    int ok;
} sv_llt6;

/* H: dense NxN (only H[i][j] for i<=j READ -- the "upper triangle" per
 * fillCCS's convention, see header), N<=SV_LLT_MAXN, identity
 * permutation assumed (verified for this leaf's only real use case, see
 * sv_eigen_amd.h). */
void sv_llt6_factorize(const double H[SV_LLT_MAXN][SV_LLT_MAXN], int n, sv_llt6* out);

/* Solves H*x = b using the factor from sv_llt6_factorize. */
void sv_llt6_solve(const sv_llt6* f, const double b[SV_LLT_MAXN], double x[SV_LLT_MAXN]);

/* ------------------------------------------------------------------------
 * General sparse SimplicialLLT<SparseMatrix<double,ColMajor,int>, Upper>
 * with an explicit fill-reducing permutation, as g2o's
 * LinearSolverEigen::CholeskyDecomposition drives it (module 4b part 2:
 * local/global BA reduced camera system). MPL-2.0 (Eigen-derived):
 *   external/eigen/Eigen/src/SparseCholesky/SimplicialCholesky.h
 *     (analyzePattern/factorize/_solve_impl, m_P/m_Pinv handling)
 *   external/eigen/Eigen/src/SparseCholesky/SimplicialCholesky_impl.h
 *     (analyzePattern_preordered: elimination tree + column counts;
 *      factorize_preordered<false>: up-looking Cholesky on the etree
 *      pattern -- adapted upstream from Davis's LDL under an MPL-2.0
 *      licence grant, see the notice at the top of that file)
 *   external/eigen/Eigen/src/SparseCore/SparseSelfAdjointView.h
 *     (permute_symm_to_symm<Upper,Upper> == `twistedBy`)
 *   external/eigen/Eigen/src/SparseCore/TriangularSolver.h
 *     (column-major lower / row-major upper (L^T) sparse triangular solves)
 * The caller (sv_g2o_ba.c) supplies the UPPER-triangle scalar CCS of the
 * Schur matrix in g2o's fillCCS order and the scalar permutation from
 * `blockToScalarPermutation` applied to sv_amd_order's block permutation
 * (`analyzePatternWithPermutation`: m_Pinv = permutation, m_P =
 * permutation.inverse()).
 * ------------------------------------------------------------------------ */
typedef struct sv_sllt {
    int n;
    int nnz_a; /* stored entries of the (upper) input */
    int nnz_l;
    int ok;
    int* P;    /* m_P.indices(): P[orig] = permuted position */
    int* Pinv; /* m_Pinv.indices(): Pinv[permuted] = orig */
    int* ap_p; /* permuted upper matrix `tmp` (CCS) */
    int* ap_i;
    double* ap_x;
    int* parent;
    int* nz_per_col;
    int* Lp;
    int* Li;
    double* Lx;
    double* y;
    int* pattern;
    int* tags;
    double* work; /* n doubles for the permuted rhs */
} sv_sllt;

/* (a_p, a_i, a_x): upper-triangle CCS of the n x n matrix, scalar_perm:
 * the list `scalarP.indices()`. Runs analyzePatternWithPermutation. */
void sv_sllt_analyze(sv_sllt* f, int n, const int* a_p, const int* a_i, const int* scalar_perm);

/* numeric factorization with new values a_x (same pattern as analyze).
 * Returns 1 on success (info == Success), 0 if not positive definite. */
int sv_sllt_factorize(sv_sllt* f, const double* a_p_x, const int* a_p, const int* a_i);

/* x = A^-1 b via the last factorization (P b, L, L^T, P^-1). */
void sv_sllt_solve(const sv_sllt* f, const double* b, double* x);

void sv_sllt_free(sv_sllt* f);

#endif /* SV_EIGEN_LLT_H */
