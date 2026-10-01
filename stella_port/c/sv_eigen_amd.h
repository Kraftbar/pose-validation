/* SPDX-License-Identifier: MPL-2.0 */
#ifndef SV_EIGEN_AMD_H
#define SV_EIGEN_AMD_H

/* Eigen::AMDOrdering<int> (external/eigen/Eigen/src/OrderingMethods/
 * Ordering.h and Amd.h: approximate minimum degree ordering with
 * quotient-graph element absorption, aggressive absorption, mass
 * elimination, supernode (indistinguishable node) detection and
 * postordering of the assembly tree) -- MPL-2.0. The AMD routine itself is
 * adapted upstream (by Eigen) from Timothy Davis's CSparse under a licence
 * Davis granted to Google for MPL-2.0 distribution inside Eigen (see the
 * notice at the top of Amd.h); this file is a C transliteration of Eigen's
 * MPL-2.0 file, not of CSparse.
 *
 * `sv_amd_order` reproduces what g2o's LinearSolverEigen does with the
 * block sparsity pattern of the Schur-reduced pose matrix
 * (`AMDOrdering<int> ordering; ordering(auxBlockMatrix, blockP)`):
 *   1. symmetrize the pattern: symm = pattern(A^T) union pattern(A)
 *      (`ordering_helper_at_plus_a`: C = A.transpose() with zeroed values,
 *      symm = C + A; the union is column-major with sorted row indices),
 *   2. run `minimum_degree_ordering` on it.
 * The output permutation is the elimination/postorder list: perm[k] is the
 * node eliminated k-th (Eigen uses it directly as a PermutationMatrix
 * indices vector; sv_eigen_llt.c consumes it exactly as g2o's
 * blockToScalarPermutation does).
 */

/* n: number of columns/rows (blocks). Ap[n+1], Ai[Ap[n]]: CSC pattern of
 * the (possibly triangular) input; row indices within a column may be in
 * any order but each column's must be duplicate-free. perm: out, n ints.
 * Returns 0 on success. */
int sv_amd_order(int n, const int* Ap, const int* Ai, int* perm);

/* Identity for a single fully-dense NxN block -- kept for the
 * pose-optimizer leaf (equal to sv_amd_order on a 1x1 pattern). */
void sv_pose_optimizer_amd_order(int n, int perm[/*n*/]);

#endif /* SV_EIGEN_AMD_H */
