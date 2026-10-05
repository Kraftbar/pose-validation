/* SPDX-License-Identifier: MPL-2.0 */
/* RD-VIO port M5: Eigen 3.4.0 SelfAdjointEigenSolver<MatrixXd> evaluation-order model for dynamic n <= RD_EIG_MAX (see rd_seig.c). */
#ifndef RD_SEIG_H
#define RD_SEIG_H
#define RD_EIG_MAX 300
/* a: n x n column-major, only the lower triangle is read. evals ascending, evecs n x n column-major (eigenvectors in columns).
 * hc_parity (alignment parity in doubles of the solver's m_hcoeffs[0], 0 = 16-byte aligned = a heap VectorXd) only matters for n > 8.
 * Returns 0 on success, 1 NoConvergence, 2 bad n / out of memory. */
int rd_selfadjoint_eig(int n, const double* a, double* evals, double* evecs, int hc_parity);
#endif
