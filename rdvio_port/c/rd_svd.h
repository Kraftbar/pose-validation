/* SPDX-License-Identifier: MPL-2.0 */
/* JacobiSVD<Matrix<double, R, C>, ComputeFullV> with FIXED rows (5x9, 8x9) or DYNAMIC rows (Nx4), V and singular values only.
 * Eigen 3.4.0 evaluation-order model, MPL-2.0; see rd_svd.c. */
#ifndef RD_SVD_H
#define RD_SVD_H
/* A: column-major rows x cols (cols <= 9, rows >= 2); V: cols x cols column-major; sv[cols] descending. */
void rd_jacobisvd_Nxc_v(const double* A, int rows, int cols, double* V, double* sv, int* rank_out);
#endif
