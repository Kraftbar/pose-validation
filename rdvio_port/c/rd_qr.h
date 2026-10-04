/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * C99 port of Eigen 3.4's Eigen::ColPivHouseholderQR<MatrixXd> (dynamic,
 * double scalar), restricted to the shapes stella_vslam's JacobiSVD
 * QR-preconditioning step actually needs (rows >= cols, column-major,
 * ComputeFull{U,V} bookkeeping only insofar as the SVD port needs it).
 * Follows (clean room, Eigen source only, MPL-2.0):
 *   external/eigen/Eigen/src/QR/ColPivHouseholderQR.h
 *   external/eigen/Eigen/src/Householder/Householder.h
 *   external/eigen/Eigen/src/Householder/HouseholderSequence.h
 *   external/eigen/Eigen/src/Core/products/GeneralMatrixVector.h
 *     (RowMajor specialization -- this is the kernel Eigen actually
 *      dispatches to for `essential.adjoint() * bottom` inside
 *      applyHouseholderOnTheLeft; its column-blocked pairwise-interleaved
 *      accumulation is replicated exactly in sv_eigen_qr.c so the QR's R
 *      factor -- and everything the SVD derives from it -- stays bit-exact
 *      under the reference build's baseline SSE2 (2-wide double packets),
 *      -ffp-contract=off, -fno-fast-math flags: see runs/stella_port/
 *      reference_build/provenance.json).
 */
#ifndef RD_QR_H
#define RD_QR_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Column-pivoting Householder QR of an m x n (rows >= cols) double matrix,
 * column-major storage, leading dimension == rows (Eigen's plain MatrixXd
 * layout: no padding). Mirrors ColPivHouseholderQR<MatrixXd>::computeInPlace
 * for the real scalar case.
 *
 * On return:
 *   qr        overwritten in place: strict-upper (incl. diagonal) holds R,
 *             strictly-below-diagonal of each column k holds that column's
 *             Householder "essential" vector (length rows-k-1).
 *   hcoeffs   length cols, Householder tau per column.
 *   perm      length cols; colsPermutation() as a dense index vector:
 *             perm[j] is the index (into the ORIGINAL, unpivoted columns)
 *             occupying pivoted position j -- i.e. matrixV()==this
 *             permutation means V(perm[j], j) == 1, all other entries 0.
 *   nonzero_pivots, maxpivot mirror the Eigen members of the same name
 *   (bookkeeping only; the decomposition itself never early-exits on
 *   this repo's shapes -- see the loop in ColPivHouseholderQR.h).
 */
void rd_qr_colpiv(double *qr, int rows, int cols,
                         double *hcoeffs, int *perm,
                         int *nonzero_pivots, double *maxpivot, int dot_mode);

/* Builds the full rows x rows orthogonal factor Q (column-major, leading
 * dimension rows) from a QR produced by rd_qr_colpiv, i.e.
 * m_qr.householderQ().evalTo(...) for the "non-blocked" HouseholderSequence
 * path (always taken here: min(rows,cols) <= 9 is always < Eigen's
 * BlockSize==48). Only the `cols` leading Householder reflectors are used
 * (that's all ColPivHouseholderQR stores), matching stella's usage where
 * this is only ever called on small (<=9-wide) QRs.
 */
void rd_qr_colpiv_householderq_full(const double *qr, int rows, int cols,
                                           const double *hcoeffs, double *Q, int dot_mode);

#ifdef __cplusplus
}
#endif

#endif /* RD_QR_H */
