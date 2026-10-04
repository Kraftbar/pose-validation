/* SPDX-License-Identifier: MPL-2.0 */
#ifndef SV_EIGEN_LU3_H
#define SV_EIGEN_LU3_H

/* Eigen::PartialPivLU<Matrix3d>(W).solve(b) -- MPL-2.0 (Eigen-derived
 * evaluation order):
 *   external/eigen/Eigen/src/LU/PartialPivLU.h
 *     (partial_lu_impl<...,3>::unblocked_lu: pivot = first maximum of |col|,
 *      column scaled by DIVISION with the pivot, rank-1 trailing update
 *      a(i,j) -= l(i)*u(j), no fused operations; _solve_impl: P*b, unit-lower
 *      then upper triangular solve)
 *   external/eigen/Eigen/src/Core/SolveTriangular.h
 *     (triangular_solver_unroller: a fixed rhs of size <= 8 is solved by
 *      complete unrolling, rhs(i) -= sum_j L(i,j)*rhs(j); rhs(i) /= L(i,i))
 * Matrices column-major (m[col*3+row]). Used by g2o::Sim3::log() (the
 * W.lu().solve(t) of the translation part). */
void sv_lu3_solve(const double w[9], const double b[3], double x[3]);

#endif
