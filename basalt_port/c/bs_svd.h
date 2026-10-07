/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0
 * Basalt port, module M9: Eigen 3.4.0 JacobiSVD<Matrix<float,4,4>>(A, ComputeFullV) (V and singular values; MPL-2.0 evaluation-order model of
 * Eigen/src/SVD/JacobiSVD.h, Jacobi/Jacobi.h, misc/RealSvd2x2.h, derived from rdvio_port/c/rd_svd.c) and
 * BundleAdjustmentBase<float>::triangulate (basalt/vi_estimator/ba_base.h, BSD-3-Clause, (c) 2019 Usenko, Demmel).
 * C99, float, column-major, bit-exact against g++ 13 -O2 -ffp-contract=off -fno-fast-math (SSE2). */
#ifndef BS_SVD_H
#define BS_SVD_H

#include "bs_lie.h"

/* JacobiSVD<Matrix4f, ComputeFullV>: A, V column-major 4x4, sv[4] descending.  Returns 0 (Success) or 1 (InvalidInput: a non-finite
 * coefficient; the C++ returns with matrixV() uninitialised, here V is set to zero). */
int bs_jacobisvd4f(const float A[16], float V[16], float sv[4]);

/* BundleAdjustmentBase<float>::triangulate(f0, f1, T_0_1) -> homogeneous point (unit direction, inverse distance) */
void bs_triangulate_f(const float f0[3], const float f1[3], const bs_se3f* T_0_1, float out[4]);

/* StereographicParam<float>::project(p3d) (Vec4 input, no Jacobian) */
void bs_stereographic_project_f(const float p3d[4], float res[2]);

#endif
