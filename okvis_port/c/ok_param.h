/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 3a: parameter blocks and manifolds (okvis_ceres: PoseManifold / PoseLocalParameterization,
 * HomogeneousPointManifold / HomogeneousPointLocalParameterization, PoseParameterBlock, SpeedAndBiasParameterBlock,
 * HomogeneousPointParameterBlock).
 *
 * Derived from OKVIS2 (okvis_ceres/include/okvis/ceres/{PoseLocalParameterization,HomogeneousPointLocalParameterization,
 * PoseParameterBlock,SpeedAndBiasParameterBlock,HomogeneousPointParameterBlock}.hpp, src/<same names>.cpp):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The evaluation-order models of the Eigen 3.4.0
 *   expressions (small lazy products) are MPL-2.0 (Eigen, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen
 *   authors). Redistribution requires retaining these notices; the names of ETH Zurich, Imperial College London and
 *   TUM may not be used to endorse derived products.
 *
 * C99, <math.h> <stdint.h> <string.h> only. A pose parameter block is [r(3), q(x,y,z,w)] (7 doubles), a speed-and-bias
 * block [v(3), b_g(3), b_a(3)], a homogeneous point block [x y z w]. Jacobians are ROW-major like the Eigen::RowMajor
 * maps of the C++ code (plusJacobian of a pose: 7x6, minusJacobian: 6x7; homogeneous point: 4x3 / 3x4; speed and bias:
 * 9x9 identity). Functions return 1 (the C++ ones return true) -- except where noted.
 *
 * Not ported: ParameterBlock/ParameterBlockSized bookkeeping (ids, fixed flag, timestamps: plain data, no arithmetic),
 * PoseManifold::verifyJacobianNumDiff (debug), the Ceres Manifold virtual wrappers (Plus == plus etc.).
 *
 * ---- record layouts of the reference dumps (patch 0007; native endian; f64/u32; matrices as written in memory, i.e.
 * the row-major Jacobian buffers) ----
 *   err_pplus.bin    : f64 x[7], f64 delta[6], u32 ret, f64 out[7]       PoseManifold::plus
 *   err_pplusj.bin   : f64 x[7], u32 ret, f64 J[42]                      PoseManifold::plusJacobian (7x6 row-major)
 *   err_pminus.bin   : f64 y[7], f64 x[7], u32 ret, f64 out[6]           PoseManifold::minus(y, x)
 *   err_pminusj.bin  : f64 x[7], u32 ret, f64 J[42]                      PoseManifold::minusJacobian (6x7 row-major)
 *   err_hplus.bin    : f64 x[4], f64 delta[3], u32 ret, f64 out[4]       HomogeneousPointManifold::plus
 *   err_hplusj.bin   : f64 x[4], u32 ret, f64 J[12]                      plusJacobian (4x3 row-major)
 *   err_hminus.bin   : f64 y[4], f64 x[4], u32 ret, f64 out[3]           minus(y, x)
 *   err_hminusj.bin  : f64 x[4], u32 ret, f64 J[12]                      minusJacobian (3x4 row-major)
 */
#ifndef OK_PARAM_H
#define OK_PARAM_H
#include "ok_kin.h"

/* ---- PoseManifold (7 ambient, 6 tangent) ---- */
int ok_pose_plus(const double x[7], const double delta[6], double out[7]);
int ok_pose_plus_jacobian(const double x[7], double J[42]);            /* 7x6 row-major */
int ok_pose_minus(const double y[7], const double x[7], double out[6]);  /* y (-) x */
int ok_pose_minus_jacobian(const double x[7], double J[42]);           /* 6x7 row-major (the "lift") */

/* ---- HomogeneousPointManifold (4 ambient, 3 tangent) ---- */
int ok_hpoint_plus(const double x[4], const double delta[3], double out[4]);
int ok_hpoint_plus_jacobian(const double x[4], double J[12]);          /* 4x3 row-major */
int ok_hpoint_minus(const double y[4], const double x[4], double out[3]);
int ok_hpoint_minus_jacobian(const double x[4], double J[12]);         /* 3x4 row-major */

/* ---- SpeedAndBiasParameterBlock (Euclidean 9) ---- */
void ok_sab_plus(const double x[9], const double delta[9], double out[9]);
void ok_sab_plus_jacobian(double J[81]);                               /* identity, row-major */
void ok_sab_minus(const double x0[9], const double x0_plus_delta[9], double out[9]);
void ok_sab_minus_jacobian(double J[81]);                              /* identity (liftJacobian) */

/* ---- parameter block <-> estimate conversions ---- */
/* PoseParameterBlock::setEstimate(Transformation) : estimate is a TransformationCacheless, coeffs stored verbatim */
void ok_pose_block_set_estimate(double block[7], const ok_tf* T);
/* PoseParameterBlock::estimate() converted to a cached Transformation (copy of the coeffs, C from q) */
void ok_pose_block_estimate(const double block[7], ok_tf* T);
/* PoseParameterBlock::setParameters: coeffs verbatim (no normalisation) */
void ok_pose_block_set_parameters(double block[7], const double params[7]);
/* HomogeneousPointParameterBlock(Vector3d) : (x, y, z, 1) */
void ok_hpoint_block_from_v3(double block[4], const double p[3]);
#endif
