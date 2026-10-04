/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/*
 * RD-VIO pure-C port: Lie-algebra helpers (rdvio_geometry/lie_algebra.{h,cpp}) and the Eigen 3.4.0 expression models
 * they need (Quaternion * Vector3 = _transformVector, AngleAxis <-> Quaternion, stableNormalized, 3x3 inverse).
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0, itself derived from XRSLAM, Copyright 2022 XRSLAM Authors; translated to C99,
 * modified). Eigen-derived evaluation-order models: MPL-2.0 (Eigen 3.4.0, Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen
 * authors). See rdvio_port/NOTICE. Matrices are column-major, quaternions are Eigen coefficient order (x, y, z, w).
 * Reference flags: g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, SSE2 baseline, no FMA.
 */
#ifndef RD_LIE_H
#define RD_LIE_H
#include "../../okvis_port/c/ok_eigen.h"

void rd_hat(const double w[3], double out[9]);
ok_quat rd_expmap(const double w[3]);          /* AngleAxisd(w.norm(), w.stableNormalized()) -> Quaterniond */
void rd_logmap(const ok_quat* q, double out[3]); /* AngleAxisd(q); angle * axis */
void rd_right_jacobian(const double w[3], double out[9]);
void rd_s2_tangential_basis(const double x[3], double out[6]);        /* 3x2, column-major */
void rd_s2_tangential_basis_barrel(const double x[3], double out[6]); /* 3x2 */

ok_quat rd_quat_conj(ok_quat q);
void rd_quat_rotate(const ok_quat* q, const double v[3], double out[3]); /* q * v  (QuaternionBase::_transformVector) */
void rd_inverse3(const double m[9], double out[9]);                        /* Matrix3d::inverse() */

#endif
