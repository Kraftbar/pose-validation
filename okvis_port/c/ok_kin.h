/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 2b: kinematics (okvis_kinematics: Transformation, operators.hpp, sinc, deltaQ,
 * rightJacobian) plus the Eigen quaternion helpers they use.
 *
 * Derived from OKVIS2 (okvis_kinematics/include/okvis/kinematics/{Transformation,operators}.hpp, implementation):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The evaluation-order models of the Eigen 3.4.0
 *   expressions (quaternion from rotation matrix, small matrix products, redux) are MPL-2.0 (Eigen,
 *   Copyright (C) Gael Guennebaud, Benoit Jacob and the Eigen authors). Redistribution requires retaining these
 *   notices; the names of ETH Zurich, Imperial College London and TUM may not be used to endorse derived products.
 *
 * C99, <math.h> <string.h> only. Matrices are column-major. A pose is [r(3), q(x,y,z,w)] like
 * Transformation::coeffs(); C is the cached rotation matrix of Transformation<CACHE_C = true>. Functions that
 * differ between Transformation (cached) and TransformationCacheless take `cached`.
 *
 * ---- record layouts of the reference dumps (patch 0005; native endian; f64/u32; "pose" below) ----
 *   pose = u32 cache, f64 coeffs[7] (r, q xyzw), and f64 C[9] when cache == 1
 *   kin_ctor_rq.bin   : f64 r[3], f64 q[4], pose(out)                 Transformation(r, q)
 *   kin_ctor_m4.bin   : f64 M[16], pose(out)                          Transformation(Matrix4d)
 *   kin_set_m4.bin    : f64 M[16], pose(out)                          set(Matrix4d)
 *   kin_set_rq.bin    : f64 r[3], f64 q[4], pose(out)                 set(r, q)
 *   kin_setcoeffs.bin : f64 c[7], pose(out)                           setCoeffs
 *   kin_convert.bin   : f64 coeffs[7] (other), pose(out, cached)      copy/move ctor from a cacheless Transformation
 *   kin_inv.bin       : pose(in), pose(out)                           inverse()
 *   kin_mul_t.bin     : pose(lhs), pose(rhs), pose(out)               operator*(Transformation)
 *   kin_mul_v3.bin    : pose(T), f64 v[3], f64 out[3]                 operator*(Vector3d)
 *   kin_mul_v4.bin    : pose(T), f64 v[4], f64 out[4]                 operator*(Vector4d)
 *   kin_oplus.bin     : pose(in), f64 delta[6], pose(out)             oplus(delta)
 *   kin_oplusj.bin    : pose, f64 J[42] (7x6)                         oplusJacobian
 *   kin_liftj.bin     : pose, f64 J[42] (6x7)                         liftJacobian
 *   kin_t4.bin        : pose, f64 T[16]                               T()
 *   kin_t3x4.bin      : pose, f64 T[12]                               T3x4()
 *   kin_c.bin         : pose (cacheless), f64 C[9]                    C() on a TransformationCacheless
 *   kin_sinc.bin      : f64 x, f64 out                                kinematics::sinc
 *   kin_deltaq.bin    : f64 dAlpha[3], f64 q[4]                       kinematics::deltaQ
 *   kin_rjac.bin      : f64 Phi[3], f64 J[9]                          kinematics::rightJacobian
 */
#ifndef OK_KIN_H
#define OK_KIN_H

#include "ok_eigen.h"

typedef struct ok_tf { double r[3]; ok_quat q; double C[9]; } ok_tf; /* C valid only for cached poses */

/* --- helpers from operators.hpp / Eigen --- */
void ok_kin_cross_mx(const double v[3], double out[9]);       /* crossMx */
void ok_kin_plus(const ok_quat* q, double out[16]);           /* plus(q_AB) */
void ok_kin_oplus(const ok_quat* q, double out[16]);          /* oplus(q_BC) */
double ok_kin_sinc(double x);
ok_quat ok_kin_delta_q(const double dAlpha[3]);               /* deltaQ */
void ok_kin_right_jacobian(const double phi[3], double out[9]);
/* Eigen::Quaterniond::operator=(const Matrix3d&) (Shoemake), trace via redux d0+(d1+d2) */
ok_quat ok_quat_from_mat3(const double m[9]);

/* --- Transformation --- */
void ok_tf_identity(ok_tf* t);                                                 /* default ctor / setIdentity */
void ok_tf_from_rq(ok_tf* t, const double r[3], const ok_quat* q, int cached); /* ctor(r,q) == set(r,q): q normalised */
void ok_tf_from_m4(ok_tf* t, const double m[16], int cached);                  /* ctor(Matrix4d): C = block verbatim */
void ok_tf_set_m4(ok_tf* t, const double m[16], int cached);                   /* set(Matrix4d) */
void ok_tf_set_coeffs(ok_tf* t, const double c[7], int cached);                /* setCoeffs / setParameters */
void ok_tf_convert(ok_tf* t, const double coeffs[7]);                          /* ctor/assign from the other cache mode */
void ok_tf_inverse(const ok_tf* t, ok_tf* out, int cached);
void ok_tf_mul(const ok_tf* a, const ok_tf* b, ok_tf* out, int cached);        /* a * b */
void ok_tf_mul_v3(const ok_tf* t, const double v[3], double out[3], int cached);
void ok_tf_mul_v4(const ok_tf* t, const double v[4], double out[4], int cached);
void ok_tf_oplus(ok_tf* t, const double delta[6], int cached);                 /* oplus(delta) in place */
void ok_tf_oplus_jacobian(const ok_tf* t, double J[42]);                       /* 7x6 */
void ok_tf_lift_jacobian(const ok_tf* t, double J[42]);                        /* 6x7 */
void ok_tf_T4(const ok_tf* t, double out[16], int cached);                     /* T() */
void ok_tf_T3x4(const ok_tf* t, double out[12], int cached);                   /* T3x4() */
void ok_tf_C(const ok_tf* t, double out[9], int cached);                       /* C() */

#endif
