/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 *
 * Basalt port, module M1: evaluation-order models of the small fixed-size Eigen 3.4.0 kernels the Lie code uses,
 * for float (suffix f, SSE packet = 4 lanes) and double (suffix d, packet = 2 lanes), as compiled with the reference
 * flags g++ 13 -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, SSE2 baseline.
 * Every function was measured bit-exact against the real Eigen classes by basalt_port/reference_tools/bs_lie_test.cc.
 * Matrices are column-major (m[row + nrows*col]) like Eigen's default.
 *
 * Rules (measured, see basalt_port/HANDOVER.md "M1"):
 *   float  Vector3 squaredNorm / dot / M3*M3 / M3*v : every sum is  x0 + (x1 + x2)        (no packet fits in 3 rows)
 *   double Vector3 squaredNorm / dot               : (x0 + x1) + x2
 *   double M3*M3, M3*v : rows 0-1 (x0 + x1) + x2 (packet of 2 rows), row 2 x0 + (x1 + x2) (scalar tail)
 *   Vector4 squaredNorm (both types): (x0*x0 + x2*x2) + (x1*x1 + x3*x3)  (predux of the packet(s))
 *   float  M6*M6 : column j has alignedStart 0 (even j) / 2 (odd j): rows [start, start+4) are a packet (left fold
 *                   ((p0 + p1) + p2) ...), the other rows are scalar coefficients ((p0+(p1+p2)) + (p3+(p4+p5)))
 *   double M6*M6 : all rows packets (left fold)
 */
#ifndef BS_EIGENF_H
#define BS_EIGENF_H

#define BS_EIGEN_DECL(S, X) \
  S bs_v3##X##_sqn(const S v[3]);                                   /* v.squaredNorm() */ \
  S bs_v3##X##_dot(const S a[3], const S b[3]);                     /* a.dot(b) */ \
  void bs_v3##X##_cross(const S a[3], const S b[3], S out[3]);      /* a.cross(b) */ \
  int bs_v3##X##_normalized(const S v[3], S out[3]);                /* v.normalized() (v/sqrt(sqn) if sqn > 0, else v); returns sqn > 0 */ \
  S bs_q##X##_sqn(const S q[4]);                                    /* Vector4 / quaternion coeffs squaredNorm */ \
  void bs_m3##X##_mul(const S a[9], const S b[9], S out[9]);        /* A*B (out may alias) */ \
  void bs_m3##X##_mulv(const S a[9], const S v[3], S out[3]);       /* A*v (out may alias v) */ \
  void bs_m3##X##_transpose(const S a[9], S out[9]); \
  void bs_m6##X##_mul(const S a[36], const S b[36], S out[36]);     /* A*B, 6x6 (out may alias) */

BS_EIGEN_DECL(float, f)
BS_EIGEN_DECL(double, d)

#endif
