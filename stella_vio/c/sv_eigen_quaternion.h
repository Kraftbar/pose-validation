/* SPDX-License-Identifier: MPL-2.0 */
#ifndef SV_EIGEN_QUATERNION_H
#define SV_EIGEN_QUATERNION_H

/* Double-precision quaternion primitives reproducing real Eigen 3.4's
 * exact formulas and evaluation order for the operations g2o's SE3Quat
 * needs -- MPL-2.0 (Eigen-derived):
 *   external/eigen/Eigen/src/Geometry/Quaternion.h
 *     (QuaternionBase::toRotationMatrix, ::_transformVector,
 *     internal::quaternionbase_assign_impl<Other,3,3>::run -- Shoemake's
 *     1987 SIGGRAPH-course rotation-matrix-to-quaternion algorithm)
 *   external/eigen/Eigen/src/Geometry/arch/Geometry_SIMD.h
 *     (quat_product<Architecture::Target, ..., double> -- the SSE2/ARM64
 *     packet-vectorized quaternion product Eigen dispatches to for
 *     Quaternion<double> on any Eigen build with EIGEN_VECTORIZE_SSE or
 *     AArch64; NOT the plain scalar quat_product<Arch=0,...> formula --
 *     see sv_eigen_quaternion.c for the addsub/preverse lane algebra
 *     unrolled into scalar form, matched term-by-term.)
 * Coefficient order matches Eigen::Quaterniond::coeffs(): [x, y, z, w].
 * Reference build flags assumed: -O2 -DNDEBUG -ffp-contract=off
 * (no FMA contraction), SSE2 baseline -- consistent with the module-3
 * "Eigen 3x3 double evaluation-order rules" measurements in HANDOVER.md.
 * `sv_quat_normalize`'s squaredNorm/division and `sv_quat_mul`,
 * `sv_quat_from_mat3`/`sv_quat_to_mat3` (via `sv_se3_exp`/`_compose`'s
 * real-Eigen-instrumented replay, see stella_port/HANDOVER.md module-4b
 * "bit-exact closure" note) are all measured bit-exact against a real
 * Eigen 3.4 build with these exact reference flags.
 */

typedef struct sv_quat {
    double x, y, z, w;
} sv_quat;

/* Quaternion product a*b (Hamilton product), scalar unrolling of Eigen's
 * SSE2 quat_product<double> specialization -- see .c for the derivation. */
void sv_quat_mul(const sv_quat* a, const sv_quat* b, sv_quat* out);

/* QuaternionBase::_transformVector: rotates v by q (q assumed unit norm). */
void sv_quat_map(const sv_quat* q, const double v[3], double out[3]);

/* QuaternionBase::toRotationMatrix -- column-major 3x3 output (out[col*3+row]). */
void sv_quat_to_mat3(const sv_quat* q, double out[9]);

/* internal::quaternionbase_assign_impl<Other,3,3>::run -- rotation matrix
 * (column-major, out convention as above) to unit quaternion, Shoemake's
 * trace-branch algorithm. `m` is column-major 3x3 (m[col*3+row]). */
void sv_quat_from_mat3(const double m[9], sv_quat* out);

/* QuaternionBase::normalize() == coeffs().normalize() (Vector4d, coeffs
 * order x,y,z,w): divides by sqrt(squaredNorm()), in place. squaredNorm's
 * 4-term reduction is measured bit-exact as the SSE2 packet-pairwise
 * grouping `(x*x + z*z) + (y*y + w*w)` (interleaved lanes: [x,y] and
 * [z,w] packets added elementwise to [x*x+z*z, y*y+w*w], then a
 * horizontal add) -- NOT sequential `((x*x+y*y)+z*z)+w*w` (measured
 * ~30% mismatch against real Eigen) nor `(x*x+y*y)+(z*z+w*w)` (measured
 * ~27% mismatch). division, not multiply-by-reciprocal (matches the
 * Vector3d `normalized()` rule in HANDOVER.md's table). */
void sv_quat_normalize(sv_quat* q);

#endif /* SV_EIGEN_QUATERNION_H */
