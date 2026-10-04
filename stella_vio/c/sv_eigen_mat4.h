/* SPDX-License-Identifier: MPL-2.0 */
#ifndef SV_EIGEN_MAT4_H
#define SV_EIGEN_MAT4_H

/* Eigen::Matrix4d * Eigen::Matrix4d evaluation order (MPL-2.0, Eigen-derived
 * behaviour: CoeffBasedProduct / etor_product_packet_impl<ColMajor,...>,
 * SSE2 baseline, -ffp-contract=off). Matrices are column-major
 * (m[col*4+row]). Used for stella_vslam's pose compositions
 * (velocity * pose_cw, curr_pose * ref_pose_wc, ...). Measured against real
 * Eigen 3.4 by stella_port/reference_tools/eigen_shape_tests_track.cc. */
void sv_mat4_mul(const double a[16], const double b[16], double out[16]);

#endif
