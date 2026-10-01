/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_SOLVE_COMMON_H
#define SV_SOLVE_COMMON_H

/* Port of stella_vslam's solve/common.{h,cc} normalize() -- BSD-2
 * (AIST 2019 + stella-cv 2022, see sv_rng.h for the full notice). */

/* pts_x/pts_y: length n input keypoint coords (cv::Point2f precision).
 * norm_x/norm_y: length n output, float precision (matches
 * std::vector<cv::Point2f> normalized_pts_ -- stella accumulates the
 * centroid/L1-deviation and normalizes in float, only the transform
 * matrix is double (Eigen Mat33_t)). transform: column-major 3x3 (Eigen
 * Matrix3d convention, matches sv_linalg.h/sv_eigen_svd.h) s.t.
 * norm == transform * homogeneous(pt) up to the float/double split above. */
void sv_solve_normalize(const float* pts_x, const float* pts_y, int n,
                         float* norm_x, float* norm_y, double transform[9]);

#endif /* SV_SOLVE_COMMON_H */
