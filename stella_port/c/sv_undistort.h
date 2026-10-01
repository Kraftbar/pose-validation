/* SPDX-License-Identifier: BSD-2-Clause AND BSD-3-Clause */
#ifndef SV_UNDISTORT_H
#define SV_UNDISTORT_H

/* Port of stella_vslam's camera::perspective::undistort_keypoints() /
 * undistort_point() (runs/stella_port/reference_build/src/src/stella_vslam/
 * camera/perspective.cc), which is a thin wrapper (CV_32F cv::Mat marshalling
 * only) around OpenCV's cv::undistortPoints -> cvUndistortPointsInternal
 * (modules/calib3d/src/undistort.dispatch.cpp), called with
 * matR = identity, matP = cv_cam_matrix_ (so RR = cam_matrix), 5-element
 * distCoeffs (k1,k2,p1,p2,k3; k[5..13]=0, no tilt), and
 * TermCriteria(EPS|MAX_ITER, 20, 1e-6) (see the reference config's
 * Camera block + stella's undistort_keypoints call). All internal
 * arithmetic is double, matching OpenCV; only the final store back to a
 * CV_32F cv::Mat truncates to float, reproduced here by the float in/out
 * of sv_undistort_point. BSD-3, "Open Source Computer Vision Library" --
 * full text in sv_image.c.
 */
typedef struct sv_camera_params {
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
} sv_camera_params;

void sv_undistort_point(const sv_camera_params* cam, float x_in, float y_in,
                         float* x_out, float* y_out);

#endif /* SV_UNDISTORT_H */
