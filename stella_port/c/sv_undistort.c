/* SV_PORT_SOURCES: sv_undistort.c
 * SPDX-License-Identifier: BSD-2-Clause AND BSD-3-Clause
 *
 * Port of OpenCV 4.6.0 modules/calib3d/src/undistort.dispatch.cpp
 * cvUndistortPointsInternal(), specialized to: matR=I, matP=cameraMatrix,
 * 5-element (k1,k2,p1,p2,k3) distortion (no thin-prism/tilt terms, i.e.
 * k[8..13]=0), which is exactly what
 * stella_vslam/camera/perspective::undistort_point()/undistort_keypoints()
 * calls with. See sv_undistort.h.
 *
 * -----------------------------------------------------------------------
 *                           License Agreement
 *                For Open Source Computer Vision Library
 *                        (3-clause BSD License)
 * Copyright (C) 2000-2020, Intel Corporation, all rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: retain the above copyright notice and this list of conditions in
 * source form; reproduce them in binary-form documentation; do not use the
 * names of the copyright holders/contributors to endorse derived products
 * without permission. Provided "as is", without warranty; see
 * https://opencv.org/license/ for the complete text.
 * -----------------------------------------------------------------------
 */
#include "sv_undistort.h"
#include <math.h>

void sv_undistort_point(const sv_camera_params* cam, float x_in, float y_in,
                         float* x_out, float* y_out) {
    /* stella_vslam/camera/perspective.cc stores cv_cam_matrix_/
     * cv_dist_params_ as cv::Mat_<float> (CV_32F), not double --
     * cvUndistortPointsInternal's cvConvert(_cameraMatrix,&matA) /
     * cvConvert(_distCoeffs,&_Dk) then widen those already-float32-rounded
     * values to double. Reproduced here by rounding every intrinsic/
     * distortion parameter through float first. */
    const double fx = (double)(float)cam->fx, fy = (double)(float)cam->fy;
    const double cx = (double)(float)cam->cx, cy = (double)(float)cam->cy;
    const double ifx = 1.0 / fx, ify = 1.0 / fy;
    const double k0 = (double)(float)cam->k1, k1c = (double)(float)cam->k2;
    const double k2c = (double)(float)cam->p1, k3c = (double)(float)cam->p2;
    const double k4c = (double)(float)cam->k3;
    /* k[5..13] = 0 (no thin-prism/tilt terms in the reference config). */

    double x = ((double)x_in - cx) * ifx;
    double y = ((double)y_in - cy) * ify;
    double x0 = x, y0 = y;

    double error = 1.0e300;
    const int max_count = 20;
    const double epsilon = 1e-6;
    int j;
    for (j = 0; ; j++) {
        if (j >= max_count) break;
        if (error < epsilon) break;

        double r2 = x * x + y * y;
        /* Full formula per cvUndistortPointsInternal:
         *   icdist = (1 + ((k7 r2 + k6) r2 + k5) r2) / (1 + ((k4 r2 + k1) r2 + k0) r2)
         * k5=k6=k7=0 (5-coefficient model) -> numerator 1, but k4=k3
         * (cam->k3, k4c below) DOES appear in the denominator -- easy to
         * miss since it looks at first glance like a plain 2nd-order
         * radial term. */
        double icdist = 1.0 / (1.0 + ((k4c * r2 + k1c) * r2 + k0) * r2);
        if (icdist < 0.0) {
            x = ((double)x_in - cx) * ifx;
            y = ((double)y_in - cy) * ify;
            break;
        }
        double deltaX = 2.0 * k2c * x * y + k3c * (r2 + 2.0 * x * x);
        double deltaY = k2c * (r2 + 2.0 * y * y) + 2.0 * k3c * x * y;
        x = (x0 - deltaX) * icdist;
        y = (y0 - deltaY) * icdist;

        {
            double r2b = x * x + y * y;
            double r4b = r2b * r2b;
            double a1 = 2.0 * x * y;
            double a2 = r2b + 2.0 * x * x;
            double a3 = r2b + 2.0 * y * y;
            double cdist = 1.0 + k0 * r2b + k1c * r4b + k4c * r4b * r2b;
            double icdist2 = 1.0; /* 1/(1+k5 r2+k6 r4+k7 r6), all zero */
            double xd0 = x * cdist * icdist2 + k2c * a1 + k3c * a2;
            double yd0 = y * cdist * icdist2 + k2c * a3 + k3c * a1;
            double xd = xd0, yd = yd0; /* matTilt = I */
            double x_proj = xd * fx + cx;
            double y_proj = yd * fy + cy;
            double dx = x_proj - (double)x_in;
            double dy = y_proj - (double)y_in;
            error = sqrt(dx * dx + dy * dy);
        }
    }

    /* RR = matP * matR = cameraMatrix * I = cameraMatrix. */
    double xx = fx * x + 0.0 * y + cx;
    double yy = 0.0 * x + fy * y + cy;
    double ww = 1.0; /* RR[2] = [0,0,1] */
    x = xx * ww;
    y = yy * ww;

    *x_out = (float)x;
    *y_out = (float)y;
}
