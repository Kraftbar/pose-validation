/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_TRIANGULATE_H
#define SV_TRIANGULATE_H

/* Port of stella_vslam's solve/triangulator.h::triangulate(bearing_1,
 * bearing_2, rot_21, trans_21) closed-form (2x2 linear system) overload --
 * the ONLY triangulate() overload initialize::base::triangulate() calls
 * (confirmed by reading initialize/base.cc: no SVD needed here) -- and
 * camera::perspective::reproject_to_image(), both BSD-2 (AIST 2019 +
 * stella-cv 2022, see sv_rng.h). Column-major 3x3/Vec3 (sv_linalg.h). */

void sv_triangulate_bearings(const double bearing1[3], const double bearing2[3],
                              const double rot21[9], const double trans21[3],
                              double pos_c_in_ref[3]);

typedef struct sv_camera_perspective {
    double fx, fy, cx, cy;
    double focal_x_baseline; /* 0 for monocular */
    float min_x, max_x, min_y, max_y; /* img_bounds_, from sv_frame.h's sv_image_bounds */
} sv_camera_perspective;

/* Returns 1 if pos_c(2) > 0 AND the projected point is strictly inside
 * img_bounds; reproj always written (even on early-out at pos_c(2)<=0 it
 * is left untouched, matching stella's early return before computing it --
 * callers must not read reproj when this returns 0 for that reason). */
int sv_camera_reproject_to_image(const sv_camera_perspective* cam,
                                  const double rot_cw[9], const double trans_cw[3],
                                  const double pos_w[3], double reproj[2]);

/* camera::perspective::convert_point_to_bearing() (undist_pt in pixel
 * coords, float precision as stored in cv::KeyPoint, widened to double for
 * the arithmetic -- matches stella's own float->double widen there). */
void sv_camera_convert_point_to_bearing(const sv_camera_perspective* cam,
                                         float undist_x, float undist_y,
                                         double bearing[3]);

#endif /* SV_TRIANGULATE_H */
