/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M8: rdvio::extra::OpenCvImage (rdvio_extra/src/opencv_image.cpp) on the bit-exact OpenCV
 * functions of module M7 (rd_cv.h). The image is the undistorted 8-bit frame the driver hands to Handler::track_camera.
 *   preprocess    : CLAHE(6, 8x8) in place (the C++ keeps a static CLAHE made with the first call's parameters), then the
 *                   21x21 / 3-level optical-flow pyramid with derivatives
 *   detect        : GFTT (Harris, quality 1e-3, min distance 20, block 3; max_points of the FIRST call: a static detector), the
 *                   second sort by response, the Poisson-disk filter against the existing keypoints, the 20 px border, appended
 *   track         : forward LK with initial flow (the predicted points, else the current ones), the image-side checks
 *                   (20 px border, rows/4 displacement), backward LK and the 0.5 px forward-backward check
 * Derived from RD-VIO (Apache-2.0). C99.
 */
#ifndef RD_SYS_IMAGE_H
#define RD_SYS_IMAGE_H
#include <stddef.h>
#include <stdint.h>
#include "rd_map.h"
#include "rd_cv.h"

typedef struct rd_sys_image {
    rd_image base;               /* first member: rd_frame keeps it as rd_image* */
    int w, h;
    uint8_t* image;              /* NULL after release_image_buffer */
    rd_cv_pyramid pyr;
    int have_pyr;
} rd_sys_image;

rd_sys_image* rd_sys_image_new(double t, const uint8_t* gray, int w, int h);   /* refs = 1, pixels copied */
void rd_sys_image_preprocess(rd_sys_image* im, double clip_limit, int tiles_w, int tiles_h);
/* keypoints: in/out pixel list (n x 2, malloc'd, may be reallocated) */
int rd_sys_image_detect(rd_sys_image* im, double** keypoints, size_t* n, size_t max_points, double keypoint_distance);
/* curr: n x 2; next: n x 2 in/out (predicted positions if have_next, else ignored and set like the C++: zeros, tracked ones
 * overwritten); status: n */
int rd_sys_image_track(const rd_sys_image* im, const rd_sys_image* next_im, const double* curr, double* next, int have_next,
                       char* status, size_t n);
void rd_sys_image_release_buffer(rd_sys_image* im);
void rd_sys_image_reset_statics(void);   /* the static CLAHE / GFTT parameters (one process = one run) */

#endif
