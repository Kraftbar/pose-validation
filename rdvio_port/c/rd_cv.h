/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * C99 specialization of OpenCV 4.6.0 image functions used by RD-VIO.
 * Retained upstream notices: ../reference_cv/LICENSE-OpenCV-source.
 */
#ifndef RD_CV_H
#define RD_CV_H
#include <stdint.h>
#include <stdlib.h>

typedef struct { float x, y; } rd_cv_point;
typedef struct {
    rd_cv_point pt;
    float size, angle, response;
    int32_t octave, class_id;
} rd_cv_keypoint;
/* All levels own a (width+42)*(height+42) allocation, ROI offset (21,21).
 * step is measured in pixels; deriv stores interleaved signed dx,dy. */
typedef struct {
    int width, height, step;
    uint8_t *image;
    int16_t *deriv;
} rd_cv_level;
typedef struct { int count; rd_cv_level level[4]; } rd_cv_pyramid;
/* Optional observations; buffers are valid only for the duration of emit. */
typedef struct {
    void (*emit)(void *user, const char *label, const void *data, size_t bytes);
    void *user;
} rd_cv_trace;

/* Images are contiguous grayscale, positive dimensions <= 16384. src==dst OK.
 * Returns 1 on success, 0 on invalid arguments/allocation failure. */
int rd_cv_clahe(const uint8_t *src, int width, int height, uint8_t *dst);
/* Pass a zero-initialized pyramid. Free it before building again. */
int rd_cv_build_pyramid(const uint8_t *src, int width, int height, rd_cv_pyramid *out);
void rd_cv_free_pyramid(rd_cv_pyramid *p);
/* 21x21, maxLevel=3, 30 iterations, epsilon .01, minEigThreshold 1e-4.
 * Initial flow is always used. err entries left unwritten by OpenCV on failure
 * are likewise left unchanged. Point arrays must contain finite coordinates
 * in [-1e6,1e6]; next is in/out. Empty point arrays are accepted. */
int rd_cv_lk(const rd_cv_pyramid *prev, const rd_cv_pyramid *next,
             const rd_cv_point *points, rd_cv_point *flow, uint8_t *status,
             float *err, size_t count);
/* blockSize=3, Sobel aperture=3, BORDER_REFLECT_101, k=.04. harris=0
 * requests min eigenvalue. Models the default AVX/AVX2 reference dispatch. */
int rd_cv_corner_response(const uint8_t *src, int width, int height, int harris,
                          float *response, const rd_cv_trace *trace);
/* GFTT: quality=.001, minDistance=20, block=3, k=.04, no mask.
 * max_points=0 means unlimited. Caller owns *out (free), even count=0.
 * The returned list is GFTT order; call sort_keypoints for RD-VIO's second sort. */
int rd_cv_gftt(const uint8_t *src, int width, int height, int max_points,
              int harris, rd_cv_keypoint **out, size_t *count);
void rd_cv_sort_keypoints(rd_cv_keypoint *points, size_t count);
double rd_cv_norm(rd_cv_point p);
/* RD-VIO image-side acceptance checks after both LK passes, including the
 * float point differences, 20 px border, rows/4 displacement and .5 reverse. */
void rd_cv_track_status(const rd_cv_point *prev, const rd_cv_point *next,
                       const rd_cv_point *reverse, uint8_t *status,
                       const uint8_t *reverse_status, size_t count, int width, int height);
#endif
