/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt module M4 (C99): everything from the image file to the keypoints the frame-to-frame optical flow starts from.
 * Basalt (c) 2019 Vladyslav Usenko, Nikolaus Demmel, BSD-3-Clause (basalt + basalt-headers); OpenCV pieces in bs_fast.c (Apache-2.0).
 *
 * Executed path (euroc_config.json, frame_to_frame flow, grid 50, levels 3), verified by reading + the oracle bs_image_test.cc:
 *  1. dataset_io_euroc.h get_image_data: cv::imread(path, IMREAD_UNCHANGED); an 8-bit gray PNG (CV_8UC1) becomes
 *     ManagedImage<uint16_t> with v = pixel << 8. (CV_8UC3 would take channel 0, CV_16UC1 would be copied: not supported here.)
 *  2. FrameToFrameOpticalFlow::processFrame: ManagedImagePyr<uint16_t>::setFromImage(img, optical_flow_levels): a mipmap image of
 *     (w + w/2) x h, zero filled, level 0 copied to (0,0), then `levels` x subsample (5-tap binomial 1 4 6 4 1, separable,
 *     reflect-101, integer, +128 >> 8), i.e. levels + 1 stored levels.
 *  3. addPoints(): detectKeypoints(pyramid[cam0].lvl(0), kd, grid_size, 1, tracked cam0 translations): per grid cell without a
 *     tracked point, cv::FAST (thresholds 40, 20, 10, 5; nonmax on) on the 8-bit (>> 8) cell patch, std::sort by response descending,
 *     the first keypoint inside the image border (19 px) is kept. No other image filtering exists on the path.
 */
#ifndef BS_IMAGE_H
#define BS_IMAGE_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

enum { BS_IMG_OK = 0, BS_IMG_IO = 1, BS_IMG_UNSUPPORTED = 2, BS_IMG_DECODE = 3, BS_IMG_NOMEM = 4 };

/* EuRoC image reader: PNG file -> w*h uint16 (pixel << 8), malloc'ed (free() by the caller). Only 8-bit gray, non-interlaced-agnostic PNG
 * without gAMA / sRGB / iCCP / sBIT / tRNS chunks is accepted (the case where imread(UNCHANGED) == the decoded gray bytes); anything
 * else returns BS_IMG_UNSUPPORTED. Decoder: okvis_port/c/ok_png.c. */
int bs_image_load_euroc(const char *path, uint16_t **out, int *w, int *h);
/* same for an in-memory file */
int bs_image_decode_euroc(const unsigned char *buf, size_t n, uint16_t **out, int *w, int *h);

/* ManagedImagePyr<uint16_t> */
typedef struct bs_pyr {
    int orig_w, h;       /* level 0 size */
    int levels;          /* number of subsample() calls; levels + 1 levels are stored */
    int pitch;           /* image width = orig_w + orig_w / 2 (elements) */
    uint16_t *data;      /* pitch * h, zero filled outside the levels */
} bs_pyr;

int  bs_pyr_set(bs_pyr *p, const uint16_t *img, int w, int h, int levels);   /* setFromImage; returns BS_IMG_OK / NOMEM */
void bs_pyr_free(bs_pyr *p);
/* lvl(l): pointer to the first element, row pitch is p->pitch, size (orig_w >> l) x (h >> l) */
const uint16_t *bs_pyr_lvl(const bs_pyr *p, int l, int *w, int *h);
/* Image::subsample: out is (w/2... any) img_sub_w x img_sub_h, in `src` (pitch sp) of w x h, out pitch dp (elements) */
void bs_subsample(const uint16_t *src, int sp, int w, int h, uint16_t *dst, int dp, int sub_w, int sub_h);

/* detectKeypoints(img_raw, kd, grid, num_points_cell, current_points): img is the level-0 view (pitch in elements).
 * cur = n_cur pairs of doubles (x, y) = the tracked cam0 translations (float widened to double). Corners are written as (x, y) double
 * pairs (float values widened, as Eigen::Vector2d(float, float)), in basalt order. Returns the number of corners, or -1 if more than
 * max_out, -2 if the image is smaller than the grid size, -3 no memory. */
int bs_detect_keypoints(const uint16_t *img, int pitch, int w, int h, int grid, int num_points_cell,
                        const double *cur, int n_cur, double *out, int max_out);

#ifdef __cplusplus
}
#endif
#endif
