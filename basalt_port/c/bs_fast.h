/* SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0 AND BSD-2-Clause
 * C99 reimplementation of OpenCV 4.6.0 cv::FAST(img, kps, threshold, nonmaxSuppression=true, TYPE_9_16) as called by Basalt's
 * detectKeypoints (src/utils/keypoints.cpp), plus the libstdc++-compatible std::sort tie order used on the response sort.
 * Derived from OpenCV modules/features2d/src/fast.cpp, fast_score.cpp, fast.avx2.cpp (Apache-2.0; FAST algorithm (c) 2006, 2008
 * Edward Rosten, BSD-2-Clause; notices in bs_fast.c and basalt_port/reference_cv/LICENSE-OpenCV-Apache-2.0).
 *
 * Dispatch on the reference machine: cv::FAST -> FAST_t<16>; no HAL / OpenCL / OpenVX replacement is active on x86-64 Linux,
 * the 16-pixel path runs the AVX2 block (32 lanes), then the SSE 128-bit block (16 lanes), then scalar columns. All three compute the
 * same integer predicate (a corner has >= 9 contiguous ring pixels brighter than v+t or darker than v-t) and the same NMS score
 * (max over the 16 nine-pixel arcs of the arc minimum / maximum of d = v - ring, minus 1), so one scalar formulation is exact; this is
 * verified at tolerance 0 against the real library by basalt_port/reference_tools/bs_image_test.cc (statement "fast").
 * Valid for 0 <= threshold <= 127 (the SIMD paths cast the threshold to char).
 */
#ifndef BS_FAST_H
#define BS_FAST_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct bs_fast_kp { float x, y, response; } bs_fast_kp;   /* cv::KeyPoint pt.x, pt.y, response (size 7, angle -1 dropped) */

/* cv::FAST(.., nonmax_suppression = true or false). img is 8-bit, row pitch `step` bytes. Output in cv order (row-major, rows 3..h-4,
 * ascending x). Returns the keypoint count; -1 if more than `cap` keypoints (out then holds the first cap), -2 bad argument, -3 no memory.
 * Without nonmax the response is cv's "(float)score" with score = 0 buffer content, i.e. cv returns response 0 (bit-exact:
 * curr[] is never written, so every keypoint carries (float)0). */
int bs_fast9_16(const uint8_t *img, int w, int h, int step, int threshold, int nonmax, bs_fast_kp *out, int cap);

/* std::sort(v.begin(), v.end(), [](a, b){ return a.response > b.response; }) with the libstdc++ introsort (threshold 16, depth limit
 * 2 floor(log2 n), heapsort fallback, final insertion sort): the tie order of unstable ranges is reproduced. */
void bs_fast_sort_response_desc(bs_fast_kp *v, size_t n);
void bs_fast_sort_depth_test(bs_fast_kp *v, size_t n, long depth);   /* test hook, see bs_image_test.cc */

#ifdef __cplusplus
}
#endif
#endif
