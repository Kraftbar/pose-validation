/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause */
#ifndef SV_IMAGE_H
#define SV_IMAGE_H

#include <math.h>
#include <stdint.h>

/* Ports of the specific OpenCV 4.6.0 code paths stella_vslam's
 * orb_extractor.cc exercises on an 8-bit single-channel image:
 *   - cv::resize(..., INTER_LINEAR) fixed-point 8U path
 *     (modules/imgproc/src/resize.cpp: HResizeLinear/VResizeLinear +
 *     FixedPtCast<int,uchar,22>, non-vectorized inner loop -- integer-exact,
 *     scalar and SIMD agree).
 *   - cv::GaussianBlur(src, dst, Size(7,7), 2, 2, BORDER_REFLECT_101) 8U
 *     bit-exact fixed-point path (modules/imgproc/src/smooth.dispatch.cpp
 *     createGaussianKernels<ufixedpoint16> + GaussianBlurFixedPoint):
 *     kernel weights computed in double precision (matching
 *     getGaussianKernelBitExact's softdouble formula bit-for-bit for this
 *     n=7/sigma=2 case -- no reduced-precision shortcuts), quantized to
 *     Q8 integers via the same error-diffusion rounding as
 *     getGaussianKernelFixedPoint_ED (guarantees the 7 taps sum to exactly
 *     256), then a plain double-Q8 (Q16 total) 2D integer convolution with
 *     round-to-nearest.
 *   - cv::fastAtan2 (modules/core/src/mathfuncs_core.simd.hpp atan_f32,
 *     scalar path -- used by orb_impl::ic_angle, NOT stella's own
 *     util::cos/sin approximation, which is a separate, deliberately
 *     different table used only by compute_orb_descriptor's point
 *     rotation; see sv_extract.c).
 * BSD-3 for resize/blur; Apache-2.0 for the OpenCV 4.6 fastAtan2 source.
 * Modified to C99 scalar specializations. Full notices: ../NOTICE and ../LICENSES/.
 */

/* dst must be caller-allocated, dst_w*dst_h bytes, row-major, step==dst_w. */
void sv_resize_linear_u8(const uint8_t* src, int src_step, int src_w, int src_h,
                          uint8_t* dst, int dst_w, int dst_h);

/* dst must be caller-allocated, w*h bytes, row-major, step==w.
 * ksize is fixed at 7 (the only size stella_vslam ever requests). */
void sv_gaussian_blur7_u8(const uint8_t* src, int step, int w, int h, uint8_t* dst);

float sv_fast_atan2(float y, float x);

/* cvRound(float): SSE round-to-nearest-even (modules/core/include/opencv2/
 * core/fast_math.hpp). Exposed for sv_extract.c (ic_angle/compute_orb_descriptor
 * both call OpenCV's cvRound on float pixel-coordinate expressions). */
static inline int sv_cv_round_f(float v) {
#if defined(__FLT_EVAL_METHOD__) && __FLT_EVAL_METHOD__ == 0
    /* |v| < 2^22: adding 1.5 * 2^23 puts the sum in [2^23, 2^24) where the float spacing is 1, so the
     * (round-to-nearest-even) addition itself rounds v to an integer; the subtraction is exact. */
    if (v > -4194304.0f && v < 4194304.0f) {
        float t = v + 12582912.0f;
        return (int)(t - 12582912.0f);
    }
#endif
    return (int)nearbyintf(v);
}

#endif /* SV_IMAGE_H */
