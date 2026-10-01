/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef SV_FAST_H
#define SV_FAST_H

#include <stdint.h>
#include "sv_types.h"

/* Port of OpenCV 4.6.0's cv::FAST (modules/features2d/src/fast.cpp,
 * FAST_t<16> scalar path -- the non-SIMD path this port always uses is
 * mathematically identical to the SIMD path: FAST/cornerScore are pure
 * integer comparisons with no rounding, so scalar vs. SIMD never disagree)
 * plus cornerScore<16> (modules/features2d/src/fast_score.cpp) and
 * makeOffsets (same file). BSD-3 (Edward Rosten's original FAST) via
 * OpenCV's own redistribution; see LICENSE text kept in sv_fast.c.
 *
 * img: row-major uint8 grayscale buffer, `step` bytes/row (>= cols).
 * Only TYPE_9_16, nonmax_suppression=true is implemented -- the only mode
 * stella_vslam's orb_extractor.cc ever calls (cv::FAST(..., true)).
 *
 * out_keypts must hold at least (rows-6)*(cols-6) entries in the worst
 * case (caller-provided cap); returns the number of keypoints written
 * (0 if img is smaller than 7x7, matching cv::FAST's own early-out for
 * rows<7/cols<7 -- see the loop bound `i = 3; i < rows-2` in fast.cpp,
 * which never executes when rows<=5, and cols must be >6 for the inner
 * `j = 3; j < cols-3` loop to run at all).
 * KeyPoint fields set: x, y (int-valued as float), response (cornerScore);
 * octave/angle/size are left at 0, filled in by the caller (orb_extractor
 * sets pt +=cell offset, octave, size, then computes angle separately --
 * see orb_extractor.cc compute_fast_keypoints()). */
int sv_fast_detect(const uint8_t* img, int step, int cols, int rows,
                    int threshold, sv_keypoint* out_keypts, int cap);

#endif /* SV_FAST_H */
