/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_MATCH_AREA_H
#define SV_MATCH_AREA_H

#include "sv_types.h"
#include "sv_frame.h"

/* Port of stella_vslam's match::area::match_in_consistent_area()
 * (match/area.{h,cc}), used by module::initializer::try_initialize_for_monocular
 * with lowe_ratio=0.9, check_orientation=true, margin=100 -- BSD-2
 * (AIST 2019 + stella-cv 2022, see sv_rng.h for the full notice text).
 *
 * NOT an orientation-histogram matcher: check_orientation gates each
 * candidate directly on |angle_1 - angle_2| <= 30 deg (util::angle::diff,
 * wrapped to [-180,180)), no top-3-histogram-bin voting.
 *
 * match_in_consistent_area only considers frm_1 keypoints at octave 0
 * (undist_keypt.octave == 0), looks up frm_2 candidates via
 * sv_frame_get_keypoints_in_cell(margin, min_level=max_level=0) around
 * prev_matched_pts[idx_1], and picks the best Hamming match under a
 * ratio test (HAMMING_DIST_THR_LOW=50, second*lowe_ratio < best rejected),
 * with mutual-match bookkeeping: if a frm_2 index already matched to an
 * earlier frm_1 index gets stolen by a closer later frm_1 index, the
 * earlier match is invalidated and num_matches decremented.
 */

/* descriptors_1/2: row-major, 32 bytes/row (ORB). prev_matched: updated
 * in place exactly as stella's prev_matched_coords_ (x,y per frm_1 index).
 * matched_2_in_1: caller-allocated, num_kp1 entries; filled with the
 * matched frm_2 index or -1. Returns num_matches. */
/* stella_vio initializer-matcher knobs (set by sv_system from its params) */
extern unsigned int sv_match_area_hamm_thr;
extern float sv_match_area_ratio;
extern int sv_match_area_max_level;

unsigned int sv_match_in_consistent_area(
    const sv_keypoint* keypts_1, unsigned int num_kp1,
    const uint8_t* descriptors_1,
    const sv_keypoint* keypts_2, unsigned int num_kp2,
    const uint8_t* descriptors_2,
    const sv_frame_grid* grid_2,
    float* prev_matched_x, float* prev_matched_y, /* length num_kp1, in/out */
    int margin,
    int* matched_2_in_1 /* out, length num_kp1 */);

#endif /* SV_MATCH_AREA_H */
