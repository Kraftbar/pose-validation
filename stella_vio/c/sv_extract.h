/* SPDX-License-Identifier: BSD-2-Clause AND BSD-3-Clause */
#ifndef SV_EXTRACT_H
#define SV_EXTRACT_H

#include <stdint.h>
#include "sv_types.h"

/* Port of stella_vslam's feature::orb_extractor
 * (runs/stella_port/reference_build/src/src/stella_vslam/feature/
 * orb_extractor.{h,cc}, orb_impl.{h,cc}, orb_params.{h,cc},
 * orb_point_pairs.h), descriptor_type::ORB path only (the only path the
 * reference config's Feature block exercises -- no HASH_SIFT/LIFTFEAT).
 * BSD-2 (stella_vslam) for the extractor control flow; BSD-3 (OpenCV,
 * via stella's own orb_extractor.cc/orb_point_pairs.h notices) for the
 * FAST/orientation/descriptor primitives it calls -- see sv_fast.c,
 * sv_image.c, orb_point_pairs.h.
 *
 * Deliberately NOT ORB-SLAM2's extractor: no quadtree/octree node
 * distribution -- stella_vslam's distribute_keypoints() buckets FAST
 * candidates into a plain uniform grid (cell size ~min_area_sqrt_/
 * scale_factor) and keeps only the single highest-response keypoint per
 * occupied cell. Level ordering is coarse-to-fine loop 0..num_levels-1 at
 * full-image granularity per level (not per-node). Patch/descriptor radii
 * (orb_patch_radius_=19, fast_patch_size_=31) and point-pair table are
 * OpenCV's own, unchanged.
 */

typedef struct sv_orb_params {
    float scale_factor; /* 1.2 */
    int num_levels; /* 8 */
    int ini_fast_thr; /* 20 */
    int min_fast_thr; /* 7 */
    unsigned int min_area; /* Preprocessing.min_size, default 800 */
} sv_orb_params;

/* Extracts ORB keypoints+descriptors from a full-resolution CV_8UC1 image,
 * exactly reproducing orb_extractor::extract() -> extract_binary_descriptor()
 * for descriptor_type::ORB with no mask (mask_rects_ empty, in_image_mask
 * empty -- the only path the reference config exercises). Returns the
 * number of keypoints (== descriptor rows), or -1 if cap was exceeded
 * (caller should retry with a bigger cap; there is no silent truncation).
 *
 * keypts/descriptors (32 bytes/row, row-major) must be caller-allocated,
 * `cap` entries/rows. Keypoints are emitted in stella's own order: level
 * 0 first (all its distributed keypoints, cell/grid order), then level 1,
 * etc -- this must match 1:1, in order, against keypoints.tsv/
 * descriptors.tsv's kp_idx for a given frame_idx. */
int sv_orb_extract(const uint8_t* gray, int w, int h,
                    const sv_orb_params* params,
                    sv_keypoint* keypts, uint8_t* descriptors, int cap);

#endif /* SV_EXTRACT_H */
