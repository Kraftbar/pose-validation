/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, AIST; Copyright (c) 2022, stella-cv.
 * See sv_match_bow.c for the full notice. */
#ifndef SV_MATCH_BOW_H
#define SV_MATCH_BOW_H
#include "sv_bow.h"
#include "sv_types.h"

/* Borrowed ORB observations, with one optional landmark token per keypoint.
 * Token 0 means absent; nonzero tokens identify caller-owned landmarks.
 * erased[i] describes that landmark's will_be_erased flag (NULL = all live).
 * BoW nodes must be strictly sorted; indices preserve upstream vector order,
 * and each keypoint may occur at most once across the feature vector.
 * Descriptor rows are exactly 32 bytes. Only keypoint.angle is accessed.
 * No camera, geometry, map ownership or synchronization is provided here. */
typedef struct {
    uint32_t count;
    const sv_keypoint *keypoints;
    const uint8_t *descriptors;
    const sv_bow_feat_vector *features;
    const uint64_t *landmarks;
    const uint8_t *erased;
} sv_match_bow_view;

/* Result is in frame-keypoint order, containing keyframe landmark tokens. */
int sv_match_bow_frame(const sv_match_bow_view *keyframe,
                       const sv_match_bow_view *frame, float ratio,
                       int check_orientation, uint64_t *matched, uint32_t *count);
/* Result is in first-keyframe order, containing second-keyframe tokens. */
int sv_match_bow_keyframes(const sv_match_bow_view *first,
                           const sv_match_bow_view *second, float ratio,
                           int check_orientation, uint64_t *matched, uint32_t *count);
/* Both calls return 0 on success, -1 for invalid input/allocation failure.
 * matched has count entries of the output view; NULL allowed for empty views.
 * Outputs must not alias inputs. On error outputs remain unchanged.
 * Ratio must be finite and >=0. Float ratio and inclusive threshold tests
 * match upstream. Only second-frame KEYPOINTS are unique, not landmark IDs.
 * Neither method implements bow_tree::match_for_triangulation. */
#endif
