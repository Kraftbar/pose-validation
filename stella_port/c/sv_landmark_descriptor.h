/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, AIST; Copyright (c) 2022, stella-cv.
 * Full notice in sv_landmark_descriptor.c. */
#ifndef SV_LANDMARK_DESCRIPTOR_H
#define SV_LANDMARK_DESCRIPTOR_H
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t keyframe_id;
    const uint8_t *descriptor; /* Borrowed 32-byte ORB row; required if live. */
    int erased;               /* Keyframe's will_be_erased, not landmark's. */
} sv_descriptor_observation;

typedef struct {
    size_t observation_index; /* Index into the original caller array. */
    uint32_t keyframe_id;
    unsigned median_distance;
    uint8_t descriptor[32];   /* Owned copy of selected row. */
} sv_landmark_descriptor_result;

/* Selection part of stella_vslam landmark::compute_descriptor, ORB only.
 * Unique keyframe IDs may arrive in any order; ties follow increasing ID,
 * matching the pinned upstream observation map. Erased keyframes are skipped.
 * Each row includes self-distance zero; median is the lower middle element
 * at (live_count-1)/2. First minimum wins, even for identical descriptors.
 * Optional medians[count] receives each input row's median, or UINT16_MAX
 * for erased observations. Caller serializes access to borrowed memory.
 * Outputs must not overlap each other or inputs. No map state is mutated.
 * Return 0 on success, -1 on invalid input/allocation failure/no live rows;
 * outputs remain unchanged on error. Upstream's all-erased case throws.
 */
int sv_landmark_select_descriptor(const sv_descriptor_observation *observations,
                                  size_t count, sv_landmark_descriptor_result *out,
                                  uint16_t *medians);
#endif
