/* SPDX-License-Identifier: MIT */
#ifndef SV_TYPES_H
#define SV_TYPES_H

#include <stdint.h>

/* Plain-C99 mirror of the fields stella_vslam's reference dumps actually
 * check (see runs/stella_port/reference_dumps/<seq>/keypoints.tsv):
 * x, y, octave, angle, response. Mirrors cv::KeyPoint's layout closely
 * enough for this port's purposes -- no class_id/pt as separate struct etc,
 * since orb_extractor never touches those fields. */
typedef struct sv_keypoint {
    float x;
    float y;
    float size;
    float angle;
    float response;
    int octave;
} sv_keypoint;

#endif /* SV_TYPES_H */
