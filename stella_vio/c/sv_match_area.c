/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_match_area.h for provenance/license (BSD-2, AIST 2019 + stella-cv 2022). */
#include "sv_match_area.h"
#include <math.h>
#include <stdlib.h>

#define SV_MAX_HAMMING_DIST 256u
#define SV_HAMMING_DIST_THR_LOW 50u

/* stella_vio: initializer matcher knobs (defaults = exact port) */
unsigned int sv_match_area_hamm_thr = SV_HAMMING_DIST_THR_LOW;
float sv_match_area_ratio = 0.9f;
int sv_match_area_max_level = 0; /* ref keypoints up to this octave are matched (candidates in the same octave) */

static unsigned int sv_popcount8(uint8_t x) {
    unsigned int c = 0;
    while (x) {
        x &= (uint8_t)(x - 1);
        ++c;
    }
    return c;
}

static unsigned int sv_hamming32(const uint8_t* a, const uint8_t* b) {
    unsigned int d = 0;
    int i;
    for (i = 0; i < 32; ++i) {
        d += sv_popcount8((uint8_t)(a[i] ^ b[i]));
    }
    return d;
}

/* util::angle::diff (BSD-2). */
static float sv_angle_diff(float a1, float a2) {
    float ret = a1 - a2;
    if (ret <= -180.0f) {
        ret += 360.0f;
    }
    if (ret > 180.0f) {
        ret -= 360.0f;
    }
    return ret;
}

unsigned int sv_match_in_consistent_area(
    const sv_keypoint* keypts_1, unsigned int num_kp1,
    const uint8_t* descriptors_1,
    const sv_keypoint* keypts_2, unsigned int num_kp2,
    const uint8_t* descriptors_2,
    const sv_frame_grid* grid_2,
    float* prev_matched_x, float* prev_matched_y,
    int margin,
    int* matched_2_in_1) {
    unsigned int num_matches = 0;
    unsigned int idx_1;

    unsigned int* matched_dists_in_2 = (unsigned int*)malloc(sizeof(unsigned int) * (num_kp2 ? num_kp2 : 1));
    int* matched_1_in_2 = (int*)malloc(sizeof(int) * (num_kp2 ? num_kp2 : 1));
    unsigned int* cell_buf = (unsigned int*)malloc(sizeof(unsigned int) * (num_kp2 ? num_kp2 : 1));

    for (idx_1 = 0; idx_1 < num_kp1; ++idx_1) {
        matched_2_in_1[idx_1] = -1;
    }
    {
        unsigned int j;
        for (j = 0; j < num_kp2; ++j) {
            matched_dists_in_2[j] = SV_MAX_HAMMING_DIST;
            matched_1_in_2[j] = -1;
        }
    }

    for (idx_1 = 0; idx_1 < num_kp1; ++idx_1) {
        const sv_keypoint* kp1 = &keypts_1[idx_1];
        int scale_level_1 = kp1->octave;
        unsigned int n_cand, k;
        unsigned int best_hamm = SV_MAX_HAMMING_DIST, second_hamm = SV_MAX_HAMMING_DIST;
        int best_idx_2 = -1;
        const uint8_t* desc_1;

        if (scale_level_1 > sv_match_area_max_level) {
            continue;
        }

        n_cand = sv_frame_get_keypoints_in_cell(grid_2, keypts_2,
                                                 prev_matched_x[idx_1], prev_matched_y[idx_1],
                                                 (float)margin, scale_level_1, scale_level_1,
                                                 cell_buf, num_kp2);
        if (n_cand == 0) {
            continue;
        }
        if (n_cand > num_kp2) {
            n_cand = num_kp2; /* out_cap sized to num_kp2, never truncated in practice */
        }

        desc_1 = descriptors_1 + (size_t)idx_1 * 32;

        for (k = 0; k < n_cand; ++k) {
            unsigned int idx_2 = cell_buf[k];
            unsigned int hd;
            const uint8_t* desc_2;

            if (fabsf(sv_angle_diff(kp1->angle, keypts_2[idx_2].angle)) > 30.0f) {
                continue;
            }

            desc_2 = descriptors_2 + (size_t)idx_2 * 32;
            hd = sv_hamming32(desc_1, desc_2);

            if (matched_dists_in_2[idx_2] <= hd) {
                continue;
            }

            if (hd < best_hamm) {
                second_hamm = best_hamm;
                best_hamm = hd;
                best_idx_2 = (int)idx_2;
            }
            else if (hd < second_hamm) {
                second_hamm = hd;
            }
        }

        if (best_hamm > sv_match_area_hamm_thr) {
            continue;
        }
        /* ratio test: second_best * lowe_ratio(0.9) < best -> reject */
        if ((float)second_hamm * sv_match_area_ratio < (float)best_hamm) {
            continue;
        }

        {
            int prev_idx_1 = matched_1_in_2[best_idx_2];
            if (prev_idx_1 >= 0) {
                matched_2_in_1[prev_idx_1] = -1;
                --num_matches;
            }
            matched_2_in_1[idx_1] = best_idx_2;
            matched_1_in_2[best_idx_2] = (int)idx_1;
            matched_dists_in_2[best_idx_2] = best_hamm;
            ++num_matches;
        }
    }

    for (idx_1 = 0; idx_1 < num_kp1; ++idx_1) {
        if (matched_2_in_1[idx_1] >= 0) {
            prev_matched_x[idx_1] = keypts_2[matched_2_in_1[idx_1]].x;
            prev_matched_y[idx_1] = keypts_2[matched_2_in_1[idx_1]].y;
        }
    }

    free(matched_dists_in_2);
    free(matched_1_in_2);
    free(cell_buf);

    return num_matches;
}
