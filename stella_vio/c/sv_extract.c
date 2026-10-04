/* SV_PORT_SOURCES: sv_extract.c sv_fast.c sv_image.c
 * SPDX-License-Identifier: BSD-2-Clause AND BSD-3-Clause
 *
 * Port of stella_vslam's feature::orb_extractor::extract() ->
 * extract_binary_descriptor() path (descriptor_type::ORB, no mask), plus
 * orb_params scale-factor tables and orb_impl::ic_angle /
 * compute_orb_descriptor. See sv_extract.h for the file-level license/
 * provenance note (BSD-2 stella_vslam control flow, BSD-3 OpenCV
 * primitives).
 *
 * stella_vslam's own util::cos/sin (src/stella_vslam/util/trigonometric.h,
 * BSD-2) -- NOT OpenCV's cv::fastAtan2 machinery -- is what
 * compute_orb_descriptor uses to rotate the point-pair pattern; ic_angle
 * uses cv::fastAtan2 (sv_image.c) for the angle itself. Both are ported
 * here verbatim (integer/float ops only, no behavior change).
 */
#include "sv_extract.h"
#include "sv_fast.h"
#include "sv_image.h"
#include "sv_undistort.h"
#include "orb_point_pairs.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ---- stella_vslam/util/trigonometric.h (BSD-2, stella_vslam) ----
 * Used only by compute_orb_descriptor's point-pair rotation. Deliberately
 * different from cv::fastAtan2/cos: a piecewise polynomial cosine
 * approximation, ported bit-for-bit. */
#define SV_UTIL_PI 3.14159265358979f
#define SV_UTIL_PI_2 (SV_UTIL_PI / 2.0f)
#define SV_UTIL_TWO_PI (2.0f * SV_UTIL_PI)
#define SV_UTIL_INV_TWO_PI (1.0f / SV_UTIL_TWO_PI)
#define SV_UTIL_THREE_PI_2 (3.0f * SV_UTIL_PI_2)

static int sv_cv_floor_f_local(float v) {
    int i = (int)v;
    return i - (i > v);
}

static float sv_util__cos(float v) {
    const float c1 = 0.99940307f;
    const float c2 = -0.49558072f;
    const float c3 = 0.03679168f;
    float v2 = v * v;
    return c1 + v2 * (c2 + c3 * v2);
}

static float sv_util_cos(float v) {
    v = v - (float)sv_cv_floor_f_local(v * SV_UTIL_INV_TWO_PI) * SV_UTIL_TWO_PI;
    v = (0.0f < v) ? v : -v;
    if (v < SV_UTIL_PI_2) return sv_util__cos(v);
    else if (v < SV_UTIL_PI) return -sv_util__cos(SV_UTIL_PI - v);
    else if (v < SV_UTIL_THREE_PI_2) return -sv_util__cos(v - SV_UTIL_PI);
    else return sv_util__cos(SV_UTIL_TWO_PI - v);
}

static float sv_util_sin(float v) {
    return sv_util_cos(SV_UTIL_PI_2 - v);
}

/* ---- orb_impl (BSD-3 OpenCV via stella_vslam/feature/orb_impl.cc) ---- */

#define SV_FAST_PATCH_SIZE 31
#define SV_FAST_HALF_PATCH_SIZE 15 /* 31/2 */

static void sv_build_umax(int u_max[SV_FAST_HALF_PATCH_SIZE + 1]) {
    const int half = SV_FAST_HALF_PATCH_SIZE;
    int vmax = (int)floor((double)half * sqrt(2.0) / 2.0 + 1.0);
    int vmin = (int)ceil((double)half * sqrt(2.0) / 2.0);
    int v, v0;
    for (v = 0; v <= vmax; v++) {
        u_max[v] = (int)floor(sqrt((double)(half * half - v * v)) + 0.5);
    }
    for (v = half, v0 = 0; v >= vmin; v--) {
        while (u_max[v0] == u_max[v0 + 1]) v0++;
        u_max[v] = v0;
        v0++;
    }
}

static float sv_ic_angle(const uint8_t* image, int step, float px, float py,
                          const int u_max[SV_FAST_HALF_PATCH_SIZE + 1]) {
    int m_01 = 0, m_10 = 0;
    int cx = sv_cv_round_f(py) * step + sv_cv_round_f(px);
    const uint8_t* center = image + cx;
    int u, v;
    for (u = -SV_FAST_HALF_PATCH_SIZE; u <= SV_FAST_HALF_PATCH_SIZE; u++) {
        m_10 += u * center[u];
    }
    for (v = 1; v <= SV_FAST_HALF_PATCH_SIZE; v++) {
        int v_sum = 0;
        int d = u_max[v];
        for (u = -d; u <= d; u++) {
            int val_plus = center[u + v * step];
            int val_minus = center[u - v * step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }
    return sv_fast_atan2((float)m_01, (float)m_10);
}

#define SV_CV_M_PI 3.14159265358979311600 /* M_PI, double precision */

static void sv_compute_orb_descriptor(float px, float py, float angle_deg,
                                       const uint8_t* image, int step, uint8_t* desc) {
    /* orb_impl.cc: `const float angle = keypt.angle * M_PI / 180.0;` --
     * M_PI/180.0 are both double, so this whole expression is computed in
     * double precision (keypt.angle promoted) and only narrowed to float
     * at the very end -- NOT the same as computing it in float
     * throughout (found via a byte-level descriptor mismatch that traced
     * to a single flipped point-pair comparison at a cvRound tie). */
    const float angle = (float)((double)angle_deg * SV_CV_M_PI / 180.0);
    const float cos_angle = sv_util_cos(angle);
    const float sin_angle = sv_util_sin(angle);
    const uint8_t* center = image + sv_cv_round_f(py) * step + sv_cv_round_f(px);
    const int interval = 32;
    unsigned int i;

    for (i = 0; i < SV_ORB_POINT_PAIRS_SIZE / interval; i++) {
        int32_t val = 0;
        int b;
        for (b = 0; b < 8; b++) {
            unsigned int shift = i * interval + (unsigned int)b * 4;
            float x1 = sv_orb_point_pairs[shift], y1 = sv_orb_point_pairs[shift + 1];
            float x2 = sv_orb_point_pairs[shift + 2], y2 = sv_orb_point_pairs[shift + 3];
            int idx1 = sv_cv_round_f(x1 * sin_angle + y1 * cos_angle) * step
                       + sv_cv_round_f(x1 * cos_angle - y1 * sin_angle);
            int idx2 = sv_cv_round_f(x2 * sin_angle + y2 * cos_angle) * step
                       + sv_cv_round_f(x2 * cos_angle - y2 * sin_angle);
            int cmp = center[idx1] < center[idx2] ? 1 : 0;
            val |= cmp << b;
        }
        desc[i] = (uint8_t)val;
    }
}

/* ---- orb_params scale tables (feature/orb_params.cc) ---- */

static void sv_calc_scale_factors(int num_levels, float scale_factor, float* out) {
    int level;
    out[0] = 1.0f;
    for (level = 1; level < num_levels; level++) {
        out[level] = scale_factor * out[level - 1];
    }
}

/* ---- extractor ---- */

typedef struct sv_kp_arr {
    sv_keypoint* items;
    int count;
    int cap;
} sv_kp_arr;

static void sv_kp_push(sv_kp_arr* arr, sv_keypoint kp) {
    if (arr->count == arr->cap) { /* grow: candidate count scales with image area (8192 overflowed at 1280x720) */
        int ncap = arr->cap > 0 ? arr->cap * 2 : 8192;
        sv_keypoint* ni = (sv_keypoint*)realloc(arr->items, sizeof(sv_keypoint) * (size_t)ncap);
        if (!ni) {
            return; /* out of memory: drop the candidate (never writes past cap) */
        }
        arr->items = ni;
        arr->cap = ncap;
    }
    arr->items[arr->count++] = kp;
}

/* orb_extractor::distribute_keypoints -- plain uniform-grid, keep highest
 * response per cell (no quadtree). keypts_to_distribute coordinates are
 * relative to (min_x, min_y) (i.e. already cell-local, 0-based). */
static int sv_distribute_keypoints(const sv_keypoint* in, int n_in,
                                    int min_x, int max_x, int min_y, int max_y,
                                    float scale_factor, unsigned int min_area_sqrt,
                                    sv_keypoint* out, int out_cap) {
    double scaled_min_area_sqrt = (double)min_area_sqrt / (double)scale_factor;
    int num_x_grid = (int)ceil((double)(max_x - min_x) / scaled_min_area_sqrt);
    int num_y_grid = (int)ceil((double)(max_y - min_y) / scaled_min_area_sqrt);
    if (num_x_grid < 1) num_x_grid = 1;
    if (num_y_grid < 1) num_y_grid = 1;
    double delta_x = (double)(max_x - min_x) / num_x_grid;
    double delta_y = (double)(max_y - min_y) / num_y_grid;

    int n_cells = num_x_grid * num_y_grid;
    int* best_idx = (int*)malloc(sizeof(int) * (size_t)n_cells);
    float* best_resp = (float*)malloc(sizeof(float) * (size_t)n_cells);
    int c;
    for (c = 0; c < n_cells; c++) { best_idx[c] = -1; best_resp[c] = 0.f; }

    int i;
    for (i = 0; i < n_in; i++) {
        unsigned int ix = (unsigned int)(in[i].x / delta_x);
        unsigned int iy = (unsigned int)(in[i].y / delta_y);
        unsigned int idx = ix + iy * (unsigned int)num_x_grid;
        if ((int)idx >= n_cells) continue; /* defensive; upstream relies on grid covering range */
        if (best_idx[idx] == -1 || in[i].response > best_resp[idx]) {
            best_idx[idx] = i;
            best_resp[idx] = in[i].response;
        }
    }

    int n_out = 0;
    for (c = 0; c < n_cells; c++) {
        if (best_idx[c] >= 0) {
            if (n_out < out_cap) out[n_out] = in[best_idx[c]];
            n_out++;
        }
    }
    free(best_idx);
    free(best_resp);
    return n_out;
}

int sv_orb_extract(const uint8_t* gray, int w, int h,
                    const sv_orb_params* params,
                    sv_keypoint* keypts, uint8_t* descriptors, int cap) {
    const int num_levels = params->num_levels;
    const int orb_patch_radius = 19;
    const unsigned int min_area_sqrt = (unsigned int)sqrt((double)params->min_area);
    int level;

    float* scale_factors = (float*)malloc(sizeof(float) * (size_t)num_levels);
    sv_calc_scale_factors(num_levels, params->scale_factor, scale_factors);

    /* ---- pyramid ---- */
    uint8_t** pyramid = (uint8_t**)malloc(sizeof(uint8_t*) * (size_t)num_levels);
    int* pw = (int*)malloc(sizeof(int) * (size_t)num_levels);
    int* ph = (int*)malloc(sizeof(int) * (size_t)num_levels);
    pw[0] = w; ph[0] = h;
    pyramid[0] = (uint8_t*)malloc((size_t)w * (size_t)h);
    memcpy(pyramid[0], gray, (size_t)w * (size_t)h);
    for (level = 1; level < num_levels; level++) {
        double scale = scale_factors[level];
        int sw = (int)round((double)w * 1.0 / scale);
        int sh = (int)round((double)h * 1.0 / scale);
        pw[level] = sw; ph[level] = sh;
        pyramid[level] = (uint8_t*)malloc((size_t)sw * (size_t)sh);
        sv_resize_linear_u8(pyramid[level - 1], pw[level - 1], pw[level - 1], ph[level - 1],
                             pyramid[level], sw, sh);
    }

    int u_max[SV_FAST_HALF_PATCH_SIZE + 1];
    sv_build_umax(u_max);

    /* ---- per-level FAST + distribute + orientation ---- */
    sv_keypoint** level_kps = (sv_keypoint**)malloc(sizeof(sv_keypoint*) * (size_t)num_levels);
    int* level_n = (int*)malloc(sizeof(int) * (size_t)num_levels);

    const int cell_size = 64;
    const int overlap = 6;

    for (level = 0; level < num_levels; level++) {
        float scale_factor = scale_factors[level];
        int min_border_x = orb_patch_radius, min_border_y = orb_patch_radius;
        int max_border_x = pw[level] - orb_patch_radius;
        int max_border_y = ph[level] - orb_patch_radius;
        int width = max_border_x - min_border_x;
        int height = max_border_y - min_border_y;

        if (width <= 0 || height <= 0) {
            level_kps[level] = NULL;
            level_n[level] = 0;
            continue;
        }

        int num_cols = width / cell_size + 1;
        int num_rows = height / cell_size + 1;

        sv_kp_arr to_distribute;
        to_distribute.cap = 8192;
        to_distribute.count = 0;
        to_distribute.items = (sv_keypoint*)malloc(sizeof(sv_keypoint) * (size_t)to_distribute.cap);

        int i, j;
        for (i = 0; i < num_rows; i++) {
            int min_y = min_border_y + i * cell_size;
            if (max_border_y - overlap <= min_y) continue;
            int max_y = min_y + cell_size + overlap;
            if (max_border_y < max_y) max_y = max_border_y;

            for (j = 0; j < num_cols; j++) {
                int min_x = min_border_x + j * cell_size;
                if (max_border_x - overlap <= min_x) continue;
                int max_x = min_x + cell_size + overlap;
                if (max_border_x < max_x) max_x = max_border_x;

                const uint8_t* cell_ptr = pyramid[level] + (size_t)min_y * pw[level] + min_x;
                int cell_w = max_x - min_x, cell_h = max_y - min_y;

                sv_keypoint cell_kps[2048];
                int n_cell = sv_fast_detect(cell_ptr, pw[level], cell_w, cell_h,
                                             params->ini_fast_thr, cell_kps, 2048);
                if (n_cell == 0) {
                    n_cell = sv_fast_detect(cell_ptr, pw[level], cell_w, cell_h,
                                             params->min_fast_thr, cell_kps, 2048);
                }
                if (n_cell <= 0) continue;
                if (n_cell > 2048) n_cell = 2048; /* extremely unlikely for a 70x70ish cell */

                int k;
                for (k = 0; k < n_cell; k++) {
                    sv_keypoint kp = cell_kps[k];
                    kp.x += (float)(j * cell_size);
                    kp.y += (float)(i * cell_size);
                    sv_kp_push(&to_distribute, kp);
                }
            }
        }

        sv_keypoint* distributed = (sv_keypoint*)malloc(sizeof(sv_keypoint) * (size_t)(to_distribute.count > 0 ? to_distribute.count : 1));
        int n_dist = sv_distribute_keypoints(to_distribute.items, to_distribute.count,
                                              min_border_x, max_border_x, min_border_y, max_border_y,
                                              scale_factor, min_area_sqrt,
                                              distributed, to_distribute.count > 0 ? to_distribute.count : 1);
        free(to_distribute.items);

        unsigned int scaled_patch_size = (unsigned int)(SV_FAST_PATCH_SIZE * scale_factor);
        int k;
        for (k = 0; k < n_dist; k++) {
            distributed[k].x += (float)min_border_x;
            distributed[k].y += (float)min_border_y;
            distributed[k].octave = level;
            distributed[k].size = (float)scaled_patch_size;
        }
        for (k = 0; k < n_dist; k++) {
            distributed[k].angle = sv_ic_angle(pyramid[level], pw[level],
                                                distributed[k].x, distributed[k].y, u_max);
        }

        level_kps[level] = distributed;
        level_n[level] = n_dist;
    }

    /* ---- descriptors (per level: blur, then descriptor per keypoint,
     * then correct_keypoint_scale) ---- */
    int total = 0;
    for (level = 0; level < num_levels; level++) total += level_n[level];

    int overflow = (total > cap);
    int out_idx = 0;

    for (level = 0; level < num_levels; level++) {
        int n = level_n[level];
        if (n == 0) continue;

        uint8_t* blurred = (uint8_t*)malloc((size_t)pw[level] * (size_t)ph[level]);
        sv_gaussian_blur7_u8(pyramid[level], pw[level], pw[level], ph[level], blurred);

        int k;
        for (k = 0; k < n; k++) {
            sv_keypoint kp = level_kps[level][k];
            if (!overflow) {
                sv_compute_orb_descriptor(kp.x, kp.y, kp.angle, blurred, pw[level],
                                           descriptors + (size_t)out_idx * 32);
            }
            /* correct_keypoint_scale: pt *= scale_at_level (skipped for level 0) */
            if (level != 0) {
                kp.x *= scale_factors[level];
                kp.y *= scale_factors[level];
            }
            if (!overflow && out_idx < cap) {
                keypts[out_idx] = kp;
            }
            out_idx++;
        }
        free(blurred);
    }

    for (level = 0; level < num_levels; level++) {
        free(pyramid[level]);
        if (level_kps[level]) free(level_kps[level]);
    }
    free(pyramid); free(pw); free(ph);
    free(level_kps); free(level_n);
    free(scale_factors);

    return overflow ? -1 : total;
}
