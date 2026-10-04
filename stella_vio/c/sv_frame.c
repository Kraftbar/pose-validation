/* SPDX-License-Identifier: BSD-2-Clause */
/* SPDX license: BSD-2-Clause, portions (c) 2019 National Institute of
 * Advanced Industrial Science and Technology (AIST), (c) 2022 stella-cv --
 * see sv_image.c for the full stella_vslam license text this file's logic
 * (data::assign_keypoints_to_grid / get_cell_indices / get_keypoints_in_cell,
 * camera::perspective::compute_image_bounds) is ported from.
 */
#include "sv_frame.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

void sv_compute_image_bounds(const sv_camera_params* cam, int cols, int rows,
                              sv_image_bounds* out) {
    if (cam->k1 == 0.0 && cam->k2 == 0.0 && cam->p1 == 0.0 && cam->p2 == 0.0 && cam->k3 == 0.0) {
        out->min_x = 0.0f;
        out->max_x = (float)cols;
        out->min_y = 0.0f;
        out->max_y = (float)rows;
        return;
    }
    /* corners: left-top, right-top, left-bottom, right-bottom */
    float ux0, uy0, ux1, uy1, ux2, uy2, ux3, uy3;
    sv_undistort_point(cam, 0.0f, 0.0f, &ux0, &uy0);
    sv_undistort_point(cam, (float)cols, 0.0f, &ux1, &uy1);
    sv_undistort_point(cam, 0.0f, (float)rows, &ux2, &uy2);
    sv_undistort_point(cam, (float)cols, (float)rows, &ux3, &uy3);

    out->min_x = (ux0 < ux2) ? ux0 : ux2;
    out->max_x = (ux1 > ux3) ? ux1 : ux3;
    out->min_y = (uy0 < uy1) ? uy0 : uy1;
    out->max_y = (uy2 > uy3) ? uy2 : uy3;
}

/* cvFloor/cvCeil: OpenCV's fast integer floor/ceil for doubles. Bit-exact
 * with floor()/ceil() for every finite double in the pixel-coordinate
 * ranges this port ever sees (image width/height, small margins). */
static int sv_floor_i(double v) { return (int)floor(v); }
static int sv_ceil_i(double v) { return (int)ceil(v); }

static int get_cell_indices(const sv_keypoint* kp, const sv_image_bounds* bounds,
                             unsigned int num_grid_cols, unsigned int num_grid_rows,
                             double inv_cell_width, double inv_cell_height,
                             int* cell_idx_x, int* cell_idx_y) {
    /* stella: cvFloor((keypt.pt.x - img_bounds_.min_x_) * inv_cell_width) --
     * pt.x and min_x_ are both float, so the subtraction happens in float
     * precision *before* widening to double for the multiply by the double
     * inv_cell_width. Doing the subtraction in double instead can round
     * differently right at cell boundaries. */
    float dx = kp->x - bounds->min_x;
    float dy = kp->y - bounds->min_y;
    *cell_idx_x = sv_floor_i((double)dx * inv_cell_width);
    *cell_idx_y = sv_floor_i((double)dy * inv_cell_height);
    return (0 <= *cell_idx_x && *cell_idx_x < (int)num_grid_cols
            && 0 <= *cell_idx_y && *cell_idx_y < (int)num_grid_rows);
}

int sv_frame_build_grid(const sv_keypoint* undist_keypts, unsigned int num_keypts,
                         const sv_image_bounds* bounds,
                         unsigned int num_grid_cols, unsigned int num_grid_rows,
                         sv_frame_grid* grid_out) {
    memset(grid_out, 0, sizeof(*grid_out));
    grid_out->num_grid_cols = num_grid_cols;
    grid_out->num_grid_rows = num_grid_rows;
    grid_out->bounds = *bounds;

    unsigned int num_cells = num_grid_cols * num_grid_rows;
    grid_out->cells = (sv_grid_cell*)calloc(num_cells ? num_cells : 1, sizeof(sv_grid_cell));
    if (!grid_out->cells) return -1;

    /* stella: (double)num_grid_cols / (img_bounds_.max_x_ - img_bounds_.min_x_)
     * -- max_x_/min_x_ are float, so their subtraction happens in float
     * precision before the division promotes to double. */
    float span_x = bounds->max_x - bounds->min_x;
    float span_y = bounds->max_y - bounds->min_y;
    double inv_cell_width = (double)num_grid_cols / (double)span_x;
    double inv_cell_height = (double)num_grid_rows / (double)span_y;

    /* Pass 1: count per cell (so pass 2 can allocate exact-size arrays --
     * matches stella's own pre-reserve intent without depending on its
     * approximate reserve size, which does not affect final content). */
    unsigned int* counts = (unsigned int*)calloc(num_cells ? num_cells : 1, sizeof(unsigned int));
    if (!counts) { free(grid_out->cells); grid_out->cells = NULL; return -1; }

    unsigned int idx;
    for (idx = 0; idx < num_keypts; idx++) {
        int cx, cy;
        if (get_cell_indices(&undist_keypts[idx], bounds, num_grid_cols, num_grid_rows,
                              inv_cell_width, inv_cell_height, &cx, &cy)) {
            counts[(unsigned int)cx * num_grid_rows + (unsigned int)cy]++;
        }
    }

    unsigned int c;
    for (c = 0; c < num_cells; c++) {
        if (counts[c]) {
            grid_out->cells[c].indices = (unsigned int*)malloc(sizeof(unsigned int) * counts[c]);
            if (!grid_out->cells[c].indices) {
                free(counts);
                sv_frame_grid_free(grid_out);
                return -1;
            }
        }
        grid_out->cells[c].count = 0; /* used as a fill cursor below */
    }
    free(counts);

    /* Pass 2: fill in keypoint-index order (== push_back order stella uses). */
    for (idx = 0; idx < num_keypts; idx++) {
        int cx, cy;
        if (get_cell_indices(&undist_keypts[idx], bounds, num_grid_cols, num_grid_rows,
                              inv_cell_width, inv_cell_height, &cx, &cy)) {
            sv_grid_cell* cell = &grid_out->cells[(unsigned int)cx * num_grid_rows + (unsigned int)cy];
            cell->indices[cell->count++] = idx;
        }
    }

    return 0;
}

void sv_frame_grid_free(sv_frame_grid* grid) {
    if (!grid || !grid->cells) return;
    unsigned int num_cells = grid->num_grid_cols * grid->num_grid_rows;
    unsigned int c;
    for (c = 0; c < num_cells; c++) {
        free(grid->cells[c].indices);
    }
    free(grid->cells);
    grid->cells = NULL;
}

unsigned int sv_frame_get_keypoints_in_cell(const sv_frame_grid* grid,
                                             const sv_keypoint* undist_keypts,
                                             float ref_x, float ref_y, float margin,
                                             int min_level, int max_level,
                                             unsigned int* out_indices, unsigned int out_cap) {
    const sv_image_bounds* b = &grid->bounds;
    unsigned int num_grid_cols = grid->num_grid_cols;
    unsigned int num_grid_rows = grid->num_grid_rows;

    float span_x = b->max_x - b->min_x;
    float span_y = b->max_y - b->min_y;
    double inv_cell_width = (double)num_grid_cols / (double)span_x;
    double inv_cell_height = (double)num_grid_rows / (double)span_y;

    unsigned int n_out = 0;

    /* stella: cvFloor/cvCeil((ref_x - img_bounds_.min_x_ -/+ margin) *
     * inv_cell_width) -- ref_x/min_x_/margin are all float, so the
     * subtraction/addition happens in float precision before widening to
     * double for the multiply (same reasoning as get_cell_indices()
     * above). */
    float lo_x = ref_x - b->min_x - margin;
    float hi_x = ref_x - b->min_x + margin;
    float lo_y = ref_y - b->min_y - margin;
    float hi_y = ref_y - b->min_y + margin;

    int min_cell_idx_x = sv_floor_i((double)lo_x * inv_cell_width);
    if (min_cell_idx_x < 0) min_cell_idx_x = 0;
    if ((int)num_grid_cols <= min_cell_idx_x) return 0;

    int max_cell_idx_x = sv_ceil_i((double)hi_x * inv_cell_width);
    if (max_cell_idx_x > (int)num_grid_cols - 1) max_cell_idx_x = (int)num_grid_cols - 1;
    if (max_cell_idx_x < 0) return 0;

    int min_cell_idx_y = sv_floor_i((double)lo_y * inv_cell_height);
    if (min_cell_idx_y < 0) min_cell_idx_y = 0;
    if ((int)num_grid_rows <= min_cell_idx_y) return 0;

    int max_cell_idx_y = sv_ceil_i((double)hi_y * inv_cell_height);
    if (max_cell_idx_y > (int)num_grid_rows - 1) max_cell_idx_y = (int)num_grid_rows - 1;
    if (max_cell_idx_y < 0) return 0;

    int check_min_level = (min_level >= 0);
    int check_max_level = (max_level >= 0);

    int cx, cy;
    for (cx = min_cell_idx_x; cx <= max_cell_idx_x; cx++) {
        for (cy = min_cell_idx_y; cy <= max_cell_idx_y; cy++) {
            const sv_grid_cell* cell = &grid->cells[(unsigned int)cx * num_grid_rows + (unsigned int)cy];
            unsigned int k;
            for (k = 0; k < cell->count; k++) {
                unsigned int idx = cell->indices[k];
                const sv_keypoint* kp = &undist_keypts[idx];

                if (check_min_level && kp->octave < min_level) continue;
                if (check_max_level && max_level < kp->octave) continue;

                float dist_x = kp->x - ref_x;
                float dist_y = kp->y - ref_y;
                if (fabsf(dist_x) < margin && fabsf(dist_y) < margin) {
                    if (n_out < out_cap) {
                        out_indices[n_out] = idx;
                    }
                    n_out++;
                }
            }
        }
    }

    return n_out;
}
