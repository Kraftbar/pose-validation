/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_FRAME_H
#define SV_FRAME_H

#include "sv_types.h"
#include "sv_undistort.h"

/* Port of stella_vslam's frame-construction grid (data/common.h/.cc:
 * assign_keypoints_to_grid() / get_cell_indices() / get_keypoints_in_cell(),
 * runs/stella_port/reference_build/src/src/stella_vslam/data/common.cc) and
 * camera::perspective::compute_image_bounds()
 * (.../camera/perspective.cc) -- BSD-2 (AIST 2019 + stella-cv 2022 notices,
 * see sv_image.c for the full text this port keeps).
 *
 * Grid semantics (data::common.cc, unchanged here):
 *   - cell index = cvFloor((pt - img_bounds.min) * num_grid_{cols,rows} /
 *     (img_bounds.max - img_bounds.min)); a keypoint is assigned to exactly
 *     one cell (or none, if outside [0,num_grid_cols)x[0,num_grid_rows)).
 *   - get_keypoints_in_cell(ref_x, ref_y, margin, min_level, max_level) is
 *     stella's only area/cell query: it walks every grid cell overlapping
 *     the box [ref-margin, ref+margin]^2 (cell range from cvFloor/cvCeil of
 *     the box edges, clamped to the grid), and for each keypoint in those
 *     cells applies octave-range filtering (min_level/max_level, -1 = no
 *     filter, inclusive bounds) then an exact |dx|<margin && |dy|<margin
 *     box check (strict less-than, not <=) -- so it is a box query even
 *     though cells make it cheap, not a plain single-cell lookup. Result
 *     order: cell_idx_x outer loop, cell_idx_y inner, keypoints within a
 *     cell in assignment (== keypoint index) order -- reproduced exactly
 *     here since callers rely on it deterministically.
 *
 * img_bounds: camera::perspective::compute_image_bounds() -- if the camera
 * has zero distortion, bounds are just [0,cols]x[0,rows]; otherwise the
 * four image corners are run through undistort_keypoints() (sv_undistort.h,
 * already ported in module 1) and the bounds are the min/max of the
 * undistorted corners. The reference TUM config always has nonzero
 * distortion, so only that branch is exercised/checked, but both are
 * implemented for completeness.
 */

typedef struct sv_image_bounds {
    float min_x, max_x, min_y, max_y;
} sv_image_bounds;

/* cols/rows: camera pixel dimensions (640x480 for the reference config). */
void sv_compute_image_bounds(const sv_camera_params* cam, int cols, int rows,
                              sv_image_bounds* out);

typedef struct sv_grid_cell {
    unsigned int* indices; /* owned, sv_frame_grid_free() releases it */
    unsigned int count;
} sv_grid_cell;

typedef struct sv_frame_grid {
    unsigned int num_grid_cols;
    unsigned int num_grid_rows;
    sv_image_bounds bounds;
    /* cells[cx * num_grid_rows + cy], matching stella's
     * keypt_indices_in_cells_[cx][cy] (vector<vector<vector<uint>>>,
     * outer index = column). */
    sv_grid_cell* cells;
} sv_frame_grid;

/* Builds the grid over `undist_keypts` (already-undistorted keypoints, as
 * produced by module 1's sv_undistort_point + sv_orb_extract). Returns 0 on
 * success, -1 on allocation failure. Caller must sv_frame_grid_free(). */
int sv_frame_build_grid(const sv_keypoint* undist_keypts, unsigned int num_keypts,
                         const sv_image_bounds* bounds,
                         unsigned int num_grid_cols, unsigned int num_grid_rows,
                         sv_frame_grid* grid_out);

void sv_frame_grid_free(sv_frame_grid* grid);

/* Reproduces stella's frame::get_keypoints_in_cell() / data::get_keypoints_in_cell().
 * min_level/max_level: -1 disables that bound (matches stella's default).
 * Writes up to out_cap indices into out_indices, in stella's exact order;
 * returns the number of matches found (may exceed out_cap -- caller should
 * size out_cap >= num_keypts to never truncate, as check_sv_frame.c does). */
unsigned int sv_frame_get_keypoints_in_cell(const sv_frame_grid* grid,
                                             const sv_keypoint* undist_keypts,
                                             float ref_x, float ref_y, float margin,
                                             int min_level, int max_level,
                                             unsigned int* out_indices, unsigned int out_cap);

#endif /* SV_FRAME_H */
