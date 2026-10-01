/* SV_PORT_SOURCES: check_sv_frame.c sv_frame.c sv_undistort.c
 * SPDX-License-Identifier: MIT
 *
 * Harness: for each frame of a TUM sequence, loads that frame's
 * already-undistorted keypoints from
 * runs/stella_port/reference_dumps/<seq>/keypoints.tsv (module 1's own
 * validated dump -- same file check_sv_extract.c checks against), builds
 * the grid with sv_frame_build_grid() (camera intrinsics/distortion +
 * 640x480 hardcoded below, same reference config as check_sv_extract.c),
 * and compares:
 *   - every non-empty grid cell's keypoint-index list, in the exact
 *     (cell_x outer, cell_y inner) order the reference tool
 *     (stella_port/reference_tools/dump_frame_bow.cc) wrote them, against
 *     grid.tsv/grid_meta.tsv;
 *   - the fixed area-query set (every 25th keypoint index x margins
 *     {5,15,50} x level ranges {(-1,-1),(0,0),(0,3),(2,7)}, same nested
 *     order the reference tool used) against area_query.tsv.
 *
 * The module-2 reference dumps live in a sibling directory to the
 * module-1 dumps this harness is invoked with
 * (runs/stella_port/reference_dumps/<seq> -> runs/stella_port/
 * reference_frame_bow/<seq>, both produced under runs/stella_port/ by
 * their respective tools) -- tools/check_stella_port.py (shared, unmodified
 * runner across all check_*.c harnesses) only knows about the former, so
 * this harness derives the latter by substring substitution on the
 * dump_dir argument rather than needing a 4th CLI argument.
 *
 * Usage: check_sv_frame <seq_label> <fixtures_dir> <dump_dir> [max_frames]
 * (fixtures_dir is accepted for CLI-shape compatibility with
 * check_stella_port.py but unused -- this harness needs no images, only
 * module 1's already-validated keypoints.)
 * Prints "<seq_label>: <mismatches>/<total>\n"; exits 0 iff mismatches==0.
 */
#include "sv_frame.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LINE 65536

/* Reference config: stella_port/reference/configs/TUM_RGBD_mono_1_deterministic.yaml */
static const sv_camera_params CAM = {
    517.306408, 516.469215, 318.643040, 255.313989,
    0.262383, -0.953104, -0.005358, 0.002628, 1.163314
};
static const int CAM_COLS = 640, CAM_ROWS = 480;

static const int MARGINS[] = {5, 15, 50};
static const int LEVEL_RANGES[][2] = {{-1, -1}, {0, 0}, {0, 3}, {2, 7}};
#define N_MARGINS 3
#define N_LEVELS 4

static char* derive_frame_bow_dir(const char* dump_dir) {
    const char* needle = "reference_dumps";
    const char* p = strstr(dump_dir, needle);
    if (!p) return NULL;
    size_t prefix_len = (size_t)(p - dump_dir);
    size_t needle_len = strlen(needle);
    const char* suffix = p + needle_len;
    const char* repl = "reference_frame_bow";
    size_t out_len = prefix_len + strlen(repl) + strlen(suffix);
    char* out = (char*)malloc(out_len + 1);
    memcpy(out, dump_dir, prefix_len);
    memcpy(out + prefix_len, repl, strlen(repl));
    memcpy(out + prefix_len + strlen(repl), suffix, strlen(suffix) + 1);
    return out;
}

/* --- keypoints.tsv (module 1): x_hex,y_hex,octave per (frame_idx,kp_idx) */
typedef struct {
    int frame_idx, kp_idx;
    float x, y;
    int octave;
} kp_row;

static int read_kp_row(FILE* f, kp_row* out) {
    char line[1024];
    if (!fgets(line, sizeof(line), f)) return 0;
    char x_hex[64], y_hex[64], angle_hex[64], response_hex[64];
    double dummy;
    int n = sscanf(line, "%d\t%d\t%lf\t%63s\t%lf\t%63s\t%d\t%lf\t%63s\t%lf\t%63s",
                   &out->frame_idx, &out->kp_idx, &dummy, x_hex, &dummy, y_hex,
                   &out->octave, &dummy, angle_hex, &dummy, response_hex);
    if (n != 11) return 0;
    out->x = strtof(x_hex, NULL);
    out->y = strtof(y_hex, NULL);
    return 1;
}

/* --- grid.tsv: frame_idx cx cy kp_indices(csv) ------------------------ */
typedef struct {
    int frame_idx, cx, cy;
    char csv[MAX_LINE];
} grid_row;

static int read_grid_row(FILE* f, grid_row* out) {
    char line[MAX_LINE];
    if (!fgets(line, sizeof(line), f)) return 0;
    int n = sscanf(line, "%d\t%d\t%d\t%65535[^\n]", &out->frame_idx, &out->cx, &out->cy, out->csv);
    if (n == 3) out->csv[0] = '\0'; /* empty csv field (should not happen for grid.tsv, non-empty cells only) */
    else if (n != 4) return 0;
    return 1;
}

/* --- area_query.tsv: frame_idx q margin min_level max_level result(csv) */
typedef struct {
    int frame_idx, q, margin, min_level, max_level;
    char csv[MAX_LINE];
} query_row;

static int read_query_row(FILE* f, query_row* out) {
    char line[MAX_LINE];
    if (!fgets(line, sizeof(line), f)) return 0;
    int n = sscanf(line, "%d\t%d\t%d\t%d\t%d\t%65535[^\n]", &out->frame_idx, &out->q, &out->margin,
                   &out->min_level, &out->max_level, out->csv);
    if (n == 5) out->csv[0] = '\0';
    else if (n != 6) return 0;
    return 1;
}

/* Compares a computed unsigned-int list against a reference CSV string.
 * Returns 1 on exact (order-sensitive) match. */
static int csv_matches(const unsigned int* computed, unsigned int n_computed, const char* csv) {
    const char* p = csv;
    unsigned int i;
    for (i = 0; i < n_computed; i++) {
        if (*p == '\0') return 0;
        char* end;
        unsigned long v = strtoul(p, &end, 10);
        if (end == p) return 0;
        if (v != computed[i]) return 0;
        p = end;
        if (*p == ',') p++;
        else if (*p != '\0') return 0;
    }
    return (*p == '\0');
}

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: check_sv_frame <seq_label> <fixtures_dir> <dump_dir> [max_frames]\n");
        return 2;
    }
    const char* seq_label = argv[1];
    const char* dump_dir = argv[3];
    long max_frames = argc > 4 ? atol(argv[4]) : -1;

    char* frame_bow_dir = derive_frame_bow_dir(dump_dir);
    if (!frame_bow_dir) {
        fprintf(stderr, "check_sv_frame: could not derive frame_bow dir from %s\n", dump_dir);
        return 2;
    }

    char kp_path[4096], meta_path[4096], grid_path[4096], query_path[4096];
    snprintf(kp_path, sizeof(kp_path), "%s/keypoints.tsv", dump_dir);
    snprintf(meta_path, sizeof(meta_path), "%s/grid_meta.tsv", frame_bow_dir);
    snprintf(grid_path, sizeof(grid_path), "%s/grid.tsv", frame_bow_dir);
    snprintf(query_path, sizeof(query_path), "%s/area_query.tsv", frame_bow_dir);
    free(frame_bow_dir);

    FILE* kf = fopen(kp_path, "r");
    FILE* mf = fopen(meta_path, "r");
    FILE* gf = fopen(grid_path, "r");
    FILE* qf = fopen(query_path, "r");
    if (!kf || !mf || !gf || !qf) {
        fprintf(stderr, "check_sv_frame: cannot open one of %s / %s / %s / %s\n",
                kp_path, meta_path, grid_path, query_path);
        return 2;
    }
    char header[MAX_LINE];
    fgets(header, sizeof(header), kf);
    fgets(header, sizeof(header), mf);
    fgets(header, sizeof(header), gf);
    fgets(header, sizeof(header), qf);

    long long total = 0, mismatches = 0;

    kp_row kp_pending; int kp_have = 0, kp_eof = 0;
    grid_row grid_pending; int grid_have = 0, grid_eof = 0;
    query_row query_pending; int query_have = 0, query_eof = 0;

    kp_row* kpts_buf = (kp_row*)malloc(sizeof(kp_row) * 200000);
    unsigned int* out_buf = (unsigned int*)malloc(sizeof(unsigned int) * 200000);

    long frame_idx;
    for (frame_idx = 0; ; frame_idx++) {
        if (max_frames >= 0 && frame_idx >= max_frames) break;

        /* gather this frame's keypoints */
        int n_kpts = 0;
        for (;;) {
            if (!kp_have && !kp_eof) {
                kp_have = read_kp_row(kf, &kp_pending);
                if (!kp_have) kp_eof = 1;
            }
            if (!kp_have || kp_pending.frame_idx != (int)frame_idx) break;
            kpts_buf[n_kpts++] = kp_pending;
            kp_have = 0;
        }
        if (n_kpts == 0 && kp_eof) break; /* end of sequence */

        /* grid_meta: one row per frame */
        int meta_frame, num_cols, num_rows;
        if (fscanf(mf, "%d\t%d\t%d\n", &meta_frame, &num_cols, &num_rows) != 3) break;
        if (meta_frame != (int)frame_idx) {
            fprintf(stderr, "check_sv_frame: grid_meta.tsv desync at frame %ld\n", frame_idx);
            break;
        }

        sv_keypoint* kpts = (sv_keypoint*)malloc(sizeof(sv_keypoint) * (size_t)n_kpts);
        int i;
        for (i = 0; i < n_kpts; i++) {
            memset(&kpts[i], 0, sizeof(kpts[i]));
            kpts[i].x = kpts_buf[i].x;
            kpts[i].y = kpts_buf[i].y;
            kpts[i].octave = kpts_buf[i].octave;
        }

        sv_image_bounds bounds;
        sv_compute_image_bounds(&CAM, CAM_COLS, CAM_ROWS, &bounds);

        sv_frame_grid grid;
        sv_frame_build_grid(kpts, (unsigned int)n_kpts, &bounds,
                             (unsigned int)num_cols, (unsigned int)num_rows, &grid);

        /* --- grid comparison: (cx outer, cy inner) over non-empty cells --- */
        unsigned int cx, cy;
        for (cx = 0; cx < grid.num_grid_cols; cx++) {
            for (cy = 0; cy < grid.num_grid_rows; cy++) {
                const sv_grid_cell* cell = &grid.cells[cx * grid.num_grid_rows + cy];
                if (cell->count == 0) continue;

                if (!grid_have && !grid_eof) {
                    grid_have = read_grid_row(gf, &grid_pending);
                    if (!grid_have) grid_eof = 1;
                }
                total++;
                if (!grid_have || grid_pending.frame_idx != (int)frame_idx ||
                    grid_pending.cx != (int)cx || grid_pending.cy != (int)cy ||
                    !csv_matches(cell->indices, cell->count, grid_pending.csv)) {
                    mismatches++;
                }
                /* Both sides walk (cx,cy) in the same deterministic order,
                 * so always resync to the next reference row regardless of
                 * match/mismatch. */
                grid_have = 0;
            }
        }

        /* --- area query comparison --- */
        size_t q;
        for (q = 0; q < (size_t)n_kpts; q += 25) {
            float ref_x = kpts[q].x, ref_y = kpts[q].y;
            int m;
            for (m = 0; m < N_MARGINS; m++) {
                int l;
                for (l = 0; l < N_LEVELS; l++) {
                    unsigned int n_res = sv_frame_get_keypoints_in_cell(
                        &grid, kpts, ref_x, ref_y, (float)MARGINS[m],
                        LEVEL_RANGES[l][0], LEVEL_RANGES[l][1], out_buf, 200000);

                    if (!query_have && !query_eof) {
                        query_have = read_query_row(qf, &query_pending);
                        if (!query_have) query_eof = 1;
                    }
                    total++;
                    if (!query_have || query_pending.frame_idx != (int)frame_idx ||
                        query_pending.q != (int)q || query_pending.margin != MARGINS[m] ||
                        query_pending.min_level != LEVEL_RANGES[l][0] ||
                        query_pending.max_level != LEVEL_RANGES[l][1] ||
                        !csv_matches(out_buf, n_res, query_pending.csv)) {
                        mismatches++;
                    }
                    /* Reference rows are 1:1 with the fixed query order we
                     * just generated -- always resync. */
                    query_have = 0;
                }
            }
        }

        sv_frame_grid_free(&grid);
        free(kpts);
    }

    free(kpts_buf);
    free(out_buf);
    fclose(kf);
    fclose(mf);
    fclose(gf);
    fclose(qf);

    printf("%s: %lld/%lld\n", seq_label, mismatches, total);
    return mismatches == 0 ? 0 : 1;
}
