#ifndef SIMPLE_SLAM_C_PLUS_FRONTEND_FEATURES_H
#define SIMPLE_SLAM_C_PLUS_FRONTEND_FEATURES_H

#define FEATURE_GRID_COLS 8
#define FEATURE_GRID_ROWS 6
#define FEATURE_GRID_CELLS (FEATURE_GRID_COLS * FEATURE_GRID_ROWS)

typedef struct {
    float x, y, score;
} CornerScore;

static int corner_score_cmp_desc(const void *a, const void *b) {
    const CornerScore *ca = (const CornerScore *)a;
    const CornerScore *cb = (const CornerScore *)b;
    if (ca->score > cb->score)
        return -1;
    if (ca->score < cb->score)
        return 1;
    return 0;
}

static int feature_grid_cell(float x, float y, int w, int h) {
    int gx = (int)((double)x * FEATURE_GRID_COLS / (double)w);
    int gy = (int)((double)y * FEATURE_GRID_ROWS / (double)h);
    if (gx < 0) gx = 0;
    if (gy < 0) gy = 0;
    if (gx >= FEATURE_GRID_COLS) gx = FEATURE_GRID_COLS - 1;
    if (gy >= FEATURE_GRID_ROWS) gy = FEATURE_GRID_ROWS - 1;
    return gy * FEATURE_GRID_COLS + gx;
}

// Rank candidates by Harris score, then take the best per grid cell first so
// features spread across the image; remaining budget is filled by raw rank.
static void select_corner_candidates(CornerScore *cand, int size, int w, int h, CornerVec *out,
                                     int max) {
    if (size <= 0 || max <= 0)
        return;
    if (size > 1)
        qsort(cand, (size_t)size, sizeof(CornerScore), corner_score_cmp_desc);
    int *used = (int *)calloc((size_t)size, sizeof(int));
    if (!used) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    int per_cell = (max + FEATURE_GRID_CELLS - 1) / FEATURE_GRID_CELLS;
    if (per_cell < 1)
        per_cell = 1;
    int counts[FEATURE_GRID_CELLS] = {0};
    for (int i = 0; i < size && out->size < max; i++) {
        int cell = feature_grid_cell(cand[i].x, cand[i].y, w, h);
        if (counts[cell] >= per_cell)
            continue;
        corner_vec_push(out, (Corner){cand[i].x, cand[i].y, -1, 0.0f, 0.0f});
        counts[cell]++;
        used[i] = 1;
    }
    for (int i = 0; i < size && out->size < max; i++) {
        if (used[i])
            continue;
        corner_vec_push(out, (Corner){cand[i].x, cand[i].y, -1, 0.0f, 0.0f});
    }
    free(used);
}

static void corner_score_vec_push(CornerScore **data, int *size, int *cap, CornerScore v) {
    if (*size == *cap) {
        *cap = *cap ? *cap * 2 : 1024;
        *data = (CornerScore *)xrealloc(*data, (size_t)*cap * sizeof(CornerScore));
    }
    (*data)[(*size)++] = v;
}

static void collect_harris_candidates(const unsigned char *g, int w, int h, float scale,
                                      CornerScore **cand, int *cand_size, int *cand_cap) {
    float *s = calloc((size_t)w * h, sizeof(float));
    float *gxx = malloc((size_t)w * h * sizeof(float));
    float *gyy = malloc((size_t)w * h * sizeof(float));
    float *gxy = malloc((size_t)w * h * sizeof(float));
    if (!s || !gxx || !gyy || !gxy) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }

    // Per-pixel gradient products, computed once and reused by all nine
    // overlapping structure-tensor windows below.
#pragma omp parallel for collapse(2)
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            float gx = (float)g[y * w + x + 1] - g[y * w + x - 1],
                  gy = (float)g[(y + 1) * w + x] - g[(y - 1) * w + x];
            gxx[y * w + x] = gx * gx;
            gyy[y * w + x] = gy * gy;
            gxy[y * w + x] = gx * gy;
        }
#pragma omp parallel for collapse(2)
    for (int y = 2; y < h - 2; y++)
        for (int x = 2; x < w - 2; x++) {
            float Ixx = 0, Iyy = 0, Ixy = 0;
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++) {
                    Ixx += gxx[(y + i) * w + x + j];
                    Iyy += gyy[(y + i) * w + x + j];
                    Ixy += gxy[(y + i) * w + x + j];
                }

            // Shi-Tomasi response: smaller eigenvalue of the structure tensor.
            //
            //   λ_min = (tr - sqrt(tr² - 4·det)) / 2
            float det = Ixx * Iyy - Ixy * Ixy, tr = Ixx + Iyy;
            s[y * w + x] = 0.5f * (tr - sqrtf(tr * tr - 4.0f * det + 1e-6f));
        }
    free(gxx);
    free(gyy);
    free(gxy);
    for (int y = 5; y < h - 5; y++)
        for (int x = 5; x < w - 5; x++) {
            float val = s[y * w + x];
            if (val < 0.1f)
                continue;
            int ok = 1;
            for (int i = -3; i <= 3 && ok; i++)
                for (int j = -3; j <= 3; j++)
                    if (s[(y + i) * w + x + j] > val) {
                        ok = 0;
                        break;
                    }
            if (ok) {
                // Score-weighted centroid over the 3x3 neighborhood.
                float sum = 0, sx = 0, sy = 0;
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++) {
                        float v = s[(y + i) * w + x + j];
                        sum += v;
                        sx += v * (x + j);
                        sy += v * (y + i);
                    }
                if (sum > 1e-6f) {
                    corner_score_vec_push(cand, cand_size, cand_cap,
                                          (CornerScore){sx * scale / sum, sy * scale / sum, val});
                } else {
                    corner_score_vec_push(cand, cand_size, cand_cap,
                                          (CornerScore){(float)x * scale, (float)y * scale, val});
                }
            }
        }
    free(s);
}

// Two-level Harris pyramid with grid-balanced selection (the promoted profile).
static void extract_features_pure(const unsigned char *g, int w, int h, CornerVec *c, int max) {
    CornerScore *cand = NULL;
    int cand_size = 0, cand_cap = 0;
    collect_harris_candidates(g, w, h, 1.0f, &cand, &cand_size, &cand_cap);
    if (w >= 128 && h >= 128) {
        int hw = w / 2, hh = h / 2;
        unsigned char *half = (unsigned char *)malloc((size_t)hw * (size_t)hh);
        if (!half) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        downsample2x(g, w, h, half);
        collect_harris_candidates(half, hw, hh, 2.0f, &cand, &cand_size, &cand_cap);
        free(half);
    }
    select_corner_candidates(cand, cand_size, w, h, c, max);
    free(cand);
}

#endif
