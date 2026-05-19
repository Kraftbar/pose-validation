#ifndef SIMPLE_SLAM_C_PLUS_FRONTEND_FEATURES_H
#define SIMPLE_SLAM_C_PLUS_FRONTEND_FEATURES_H

#define FAST_CIRCLE_RADIUS 3
#define FAST_INTENSITY_THRESHOLD 20
#define FAST_CONTIGUOUS_ARC 9
#define FAST_NMS_RADIUS 7
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

static int corner_far_enough(const CornerVec *selected, float x, float y, int min_dist2) {
    for (int j = 0; j < selected->size; j++) {
        float dx = selected->data[j].x - x;
        float dy = selected->data[j].y - y;
        if (dx * dx + dy * dy < (float)min_dist2)
            return 0;
    }
    return 1;
}

static void select_corner_candidates(CornerScore *cand, int size, int w, int h, CornerVec *out,
                                     int max, int min_dist, int distributed) {
    if (size <= 0 || max <= 0)
        return;
    if (size > 1)
        qsort(cand, (size_t)size, sizeof(CornerScore), corner_score_cmp_desc);
    int min_dist2 = min_dist * min_dist;
    int *used = NULL;
    if (distributed) {
        used = (int *)calloc((size_t)size, sizeof(int));
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
            if (min_dist2 > 0 &&
                !corner_far_enough(out, cand[i].x, cand[i].y, min_dist2))
                continue;
            corner_vec_push(out, (Corner){cand[i].x, cand[i].y, -1, -1, 0.0f, 0.0f});
            counts[cell]++;
            used[i] = 1;
        }
    }
    for (int i = 0; i < size && out->size < max; i++) {
        if (used && used[i])
            continue;
        if (min_dist2 > 0 &&
            !corner_far_enough(out, cand[i].x, cand[i].y, min_dist2))
            continue;
        corner_vec_push(out, (Corner){cand[i].x, cand[i].y, -1, -1, 0.0f, 0.0f});
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

static void extract_corners_pure_legacy(const unsigned char *g, int w, int h, CornerVec *c,
                                        int max) {
    float *s = calloc((size_t)w * h, sizeof(float));
#pragma omp parallel for collapse(2)
    for (int y = 2; y < h - 2; y++)
        for (int x = 2; x < w - 2; x++) {
            float Ixx = 0, Iyy = 0, Ixy = 0;
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++) {
                    float gx = (float)g[(y + i) * w + x + j + 1] - g[(y + i) * w + x + j - 1],
                          gy = (float)g[(y + i + 1) * w + x + j] - g[(y + i - 1) * w + x + j];
                    Ixx += gx * gx;
                    Iyy += gy * gy;
                    Ixy += gx * gy;
                }
            float det = Ixx * Iyy - Ixy * Ixy, tr = Ixx + Iyy;
            s[y * w + x] = 0.5f * (tr - sqrtf(tr * tr - 4.0f * det + 1e-6f));
        }
    for (int y = 5; y < h - 5; y++)
        for (int x = 5; x < w - 5; x++) {
            float val = s[y * w + x];
            if (val < 0.1f)
                continue;
            int ok = 1;
            for (int i = -3; i <= 3; i++)
                for (int j = -3; j <= 3; j++)
                    if (s[(y + i) * w + x + j] > val) {
                        ok = 0;
                        break;
                    }
            if (ok) {
                float sum = 0, sx = 0, sy = 0;
                for (int i = -1; i <= 1; i++)
                    for (int j = -1; j <= 1; j++) {
                        float v = s[(y + i) * w + x + j];
                        sum += v;
                        sx += v * (x + j);
                        sy += v * (y + i);
                    }
                if (sum > 1e-6f)
                    corner_vec_push(c, (Corner){sx / sum, sy / sum, -1, -1, 0.0f, 0.0f});
                else
                    corner_vec_push(c, (Corner){(float)x, (float)y, -1, -1, 0.0f, 0.0f});
                if (c->size >= max)
                    break;
            }
        }
    free(s);
}

static void collect_harris_candidates(const unsigned char *g, int w, int h, float scale,
                                      CornerScore **cand, int *cand_size, int *cand_cap) {
    float *s = calloc((size_t)w * h, sizeof(float));
#pragma omp parallel for collapse(2)
    for (int y = 2; y < h - 2; y++)
        for (int x = 2; x < w - 2; x++) {
            float Ixx = 0, Iyy = 0, Ixy = 0;
            for (int i = -1; i <= 1; i++)
                for (int j = -1; j <= 1; j++) {
                    float gx = (float)g[(y + i) * w + x + j + 1] - g[(y + i) * w + x + j - 1],
                          gy = (float)g[(y + i + 1) * w + x + j] - g[(y + i - 1) * w + x + j];
                    Ixx += gx * gx;
                    Iyy += gy * gy;
                    Ixy += gx * gy;
                }
            float det = Ixx * Iyy - Ixy * Ixy, tr = Ixx + Iyy;
            s[y * w + x] = 0.5f * (tr - sqrtf(tr * tr - 4.0f * det + 1e-6f));
        }
    for (int y = 5; y < h - 5; y++)
        for (int x = 5; x < w - 5; x++) {
            float val = s[y * w + x];
            if (val < 0.1f)
                continue;
            int ok = 1;
            for (int i = -3; i <= 3; i++)
                for (int j = -3; j <= 3; j++)
                    if (s[(y + i) * w + x + j] > val) {
                        ok = 0;
                        break;
                    }
            if (ok) {
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

static void extract_corners_pure(const unsigned char *g, int w, int h, CornerVec *c, int max,
                                 int distributed, int pyramid, int min_dist) {
    if (!distributed && !pyramid) {
        extract_corners_pure_legacy(g, w, h, c, max);
        return;
    }
    CornerScore *cand = NULL;
    int cand_size = 0, cand_cap = 0;
    collect_harris_candidates(g, w, h, 1.0f, &cand, &cand_size, &cand_cap);
    if (pyramid && w >= 128 && h >= 128) {
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
    if (distributed || pyramid)
        select_corner_candidates(cand, cand_size, w, h, c, max, min_dist, 1);
    else {
        for (int i = 0; i < cand_size && c->size < max; i++)
            corner_vec_push(c, (Corner){cand[i].x, cand[i].y, -1, -1, 0.0f, 0.0f});
    }
    free(cand);
}

static void extract_corners_fast(const unsigned char *g, int w, int h, CornerVec *c, int max,
                                 int distributed) {
    static const int circle[16][2] = {
        {0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3},
        {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3},
    };
    CornerScore *cand = NULL;
    int size = 0, cap = 0;
    for (int y = FAST_CIRCLE_RADIUS; y < h - FAST_CIRCLE_RADIUS; y++) {
        for (int x = FAST_CIRCLE_RADIUS; x < w - FAST_CIRCLE_RADIUS; x++) {
            int center = g[y * w + x];
            int bright[32], dark[32];
            int score = 0;
            for (int i = 0; i < 16; i++) {
                int v = g[(y + circle[i][1]) * w + x + circle[i][0]];
                int d = v - center;
                bright[i] = bright[i + 16] = d > FAST_INTENSITY_THRESHOLD;
                dark[i] = dark[i + 16] = -d > FAST_INTENSITY_THRESHOLD;
                score += abs(d);
            }
            int ok = 0;
            for (int start = 0; start < 16 && !ok; start++) {
                int b = 0, d = 0;
                for (int k = 0; k < FAST_CONTIGUOUS_ARC; k++) {
                    b += bright[start + k];
                    d += dark[start + k];
                }
                ok = b == FAST_CONTIGUOUS_ARC || d == FAST_CONTIGUOUS_ARC;
            }
            if (!ok)
                continue;
            if (size == cap) {
                cap = cap ? cap * 2 : 1024;
                cand = (CornerScore *)xrealloc(cand, (size_t)cap * sizeof(CornerScore));
            }
            cand[size++] = (CornerScore){(float)x, (float)y, (float)score};
        }
    }
    select_corner_candidates(cand, size, w, h, c, max, FAST_NMS_RADIUS, distributed);
    free(cand);
}

static void refine_corners_subpixel(const unsigned char *g, int w, int h, CornerVec *c) {
    const int win = 5;
    const int margin = win + 2;
    for (int i = 0; i < c->size; i++) {
        float start_x = c->data[i].x;
        float start_y = c->data[i].y;
        float x = start_x;
        float y = start_y;
        if (x < margin || x >= w - margin || y < margin || y >= h - margin)
            continue;
        for (int iter = 0; iter < 8; iter++) {
            double Gxx = 0.0, Gxy = 0.0, Gyy = 0.0;
            double bx = 0.0, by = 0.0;
            for (int dy = -win; dy <= win; dy++) {
                for (int dx = -win; dx <= win; dx++) {
                    float px = x + (float)dx;
                    float py = y + (float)dy;
                    float Ix = (get_pixel_bilinear(g, w, h, px + 1.0f, py) -
                                get_pixel_bilinear(g, w, h, px - 1.0f, py)) *
                               0.5f;
                    float Iy = (get_pixel_bilinear(g, w, h, px, py + 1.0f) -
                                get_pixel_bilinear(g, w, h, px, py - 1.0f)) *
                               0.5f;
                    double gxx = (double)Ix * (double)Ix;
                    double gxy = (double)Ix * (double)Iy;
                    double gyy = (double)Iy * (double)Iy;
                    Gxx += gxx;
                    Gxy += gxy;
                    Gyy += gyy;
                    bx += gxx * (double)px + gxy * (double)py;
                    by += gxy * (double)px + gyy * (double)py;
                }
            }
            double det = Gxx * Gyy - Gxy * Gxy;
            if (fabs(det) < 1e-6)
                break;
            double nx = (Gyy * bx - Gxy * by) / det;
            double ny = (Gxx * by - Gxy * bx) / det;
            if (!isfinite(nx) || !isfinite(ny))
                break;
            double sx = nx - (double)x;
            double sy = ny - (double)y;
            double step2 = sx * sx + sy * sy;
            if (step2 > 1.0) {
                double inv = 1.0 / sqrt(step2);
                sx *= inv;
                sy *= inv;
                nx = (double)x + sx;
                ny = (double)y + sy;
            }
            if (nx < margin || nx >= w - margin || ny < margin || ny >= h - margin)
                break;
            x = (float)nx;
            y = (float)ny;
            if (step2 < 1e-4)
                break;
        }
        float ox = x - start_x;
        float oy = y - start_y;
        if (ox * ox + oy * oy <= 9.0f) {
            c->data[i].x = x;
            c->data[i].y = y;
        }
    }
}

static void extract_features_pure(const Config *cfg, const unsigned char *g, int w, int h,
                                  CornerVec *c, int max) {
    if (cfg->fast_corners)
        extract_corners_fast(g, w, h, c, max, cfg->distributed_features);
    else
        extract_corners_pure(g, w, h, c, max, cfg->distributed_features,
                             cfg->pyramid_features, cfg->feature_min_dist);
    if (cfg->subpixel_features)
        refine_corners_subpixel(g, w, h, c);
}

#endif
