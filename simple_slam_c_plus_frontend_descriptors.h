#ifndef SIMPLE_SLAM_C_PLUS_FRONTEND_DESCRIPTORS_H
#define SIMPLE_SLAM_C_PLUS_FRONTEND_DESCRIPTORS_H

static int g_brief_pattern[256 * 4];
static int g_oriented_brief = 0;
static int g_brief_patch_radius = 0;

static void brief_init_pattern(void) {
    srand(42);
    for (int i = 0; i < 256 * 4; i++)
        g_brief_pattern[i] = (rand() % 25) - 12;
}

static int compute_brief(const unsigned char *g, int w, int h, float fx, float fy, Brief256 *out) {
    int cx = (int)fx, cy = (int)fy;
    int patch_radius = g_brief_patch_radius;
    int margin = (g_oriented_brief ? 18 : 13) + patch_radius;
    if (cx < margin || cx >= w - margin || cy < margin || cy >= h - margin)
        return 0;
    double ct = 1.0, st = 0.0;
    if (g_oriented_brief) {
        // Intensity-centroid moments; integer sums are exact (|m| << 2^53),
        // so this matches the previous double accumulation bit for bit.
        long m10i = 0, m01i = 0;
        for (int dy = -12; dy <= 12; dy++) {
            for (int dx = -12; dx <= 12; dx++) {
                int v = g[(cy + dy) * w + (cx + dx)];
                m10i += dx * v;
                m01i += dy * v;
            }
        }
        double m10 = (double)m10i, m01 = (double)m01i;
        double norm = sqrt(m10 * m10 + m01 * m01);
        if (norm > 1e-9) {
            ct = m10 / norm;
            st = m01 / norm;
        }
    }
    memset(out->bits, 0, sizeof(out->bits));
    for (int i = 0; i < 256; i++) {
        int dx1 = g_brief_pattern[i * 4 + 0], dy1 = g_brief_pattern[i * 4 + 1],
            dx2 = g_brief_pattern[i * 4 + 2], dy2 = g_brief_pattern[i * 4 + 3];
        if (g_oriented_brief) {
            int rdx1 = (int)lrint(ct * (double)dx1 - st * (double)dy1);
            int rdy1 = (int)lrint(st * (double)dx1 + ct * (double)dy1);
            int rdx2 = (int)lrint(ct * (double)dx2 - st * (double)dy2);
            int rdy2 = (int)lrint(st * (double)dx2 + ct * (double)dy2);
            dx1 = rdx1;
            dy1 = rdy1;
            dx2 = rdx2;
            dy2 = rdy2;
        }
        int ax = cx + dx1, ay = cy + dy1, bx = cx + dx2, by = cy + dy2;
        int a = 0, b = 0;
        if (patch_radius > 0) {
            for (int py = -patch_radius; py <= patch_radius; py++) {
                for (int px = -patch_radius; px <= patch_radius; px++) {
                    a += g[(ay + py) * w + (ax + px)];
                    b += g[(by + py) * w + (bx + px)];
                }
            }
        } else {
            a = g[ay * w + ax];
            b = g[by * w + bx];
        }
        if (a > b)
            out->bits[i / 64] |= (uint64_t)1 << (i % 64);
    }
    return 1;
}

static int brief_hamming(const Brief256 *a, const Brief256 *b) {
    int d = 0;
    for (int i = 0; i < 4; i++)
        d += __builtin_popcountll(a->bits[i] ^ b->bits[i]);
    return d;
}

static int match_cmp_score_asc(const void *a, const void *b) {
    const Match *ma = (const Match *)a;
    const Match *mb = (const Match *)b;
    if (ma->score < mb->score)
        return -1;
    if (ma->score > mb->score)
        return 1;
    return 0;
}

static int brief_relink(const unsigned char *g, int w, int h, CornerVec *tracked, const Map *map) {
    if (map->size < 50)
        return 0;
    int relinked = 0;
    for (int i = 0; i < tracked->size; i++) {
        if (tracked->data[i].pt_idx != -1)
            continue;
        Brief256 d;
        if (!compute_brief(g, w, h, tracked->data[i].x, tracked->data[i].y, &d))
            continue;
        int best = 257, second = 257, best_idx = -1;
        for (int k = 0; k < map->size; k++) {
            int dist = brief_hamming(&d, &map->data[k].desc);
            if (dist < best) {
                second = best;
                best = dist;
                best_idx = k;
            } else if (dist < second)
                second = dist;
        }
        if (best < 35 && best * 10 < second * 6) {
            tracked->data[i].pt_idx = best_idx;
            relinked++;
        }
    }
    return relinked;
}

static void anchor_set_free(AnchorSet *a) {
    free(a->corners.data);
    free(a->desc);
    memset(a, 0, sizeof(*a));
}

static void anchor_set_build(AnchorSet *a, const unsigned char *gray, int w, int h,
                             const CornerVec *corners, const Pose *pose, int frame_id,
                             int max_features) {
    anchor_set_free(a);
    a->pose = *pose;
    a->frame_id = frame_id;
    int limit = max_features > 0 && max_features < corners->size ? max_features : corners->size;
    a->desc = (Brief256 *)malloc((size_t)limit * sizeof(Brief256));
    if (limit > 0 && !a->desc) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    for (int i = 0; i < corners->size && a->corners.size < limit; i++) {
        Brief256 d;
        if (!compute_brief(gray, w, h, corners->data[i].x, corners->data[i].y, &d))
            continue;
        corner_vec_push(&a->corners, corners->data[i]);
        a->desc[a->corners.size - 1] = d;
    }
}

static void match_anchor_sets(const AnchorSet *a, const AnchorSet *b, int max_hamming,
                              int mutual, double ratio, MatchVec *out) {
    out->size = 0;
    if (a->corners.size <= 0 || b->corners.size <= 0)
        return;
    int *reverse_best = NULL;
    if (mutual) {
        reverse_best = (int *)malloc((size_t)b->corners.size * sizeof(int));
        if (!reverse_best) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        for (int j = 0; j < b->corners.size; j++) {
            int best = 257, best_idx = -1;
            for (int i = 0; i < a->corners.size; i++) {
                int d = brief_hamming(&b->desc[j], &a->desc[i]);
                if (d < best) {
                    best = d;
                    best_idx = i;
                }
            }
            reverse_best[j] = best_idx;
        }
    }
    for (int i = 0; i < a->corners.size; i++) {
        int best = 257, second = 257, best_idx = -1;
        for (int j = 0; j < b->corners.size; j++) {
            int d = brief_hamming(&a->desc[i], &b->desc[j]);
            if (d < best) {
                second = best;
                best = d;
                best_idx = j;
            } else if (d < second) {
                second = d;
            }
        }
        if (best_idx < 0)
            continue;
        if (max_hamming > 0 && best > max_hamming)
            continue;
        if (ratio > 0.0 && second < 257 && (double)best >= ratio * (double)second)
            continue;
        if (mutual && reverse_best && reverse_best[best_idx] != i)
            continue;
        match_vec_push(out, (Match){i, best_idx, (float)best});
    }
    if (out->size > 1)
        qsort(out->data, (size_t)out->size, sizeof(Match), match_cmp_score_asc);
    free(reverse_best);
}

#endif
