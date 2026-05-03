#include <float.h>
#include <limits.h>
#include <math.h>
#include <omp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pure_c_math.h"

static void jacobi_9x9(double A[81], double W[9], double V[81]) {
    jacobi_nxn(A, 9, W, V);
}
static void jacobi_12x12(double A[144], double W[12], double V[144]) {
    jacobi_nxn(A, 12, W, V);
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define CANDIDATE_MAX_OBS 8
#define ADMISSION_GRID_COLS 8
#define ADMISSION_GRID_ROWS 6
#define ADMISSION_GRID_CELLS (ADMISSION_GRID_COLS * ADMISSION_GRID_ROWS)
#define ADMISSION_MATCH_RADIUS_PX 3.0
#define ADMISSION_BATCH_INLIER_MULTIPLIER 4
#define FAST_CIRCLE_RADIUS 3
#define FAST_INTENSITY_THRESHOLD 20
#define FAST_CONTIGUOUS_ARC 9
#define FAST_NMS_RADIUS 7
#define FEATURE_GRID_COLS 8
#define FEATURE_GRID_ROWS 6
#define FEATURE_GRID_CELLS (FEATURE_GRID_COLS * FEATURE_GRID_ROWS)
#define HEALTHY_KF_MIN_LINKED 80
#define HEALTHY_KF_START_FRAME 200
#define LATE_KF_COOLDOWN_START_FRAME 200
#define LATE_KF_MIN_INTERVAL 2
#define MAP_HYGIENE_WINDOW 5
#define MAP_HYGIENE_REPROJ_PX 8.0
#define MAP_HYGIENE_START_FRAME 200

typedef struct {
    const char *video_path;
    double seconds, timeout;
    const char *metrics_out;
    const char *pnp_dump;
    const char *profile_out;
    const char *speed_profile;
    const char *track_dump;
    const char *map_admission_dump;
    const char *e_inlier_dump;
    int kf_min_inliers;
    double kf_max_rot_deg;
    int max_points;
    int new_point_obs;
    int pnp_min_obs;
    int pnp_start_frame;
    int delayed_init_frames;
    int candidate_tracks;
    int candidate_min_obs;
    int candidate_min_age;
    int candidate_grid_cols;
    int candidate_grid_rows;
    int candidate_promote_per_cell;
    double candidate_max_fb_err;
    double candidate_min_disp;
    double candidate_max_disp;
    int joint_ba;
    int proc_w, proc_h;
    int kf_period;
    int kf_min_interval;
    int healthy_keyframes;
    int late_kf_cooldown;
    int ba_interval;
    int global_ba_interval;
    int map_hygiene;
    int kf_warmup_frames;
    int first_kf_observations;
    int unique_kf_observations;
    int essential_cheirality_max;
    double tri_min_parallax_deg;
    double tri_max_reproj_px;
    double tri_max_depth;
    double tri_max_depth_ratio;
    int tri_source_kf_gap;
    double pnp_pred_reproj_gate;
    double pnp_quality_gate_px;
    int pnp_quality_min_obs;
    int pnp_quality_window;
    double obs_stat_gate_px;
    int obs_stat_min_good;
    double obs_stat_max_bad_ratio;
    int pnp_p3p_fallback;
    int pnp_p3p_observe;
    int pnp_p3p_iterations;
    double pnp_p3p_max_jump;
    int pnp_p3p_min_inl2;
    int pnp_p3p_min_gain;
    double pnp_p3p_max_mederr;
    double pnp_p3p_min_posz;
    int pnp_dlt_iters;
    int pnp_dlt_pretest;
    int pnp_dlt_pretest_margin;
    int ffmpeg_gray;
    int fast_corners;
    int distributed_features;
    int pyramid_features;
    int lk_iters;
    int lk_back_iters;
    int pose_lm_iters;
    int essential_iters;
    int descriptor_map_admission;
    int descriptor_primary_admission;
    int descriptor_mutual_admission;
    int oriented_brief;
    int descriptor_admission_max_hamming;
    double descriptor_admission_ratio;
    int descriptor_primary_map_cap;
    int descriptor_source_kf_gap;
    int triangulate_with_e_pose;
    int triangulate_relative_frame;
    int shape_e_inliers;
    int e_shape_max_matches;
    int e_shape_grid_cap;
    double e_shape_max_disp;
    double e_shape_max_fb_err;
    double e_shape_target_disp;
    int anchor_e_pose;
    int anchor_max_features;
    int anchor_max_hamming;
    double anchor_ratio;
    int admission_ranked;
    int admission_finite_only;
    int admission_batch_ranked;
    int admission_batch_deferred;
    double admission_target_disp;
    double admission_fb_weight;
    int admission_max_new_points;
    int admission_grid_cap;
} Config;
typedef struct {
    float x, y;
    int pt_idx;
    int cand_idx;
    float fb_err;
    float track_disp;
} Corner;
typedef struct {
    int query_idx, train_idx;
    float score;
} Match;
typedef struct {
    int frame_id, inliers, is_keyframe, points_added, points_total, method;
    int tracked_count, linked_points, linked_before_relink, relinked_points;
    int pnp_inliers, pred_lm_inliers, e_inliers;
    int pnp_fallback_used;
    int pnp_p3p_attempted, pnp_p3p_solved, pnp_p3p_accepted, pnp_p3p_inliers;
    int pnp_p3p_inliers2, pnp_p3p_inliers3, pnp_p3p_inliers5;
    double pnp_p3p_mederr, pnp_p3p_posz, pnp_p3p_jump;
    double pnp_p3p_xyz[3];
    double trans_jump;
    double xyz[3];
} FrameStat;
typedef struct {
    Corner *data;
    int size, cap;
} CornerVec;
typedef struct {
    Match *data;
    int size, cap;
} MatchVec;
typedef struct {
    FrameStat *data;
    int size, cap;
} FrameStatVec;
typedef struct {
    int frame_id;
    double decode, gray_blur, feature, lk, relink, pred_lm, pnp, essential;
    double pose_lm, triangulate, refill, loop, ba, global_ba, metrics;
} ProfileStat;
typedef struct {
    ProfileStat *data;
    int size, cap;
} ProfileStatVec;
typedef struct {
    double m[16];
} Pose;
typedef struct {
    unsigned char *data;
    int w, h;
} ImageGray;
typedef struct {
    CornerVec corners;
    Pose pose;
} FrameLite;
typedef struct {
    uint64_t bits[4];
} Brief256;
typedef struct {
    double x, y, z;
    int obs;
    unsigned short good_obs;
    unsigned short bad_obs;
    Brief256 desc;
} MapPoint;
typedef struct {
    MapPoint *data;
    int size, cap;
} Map;
typedef struct {
    CornerVec corners;
    Brief256 *desc;
    Pose pose;
    int frame_id;
} AnchorSet;
typedef struct {
    int frame_id;
    Pose pose;
    Corner corner;
} CandidateObs;
typedef struct {
    int valid;
    int first_frame;
    int last_frame;
    int obs;
    int obs_count;
    Pose anchor_pose;
    Corner anchor_corner;
    Corner last_corner;
    CandidateObs history[CANDIDATE_MAX_OBS];
    Brief256 desc;
} TrackCandidate;
typedef struct {
    TrackCandidate *data;
    int size, cap;
} CandidateVec;
typedef struct {
    int cand_idx;
    int corner_idx;
    int cell;
    double score;
    double X[3];
} CandidatePromotion;
typedef struct {
    CandidatePromotion *data;
    int size, cap;
} CandidatePromotionVec;
typedef struct {
    int match_idx;
    double score;
} AdmissionOrder;
typedef struct {
    int query_idx, train_idx, cell;
    double score, reproj, parallax, depth;
    double X[3], Xstat[3];
    Brief256 desc;
    Corner prev_corner, cur_corner;
    Pose prev_pose, cur_pose;
} AdmissionCandidate;
typedef struct {
    AdmissionCandidate *data;
    int size, cap;
} AdmissionCandidateVec;
typedef struct {
    float x, y, score;
} CornerScore;
typedef struct {
    int inliers2;
    int inliers3;
    int inliers5;
    double median_error;
    double positive_depth_ratio;
} PnPScore;
typedef struct {
    int attempted;
    int solved;
    int accepted;
    int inliers;
    int inliers2;
    int inliers3;
    int inliers5;
    double median_error;
    double positive_depth_ratio;
    double predicted_jump;
    double center[3];
} PnPProbe;

static int g_brief_pattern[256 * 4];
static int g_oriented_brief = 0;
static double vec3_dist(const double a[3], const double b[3]);
static void camera_center_from_pose(const Pose *pose, double c[3]);
static int triangulate_point(const Pose *p1, const Pose *p2, Corner pt1, Corner pt2,
                             double fx, double fy, double cx, double cy, double X[3]);
static int triangulation_quality_ok(const Pose *p1, const Pose *p2, Corner pt1, Corner pt2,
                                    double fx, double fy, double cx, double cy,
                                    const double X[3], double min_parallax_deg,
                                    double max_reproj_px, double max_depth,
                                    double max_depth_ratio);
static void brief_init_pattern(void) {
    srand(42);
    for (int i = 0; i < 256 * 4; i++)
        g_brief_pattern[i] = (rand() % 25) - 12;
}
static int compute_brief(const unsigned char *g, int w, int h, float fx, float fy, Brief256 *out) {
    int cx = (int)fx, cy = (int)fy;
    int margin = g_oriented_brief ? 18 : 13;
    if (cx < margin || cx >= w - margin || cy < margin || cy >= h - margin)
        return 0;
    double ct = 1.0, st = 0.0;
    if (g_oriented_brief) {
        double m10 = 0.0, m01 = 0.0;
        for (int dy = -12; dy <= 12; dy++) {
            for (int dx = -12; dx <= 12; dx++) {
                unsigned char v = g[(cy + dy) * w + (cx + dx)];
                m10 += (double)dx * (double)v;
                m01 += (double)dy * (double)v;
            }
        }
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
        unsigned char a = g[(cy + dy1) * w + (cx + dx1)], b = g[(cy + dy2) * w + (cx + dx2)];
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
static void corner_vec_push(CornerVec *vec, Corner v);
static void match_vec_push(MatchVec *vec, Match v);

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

static int match_cmp_score_asc(const void *a, const void *b) {
    const Match *ma = (const Match *)a;
    const Match *mb = (const Match *)b;
    if (ma->score < mb->score)
        return -1;
    if (ma->score > mb->score)
        return 1;
    return 0;
}

static int admission_order_cmp_score_asc(const void *a, const void *b) {
    const AdmissionOrder *oa = (const AdmissionOrder *)a;
    const AdmissionOrder *ob = (const AdmissionOrder *)b;
    if (oa->score < ob->score)
        return -1;
    if (oa->score > ob->score)
        return 1;
    return oa->match_idx - ob->match_idx;
}

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
    int *used = (int *)calloc((size_t)size, sizeof(int));
    if (!used) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    if (distributed) {
        int per_cell = (max + FEATURE_GRID_CELLS - 1) / FEATURE_GRID_CELLS;
        if (per_cell < 1)
            per_cell = 1;
        int counts[FEATURE_GRID_CELLS] = {0};
        for (int i = 0; i < size && out->size < max; i++) {
            int cell = feature_grid_cell(cand[i].x, cand[i].y, w, h);
            if (counts[cell] >= per_cell)
                continue;
            if (!corner_far_enough(out, cand[i].x, cand[i].y, min_dist2))
                continue;
            corner_vec_push(out, (Corner){cand[i].x, cand[i].y, -1, -1, 0.0f, 0.0f});
            counts[cell]++;
            used[i] = 1;
        }
    }
    for (int i = 0; i < size && out->size < max; i++) {
        if (used[i])
            continue;
        if (!corner_far_enough(out, cand[i].x, cand[i].y, min_dist2))
            continue;
        corner_vec_push(out, (Corner){cand[i].x, cand[i].y, -1, -1, 0.0f, 0.0f});
    }
    free(used);
}

static void build_brief_descriptor_matches(const unsigned char *prev_gray,
                                           const unsigned char *cur_gray,
                                           int w, int h,
                                           const CornerVec *prev_pts,
                                           const CornerVec *cur_pts,
                                           int max_hamming, double ratio,
                                           int mutual,
                                           MatchVec *out) {
    out->size = 0;
    if (prev_pts->size <= 0 || cur_pts->size <= 0)
        return;

    Brief256 *prev_desc = (Brief256 *)malloc((size_t)prev_pts->size * sizeof(Brief256));
    Brief256 *cur_desc = (Brief256 *)malloc((size_t)cur_pts->size * sizeof(Brief256));
    unsigned char *prev_valid = (unsigned char *)calloc((size_t)prev_pts->size, 1);
    unsigned char *cur_valid = (unsigned char *)calloc((size_t)cur_pts->size, 1);
    if (!prev_desc || !cur_desc || !prev_valid || !cur_valid) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    for (int i = 0; i < prev_pts->size; i++)
        prev_valid[i] = compute_brief(prev_gray, w, h, prev_pts->data[i].x,
                                      prev_pts->data[i].y, &prev_desc[i]) ? 1 : 0;
    for (int i = 0; i < cur_pts->size; i++)
        cur_valid[i] = compute_brief(cur_gray, w, h, cur_pts->data[i].x,
                                     cur_pts->data[i].y, &cur_desc[i]) ? 1 : 0;
    int *reverse_best = NULL;
    if (mutual) {
        reverse_best = (int *)malloc((size_t)cur_pts->size * sizeof(int));
        if (!reverse_best) {
            fprintf(stderr, "out of memory\n");
            exit(1);
        }
        for (int j = 0; j < cur_pts->size; j++) {
            reverse_best[j] = -1;
            if (!cur_valid[j])
                continue;
            int best = 257, best_idx = -1;
            for (int i = 0; i < prev_pts->size; i++) {
                if (!prev_valid[i])
                    continue;
                int d = brief_hamming(&cur_desc[j], &prev_desc[i]);
                if (d < best) {
                    best = d;
                    best_idx = i;
                }
            }
            reverse_best[j] = best_idx;
        }
    }

    for (int i = 0; i < prev_pts->size; i++) {
        if (!prev_valid[i])
            continue;
        int best = 257, second = 257, best_idx = -1;
        for (int j = 0; j < cur_pts->size; j++) {
            if (!cur_valid[j])
                continue;
            int d = brief_hamming(&prev_desc[i], &cur_desc[j]);
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
    free(prev_desc);
    free(cur_desc);
    free(prev_valid);
    free(cur_valid);
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
                              double ratio, MatchVec *out) {
    out->size = 0;
    if (a->corners.size <= 0 || b->corners.size <= 0)
        return;
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
        match_vec_push(out, (Match){i, best_idx, (float)best});
    }
    if (out->size > 1)
        qsort(out->data, (size_t)out->size, sizeof(Match), match_cmp_score_asc);
}

static void *xrealloc(void *ptr, size_t size) {
    void *out = realloc(ptr, size);
    if (!out) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return out;
}
static void corner_vec_push(CornerVec *vec, Corner v) {
    if (vec->size == vec->cap) {
        vec->cap = vec->cap ? vec->cap * 2 : 256;
        vec->data = (Corner *)xrealloc(vec->data, (size_t)vec->cap * sizeof(Corner));
    }
    vec->data[vec->size++] = v;
}
static void match_vec_push(MatchVec *vec, Match v) {
    if (vec->size == vec->cap) {
        vec->cap = vec->cap ? vec->cap * 2 : 256;
        vec->data = (Match *)xrealloc(vec->data, (size_t)vec->cap * sizeof(Match));
    }
    vec->data[vec->size++] = v;
}
static ImageGray image_gray_clone(const unsigned char *src, int w, int h) {
    ImageGray img = {0};
    img.w = w;
    img.h = h;
    if (w <= 0 || h <= 0)
        return img;
    img.data = (unsigned char *)malloc((size_t)w * (size_t)h);
    if (!img.data) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    memcpy(img.data, src, (size_t)w * (size_t)h);
    return img;
}
static ImageGray image_gray_alloc_zero(int w, int h) {
    ImageGray img = {0};
    img.w = w;
    img.h = h;
    if (w <= 0 || h <= 0)
        return img;
    img.data = (unsigned char *)calloc((size_t)w * (size_t)h, 1);
    if (!img.data) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return img;
}
static void image_gray_free(ImageGray *img) {
    free(img->data);
    memset(img, 0, sizeof(*img));
}
static void frame_stat_vec_push(FrameStatVec *vec, FrameStat v) {
    if (vec->size == vec->cap) {
        vec->cap = vec->cap ? vec->cap * 2 : 256;
        vec->data = (FrameStat *)xrealloc(vec->data, (size_t)vec->cap * sizeof(FrameStat));
    }
    vec->data[vec->size++] = v;
}
static void profile_stat_vec_push(ProfileStatVec *vec, ProfileStat v) {
    if (vec->size == vec->cap) {
        vec->cap = vec->cap ? vec->cap * 2 : 256;
        vec->data = (ProfileStat *)xrealloc(vec->data, (size_t)vec->cap * sizeof(ProfileStat));
    }
    vec->data[vec->size++] = v;
}
static void map_push(Map *m, MapPoint p) {
    if (m->size == m->cap) {
        m->cap = m->cap ? m->cap * 2 : 1024;
        m->data = (MapPoint *)xrealloc(m->data, (size_t)m->cap * sizeof(MapPoint));
    }
    m->data[m->size++] = p;
}
static int candidate_vec_push(CandidateVec *v, TrackCandidate c) {
    if (v->size == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 1024;
        v->data = (TrackCandidate *)xrealloc(v->data, (size_t)v->cap * sizeof(TrackCandidate));
    }
    v->data[v->size] = c;
    return v->size++;
}
static void candidate_promotion_vec_push(CandidatePromotionVec *v, CandidatePromotion p) {
    if (v->size == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 256;
        v->data = (CandidatePromotion *)xrealloc(v->data,
                                                 (size_t)v->cap * sizeof(CandidatePromotion));
    }
    v->data[v->size++] = p;
}
static void admission_candidate_vec_push(AdmissionCandidateVec *v, AdmissionCandidate p) {
    if (v->size == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 256;
        v->data = (AdmissionCandidate *)xrealloc(v->data,
                                                 (size_t)v->cap * sizeof(AdmissionCandidate));
    }
    v->data[v->size++] = p;
}
static int candidate_promotion_cmp_desc(const void *a, const void *b) {
    const CandidatePromotion *pa = (const CandidatePromotion *)a;
    const CandidatePromotion *pb = (const CandidatePromotion *)b;
    if (pa->score < pb->score)
        return 1;
    if (pa->score > pb->score)
        return -1;
    return 0;
}
static int admission_candidate_cmp_score_asc(const void *a, const void *b) {
    const AdmissionCandidate *pa = (const AdmissionCandidate *)a;
    const AdmissionCandidate *pb = (const AdmissionCandidate *)b;
    if (pa->score < pb->score)
        return -1;
    if (pa->score > pb->score)
        return 1;
    return pa->train_idx - pb->train_idx;
}
static void candidate_add_observation(TrackCandidate *tc, int frame_id, const Pose *pose,
                                      Corner corner) {
    if (!tc->valid)
        return;
    if (tc->obs_count > 0 && tc->history[tc->obs_count - 1].frame_id == frame_id)
        return;
    CandidateObs obs;
    obs.frame_id = frame_id;
    obs.pose = *pose;
    obs.corner = corner;
    obs.corner.pt_idx = -1;
    obs.corner.cand_idx = -1;
    if (tc->obs_count < CANDIDATE_MAX_OBS) {
        tc->history[tc->obs_count++] = obs;
    } else {
        memmove(&tc->history[0], &tc->history[1],
                (CANDIDATE_MAX_OBS - 1) * sizeof(CandidateObs));
        tc->history[CANDIDATE_MAX_OBS - 1] = obs;
    }
}

static int candidate_score_if_ready(CandidateVec *cands, int cand_idx, int frame_id,
                                    const Config *cfg, double fx, double fy, double cx,
                                    double cy, double X[3], double *score_out) {
    if (cand_idx < 0 || cand_idx >= cands->size)
        return 0;
    TrackCandidate *tc = &cands->data[cand_idx];
    if (!tc->valid)
        return 0;
    if (tc->obs < cfg->candidate_min_obs || frame_id - tc->first_frame < cfg->candidate_min_age)
        return 0;
    if (tc->obs_count < 2)
        return 0;
    int best_a = -1, best_b = -1;
    double best_dist = -1.0;
    for (int a = 0; a < tc->obs_count; a++) {
        double ca[3];
        camera_center_from_pose(&tc->history[a].pose, ca);
        for (int b = a + 1; b < tc->obs_count; b++) {
            double cb[3];
            camera_center_from_pose(&tc->history[b].pose, cb);
            double dist = vec3_dist(ca, cb);
            if (dist > best_dist) {
                best_dist = dist;
                best_a = a;
                best_b = b;
            }
        }
    }
    if (best_a < 0 || best_b < 0)
        return 0;
    CandidateObs *oa = &tc->history[best_a], *ob = &tc->history[best_b];
    if (!triangulate_point(&oa->pose, &ob->pose, oa->corner, ob->corner, fx, fy, cx, cy, X))
        return 0;
    if ((cfg->tri_min_parallax_deg > 0.0 || cfg->tri_max_reproj_px > 0.0 ||
         cfg->tri_max_depth > 0.0 || cfg->tri_max_depth_ratio > 0.0) &&
        !triangulation_quality_ok(&oa->pose, &ob->pose, oa->corner, ob->corner,
                                  fx, fy, cx, cy, X, cfg->tri_min_parallax_deg,
                                  cfg->tri_max_reproj_px, cfg->tri_max_depth,
                                  cfg->tri_max_depth_ratio))
        return 0;
    double dx = ob->corner.x - oa->corner.x, dy = ob->corner.y - oa->corner.y;
    double pix_disp = sqrt(dx * dx + dy * dy);
    if (score_out)
        *score_out = best_dist * 1000.0 + pix_disp + 2.0 * (double)tc->obs;
    return 1;
}

static void pose_identity(Pose *p) {
    memset(p->m, 0, sizeof(p->m));
    for (int i = 0; i < 4; i++)
        p->m[i * 4 + i] = 1.0;
}
static void pose_from_rt(const double R[9], const double t[3], Pose *p) {
    pose_identity(p);
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++)
            p->m[r * 4 + c] = R[r * 3 + c];
        p->m[r * 4 + 3] = t[r];
    }
}
static void pose_get_rotation(const Pose *p, double R[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            R[r * 3 + c] = p->m[r * 4 + c];
}
static void pose_get_translation(const Pose *p, double t[3]) {
    t[0] = p->m[3];
    t[1] = p->m[7];
    t[2] = p->m[11];
}
static void mat3_transpose(const double A[9], double T[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            T[r * 3 + c] = A[c * 3 + r];
}
static void mat3_mul(const double A[9], const double B[9], double C[9]) {
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++) {
            double s = 0;
            for (int k = 0; k < 3; k++)
                s += A[r * 3 + k] * B[k * 3 + c];
            C[r * 3 + c] = s;
        }
}
static void mat3_vec_mul(const double A[9], const double x[3], double y[3]) {
    for (int r = 0; r < 3; r++) {
        double s = 0;
        for (int k = 0; k < 3; k++)
            s += A[r * 3 + k] * x[k];
        y[r] = s;
    }
}
static void camera_point_to_world(const Pose *camera_pose, const double Xcam[3], double Xworld[3]) {
    double R[9], Rt[9], t[3], y[3];
    pose_get_rotation(camera_pose, R);
    pose_get_translation(camera_pose, t);
    mat3_transpose(R, Rt);
    y[0] = Xcam[0] - t[0];
    y[1] = Xcam[1] - t[1];
    y[2] = Xcam[2] - t[2];
    mat3_vec_mul(Rt, y, Xworld);
}
static double mat3_det(const double A[9]) {
    return A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) +
           A[2] * (A[3] * A[7] - A[4] * A[6]);
}
static double vec3_dot(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
static double vec3_dist(const double a[3], const double b[3]) {
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}
static void vec3_normalize(double v[3]) {
    double n = sqrt(vec3_dot(v, v));
    if (n > 1e-12) {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
    }
}
static int solve_3x3(double A[9], double b[3], double x[3]) {
    double M[12] = {
        A[0], A[1], A[2], b[0],
        A[3], A[4], A[5], b[1],
        A[6], A[7], A[8], b[2],
    };
    for (int k = 0; k < 3; k++) {
        int piv = k;
        double best = fabs(M[k * 4 + k]);
        for (int r = k + 1; r < 3; r++) {
            double v = fabs(M[r * 4 + k]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < 1e-12)
            return 0;
        if (piv != k) {
            for (int c = k; c < 4; c++) {
                double tmp = M[k * 4 + c];
                M[k * 4 + c] = M[piv * 4 + c];
                M[piv * 4 + c] = tmp;
            }
        }
        double inv = 1.0 / M[k * 4 + k];
        for (int c = k; c < 4; c++)
            M[k * 4 + c] *= inv;
        for (int r = 0; r < 3; r++) {
            if (r == k)
                continue;
            double f = M[r * 4 + k];
            for (int c = k; c < 4; c++)
                M[r * 4 + c] -= f * M[k * 4 + c];
        }
    }
    x[0] = M[3];
    x[1] = M[7];
    x[2] = M[11];
    return 1;
}
static void pose_compose_relative(const Pose *rel, const Pose *prev, Pose *out) {
    double Rrel[9], Rprev[9], Rout[9], trel[3], tprev[3], tout[3];
    pose_get_rotation(rel, Rrel);
    pose_get_rotation(prev, Rprev);
    pose_get_translation(rel, trel);
    pose_get_translation(prev, tprev);
    mat3_mul(Rrel, Rprev, Rout);
    mat3_vec_mul(Rrel, tprev, tout);
    tout[0] += trel[0];
    tout[1] += trel[1];
    tout[2] += trel[2];
    pose_from_rt(Rout, tout, out);
}
static void camera_center_from_pose(const Pose *pose, double c[3]) {
    double R[9], Rt[9], t[3], tmp[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    mat3_transpose(R, Rt);
    mat3_vec_mul(Rt, t, tmp);
    c[0] = -tmp[0];
    c[1] = -tmp[1];
    c[2] = -tmp[2];
}
static void pose_inverse(const Pose *p, Pose *out) {
    double R[9], Rt[9], t[3], nt[3];
    pose_get_rotation(p, R);
    pose_get_translation(p, t);
    mat3_transpose(R, Rt);
    mat3_vec_mul(Rt, t, nt);
    pose_from_rt(Rt, (double[]){-nt[0], -nt[1], -nt[2]}, out);
}
static double rotation_degrees_between(const Pose *a, const Pose *b) {
    double Ra[9], Rb[9], Rat[9], R[9];
    pose_get_rotation(a, Ra);
    pose_get_rotation(b, Rb);
    mat3_transpose(Ra, Rat);
    mat3_mul(Rb, Rat, R);
    double tr = R[0] + R[4] + R[8], cos_th = (tr - 1.0) * 0.5;
    if (cos_th > 1)
        cos_th = 1;
    if (cos_th < -1)
        cos_th = -1;
    return acos(cos_th) * 180.0 / M_PI;
}
static void normalize_point(double fx, double fy, double cx, double cy, Corner p, double out[3]) {
    out[0] = ((double)p.x - cx) / fx;
    out[1] = ((double)p.y - cy) / fy;
    out[2] = 1.0;
}

static int triangulate_point(const Pose *p1, const Pose *p2, Corner pt1, Corner pt2, double fx,
                             double fy, double cx, double cy, double X[3]) {
    double K[9] = {fx, 0, cx,
                   0, fy, cy,
                   0, 0, 1};
    double R1[9], R2[9], t1[3], t2[3], Rt1[12], Rt2[12], P1[12], P2[12], A[16], VT[4];
    pose_get_rotation(p1, R1);
    pose_get_rotation(p2, R2);
    pose_get_translation(p1, t1);
    pose_get_translation(p2, t2);
    // Pack [R | t] into 3x4 extrinsics for both cameras.
    for (int r = 0; r < 3; r++) {
        Rt1[r*4 + 0] = R1[r*3 + 0];  Rt1[r*4 + 1] = R1[r*3 + 1];  Rt1[r*4 + 2] = R1[r*3 + 2];  Rt1[r*4 + 3] = t1[r];
        Rt2[r*4 + 0] = R2[r*3 + 0];  Rt2[r*4 + 1] = R2[r*3 + 1];  Rt2[r*4 + 2] = R2[r*3 + 2];  Rt2[r*4 + 3] = t2[r];
    }
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 4; c++) {
            P1[r * 4 + c] = K[r * 3 + 0] * Rt1[c] + K[r * 3 + 1] * Rt1[c + 4] + K[r * 3 + 2] * Rt1[c + 8];
            P2[r * 4 + c] = K[r * 3 + 0] * Rt2[c] + K[r * 3 + 1] * Rt2[c + 4] + K[r * 3 + 2] * Rt2[c + 8];
        }
    // Build linear DLT system A·X = 0 (Hartley & Zisserman §12.2):
    //   row 0: x1 · P1[2,:] - P1[0,:]      row 2: x2 · P2[2,:] - P2[0,:]
    //   row 1: y1 · P1[2,:] - P1[1,:]      row 3: y2 · P2[2,:] - P2[1,:]
    for (int c = 0; c < 4; c++) {
        A[0*4 + c] = pt1.x*P1[2*4 + c] - P1[0*4 + c];
        A[1*4 + c] = pt1.y*P1[2*4 + c] - P1[1*4 + c];
        A[2*4 + c] = pt2.x*P2[2*4 + c] - P2[0*4 + c];
        A[3*4 + c] = pt2.y*P2[2*4 + c] - P2[1*4 + c];
    }
    double AtA[16] = {0}, V[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            for (int k = 0; k < 4; k++)
                AtA[r * 4 + c] += A[k * 4 + r] * A[k * 4 + c];
    for (int it = 0; it < 30; it++) {
        int p = 0, q = 1;
        double mv = 0;
        for (int i = 0; i < 3; i++)
            for (int j = i + 1; j < 4; j++)
                if (fabs(AtA[i * 4 + j]) > mv) {
                    mv = fabs(AtA[i * 4 + j]);
                    p = i;
                    q = j;
                }
        if (mv < 1e-15)
            break;
        double ph = 0.5 * atan2(2.0 * AtA[p * 4 + q], AtA[q * 4 + q] - AtA[p * 4 + p]),
               co = cos(ph), si = sin(ph);
        for (int i = 0; i < 4; i++) {
            double r1 = AtA[i * 4 + p], r2 = AtA[i * 4 + q];
            AtA[i * 4 + p] = co * r1 - si * r2;
            AtA[i * 4 + q] = si * r1 + co * r2;
        }
        for (int i = 0; i < 4; i++) {
            double r1 = AtA[p * 4 + i], r2 = AtA[q * 4 + i];
            AtA[p * 4 + i] = co * r1 - si * r2;
            AtA[q * 4 + i] = si * r1 + co * r2;
        }
        for (int i = 0; i < 4; i++) {
            double v1 = V[i * 4 + p], v2 = V[i * 4 + q];
            V[i * 4 + p] = co * v1 - si * v2;
            V[i * 4 + q] = si * v1 + co * v2;
        }
    }
    int bi = 0;
    double mw = AtA[0];
    for (int i = 1; i < 4; i++)
        if (AtA[i * 4 + i] < mw) {
            mw = AtA[i * 4 + i];
            bi = i;
        }
    for (int i = 0; i < 4; i++)
        VT[i] = V[i * 4 + bi];
    if (fabs(VT[3]) < 1e-8)
        return 0;
    X[0] = VT[0] / VT[3];
    X[1] = VT[1] / VT[3];
    X[2] = VT[2] / VT[3];

    // Cheirality: depth in both cameras must be positive.
    double z1 = R1[6]*X[0] + R1[7]*X[1] + R1[8]*X[2] + t1[2];
    double z2 = R2[6]*X[0] + R2[7]*X[1] + R2[8]*X[2] + t2[2];
    return (isfinite(X[0]) && z1 > 0 && z2 > 0);
}

static double reprojection_error_xyz(const Pose *pose, const double X[3], Corner corner,
                                     double fx, double fy, double cx, double cy, double *out_depth) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double x = R[0] * X[0] + R[1] * X[1] + R[2] * X[2] + t[0],
           y = R[3] * X[0] + R[4] * X[1] + R[5] * X[2] + t[1],
           z = R[6] * X[0] + R[7] * X[1] + R[8] * X[2] + t[2];
    if (out_depth)
        *out_depth = z;
    if (z < 0.1)
        return INFINITY;
    double u = fx * x / z + cx, v = fy * y / z + cy;
    double dx = u - corner.x, dy = v - corner.y;
    return sqrt(dx * dx + dy * dy);
}

static int triangulation_quality_ok(const Pose *p1, const Pose *p2, Corner pt1, Corner pt2,
                                    double fx, double fy, double cx, double cy,
                                    const double X[3], double min_parallax_deg,
                                    double max_reproj_px, double max_depth,
                                    double max_depth_ratio) {
    double z1 = 0.0, z2 = 0.0;
    double e1 = reprojection_error_xyz(p1, X, pt1, fx, fy, cx, cy, &z1);
    double e2 = reprojection_error_xyz(p2, X, pt2, fx, fy, cx, cy, &z2);
    if (!isfinite(e1) || !isfinite(e2))
        return 0;
    if (max_reproj_px > 0.0 && (e1 > max_reproj_px || e2 > max_reproj_px))
        return 0;
    if (max_depth > 0.0 && (z1 > max_depth || z2 > max_depth))
        return 0;
    if (max_depth_ratio > 0.0) {
        double mn = z1 < z2 ? z1 : z2, mx = z1 > z2 ? z1 : z2;
        if (mn <= 0.0 || mx / mn > max_depth_ratio)
            return 0;
    }
    if (min_parallax_deg > 0.0) {
        double c1[3], c2[3], r1[3], r2[3];
        camera_center_from_pose(p1, c1);
        camera_center_from_pose(p2, c2);
        for (int i = 0; i < 3; i++) {
            r1[i] = X[i] - c1[i];
            r2[i] = X[i] - c2[i];
        }
        vec3_normalize(r1);
        vec3_normalize(r2);
        double cs = vec3_dot(r1, r2);
        if (cs > 1.0)
            cs = 1.0;
        if (cs < -1.0)
            cs = -1.0;
        double parallax_deg = acos(cs) * 180.0 / M_PI;
        if (parallax_deg < min_parallax_deg)
            return 0;
    }
    return 1;
}

static int find_near_corner(const CornerVec *corners, Corner p, double radius_px) {
    double best = radius_px * radius_px;
    int best_i = -1;
    for (int i = 0; i < corners->size; i++) {
        double dx = corners->data[i].x - p.x, dy = corners->data[i].y - p.y;
        double d2 = dx * dx + dy * dy;
        if (d2 <= best) {
            best = d2;
            best_i = i;
        }
    }
    return best_i;
}

static void build_shaped_e_matches(const Config *cfg, const CornerVec *prev_pts,
                                   const CornerVec *cur_pts, const MatchVec *matches,
                                   int w, int h, MatchVec *out) {
    out->size = 0;
    if (!cfg->shape_e_inliers)
        return;
    int cell_counts[48] = {0};
    MatchVec candidates = {0};
    for (int i = 0; i < matches->size; i++) {
        Match m = matches->data[i];
        if (m.query_idx < 0 || m.query_idx >= prev_pts->size ||
            m.train_idx < 0 || m.train_idx >= cur_pts->size)
            continue;
        Corner a = prev_pts->data[m.query_idx];
        Corner b = cur_pts->data[m.train_idx];
        double dx = (double)b.x - (double)a.x;
        double dy = (double)b.y - (double)a.y;
        double disp = sqrt(dx * dx + dy * dy);
        if (cfg->e_shape_max_disp > 0.0 && disp > cfg->e_shape_max_disp)
            continue;
        if (cfg->e_shape_max_fb_err > 0.0 && b.fb_err > cfg->e_shape_max_fb_err)
            continue;
        double fb = b.fb_err > 0.0f ? b.fb_err : 0.0;
        m.score = (float)(fabs(disp - cfg->e_shape_target_disp) + 10.0 * fb);
        match_vec_push(&candidates, m);
    }
    if (candidates.size > 1)
        qsort(candidates.data, (size_t)candidates.size, sizeof(Match), match_cmp_score_asc);
    for (int i = 0; i < candidates.size; i++) {
        Match m = candidates.data[i];
        Corner b = cur_pts->data[m.train_idx];
        int gx = (int)(b.x * 8.0 / (double)w);
        int gy = (int)(b.y * 6.0 / (double)h);
        if (gx < 0) gx = 0;
        if (gy < 0) gy = 0;
        if (gx > 7) gx = 7;
        if (gy > 5) gy = 5;
        int cell = gy * 8 + gx;
        if (cfg->e_shape_grid_cap > 0 && cell_counts[cell] >= cfg->e_shape_grid_cap)
            continue;
        match_vec_push(out, m);
        cell_counts[cell]++;
        if (cfg->e_shape_max_matches > 0 && out->size >= cfg->e_shape_max_matches)
            break;
    }
    free(candidates.data);
}

static int promote_candidate_if_ready(CandidateVec *cands, int cand_idx, int frame_id,
                                      const Pose *pose, Corner current, double fx, double fy,
                                      double cx, double cy, const Config *cfg, Map *map,
                                      unsigned char *gray, int w, int h, int *pts, int *tri) {
    if (cand_idx < 0 || cand_idx >= cands->size)
        return -1;
    TrackCandidate *tc = &cands->data[cand_idx];
    if (!tc->valid)
        return -1;
    candidate_add_observation(tc, frame_id, pose, current);
    if (tc->obs < cfg->candidate_min_obs || frame_id - tc->first_frame < cfg->candidate_min_age)
        return -1;
    double X[3];
    if (!candidate_score_if_ready(cands, cand_idx, frame_id, cfg, fx, fy, cx, cy, X, NULL))
        return -1;
    Brief256 d = {{0, 0, 0, 0}};
    compute_brief(gray, w, h, current.x, current.y, &d);
    int map_idx = map->size;
    MapPoint mp = {X[0], X[1], X[2], cfg->new_point_obs > 2 ? cfg->new_point_obs : 2, 0, 0, d};
    map_push(map, mp);
    tc->valid = 0;
    if (pts)
        (*pts)++;
    if (tri)
        (*tri)++;
    return map_idx;
}

// Project a 3x3 onto the essential-matrix manifold:
//   E = U · diag(σ, σ, 0) · V^T,  with σ = (σ₁ + σ₂) / 2.
// (Hartley & Zisserman §11.7.3.)
static void enforce_essential_constraints(double E[9]) {
    double W[3], U[9], V[9], S[9] = {0}, tmp[9], out[9];
    svd_3x3(E, W, U, V);

    double sig = 0.5 * (W[0] + W[1]);
    S[0] = sig;
    S[4] = sig;
    S[8] = 0;

    mat3_mul(U, S, tmp);
    mat3_mul(tmp, V, out);
    memcpy(E, out, 9 * sizeof(double));
}

static int decompose_and_choose_pose(const double E[9], const CornerVec *p_pts,
                                     const CornerVec *c_pts, const MatchVec *matches,
                                     const unsigned char *mask, double fx, double fy, double cx,
                                     double cy, int max_check, Pose *out_rel) {
    // Recover (R, t) from E by SVD (Hartley & Zisserman §9.6.2).
    //   E = U · Σ · V^T
    //   t = ±U[:,2]
    //   R = U · W · V^T   or   U · W^T · V^T   (with W the rotation-by-π/2 matrix)
    // Up to a fourfold sign ambiguity → 4 candidate poses; pick the one
    // with the most triangulated points in front of both cameras (cheirality).
    double W[3], U[9], V[9], R1[9], R2[9], t[3];
    Pose cands[4];
    int bi = -1, bc = -1;
    svd_3x3(E, W, U, V);

    t[0] = U[2];
    t[1] = U[5];
    t[2] = U[8];

    double Wm[9]  = {0, -1, 0,
                     1,  0, 0,
                     0,  0, 1};
    double Wmt[9] = { 0, 1, 0,
                     -1, 0, 0,
                      0, 0, 1};

    double tmp[9], VT[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            VT[i*3 + j] = V[j*3 + i];

    mat3_mul(U, Wm,  tmp);  mat3_mul(tmp, VT, R1);
    mat3_mul(U, Wmt, tmp);  mat3_mul(tmp, VT, R2);

    // Fix reflection: enforce det(R) = +1 (proper rotation).
    if (mat3_det(R1) < 0) for (int i = 0; i < 9; i++) R1[i] = -R1[i];
    if (mat3_det(R2) < 0) for (int i = 0; i < 9; i++) R2[i] = -R2[i];

    pose_from_rt(R1,  t,                            &cands[0]);
    pose_from_rt(R1, (double[]){-t[0], -t[1], -t[2]}, &cands[1]);
    pose_from_rt(R2,  t,                            &cands[2]);
    pose_from_rt(R2, (double[]){-t[0], -t[1], -t[2]}, &cands[3]);
    for (int i = 0; i < 4; i++) {
        Pose id;
        pose_identity(&id);
        int g = 0;
        int limit = max_check > 0 && max_check < matches->size ? max_check : matches->size;
        for (int j = 0; j < limit; j++) {
            double X[3];
            if (mask && !mask[j])
                continue;
            if (triangulate_point(&id, &cands[i], p_pts->data[matches->data[j].query_idx],
                                  c_pts->data[matches->data[j].train_idx], fx, fy, cx, cy, X))
                g++;
        }
        if (g > bc) {
            bc = g;
            bi = i;
        }
    }
    if (bi >= 0) {
        *out_rel = cands[bi];
        return 1;
    }
    return 0;
}

static int estimate_essential_from_indices(const CornerVec *p_pts, const CornerVec *c_pts,
                                           const MatchVec *matches, const int *idxs, int n,
                                           double fx, double fy, double cx, double cy,
                                           double E[9]) {
    double AtA[81] = {0}, W[9], V[81];
    if (n < 8)
        return 0;
    for (int i = 0; i < n; i++) {
        double x1[3], x2[3], A[9];
        Match m = matches->data[idxs[i]];
        normalize_point(fx, fy, cx, cy, p_pts->data[m.query_idx], x1);
        normalize_point(fx, fy, cx, cy, c_pts->data[m.train_idx], x2);
        A[0] = x2[0] * x1[0];
        A[1] = x2[0] * x1[1];
        A[2] = x2[0];
        A[3] = x2[1] * x1[0];
        A[4] = x2[1] * x1[1];
        A[5] = x2[1];
        A[6] = x1[0];
        A[7] = x1[1];
        A[8] = 1.0;
        for (int r = 0; r < 9; r++)
            for (int c = 0; c < 9; c++)
                AtA[r * 9 + c] += A[r] * A[c];
    }
    jacobi_9x9(AtA, W, V);
    int bi = 0;
    double mw = W[0];
    for (int k = 1; k < 9; k++)
        if (W[k] < mw) {
            mw = W[k];
            bi = k;
        }
    for (int k = 0; k < 9; k++)
        E[k] = V[k * 9 + bi];
    return 1;
}

static int estimate_pose_E(const CornerVec *p_pts, const CornerVec *c_pts, const MatchVec *matches,
                           double fx, double fy, double cx, double cy, Pose *out_rel,
                           int max_iters, int cheirality_max, unsigned char **out_mask,
                           int *out_inliers) {
    const double th = 1e-4;
    int iters = max_iters > 0 ? max_iters : 500;
    double best_E[9] = {0};
    int best_inl = 0;
    if (matches->size < 8)
        return 0;
    double *n1 = malloc(matches->size * 3 * sizeof(double)),
           *n2 = malloc(matches->size * 3 * sizeof(double));
    for (int i = 0; i < matches->size; i++) {
        normalize_point(fx, fy, cx, cy, p_pts->data[matches->data[i].query_idx], &n1[i * 3]);
        normalize_point(fx, fy, cx, cy, c_pts->data[matches->data[i].train_idx], &n2[i * 3]);
    }
    unsigned char *bmask = calloc(matches->size, 1);
    srand(0);
    for (int i = 0; i < iters; i++) {
        int smp[8];
        unsigned char used[4096] = {0};
        double cand_E[9], AtA[81] = {0}, W[9], V[81];
        for (int j = 0; j < 8; j++) {
            int idx;
            do {
                idx = rand() % matches->size;
            } while (used[idx % 4096]);
            used[idx % 4096] = 1;
            smp[j] = idx;
        }
        for (int k = 0; k < 8; k++) {
            double *x1 = &n1[smp[k] * 3], *x2 = &n2[smp[k] * 3];
            double A[9] = {x2[0] * x1[0], x2[0] * x1[1], x2[0],
                           x2[1] * x1[0], x2[1] * x1[1], x2[1],
                           x1[0], x1[1], 1.0};
            for (int r = 0; r < 9; r++)
                for (int c = 0; c < 9; c++)
                    AtA[r * 9 + c] += A[r] * A[c];
        }
        jacobi_9x9(AtA, W, V);
        int bi = 0;
        double mw = W[0];
        for (int k = 1; k < 9; k++)
            if (W[k] < mw) {
                mw = W[k];
                bi = k;
            }
        for (int k = 0; k < 9; k++)
            cand_E[k] = V[k * 9 + bi];
        enforce_essential_constraints(cand_E);
        int inl = 0;
        double Et[9];
        mat3_transpose(cand_E, Et);
        for (int j = 0; j < matches->size; j++) {
            double *x1 = &n1[j * 3], *x2 = &n2[j * 3],
                   Ex1[3] = {cand_E[0] * x1[0] + cand_E[1] * x1[1] + cand_E[2],
                             cand_E[3] * x1[0] + cand_E[4] * x1[1] + cand_E[5],
                             cand_E[6] * x1[0] + cand_E[7] * x1[1] + cand_E[8]},
                   Etx2[3] = {Et[0] * x2[0] + Et[1] * x2[1] + Et[2],
                              Et[3] * x2[0] + Et[4] * x2[1] + Et[5],
                              Et[6] * x2[0] + Et[7] * x2[1] + Et[8]},
                   num = x2[0] * Ex1[0] + x2[1] * Ex1[1] + Ex1[2],
                   den = Ex1[0] * Ex1[0] + Ex1[1] * Ex1[1] + Etx2[0] * Etx2[0] + Etx2[1] * Etx2[1] +
                         1e-12;
            if (num * num / den < th)
                inl++;
        }
        if (inl > best_inl) {
            best_inl = inl;
            memcpy(best_E, cand_E, 9 * sizeof(double));
            if (inl > matches->size * 0.95)
                break;
        }
    }
    if (best_inl >= 8) {
        double Et[9];
        mat3_transpose(best_E, Et);
        int *iidx = malloc(matches->size * sizeof(int));
        int k = 0;
        for (int j = 0; j < matches->size; j++) {
            double *x1 = &n1[j * 3], *x2 = &n2[j * 3],
                   Ex1[3] = {best_E[0] * x1[0] + best_E[1] * x1[1] + best_E[2],
                             best_E[3] * x1[0] + best_E[4] * x1[1] + best_E[5],
                             best_E[6] * x1[0] + best_E[7] * x1[1] + best_E[8]},
                   Etx2[3] = {Et[0] * x2[0] + Et[1] * x2[1] + Et[2],
                              Et[3] * x2[0] + Et[4] * x2[1] + Et[5],
                              Et[6] * x2[0] + Et[7] * x2[1] + Et[8]},
                   num = x2[0] * Ex1[0] + x2[1] * Ex1[1] + Ex1[2],
                   den = Ex1[0] * Ex1[0] + Ex1[1] * Ex1[1] + Etx2[0] * Etx2[0] + Etx2[1] * Etx2[1] +
                         1e-12;
            if (num * num / den < th) {
                bmask[j] = 1;
                iidx[k++] = j;
            }
        }
        estimate_essential_from_indices(p_pts, c_pts, matches, iidx, k, fx, fy, cx, cy, best_E);
        enforce_essential_constraints(best_E);
        free(iidx);
    }
    free(n1);
    free(n2);
    if (best_inl >= 8 &&
        decompose_and_choose_pose(best_E, p_pts, c_pts, matches, bmask, fx, fy, cx, cy,
                                  cheirality_max, out_rel)) {
        *out_mask = bmask;
        *out_inliers = best_inl;
        return 1;
    }
    free(bmask);
    return 0;
}

static int count_pnp_linear_inliers(const Map *map, const CornerVec *corners, const int *ids,
                                    int n, double fx, double fy, double cx, double cy,
                                    const double P[12], int max_count) {
    double R[9] = {P[0], P[1], P[2], P[4], P[5], P[6], P[8], P[9], P[10]},
           t[3] = {P[3], P[7], P[11]};
    double norm = sqrt(R[6] * R[6] + R[7] * R[7] + R[8] * R[8]);
    if (!isfinite(norm) || norm < 1e-12)
        return 0;
    double sc = 1.0 / norm;
    if (P[11] * sc < 0)
        sc = -sc;
    for (int j = 0; j < 9; j++)
        R[j] *= sc;
    for (int j = 0; j < 3; j++)
        t[j] *= sc;
    int limit = max_count > 0 && max_count < n ? max_count : n;
    int inl = 0;
    for (int j = 0; j < limit; j++) {
        MapPoint p = map->data[corners->data[ids[j]].pt_idx];
        double cx_p = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
               cy_p = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
               cz_p = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (cz_p < 0.1)
            continue;
        double u = fx * cx_p / cz_p + cx, v = fy * cy_p / cz_p + cy;
        double dx = u - corners->data[ids[j]].x, dy = v - corners->data[ids[j]].y;
        if (dx * dx + dy * dy < 4.0)
            inl++;
    }
    return inl;
}

static int estimate_pose_PnP(const Map *map, const CornerVec *corners, double fx, double fy,
                             double cx, double cy, int dlt_iters, int dlt_pretest,
                             int dlt_pretest_margin, int min_obs, const unsigned char *point_ok,
                             Pose *out_pose, int *out_inl) {
    if (out_inl)
        *out_inl = 0;
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1)
            n++;
    if (n < 12)
        return 0;
    int *ids = malloc(n * sizeof(int));
    int k = 0;
    for (int i = 0; i < corners->size; i++) {
        int pi = corners->data[i].pt_idx;
        if (pi != -1 && map->data[pi].obs >= min_obs && (!point_ok || point_ok[pi]))
            ids[k++] = i;
    }
    n = k;
    if (n < 12) {
        free(ids);
        return 0;
    }
    double best_P[12];
    int best_inl = 0;
    int best_pre = 0;
    int iters = dlt_iters > 0 ? dlt_iters : 500;
    srand(0);
    for (int it = 0; it < iters; it++) {
        double AtA[144] = {0}, W[12], V[144], P[12];
        for (int i = 0; i < 6; i++) {
            int idx = ids[rand() % n];
            MapPoint p = map->data[corners->data[idx].pt_idx];
            double u = (corners->data[idx].x - cx) / fx, v = (corners->data[idx].y - cy) / fy;
            double r1[12] = {p.x, p.y, p.z, 1,   0,   0,   0,   0,  -u*p.x, -u*p.y, -u*p.z, -u};
            double r2[12] = {  0,   0,   0, 0, p.x, p.y, p.z, 1,   -v*p.x, -v*p.y, -v*p.z, -v};
            for (int r = 0; r < 12; r++)
                for (int c = 0; c < 12; c++)
                    AtA[r * 12 + c] += r1[r] * r1[c] + r2[r] * r2[c];
        }
        jacobi_12x12(AtA, W, V);
        int bi = 0;
        double mw = W[0];
        for (int j = 1; j < 12; j++)
            if (W[j] < mw) {
                mw = W[j];
                bi = j;
            }
        for (int j = 0; j < 12; j++)
            P[j] = V[j * 12 + bi];
        int pre_inl = n;
        if (dlt_pretest > 0) {
            pre_inl = count_pnp_linear_inliers(map, corners, ids, n, fx, fy, cx, cy, P,
                                               dlt_pretest);
            if (best_pre > 0 && pre_inl + dlt_pretest_margin < best_pre)
                continue;
        }
        int inl = count_pnp_linear_inliers(map, corners, ids, n, fx, fy, cx, cy, P, 0);
        if (inl > best_inl) {
            best_inl = inl;
            best_pre = pre_inl;
            memcpy(best_P, P, 12 * sizeof(double));
        }
        if (inl > n * 0.8)
            break;
    }
    if (out_inl)
        *out_inl = best_inl;
    if (best_inl >= 12) {
        double R[9] = {best_P[0], best_P[1], best_P[2], best_P[4], best_P[5],
                       best_P[6], best_P[8], best_P[9], best_P[10]},
               t[3] = {best_P[3], best_P[7], best_P[11]};
        double sc = 1.0 / sqrt(R[6] * R[6] + R[7] * R[7] + R[8] * R[8]);
        if (best_P[11] * sc < 0)
            sc = -sc;
        for (int j = 0; j < 9; j++)
            R[j] *= sc;
        for (int j = 0; j < 3; j++)
            t[j] *= sc;
        double W[3], U[9], V[9], Ro[9];
        svd_3x3(R, W, U, V);
        double VT[9];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                VT[i * 3 + j] = V[j * 3 + i];
        mat3_mul(U, VT, Ro);
        if (mat3_det(Ro) < 0)
            for (int j = 0; j < 9; j++)
                Ro[j] = -Ro[j];
        pose_from_rt(Ro, t, out_pose);
        free(ids);
        return 1;
    }
    free(ids);
    return 0;
}

static int pose_from_3d3d(double Pw[3][3], double Pc[3][3], Pose *pose) {
    double cw[3] = {0}, cc[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            cw[k] += Pw[i][k];
            cc[k] += Pc[i][k];
        }
    }
    for (int k = 0; k < 3; k++) {
        cw[k] /= 3.0;
        cc[k] /= 3.0;
    }
    double H[9] = {0};
    for (int i = 0; i < 3; i++) {
        double pw[3] = {Pw[i][0] - cw[0], Pw[i][1] - cw[1], Pw[i][2] - cw[2]};
        double pc[3] = {Pc[i][0] - cc[0], Pc[i][1] - cc[1], Pc[i][2] - cc[2]};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                H[r * 3 + c] += pc[r] * pw[c];
    }
    double W[3], U[9], V[9], VT[9], R[9];
    svd_3x3(H, W, U, V);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            VT[i * 3 + j] = V[j * 3 + i];
    mat3_mul(U, VT, R);
    if (mat3_det(R) < 0) {
        for (int r = 0; r < 3; r++)
            U[r * 3 + 2] = -U[r * 3 + 2];
        mat3_mul(U, VT, R);
    }
    double t[3] = {
        cc[0] - (R[0] * cw[0] + R[1] * cw[1] + R[2] * cw[2]),
        cc[1] - (R[3] * cw[0] + R[4] * cw[1] + R[5] * cw[2]),
        cc[2] - (R[6] * cw[0] + R[7] * cw[1] + R[8] * cw[2]),
    };
    if (!isfinite(t[0]) || !isfinite(t[1]) || !isfinite(t[2]))
        return 0;
    pose_from_rt(R, t, pose);
    return 1;
}

static int numeric_p3p_depths(double f[3][3], double Pw[3][3], const double seed[3],
                              double out_lam[3]) {
    const double d01 = vec3_dist(Pw[0], Pw[1]);
    const double d02 = vec3_dist(Pw[0], Pw[2]);
    const double d12 = vec3_dist(Pw[1], Pw[2]);
    const double c01 = vec3_dot(f[0], f[1]);
    const double c02 = vec3_dot(f[0], f[2]);
    const double c12 = vec3_dot(f[1], f[2]);
    double l[3] = {seed[0], seed[1], seed[2]};
    if (l[0] <= 0 || l[1] <= 0 || l[2] <= 0)
        return 0;
    for (int iter = 0; iter < 30; iter++) {
        double r[3] = {
            l[0] * l[0] + l[1] * l[1] - 2.0 * l[0] * l[1] * c01 - d01 * d01,
            l[0] * l[0] + l[2] * l[2] - 2.0 * l[0] * l[2] * c02 - d02 * d02,
            l[1] * l[1] + l[2] * l[2] - 2.0 * l[1] * l[2] * c12 - d12 * d12,
        };
        double J[9] = {
            2 * l[0] - 2 * l[1] * c01, 2 * l[1] - 2 * l[0] * c01, 0,
            2 * l[0] - 2 * l[2] * c02, 0, 2 * l[2] - 2 * l[0] * c02,
            0, 2 * l[1] - 2 * l[2] * c12, 2 * l[2] - 2 * l[1] * c12,
        };
        double A[9] = {0}, b[3] = {0};
        for (int row = 0; row < 3; row++) {
            for (int a = 0; a < 3; a++) {
                b[a] -= J[row * 3 + a] * r[row];
                for (int c = 0; c < 3; c++)
                    A[a * 3 + c] += J[row * 3 + a] * J[row * 3 + c];
            }
        }
        for (int k = 0; k < 3; k++)
            A[k * 3 + k] += 1e-6;
        double dl[3];
        if (!solve_3x3(A, b, dl))
            return 0;
        double step2 = dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2];
        for (int k = 0; k < 3; k++) {
            l[k] += dl[k];
            if (l[k] <= 1e-6 || !isfinite(l[k]))
                return 0;
        }
        if (step2 < 1e-12)
            break;
    }
    double err = fabs(l[0] * l[0] + l[1] * l[1] - 2.0 * l[0] * l[1] * c01 - d01 * d01) +
                 fabs(l[0] * l[0] + l[2] * l[2] - 2.0 * l[0] * l[2] * c02 - d02 * d02) +
                 fabs(l[1] * l[1] + l[2] * l[2] - 2.0 * l[1] * l[2] * c12 - d12 * d12);
    double scale = d01 * d01 + d02 * d02 + d12 * d12 + 1e-9;
    if (err / scale > 1e-4)
        return 0;
    out_lam[0] = l[0];
    out_lam[1] = l[1];
    out_lam[2] = l[2];
    return 1;
}

static int p3p_numeric_pose_from_sample(const Map *map, const CornerVec *corners,
                                        const int sample_ids[4], double fx, double fy,
                                        double cx, double cy, Pose *best_pose) {
    double f[3][3], Pw[3][3];
    for (int i = 0; i < 3; i++) {
        int idx = sample_ids[i];
        MapPoint p = map->data[corners->data[idx].pt_idx];
        Pw[i][0] = p.x;
        Pw[i][1] = p.y;
        Pw[i][2] = p.z;
        f[i][0] = (corners->data[idx].x - cx) / fx;
        f[i][1] = (corners->data[idx].y - cy) / fy;
        f[i][2] = 1.0;
        vec3_normalize(f[i]);
    }
    double side = (vec3_dist(Pw[0], Pw[1]) + vec3_dist(Pw[0], Pw[2]) +
                   vec3_dist(Pw[1], Pw[2])) / 3.0;
    if (!isfinite(side) || side < 1e-9)
        return 0;
    const double scales[] = {0.35, 0.7, 1.0, 1.4, 2.5, 5.0};
    const double ratios[][3] = {
        {1, 1, 1}, {0.7, 1, 1.3}, {1.3, 1, 0.7}, {1, 0.7, 1.3}, {1, 1.3, 0.7},
    };
    int found = 0;
    double best_err = 1e300;
    for (int si = 0; si < (int)(sizeof(scales) / sizeof(scales[0])); si++) {
        for (int ri = 0; ri < (int)(sizeof(ratios) / sizeof(ratios[0])); ri++) {
            double seed[3] = {
                side * scales[si] * ratios[ri][0],
                side * scales[si] * ratios[ri][1],
                side * scales[si] * ratios[ri][2],
            };
            double lam[3];
            if (!numeric_p3p_depths(f, Pw, seed, lam))
                continue;
            double Pc[3][3];
            for (int i = 0; i < 3; i++) {
                Pc[i][0] = lam[i] * f[i][0];
                Pc[i][1] = lam[i] * f[i][1];
                Pc[i][2] = lam[i] * f[i][2];
            }
            Pose pose;
            if (!pose_from_3d3d(Pw, Pc, &pose))
                continue;
            int idx4 = sample_ids[3];
            MapPoint p4 = map->data[corners->data[idx4].pt_idx];
            double x = pose.m[0] * p4.x + pose.m[1] * p4.y + pose.m[2] * p4.z + pose.m[3];
            double y = pose.m[4] * p4.x + pose.m[5] * p4.y + pose.m[6] * p4.z + pose.m[7];
            double z = pose.m[8] * p4.x + pose.m[9] * p4.y + pose.m[10] * p4.z + pose.m[11];
            if (z < 0.1)
                continue;
            double u = fx * x / z + cx, v = fy * y / z + cy;
            double du = u - corners->data[idx4].x, dv = v - corners->data[idx4].y;
            double err = du * du + dv * dv;
            if (err < best_err) {
                best_err = err;
                *best_pose = pose;
                found = 1;
            }
        }
    }
    return found;
}

static void sample_unique4(const int *ids, int n, int out[4]) {
    for (int i = 0; i < 4; i++) {
        int pick, duplicate;
        do {
            duplicate = 0;
            pick = ids[rand() % n];
            for (int j = 0; j < i; j++) {
                if (out[j] == pick) {
                    duplicate = 1;
                    break;
                }
            }
        } while (duplicate);
        out[i] = pick;
    }
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

static PnPScore score_pnp_pose(const Map *map, const CornerVec *corners, double fx, double fy,
                               double cx, double cy, const Pose *pose) {
    PnPScore score = {0};
    if (corners->size <= 0)
        return score;
    double *errors = malloc((size_t)corners->size * sizeof(double));
    int nerr = 0, pos = 0, total = 0;
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
            continue;
        total++;
        MapPoint p = map->data[corners->data[i].pt_idx];
        double x = pose->m[0] * p.x + pose->m[1] * p.y + pose->m[2] * p.z + pose->m[3];
        double y = pose->m[4] * p.x + pose->m[5] * p.y + pose->m[6] * p.z + pose->m[7];
        double z = pose->m[8] * p.x + pose->m[9] * p.y + pose->m[10] * p.z + pose->m[11];
        if (z > 0.1)
            pos++;
        if (z < 0.1)
            continue;
        double u = fx * x / z + cx, v = fy * y / z + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        double err = sqrt(dx * dx + dy * dy);
        errors[nerr++] = err;
        if (err < 2.0)
            score.inliers2++;
        if (err < 3.0)
            score.inliers3++;
        if (err < 5.0)
            score.inliers5++;
    }
    if (nerr > 0) {
        qsort(errors, (size_t)nerr, sizeof(double), cmp_double);
        score.median_error = errors[nerr / 2];
    }
    score.positive_depth_ratio = total > 0 ? (double)pos / (double)total : 0.0;
    free(errors);
    return score;
}

static int count_pose_inliers(const Map *map, const CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose);

static int estimate_pose_PnP_p3p_numeric(const Map *map, const CornerVec *corners, double fx,
                                         double fy, double cx, double cy, const Pose *predicted,
                                         int baseline_inliers, int iterations, double max_jump,
                                         int min_inl2, int min_gain, double max_mederr,
                                         double min_posz,
                                         Pose *out_pose, int *out_inl, PnPProbe *probe) {
    if (out_inl)
        *out_inl = 0;
    if (probe)
        memset(probe, 0, sizeof(*probe));
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1 && map->data[corners->data[i].pt_idx].obs > 0)
            n++;
    if (n < 12)
        return 0;
    if (probe)
        probe->attempted = 1;
    int *ids = malloc((size_t)n * sizeof(int));
    int k = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1 && map->data[corners->data[i].pt_idx].obs > 0)
            ids[k++] = i;

    Pose best_pose;
    int best_inl = 0;
    int iters = iterations > 0 ? iterations : 500;
    srand(0);
    for (int it = 0; it < iters; it++) {
        int sample[4];
        sample_unique4(ids, n, sample);
        Pose pose;
        if (!p3p_numeric_pose_from_sample(map, corners, sample, fx, fy, cx, cy, &pose))
            continue;
        int inl = count_pose_inliers(map, corners, fx, fy, cx, cy, &pose);
        if (inl > best_inl) {
            best_inl = inl;
            best_pose = pose;
        }
        if (inl > n * 0.8)
            break;
    }
    free(ids);
    if (out_inl)
        *out_inl = best_inl;
    if (probe)
        probe->inliers = best_inl;
    if (best_inl < 16)
        return 0;

    PnPScore score = score_pnp_pose(map, corners, fx, fy, cx, cy, &best_pose);
    if (probe) {
        probe->solved = 1;
        probe->inliers2 = score.inliers2;
        probe->inliers3 = score.inliers3;
        probe->inliers5 = score.inliers5;
        probe->median_error = score.median_error;
        probe->positive_depth_ratio = score.positive_depth_ratio;
        camera_center_from_pose(&best_pose, probe->center);
    }
    double predicted_jump = 0.0;
    if (predicted) {
        double a[3], b[3];
        camera_center_from_pose(predicted, a);
        camera_center_from_pose(&best_pose, b);
        predicted_jump = vec3_dist(a, b);
        if (probe)
            probe->predicted_jump = predicted_jump;
    }
    if (score.inliers2 < min_inl2 || score.inliers2 - baseline_inliers < min_gain ||
        score.median_error > max_mederr || score.positive_depth_ratio < min_posz)
        return 0;
    if (predicted && max_jump > 0.0 && predicted_jump > max_jump)
        return 0;
    *out_pose = best_pose;
    if (probe)
        probe->accepted = 1;
    return 1;
}

static int count_pose_inliers(const Map *map, const CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    int inl = 0;
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
            continue;
        MapPoint p = map->data[corners->data[i].pt_idx];
        double px = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0];
        double py = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1];
        double pz = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (pz < 0.1)
            continue;
        double u = fx * px / pz + cx, v = fy * py / pz + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        if (dx * dx + dy * dy < 4.0)
            inl++;
    }
    return inl;
}

static int gate_links_by_pose(const Map *map, CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose, double gate_px) {
    if (gate_px <= 0.0)
        return 0;
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double th2 = gate_px * gate_px;
    int removed = 0;
    for (int i = 0; i < corners->size; i++) {
        int pi = corners->data[i].pt_idx;
        if (pi < 0 || pi >= map->size)
            continue;
        MapPoint p = map->data[pi];
        double x = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
               y = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
               z = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (z < 0.1) {
            corners->data[i].pt_idx = -1;
            removed++;
            continue;
        }
        double u = fx * x / z + cx, v = fy * y / z + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        if (dx * dx + dy * dy > th2) {
            corners->data[i].pt_idx = -1;
            removed++;
        }
    }
    return removed;
}

static double reprojection_error_point(const MapPoint *p, const Corner *corner, double fx,
                                       double fy, double cx, double cy, const Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double x = R[0] * p->x + R[1] * p->y + R[2] * p->z + t[0],
           y = R[3] * p->x + R[4] * p->y + R[5] * p->z + t[1],
           z = R[6] * p->x + R[7] * p->y + R[8] * p->z + t[2];
    if (z < 0.1)
        return INFINITY;
    double u = fx * x / z + cx, v = fy * y / z + cy;
    double dx = u - corner->x, dy = v - corner->y;
    return sqrt(dx * dx + dy * dy);
}

static void update_observation_stats(Map *map, const CornerVec *corners, double fx, double fy,
                                     double cx, double cy, const Pose *pose, double gate_px) {
    if (gate_px <= 0.0)
        return;
    for (int i = 0; i < corners->size; i++) {
        int pi = corners->data[i].pt_idx;
        if (pi < 0 || pi >= map->size)
            continue;
        double err = reprojection_error_point(&map->data[pi], &corners->data[i], fx, fy, cx, cy, pose);
        if (err <= gate_px) {
            if (map->data[pi].good_obs < 65535)
                map->data[pi].good_obs++;
        } else {
            if (map->data[pi].bad_obs < 65535)
                map->data[pi].bad_obs++;
        }
    }
}

static unsigned char *build_observation_stat_mask(const Map *map, int min_good,
                                                  double max_bad_ratio) {
    if (min_good <= 0 && max_bad_ratio >= 1.0)
        return NULL;
    unsigned char *ok = malloc((size_t)map->size);
    if (!ok)
        return NULL;
    for (int i = 0; i < map->size; i++) {
        int total = (int)map->data[i].good_obs + (int)map->data[i].bad_obs;
        double bad_ratio = total > 0 ? (double)map->data[i].bad_obs / (double)total : 1.0;
        ok[i] = map->data[i].good_obs >= min_good && bad_ratio <= max_bad_ratio;
    }
    return ok;
}

static double pose_reprojection_rmse(const Map *map, const CornerVec *corners, double fx,
                                     double fy, double cx, double cy, const Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
            continue;
        MapPoint p = map->data[corners->data[i].pt_idx];
        double px = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0];
        double py = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1];
        double pz = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (pz < 0.1)
            continue;
        double u = fx * px / pz + cx, v = fy * py / pz + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        sum += dx * dx + dy * dy;
        n++;
    }
    return n ? sqrt(sum / n) : 0.0;
}

static void dump_pnp_frame(FILE *f, int frame_id, const Map *map, const CornerVec *corners,
                           double fx, double fy, double cx, double cy, const Pose *predicted,
                           const Pose *pnp_pose, int pnp_inliers, const Pose *lm_pose) {
    if (!f)
        return;
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1 && map->data[corners->data[i].pt_idx].obs > 0)
            n++;

    double pred_t[3], pnp_t[3] = {0}, lm_t[3] = {0};
    pose_get_translation(predicted, pred_t);
    if (pnp_pose)
        pose_get_translation(pnp_pose, pnp_t);
    if (lm_pose)
        pose_get_translation(lm_pose, lm_t);
    double pnp_rmse = pnp_pose ? pose_reprojection_rmse(map, corners, fx, fy, cx, cy, pnp_pose) : 0.0;
    double lm_rmse = lm_pose ? pose_reprojection_rmse(map, corners, fx, fy, cx, cy, lm_pose) : 0.0;

    fprintf(f,
            "FRAME %d %.17g %.17g %.17g %.17g %d %.17g %.17g %.17g %d %d %.17g %.17g %.17g %.17g "
            "%.17g %.17g %.17g %.17g\n",
            frame_id, fx, fy, cx, cy, n, pred_t[0], pred_t[1], pred_t[2], pnp_pose ? 1 : 0,
            pnp_inliers, pnp_t[0], pnp_t[1], pnp_t[2], pnp_rmse, lm_t[0], lm_t[1], lm_t[2],
            lm_rmse);
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs <= 0)
            continue;
        MapPoint p = map->data[corners->data[i].pt_idx];
        fprintf(f, "C %.9g %.9g %.17g %.17g %.17g %d\n", corners->data[i].x,
                corners->data[i].y, p.x, p.y, p.z, p.obs);
    }
    fprintf(f, "END\n");
}

static void refine_pose_lm(const Map *map, const CornerVec *corners, double fx, double fy,
                           double cx, double cy, int max_iters, Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double lambda = 1e-3;
    int iters = max_iters > 0 ? max_iters : 10;
    for (int iter = 0; iter < iters; iter++) {
        double H[36] = {0}, b[6] = {0};
        for (int i = 0; i < corners->size; i++) {
            if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
                continue;
            MapPoint p = map->data[corners->data[i].pt_idx];

            // Project point P into camera frame:  cp = R·P + t
            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            if (cp[2] < 0.1)
                continue;

            // Pinhole projection and reprojection residual:
            double inv_z  = 1.0 / cp[2];
            double inv_z2 = inv_z * inv_z;
            double u  = fx*cp[0]*inv_z + cx;
            double v  = fy*cp[1]*inv_z + cy;
            double du = corners->data[i].x - u;
            double dv = corners->data[i].y - v;

            // Huber-ish weight on reprojection error.
            double err2   = du*du + dv*dv;
            double weight = (err2 > 4.0) ? 2.0 / sqrt(err2) : 1.0;
            double J[2][6] = {
                {fx*inv_z, 0,        -fx*cp[0]*inv_z2,  -fx*cp[0]*cp[1]*inv_z2,             fx*(1 + cp[0]*cp[0]*inv_z2),  -fx*cp[1]*inv_z},
                {0,        fy*inv_z, -fy*cp[1]*inv_z2,  -fy*(1 + cp[1]*cp[1]*inv_z2),       fy*cp[0]*cp[1]*inv_z2,         fy*cp[0]*inv_z},
            };
            for (int r = 0; r < 6; r++) {
                b[r] += weight * (J[0][r] * du + J[1][r] * dv);
                for (int c = 0; c < 6; c++)
                    H[r * 6 + c] += weight * (J[0][r] * J[0][c] + J[1][r] * J[1][c]);
            }
        }

        for (int i = 0; i < 6; i++)
            H[i * 6 + i] += lambda * H[i * 6 + i] + 1e-6;
        double dx[6];
        if (!solve_6x6(H, b, dx))
            break;
        t[0] += dx[0];
        t[1] += dx[1];
        t[2] += dx[2];
        double dr[9] = {1, -dx[5], dx[4],
                        dx[5], 1, -dx[3],
                        -dx[4], dx[3], 1};
        double Rn[9];
        mat3_mul(dr, R, Rn);
        double W[3], U[9], V[9], VT[9];
        svd_3x3(Rn, W, U, V);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                VT[i * 3 + j] = V[j * 3 + i];
        mat3_mul(U, VT, R);
        if (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2] < 1e-8)
            break;
    }
    pose_from_rt(R, t, pose);
}

typedef struct {
    FILE *pipe;
    int w, h;
    int bytes_per_frame;
} FFmpegCap;
static FFmpegCap *ffmpeg_open(const char *p, int w, int h, int gray) {
    FFmpegCap *c = malloc(sizeof(FFmpegCap));
    c->w = w;
    c->h = h;
    c->bytes_per_frame = w * h * (gray ? 1 : 3);
    char cmd[1024];
    snprintf(
        cmd, 1024,
        "ffmpeg -hide_banner -loglevel error -i \"%s\" -f rawvideo -pix_fmt %s -s %dx%d -",
        p, gray ? "gray" : "bgr24", w, h);
    c->pipe = popen(cmd, "r");
    if (!c->pipe) {
        free(c);
        return NULL;
    }
    return c;
}
static int ffmpeg_read(FFmpegCap *c, unsigned char *b) {
    return fread(b, 1, (size_t)c->bytes_per_frame, c->pipe) == (size_t)c->bytes_per_frame;
}
static void ffmpeg_close(FFmpegCap *c) {
    if (c) {
        pclose(c->pipe);
        free(c);
    }
}
static void bgr_to_gray(const unsigned char *b, int w, int h, unsigned char *g) {
    for (int i = 0; i < w * h; i++)
        g[i] = (unsigned char)(0.299f * b[i * 3 + 2] + 0.587f * b[i * 3 + 1] + 0.114f * b[i * 3]);
}
static void downsample2x(const unsigned char *src, int sw, int sh, unsigned char *dst) {
    int dw = sw / 2, dh = sh / 2;
    for (int y = 0; y < dh; y++)
        for (int x = 0; x < dw; x++)
            dst[y * dw + x] = src[(y * 2) * sw + (x * 2)];
}

static float get_pixel_bilinear(const unsigned char *g, int w, int h, float x, float y) {
    int ix = (int)x, iy = (int)y;
    float dx = x - ix, dy = y - iy;
    if (ix < 0 || ix >= w - 1 || iy < 0 || iy >= h - 1)
        return 0;
    return (1 - dx) * (1 - dy) * g[iy * w + ix] + dx * (1 - dy) * g[iy * w + ix + 1] +
           (1 - dx) * dy * g[(iy + 1) * w + ix] + dx * dy * g[(iy + 1) * w + ix + 1];
}

static void blur_3x3(const unsigned char *src, int w, int h, unsigned char *dst) {
#pragma omp parallel for collapse(2)
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            int s = src[(y - 1) * w + x - 1] + 2 * src[(y - 1) * w + x] + src[(y - 1) * w + x + 1] +
                    2 * src[y * w + x - 1] + 4 * src[y * w + x] + 2 * src[y * w + x + 1] +
                    src[(y + 1) * w + x - 1] + 2 * src[(y + 1) * w + x] + src[(y + 1) * w + x + 1];
            dst[y * w + x] = (unsigned char)(s >> 4);
        }
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
                                 int distributed, int pyramid) {
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
        select_corner_candidates(cand, cand_size, w, h, c, max, 0, 1);
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

static void extract_features_pure(const Config *cfg, const unsigned char *g, int w, int h,
                                  CornerVec *c, int max) {
    if (cfg->fast_corners)
        extract_corners_fast(g, w, h, c, max, cfg->distributed_features);
    else
        extract_corners_pure(g, w, h, c, max, cfg->distributed_features, cfg->pyramid_features);
}

typedef struct {
    unsigned char *p_g1, *c_g1, *p_g2, *c_g2, *p_g3, *c_g3;
    size_t pyr_cap;
    Corner *res;
    int *ok;
    int point_cap;
} LKScratch;

static void lk_scratch_reserve(LKScratch *s, int w, int h, int npoints) {
    size_t pyr_need = (size_t)(w / 2) * (h / 2);
    if (pyr_need > s->pyr_cap) {
        s->p_g1 = (unsigned char *)xrealloc(s->p_g1, pyr_need);
        s->c_g1 = (unsigned char *)xrealloc(s->c_g1, pyr_need);
        s->p_g2 = (unsigned char *)xrealloc(s->p_g2, pyr_need);
        s->c_g2 = (unsigned char *)xrealloc(s->c_g2, pyr_need);
        s->p_g3 = (unsigned char *)xrealloc(s->p_g3, pyr_need);
        s->c_g3 = (unsigned char *)xrealloc(s->c_g3, pyr_need);
        s->pyr_cap = pyr_need;
    }
    if (npoints > s->point_cap) {
        s->res = (Corner *)xrealloc(s->res, (size_t)npoints * sizeof(Corner));
        s->ok = (int *)xrealloc(s->ok, (size_t)npoints * sizeof(int));
        s->point_cap = npoints;
    }
}

static void lk_scratch_free(LKScratch *s) {
    free(s->p_g1);
    free(s->c_g1);
    free(s->p_g2);
    free(s->c_g2);
    free(s->p_g3);
    free(s->c_g3);
    free(s->res);
    free(s->ok);
}

static void track_corners_pure_lk(const unsigned char *p_g, const unsigned char *c_g, int w, int h,
                                  const CornerVec *p_pts, CornerVec *c_pts, MatchVec *m,
                                  LKScratch *scratch, int lk_iters, int lk_back_iters) {
    lk_scratch_reserve(scratch, w, h, p_pts->size);
    unsigned char *p_g1 = scratch->p_g1, *c_g1 = scratch->c_g1,
                  *p_g2 = scratch->p_g2, *c_g2 = scratch->c_g2,
                  *p_g3 = scratch->p_g3, *c_g3 = scratch->c_g3;
    downsample2x(p_g, w, h, p_g1);
    downsample2x(c_g, w, h, c_g1);
    downsample2x(p_g1, w / 2, h / 2, p_g2);
    downsample2x(c_g1, w / 2, h / 2, c_g2);
    downsample2x(p_g2, w / 4, h / 4, p_g3);
    downsample2x(c_g2, w / 4, h / 4, c_g3);
    Corner *res = scratch->res;
    int *ok = scratch->ok;
    memset(ok, 0, (size_t)p_pts->size * sizeof(int));
#pragma omp parallel for
    for (int i = 0; i < p_pts->size; i++) {
        Corner p = p_pts->data[i];
        float dx = 0, dy = 0;
        for (int level = 3; level >= 0; level--) {
            const unsigned char *pg, *cg;
            int lw, lh;
            float sc;
            if (level == 3) {
                pg = p_g3;
                cg = c_g3;
                lw = w / 8;
                lh = h / 8;
                sc = 0.125f;
            } else if (level == 2) {
                pg = p_g2;
                cg = c_g2;
                lw = w / 4;
                lh = h / 4;
                sc = 0.25f;
            } else if (level == 1) {
                pg = p_g1;
                cg = c_g1;
                lw = w / 2;
                lh = h / 2;
                sc = 0.5f;
            } else {
                pg = p_g;
                cg = c_g;
                lw = w;
                lh = h;
                sc = 1.0f;
            }
            float lx = p.x * sc, ly = p.y * sc;
            float l_dx = dx * sc, l_dy = dy * sc;
            for (int it = 0; it < lk_iters; it++) {
                float G[4] = {0}, b[2] = {0};
                for (int y = -3; y <= 3; y++)
                    for (int x = -3; x <= 3; x++) {
                        float cur_x = lx + x, cur_y = ly + y, nxt_x = lx + l_dx + x,
                              nxt_y = ly + l_dy + y;
                        if (cur_x < 1 || cur_x >= lw - 1 || cur_y < 1 || cur_y >= lh - 1 ||
                            nxt_x < 1 || nxt_x >= lw - 1 || nxt_y < 1 || nxt_y >= lh - 1)
                            continue;
                        float Ix = (get_pixel_bilinear(pg, lw, lh, cur_x + 1, cur_y) -
                                    get_pixel_bilinear(pg, lw, lh, cur_x - 1, cur_y)) *
                                   0.5f;
                        float Iy = (get_pixel_bilinear(pg, lw, lh, cur_x, cur_y + 1) -
                                    get_pixel_bilinear(pg, lw, lh, cur_x, cur_y - 1)) *
                                   0.5f;
                        float It = get_pixel_bilinear(cg, lw, lh, nxt_x, nxt_y) -
                                   get_pixel_bilinear(pg, lw, lh, cur_x, cur_y);
                        G[0] += Ix * Ix;
                        G[1] += Ix * Iy;
                        G[3] += Iy * Iy;
                        b[0] -= Ix * It;
                        b[1] -= Iy * It;
                    }
                G[2] = G[1];
                float det = G[0] * G[3] - G[1] * G[2];
                if (fabs(det) < 1e-6)
                    break;
                float vx = (G[3] * b[0] - G[1] * b[1]) / det,
                      vy = (G[0] * b[1] - G[2] * b[0]) / det;
                l_dx += vx;
                l_dy += vy;
                if (vx * vx + vy * vy < 1e-6)
                    break;
            }
            dx = l_dx / sc;
            dy = l_dy / sc;
        }
        float back_dx = -dx, back_dy = -dy;
        for (int it = 0; it < lk_back_iters; it++) {
            float G[4] = {0}, b[2] = {0};
            for (int y = -3; y <= 3; y++)
                for (int x = -3; x <= 3; x++) {
                    float cur_x = p.x + dx + x, cur_y = p.y + dy + y,
                          nxt_x = p.x + dx + back_dx + x, nxt_y = p.y + dy + back_dy + y;
                    if (cur_x < 1 || cur_x >= w - 1 || cur_y < 1 || cur_y >= h - 1 || nxt_x < 1 ||
                        nxt_x >= w - 1 || nxt_y < 1 || nxt_y >= h - 1)
                        continue;
                    float Ix = (get_pixel_bilinear(c_g, w, h, cur_x + 1, cur_y) -
                                get_pixel_bilinear(c_g, w, h, cur_x - 1, cur_y)) *
                               0.5f;
                    float Iy = (get_pixel_bilinear(c_g, w, h, cur_x, cur_y + 1) -
                                get_pixel_bilinear(c_g, w, h, cur_x, cur_y - 1)) *
                               0.5f;
                    float It = get_pixel_bilinear(p_g, w, h, nxt_x, nxt_y) -
                               get_pixel_bilinear(c_g, w, h, cur_x, cur_y);
                    G[0] += Ix * Ix;
                    G[1] += Ix * Iy;
                    G[3] += Iy * Iy;
                    b[0] -= Ix * It;
                    b[1] -= Iy * It;
                }
            G[2] = G[1];
            float det = G[0] * G[3] - G[1] * G[2];
            if (fabs(det) < 1e-6)
                break;
            float vx = (G[3] * b[0] - G[1] * b[1]) / det, vy = (G[0] * b[1] - G[2] * b[0]) / det;
            back_dx += vx;
            back_dy += vy;
        }
        float fb_err = (dx + back_dx) * (dx + back_dx) + (dy + back_dy) * (dy + back_dy);
        float tx = p.x + dx, ty = p.y + dy;
        if (fb_err < 0.5f && tx >= 2 && tx < w - 2 && ty >= 2 && ty < h - 2) {
            res[i] = (Corner){tx, ty, p.pt_idx, p.cand_idx, fb_err, sqrtf(dx * dx + dy * dy)};
            ok[i] = 1;
        }
    }
    for (int i = 0; i < p_pts->size; i++)
        if (ok[i]) {
            match_vec_push(m, (Match){i, c_pts->size, 0});
            corner_vec_push(c_pts, res[i]);
        }
}

typedef struct {
    unsigned char thumb[256];
    int frame_id;
    Pose pose;
    CornerVec corners;
    ImageGray gray;
} KFEntry;
typedef struct {
    KFEntry *data;
    int size, cap;
} KFDB;
static void kfdb_push(KFDB *db, KFEntry e) {
    if (db->size == db->cap) {
        db->cap = db->cap ? db->cap * 2 : 64;
        db->data = xrealloc(db->data, db->cap * sizeof(KFEntry));
    }
    db->data[db->size++] = e;
}
static void kfentry_free(KFEntry *e) {
    free(e->corners.data);
    image_gray_free(&e->gray);
    memset(e, 0, sizeof(*e));
}
static void kfdb_free(KFDB *db) {
    for (int i = 0; i < db->size; i++)
        kfentry_free(&db->data[i]);
    free(db->data);
    memset(db, 0, sizeof(*db));
}
static void gen_thumbnail(const unsigned char *g, int w, int h, unsigned char *out) {
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 16; x++)
            out[y * 16 + x] = g[(y * h / 16) * w + (x * w / 16)];
}
static int find_loop_candidate(KFDB *db, const unsigned char *thumb, int current_id) {
    int bi = -1;
    uint32_t bs = 0xFFFFFFFF;
    for (int i = 0; i < db->size - 50; i++) {
        uint32_t s = 0;
        for (int j = 0; j < 256; j++)
            s += abs(thumb[j] - db->data[i].thumb[j]);
        if (s < bs) {
            bs = s;
            bi = i;
        }
    }
    return (bs < 3500) ? bi : -1;
}

static int verify_loop(KFDB *db, int lidx, const unsigned char *cgray, int w, int h, double fx,
                       double fy, double cx, double cy, Pose *out_pose) {
    KFEntry *kf = &db->data[lidx];
    MatchVec matches = {0};
    CornerVec cpts = {0};
    for (int i = 0; i < kf->corners.size; i += 2) {
        Corner p = kf->corners.data[i];
        float best_ncc = -1;
        int best_x = -1, best_y = -1;
        for (int y = (int)p.y - 20; y < (int)p.y + 20; y += 2)
            for (int x = (int)p.x - 20; x < (int)p.x + 20; x += 2) {
                if (x < 4 || x >= w - 4 || y < 4 || y >= h - 4)
                    continue;
                float ncc = 0, s1 = 0, s2 = 0;
                for (int dy = -3; dy <= 3; dy++)
                    for (int dx = -3; dx <= 3; dx++) {
                        float v1 = kf->gray.data[((int)p.y + dy) * w + (int)p.x + dx],
                              v2 = cgray[(y + dy) * w + x + dx];
                        ncc += v1 * v2;
                        s1 += v1 * v1;
                        s2 += v2 * v2;
                    }
                ncc /= sqrt(s1 * s2 + 1e-6);
                if (ncc > best_ncc) {
                    best_ncc = ncc;
                    best_x = x;
                    best_y = y;
                }
            }
        if (best_ncc > 0.95) {
            match_vec_push(&matches, (Match){i, cpts.size, 0});
            corner_vec_push(&cpts, (Corner){(float)best_x, (float)best_y, -1, -1, 0.0f, 0.0f});
        }
    }
    Pose rel;
    unsigned char *mask;
    int inl;
    int ok = estimate_pose_E(&kf->corners, &cpts, &matches, fx, fy, cx, cy, &rel, 500, 32,
                             &mask, &inl);
    if (ok && inl > 20) {
        pose_compose_relative(&rel, &kf->pose, out_pose);
        free(mask);
        free(matches.data);
        free(cpts.data);
        return 1;
    }
    free(matches.data);
    free(cpts.data);
    return 0;
}

static void refine_map_point_against_kfs(Map *map, int i, KFDB *db, int k_start, int k_end,
                                         double fx, double fy, double cx, double cy) {
    if (map->data[i].obs < 2)
        return;
    double H[9] = {0}, b[3] = {0};
    for (int k = k_start; k < k_end; k++) {
        KFEntry *kf = &db->data[k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            if (kf->corners.data[j].pt_idx != i)
                continue;
            MapPoint p = map->data[i];

            // Project P into camera k:  cp = R·P + t
            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            if (cp[2] < 0.1)
                continue;

            double inv_z  = 1.0 / cp[2];
            double inv_z2 = inv_z * inv_z;
            double u  = fx*cp[0]*inv_z + cx;
            double v  = fy*cp[1]*inv_z + cy;
            double du = kf->corners.data[j].x - u;
            double dv = kf->corners.data[j].y - v;

            // J[2][3] = ∂(u,v)/∂P  (3-DoF Jacobian wrt world point):
            double J[2][3] = {
                {fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2,  fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2,  fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2},
                {fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2,  fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2,  fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2},
            };
            for (int r = 0; r < 3; r++) {
                b[r] += J[0][r] * du + J[1][r] * dv;
                for (int c = 0; c < 3; c++)
                    H[r * 3 + c] += J[0][r] * J[0][c] + J[1][r] * J[1][c];
            }
        }
    }
    for (int r = 0; r < 3; r++)
        H[r * 3 + r] += 1e-4;
    double det = H[0] * (H[4] * H[8] - H[5] * H[7]) - H[1] * (H[3] * H[8] - H[5] * H[6]) +
                 H[2] * (H[3] * H[7] - H[4] * H[6]);
    if (fabs(det) > 1e-9) {
        double dx = (b[0]*(H[4]*H[8] - H[5]*H[7]) - H[1]*(b[1]*H[8] - H[5]*b[2]) + H[2]*(b[1]*H[7] - H[4]*b[2])) / det;
        double dy = (H[0]*(b[1]*H[8] - H[5]*b[2]) - b[0]*(H[3]*H[8] - H[5]*H[6]) + H[2]*(H[3]*b[2] - b[1]*H[6])) / det;
        double dz = (H[0]*(H[4]*b[2] - b[1]*H[7]) - H[1]*(H[3]*b[2] - b[1]*H[6]) + b[0]*(H[3]*H[7] - H[4]*H[6])) / det;
        map->data[i].x += dx;
        map->data[i].y += dy;
        map->data[i].z += dz;
    }
}

static unsigned char *build_pnp_quality_mask(const KFDB *db, const Map *map, double fx, double fy,
                                             double cx, double cy, double gate_px, int min_obs,
                                             int window) {
    if (gate_px <= 0.0 || map->size <= 0 || db->size <= 0)
        return NULL;
    if (min_obs < 1)
        min_obs = 1;
    if (window < 1 || window > db->size)
        window = db->size;
    unsigned char *ok = malloc((size_t)map->size);
    unsigned short *total = calloc((size_t)map->size, sizeof(unsigned short));
    unsigned short *bad = calloc((size_t)map->size, sizeof(unsigned short));
    if (!ok || !total || !bad) {
        free(ok);
        free(total);
        free(bad);
        return NULL;
    }
    memset(ok, 1, (size_t)map->size);
    double th2 = gate_px * gate_px;
    int start = db->size - window;
    if (start < 0)
        start = 0;
    for (int k = start; k < db->size; k++) {
        const KFEntry *kf = &db->data[k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || pi >= map->size)
                continue;
            MapPoint p = map->data[pi];
            double x = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
                   y = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
                   z = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
            total[pi]++;
            if (z < 0.1) {
                bad[pi]++;
                continue;
            }
            double u = fx * x / z + cx, v = fy * y / z + cy;
            double du = kf->corners.data[j].x - u, dv = kf->corners.data[j].y - v;
            if (du * du + dv * dv > th2)
                bad[pi]++;
        }
    }
    for (int pi = 0; pi < map->size; pi++)
        if (total[pi] >= min_obs && bad[pi] == total[pi])
            ok[pi] = 0;
    free(total);
    free(bad);
    return ok;
}

static void global_ba(KFDB *db, Map *map, double fx, double fy, double cx, double cy,
                      int iters, int pose_lm_iters) {
    if (db->size < 3)
        return;
    for (int iter = 0; iter < iters; iter++) {
#pragma omp parallel for
        for (int k = 1; k < db->size; k++)
            refine_pose_lm(map, &db->data[k].corners, fx, fy, cx, cy, pose_lm_iters,
                           &db->data[k].pose);
#pragma omp parallel for
        for (int i = 0; i < map->size; i++)
            refine_map_point_against_kfs(map, i, db, 0, db->size, fx, fy, cx, cy);
    }
}

static void local_ba(KFDB *db, Map *map, double fx, double fy, double cx, double cy,
                     int pose_lm_iters) {
    if (db->size < 3)
        return;
    for (int iter = 0; iter < 5; iter++) {

        for (int k = db->size - 3; k < db->size; k++) {
            refine_pose_lm(map, &db->data[k].corners, fx, fy, cx, cy, pose_lm_iters,
                           &db->data[k].pose);
        }
        for (int i = 0; i < map->size; i++) {
            if (map->data[i].obs < 2)
                continue;
            double H[9] = {0}, b[3] = {0};
            for (int k = db->size - 3; k < db->size; k++) {
                KFEntry *kf = &db->data[k];
                double R[9], t[3];
                pose_get_rotation(&kf->pose, R);
                pose_get_translation(&kf->pose, t);
                for (int j = 0; j < kf->corners.size; j++) {
                    if (kf->corners.data[j].pt_idx != i)
                        continue;
                    MapPoint p = map->data[i];

                    // Project P into camera k:  cp = R·P + t
                    double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                                    R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                                    R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
                    if (cp[2] < 0.1)
                        continue;

                    double inv_z  = 1.0 / cp[2];
                    double inv_z2 = inv_z * inv_z;
                    double u  = fx*cp[0]*inv_z + cx;
                    double v  = fy*cp[1]*inv_z + cy;
                    double du = kf->corners.data[j].x - u;
                    double dv = kf->corners.data[j].y - v;

                    // J[2][3] = ∂(u,v)/∂P
                    double J[2][3] = {
                        {fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2,  fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2,  fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2},
                        {fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2,  fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2,  fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2},
                    };
                    for (int r = 0; r < 3; r++) {
                        b[r] += J[0][r] * du + J[1][r] * dv;
                        for (int c = 0; c < 3; c++)
                            H[r * 3 + c] += J[0][r] * J[0][c] + J[1][r] * J[1][c];
                    }
                }
            }
            for (int r = 0; r < 3; r++)
                H[r * 3 + r] += 1e-4;
            double det = H[0] * (H[4] * H[8] - H[5] * H[7]) - H[1] * (H[3] * H[8] - H[5] * H[6]) +
                         H[2] * (H[3] * H[7] - H[4] * H[6]);
            if (fabs(det) > 1e-9) {
                double dx = (b[0]*(H[4]*H[8] - H[5]*H[7]) - H[1]*(b[1]*H[8] - H[5]*b[2]) + H[2]*(b[1]*H[7] - H[4]*b[2])) / det;
                double dy = (H[0]*(b[1]*H[8] - H[5]*b[2]) - b[0]*(H[3]*H[8] - H[5]*H[6]) + H[2]*(H[3]*b[2] - b[1]*H[6])) / det;
                double dz = (H[0]*(H[4]*b[2] - b[1]*H[7]) - H[1]*(H[3]*b[2] - b[1]*H[6]) + b[0]*(H[3]*H[7] - H[4]*H[6])) / det;
                map->data[i].x += dx;
                map->data[i].y += dy;
                map->data[i].z += dz;
            }
        }
    }
}

// Robust χ² (Huber-rho) over all observations in the local BA window — used to
// score tentative LM steps in joint_local_ba.
static double window_chi2(const KFDB *db, const Map *map, int kf_start, int nkf_window,
                          double fx, double fy, double cx, double cy,
                          const int *point_local, double huber) {
    (void)point_local;
    double chi2 = 0.0;
    for (int k = 0; k < nkf_window; k++) {
        const KFEntry *kf = &db->data[kf_start + k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || map->data[pi].obs == 0)
                continue;
            MapPoint p = map->data[pi];

            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            if (cp[2] < 0.1)
                continue;

            double u  = fx*cp[0]/cp[2] + cx;
            double v  = fy*cp[1]/cp[2] + cy;
            double du = kf->corners.data[j].x - u;
            double dv = kf->corners.data[j].y - v;
            double e2 = du*du + dv*dv;

            chi2 += (e2 > huber) ? (2.0 * sqrt(huber * e2) - huber) : e2;
        }
    }
    return chi2;
}

// Joint local Bundle Adjustment (Schur-complement form, ORB-SLAM-style).
//
//   Window: last NK keyframes; the oldest is held FIXED as a gauge anchor.
//   Free states:  ξ_k ∈ se(3) for each free KF (6 dof each)
//                 P_i ∈ ℝ³    for each map point seen by ≥1 window KF (3 dof each)
//
//   Cost:  Σ_{k,i}  ρ( || z_{k,i} − π(R_k P_i + t_k) ||² )
//          ρ = Huber with χ²(2 dof, 95%) = 5.991.
//
//   Normal equations (LM-damped, then block-eliminated):
//
//     ⎡ U   W ⎤ ⎡ dξ ⎤   ⎡ b_ξ ⎤
//     ⎣ Wᵀ  V ⎦ ⎣ dP ⎦ = ⎣ b_P ⎦
//
//          U_k = Σ_i wᵢ J_ξᵀ J_ξ          (block-diag in k, 6×6 each)
//          V_i = Σ_k wᵢ J_Pᵀ J_P          (block-diag in i, 3×3 each)
//        W_{k,i} = wᵢ J_ξᵀ J_P            (cross block, 6×3)
//
//   Schur-eliminate the points (V is block-diag, cheap to invert):
//
//        S       = U − W V⁻¹ Wᵀ
//        g       = b_ξ − W V⁻¹ b_P
//        S · dξ  = g
//        dP_i    = V_i⁻¹ (b_P_i − Σ_k W_{k,i}ᵀ dξ_k)
//
//   LM control: damp diag(U), diag(V) by (1+λ); accept step if χ² decreased,
//   else revert and retry with 10·λ. Inner LM tries up to 5 λ values per
//   linearization; outer loop relinearizes up to 10 times.
static void joint_local_ba(KFDB *db, Map *map, double fx, double fy, double cx, double cy) {
    const int    NK    = 5;
    const double huber = 5.991;

    if (db->size < 3)
        return;
    int kf_start   = db->size - NK;
    if (kf_start < 0) kf_start = 0;
    int nkf_window = db->size - kf_start;
    int nfree      = nkf_window - 1;
    if (nfree < 1)
        return;

    // Local-point set: points with ≥2 observations within the window itself.
    // (Points seen only once in-window are weakly constrained when co-optimized,
    //  so we keep them as fixed-position pose constraints via the li<0 path.)
    int *point_local  = malloc(map->size * sizeof(int));
    int *win_obs_cnt  = calloc(map->size, sizeof(int));
    for (int i = 0; i < map->size; i++)
        point_local[i] = -1;
    for (int k = 0; k < nkf_window; k++) {
        KFEntry *kf = &db->data[kf_start + k];
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || map->data[pi].obs < 2)
                continue;
            win_obs_cnt[pi]++;
        }
    }
    int Np = 0;
    for (int i = 0; i < map->size; i++)
        if (win_obs_cnt[i] >= 2)
            point_local[i] = Np++;
    free(win_obs_cnt);
    if (Np < 5) {
        free(point_local);
        return;
    }
    int *local_to_global = malloc(Np * sizeof(int));
    for (int i = 0; i < map->size; i++)
        if (point_local[i] >= 0)
            local_to_global[point_local[i]] = i;

    // Block storage.
    int     N_pose = nfree * 6;
    double *U      = malloc((size_t)nfree * 36 * sizeof(double));
    double *V      = malloc((size_t)Np    * 9  * sizeof(double));
    double *W      = malloc((size_t)nfree * Np * 18 * sizeof(double));
    double *bxi    = malloc((size_t)nfree * 6  * sizeof(double));
    double *bp     = malloc((size_t)Np    * 3  * sizeof(double));
    double *U_damp = malloc((size_t)nfree * 36 * sizeof(double));
    double *V_damp = malloc((size_t)Np    * 9  * sizeof(double));
    double *Vinv   = malloc((size_t)Np    * 9  * sizeof(double));
    unsigned char *V_ok = malloc(Np);
    double *S      = malloc((size_t)N_pose * N_pose * sizeof(double));
    double *g      = malloc((size_t)N_pose * sizeof(double));
    double *dxi    = malloc((size_t)N_pose * sizeof(double));
    double *Y_buf  = malloc((size_t)nfree * 18 * sizeof(double));

    // Snapshots for rollback.
    Pose   *kf_save = malloc(nfree * sizeof(Pose));
    double *pt_save = malloc((size_t)Np * 3 * sizeof(double));

    double lambda    = 1e-3;
    double prev_chi2 = window_chi2(db, map, kf_start, nkf_window, fx, fy, cx, cy, point_local, huber);

    for (int outer = 0; outer < 10; outer++) {
        // Snapshot pre-step state.
        for (int k = 0; k < nfree; k++)
            kf_save[k] = db->data[kf_start + 1 + k].pose;
        for (int i = 0; i < Np; i++) {
            MapPoint p = map->data[local_to_global[i]];
            pt_save[i*3 + 0] = p.x;
            pt_save[i*3 + 1] = p.y;
            pt_save[i*3 + 2] = p.z;
        }

        // Build U, V, W, bxi, bp at current state.
        memset(U,   0, (size_t)nfree * 36 * sizeof(double));
        memset(V,   0, (size_t)Np    * 9  * sizeof(double));
        memset(W,   0, (size_t)nfree * Np * 18 * sizeof(double));
        memset(bxi, 0, (size_t)nfree * 6  * sizeof(double));
        memset(bp,  0, (size_t)Np    * 3  * sizeof(double));

        for (int k = 0; k < nkf_window; k++) {
            int      free_idx = (k == 0) ? -1 : k - 1;
            KFEntry *kf       = &db->data[kf_start + k];
            double R[9], t[3];
            pose_get_rotation(&kf->pose, R);
            pose_get_translation(&kf->pose, t);

            for (int j = 0; j < kf->corners.size; j++) {
                int pi = kf->corners.data[j].pt_idx;
                if (pi < 0 || map->data[pi].obs == 0)
                    continue;
                int li = point_local[pi];      // -1 ⇒ point is fixed (obs<2 or out of window)
                MapPoint p = map->data[pi];

                // cp = R · P + t
                double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                                R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                                R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
                if (cp[2] < 0.1)
                    continue;

                double inv_z  = 1.0 / cp[2];
                double inv_z2 = inv_z * inv_z;
                double u  = fx*cp[0]*inv_z + cx;
                double v  = fy*cp[1]*inv_z + cy;
                double du = kf->corners.data[j].x - u;
                double dv = kf->corners.data[j].y - v;

                double e2 = du*du + dv*dv;
                double w_robust = (e2 > huber) ? sqrt(huber / e2) : 1.0;

                // J_ξ (2×6): residual w.r.t. left-multiplied SE(3) perturbation (ρ, φ).
                double J_xi[2][6] = {
                    {fx*inv_z, 0,        -fx*cp[0]*inv_z2,  -fx*cp[0]*cp[1]*inv_z2,        fx*(1 + cp[0]*cp[0]*inv_z2),  -fx*cp[1]*inv_z},
                    {0,        fy*inv_z, -fy*cp[1]*inv_z2,  -fy*(1 + cp[1]*cp[1]*inv_z2),  fy*cp[0]*cp[1]*inv_z2,         fy*cp[0]*inv_z},
                };
                // J_P (2×3): residual w.r.t. world point P  (∂π/∂P = (∂π/∂cp) · R).
                double J_p[2][3] = {
                    {fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2,  fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2,  fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2},
                    {fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2,  fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2,  fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2},
                };

                if (li >= 0) {
                    // V_i  += w · J_Pᵀ J_P,    b_P_i += w · J_Pᵀ r   (free point)
                    double *Vi  = V  + li * 9;
                    double *bpi = bp + li * 3;
                    for (int r = 0; r < 3; r++) {
                        bpi[r] += w_robust * (J_p[0][r]*du + J_p[1][r]*dv);
                        for (int c = 0; c < 3; c++)
                            Vi[r*3 + c] += w_robust * (J_p[0][r]*J_p[0][c] + J_p[1][r]*J_p[1][c]);
                    }
                }

                if (free_idx >= 0) {
                    // U_k    += w · J_ξᵀ J_ξ,    b_ξ_k  += w · J_ξᵀ r
                    double *Uk   = U   + free_idx * 36;
                    double *bxik = bxi + free_idx * 6;
                    for (int r = 0; r < 6; r++) {
                        bxik[r] += w_robust * (J_xi[0][r]*du + J_xi[1][r]*dv);
                        for (int c = 0; c < 6; c++)
                            Uk[r*6 + c] += w_robust * (J_xi[0][r]*J_xi[0][c] + J_xi[1][r]*J_xi[1][c]);
                    }
                    if (li >= 0) {
                        // W_{k,i} += w · J_ξᵀ J_P     (only when point is free)
                        double *Wki = W + ((size_t)free_idx * Np + li) * 18;
                        for (int r = 0; r < 6; r++)
                            for (int c = 0; c < 3; c++)
                                Wki[r*3 + c] += w_robust * (J_xi[0][r]*J_p[0][c] + J_xi[1][r]*J_p[1][c]);
                    }
                }
            }
        }

        int accepted = 0;
        for (int inner = 0; inner < 5 && !accepted; inner++) {
            // LM diagonal damping (Marquardt scaling):
            //   U_damp = U + λ · diag(U)        V_damp = V + λ · diag(V)
            memcpy(U_damp, U, (size_t)nfree * 36 * sizeof(double));
            memcpy(V_damp, V, (size_t)Np    * 9  * sizeof(double));
            for (int k = 0; k < nfree; k++) {
                double *Uk = U_damp + k * 36;
                for (int r = 0; r < 6; r++)
                    Uk[r*6 + r] *= (1.0 + lambda);
            }
            for (int i = 0; i < Np; i++) {
                double *Vi = V_damp + i * 9;
                for (int r = 0; r < 3; r++)
                    Vi[r*3 + r] *= (1.0 + lambda);
            }
            for (int i = 0; i < Np; i++)
                V_ok[i] = mat3_inverse(V_damp + i * 9, Vinv + i * 9);

            // S ← block-diag(U_damp);      g ← b_ξ
            memset(S, 0, (size_t)N_pose * N_pose * sizeof(double));
            for (int k = 0; k < nfree; k++) {
                double *Uk = U_damp + k * 36;
                for (int r = 0; r < 6; r++)
                    for (int c = 0; c < 6; c++)
                        S[(k*6 + r) * N_pose + (k*6 + c)] = Uk[r*6 + c];
            }
            memcpy(g, bxi, (size_t)N_pose * sizeof(double));

            // S −= Σ_i W_{·,i} V_i⁻¹ W_{·,i}ᵀ;    g −= Σ_i W_{·,i} V_i⁻¹ b_P_i
            for (int i = 0; i < Np; i++) {
                if (!V_ok[i])
                    continue;
                double *Vi_inv = Vinv + i * 9;

                // Y_k = W_{k,i} · V_i⁻¹  (6×3) for each free k.
                for (int k = 0; k < nfree; k++) {
                    double *Wki = W + ((size_t)k * Np + i) * 18;
                    double *Yk  = Y_buf + k * 18;
                    for (int r = 0; r < 6; r++)
                        for (int c = 0; c < 3; c++) {
                            double s = 0;
                            for (int kk = 0; kk < 3; kk++)
                                s += Wki[r*3 + kk] * Vi_inv[kk*3 + c];
                            Yk[r*3 + c] = s;
                        }
                }
                // S[k1][k2] −= Y_{k1} · W_{k2,i}ᵀ
                for (int k1 = 0; k1 < nfree; k1++) {
                    double *Yk1 = Y_buf + k1 * 18;
                    for (int k2 = 0; k2 < nfree; k2++) {
                        double *Wk2i = W + ((size_t)k2 * Np + i) * 18;
                        for (int r = 0; r < 6; r++)
                            for (int c = 0; c < 6; c++) {
                                double s = 0;
                                for (int kk = 0; kk < 3; kk++)
                                    s += Yk1[r*3 + kk] * Wk2i[c*3 + kk];
                                S[(k1*6 + r) * N_pose + (k2*6 + c)] -= s;
                            }
                    }
                }
                // g[k] −= Y_k · b_P_i
                double *bpi = bp + i * 3;
                for (int k = 0; k < nfree; k++) {
                    double *Yk = Y_buf + k * 18;
                    for (int r = 0; r < 6; r++) {
                        double s = 0;
                        for (int c = 0; c < 3; c++)
                            s += Yk[r*3 + c] * bpi[c];
                        g[k*6 + r] -= s;
                    }
                }
            }

            if (!solve_dense_n(S, g, dxi, N_pose)) {
                lambda *= 10.0;
                if (lambda > 1e8) goto done;
                continue;
            }

            // Apply pose updates: t += ρ;   R ← (I + [φ]_×) R, then re-orthogonalize.
            for (int k = 0; k < nfree; k++) {
                Pose  *pose = &db->data[kf_start + 1 + k].pose;
                double R[9], t[3];
                pose_get_rotation(pose, R);
                pose_get_translation(pose, t);

                double rho[3] = {dxi[k*6 + 0], dxi[k*6 + 1], dxi[k*6 + 2]};
                double phi[3] = {dxi[k*6 + 3], dxi[k*6 + 4], dxi[k*6 + 5]};
                t[0] += rho[0];
                t[1] += rho[1];
                t[2] += rho[2];

                double dR[9] = { 1,       -phi[2],  phi[1],
                                 phi[2],   1,      -phi[0],
                                -phi[1],   phi[0],  1     };
                double Rn[9], R_proj[9];
                mat3_mul(dR, R, Rn);
                project_to_SO3(Rn, R_proj);
                pose_from_rt(R_proj, t, pose);
            }
            // Back-substitute point updates: dP_i = V_i⁻¹ (b_P_i − Σ_k W_{k,i}ᵀ dξ_k)
            for (int i = 0; i < Np; i++) {
                if (!V_ok[i])
                    continue;
                double rhs[3] = {bp[i*3 + 0], bp[i*3 + 1], bp[i*3 + 2]};
                for (int k = 0; k < nfree; k++) {
                    double *Wki  = W   + ((size_t)k * Np + i) * 18;
                    double *dxik = dxi + k * 6;
                    for (int c = 0; c < 3; c++) {
                        double s = 0;
                        for (int r = 0; r < 6; r++)
                            s += Wki[r*3 + c] * dxik[r];
                        rhs[c] -= s;
                    }
                }
                double *Vi_inv = Vinv + i * 9;
                int gi = local_to_global[i];
                map->data[gi].x += Vi_inv[0]*rhs[0] + Vi_inv[1]*rhs[1] + Vi_inv[2]*rhs[2];
                map->data[gi].y += Vi_inv[3]*rhs[0] + Vi_inv[4]*rhs[1] + Vi_inv[5]*rhs[2];
                map->data[gi].z += Vi_inv[6]*rhs[0] + Vi_inv[7]*rhs[1] + Vi_inv[8]*rhs[2];
            }

            double new_chi2 = window_chi2(db, map, kf_start, nkf_window, fx, fy, cx, cy,
                                          point_local, huber);
            if (new_chi2 < prev_chi2) {
                prev_chi2 = new_chi2;
                lambda *= 0.5;
                if (lambda < 1e-8) lambda = 1e-8;
                accepted = 1;
            } else {
                // Revert and try again with stronger damping.
                for (int k = 0; k < nfree; k++)
                    db->data[kf_start + 1 + k].pose = kf_save[k];
                for (int i = 0; i < Np; i++) {
                    int gi = local_to_global[i];
                    map->data[gi].x = pt_save[i*3 + 0];
                    map->data[gi].y = pt_save[i*3 + 1];
                    map->data[gi].z = pt_save[i*3 + 2];
                }
                lambda *= 10.0;
                if (lambda > 1e8) goto done;
            }
        }
        if (!accepted)
            break;
    }
done:
    free(U);     free(V);     free(W);     free(bxi);   free(bp);
    free(U_damp); free(V_damp); free(Vinv); free(V_ok);
    free(S);     free(g);     free(dxi);   free(Y_buf);
    free(point_local); free(local_to_global);
    free(kf_save); free(pt_save);
}

// Cull map points that reproject above `err_thresh_px` in EVERY observing
// keyframe of the local window (with ≥2 obs in window).  Setting obs=0 marks
// the point invalid; existing pt_idx links are gated on map->data[i].obs > 0.
static void cull_map_points_window(KFDB *db, Map *map, int kf_start, int kf_end,
                                   double fx, double fy, double cx, double cy,
                                   double err_thresh_px) {
    double th2   = err_thresh_px * err_thresh_px;
    int   *total = calloc(map->size, sizeof(int));
    int   *bad   = calloc(map->size, sizeof(int));

    for (int k = kf_start; k < kf_end; k++) {
        KFEntry *kf = &db->data[k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || map->data[pi].obs < 2)
                continue;
            MapPoint p = map->data[pi];

            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            total[pi]++;
            if (cp[2] < 0.1) {
                bad[pi]++;
                continue;
            }
            double u  = fx*cp[0]/cp[2] + cx;
            double v  = fy*cp[1]/cp[2] + cy;
            double du = kf->corners.data[j].x - u;
            double dv = kf->corners.data[j].y - v;
            if (du*du + dv*dv > th2)
                bad[pi]++;
        }
    }
    for (int pi = 0; pi < map->size; pi++) {
        if (total[pi] >= 2 && bad[pi] == total[pi])
            map->data[pi].obs = 0;
    }
    free(total);
    free(bad);
}

static void ensure_parent_dir(const char *p) {
    char b[512], *s;
    snprintf(b, 512, "%s", p);
    s = strrchr(b, '/');
    if (s) {
        *s = '\0';
        char c[640];
        snprintf(c, 640, "mkdir -p \"%s\"", b);
        (void)!system(c);
    }
}
static void write_metrics_json(FILE *f, const Config *cfg, const FrameStatVec *s, int pts, int tri,
                               double dur) {
    int kf = 0;
    double av = 0;
    for (int i = 0; i < s->size; i++) {
        if (s->data[i].is_keyframe)
            kf++;
        if (i > 0)
            av += s->data[i].inliers;
    }
    if (s->size > 1)
        av /= (s->size - 1);
    fprintf(f,
            "{\n  \"frames\": %d, \"points\": %d, \"duration_sec\": %f, \"video_path\": \"%s\", "
            "\"proc_w\": %d, \"proc_h\": %d, \"keyframes\": %d, \"tri_points_total\": %d, "
            "\"speed_profile\": \"%s\", "
            "\"avg_inliers_after_first\": %f, \"kf_min_inliers\": %d, \"kf_period\": %d, "
            "\"kf_min_interval\": %d, \"healthy_keyframes\": %d, "
            "\"late_kf_cooldown\": %d, \"ba_interval\": %d, "
            "\"global_ba_interval\": %d, \"map_hygiene\": %d, "
            "\"kf_warmup_frames\": %d, "
            "\"new_point_obs\": %d, \"pnp_min_obs\": %d, \"pnp_start_frame\": %d, "
            "\"delayed_init_frames\": %d, "
            "\"candidate_tracks\": %d, \"candidate_min_obs\": %d, "
            "\"candidate_min_age\": %d, \"candidate_grid_cols\": %d, "
            "\"candidate_grid_rows\": %d, \"candidate_promote_per_cell\": %d, "
            "\"candidate_max_fb_err\": %f, \"candidate_min_disp\": %f, "
            "\"candidate_max_disp\": %f, "
            "\"first_kf_observations\": %d, "
            "\"unique_kf_observations\": %d, \"essential_cheirality_max\": %d, "
            "\"tri_min_parallax_deg\": %f, \"tri_max_reproj_px\": %f, "
            "\"tri_max_depth\": %f, \"tri_max_depth_ratio\": %f, "
            "\"tri_source_kf_gap\": %d, "
            "\"pnp_pred_reproj_gate\": %f, "
            "\"pnp_quality_gate_px\": %f, \"pnp_quality_min_obs\": %d, "
            "\"pnp_quality_window\": %d, "
            "\"obs_stat_gate_px\": %f, \"obs_stat_min_good\": %d, "
            "\"obs_stat_max_bad_ratio\": %f, "
            "\"pnp_p3p_fallback\": %d, \"pnp_p3p_observe\": %d, "
            "\"pnp_p3p_iterations\": %d, "
            "\"pnp_p3p_max_jump\": %f, \"pnp_p3p_min_inl2\": %d, "
            "\"pnp_p3p_min_gain\": %d, \"pnp_p3p_max_mederr\": %f, "
            "\"pnp_p3p_min_posz\": %f, \"pnp_dlt_iters\": %d, "
            "\"pnp_dlt_pretest\": %d, \"pnp_dlt_pretest_margin\": %d, "
            "\"ffmpeg_gray\": %d, \"fast_corners\": %d, \"distributed_features\": %d, "
            "\"pyramid_features\": %d, "
            "\"max_points\": %d, \"lk_iters\": %d, \"lk_back_iters\": %d, "
            "\"pose_lm_iters\": %d, \"essential_iters\": %d, "
            "\"descriptor_map_admission\": %d, \"descriptor_primary_admission\": %d, "
            "\"descriptor_mutual_admission\": %d, "
            "\"oriented_brief\": %d, "
            "\"descriptor_admission_max_hamming\": %d, "
            "\"descriptor_admission_ratio\": %f, \"descriptor_primary_map_cap\": %d, "
            "\"descriptor_source_kf_gap\": %d, "
            "\"triangulate_with_e_pose\": %d, "
            "\"triangulate_relative_frame\": %d, "
            "\"shape_e_inliers\": %d, \"e_shape_max_matches\": %d, "
            "\"e_shape_grid_cap\": %d, \"e_shape_max_disp\": %f, "
            "\"e_shape_max_fb_err\": %f, \"e_shape_target_disp\": %f, "
            "\"anchor_e_pose\": %d, \"anchor_max_features\": %d, "
            "\"anchor_max_hamming\": %d, \"anchor_ratio\": %f, "
            "\"admission_ranked\": %d, \"admission_finite_only\": %d, "
            "\"admission_batch_ranked\": %d, \"admission_batch_deferred\": %d, "
            "\"admission_target_disp\": %f, "
            "\"admission_fb_weight\": %f, "
            "\"admission_max_new_points\": %d, \"admission_grid_cap\": %d, "
            "\"timeline\": [\n",
            s->size, pts, dur, cfg->video_path, cfg->proc_w, cfg->proc_h, kf, tri,
            cfg->speed_profile ? cfg->speed_profile : "", av,
            cfg->kf_min_inliers, cfg->kf_period, cfg->kf_min_interval, cfg->healthy_keyframes,
            cfg->late_kf_cooldown, cfg->ba_interval, cfg->global_ba_interval, cfg->map_hygiene,
            cfg->kf_warmup_frames, cfg->new_point_obs, cfg->pnp_min_obs, cfg->pnp_start_frame,
            cfg->delayed_init_frames, cfg->candidate_tracks, cfg->candidate_min_obs, cfg->candidate_min_age,
            cfg->candidate_grid_cols, cfg->candidate_grid_rows, cfg->candidate_promote_per_cell,
            cfg->candidate_max_fb_err, cfg->candidate_min_disp, cfg->candidate_max_disp,
            cfg->first_kf_observations,
            cfg->unique_kf_observations, cfg->essential_cheirality_max,
            cfg->tri_min_parallax_deg, cfg->tri_max_reproj_px,
            cfg->tri_max_depth, cfg->tri_max_depth_ratio, cfg->tri_source_kf_gap,
            cfg->pnp_pred_reproj_gate,
            cfg->pnp_quality_gate_px, cfg->pnp_quality_min_obs, cfg->pnp_quality_window,
            cfg->obs_stat_gate_px, cfg->obs_stat_min_good, cfg->obs_stat_max_bad_ratio,
            cfg->pnp_p3p_fallback, cfg->pnp_p3p_observe, cfg->pnp_p3p_iterations,
            cfg->pnp_p3p_max_jump, cfg->pnp_p3p_min_inl2, cfg->pnp_p3p_min_gain,
            cfg->pnp_p3p_max_mederr, cfg->pnp_p3p_min_posz, cfg->pnp_dlt_iters, cfg->pnp_dlt_pretest,
            cfg->pnp_dlt_pretest_margin, cfg->ffmpeg_gray, cfg->fast_corners,
            cfg->distributed_features, cfg->pyramid_features,
            cfg->max_points, cfg->lk_iters, cfg->lk_back_iters, cfg->pose_lm_iters, cfg->essential_iters,
            cfg->descriptor_map_admission, cfg->descriptor_primary_admission,
            cfg->descriptor_mutual_admission, cfg->oriented_brief,
            cfg->descriptor_admission_max_hamming,
            cfg->descriptor_admission_ratio, cfg->descriptor_primary_map_cap,
            cfg->descriptor_source_kf_gap, cfg->triangulate_with_e_pose,
            cfg->triangulate_relative_frame, cfg->shape_e_inliers,
            cfg->e_shape_max_matches, cfg->e_shape_grid_cap,
            cfg->e_shape_max_disp, cfg->e_shape_max_fb_err,
            cfg->e_shape_target_disp, cfg->anchor_e_pose,
            cfg->anchor_max_features, cfg->anchor_max_hamming, cfg->anchor_ratio,
            cfg->admission_ranked, cfg->admission_finite_only, cfg->admission_batch_ranked,
            cfg->admission_batch_deferred,
            cfg->admission_target_disp, cfg->admission_fb_weight,
            cfg->admission_max_new_points, cfg->admission_grid_cap);
    for (int i = 0; i < s->size; i++)
        fprintf(f,
                "    {\"frame_id\": %d, \"inliers\": %d, \"is_keyframe\": %s, \"points_added\": "
                "%d, \"points_total\": %d, \"method\": %d, \"tracked_count\": %d, "
                "\"linked_points\": %d, \"linked_before_relink\": %d, \"relinked_points\": %d, "
                "\"pnp_inliers\": %d, \"pred_lm_inliers\": %d, \"e_inliers\": %d, "
                "\"pnp_fallback_used\": %d, \"pnp_p3p_attempted\": %d, "
                "\"pnp_p3p_solved\": %d, \"pnp_p3p_accepted\": %d, "
                "\"pnp_p3p_inliers\": %d, \"pnp_p3p_inliers2\": %d, "
                "\"pnp_p3p_inliers3\": %d, \"pnp_p3p_inliers5\": %d, "
                "\"pnp_p3p_mederr\": %f, \"pnp_p3p_posz\": %f, "
                "\"pnp_p3p_jump\": %f, \"pnp_p3p_xyz\": [%f,%f,%f], "
                "\"trans_jump\": %f, \"xyz\": [%f,%f,%f]}%s\n",
                s->data[i].frame_id, s->data[i].inliers, s->data[i].is_keyframe ? "true" : "false",
                s->data[i].points_added, s->data[i].points_total, s->data[i].method,
                s->data[i].tracked_count, s->data[i].linked_points, s->data[i].linked_before_relink,
                s->data[i].relinked_points, s->data[i].pnp_inliers, s->data[i].pred_lm_inliers,
                s->data[i].e_inliers, s->data[i].pnp_fallback_used,
                s->data[i].pnp_p3p_attempted, s->data[i].pnp_p3p_solved,
                s->data[i].pnp_p3p_accepted, s->data[i].pnp_p3p_inliers,
                s->data[i].pnp_p3p_inliers2, s->data[i].pnp_p3p_inliers3,
                s->data[i].pnp_p3p_inliers5, s->data[i].pnp_p3p_mederr,
                s->data[i].pnp_p3p_posz, s->data[i].pnp_p3p_jump,
                s->data[i].pnp_p3p_xyz[0], s->data[i].pnp_p3p_xyz[1],
                s->data[i].pnp_p3p_xyz[2], s->data[i].trans_jump, s->data[i].xyz[0],
                s->data[i].xyz[1], s->data[i].xyz[2],
                (i + 1 < s->size) ? "," : "");
    fprintf(f, "  ]\n}\n");
}

static void write_profile_csv(FILE *f, const ProfileStatVec *p) {
    fprintf(f,
            "frame_id,decode,gray_blur,feature,lk,relink,pred_lm,pnp,essential,pose_lm,"
            "triangulate,refill,loop,ba,global_ba,metrics,total\n");
    for (int i = 0; i < p->size; i++) {
        const ProfileStat *s = &p->data[i];
        double total = s->decode + s->gray_blur + s->feature + s->lk + s->relink + s->pred_lm +
                       s->pnp + s->essential + s->pose_lm + s->triangulate + s->refill +
                       s->loop + s->ba + s->global_ba + s->metrics;
        fprintf(f,
                "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                "%.9f,%.9f,%.9f,%.9f\n",
                s->frame_id, s->decode, s->gray_blur, s->feature, s->lk, s->relink,
                s->pred_lm, s->pnp, s->essential, s->pose_lm, s->triangulate, s->refill,
                s->loop, s->ba, s->global_ba, s->metrics, total);
    }
}
static void write_track_summary(FILE *f, int frame_id, const CornerVec *tracked,
                                const MatchVec *matches, const Map *map, int w, int h) {
    if (!f || frame_id <= 0)
        return;
    int n = tracked->size;
    int linked = 0, linked_valid = 0;
    double sum_fb = 0.0, sum_disp = 0.0;
    double min_x = w, min_y = h, max_x = 0.0, max_y = 0.0;
    unsigned char cells[48] = {0};
    int occupied = 0;
    for (int i = 0; i < tracked->size; i++) {
        const Corner *c = &tracked->data[i];
        sum_fb += c->fb_err;
        sum_disp += c->track_disp;
        if (c->x < min_x) min_x = c->x;
        if (c->y < min_y) min_y = c->y;
        if (c->x > max_x) max_x = c->x;
        if (c->y > max_y) max_y = c->y;
        int gx = (int)(c->x * 8.0 / (double)w);
        int gy = (int)(c->y * 6.0 / (double)h);
        if (gx < 0) gx = 0;
        if (gy < 0) gy = 0;
        if (gx > 7) gx = 7;
        if (gy > 5) gy = 5;
        int cell = gy * 8 + gx;
        if (!cells[cell]) {
            cells[cell] = 1;
            occupied++;
        }
        if (c->pt_idx >= 0) {
            linked++;
            if (c->pt_idx < map->size && map->data[c->pt_idx].obs > 0)
                linked_valid++;
        }
    }
    double mean_fb = n > 0 ? sum_fb / (double)n : 0.0;
    double mean_disp = n > 0 ? sum_disp / (double)n : 0.0;
    if (n == 0) {
        min_x = min_y = max_x = max_y = 0.0;
    }
    fprintf(f,
            "%d,%d,%d,%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%d\n",
            frame_id, matches->size, n, linked, linked_valid, mean_fb, mean_disp,
            min_x, min_y, max_x, max_y, occupied);
}

static void write_e_inlier_pairs(FILE *f, int frame_id, const char *source,
                                 const CornerVec *prev_pts, const CornerVec *cur_pts,
                                 const MatchVec *matches, const unsigned char *mask, int w, int h) {
    if (!f || !mask)
        return;
    for (int i = 0; i < matches->size; i++) {
        if (!mask[i])
            continue;
        int qi = matches->data[i].query_idx;
        int ti = matches->data[i].train_idx;
        if (qi < 0 || qi >= prev_pts->size || ti < 0 || ti >= cur_pts->size)
            continue;
        Corner a = prev_pts->data[qi];
        Corner b = cur_pts->data[ti];
        double dx = (double)b.x - (double)a.x;
        double dy = (double)b.y - (double)a.y;
        double disp = sqrt(dx * dx + dy * dy);
        int gx = (int)(b.x * 8.0 / (double)w);
        int gy = (int)(b.y * 6.0 / (double)h);
        if (gx < 0) gx = 0;
        if (gy < 0) gy = 0;
        if (gx > 7) gx = 7;
        if (gy > 5) gy = 5;
        fprintf(f,
                "%d,%s,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.3f,%d,%.6f,%.6f,%d\n",
                frame_id, source, i, qi, ti, a.x, a.y, b.x, b.y, dx, dy, disp,
                matches->data[i].score, b.pt_idx, b.fb_err, b.track_disp, gy * 8 + gx);
    }
}

static double triangulation_parallax_deg(const Pose *p1, const Pose *p2, const double X[3]) {
    double c1[3], c2[3], r1[3], r2[3];
    camera_center_from_pose(p1, c1);
    camera_center_from_pose(p2, c2);
    for (int i = 0; i < 3; i++) {
        r1[i] = X[i] - c1[i];
        r2[i] = X[i] - c2[i];
    }
    vec3_normalize(r1);
    vec3_normalize(r2);
    double cs = vec3_dot(r1, r2);
    if (cs > 1.0) cs = 1.0;
    if (cs < -1.0) cs = -1.0;
    return acos(cs) * 180.0 / M_PI;
}
static void write_map_admission_summary(FILE *f, int frame_id, int candidates, int accepted,
                                        const Pose *p1, const Pose *p2, double fx, double fy,
                                        double cx, double cy, double sum_reproj,
                                        double sum_parallax, double sum_depth,
                                        double min_depth, double max_depth) {
    if (!f)
        return;
    double mean_reproj = accepted > 0 ? sum_reproj / (double)accepted : 0.0;
    double mean_parallax = accepted > 0 ? sum_parallax / (double)accepted : 0.0;
    double mean_depth = accepted > 0 ? sum_depth / (double)accepted : 0.0;
    if (accepted <= 0) {
        min_depth = 0.0;
        max_depth = 0.0;
    }
    double c1[3], c2[3];
    camera_center_from_pose(p1, c1);
    camera_center_from_pose(p2, c2);
    double baseline = vec3_dist(c1, c2);
    fprintf(f, "%d,%d,%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            frame_id, candidates, accepted, baseline, mean_reproj, mean_parallax,
            mean_depth, min_depth, max_depth);
}

static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void apply_speed_profile(Config *c, const char *name) {
    c->speed_profile = name;
    if (!strcmp(name, "fast_640")) {
        c->ffmpeg_gray = 1;
        c->pnp_dlt_iters = 250;
        c->lk_iters = 5;
        c->lk_back_iters = 2;
        c->ba_interval = 0;
        c->global_ba_interval = 0;
    } else if (!strcmp(name, "fast_320_noba")) {
        c->proc_w = 320;
        c->proc_h = 240;
        c->ffmpeg_gray = 1;
        c->pnp_dlt_iters = 150;
        c->lk_iters = 5;
        c->lk_back_iters = 2;
        c->ba_interval = 0;
        c->global_ba_interval = 0;
    } else if (!strcmp(name, "fast_320_ba")) {
        c->proc_w = 320;
        c->proc_h = 240;
        c->ffmpeg_gray = 1;
        c->pnp_dlt_iters = 250;
        c->lk_iters = 5;
        c->lk_back_iters = 2;
        c->ba_interval = 1;
        c->global_ba_interval = 0;
    } else {
        fprintf(stderr, "unknown --speed_profile %s\n", name);
    }
}

static int config_uses_descriptor_admission(const Config *c) {
    return c->descriptor_map_admission || c->descriptor_primary_admission;
}

static int config_suppresses_lk_landmarks(const Config *c) {
    return config_uses_descriptor_admission(c) && !c->candidate_tracks;
}

static int config_allows_new_landmarks(const Config *c, int frame_id) {
    return c->delayed_init_frames <= 0 || frame_id >= c->delayed_init_frames;
}

static int should_make_keyframe(const Config *c, int frame_id, int inliers,
                                int linked_points, double rot_deg) {
    if (frame_id <= c->kf_warmup_frames)
        return 1;
    if (rot_deg > c->kf_max_rot_deg)
        return 1;
    if (frame_id % c->kf_period == 0)
        return 1;
    if (inliers < c->kf_min_inliers) {
        if (!c->healthy_keyframes || frame_id < HEALTHY_KF_START_FRAME)
            return 1;
        return linked_points >= HEALTHY_KF_MIN_LINKED;
    }
    return 0;
}

static int keyframe_interval_for_frame(const Config *c, int frame_id) {
    int interval = c->kf_min_interval;
    if (c->late_kf_cooldown && frame_id >= LATE_KF_COOLDOWN_START_FRAME &&
        interval < LATE_KF_MIN_INTERVAL)
        interval = LATE_KF_MIN_INTERVAL;
    return interval;
}

static int create_deferred_candidate(CandidateVec *candidates, int frame_id,
                                     const Pose *prev_pose, const Pose *cur_pose,
                                     Corner prev_corner, Corner cur_corner,
                                     const Brief256 *desc) {
    TrackCandidate tc;
    memset(&tc, 0, sizeof(tc));
    tc.valid = 1;
    tc.first_frame = frame_id - 1;
    tc.last_frame = frame_id;
    tc.obs = 2;
    tc.anchor_pose = *prev_pose;
    tc.anchor_corner = prev_corner;
    tc.anchor_corner.pt_idx = -1;
    tc.anchor_corner.cand_idx = -1;
    tc.last_corner = cur_corner;
    tc.desc = *desc;
    candidate_add_observation(&tc, frame_id - 1, prev_pose, prev_corner);
    candidate_add_observation(&tc, frame_id, cur_pose, cur_corner);
    return candidate_vec_push(candidates, tc);
}

static int image_grid_cell(double x, double y, int w, int h, int cols, int rows) {
    if (cols < 1)
        cols = 1;
    if (rows < 1)
        rows = 1;
    int gx = (int)(x * (double)cols / (double)w);
    int gy = (int)(y * (double)rows / (double)h);
    if (gx < 0) gx = 0;
    if (gy < 0) gy = 0;
    if (gx >= cols) gx = cols - 1;
    if (gy >= rows) gy = rows - 1;
    return gy * cols + gx;
}

static Config parse_args(int argc, char **argv) {
    Config c = {
        .video_path = "test_kitti984.mp4",
        .seconds = 5.0,
        .timeout = 30.0,
        .kf_min_inliers = 40,
        .kf_max_rot_deg = 5.0,
        .max_points = 1000,
        .new_point_obs = 1,
        .pnp_min_obs = 1,
        .candidate_min_obs = 3,
        .candidate_min_age = 5,
        .candidate_grid_cols = 4,
        .candidate_grid_rows = 3,
        .proc_w = 640,
        .proc_h = 480,
        .kf_period = 10,
        .ba_interval = 1,
        .global_ba_interval = 10,
        .essential_cheirality_max = 32,
        .pnp_quality_min_obs = 2,
        .pnp_quality_window = 5,
        .obs_stat_max_bad_ratio = 1.0,
        .pnp_p3p_iterations = 500,
        .pnp_p3p_max_jump = 500000.0,
        .pnp_p3p_min_inl2 = 16,
        .pnp_p3p_min_gain = 10,
        .pnp_p3p_max_mederr = 15.0,
        .pnp_p3p_min_posz = 0.8,
        .pnp_dlt_iters = 500,
        .pnp_dlt_pretest_margin = 2,
        .lk_iters = 10,
        .lk_back_iters = 5,
        .pose_lm_iters = 10,
        .essential_iters = 500,
        .descriptor_admission_max_hamming = 80,
        .descriptor_admission_ratio = 0.80,
        .descriptor_primary_map_cap = 15000,
        .e_shape_max_matches = 64,
        .e_shape_grid_cap = 2,
        .e_shape_max_disp = 14.0,
        .e_shape_max_fb_err = 0.20,
        .e_shape_target_disp = 8.0,
        .anchor_max_features = 600,
        .anchor_max_hamming = 80,
        .anchor_ratio = 0.80,
        .admission_target_disp = 12.0,
        .admission_fb_weight = 10.0,
    };
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--video_path") && i + 1 < argc)
            c.video_path = argv[++i];
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc)
            c.seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "--timeout") && i + 1 < argc)
            c.timeout = atof(argv[++i]);
        else if (!strcmp(argv[i], "--metrics_out") && i + 1 < argc)
            c.metrics_out = argv[++i];
        else if (!strcmp(argv[i], "--pnp_dump") && i + 1 < argc)
            c.pnp_dump = argv[++i];
        else if (!strcmp(argv[i], "--profile_out") && i + 1 < argc)
            c.profile_out = argv[++i];
        else if (!strcmp(argv[i], "--track_dump") && i + 1 < argc)
            c.track_dump = argv[++i];
        else if (!strcmp(argv[i], "--map_admission_dump") && i + 1 < argc)
            c.map_admission_dump = argv[++i];
        else if (!strcmp(argv[i], "--e_inlier_dump") && i + 1 < argc)
            c.e_inlier_dump = argv[++i];
        else if (!strcmp(argv[i], "--speed_profile") && i + 1 < argc)
            apply_speed_profile(&c, argv[++i]);
        else if (!strcmp(argv[i], "--proc_w") && i + 1 < argc)
            c.proc_w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--proc_h") && i + 1 < argc)
            c.proc_h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_min_inliers") && i + 1 < argc)
            c.kf_min_inliers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_max_rot_deg") && i + 1 < argc)
            c.kf_max_rot_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--max_points") && i + 1 < argc)
            c.max_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--new_point_obs") && i + 1 < argc)
            c.new_point_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_min_obs") && i + 1 < argc)
            c.pnp_min_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_start_frame") && i + 1 < argc)
            c.pnp_start_frame = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--delayed_init_frames") && i + 1 < argc)
            c.delayed_init_frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_tracks"))
            c.candidate_tracks = 1;
        else if (!strcmp(argv[i], "--descriptor_map_admission"))
            c.descriptor_map_admission = 1;
        else if (!strcmp(argv[i], "--descriptor_primary_admission")) {
            c.descriptor_primary_admission = 1;
            c.descriptor_map_admission = 1;
        }
        else if (!strcmp(argv[i], "--descriptor_mutual_admission"))
            c.descriptor_mutual_admission = 1;
        else if (!strcmp(argv[i], "--oriented_brief"))
            c.oriented_brief = 1;
        else if (!strcmp(argv[i], "--descriptor_admission_max_hamming") && i + 1 < argc)
            c.descriptor_admission_max_hamming = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_admission_ratio") && i + 1 < argc)
            c.descriptor_admission_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_primary_map_cap") && i + 1 < argc)
            c.descriptor_primary_map_cap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_source_kf_gap") && i + 1 < argc)
            c.descriptor_source_kf_gap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--triangulate_with_e_pose"))
            c.triangulate_with_e_pose = 1;
        else if (!strcmp(argv[i], "--triangulate_relative_frame"))
            c.triangulate_relative_frame = 1;
        else if (!strcmp(argv[i], "--shape_e_inliers"))
            c.shape_e_inliers = 1;
        else if (!strcmp(argv[i], "--e_shape_max_matches") && i + 1 < argc)
            c.e_shape_max_matches = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_grid_cap") && i + 1 < argc)
            c.e_shape_grid_cap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_max_disp") && i + 1 < argc)
            c.e_shape_max_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_max_fb_err") && i + 1 < argc)
            c.e_shape_max_fb_err = atof(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_target_disp") && i + 1 < argc)
            c.e_shape_target_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--anchor_e_pose"))
            c.anchor_e_pose = 1;
        else if (!strcmp(argv[i], "--anchor_max_features") && i + 1 < argc)
            c.anchor_max_features = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--anchor_max_hamming") && i + 1 < argc)
            c.anchor_max_hamming = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--anchor_ratio") && i + 1 < argc)
            c.anchor_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--admission_ranked"))
            c.admission_ranked = 1;
        else if (!strcmp(argv[i], "--admission_finite_only"))
            c.admission_finite_only = 1;
        else if (!strcmp(argv[i], "--admission_batch_ranked"))
            c.admission_batch_ranked = 1;
        else if (!strcmp(argv[i], "--admission_batch_deferred")) {
            c.admission_batch_deferred = 1;
            c.admission_batch_ranked = 1;
        }
        else if (!strcmp(argv[i], "--admission_target_disp") && i + 1 < argc)
            c.admission_target_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--admission_fb_weight") && i + 1 < argc)
            c.admission_fb_weight = atof(argv[++i]);
        else if (!strcmp(argv[i], "--admission_max_new_points") && i + 1 < argc)
            c.admission_max_new_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--admission_grid_cap") && i + 1 < argc)
            c.admission_grid_cap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_min_obs") && i + 1 < argc)
            c.candidate_min_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_min_age") && i + 1 < argc)
            c.candidate_min_age = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_grid_cols") && i + 1 < argc)
            c.candidate_grid_cols = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_grid_rows") && i + 1 < argc)
            c.candidate_grid_rows = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_promote_per_cell") && i + 1 < argc)
            c.candidate_promote_per_cell = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_max_fb_err") && i + 1 < argc)
            c.candidate_max_fb_err = atof(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_min_disp") && i + 1 < argc)
            c.candidate_min_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_max_disp") && i + 1 < argc)
            c.candidate_max_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--kf_period") && i + 1 < argc)
            c.kf_period = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_min_interval") && i + 1 < argc)
            c.kf_min_interval = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--healthy_keyframes"))
            c.healthy_keyframes = 1;
        else if (!strcmp(argv[i], "--late_kf_cooldown"))
            c.late_kf_cooldown = 1;
        else if (!strcmp(argv[i], "--ba_interval") && i + 1 < argc)
            c.ba_interval = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--global_ba_interval") && i + 1 < argc)
            c.global_ba_interval = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--map_hygiene"))
            c.map_hygiene = 1;
        else if (!strcmp(argv[i], "--kf_warmup_frames") && i + 1 < argc)
            c.kf_warmup_frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--first_kf_observations"))
            c.first_kf_observations = 1;
        else if (!strcmp(argv[i], "--unique_kf_observations"))
            c.unique_kf_observations = 1;
        else if (!strcmp(argv[i], "--essential_cheirality_max") && i + 1 < argc)
            c.essential_cheirality_max = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tri_min_parallax_deg") && i + 1 < argc)
            c.tri_min_parallax_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_reproj_px") && i + 1 < argc)
            c.tri_max_reproj_px = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_depth") && i + 1 < argc)
            c.tri_max_depth = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_depth_ratio") && i + 1 < argc)
            c.tri_max_depth_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_source_kf_gap") && i + 1 < argc)
            c.tri_source_kf_gap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_pred_reproj_gate") && i + 1 < argc)
            c.pnp_pred_reproj_gate = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_quality_gate_px") && i + 1 < argc)
            c.pnp_quality_gate_px = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_quality_min_obs") && i + 1 < argc)
            c.pnp_quality_min_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_quality_window") && i + 1 < argc)
            c.pnp_quality_window = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--obs_stat_gate_px") && i + 1 < argc)
            c.obs_stat_gate_px = atof(argv[++i]);
        else if (!strcmp(argv[i], "--obs_stat_min_good") && i + 1 < argc)
            c.obs_stat_min_good = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--obs_stat_max_bad_ratio") && i + 1 < argc)
            c.obs_stat_max_bad_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_fallback"))
            c.pnp_p3p_fallback = 1;
        else if (!strcmp(argv[i], "--pnp_p3p_observe"))
            c.pnp_p3p_observe = 1;
        else if (!strcmp(argv[i], "--pnp_solver") && i + 1 < argc) {
            const char *name = argv[++i];
            if (!strcmp(name, "dlt"))
                c.pnp_p3p_fallback = 0;
            else if (!strcmp(name, "p3p-numeric"))
                c.pnp_p3p_fallback = 1;
            else
                fprintf(stderr, "unknown --pnp_solver %s (expected dlt or p3p-numeric)\n", name);
        }
        else if (!strcmp(argv[i], "--pnp_p3p_iterations") && i + 1 < argc)
            c.pnp_p3p_iterations = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_max_jump") && i + 1 < argc)
            c.pnp_p3p_max_jump = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_min_inl2") && i + 1 < argc)
            c.pnp_p3p_min_inl2 = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_min_gain") && i + 1 < argc)
            c.pnp_p3p_min_gain = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_max_mederr") && i + 1 < argc)
            c.pnp_p3p_max_mederr = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_min_posz") && i + 1 < argc)
            c.pnp_p3p_min_posz = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_iters") && i + 1 < argc)
            c.pnp_dlt_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_pretest") && i + 1 < argc)
            c.pnp_dlt_pretest = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_pretest_margin") && i + 1 < argc)
            c.pnp_dlt_pretest_margin = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ffmpeg_gray"))
            c.ffmpeg_gray = 1;
        else if (!strcmp(argv[i], "--fast_corners"))
            c.fast_corners = 1;
        else if (!strcmp(argv[i], "--distributed_features"))
            c.distributed_features = 1;
        else if (!strcmp(argv[i], "--pyramid_features"))
            c.pyramid_features = 1;
        else if (!strcmp(argv[i], "--lk_iters") && i + 1 < argc)
            c.lk_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--lk_back_iters") && i + 1 < argc)
            c.lk_back_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pose_lm_iters") && i + 1 < argc)
            c.pose_lm_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--essential_iters") && i + 1 < argc)
            c.essential_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--joint_ba"))
            c.joint_ba = 1;
        else if (argv[i][0] != '-')
            c.video_path = argv[i];
    }
    if (c.proc_w < 64)
        c.proc_w = 64;
    if (c.proc_h < 64)
        c.proc_h = 64;
    if (c.kf_min_inliers < 1)
        c.kf_min_inliers = 1;
    if (c.kf_period < 1)
        c.kf_period = 1;
    if (c.kf_min_interval < 0)
        c.kf_min_interval = 0;
    if (c.max_points < 50)
        c.max_points = 50;
    if (c.new_point_obs < 0)
        c.new_point_obs = 0;
    if (c.pnp_min_obs < 0)
        c.pnp_min_obs = 0;
    if (c.pnp_start_frame < 0)
        c.pnp_start_frame = 0;
    if (c.delayed_init_frames < 0)
        c.delayed_init_frames = 0;
    if (c.candidate_min_obs < 2)
        c.candidate_min_obs = 2;
    if (c.candidate_min_age < 1)
        c.candidate_min_age = 1;
    if (c.candidate_grid_cols < 1)
        c.candidate_grid_cols = 1;
    if (c.candidate_grid_rows < 1)
        c.candidate_grid_rows = 1;
    if (c.candidate_promote_per_cell < 0)
        c.candidate_promote_per_cell = 0;
    if (c.candidate_max_fb_err < 0.0)
        c.candidate_max_fb_err = 0.0;
    if (c.candidate_min_disp < 0.0)
        c.candidate_min_disp = 0.0;
    if (c.candidate_max_disp < 0.0)
        c.candidate_max_disp = 0.0;
    if (c.descriptor_admission_max_hamming < 0)
        c.descriptor_admission_max_hamming = 0;
    if (c.descriptor_admission_ratio < 0.0)
        c.descriptor_admission_ratio = 0.0;
    if (c.descriptor_admission_ratio > 1.0)
        c.descriptor_admission_ratio = 1.0;
    if (c.descriptor_primary_map_cap < 0)
        c.descriptor_primary_map_cap = 0;
    if (c.descriptor_source_kf_gap < 0)
        c.descriptor_source_kf_gap = 0;
    if (c.kf_warmup_frames < 0)
        c.kf_warmup_frames = 0;
    if (c.essential_cheirality_max < 0)
        c.essential_cheirality_max = 32;
    if (c.tri_min_parallax_deg < 0.0)
        c.tri_min_parallax_deg = 0.0;
    if (c.tri_max_reproj_px < 0.0)
        c.tri_max_reproj_px = 0.0;
    if (c.tri_max_depth < 0.0)
        c.tri_max_depth = 0.0;
    if (c.tri_max_depth_ratio < 0.0)
        c.tri_max_depth_ratio = 0.0;
    if (c.tri_source_kf_gap < 0)
        c.tri_source_kf_gap = 0;
    if (c.candidate_tracks) {
        if (c.tri_min_parallax_deg == 0.0)
            c.tri_min_parallax_deg = 0.5;
        if (c.tri_max_reproj_px == 0.0)
            c.tri_max_reproj_px = 8.0;
    }
    if (c.pnp_pred_reproj_gate < 0.0)
        c.pnp_pred_reproj_gate = 0.0;
    if (c.pnp_quality_gate_px < 0.0)
        c.pnp_quality_gate_px = 0.0;
    if (c.pnp_quality_min_obs < 1)
        c.pnp_quality_min_obs = 1;
    if (c.pnp_quality_window < 1)
        c.pnp_quality_window = 1;
    if (c.obs_stat_gate_px < 0.0)
        c.obs_stat_gate_px = 0.0;
    if (c.obs_stat_min_good < 0)
        c.obs_stat_min_good = 0;
    if (c.obs_stat_max_bad_ratio < 0.0)
        c.obs_stat_max_bad_ratio = 0.0;
    if (c.obs_stat_max_bad_ratio > 1.0)
        c.obs_stat_max_bad_ratio = 1.0;
    if (c.pnp_p3p_iterations < 1)
        c.pnp_p3p_iterations = 1;
    if (c.pnp_p3p_max_jump < 0.0)
        c.pnp_p3p_max_jump = 0.0;
    if (c.pnp_p3p_min_inl2 < 0)
        c.pnp_p3p_min_inl2 = 0;
    if (c.pnp_p3p_max_mederr < 0.0)
        c.pnp_p3p_max_mederr = 0.0;
    if (c.pnp_p3p_min_posz < 0.0)
        c.pnp_p3p_min_posz = 0.0;
    if (c.pnp_dlt_iters < 1)
        c.pnp_dlt_iters = 1;
    if (c.pnp_dlt_pretest < 0)
        c.pnp_dlt_pretest = 0;
    if (c.pnp_dlt_pretest_margin < 0)
        c.pnp_dlt_pretest_margin = 0;
    if (c.lk_iters < 1)
        c.lk_iters = 1;
    if (c.lk_back_iters < 0)
        c.lk_back_iters = 0;
    if (c.pose_lm_iters < 1)
        c.pose_lm_iters = 1;
    if (c.essential_iters < 1)
        c.essential_iters = 1;
    if (c.e_shape_max_matches < 0)
        c.e_shape_max_matches = 0;
    if (c.e_shape_grid_cap < 0)
        c.e_shape_grid_cap = 0;
    if (c.e_shape_max_disp < 0.0)
        c.e_shape_max_disp = 0.0;
    if (c.e_shape_max_fb_err < 0.0)
        c.e_shape_max_fb_err = 0.0;
    if (c.e_shape_target_disp < 0.0)
        c.e_shape_target_disp = 0.0;
    if (c.anchor_max_features < 0)
        c.anchor_max_features = 0;
    if (c.anchor_max_hamming < 0)
        c.anchor_max_hamming = 0;
    if (c.anchor_ratio < 0.0)
        c.anchor_ratio = 0.0;
    if (c.anchor_ratio > 1.0)
        c.anchor_ratio = 1.0;
    if (c.admission_target_disp < 0.0)
        c.admission_target_disp = 0.0;
    if (c.admission_fb_weight < 0.0)
        c.admission_fb_weight = 0.0;
    if (c.admission_max_new_points < 0)
        c.admission_max_new_points = 0;
    if (c.admission_grid_cap < 0)
        c.admission_grid_cap = 0;
    return c;
}

int main(int argc, char **argv) {
    Config cfg = parse_args(argc, argv);
    brief_init_pattern();
    g_oriented_brief = cfg.oriented_brief;
    FFmpegCap *cap = ffmpeg_open(cfg.video_path, cfg.proc_w, cfg.proc_h, cfg.ffmpeg_gray);
    if (!cap) {
        fprintf(stderr, "Failed pipe.\n");
        return 1;
    }
    int w = cap->w, h = cap->h;
    unsigned char *raw = (unsigned char *)malloc((size_t)cap->bytes_per_frame);
    if (!raw) {
        fprintf(stderr, "out of memory\n");
        ffmpeg_close(cap);
        return 1;
    }
    ImageGray pgray_img = image_gray_alloc_zero(w, h);
    ImageGray cgray_img = image_gray_alloc_zero(w, h);
    ImageGray cblur_img = image_gray_alloc_zero(w, h);
    unsigned char *pgray = pgray_img.data;
    unsigned char *cgray = cgray_img.data;
    unsigned char *cblur = cblur_img.data;
    FrameLite prev = {0}, curr = {0};
    FrameStatVec stats = {0};
    ProfileStatVec profile = {0};
    LKScratch lk_scratch = {0};
    Pose lkf_pose, last_rel;
    pose_identity(&lkf_pose);
    pose_identity(&last_rel);
    KFDB kf_db = {0};
    Map map = {0};
    CandidateVec candidates = {0};
    AnchorSet anchors = {0};
    int frame_id = 0, pts = 0, tri = 0;
    double start = now_seconds();
    double fx = 525.0 * ((double)w / 640.0), fy = 525.0 * ((double)h / 480.0);
    double cx = 319.5 * ((double)w / 640.0), cy = 239.5 * ((double)h / 480.0);
    FILE *pnp_dump = NULL;
    if (cfg.pnp_dump) {
        ensure_parent_dir(cfg.pnp_dump);
        pnp_dump = fopen(cfg.pnp_dump, "w");
        if (!pnp_dump) {
            fprintf(stderr, "Failed to open PnP dump: %s\n", cfg.pnp_dump);
            return 1;
        }
    }
    FILE *track_dump = NULL;
    if (cfg.track_dump) {
        ensure_parent_dir(cfg.track_dump);
        track_dump = fopen(cfg.track_dump, "w");
        if (!track_dump) {
            fprintf(stderr, "Failed to open track dump: %s\n", cfg.track_dump);
            return 1;
        }
        fprintf(track_dump,
                "frame_id,matches,tracked,linked,linked_valid,mean_fb_err,mean_disp,min_x,min_y,max_x,max_y,grid_8x6\n");
    }
    FILE *map_admission_dump = NULL;
    if (cfg.map_admission_dump) {
        ensure_parent_dir(cfg.map_admission_dump);
        map_admission_dump = fopen(cfg.map_admission_dump, "w");
        if (!map_admission_dump) {
            fprintf(stderr, "Failed to open map admission dump: %s\n", cfg.map_admission_dump);
            return 1;
        }
        fprintf(map_admission_dump,
                "frame_id,candidates,accepted,baseline,mean_reproj,mean_parallax,mean_depth,min_depth,max_depth\n");
    }
    FILE *e_inlier_dump = NULL;
    if (cfg.e_inlier_dump) {
        ensure_parent_dir(cfg.e_inlier_dump);
        e_inlier_dump = fopen(cfg.e_inlier_dump, "w");
        if (!e_inlier_dump) {
            fprintf(stderr, "Failed to open E inlier dump: %s\n", cfg.e_inlier_dump);
            return 1;
        }
        fprintf(e_inlier_dump,
                "frame_id,source,match_idx,query_idx,train_idx,x1,y1,x2,y2,dx,dy,disp,score,linked_pt,fb_err,track_disp,grid_8x6\n");
    }
    while (frame_id < (int)(25.0 * cfg.seconds)) {
        if (now_seconds() - start > cfg.timeout)
            break;
        ProfileStat prof = {0};
        prof.frame_id = frame_id;
        double t0 = now_seconds();
        if (!ffmpeg_read(cap, raw))
            break;
        prof.decode += now_seconds() - t0;
        t0 = now_seconds();
        if (cfg.ffmpeg_gray)
            memcpy(cgray, raw, (size_t)w * h);
        else
            bgr_to_gray(raw, w, h, cgray);
        blur_3x3(cgray, w, h, cblur);
        prof.gray_blur += now_seconds() - t0;
        MatchVec matches = {0};
        MatchVec shaped_e_matches = {0};
        MatchVec anchor_matches = {0};
        AnchorSet cur_anchors = {0};
        const MatchVec *active_e_matches = &matches;
        const CornerVec *active_e_prev = NULL;
        const CornerVec *active_e_cur = NULL;
        const Pose *active_e_prev_pose = NULL;
        CornerVec tracked = {0};
        Pose pose, rel;
        unsigned char *mask = NULL;
        int inl = 0, mkf = 0, added = 0, method = 0;
        int tracked_count = 0, linked_points = 0, linked_before_relink = 0, relinked_points = 0;
        int pnp_inliers = 0, pred_lm_inliers = 0, e_inliers = 0;
        int pnp_fallback_used = 0;
        PnPProbe pnp_p3p_probe = {0};
        double trans_jump = 0.0;
        if (frame_id == 0) {
            t0 = now_seconds();
            extract_features_pure(&cfg, cblur, w, h, &curr.corners, cfg.max_points);
            prof.feature += now_seconds() - t0;
            mkf = 1;
            pose_identity(&pose);
        } else {
            active_e_prev = &prev.corners;
            active_e_cur = &tracked;
            active_e_prev_pose = &prev.pose;
            Pose predicted;
            pose_compose_relative(&last_rel, &prev.pose, &predicted);
            t0 = now_seconds();
            track_corners_pure_lk(pgray, cblur, w, h, &prev.corners, &tracked, &matches,
                                  &lk_scratch, cfg.lk_iters, cfg.lk_back_iters);
            prof.lk += now_seconds() - t0;
            tracked_count = tracked.size;
            if (cfg.shape_e_inliers) {
                build_shaped_e_matches(&cfg, &prev.corners, &tracked, &matches, w, h,
                                       &shaped_e_matches);
                active_e_matches = &shaped_e_matches;
            }
            if (cfg.anchor_e_pose && anchors.corners.size >= 8) {
                CornerVec anchor_cur_corners = {0};
                extract_features_pure(&cfg, cblur, w, h, &anchor_cur_corners,
                                      cfg.anchor_max_features);
                anchor_set_build(&cur_anchors, cblur, w, h, &anchor_cur_corners, &predicted,
                                 frame_id, cfg.anchor_max_features);
                free(anchor_cur_corners.data);
                match_anchor_sets(&anchors, &cur_anchors, cfg.anchor_max_hamming,
                                  cfg.anchor_ratio, &anchor_matches);
                if (anchor_matches.size >= 8) {
                    active_e_prev = &anchors.corners;
                    active_e_cur = &cur_anchors.corners;
                    active_e_prev_pose = &anchors.pose;
                    active_e_matches = &anchor_matches;
                }
            }

            int _nlink = 0;
            for (int _i = 0; _i < tracked.size; _i++)
                if (tracked.data[_i].pt_idx != -1)
                    _nlink++;
            linked_before_relink = _nlink;
            linked_points = _nlink;
            if (_nlink < 50 && map.size > 100) {
                t0 = now_seconds();
                relinked_points = brief_relink(cblur, w, h, &tracked, &map);
                prof.relink += now_seconds() - t0;
            }
            linked_points += relinked_points;
            if (cfg.pnp_pred_reproj_gate > 0.0) {
                (void)gate_links_by_pose(&map, &tracked, fx, fy, cx, cy, &predicted,
                                         cfg.pnp_pred_reproj_gate);
                linked_points = 0;
                for (int i = 0; i < tracked.size; i++)
                    if (tracked.data[i].pt_idx >= 0)
                        linked_points++;
            }
            if (map.size > 1000 && linked_points >= 12) {
                Pose pred_lm = predicted;
                t0 = now_seconds();
                refine_pose_lm(&map, &tracked, fx, fy, cx, cy, cfg.pose_lm_iters, &pred_lm);
                prof.pred_lm += now_seconds() - t0;
                pred_lm_inliers = count_pose_inliers(&map, &tracked, fx, fy, cx, cy, &pred_lm);
            }
            Pose pnp_pose_for_dump;
            int has_pnp_pose_for_dump = 0;
            unsigned char *pnp_point_ok = NULL;
            int pnp_ok = 0;
            if (frame_id >= cfg.pnp_start_frame) {
                pnp_point_ok = build_pnp_quality_mask(
                    &kf_db, &map, fx, fy, cx, cy, cfg.pnp_quality_gate_px,
                    cfg.pnp_quality_min_obs, cfg.pnp_quality_window);
                unsigned char *obs_stat_ok = build_observation_stat_mask(
                    &map, cfg.obs_stat_min_good, cfg.obs_stat_max_bad_ratio);
                if (pnp_point_ok && obs_stat_ok) {
                    for (int i = 0; i < map.size; i++)
                        pnp_point_ok[i] = pnp_point_ok[i] && obs_stat_ok[i];
                    free(obs_stat_ok);
                } else if (obs_stat_ok) {
                    pnp_point_ok = obs_stat_ok;
                }
            }
            t0 = now_seconds();
            if (frame_id >= cfg.pnp_start_frame)
                pnp_ok = estimate_pose_PnP(&map, &tracked, fx, fy, cx, cy, cfg.pnp_dlt_iters,
                                           cfg.pnp_dlt_pretest, cfg.pnp_dlt_pretest_margin,
                                           cfg.pnp_min_obs, pnp_point_ok, &pose, &pnp_inliers);
            free(pnp_point_ok);
            if (!pnp_ok && (cfg.pnp_p3p_fallback || cfg.pnp_p3p_observe)) {
                Pose p3p_pose;
                int p3p_inliers = 0;
                int p3p_ok = estimate_pose_PnP_p3p_numeric(
                    &map, &tracked, fx, fy, cx, cy, &predicted, pnp_inliers,
                    cfg.pnp_p3p_iterations, cfg.pnp_p3p_max_jump, cfg.pnp_p3p_min_inl2,
                    cfg.pnp_p3p_min_gain, cfg.pnp_p3p_max_mederr, cfg.pnp_p3p_min_posz,
                    &p3p_pose, &p3p_inliers, &pnp_p3p_probe);
                if (cfg.pnp_p3p_fallback && p3p_ok) {
                    pose = p3p_pose;
                    pnp_inliers = p3p_inliers;
                    pnp_ok = 1;
                    pnp_fallback_used = 1;
                }
            }
            if (pnp_ok) {
                prof.pnp += now_seconds() - t0;
                inl = pnp_inliers;
                pnp_pose_for_dump = pose;
                has_pnp_pose_for_dump = 1;
                t0 = now_seconds();
                refine_pose_lm(&map, &tracked, fx, fy, cx, cy, cfg.pose_lm_iters, &pose);
                prof.pose_lm += now_seconds() - t0;
                method = 2;
            } else {
                prof.pnp += now_seconds() - t0;
                t0 = now_seconds();
                int e_ok = estimate_pose_E(active_e_prev, active_e_cur, active_e_matches,
                                           fx, fy, cx, cy, &rel,
                                           cfg.essential_iters, cfg.essential_cheirality_max,
                                           &mask, &inl);
                if (e_ok)
                    write_e_inlier_pairs(e_inlier_dump, frame_id,
                                         active_e_cur == &tracked ? "pose" : "anchor_pose",
                                         active_e_prev, active_e_cur, active_e_matches, mask, w, h);
                prof.essential += now_seconds() - t0;
                if (e_ok) {
                    e_inliers = inl;
                    pose_compose_relative(&rel, active_e_prev_pose, &pose);
                    t0 = now_seconds();
                    refine_pose_lm(&map, &tracked, fx, fy, cx, cy, cfg.pose_lm_iters, &pose);
                    prof.pose_lm += now_seconds() - t0;
                    method = 1;
                } else {
                    pose = predicted;
                    t0 = now_seconds();
                    refine_pose_lm(&map, &tracked, fx, fy, cx, cy, cfg.pose_lm_iters, &pose);
                    prof.pose_lm += now_seconds() - t0;
                    method = 3;
                }
            }
            write_track_summary(track_dump, frame_id, &tracked, &matches, &map, w, h);
            dump_pnp_frame(pnp_dump, frame_id, &map, &tracked, fx, fy, cx, cy, &predicted,
                           has_pnp_pose_for_dump ? &pnp_pose_for_dump : NULL, pnp_inliers,
                           has_pnp_pose_for_dump ? &pose : NULL);
            mkf = should_make_keyframe(&cfg, frame_id, inl, linked_points,
                                       rotation_degrees_between(&lkf_pose, &pose));
            int kf_min_interval = keyframe_interval_for_frame(&cfg, frame_id);
            if (mkf && kf_min_interval > 0 && kf_db.size > 0) {
                int last_kf_frame = kf_db.data[kf_db.size - 1].frame_id;
                if (frame_id - last_kf_frame < kf_min_interval)
                    mkf = 0;
            }
            if (mkf) {
                if (inl >= 8) {
                    t0 = now_seconds();
	                    int obs_seen_cap = map.size;
	                    unsigned char *obs_seen = cfg.unique_kf_observations && obs_seen_cap > 0
	                                                  ? calloc((size_t)obs_seen_cap, 1)
	                                                  : NULL;
	                    CandidatePromotionVec promotions = {0};
	                    AdmissionCandidateVec admission_batch = {0};
	                    int adm_candidates = 0, adm_accepted = 0;
	                    double adm_sum_reproj = 0.0, adm_sum_parallax = 0.0, adm_sum_depth = 0.0;
	                    double adm_min_depth = DBL_MAX, adm_max_depth = 0.0;
	                    int adm_cell_counts[ADMISSION_GRID_CELLS] = {0};
	                    Pose admission_pose = pose;
	                    Pose admission_rel;
	                    int has_admission_rel = 0;
	                    const MatchVec *admission_e_matches =
	                        cfg.shape_e_inliers ? &shaped_e_matches : &matches;
	                    unsigned char *admission_mask =
	                        (active_e_prev == &prev.corners && active_e_cur == &tracked &&
	                         active_e_matches == admission_e_matches) ? mask : NULL;
	                    unsigned char *owned_admission_mask = NULL;
	                    if (cfg.triangulate_with_e_pose || cfg.triangulate_relative_frame ||
	                        active_e_cur != &tracked) {
	                        int adm_inl = 0;
	                        if (estimate_pose_E(&prev.corners, &tracked, admission_e_matches,
	                                            fx, fy, cx, cy,
	                                            &admission_rel, cfg.essential_iters,
	                                            cfg.essential_cheirality_max,
	                                            &owned_admission_mask, &adm_inl)) {
	                            write_e_inlier_pairs(e_inlier_dump, frame_id, "admission_lk",
	                                                 &prev.corners, &tracked, admission_e_matches,
	                                                 owned_admission_mask, w, h);
	                            has_admission_rel = 1;
	                            if (cfg.triangulate_with_e_pose || cfg.triangulate_relative_frame)
	                                pose_compose_relative(&admission_rel, &prev.pose, &admission_pose);
	                            admission_mask = owned_admission_mask;
	                        } else {
	                            free(owned_admission_mask);
	                            owned_admission_mask = NULL;
	                        }
	                    }
	                    AdmissionOrder *admission_order = NULL;
	                    if (admission_e_matches->size > 0) {
	                        admission_order = (AdmissionOrder *)malloc(
	                            (size_t)admission_e_matches->size * sizeof(AdmissionOrder));
	                        if (!admission_order) {
	                            fprintf(stderr, "out of memory\n");
	                            exit(1);
	                        }
	                        for (int j = 0; j < admission_e_matches->size; j++) {
	                            double score = (double)j;
	                            if (cfg.admission_ranked) {
	                                Match am = admission_e_matches->data[j];
	                                if (am.train_idx >= 0 && am.train_idx < tracked.size) {
	                                    Corner cc = tracked.data[am.train_idx];
	                                    score = fabs((double)cc.track_disp -
	                                                 cfg.admission_target_disp) +
	                                            cfg.admission_fb_weight * (double)cc.fb_err;
	                                } else {
	                                    score = 1.0e30;
	                                }
	                            }
	                            admission_order[j] = (AdmissionOrder){j, score};
	                        }
	                        if (cfg.admission_ranked && admission_e_matches->size > 1)
	                            qsort(admission_order, (size_t)admission_e_matches->size,
	                                  sizeof(AdmissionOrder), admission_order_cmp_score_asc);
	                    }
	                    for (int oi = 0; oi < admission_e_matches->size; oi++) {
	                        int j = admission_order ? admission_order[oi].match_idx : oi;
                        double X[3];
                        if (!admission_mask || !admission_mask[j])
                            continue;
	                        Match am = admission_e_matches->data[j];
	                        if (tracked.data[am.train_idx].pt_idx == -1) {
	                            if (config_suppresses_lk_landmarks(&cfg))
	                                continue;
	                            if (!config_allows_new_landmarks(&cfg, frame_id))
	                                continue;
	                            adm_candidates++;
		                            Corner pc = prev.corners.data[am.query_idx];
		                            Corner cc = tracked.data[am.train_idx];
	                            if (cfg.admission_batch_deferred && pc.cand_idx >= 0 &&
	                                pc.cand_idx < candidates.size &&
	                                candidates.data[pc.cand_idx].valid) {
	                                int ci = pc.cand_idx;
	                                candidates.data[ci].last_frame = frame_id;
	                                candidates.data[ci].last_corner = cc;
	                                if (candidates.data[ci].obs < 1000000)
	                                    candidates.data[ci].obs++;
	                                tracked.data[am.train_idx].cand_idx = ci;
	                                candidate_add_observation(&candidates.data[ci], frame_id,
	                                                          &admission_pose, cc);
	                                double Xc[3], score = 0.0;
	                                if (candidate_score_if_ready(&candidates, ci, frame_id,
	                                                             &cfg, fx, fy, cx, cy, Xc,
	                                                             &score)) {
	                                    Brief256 _d = candidates.data[ci].desc;
	                                    int map_idx = map.size;
	                                    MapPoint _mp = {Xc[0], Xc[1], Xc[2],
	                                                    cfg.new_point_obs > 2 ? cfg.new_point_obs : 2,
	                                                    0,
	                                                    0,
	                                                    _d};
	                                    map_push(&map, _mp);
	                                    candidates.data[ci].valid = 0;
	                                    tracked.data[am.train_idx].pt_idx = map_idx;
	                                    tracked.data[am.train_idx].cand_idx = -1;
	                                    pts++;
	                                    added++;
	                                    tri++;
	                                    adm_accepted++;
	                                }
	                                continue;
	                            }
	                            int adm_cell = -1;
	                            if (!cfg.candidate_tracks) {
	                                adm_cell = image_grid_cell(cc.x, cc.y, w, h,
	                                                           ADMISSION_GRID_COLS,
	                                                           ADMISSION_GRID_ROWS);
	                            }
	                            if (!cfg.candidate_tracks && !cfg.admission_batch_ranked) {
	                                if (cfg.admission_max_new_points > 0 &&
	                                    adm_accepted >= cfg.admission_max_new_points)
	                                    continue;
	                                if (cfg.admission_grid_cap > 0 &&
	                                    adm_cell_counts[adm_cell] >= cfg.admission_grid_cap)
	                                    continue;
	                            }
		                            if (cfg.candidate_tracks) {
	                                if ((cfg.candidate_max_fb_err > 0.0 &&
	                                     cc.fb_err > cfg.candidate_max_fb_err) ||
	                                    (cfg.candidate_min_disp > 0.0 &&
	                                     cc.track_disp < cfg.candidate_min_disp) ||
	                                    (cfg.candidate_max_disp > 0.0 &&
	                                     cc.track_disp > cfg.candidate_max_disp))
	                                    continue;
	                                int ci = pc.cand_idx;
	                                if (ci < 0 || ci >= candidates.size || !candidates.data[ci].valid) {
	                                    TrackCandidate tc;
	                                    memset(&tc, 0, sizeof(tc));
	                                    tc.valid = 1;
	                                    tc.first_frame = frame_id - 1;
	                                    tc.last_frame = frame_id;
	                                    tc.obs = 2;
	                                    tc.anchor_pose = prev.pose;
	                                    tc.anchor_corner = pc;
	                                    tc.anchor_corner.pt_idx = -1;
	                                    tc.anchor_corner.cand_idx = -1;
	                                    tc.last_corner = cc;
	                                    candidate_add_observation(&tc, frame_id - 1, &prev.pose, pc);
	                                    compute_brief(pgray, w, h, pc.x, pc.y, &tc.desc);
	                                    ci = candidate_vec_push(&candidates, tc);
	                                } else {
	                                    candidates.data[ci].last_frame = frame_id;
	                                    candidates.data[ci].last_corner = cc;
	                                    if (candidates.data[ci].obs < 1000000)
	                                        candidates.data[ci].obs++;
	                                }
	                                tracked.data[am.train_idx].cand_idx = ci;
	                                if (cfg.candidate_promote_per_cell > 0) {
	                                    candidate_add_observation(&candidates.data[ci], frame_id,
	                                                              &admission_pose, cc);
	                                    double Xc[3], score = 0.0;
	                                    if (candidate_score_if_ready(&candidates, ci, frame_id,
	                                                                 &cfg, fx, fy, cx, cy, Xc,
	                                                                 &score)) {
	                                        int gx = (int)(cc.x * cfg.candidate_grid_cols / (double)w);
	                                        int gy = (int)(cc.y * cfg.candidate_grid_rows / (double)h);
	                                        if (gx < 0) gx = 0;
	                                        if (gy < 0) gy = 0;
	                                        if (gx >= cfg.candidate_grid_cols) gx = cfg.candidate_grid_cols - 1;
	                                        if (gy >= cfg.candidate_grid_rows) gy = cfg.candidate_grid_rows - 1;
	                                        CandidatePromotion cp = {ci,
	                                                                 am.train_idx,
	                                                                 gy * cfg.candidate_grid_cols + gx,
	                                                                 score,
	                                                                 {Xc[0], Xc[1], Xc[2]}};
	                                        candidate_promotion_vec_push(&promotions, cp);
	                                    }
	                                } else {
	                                    int promoted = promote_candidate_if_ready(
	                                        &candidates, ci, frame_id, &admission_pose, cc, fx, fy, cx, cy,
	                                        &cfg, &map, cblur, w, h, &pts, &tri);
	                                    if (promoted >= 0) {
	                                        tracked.data[am.train_idx].pt_idx = promoted;
	                                        tracked.data[am.train_idx].cand_idx = -1;
	                                        added++;
	                                    }
	                                }
	                            } else {
	                                Pose id_pose;
	                                const Pose *stat_p1 = &prev.pose, *stat_p2 = &admission_pose;
	                                double Xstat[3] = {0.0, 0.0, 0.0};
	                                int tri_ok = 0;
	                                if (cfg.triangulate_relative_frame && has_admission_rel) {
	                                    pose_identity(&id_pose);
	                                    tri_ok = triangulate_point(&id_pose, &admission_rel, pc, cc,
	                                                               fx, fy, cx, cy, Xstat);
	                                    if (tri_ok)
	                                        camera_point_to_world(&prev.pose, Xstat, X);
	                                    stat_p1 = &id_pose;
	                                    stat_p2 = &admission_rel;
	                                } else {
	                                    tri_ok = triangulate_point(&prev.pose, &admission_pose, pc, cc,
	                                                               fx, fy, cx, cy, X);
	                                    memcpy(Xstat, X, sizeof(Xstat));
	                                }
	                                if (!tri_ok)
	                                    continue;
	                                if ((cfg.tri_min_parallax_deg > 0.0 ||
	                                     cfg.tri_max_reproj_px > 0.0 ||
	                                     cfg.tri_max_depth > 0.0 ||
	                                     cfg.tri_max_depth_ratio > 0.0) &&
	                                    !triangulation_quality_ok(
	                                        stat_p1, stat_p2, pc, cc, fx, fy, cx, cy, Xstat,
	                                        cfg.tri_min_parallax_deg,
	                                        cfg.tri_max_reproj_px,
	                                        cfg.tri_max_depth,
	                                        cfg.tri_max_depth_ratio))
	                                    continue;
	                                Brief256 _d = {{0, 0, 0, 0}};
	                                compute_brief(cblur, w, h, cc.x, cc.y, &_d);
	                                double z1 = 0.0, z2 = 0.0;
	                                double e1 = reprojection_error_xyz(stat_p1, Xstat, pc, fx, fy, cx, cy, &z1);
	                                double e2 = reprojection_error_xyz(stat_p2, Xstat, cc, fx, fy, cx, cy, &z2);
	                                double depth = 0.5 * (z1 + z2);
	                                double reproj = 0.5 * (e1 + e2);
	                                double parallax = triangulation_parallax_deg(stat_p1, stat_p2, Xstat);
	                                if (cfg.admission_finite_only &&
	                                    (!isfinite(e1) || !isfinite(e2) || !isfinite(depth) ||
	                                     !isfinite(X[0]) || !isfinite(X[1]) || !isfinite(X[2])))
	                                    continue;
	                                if (cfg.admission_batch_ranked) {
	                                    double score = isfinite(reproj) ? reproj : 1.0e30;
	                                    AdmissionCandidate ac = {am.query_idx,
	                                                             am.train_idx,
	                                                             adm_cell,
	                                                             score,
	                                                             reproj,
	                                                             parallax,
	                                                             depth,
	                                                             {X[0], X[1], X[2]},
	                                                             {Xstat[0], Xstat[1], Xstat[2]},
	                                                             _d,
	                                                             pc,
	                                                             cc,
	                                                             prev.pose,
	                                                             admission_pose};
	                                    admission_candidate_vec_push(&admission_batch, ac);
	                                    continue;
	                                }
	                                tracked.data[am.train_idx].pt_idx = map.size;
	                                tracked.data[am.train_idx].cand_idx = -1;
	                                if (cfg.first_kf_observations && kf_db.size == 0)
	                                    prev.corners.data[am.query_idx].pt_idx = map.size;
	                                adm_sum_reproj += reproj;
	                                adm_sum_parallax += parallax;
	                                adm_sum_depth += depth;
	                                if (depth < adm_min_depth)
	                                    adm_min_depth = depth;
	                                if (depth > adm_max_depth)
	                                    adm_max_depth = depth;
	                                adm_accepted++;
	                                if (adm_cell >= 0)
	                                    adm_cell_counts[adm_cell]++;
	                                MapPoint _mp = {X[0], X[1], X[2],
	                                                cfg.first_kf_observations && kf_db.size == 0
	                                                    ? 2
	                                                    : cfg.new_point_obs,
	                                                0,
	                                                0,
	                                                _d};
	                                map_push(&map, _mp);
	                                pts++;
	                                added++;
	                                tri++;
	                            }
	                        } else {
	                            int pi = tracked.data[am.train_idx].pt_idx;
	                            if (!obs_seen || pi >= obs_seen_cap || !obs_seen[pi]) {
	                                map.data[pi].obs++;
                                if (obs_seen && pi < obs_seen_cap)
                                    obs_seen[pi] = 1;
	                            }
	                        }
	                    }
	                    if (admission_batch.size > 0) {
	                        qsort(admission_batch.data, (size_t)admission_batch.size,
	                              sizeof(AdmissionCandidate), admission_candidate_cmp_score_asc);
	                        int batch_cap = inl > 0 ? ADMISSION_BATCH_INLIER_MULTIPLIER * inl
	                                                : admission_batch.size;
	                        if (cfg.admission_max_new_points > 0 &&
	                            cfg.admission_max_new_points < batch_cap)
	                            batch_cap = cfg.admission_max_new_points;
	                        int batch_added = 0;
	                        for (int bi = 0; bi < admission_batch.size && batch_added < batch_cap; bi++) {
	                            AdmissionCandidate *ac = &admission_batch.data[bi];
	                            if (!isfinite(ac->score) || !isfinite(ac->depth) ||
	                                !isfinite(ac->X[0]) || !isfinite(ac->X[1]) ||
	                                !isfinite(ac->X[2]))
	                                continue;
	                            if (ac->train_idx < 0 || ac->train_idx >= tracked.size ||
	                                tracked.data[ac->train_idx].pt_idx != -1)
	                                continue;
	                            if (cfg.admission_grid_cap > 0 && ac->cell >= 0 &&
	                                adm_cell_counts[ac->cell] >= cfg.admission_grid_cap)
	                                continue;
	                            if (cfg.admission_batch_deferred) {
	                                int ci = create_deferred_candidate(&candidates, frame_id,
	                                                                   &ac->prev_pose,
	                                                                   &ac->cur_pose,
	                                                                   ac->prev_corner,
	                                                                   ac->cur_corner,
	                                                                   &ac->desc);
	                                tracked.data[ac->train_idx].cand_idx = ci;
	                                if (ac->cell >= 0)
	                                    adm_cell_counts[ac->cell]++;
	                                batch_added++;
	                                continue;
	                            }
	                            int map_idx = map.size;
	                            tracked.data[ac->train_idx].pt_idx = map_idx;
	                            tracked.data[ac->train_idx].cand_idx = -1;
	                            if (cfg.first_kf_observations && kf_db.size == 0 &&
	                                ac->query_idx >= 0 && ac->query_idx < prev.corners.size)
	                                prev.corners.data[ac->query_idx].pt_idx = map_idx;
	                            adm_sum_reproj += ac->reproj;
	                            adm_sum_parallax += ac->parallax;
	                            adm_sum_depth += ac->depth;
	                            if (ac->depth < adm_min_depth)
	                                adm_min_depth = ac->depth;
	                            if (ac->depth > adm_max_depth)
	                                adm_max_depth = ac->depth;
	                            adm_accepted++;
	                            if (ac->cell >= 0)
	                                adm_cell_counts[ac->cell]++;
	                            MapPoint _mp = {ac->X[0], ac->X[1], ac->X[2],
	                                            cfg.first_kf_observations && kf_db.size == 0
	                                                ? 2
	                                                : cfg.new_point_obs,
	                                            0,
	                                            0,
	                                            ac->desc};
	                            map_push(&map, _mp);
	                            pts++;
	                            added++;
	                            tri++;
	                            batch_added++;
	                        }
	                    }
	                    free(admission_batch.data);
	                    free(admission_order);
	                    if (config_suppresses_lk_landmarks(&cfg)) {
	                        CornerVec desc_cur = {0};
	                        extract_features_pure(&cfg, cblur, w, h, &desc_cur, cfg.max_points);
	                        const CornerVec *desc_src_corners = &prev.corners;
	                        const unsigned char *desc_src_gray = pgray;
	                        const Pose *desc_src_pose = &prev.pose;
	                        const char *desc_dump_kind = "admission_desc";
	                        KFEntry *desc_src_kf = NULL;
	                        int desc_source_ready = cfg.descriptor_source_kf_gap <= 0;
	                        if (cfg.descriptor_primary_admission &&
	                            cfg.descriptor_source_kf_gap > 0 && kf_db.size > 0) {
	                            int src_idx = -1;
	                            for (int ki = kf_db.size - 1; ki >= 0; ki--) {
	                                if (frame_id - kf_db.data[ki].frame_id >=
	                                    cfg.descriptor_source_kf_gap) {
	                                    src_idx = ki;
	                                    break;
	                                }
	                            }
	                            if (src_idx >= 0) {
	                                KFEntry *src = &kf_db.data[src_idx];
	                                desc_src_corners = &src->corners;
	                                desc_src_gray = src->gray.data;
	                                desc_src_pose = &src->pose;
	                                desc_dump_kind = "admission_desc_kf";
	                                desc_src_kf = src;
	                                desc_source_ready = 1;
	                            }
	                        }
	                        if (cfg.descriptor_primary_admission &&
	                            cfg.delayed_init_frames > 0) {
	                            desc_source_ready = 0;
	                            if (frame_id >= cfg.delayed_init_frames && kf_db.size > 0) {
	                                KFEntry *src = &kf_db.data[0];
	                                desc_src_corners = &src->corners;
	                                desc_src_gray = src->gray.data;
	                                desc_src_pose = &src->pose;
	                                desc_dump_kind = "admission_desc_init";
	                                desc_src_kf = src;
	                                desc_source_ready = 1;
	                            }
	                        }
	                        MatchVec desc_matches = {0};
	                        if (desc_source_ready)
	                            build_brief_descriptor_matches(
	                                desc_src_gray, cblur, w, h, desc_src_corners, &desc_cur,
	                                cfg.descriptor_admission_max_hamming,
	                                cfg.descriptor_admission_ratio,
	                                cfg.descriptor_mutual_admission, &desc_matches);
	                        unsigned char *desc_mask = NULL;
	                        int desc_inl = 0;
	                        Pose desc_rel;
	                        int desc_e_ok = estimate_pose_E(
	                            desc_src_corners, &desc_cur, &desc_matches, fx, fy, cx, cy,
	                            &desc_rel, cfg.essential_iters, cfg.essential_cheirality_max,
	                            &desc_mask, &desc_inl);
	                        if (desc_e_ok) {
	                            write_e_inlier_pairs(e_inlier_dump, frame_id, desc_dump_kind,
	                                                 desc_src_corners, &desc_cur, &desc_matches,
	                                                 desc_mask, w, h);
	                            Pose desc_admission_pose = admission_pose;
	                            if (cfg.triangulate_with_e_pose ||
	                                cfg.descriptor_primary_admission)
	                                pose_compose_relative(&desc_rel, desc_src_pose,
	                                                      &desc_admission_pose);
	                            int desc_add_cap = INT_MAX;
	                            if (cfg.descriptor_primary_admission &&
	                                cfg.descriptor_primary_map_cap > 0) {
	                                desc_add_cap = cfg.descriptor_primary_map_cap - map.size;
	                                if (desc_add_cap < 0)
	                                    desc_add_cap = 0;
	                            }
	                            if (cfg.admission_max_new_points > 0 &&
	                                cfg.admission_max_new_points < desc_add_cap)
	                                desc_add_cap = cfg.admission_max_new_points;
	                            int desc_added = 0;
	                            for (int j = 0; j < desc_matches.size; j++) {
	                                if (!desc_mask || !desc_mask[j])
	                                    continue;
	                                if (desc_added >= desc_add_cap)
	                                    break;
	                                Match dm = desc_matches.data[j];
	                                if (dm.query_idx < 0 || dm.query_idx >= desc_src_corners->size ||
	                                    dm.train_idx < 0 || dm.train_idx >= desc_cur.size)
	                                    continue;
	                                if (desc_src_corners->data[dm.query_idx].pt_idx != -1)
	                                    continue;
	                                adm_candidates++;
	                                Corner pc = desc_src_corners->data[dm.query_idx];
	                                Corner cc = desc_cur.data[dm.train_idx];
	                                int cur_idx = find_near_corner(&tracked, cc,
	                                                               ADMISSION_MATCH_RADIUS_PX);
	                                if (cur_idx >= 0 && tracked.data[cur_idx].pt_idx != -1)
	                                    continue;
	                                double X[3];
	                                double Xstat[3] = {0.0, 0.0, 0.0};
	                                Pose id_pose;
	                                const Pose *stat_p1 = desc_src_pose,
	                                           *stat_p2 = &desc_admission_pose;
	                                int tri_ok = 0;
	                                if (cfg.triangulate_relative_frame) {
	                                    pose_identity(&id_pose);
	                                    tri_ok = triangulate_point(&id_pose, &desc_rel, pc, cc,
	                                                               fx, fy, cx, cy, Xstat);
	                                    if (tri_ok)
	                                        camera_point_to_world(desc_src_pose, Xstat, X);
	                                    stat_p1 = &id_pose;
	                                    stat_p2 = &desc_rel;
	                                } else {
	                                    tri_ok = triangulate_point(desc_src_pose, &desc_admission_pose,
	                                                               pc, cc, fx, fy, cx, cy, X);
	                                    memcpy(Xstat, X, sizeof(Xstat));
	                                }
	                                if (!tri_ok)
	                                    continue;
	                                if ((cfg.tri_min_parallax_deg > 0.0 ||
	                                     cfg.tri_max_reproj_px > 0.0 ||
	                                     cfg.tri_max_depth > 0.0 ||
	                                     cfg.tri_max_depth_ratio > 0.0) &&
	                                    !triangulation_quality_ok(
	                                        stat_p1, stat_p2, pc, cc, fx, fy, cx, cy, Xstat,
	                                        cfg.tri_min_parallax_deg,
	                                        cfg.tri_max_reproj_px,
	                                        cfg.tri_max_depth,
	                                        cfg.tri_max_depth_ratio))
	                                    continue;
	                                Brief256 _d = {{0, 0, 0, 0}};
	                                compute_brief(cblur, w, h, cc.x, cc.y, &_d);
	                                int map_idx = map.size;
	                                if (cur_idx < 0) {
	                                    cur_idx = tracked.size;
	                                    cc.pt_idx = -1;
	                                    cc.cand_idx = -1;
	                                    cc.fb_err = 0.0f;
	                                    cc.track_disp = 0.0f;
	                                    corner_vec_push(&tracked, cc);
	                                }
	                                tracked.data[cur_idx].pt_idx = map_idx;
	                                tracked.data[cur_idx].cand_idx = -1;
	                                if (cfg.first_kf_observations && kf_db.size == 0 &&
	                                    desc_src_corners == &prev.corners)
	                                    prev.corners.data[dm.query_idx].pt_idx = map_idx;
	                                if (desc_src_kf)
	                                    desc_src_kf->corners.data[dm.query_idx].pt_idx = map_idx;
	                                double z1 = 0.0, z2 = 0.0;
	                                double e1 = reprojection_error_xyz(stat_p1, Xstat, pc, fx, fy, cx, cy, &z1);
	                                double e2 = reprojection_error_xyz(stat_p2, Xstat, cc, fx, fy, cx, cy, &z2);
	                                double depth = 0.5 * (z1 + z2);
	                                if (cfg.admission_finite_only &&
	                                    (!isfinite(e1) || !isfinite(e2) || !isfinite(depth) ||
	                                     !isfinite(X[0]) || !isfinite(X[1]) || !isfinite(X[2])))
	                                    continue;
	                                adm_sum_reproj += 0.5 * (e1 + e2);
	                                adm_sum_parallax += triangulation_parallax_deg(stat_p1, stat_p2, Xstat);
	                                adm_sum_depth += depth;
	                                if (depth < adm_min_depth)
	                                    adm_min_depth = depth;
	                                if (depth > adm_max_depth)
	                                    adm_max_depth = depth;
	                                adm_accepted++;
	                                desc_added++;
	                                MapPoint _mp = {X[0], X[1], X[2],
	                                                cfg.first_kf_observations && kf_db.size == 0
	                                                    ? 2
	                                                    : cfg.new_point_obs,
	                                                0,
	                                                0,
	                                                _d};
	                                map_push(&map, _mp);
	                                pts++;
	                                added++;
	                                tri++;
	                            }
	                        }
	                        free(desc_mask);
	                        free(desc_matches.data);
	                        free(desc_cur.data);
	                    }
	                    Pose summary_id_pose;
	                    const Pose *summary_p1 = &prev.pose, *summary_p2 = &admission_pose;
	                    if (cfg.triangulate_relative_frame && has_admission_rel) {
	                        pose_identity(&summary_id_pose);
	                        summary_p1 = &summary_id_pose;
	                        summary_p2 = &admission_rel;
	                    }
	                    write_map_admission_summary(map_admission_dump, frame_id, adm_candidates,
	                                                adm_accepted, summary_p1, summary_p2,
	                                                fx, fy, cx, cy,
	                                                adm_sum_reproj, adm_sum_parallax,
	                                                adm_sum_depth, adm_min_depth, adm_max_depth);
	                    free(owned_admission_mask);
	                    if (promotions.size > 0) {
	                        int cells = cfg.candidate_grid_cols * cfg.candidate_grid_rows;
	                        int *cell_counts = calloc((size_t)cells, sizeof(int));
	                        qsort(promotions.data, (size_t)promotions.size,
	                              sizeof(CandidatePromotion), candidate_promotion_cmp_desc);
	                        for (int pi = 0; pi < promotions.size; pi++) {
	                            CandidatePromotion *cp = &promotions.data[pi];
	                            if (cp->cell < 0 || cp->cell >= cells ||
	                                cell_counts[cp->cell] >= cfg.candidate_promote_per_cell)
	                                continue;
	                            if (cp->cand_idx < 0 || cp->cand_idx >= candidates.size ||
	                                !candidates.data[cp->cand_idx].valid)
	                                continue;
	                            if (cp->corner_idx < 0 || cp->corner_idx >= tracked.size ||
	                                tracked.data[cp->corner_idx].pt_idx != -1)
	                                continue;
	                            Corner cc = tracked.data[cp->corner_idx];
	                            Brief256 _d = {{0, 0, 0, 0}};
	                            compute_brief(cblur, w, h, cc.x, cc.y, &_d);
	                            int map_idx = map.size;
	                            MapPoint _mp = {cp->X[0], cp->X[1], cp->X[2],
	                                            cfg.new_point_obs > 2 ? cfg.new_point_obs : 2,
	                                            0,
	                                            0,
	                                            _d};
	                            map_push(&map, _mp);
	                            candidates.data[cp->cand_idx].valid = 0;
	                            tracked.data[cp->corner_idx].pt_idx = map_idx;
	                            tracked.data[cp->corner_idx].cand_idx = -1;
	                            cell_counts[cp->cell]++;
	                            pts++;
	                            added++;
	                            tri++;
	                        }
	                        free(cell_counts);
	                    }
	                    free(promotions.data);
		                    free(obs_seen);
	                    if (cfg.tri_source_kf_gap > 0 && kf_db.size > 0 &&
	                        config_allows_new_landmarks(&cfg, frame_id) &&
	                        frame_id % cfg.tri_source_kf_gap == 0) {
	                        int src_idx = -1;
	                        for (int ki = kf_db.size - 1; ki >= 0; ki--) {
	                            if (frame_id - kf_db.data[ki].frame_id >= cfg.tri_source_kf_gap) {
	                                src_idx = ki;
	                                break;
	                            }
	                        }
	                        if (src_idx >= 0) {
	                            KFEntry *src = &kf_db.data[src_idx];
	                            CornerVec src_subset = {0};
	                            int src_cap = src->corners.size < 250 ? src->corners.size : 250;
	                            int *src_orig = malloc((size_t)src_cap * sizeof(int));
	                            for (int si = 0; si < src_cap; si++) {
	                                src_orig[si] = si;
	                                corner_vec_push(&src_subset, src->corners.data[si]);
	                            }
	                            CornerVec kf_tracked = {0};
	                            MatchVec kf_matches = {0};
	                            track_corners_pure_lk(src->gray.data, cblur, w, h, &src_subset,
	                                                  &kf_tracked, &kf_matches, &lk_scratch,
	                                                  cfg.lk_iters, cfg.lk_back_iters);
	                            unsigned char *src_mask = NULL;
	                            int src_inl = 0;
	                            Pose src_rel;
	                            int src_e_ok = estimate_pose_E(&src_subset, &kf_tracked,
	                                                          &kf_matches, fx, fy, cx, cy,
	                                                          &src_rel, cfg.essential_iters,
	                                                          cfg.essential_cheirality_max,
	                                                          &src_mask, &src_inl);
	                            if (src_e_ok && src_inl >= 8) {
	                                for (int j = 0; j < kf_matches.size; j++) {
	                                    if (!src_mask || !src_mask[j])
	                                        continue;
	                                    int qi = kf_matches.data[j].query_idx;
	                                    int ti = kf_matches.data[j].train_idx;
	                                    int orig_qi = src_orig[qi];
	                                    if (src->corners.data[orig_qi].pt_idx != -1)
	                                        continue;
	                                    Corner cur_corner = kf_tracked.data[ti];
	                                    int cur_idx = find_near_corner(&tracked, cur_corner,
	                                                                   ADMISSION_MATCH_RADIUS_PX);
	                                    if (cur_idx >= 0 && tracked.data[cur_idx].pt_idx != -1)
	                                        continue;
	                                    double X[3];
	                                    if (!triangulate_point(&src->pose, &pose,
	                                                           src->corners.data[orig_qi], cur_corner,
	                                                           fx, fy, cx, cy, X))
	                                        continue;
	                                    if ((cfg.tri_min_parallax_deg > 0.0 ||
	                                         cfg.tri_max_reproj_px > 0.0 ||
	                                         cfg.tri_max_depth > 0.0 ||
	                                         cfg.tri_max_depth_ratio > 0.0) &&
	                                        !triangulation_quality_ok(
	                                            &src->pose, &pose, src->corners.data[orig_qi],
	                                            cur_corner, fx, fy, cx, cy, X,
	                                            cfg.tri_min_parallax_deg,
	                                            cfg.tri_max_reproj_px,
	                                            cfg.tri_max_depth,
	                                            cfg.tri_max_depth_ratio))
	                                        continue;
	                                    if (cur_idx < 0) {
	                                        cur_idx = tracked.size;
	                                        cur_corner.pt_idx = -1;
	                                        cur_corner.cand_idx = -1;
	                                        cur_corner.fb_err = 0.0f;
	                                        cur_corner.track_disp = 0.0f;
	                                        corner_vec_push(&tracked, cur_corner);
	                                    }
	                                    Brief256 _d = {{0, 0, 0, 0}};
	                                    compute_brief(cblur, w, h, cur_corner.x, cur_corner.y, &_d);
	                                    int new_idx = map.size;
	                                    tracked.data[cur_idx].pt_idx = new_idx;
	                                    src->corners.data[orig_qi].pt_idx = new_idx;
	                                    MapPoint _mp = {X[0], X[1], X[2],
	                                                    cfg.new_point_obs > 2 ? cfg.new_point_obs : 2,
	                                                    0,
	                                                    0,
	                                                    _d};
	                                    map_push(&map, _mp);
	                                    pts++;
	                                    added++;
	                                    tri++;
	                                }
	                            }
	                            free(src_mask);
	                            free(kf_matches.data);
	                            free(kf_tracked.data);
	                            free(src_orig);
	                            free(src_subset.data);
	                        }
	                    }
	                    prof.triangulate += now_seconds() - t0;
	                }
                if (tracked.size < (cfg.max_points * 3) / 5) {
                    t0 = now_seconds();
                    extract_features_pure(&cfg, cblur, w, h, &tracked, cfg.max_points);
                    prof.refill += now_seconds() - t0;
                }
                t0 = now_seconds();
                unsigned char thumb[256];
                gen_thumbnail(cgray, w, h, thumb);
                int lidx = find_loop_candidate(&kf_db, thumb, frame_id);
                if (lidx >= 0) {
                    Pose loop_pose;
                    if (verify_loop(&kf_db, lidx, cgray, w, h, fx, fy, cx, cy, &loop_pose)) {
                        printf("LOOP CLOSED: %d with %d\n", frame_id, kf_db.data[lidx].frame_id);
                        pose = loop_pose;
                    }
                }
                prof.loop += now_seconds() - t0;
                update_observation_stats(&map, &tracked, fx, fy, cx, cy, &pose,
                                         cfg.obs_stat_gate_px);
                if (cfg.first_kf_observations && kf_db.size == 0 && added > 0) {
                    KFEntry pe;
                    gen_thumbnail(pgray, w, h, pe.thumb);
                    pe.frame_id = frame_id - 1;
                    pe.pose = prev.pose;
                    pe.corners.size = prev.corners.size;
                    pe.corners.data = malloc((size_t)prev.corners.size * sizeof(Corner));
                    memcpy(pe.corners.data, prev.corners.data,
                           (size_t)prev.corners.size * sizeof(Corner));
                    pe.gray = image_gray_clone(pgray, w, h);
                    kfdb_push(&kf_db, pe);
                }
                KFEntry e;
                memcpy(e.thumb, thumb, 256);
                e.frame_id = frame_id;
                e.pose = pose;
                e.corners.size = tracked.size;
                e.corners.data = malloc(tracked.size * sizeof(Corner));
                memcpy(e.corners.data, tracked.data, tracked.size * sizeof(Corner));
                e.gray = image_gray_clone(cgray, w, h);
                kfdb_push(&kf_db, e);
                if (cfg.anchor_e_pose)
                    anchor_set_build(&anchors, cblur, w, h, &tracked, &pose, frame_id,
                                     cfg.anchor_max_features);
                t0 = now_seconds();
                if (cfg.ba_interval > 0 && kf_db.size % cfg.ba_interval == 0) {
                    if (cfg.joint_ba) {
                        joint_local_ba(&kf_db, &map, fx, fy, cx, cy);
                        int win_start = kf_db.size - 5;
                        if (win_start < 0) win_start = 0;
                        cull_map_points_window(&kf_db, &map, win_start, kf_db.size,
                                               fx, fy, cx, cy, 8.0);
                    } else {
                        local_ba(&kf_db, &map, fx, fy, cx, cy, cfg.pose_lm_iters);
                    }
                }
                if (cfg.map_hygiene && frame_id >= MAP_HYGIENE_START_FRAME) {
                    int win_start = kf_db.size - MAP_HYGIENE_WINDOW;
                    if (win_start < 0) win_start = 0;
                    cull_map_points_window(&kf_db, &map, win_start, kf_db.size,
                                           fx, fy, cx, cy, MAP_HYGIENE_REPROJ_PX);
                }
                prof.ba += now_seconds() - t0;
                if (cfg.global_ba_interval > 0 && kf_db.size > 0 &&
                    kf_db.size % cfg.global_ba_interval == 0) {
                    t0 = now_seconds();
                    global_ba(&kf_db, &map, fx, fy, cx, cy, 3, cfg.pose_lm_iters);
                    prof.global_ba += now_seconds() - t0;
                }
                if (added > 0 || frame_id == 1)
                    lkf_pose = pose;
            }
            curr.corners = tracked;
        }
        curr.pose = pose;
        if (frame_id > 0) {
            double pc[3], cc[3];
            camera_center_from_pose(&prev.pose, pc);
            camera_center_from_pose(&pose, cc);
            double dx = cc[0] - pc[0], dy = cc[1] - pc[1], dz = cc[2] - pc[2];
            trans_jump = sqrt(dx * dx + dy * dy + dz * dz);
        }
        if (frame_id > 0) {
            Pose inv_prev;
            pose_inverse(&prev.pose, &inv_prev);
            pose_compose_relative(&pose, &inv_prev, &last_rel);
        }
        double c[3];
        camera_center_from_pose(&pose, c);
        t0 = now_seconds();
        frame_stat_vec_push(&stats, (FrameStat){frame_id,
                                                inl,
                                                mkf,
                                                added,
                                                pts,
                                                method,
                                                tracked_count,
                                                linked_points,
                                                linked_before_relink,
                                                relinked_points,
                                                pnp_inliers,
                                                pred_lm_inliers,
                                                e_inliers,
                                                pnp_fallback_used,
                                                pnp_p3p_probe.attempted,
                                                pnp_p3p_probe.solved,
                                                pnp_p3p_probe.accepted,
                                                pnp_p3p_probe.inliers,
                                                pnp_p3p_probe.inliers2,
                                                pnp_p3p_probe.inliers3,
                                                pnp_p3p_probe.inliers5,
                                                pnp_p3p_probe.median_error,
                                                pnp_p3p_probe.positive_depth_ratio,
                                                pnp_p3p_probe.predicted_jump,
                                                {pnp_p3p_probe.center[0],
                                                 pnp_p3p_probe.center[1],
                                                 pnp_p3p_probe.center[2]},
                                                trans_jump,
                                                {c[0], c[1], c[2]}});
        profile_stat_vec_push(&profile, prof);
        profile.data[profile.size - 1].metrics += now_seconds() - t0;
        if ((frame_id + 1) % 10 == 0)
            printf("Frames=%d Pts=%d KF=%d Map=%d\n", frame_id + 1, pts, kf_db.size, map.size);
        memcpy(pgray, cblur, w * h);
        free(prev.corners.data);
        prev = curr;
        memset(&curr, 0, sizeof(curr));
        free(matches.data);
        free(shaped_e_matches.data);
        free(anchor_matches.data);
        anchor_set_free(&cur_anchors);
        free(mask);
        frame_id++;
    }
    const char *out = cfg.metrics_out ? cfg.metrics_out : "runs/pure_c_metrics.json";
    ensure_parent_dir(out);
    FILE *f = fopen(out, "wb");
    if (f) {
        write_metrics_json(f, &cfg, &stats, pts, tri, now_seconds() - start);
        fclose(f);
    }
    if (cfg.profile_out) {
        ensure_parent_dir(cfg.profile_out);
        FILE *pf = fopen(cfg.profile_out, "wb");
        if (pf) {
            write_profile_csv(pf, &profile);
            fclose(pf);
        }
    }
    if (pnp_dump)
        fclose(pnp_dump);
    if (track_dump)
        fclose(track_dump);
    if (map_admission_dump)
        fclose(map_admission_dump);
    if (e_inlier_dump)
        fclose(e_inlier_dump);
    ffmpeg_close(cap);
    free(raw);
    image_gray_free(&pgray_img);
    image_gray_free(&cgray_img);
    image_gray_free(&cblur_img);
    free(prev.corners.data);
    lk_scratch_free(&lk_scratch);
    free(profile.data);
    free(stats.data);
    free(candidates.data);
    anchor_set_free(&anchors);
    kfdb_free(&kf_db);
    free(map.data);
    return 0;
}
