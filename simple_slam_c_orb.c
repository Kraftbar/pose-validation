#include <float.h>
#include <math.h>
#include <omp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "pure_c_math.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Structs ---

typedef struct {
    const char *video_path;
    double seconds, timeout;
    const char *metrics_out;
    int kf_min_inliers;
    double kf_max_rot_deg;
    int max_points;
} Config;
typedef struct {
    uint64_t bits[4];
} OrbDescriptor;
typedef struct {
    float x, y;
    int pt_idx;
    float angle;
    int level;
    OrbDescriptor desc;
} Corner;
typedef struct {
    Corner *data;
    int size, cap;
} CornerVec;
typedef struct {
    double m[16];
} Pose;
typedef struct {
    double x, y, z;
    int obs;
    OrbDescriptor desc;
    int last_frame_id;
    bool is_bad;
} MapPoint;
typedef struct {
    MapPoint *data;
    int size, cap;
} Map;
typedef struct {
    int frame_id, inliers, is_keyframe, points_added, points_total, method;
    double xyz[3], rotation[9];
} FrameStat;
typedef struct {
    FrameStat *data;
    int size, cap;
} FrameStatVec;
typedef struct {
    int query_idx, train_idx;
    int dist;
} Match;
typedef struct {
    Match *data;
    int size, cap;
} MatchVec;

typedef struct {
    unsigned char *data;
    int w, h;
    float scale;
} ImageLevel;
typedef struct {
    ImageLevel levels[8];
    int num_levels;
} ImagePyramid;
typedef struct {
    int frame_id;
    Pose pose;
    CornerVec corners;
} KeyFrame;
typedef struct {
    KeyFrame *data;
    int size, cap;
} KeyFrameDB;

// --- Helpers ---

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
static void map_push(Map *m, MapPoint p) {
    if (m->size == m->cap) {
        m->cap = m->cap ? m->cap * 2 : 1024;
        m->data = (MapPoint *)xrealloc(m->data, (size_t)m->cap * sizeof(MapPoint));
    }
    m->data[m->size++] = p;
}
static void frame_stat_vec_push(FrameStatVec *vec, FrameStat v) {
    if (vec->size == vec->cap) {
        vec->cap = vec->cap ? vec->cap * 2 : 256;
        vec->data = (FrameStat *)xrealloc(vec->data, (size_t)vec->cap * sizeof(FrameStat));
    }
    vec->data[vec->size++] = v;
}
static void keyframe_db_push(KeyFrameDB *db, KeyFrame kf) {
    if (db->size == db->cap) {
        db->cap = db->cap ? db->cap * 2 : 16;
        db->data = (KeyFrame *)xrealloc(db->data, (size_t)db->cap * sizeof(KeyFrame));
    }
    db->data[db->size++] = kf;
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
static double mat3_det(const double A[9]) {
    return A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) +
           A[2] * (A[3] * A[7] - A[4] * A[6]);
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

static void downsample_area(const unsigned char *src, int sw, int sh, unsigned char *dst, int dw,
                            int dh) {
    float x_ratio = (float)sw / dw, y_ratio = (float)sh / dh;
    for (int y = 0; y < dh; y++)
        for (int x = 0; x < dw; x++) {
            int sx_start = (int)(x * x_ratio), sy_start = (int)(y * y_ratio),
                sx_end = (int)((x + 1) * x_ratio), sy_end = (int)((y + 1) * y_ratio);
            if (sx_end >= sw)
                sx_end = sw - 1;
            if (sy_end >= sh)
                sy_end = sh - 1;
            int sum = 0, count = 0;
            for (int sy = sy_start; sy <= sy_end; sy++)
                for (int sx = sx_start; sx <= sx_end; sx++) {
                    sum += src[sy * sw + sx];
                    count++;
                }
            dst[y * dw + x] = (unsigned char)(sum / count);
        }
}
static void compute_pyramid(const unsigned char *gray, int w, int h, int num_levels,
                            float scale_factor, ImagePyramid *pyr) {
    pyr->num_levels = num_levels;
    pyr->levels[0].data = malloc(w * h);
    memcpy(pyr->levels[0].data, gray, w * h);
    pyr->levels[0].w = w;
    pyr->levels[0].h = h;
    pyr->levels[0].scale = 1.0f;
    for (int i = 1; i < num_levels; i++) {
        pyr->levels[i].scale = pyr->levels[i - 1].scale * scale_factor;
        pyr->levels[i].w = (int)(w / pyr->levels[i].scale + 0.5f);
        pyr->levels[i].h = (int)(h / pyr->levels[i].scale + 0.5f);
        pyr->levels[i].data = malloc(pyr->levels[i].w * pyr->levels[i].h);
        downsample_area(gray, w, h, pyr->levels[i].data, pyr->levels[i].w, pyr->levels[i].h);
    }
}
static void free_pyramid(ImagePyramid *pyr) {
    for (int i = 0; i < pyr->num_levels; i++)
        free(pyr->levels[i].data);
}

static int fast9_offsets[16][2] = {{0, -3}, {1, -3}, {2, -2}, {3, -1}, {3, 0}, {3, 1}, {2, 2}, {1, 3}, {0, 3}, {-1, 3}, {-2, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-2, -2}, {-1, -3}};
static bool is_fast9(const unsigned char *g, int w, int h, int x, int y, int threshold) {
    if (x < 3 || x >= w - 3 || y < 3 || y >= h - 3)
        return false;
    int v = g[y * w + x], v_min = v - threshold, v_max = v + threshold, p[16];
    for (int i = 0; i < 16; i++)
        p[i] = g[(y + fast9_offsets[i][1]) * w + (x + fast9_offsets[i][0])];
    int count_bright = 0, count_dark = 0;
    if (p[0] > v_max)
        count_bright++;
    else if (p[0] < v_min)
        count_dark++;
    if (p[8] > v_max)
        count_bright++;
    else if (p[8] < v_min)
        count_dark++;
    if (p[4] > v_max)
        count_bright++;
    else if (p[4] < v_min)
        count_dark++;
    if (p[12] > v_max)
        count_bright++;
    else if (p[12] < v_min)
        count_dark++;
    if (count_bright < 3 && count_dark < 3)
        return false;
    for (int i = 0; i < 16; i++) {
        bool all_bright = true, all_dark = true;
        for (int j = 0; j < 9; j++) {
            int val = p[(i + j) % 16];
            if (val <= v_max)
                all_bright = false;
            if (val >= v_min)
                all_dark = false;
            if (!all_bright && !all_dark)
                break;
        }
        if (all_bright || all_dark)
            return true;
    }
    return false;
}
static float fast_score(const unsigned char *g, int w, int h, int x, int y) {
    float Ixx = 0, Iyy = 0, Ixy = 0;
    for (int i = -1; i <= 1; i++)
        for (int j = -1; j <= 1; j++) {
            float gx = (float)g[(y + i) * w + x + j + 1] - g[(y + i) * w + x + j - 1],
                  gy = (float)g[(y + i + 1) * w + x + j] - g[(y + i - 1) * w + x + j];
            Ixx += gx * gx;
            Iyy += gy * gy;
            Ixy += gx * gy;
        }
    float tr = Ixx + Iyy, det = Ixx * Iyy - Ixy * Ixy;
    return 0.5f * (tr - sqrtf(fmaxf(0, tr * tr - 4.0f * det)));
}

static int orb_bit_pattern[256 * 4] = {
    8, -3, 9, 5, 4, 2, 7, -12, -11, 9, -8, 2, 7, -12, 12, -13, 2, -13, 2,
    12, 1, -7, 1, 6, -2, -10, -2, -4, -13, -13, -11, -8, -13, -3, -12, -9, 10, 4,
    11, 9, -13, -8, -8, -9, -11, 7, -9, 12, 7, 7, 12, 6, -4, -5, -3, 0, -13,
    2, -12, -3, -9, 0, -7, 5, 12, -6, 12, -1, -3, 6, -2, 12, -6, -13, -4, -8,
    11, -13, 12, -8, 4, 7, 5, 1, 5, -3, 10, -3, 3, -7, 6, 12, -8, -7, -6,
    -2, -2, 11, -1, -10, -13, 12, -8, 10, -7, 3, -5, -3, -4, 2, -3, 7, -10, -12,
    -6, 11, 5, -12, 6, -7, 5, -6, 7, -1, 1, 0, 4, -5, 9, 11, 11, -13, 4,
    7, 4, 12, 2, -1, 4, 4, -4, -12, -2, 7, -8, -5, -7, -10, 4, 11, 9, 12,
    0, -8, 1, -13, -13, -2, -8, 2, -3, -2, -2, 3, -6, 9, -4, -9, 8, 12, 10,
    7, 0, 9, 1, 3, 7, -5, 11, -10, -13, -6, -11, 0, 10, 7, 12, 1, -6, -3,
    -6, 12, 10, -9, 12, -4, -13, 8, -8, -12, -13, 0, -8, -4, 3, 3, 7, 8, 5,
    7, 10, -7, -1, 7, 1, -12, 3, -10, 5, 6, 2, -4, 3, -10, -13, 0, -13, 5,
    -13, -7, -12, 12, -13, 3, -11, 8, -7, 12, -4, 7, 6, -10, 12, 8, -9, -1, -7,
    -6, -2, -5, 0, 12, -12, 5, -7, 5, 3, -10, 8, -13, -7, -7, -4, 5, -3, -2,
    -1, -7, 2, 9, 5, -11, -11, -13, -5, -13, -1, 6, 0, -1, 5, -3, 5, 2, -4,
    -13, -4, 12, -9, -6, -9, 6, -12, -10, -8, -4, 10, 2, 12, -3, 7, 12, 12, 12,
    -7, -13, -6, 5, -4, 9, -3, 4, 7, -1, 12, 2, -7, 6, -5, 1, -13, 11, -12,
    5, -3, 7, -2, -6, 7, -8, 12, -7, -13, -7, -11, -12, 1, -3, 12, 12, 2, -6,
    3, 0, -4, 3, -2, -13, -1, -13, 1, 9, 7, 1, 8, -6, 1, -1, 3, 12, 9,
    1, 12, 6, -1, -9, -1, 3, -13, -13, -10, 5, 7, 7, 10, 12, 12, -5, 12, 9,
    6, 3, 7, 11, 5, -13, 6, 10, 2, -12, 2, 3, 3, 8, 4, -6, 2, 6, 12,
    -13, 9, -12, 10, 3, -8, 4, -7, 9, -11, 12, -4, -6, 1, 12, 2, -8, 6, -9,
    7, -4, 2, 3, 3, -2, 6, 3, 11, 0, 3, -3, 8, -8, 7, 8, 9, 3, -11,
    -5, -6, -4, -10, 11, -5, 10, -5, -8, -3, 12, -10, 5, -9, 0, 8, -1, 12, -6,
    4, -6, 6, -11, -10, 12, -8, 7, 4, -2, 6, 7, -2, 0, -2, 12, -5, -8, -5,
    2, 7, -6, 10, 12, -9, -13, -8, -8, -5, -13, -5, -2, 8, -8, 9, -13, -9, -11,
    -9, 0, 1, -8, 1, -2, 7, -4, 9, 1, -2, 1, -1, -4, 11, -6, 12, -11, -12,
    -9, -6, 4, 3, 7, 7, 12, 5, 5, 10, 8, 0, -4, 2, 8, -9, 12, -5, -13,
    0, 7, 2, 12, -1, 2, 1, 7, 5, 11, 7, -9, 3, 5, 6, -8, -13, -4, -8,
    9, -5, 9, -3, -3, -4, -7, -3, -12, 6, 5, 8, 0, -7, 6, -6, 12, -13, 6,
    -5, -2, 1, -10, 3, 10, 4, 1, 8, -4, -2, -2, 2, -13, 2, -12, 12, 12, -2,
    -13, 0, -6, 4, 1, 9, 3, -6, -10, -3, -5, -3, -13, -1, 1, 7, 5, 12, -11,
    4, -2, 5, -7, -13, 9, -9, -5, 7, 1, 8, 6, 7, -8, 7, 6, -7, -4, -7,
    1, -8, 11, -7, -8, -13, 6, -12, -8, 2, 4, 3, 9, 10, -5, 12, 3, -6, -5,
    -6, 7, 8, -3, 9, -8, 2, -12, 2, 8, -11, -2, -10, 3, -12, -13, -7, -9, -11,
    0, -10, -5, 5, -3, 11, 8, -2, -13, -1, 12, -1, -8, 0, 9, -13, -11, -12, -5,
    -10, -2, -10, 11, -3, 9, -2, -13, 2, -3, 3, 2, -9, -13, -4, 0, -4, 6, -3,
    -10, -4, 12, -2, -7, -6, -11, -4, 9, 6, -3, 6, 11, -13, 11, -5, 5, 11, 11,
    12, 6, 7, -5, 12, -2, -1, 12, 0, 7, -4, -8, -3, -2, -7, 1, -6, 7, -13,
    -12, -8, -13, -7, -2, -6, -8, -8, 5, -6, -9, -5, -1, -4, 5, -13, 7, -8, 10,
    1, 5, 5, -13, 1, 0, 10, -13, 9, 12, 10, -1, 5, -8, 10, -9, -1, 11, 1,
    -13, -9, -3, -6, 2, -1, -10, 1, 12, -13, 1, -8, -10, 8, -11, 10, -6, 2, -13,
    3, -6, 7, -13, 12, -9, -10, -10, -5, -7, -10, -8, -8, -13, 4, -6, 8, 5, 3,
    12, 8, -13, -4, 2, -3, -3, 5, -13, 10, -12, 4, -13, 5, -1, -9, 9, -4, 3,
    0, 3, 3, -9, -12, 1, -6, 1, 3, 2, 4, -8, -10, -10, -10, 9, 8, -13, 12,
    12, -8, -12, -6, -5, 2, 2, 3, 7, 10, 6, 11, -8, 6, 8, 8, -12, -7, 10,
    -6, 5, -3, -9, -3, 9, -1, -13, -1, 5, -3, -7, -3, 4, -8, -2, -8, 3, 4,
    2, 12, 12, 2, -5, 3, 11, 6, -9, 11, -13, 3, -1, 7, 12, 11, -1, 12, 4,
    -3, 0, -3, 6, 4, -11, 4, 12, 2, -4, 2, 1, -10, -6, -8, 1, -13, 7, -11,
    1, -13, 12, -11, -13, 6, 0, 11, -13, 0, -1, 1, 4, -13, 3, -9, -2, -9, 8,
    -6, -3, -13, -6, -8, -2, 5, -9, 8, 10, 2, 7, 3, -9, -1, -6, -1, -1, 9,
    5, 11, -2, 11, -3, 12, -8, 3, 0, 3, 5, -1, 4, 0, 10, 3, -6, 4, 5,
    -13, 0, -10, 5, 5, 8, 12, 11, 8, 9, 9, -6, 7, -4, 8, -12, -10, 4, -10,
    9, 7, 3, 12, 4, 9, -7, 10, -2, 7, 0, 12, -2, -1, -6, 0, -11};
static void compute_orb_descriptors(const unsigned char *g, int w, int h, CornerVec *corners) {
#pragma omp parallel for
    for (int i = 0; i < corners->size; i++) {
        Corner *c = &corners->data[i];
        float angle = c->angle, cos_a = cosf(angle), sin_a = sinf(angle);
        memset(c->desc.bits, 0, sizeof(c->desc.bits));
        int cx = (int)(c->x / powf(1.2f, (float)c->level) + 0.5f),
            cy = (int)(c->y / powf(1.2f, (float)c->level) + 0.5f);
        for (int j = 0; j < 256; j++) {
            int x1 = orb_bit_pattern[j * 4 + 0], y1 = orb_bit_pattern[j * 4 + 1],
                x2 = orb_bit_pattern[j * 4 + 2], y2 = orb_bit_pattern[j * 4 + 3];
            int tx1 = (int)(x1 * cos_a - y1 * sin_a + 0.5f),
                ty1 = (int)(x1 * sin_a + y1 * cos_a + 0.5f),
                tx2 = (int)(x2 * cos_a - y2 * sin_a + 0.5f),
                ty2 = (int)(x2 * sin_a + y2 * cos_a + 0.5f);
            if (cx + tx1 < 0 || cx + tx1 >= w || cy + ty1 < 0 || cy + ty1 >= h || cx + tx2 < 0 ||
                cx + tx2 >= w || cy + ty2 < 0 || cy + ty2 >= h)
                continue;
            if (g[(cy + ty1) * w + (cx + tx1)] < g[(cy + ty2) * w + (cx + tx2)])
                c->desc.bits[j / 64] |= (uint64_t)1 << (j % 64);
        }
    }
}
static int hamming_dist(const OrbDescriptor *a, const OrbDescriptor *b) {
    int d = 0;
    for (int i = 0; i < 4; i++)
        d += __builtin_popcountll(a->bits[i] ^ b->bits[i]);
    return d;
}
static float compute_orientation(const unsigned char *g, int w, int h, int x, int y,
                                 int patch_size) {
    int half_size = patch_size / 2;
    if (x < half_size || x >= w - half_size || y < half_size || y >= h - half_size)
        return 0;
    long m10 = 0, m01 = 0;
    for (int dy = -half_size; dy <= half_size; dy++)
        for (int dx = -half_size; dx <= half_size; dx++) {
            if (dx * dx + dy * dy > half_size * half_size)
                continue;
            int val = g[(y + dy) * w + (x + dx)];
            m10 += dx * val;
            m01 += dy * val;
        }
    return atan2f((float)m01, (float)m10);
}
static void extract_orb_features(const ImagePyramid *pyr, CornerVec *all_corners,
                                 int total_target) {
    int target_per_level = total_target / pyr->num_levels;
    for (int l = 0; l < pyr->num_levels; l++) {
        const ImageLevel *level = &pyr->levels[l];
        int w = level->w, h = level->h, grid_w = 20, grid_h = 20, n_cols = w / grid_w,
            n_rows = h / grid_h;
        CornerVec level_corners = {0};
        for (int r = 0; r < n_rows; r++)
            for (int c = 0; c < n_cols; c++) {
                int x_start = c * grid_w, x_end = (c + 1) * grid_w, y_start = r * grid_h,
                    y_end = (r + 1) * grid_h;
                if (x_end > w - 3)
                    x_end = w - 3;
                if (y_end > h - 3)
                    y_end = h - 3;
                if (x_start < 3)
                    x_start = 3;
                if (y_start < 3)
                    y_start = 3;
                int bx = -1, by = -1;
                float bs = -1e10f;
                for (int y = y_start; y < y_end; y++)
                    for (int x = x_start; x < x_end; x++)
                        if (is_fast9(level->data, w, h, x, y, 20)) {
                            float s = fast_score(level->data, w, h, x, y);
                            if (s > bs) {
                                bs = s;
                                bx = x;
                                by = y;
                            }
                        }
                if (bx != -1) {
                    float angle = compute_orientation(level->data, w, h, bx, by, 31);
                    corner_vec_push(
                        &level_corners,
                        (Corner){(float)bx * level->scale, (float)by * level->scale, -1, angle, l});
                }
            }
        compute_orb_descriptors(level->data, w, h, &level_corners);
        // Sort level_corners by some score? FAST score is not stored in Corner.
        // For now, just take them all if they are within target_per_level, or sub-sample.
        // Actually, just pushing them all is fine if we don't exceed total_target by much.
        for (int i = 0; i < level_corners.size; i++) {
            if (all_corners->size < total_target)
                corner_vec_push(all_corners, level_corners.data[i]);
        }
        free(level_corners.data);
    }
}
static void match_orb_features(const CornerVec *q, const CornerVec *t, MatchVec *matches) {
    for (int i = 0; i < q->size; i++) {
        int best_dist = 257, second_best = 257, best_idx = -1;
        for (int j = 0; j < t->size; j++) {
            int dist = hamming_dist(&q->data[i].desc, &t->data[j].desc);
            if (dist < best_dist) {
                second_best = best_dist;
                best_dist = dist;
                best_idx = j;
            } else if (dist < second_best)
                second_best = dist;
        }
        if (best_idx != -1 && best_dist < 50 && (float)best_dist < 0.8f * (float)second_best)
            match_vec_push(matches, (Match){i, best_idx, best_dist});
    }
}

static int triangulate_point(const Pose *p1, const Pose *p2, Corner pt1, Corner pt2, double fx,
                             double fy, double cx, double cy, double X[3]) {
    double K[9] = {fx, 0, cx,
                   0, fy, cy,
                   0, 0, 1};
    double R1[9], R2[9], t1[3], t2[3], P1[12], P2[12], A[16], VT[4];
    pose_get_rotation(p1, R1);
    pose_get_rotation(p2, R2);
    pose_get_translation(p1, t1);
    pose_get_translation(p2, t2);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 4; c++) {
            P1[r * 4 + c] = K[r * 3 + 0] * ((c < 3) ? R1[0 * 3 + c] : t1[0]) +
                            K[r * 3 + 1] * ((c < 3) ? R1[1 * 3 + c] : t1[1]) +
                            K[r * 3 + 2] * ((c < 3) ? R1[2 * 3 + c] : t1[2]);
            P2[r * 4 + c] = K[r * 3 + 0] * ((c < 3) ? R2[0 * 3 + c] : t2[0]) +
                            K[r * 3 + 1] * ((c < 3) ? R2[1 * 3 + c] : t2[1]) +
                            K[r * 3 + 2] * ((c < 3) ? R2[2 * 3 + c] : t2[2]);
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
    double AtA[16] = {0}, V[16], W4[4];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            for (int k = 0; k < 4; k++)
                AtA[r * 4 + c] += A[k * 4 + r] * A[k * 4 + c];
    jacobi_nxn(AtA, 4, W4, V);
    int bi = 0;
    double mw = W4[0];
    for (int i = 1; i < 4; i++)
        if (W4[i] < mw) {
            mw = W4[i];
            bi = i;
        }
    for (int i = 0; i < 4; i++)
        VT[i] = V[i * 4 + bi];
    if (fabs(VT[3]) < 1e-12)
        return 0;
    X[0] = VT[0] / VT[3];
    X[1] = VT[1] / VT[3];
    X[2] = VT[2] / VT[3];
    // Cheirality: depth in both cameras must be positive.
    double z1 = R1[6]*X[0] + R1[7]*X[1] + R1[8]*X[2] + t1[2];
    double z2 = R2[6]*X[0] + R2[7]*X[1] + R2[8]*X[2] + t2[2];
    if (z1 < 0.1 || z2 < 0.1)
        return 0;
    double c1[3], c2[3];
    camera_center_from_pose(p1, c1);
    camera_center_from_pose(p2, c2);
    double v1[3] = {X[0] - c1[0], X[1] - c1[1], X[2] - c1[2]},
           v2[3] = {X[0] - c2[0], X[1] - c2[1], X[2] - c2[2]};
    double n1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]),
           n2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
    if ((v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (n1 * n2) > 0.9998)
        return 0;
    return isfinite(X[0]);
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
                                     double cy, Pose *out_rel) {
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
        for (int j = 0; j < matches->size && j < 200; j++) {
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
    if (bi >= 0 && bc > 0) {
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
    jacobi_nxn(AtA, 9, W, V);
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
                           unsigned char **out_mask, int *out_inliers) {
    const double th = 1e-4;
    const int iters = 500;
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
        jacobi_nxn(AtA, 9, W, V);
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
                   num = x2[0] * Ex1[0] + x2[1] * Ex1[1] + Ex1[2],
                   den = Ex1[0] * Ex1[0] + Ex1[1] * Ex1[1] +
                         (Et[0] * x2[0] + Et[1] * x2[1] + Et[2]) *
                             (Et[0] * x2[0] + Et[1] * x2[1] + Et[2]) +
                         (Et[3] * x2[0] + Et[4] * x2[1] + Et[5]) *
                             (Et[3] * x2[0] + Et[4] * x2[1] + Et[5]) +
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
                   num = x2[0] * Ex1[0] + x2[1] * Ex1[1] + Ex1[2],
                   den = Ex1[0] * Ex1[0] + Ex1[1] * Ex1[1] +
                         (Et[0] * x2[0] + Et[1] * x2[1] + Et[2]) *
                             (Et[0] * x2[0] + Et[1] * x2[1] + Et[2]) +
                         (Et[3] * x2[0] + Et[4] * x2[1] + Et[5]) *
                             (Et[3] * x2[0] + Et[4] * x2[1] + Et[5]) +
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
        decompose_and_choose_pose(best_E, p_pts, c_pts, matches, bmask, fx, fy, cx, cy, out_rel)) {
        *out_mask = bmask;
        *out_inliers = best_inl;
        return 1;
    }
    free(bmask);
    return 0;
}
static int estimate_pose_PnP(const Map *map, const CornerVec *corners, double fx, double fy,
                             double cx, double cy, Pose *out_pose, int *out_inl) {
    int *ids = malloc(corners->size * sizeof(int));
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1)
            ids[n++] = i;
    if (n < 12) {
        free(ids);
        return 0;
    }
    double best_P[12];
    int best_inl = 0;
    for (int it = 0; it < 500; it++) {
        double AtA[144] = {0}, W[12], V[144], P[12];
        for (int i = 0; i < 10; i++) {
            int idx = ids[rand() % n];
            MapPoint p = map->data[corners->data[idx].pt_idx];
            double u = (corners->data[idx].x - cx) / fx, v = (corners->data[idx].y - cy) / fy;
            double r1[12] = {p.x, p.y, p.z, 1,   0,   0,   0,   0,  -u*p.x, -u*p.y, -u*p.z, -u};
            double r2[12] = {  0,   0,   0, 0, p.x, p.y, p.z, 1,   -v*p.x, -v*p.y, -v*p.z, -v};
            for (int r = 0; r < 12; r++)
                for (int c = 0; c < 12; c++)
                    AtA[r * 12 + c] += r1[r] * r1[c] + r2[r] * r2[c];
        }

        jacobi_nxn(AtA, 12, W, V);
        int bi = 0;
        double mw = W[0];
        for (int j = 1; j < 12; j++)
            if (W[j] < mw) {
                mw = W[j];
                bi = j;
            }
        for (int j = 0; j < 12; j++)
            P[j] = V[j * 12 + bi];
        int inl = 0;
        double R[9], t[3];
        pnp_unpack(P, R, t);
        for (int j = 0; j < n; j++) {
            MapPoint p = map->data[corners->data[ids[j]].pt_idx];
            double cx_p = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
                   cy_p = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
                   cz_p = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
            if (cz_p < 0.1)
                continue;
            double u = fx * cx_p / cz_p + cx, v = fy * cy_p / cz_p + cy,
                   dx = u - corners->data[ids[j]].x, dy = v - corners->data[ids[j]].y;
            if (dx * dx + dy * dy < 100.0)
                inl++;
        }
        if (inl > best_inl) {
            best_inl = inl;
            memcpy(best_P, P, 12 * sizeof(double));
        }
        if (inl > n * 0.8)
            break;
    }
    if (best_inl >= 12) {
        double R[9], t[3], Ro[9];
        pnp_unpack(best_P, R, t);
        project_to_SO3(R, Ro);
        pose_from_rt(Ro, t, out_pose);
        *out_inl = best_inl;
        free(ids);
        return 1;
    }
    free(ids);
    return 0;
}
static int solve_3x3(double A[9], double b[3], double x[3]) {
    double det = A[0] * (A[4] * A[8] - A[5] * A[7]) - A[1] * (A[3] * A[8] - A[5] * A[6]) +
                 A[2] * (A[3] * A[7] - A[4] * A[6]);
    if (fabs(det) < 1e-12)
        return 0;
    x[0] = (b[0]*(A[4]*A[8] - A[5]*A[7]) - A[1]*(b[1]*A[8] - A[5]*b[2]) + A[2]*(b[1]*A[7] - A[4]*b[2])) / det;
    x[1] = (A[0]*(b[1]*A[8] - A[5]*b[2]) - b[0]*(A[3]*A[8] - A[5]*A[6]) + A[2]*(A[3]*b[2] - b[1]*A[6])) / det;
    x[2] = (A[0]*(A[4]*b[2] - b[1]*A[7]) - A[1]*(A[3]*b[2] - b[1]*A[6]) + b[0]*(A[3]*A[7] - A[4]*A[6])) / det;
    return 1;
}
static void refine_pose_lm(const Map *map, const CornerVec *corners, double fx, double fy,
                           double cx, double cy, Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    for (int iter = 0; iter < 10; iter++) {
        double H[36] = {0}, b[6] = {0};
        for (int i = 0; i < corners->size; i++) {
            if (corners->data[i].pt_idx == -1)
                continue;
            MapPoint p = map->data[corners->data[i].pt_idx];

            // Project point P into camera frame:  (xc, yc, zc) = R·P + t
            double xc = R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0];
            double yc = R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1];
            double zc = R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2];
            if (zc < 0.1)
                continue;

            // Pinhole projection and reprojection residual:
            double invz  = 1.0 / zc;
            double invz2 = invz * invz;
            double u  = fx*xc*invz + cx;
            double v  = fy*yc*invz + cy;
            double ex = corners->data[i].x - u;
            double ey = corners->data[i].y - v;
            double J[2][6] = {
                {fx*invz, 0,       -fx*xc*invz2,  -fx*xc*yc*invz2,             fx*(1 + xc*xc*invz2),  -fx*yc*invz},
                {0,       fy*invz, -fy*yc*invz2,  -fy*(1 + yc*yc*invz2),       fy*xc*yc*invz2,         fy*xc*invz},
            };
            for (int r = 0; r < 6; r++) {
                for (int c = 0; c < 6; c++)
                    H[r * 6 + c] += J[0][r] * J[0][c] + J[1][r] * J[1][c];
                b[r] += J[0][r] * ex + J[1][r] * ey;
            }
        }
        for (int i = 0; i < 6; i++)
            H[i * 6 + i] *= 1.01;
        double dx[6];
        if (!solve_6x6(H, b, dx))
            break;
        t[0] += dx[0];
        t[1] += dx[1];
        t[2] += dx[2];
        double dR[9], th = sqrt(dx[3] * dx[3] + dx[4] * dx[4] + dx[5] * dx[5]);
        if (th < 1e-12) {
            dR[0] = 1;
            dR[1] = 0;
            dR[2] = 0;
            dR[3] = 0;
            dR[4] = 1;
            dR[5] = 0;
            dR[6] = 0;
            dR[7] = 0;
            dR[8] = 1;
        } else {
            double s = sin(th) / th, c = (1 - cos(th)) / (th * th);
            dR[0] = 1 - c * (dx[4] * dx[4] + dx[5] * dx[5]);
            dR[1] = c * dx[3] * dx[4] - s * dx[5];
            dR[2] = c * dx[3] * dx[5] + s * dx[4];
            dR[3] = c * dx[3] * dx[4] + s * dx[5];
            dR[4] = 1 - c * (dx[3] * dx[3] + dx[5] * dx[5]);
            dR[5] = c * dx[4] * dx[5] - s * dx[3];
            dR[6] = c * dx[3] * dx[5] - s * dx[4];
            dR[7] = c * dx[4] * dx[5] + s * dx[3];
            dR[8] = 1 - c * (dx[3] * dx[3] + dx[4] * dx[4]);
        }
        double Rt[9];
        mat3_mul(dR, R, Rt);
        memcpy(R, Rt, 9 * sizeof(double));
        if (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2] < 1e-10)
            break;
    }
    pose_from_rt(R, t, pose);
}
static void cull_map(Map *map, KeyFrameDB *db, CornerVec *curr, int frame_id) {
    int *remap = malloc(map->size * sizeof(int));
    int n = 0;
    for (int i = 0; i < map->size; i++) {
        MapPoint *p = &map->data[i];
        if (p->is_bad || (frame_id - p->last_frame_id > 2 && p->obs < 3)) {
            p->is_bad = true;
            remap[i] = -1;
            continue;
        }
        remap[i] = n;
        map->data[n++] = *p;
    }
    // Update indices in KeyFrames
    for (int j = 0; j < db->size; j++) {
        for (int k = 0; k < db->data[j].corners.size; k++) {
            int old_idx = db->data[j].corners.data[k].pt_idx;
            if (old_idx != -1)
                db->data[j].corners.data[k].pt_idx = remap[old_idx];
        }
    }
    // Update current frame
    for (int k = 0; k < curr->size; k++) {
        int old_idx = curr->data[k].pt_idx;
        if (old_idx != -1)
            curr->data[k].pt_idx = remap[old_idx];
    }
    map->size = n;
    free(remap);
}

static void local_ba(KeyFrameDB *db, Map *map, double fx, double fy, double cx, double cy) {
    if (db->size < 2)
        return;
    int n_kf = db->size > 10 ? 10 : db->size;
    int start_kf = db->size - n_kf;
    for (int iter = 0; iter < 5; iter++) {
        double *H_all = calloc(map->size * 9, sizeof(double)),
               *b_all = calloc(map->size * 3, sizeof(double));
        int *obs_count = calloc(map->size, sizeof(int));
        for (int j = start_kf; j < db->size; j++) {
            KeyFrame *kf = &db->data[j];
            double R[9], t[3];
            pose_get_rotation(&kf->pose, R);
            pose_get_translation(&kf->pose, t);
            for (int k = 0; k < kf->corners.size; k++) {
                int p_idx = kf->corners.data[k].pt_idx;
                if (p_idx == -1)
                    continue;
                MapPoint *p = &map->data[p_idx];

                // Project P into camera k:  (xc, yc, zc) = R·P + t
                double xc = R[0]*p->x + R[1]*p->y + R[2]*p->z + t[0];
                double yc = R[3]*p->x + R[4]*p->y + R[5]*p->z + t[1];
                double zc = R[6]*p->x + R[7]*p->y + R[8]*p->z + t[2];
                if (zc < 0.1)
                    continue;

                double invz  = 1.0 / zc;
                double invz2 = invz * invz;
                double u  = fx*xc*invz + cx;
                double v  = fy*yc*invz + cy;
                double ex = kf->corners.data[k].x - u;
                double ey = kf->corners.data[k].y - v;
                double J[2][3] = {
                    {fx*invz*R[0] - fx*xc*invz2*R[6],  fx*invz*R[1] - fx*xc*invz2*R[7],  fx*invz*R[2] - fx*xc*invz2*R[8]},
                    {fy*invz*R[3] - fy*yc*invz2*R[6],  fy*invz*R[4] - fy*yc*invz2*R[7],  fy*invz*R[5] - fy*yc*invz2*R[8]},
                };
                for (int r = 0; r < 3; r++) {
                    for (int c = 0; c < 3; c++)
                        H_all[p_idx * 9 + r * 3 + c] += J[0][r] * J[0][c] + J[1][r] * J[1][c];
                    b_all[p_idx * 3 + r] += J[0][r] * ex + J[1][r] * ey;
                }
                obs_count[p_idx]++;
            }
        }
        for (int i = 0; i < map->size; i++)
            if (obs_count[i] >= 2) {
                double dp[3];
                if (solve_3x3(&H_all[i * 9], &b_all[i * 3], dp)) {
                    map->data[i].x += dp[0];
                    map->data[i].y += dp[1];
                    map->data[i].z += dp[2];
                }
            }
        free(H_all);
        free(b_all);
        free(obs_count);
    }
}
static void match_frame_to_map(const CornerVec *corners, Map *map, const Pose *pose, double fx,
                               double fy, double cx, double cy, int frame_id, int *matches_count) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    *matches_count = 0;
    for (int i = 0; i < corners->size; i++) {
        int best_dist = 257, second_best = 257, best_idx = -1;
        float cur_x = corners->data[i].x, cur_y = corners->data[i].y;
        for (int j = 0; j < map->size; j++) {
            MapPoint *p = &map->data[j];
            if (p->is_bad)
                continue;
            double xc = R[0] * p->x + R[1] * p->y + R[2] * p->z + t[0],
                   yc = R[3] * p->x + R[4] * p->y + R[5] * p->z + t[1],
                   zc = R[6] * p->x + R[7] * p->y + R[8] * p->z + t[2];
            if (zc < 0.1)
                continue;
            float u = (float)(fx * xc / zc + cx), v = (float)(fy * yc / zc + cy);
            if (fabsf(u - cur_x) > 100 || fabsf(v - cur_y) > 100)
                continue;
            int dist = hamming_dist(&corners->data[i].desc, &p->desc);
            if (dist < best_dist) {
                second_best = best_dist;
                best_dist = dist;
                best_idx = j;
            } else if (dist < second_best)
                second_best = dist;
        }
        if (best_idx != -1 && best_dist < 50 && (float)best_dist < 0.8f * (float)second_best) {
            corners->data[i].pt_idx = best_idx;
            map->data[best_idx].obs++;
            map->data[best_idx].last_frame_id = frame_id;
            (*matches_count)++;
        }
    }
}
static void search_map_by_descriptor(const CornerVec *corners, Map *map, int frame_id,
                                     int *matches_count) {
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx != -1)
            continue;
        int best_dist = 257, second_best = 257, best_idx = -1;
        for (int j = 0; j < map->size; j++) {
            if (map->data[j].is_bad)
                continue;
            int dist = hamming_dist(&corners->data[i].desc, &map->data[j].desc);
            if (dist < best_dist) {
                second_best = best_dist;
                best_dist = dist;
                best_idx = j;
            } else if (dist < second_best)
                second_best = dist;
        }
        if (best_idx != -1 && best_dist < 60 && (float)best_dist < 0.8f * (float)second_best) {
            corners->data[i].pt_idx = best_idx;
            map->data[best_idx].obs++;
            map->data[best_idx].last_frame_id = frame_id;
            (*matches_count)++;
        }
    }
}
static bool pose_is_finite(const Pose *p) {
    for (int i = 0; i < 16; i++)
        if (!isfinite(p->m[i]))
            return false;
    return true;
}
static void write_metrics_json(FILE *f, Config *cfg, FrameStatVec *s, int pts, int tri,
                               double dur) {
    int kf = 0;
    for (int i = 0; i < s->size; i++)
        if (s->data[i].is_keyframe)
            kf++;
    double av = 0;
    int nav = 0;
    for (int i = 1; i < s->size; i++) {
        av += s->data[i].inliers;
        nav++;
    }
    if (nav > 0)
        av /= nav;
    fprintf(f,
            "{\n  \"frames\": %d, \"points\": %d, \"duration_sec\": %f, \"video_path\": \"%s\", "
            "\"keyframes\": %d, \"tri_points_total\": %d, \"avg_inliers_after_first\": %f, "
            "\"timeline\": [\n",
            s->size, pts, dur, cfg->video_path, kf, tri, av);
    for (int i = 0; i < s->size; i++) {
        FrameStat *fs = &s->data[i];
        fprintf(f,
                "    {\"frame_id\": %d, \"inliers\": %d, \"is_keyframe\": %s, \"points_added\": "
                "%d, \"points_total\": %d, \"method\": %d, \"xyz\": [%f,%f,%f], \"rotation\": "
                "[%f,%f,%f,%f,%f,%f,%f,%f,%f]}%s\n",
                fs->frame_id, fs->inliers, fs->is_keyframe ? "true" : "false", fs->points_added,
                fs->points_total, fs->method, fs->xyz[0], fs->xyz[1], fs->xyz[2], fs->rotation[0],
                fs->rotation[1], fs->rotation[2], fs->rotation[3], fs->rotation[4], fs->rotation[5],
                fs->rotation[6], fs->rotation[7], fs->rotation[8], (i + 1 < s->size) ? "," : "");
    }
    fprintf(f, "  ]\n}\n");
}
typedef struct {
    FILE *pipe;
    int w, h;
} FFmpegCap;
static FFmpegCap *ffmpeg_open(const char *p) {
    FFmpegCap *c = malloc(sizeof(FFmpegCap));
    c->w = 640;
    c->h = 480;
    char cmd[1024];
    snprintf(
        cmd, 1024,
        "ffmpeg -hide_banner -loglevel error -i \"%s\" -f rawvideo -pix_fmt bgr24 -s 640x480 -", p);
    c->pipe = popen(cmd, "r");
    if (!c->pipe) {
        free(c);
        return NULL;
    }
    return c;
}
static int ffmpeg_read(FFmpegCap *c, unsigned char *b) {
    return fread(b, 1, (size_t)c->w * c->h * 3, c->pipe) == (size_t)c->w * c->h * 3;
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
static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
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
static Config parse_args(int argc, char **argv) {
    Config c = {"test_kitti984.mp4", 5.0, 30.0, NULL, 20, 5.0, 15000};
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--video_path") && i + 1 < argc)
            c.video_path = argv[++i];
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc)
            c.seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "--timeout") && i + 1 < argc)
            c.timeout = atof(argv[++i]);
        else if (!strcmp(argv[i], "--metrics_out") && i + 1 < argc)
            c.metrics_out = argv[++i];
        else if (argv[i][0] != '-')
            c.video_path = argv[i];
    }
    return c;
}

int main(int argc, char **argv) {
    Config cfg = parse_args(argc, argv);
    FFmpegCap *cap = ffmpeg_open(cfg.video_path);
    if (!cap)
        return 1;
    srand((unsigned int)time(NULL));
    int w = 640, h = 480;
    unsigned char *raw = malloc(w * h * 3), *cgray = malloc(w * h), *cblur = malloc(w * h);
    KeyFrameDB kf_db = {0};
    Map map = {0};
    FrameStatVec stats = {0};
    Pose pose, last_kf_pose;
    pose_identity(&pose);
    pose_identity(&last_kf_pose);
    int frame_id = 0, pts = 0, tri = 0, last_kf_pose_id = 0;
    double start = now_seconds(), fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5;
    Pose last_rel, prev_pose;
    pose_identity(&last_rel);
    pose_identity(&prev_pose);
    while (frame_id < (int)(25.0 * cfg.seconds) && ffmpeg_read(cap, raw)) {
        if (now_seconds() - start > cfg.timeout)
            break;
        bgr_to_gray(raw, w, h, cgray);
        blur_3x3(cgray, w, h, cblur);
        ImagePyramid pyr;
        compute_pyramid(cblur, w, h, 8, 1.2f, &pyr);
        CornerVec curr_corners = {0};
        extract_orb_features(&pyr, &curr_corners, 3000);
        int inl = 0, mkf = 0, added = 0, method = 0;
        if (frame_id == 0) {
            KeyFrame kf = {frame_id, pose, {0}};
            kf.corners.size = curr_corners.size;
            kf.corners.data = malloc(curr_corners.size * sizeof(Corner));
            memcpy(kf.corners.data, curr_corners.data, curr_corners.size * sizeof(Corner));
            keyframe_db_push(&kf_db, kf);
            mkf = 1;
            method = 0;
        } else {
            Pose predicted;
            pose_compose_relative(&last_rel, &prev_pose, &predicted);
            match_frame_to_map(&curr_corners, &map, &predicted, fx, fy, cx, cy, frame_id, &inl);
            if (inl < 20) {
                int extra = 0;
                search_map_by_descriptor(&curr_corners, &map, frame_id, &extra);
                inl += extra;
            }
            bool tracked = false;
            if (inl >= 15) {
                Pose refined = predicted;
                refine_pose_lm(&map, &curr_corners, fx, fy, cx, cy, &refined);
                if (pose_is_finite(&refined)) {
                    // Re-count inliers with refined pose
                    int new_inl = 0;
                    double R[9], t[3];
                    pose_get_rotation(&refined, R);
                    pose_get_translation(&refined, t);
                    for (int i = 0; i < curr_corners.size; i++)
                        if (curr_corners.data[i].pt_idx != -1) {
                            MapPoint p = map.data[curr_corners.data[i].pt_idx];
                            double xc = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
                                   yc = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
                                   zc = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
                            if (zc < 0.1)
                                continue;
                            double u = fx * xc / zc + cx, v = fy * yc / zc + cy,
                                   dx = u - curr_corners.data[i].x, dy = v - curr_corners.data[i].y;
                            if (dx * dx + dy * dy < 100.0)
                                new_inl++;
                        }
                    if (new_inl >= 12) {
                        pose = refined;
                        inl = new_inl;
                        method = 2;
                        tracked = true;
                    }
                }
            }
            if (!tracked && inl >= 15 &&
                estimate_pose_PnP(&map, &curr_corners, fx, fy, cx, cy, &predicted, &inl)) {
                Pose refined = predicted;
                refine_pose_lm(&map, &curr_corners, fx, fy, cx, cy, &refined);
                if (pose_is_finite(&refined)) {
                    pose = refined;
                    method = 2;
                    tracked = true;
                } else {
                    pose = predicted;
                    method = 2;
                    tracked = true;
                }
            }
            if (!tracked && kf_db.size > 0) {
                MatchVec matches = {0};
                match_orb_features(&kf_db.data[kf_db.size - 1].corners, &curr_corners, &matches);
                Pose rel;
                unsigned char *mask = NULL;
                if (estimate_pose_E(&kf_db.data[kf_db.size - 1].corners, &curr_corners, &matches,
                                    fx, fy, cx, cy, &rel, &mask, &inl)) {
                    Pose new_pose;
                    pose_compose_relative(&rel, &kf_db.data[kf_db.size - 1].pose, &new_pose);
                    Pose refined = new_pose;
                    refine_pose_lm(&map, &curr_corners, fx, fy, cx, cy, &refined);
                    if (pose_is_finite(&refined)) {
                        pose = refined;
                        method = 1;
                    } else if (pose_is_finite(&new_pose)) {
                        pose = new_pose;
                        method = 1;
                    } else {
                        pose = predicted;
                        method = 3;
                    }
                    if (mask) {
                        for (int i = 0; i < matches.size; i++)
                            if (mask[i]) {
                                int qidx = matches.data[i].query_idx,
                                    tidx = matches.data[i].train_idx;
                                if (kf_db.data[kf_db.size - 1].corners.data[qidx].pt_idx != -1) {
                                    int p_idx =
                                        kf_db.data[kf_db.size - 1].corners.data[qidx].pt_idx;
                                    curr_corners.data[tidx].pt_idx = p_idx;
                                    map.data[p_idx].obs++;
                                    map.data[p_idx].last_frame_id = frame_id;
                                } else if (pose_is_finite(&pose)) {
                                    double X[3];
                                    if (triangulate_point(
                                            &kf_db.data[kf_db.size - 1].pose, &pose,
                                            kf_db.data[kf_db.size - 1].corners.data[qidx],
                                            curr_corners.data[tidx], fx, fy, cx, cy, X)) {
                                        curr_corners.data[tidx].pt_idx = map.size;
                                        kf_db.data[kf_db.size - 1].corners.data[qidx].pt_idx =
                                            map.size;
                                        map_push(&map, (MapPoint){X[0], X[1], X[2], 2,
                                                                  curr_corners.data[tidx].desc,
                                                                  frame_id, false});
                                        pts++;
                                        added++;
                                        tri++;
                                    }
                                }
                            }
                        free(mask);
                    }
                } else {
                    pose = predicted;
                    method = 3;
                }
                free(matches.data);
            }
            if (!pose_is_finite(&pose)) {
                pose = prev_pose;
                method = 3;
            }
            if ((inl < 50 && frame_id - last_kf_pose_id > 5) || frame_id - last_kf_pose_id > 20)
                mkf = 1;
        }
        if (mkf && frame_id > 0) {
            last_kf_pose_id = frame_id;
            MatchVec matches = {0};
            match_orb_features(&kf_db.data[kf_db.size - 1].corners, &curr_corners, &matches);
            int tri_added = 0;
            for (int i = 0; i < matches.size; i++) {
                int qidx = matches.data[i].query_idx, tidx = matches.data[i].train_idx;
                if (curr_corners.data[tidx].pt_idx != -1)
                    continue;
                if (kf_db.data[kf_db.size - 1].corners.data[qidx].pt_idx != -1) {
                    int p_idx = kf_db.data[kf_db.size - 1].corners.data[qidx].pt_idx;
                    curr_corners.data[tidx].pt_idx = p_idx;
                    map.data[p_idx].obs++;
                    map.data[p_idx].last_frame_id = frame_id;
                    continue;
                }
                double X[3];
                if (triangulate_point(&kf_db.data[kf_db.size - 1].pose, &pose,
                                      kf_db.data[kf_db.size - 1].corners.data[qidx],
                                      curr_corners.data[tidx], fx, fy, cx, cy, X)) {
                    curr_corners.data[tidx].pt_idx = map.size;
                    map_push(&map, (MapPoint){X[0], X[1], X[2], 2, curr_corners.data[tidx].desc,
                                              frame_id, false});
                    pts++;
                    added++;
                    tri++;
                    tri_added++;
                }
            }
            if (tri_added > 0 || mkf) {
                KeyFrame kf = {frame_id, pose, {0}};
                kf.corners.size = curr_corners.size;
                kf.corners.data = malloc(curr_corners.size * sizeof(Corner));
                memcpy(kf.corners.data, curr_corners.data, curr_corners.size * sizeof(Corner));
                keyframe_db_push(&kf_db, kf);
                last_kf_pose = pose;
                local_ba(&kf_db, &map, fx, fy, cx, cy);
                cull_map(&map, &kf_db, &curr_corners, frame_id);
            }
            free(matches.data);
        }
        if (frame_id > 0) {
            Pose inv_prev;
            pose_inverse(&prev_pose, &inv_prev);
            pose_compose_relative(&pose, &inv_prev, &last_rel);
        }
        prev_pose = pose;
        double c_pos[3], R_mat[9];
        camera_center_from_pose(&pose, c_pos);
        pose_get_rotation(&pose, R_mat);
        frame_stat_vec_push(&stats, (FrameStat){frame_id,
                                                inl,
                                                mkf,
                                                added,
                                                pts,
                                                method,
                                                {c_pos[0], c_pos[1], c_pos[2]},
                                                {R_mat[0], R_mat[1], R_mat[2], R_mat[3], R_mat[4],
                                                 R_mat[5], R_mat[6], R_mat[7], R_mat[8]}});
        if ((frame_id + 1) % 10 == 0)
            printf("Frame=%d Inl=%d Pts=%d KF=%d Map=%d Meth=%d\n", frame_id + 1, inl, pts,
                   kf_db.size, map.size, method);
        free_pyramid(&pyr);
        free(curr_corners.data);
        frame_id++;
    }
    const char *out = cfg.metrics_out ? cfg.metrics_out : "runs/pure_c_orb_metrics.json";
    ensure_parent_dir(out);
    FILE *f = fopen(out, "wb");
    if (f) {
        write_metrics_json(f, &cfg, &stats, pts, tri, now_seconds() - start);
        fclose(f);
    }
    ffmpeg_close(cap);
    free(raw);
    free(cgray);
    free(cblur);
    free(stats.data);
    free(map.data);
    for (int i = 0; i < kf_db.size; i++)
        free(kf_db.data[i].corners.data);
    free(kf_db.data);
    return 0;
}
