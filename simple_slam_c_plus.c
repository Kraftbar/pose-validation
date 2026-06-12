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
#include "simple_slam_c_plus_config.h"
#include "simple_slam_c_plus_image.h"

static void jacobi_9x9(double A[81], double W[9], double V[81]) {
    jacobi_nxn(A, 9, W, V);
}
static void jacobi_12x12(double A[144], double W[12], double V[144]) {
    jacobi_nxn(A, 12, W, V);
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define ADMISSION_GRID_COLS 8
#define ADMISSION_GRID_ROWS 6
#define ADMISSION_GRID_CELLS (ADMISSION_GRID_COLS * ADMISSION_GRID_ROWS)

static unsigned int g_ransac_seed = 0;

typedef struct {
    float x, y;
    int pt_idx;
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
    double trans_jump;
    double raw_xyz[3];
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
    double pose_lm, triangulate, refill, loop, ba, ba_local, ba_joint;
    double pnp_quality, map_hygiene, global_ba, metrics;
} ProfileStat;
typedef struct {
    ProfileStat *data;
    int size, cap;
} ProfileStatVec;
typedef struct {
    double m[16];
} Pose;
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
    int map_idx;
    int birth_frame;
    int last_frame;
    int method;
    int inliers;
    int cell;
    double reproj;
    double parallax;
    double depth;
    double fb_err;
    double track_disp;
    double score;
    const char *source;
} MapLifecycle;
typedef struct {
    MapLifecycle *data;
    int size, cap;
} MapLifecycleVec;
typedef struct {
    CornerVec corners;
    Brief256 *desc;
    Pose pose;
    int frame_id;
} AnchorSet;
static double vec3_dist(const double a[3], const double b[3]);
static void camera_center_from_pose(const Pose *pose, double c[3]);
static int triangulate_point(const Pose *p1, const Pose *p2, Corner pt1, Corner pt2,
                             double fx, double fy, double cx, double cy, double X[3]);
static void *xrealloc(void *ptr, size_t size);
static void corner_vec_push(CornerVec *vec, Corner v);
static void match_vec_push(MatchVec *vec, Match v);

#include "simple_slam_c_plus_frontend_descriptors.h"
#include "simple_slam_c_plus_frontend_features.h"
#include "simple_slam_c_plus_frontend_lk.h"

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
static int compare_double_asc(const void *a, const void *b) {
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}
static double median_recent_values(const double *values, int count) {
    double tmp[64];
    if (count <= 0)
        return NAN;
    if (count > 64)
        count = 64;
    for (int i = 0; i < count; i++)
        tmp[i] = values[i];
    qsort(tmp, (size_t)count, sizeof(double), compare_double_asc);
    if (count & 1)
        return tmp[count / 2];
    return 0.5 * (tmp[count / 2 - 1] + tmp[count / 2]);
}
static int recent_int_sum(const int *values, int count, int pos, int window) {
    if (window > count)
        window = count;
    int sum = 0;
    for (int i = 0; i < window; i++) {
        int idx = pos - 1 - i;
        while (idx < 0)
            idx += 64;
        sum += values[idx % 64];
    }
    return sum;
}
static int recent_double_count_gt(const double *values, int count, int pos,
                                  int window, double threshold) {
    if (window > count)
        window = count;
    int n = 0;
    for (int i = 0; i < window; i++) {
        int idx = pos - 1 - i;
        while (idx < 0)
            idx += 64;
        if (values[idx % 64] > threshold)
            n++;
    }
    return n;
}
static int recent_int_count_lt(const int *values, int count, int pos,
                               int window, int threshold) {
    if (window > count)
        window = count;
    int n = 0;
    for (int i = 0; i < window; i++) {
        int idx = pos - 1 - i;
        while (idx < 0)
            idx += 64;
        if (values[idx % 64] < threshold)
            n++;
    }
    return n;
}
static void map_lifecycle_vec_push(MapLifecycleVec *v, MapLifecycle row) {
    if (v->size == v->cap) {
        v->cap = v->cap ? v->cap * 2 : 1024;
        v->data = (MapLifecycle *)xrealloc(v->data,
                                           (size_t)v->cap * sizeof(MapLifecycle));
    }
    v->data[v->size++] = row;
}
static void record_map_lifecycle(MapLifecycleVec *v, int enabled, int map_idx,
                                 int birth_frame, const char *source, int method,
                                 int inliers, int cell, double reproj,
                                 double parallax, double depth, double fb_err,
                                 double track_disp, double score) {
    if (!enabled)
        return;
    MapLifecycle row = {map_idx, birth_frame, birth_frame, method, inliers, cell, reproj,
                        parallax, depth, fb_err, track_disp, score, source};
    map_lifecycle_vec_push(v, row);
}
static void mark_map_lifecycle_observed(MapLifecycleVec *v, int enabled, int map_idx,
                                        int frame_id) {
    if (!enabled || map_idx < 0)
        return;
    if (map_idx < v->size && v->data[map_idx].map_idx == map_idx) {
        v->data[map_idx].last_frame = frame_id;
        return;
    }
    for (int i = v->size - 1; i >= 0; i--) {
        if (v->data[i].map_idx == map_idx) {
            v->data[i].last_frame = frame_id;
            return;
        }
    }
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

static int mat3_inverse(const double A[9], double inv[9]) {
    double det = mat3_det(A);
    if (fabs(det) < 1e-12)
        return 0;
    double s = 1.0 / det;
    inv[0] =  (A[4] * A[8] - A[5] * A[7]) * s;
    inv[1] = -(A[1] * A[8] - A[2] * A[7]) * s;
    inv[2] =  (A[1] * A[5] - A[2] * A[4]) * s;
    inv[3] = -(A[3] * A[8] - A[5] * A[6]) * s;
    inv[4] =  (A[0] * A[8] - A[2] * A[6]) * s;
    inv[5] = -(A[0] * A[5] - A[2] * A[3]) * s;
    inv[6] =  (A[3] * A[7] - A[4] * A[6]) * s;
    inv[7] = -(A[0] * A[7] - A[1] * A[6]) * s;
    inv[8] =  (A[0] * A[4] - A[1] * A[3]) * s;
    return 1;
}

static int solve_dense_n(double *A, double *b, double *x, int n) {
    if (n <= 0)
        return 0;
    double *M = (double *)malloc((size_t)n * (size_t)(n + 1) * sizeof(double));
    if (!M)
        return 0;
    for (int r = 0; r < n; r++) {
        for (int c = 0; c < n; c++)
            M[r * (n + 1) + c] = A[r * n + c];
        M[r * (n + 1) + n] = b[r];
    }
    for (int k = 0; k < n; k++) {
        int piv = k;
        double best = fabs(M[k * (n + 1) + k]);
        for (int r = k + 1; r < n; r++) {
            double v = fabs(M[r * (n + 1) + k]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < 1e-12) {
            free(M);
            return 0;
        }
        if (piv != k) {
            for (int c = k; c <= n; c++) {
                double tmp = M[k * (n + 1) + c];
                M[k * (n + 1) + c] = M[piv * (n + 1) + c];
                M[piv * (n + 1) + c] = tmp;
            }
        }
        double inv_piv = 1.0 / M[k * (n + 1) + k];
        for (int c = k; c <= n; c++)
            M[k * (n + 1) + c] *= inv_piv;
        for (int r = 0; r < n; r++) {
            if (r == k)
                continue;
            double f = M[r * (n + 1) + k];
            for (int c = k; c <= n; c++)
                M[r * (n + 1) + c] -= f * M[k * (n + 1) + c];
        }
    }
    for (int i = 0; i < n; i++)
        x[i] = M[i * (n + 1) + n];
    free(M);
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

static double pose_point_depth(const Pose *pose, const double X[3]) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);

    return R[6]*X[0] + R[7]*X[1] + R[8]*X[2] + t[2];
}

static int cmp_double_asc(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    if (da < db)
        return -1;
    if (da > db)
        return 1;
    return 0;
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
    srand(g_ransac_seed);
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

#include "simple_slam_c_plus_pnp.h"
#include "simple_slam_c_plus_backend_keyframes.h"
#include "simple_slam_c_plus_backend_ba.h"
#include "simple_slam_c_plus_backend_map.h"

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
static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
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

static void scale_pose_translation(Pose *pose, double s) {
    pose->m[3] *= s;
    pose->m[7] *= s;
    pose->m[11] *= s;
}

static void scale_world_state(double s, Map *map, KFDB *kf_db, AnchorSet *anchors,
                              FrameLite *prev, Pose *pose, Pose *lkf_pose,
                              Pose *last_rel) {
    if (!isfinite(s) || s <= 0.0 || fabs(s - 1.0) < 1e-12)
        return;
    for (int i = 0; i < map->size; i++) {
        map->data[i].x *= s;
        map->data[i].y *= s;
        map->data[i].z *= s;
    }
    for (int i = 0; i < kf_db->size; i++)
        scale_pose_translation(&kf_db->data[i].pose, s);
    scale_pose_translation(&anchors->pose, s);
    scale_pose_translation(&prev->pose, s);
    scale_pose_translation(pose, s);
    scale_pose_translation(lkf_pose, s);
    scale_pose_translation(last_rel, s);
}

static void maybe_normalize_world_scale(const Config *cfg, Map *map, KFDB *kf_db,
                                        AnchorSet *anchors, FrameLite *prev,
                                        Pose *pose, Pose *lkf_pose, Pose *last_rel) {
    if (!cfg->normalize_world_scale || map->size < 100)
        return;
    double center[3];
    camera_center_from_pose(pose, center);
    double norm = sqrt(center[0] * center[0] +
                       center[1] * center[1] +
                       center[2] * center[2]);
    if (!isfinite(norm) || norm <= 1000.0)
        return;
    scale_world_state(10.0 / norm, map, kf_db, anchors, prev, pose,
                      lkf_pose, last_rel);
}

#include "simple_slam_c_plus_diagnostics.h"

int main(int argc, char **argv) {
    Config cfg = parse_args(argc, argv);
    g_ransac_seed = (unsigned int)cfg.ransac_seed;
    brief_init_pattern();
    g_oriented_brief = cfg.oriented_brief;
    g_brief_patch_radius = cfg.brief_patch_radius;
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
    MapLifecycleVec map_lifecycle = {0};
    AnchorSet anchors = {0};
    int frame_id = 0, pts = 0, tri = 0;
    int output_smooth_initialized = 0;
    double output_smooth_xyz[3] = {0.0, 0.0, 0.0};
    double output_smooth_residuals[64] = {0.0};
    int output_smooth_residual_count = 0;
    int output_smooth_residual_pos = 0;
    int output_smooth_added_history[64] = {0};
    int output_smooth_linked_history[64] = {0};
    double output_smooth_jump_history[64] = {0.0};
    int output_smooth_history_count = 0;
    int output_smooth_history_pos = 0;
    double start = now_seconds();
    double fx = 525.0 * ((double)w / 640.0), fy = 525.0 * ((double)h / 480.0);
    double cx = 319.5 * ((double)w / 640.0), cy = 239.5 * ((double)h / 480.0);
    int map_lifecycle_enabled = cfg.map_lifecycle_dump != NULL;
    DiagnosticsFiles diag;
    if (!diagnostics_open_files(&cfg, &diag))
        return 1;
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
        double trans_jump = 0.0;
        if (frame_id == 0) {
            t0 = now_seconds();
            extract_features_pure(cblur, w, h, &curr.corners, cfg.max_points);
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
            if (cfg.anchor_e_pose && anchors.corners.size >= 8) {
                t0 = now_seconds();
                CornerVec anchor_cur_corners = {0};
                extract_features_pure(cblur, w, h, &anchor_cur_corners,
                                      cfg.anchor_max_features);
                anchor_set_build(&cur_anchors, cblur, w, h, &anchor_cur_corners, &predicted,
                                 frame_id, cfg.anchor_max_features);
                free(anchor_cur_corners.data);
                match_anchor_sets(&anchors, &cur_anchors, cfg.anchor_max_hamming,
                                  cfg.anchor_mutual, cfg.anchor_ratio, &anchor_matches);
                if (anchor_matches.size >= 8) {
                    active_e_prev = &anchors.corners;
                    active_e_cur = &cur_anchors.corners;
                    active_e_prev_pose = &anchors.pose;
                    active_e_matches = &anchor_matches;
                }
                prof.feature += now_seconds() - t0;
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
            if (map.size > 1000 && linked_points >= 12) {
                Pose pred_lm = predicted;
                t0 = now_seconds();
                refine_pose_lm(&map, &tracked, fx, fy, cx, cy, cfg.pose_lm_iters, &pred_lm);
                prof.pred_lm += now_seconds() - t0;
                pred_lm_inliers = count_pose_inliers(&map, &tracked, fx, fy, cx, cy, &pred_lm);
            }
            Pose pnp_pose_for_dump;
            int has_pnp_pose_for_dump = 0;
            int pnp_ok = 0;
            int low_e_override = 0;
            Pose low_e_pose;
            t0 = now_seconds();
            pnp_ok = estimate_pose_PnP(&map, &tracked, fx, fy, cx, cy, cfg.pnp_dlt_iters,
                                       cfg.pnp_min_obs, &pose, &pnp_inliers);
            prof.pnp += now_seconds() - t0;
            if (pnp_ok) {
                pnp_pose_for_dump = pose;
                has_pnp_pose_for_dump = 1;
            }
            double pnp_predicted_jump = 0.0;
            double pnp_pose_jump = 0.0;
            if (pnp_ok && cfg.pnp_low_e_fallback &&
                (cfg.pnp_low_e_min_jump > 0.0 ||
                 cfg.pnp_low_e_min_pose_jump > 0.0)) {
                double pnp_center[3];
                camera_center_from_pose(&pose, pnp_center);
                if (cfg.pnp_low_e_min_jump > 0.0) {
                    double pred_center[3];
                    camera_center_from_pose(&predicted, pred_center);
                    pnp_predicted_jump = vec3_dist(pred_center, pnp_center);
                }
                if (cfg.pnp_low_e_min_pose_jump > 0.0) {
                    double prev_center[3];
                    camera_center_from_pose(&prev.pose, prev_center);
                    pnp_pose_jump = vec3_dist(prev_center, pnp_center);
                }
            }
            if (pnp_ok && cfg.pnp_low_e_fallback &&
                (cfg.pnp_low_e_min_map_points <= 0 ||
                 map.size >= cfg.pnp_low_e_min_map_points) &&
                pnp_inliers <= cfg.pnp_low_e_max_inliers &&
                (cfg.pnp_low_e_min_jump <= 0.0 ||
                 pnp_predicted_jump >= cfg.pnp_low_e_min_jump) &&
                (cfg.pnp_low_e_min_pose_jump <= 0.0 ||
                 pnp_pose_jump >= cfg.pnp_low_e_min_pose_jump)) {
                Pose low_rel;
                unsigned char *low_mask = NULL;
                int low_inliers = 0;
                t0 = now_seconds();
                int low_e_ok = estimate_pose_E(active_e_prev, active_e_cur, active_e_matches,
                                               fx, fy, cx, cy, &low_rel,
                                               cfg.essential_iters,
                                               cfg.essential_cheirality_max,
                                               &low_mask, &low_inliers);
                if (low_e_ok)
                    write_e_inlier_pairs(diag.e_inlier_dump, frame_id,
                                         active_e_cur == &tracked ? "pnp_low_pose"
                                                                 : "pnp_low_anchor_pose",
                                         active_e_prev, active_e_cur, active_e_matches,
                                         low_mask, w, h);
                prof.essential += now_seconds() - t0;
                if (low_e_ok &&
                    low_inliers >= cfg.pnp_low_e_min_inliers &&
                    low_inliers >= pnp_inliers + cfg.pnp_low_e_min_gain) {
                    pose_compose_relative(&low_rel, active_e_prev_pose, &low_e_pose);
                    free(mask);
                    mask = low_mask;
                    low_mask = NULL;
                    e_inliers = low_inliers;
                    low_e_override = 1;
                }
                free(low_mask);
            }
            if (pnp_ok && !low_e_override) {
                inl = pnp_inliers;
                t0 = now_seconds();
                refine_pose_lm(&map, &tracked, fx, fy, cx, cy, cfg.pose_lm_iters, &pose);
                prof.pose_lm += now_seconds() - t0;
                method = 2;
            } else {
                int e_ok = 0;
                if (low_e_override) {
                    pose = low_e_pose;
                    inl = e_inliers;
                    e_ok = 1;
                } else {
                    t0 = now_seconds();
                    e_ok = estimate_pose_E(active_e_prev, active_e_cur, active_e_matches,
                                           fx, fy, cx, cy, &rel,
                                           cfg.essential_iters, cfg.essential_cheirality_max,
                                           &mask, &inl);
                    if (e_ok)
                        write_e_inlier_pairs(diag.e_inlier_dump, frame_id,
                                             active_e_cur == &tracked ? "pose" : "anchor_pose",
                                             active_e_prev, active_e_cur, active_e_matches, mask, w, h);
                    prof.essential += now_seconds() - t0;
                }
                if (e_ok) {
                    if (!low_e_override) {
                        e_inliers = inl;
                        pose_compose_relative(&rel, active_e_prev_pose, &pose);
                    }
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
            write_track_summary(diag.track_dump, frame_id, &tracked, &matches, &map, w, h);
            dump_pnp_frame(diag.pnp_dump, frame_id, &map, &tracked, fx, fy, cx, cy, &predicted,
                           has_pnp_pose_for_dump ? &pnp_pose_for_dump : NULL, pnp_inliers,
                           has_pnp_pose_for_dump ? &pose : NULL);
            mkf = should_make_keyframe(&cfg, frame_id, inl,
                                       rotation_degrees_between(&lkf_pose, &pose));
            if (mkf && cfg.kf_min_interval > 0 && kf_db.size > 0) {
                int last_kf_frame = kf_db.data[kf_db.size - 1].frame_id;
                if (frame_id - last_kf_frame < cfg.kf_min_interval)
                    mkf = 0;
            }
            if (mkf) {
                if (inl >= 8) {
                    t0 = now_seconds();
                    int adm_candidates = 0, adm_accepted = 0;
                    double adm_sum_reproj = 0.0, adm_sum_parallax = 0.0, adm_sum_depth = 0.0;
                    double adm_min_depth = DBL_MAX, adm_max_depth = 0.0;
                    Pose admission_pose = pose;
                    Pose admission_rel;
                    int have_admission_rel = 0;
                    unsigned char *admission_mask =
                        (active_e_prev == &prev.corners && active_e_cur == &tracked &&
                         active_e_matches == &matches) ? mask : NULL;
                    unsigned char *owned_admission_mask = NULL;
                    if (active_e_cur != &tracked || cfg.tri_map_scale) {
                        // Pose E ran on anchors; re-estimate E on the LK matches to get
                        // an inlier mask over prev->tracked correspondences for admission.
                        // The map-scale triangulation path also re-estimates so its
                        // relative pose and mask come from the same geometry.
                        int adm_inl = 0;
                        if (estimate_pose_E(&prev.corners, &tracked, &matches,
                                            fx, fy, cx, cy, &admission_rel,
                                            cfg.essential_iters,
                                            cfg.essential_cheirality_max,
                                            &owned_admission_mask, &adm_inl)) {
                            write_e_inlier_pairs(diag.e_inlier_dump, frame_id, "admission_lk",
                                                 &prev.corners, &tracked, &matches,
                                                 owned_admission_mask, w, h);
                            admission_mask = owned_admission_mask;
                            have_admission_rel = 1;
                        } else {
                            free(owned_admission_mask);
                            owned_admission_mask = NULL;
                        }
                    }
                    if (cfg.tri_map_scale && have_admission_rel && admission_mask) {
                        // The world pose pair drifts in scale, so triangulating against
                        // it bakes the drift into every new landmark (birth depth tracks
                        // the world baseline almost exactly). The E relative pose carries
                        // the geometry the inlier mask actually certifies but with an
                        // arbitrary translation scale; recover that scale from the depth
                        // ratios of already-linked map points and triangulate new
                        // landmarks against the rescaled E pose instead.
                        Pose e_pose;
                        pose_compose_relative(&admission_rel, &prev.pose, &e_pose);
                        double *ratios = (double *)malloc((size_t)matches.size * sizeof(double));
                        int n_ratios = 0;
                        if (!ratios) {
                            fprintf(stderr, "out of memory\n");
                            exit(1);
                        }
                        for (int j = 0; j < matches.size; j++) {
                            if (!admission_mask[j])
                                continue;
                            Match am = matches.data[j];
                            int pi = tracked.data[am.train_idx].pt_idx;
                            if (pi == -1)
                                continue;
                            double Xe[3];
                            if (!triangulate_point(&prev.pose, &e_pose,
                                                   prev.corners.data[am.query_idx],
                                                   tracked.data[am.train_idx],
                                                   fx, fy, cx, cy, Xe))
                                continue;
                            double Xm[3] = {map.data[pi].x, map.data[pi].y, map.data[pi].z};

                            double z_tri = pose_point_depth(&e_pose, Xe);
                            double z_map = pose_point_depth(&e_pose, Xm);
                            if (z_tri > 1e-9 && z_map > 1e-9 &&
                                isfinite(z_tri) && isfinite(z_map))
                                ratios[n_ratios++] = z_map / z_tri;
                        }
                        if (n_ratios >= 8) {
                            qsort(ratios, (size_t)n_ratios, sizeof(double), cmp_double_asc);
                            double s = ratios[n_ratios / 2];

                            // Scaling the relative translation by s scales the whole
                            // two-view triangulation by s about the previous camera
                            // center, so new births land at the linked points' scale.
                            Pose rel_scaled = admission_rel;
                            rel_scaled.m[3]  *= s;
                            rel_scaled.m[7]  *= s;
                            rel_scaled.m[11] *= s;
                            pose_compose_relative(&rel_scaled, &prev.pose, &admission_pose);
                        }
                        free(ratios);
                    }
                    for (int j = 0; j < matches.size; j++) {
                        if (!admission_mask || !admission_mask[j])
                            continue;
                        Match am = matches.data[j];
                        if (tracked.data[am.train_idx].pt_idx == -1) {
                            adm_candidates++;
                            Corner pc = prev.corners.data[am.query_idx];
                            Corner cc = tracked.data[am.train_idx];
                            int adm_cell = image_grid_cell(cc.x, cc.y, w, h,
                                                           ADMISSION_GRID_COLS,
                                                           ADMISSION_GRID_ROWS);
                            double X[3];
                            if (!triangulate_point(&prev.pose, &admission_pose, pc, cc,
                                                   fx, fy, cx, cy, X)) {
                                write_map_admission_detail(
                                    diag.map_admission_detail_dump, frame_id, "lk",
                                    "tri_fail", method, inl, j, am.query_idx,
                                    am.train_idx, adm_cell, &prev.pose, &admission_pose,
                                    NAN, NAN, NAN, NAN, NAN, cc.fb_err, cc.track_disp,
                                    (double)j);
                                continue;
                            }
                            Brief256 _d = {{0, 0, 0, 0}};
                            compute_brief(cblur, w, h, cc.x, cc.y, &_d);

                            double z1 = 0.0, z2 = 0.0;
                            double e1 = reprojection_error_xyz(&prev.pose, X, pc, fx, fy, cx, cy, &z1);
                            double e2 = reprojection_error_xyz(&admission_pose, X, cc, fx, fy, cx, cy, &z2);
                            double depth    = 0.5 * (z1 + z2);
                            double reproj   = 0.5 * (e1 + e2);
                            double parallax = triangulation_parallax_deg(&prev.pose, &admission_pose, X);

                            tracked.data[am.train_idx].pt_idx = map.size;
                            adm_sum_reproj += reproj;
                            adm_sum_parallax += parallax;
                            adm_sum_depth += depth;
                            if (depth < adm_min_depth)
                                adm_min_depth = depth;
                            if (depth > adm_max_depth)
                                adm_max_depth = depth;
                            adm_accepted++;
                            write_map_admission_detail(
                                diag.map_admission_detail_dump, frame_id, "lk", "accept",
                                method, inl, j, am.query_idx, am.train_idx, adm_cell,
                                &prev.pose, &admission_pose, reproj, parallax, depth, z1, z2,
                                cc.fb_err, cc.track_disp, (double)j);
                            MapPoint _mp = {X[0], X[1], X[2], cfg.new_point_obs, 0, 0, _d};
                            map_push(&map, _mp);
                            record_map_lifecycle(
                                &map_lifecycle, map_lifecycle_enabled,
                                tracked.data[am.train_idx].pt_idx, frame_id, "lk",
                                method, inl, adm_cell, reproj, parallax, depth,
                                cc.fb_err, cc.track_disp, (double)j);
                            pts++;
                            added++;
                            tri++;
                        } else {
                            int pi = tracked.data[am.train_idx].pt_idx;
                            map.data[pi].obs++;
                            mark_map_lifecycle_observed(&map_lifecycle,
                                                        map_lifecycle_enabled,
                                                        pi, frame_id);
                        }
                    }
                    write_map_admission_summary(diag.map_admission_dump, frame_id, adm_candidates,
                                                adm_accepted, &prev.pose, &admission_pose,
                                                fx, fy, cx, cy,
                                                adm_sum_reproj, adm_sum_parallax,
                                                adm_sum_depth, adm_min_depth, adm_max_depth);
                    free(owned_admission_mask);
                    prof.triangulate += now_seconds() - t0;
                }
                if (tracked.size < (cfg.max_points * 3) / 5) {
                    t0 = now_seconds();
                    extract_features_pure(cblur, w, h, &tracked, cfg.max_points);
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
                if (config_allows_ba(&cfg, kf_db.size, map.size)) {
                    t0 = now_seconds();
                    if (cfg.joint_ba) {
                        joint_local_ba(&kf_db, &map, fx, fy, cx, cy);
                        int win_start = kf_db.size - 5;
                        if (win_start < 0) win_start = 0;
                        prof.ba_joint += now_seconds() - t0;
                        prof.ba += prof.ba_joint;
                        t0 = now_seconds();
                        cull_map_points_window(&kf_db, &map, win_start, kf_db.size,
                                               fx, fy, cx, cy, 8.0);
                        prof.map_hygiene += now_seconds() - t0;
                    } else {
                        local_ba(&kf_db, &map, fx, fy, cx, cy, cfg.pose_lm_iters,
                                 cfg.local_ba_fix_oldest);
                        prof.ba_local += now_seconds() - t0;
                        prof.ba += prof.ba_local;
                    }
                }
                if (config_allows_global_ba(&cfg, kf_db.size, map.size)) {
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
        maybe_normalize_world_scale(&cfg, &map, &kf_db, &anchors, &prev, &pose,
                                    &lkf_pose, &last_rel);
        double c[3];
        camera_center_from_pose(&pose, c);
        double out_c[3] = {c[0], c[1], c[2]};
        int output_smooth_unstable = 0;
        if (cfg.output_smooth_alpha > 0.0 && cfg.output_smooth_alpha < 1.0) {
            if (cfg.output_smooth_unstable_window > 0 &&
                cfg.output_smooth_unstable_points_added > 0 &&
                cfg.output_smooth_unstable_jump_count > 0 &&
                output_smooth_history_count > 0) {
                int recent_added = recent_int_sum(
                    output_smooth_added_history, output_smooth_history_count,
                    output_smooth_history_pos, cfg.output_smooth_unstable_window);
                int recent_jumps = recent_double_count_gt(
                    output_smooth_jump_history, output_smooth_history_count,
                    output_smooth_history_pos, cfg.output_smooth_unstable_window,
                    cfg.output_smooth_unstable_jump);
                output_smooth_unstable =
                    recent_added >= cfg.output_smooth_unstable_points_added &&
                    recent_jumps >= cfg.output_smooth_unstable_jump_count;
            }
            if (!output_smooth_initialized) {
                output_smooth_xyz[0] = c[0];
                output_smooth_xyz[1] = c[1];
                output_smooth_xyz[2] = c[2];
                output_smooth_initialized = 1;
            } else {
                double a = output_smooth_unstable &&
                                   cfg.output_smooth_unstable_alpha > 0.0
                               ? cfg.output_smooth_unstable_alpha
                               : cfg.output_smooth_alpha;
                double residual_k = output_smooth_unstable &&
                                            cfg.output_smooth_unstable_residual_k > 0.0
                                        ? cfg.output_smooth_unstable_residual_k
                                        : cfg.output_smooth_residual_k;
                if (cfg.output_smooth_low_link_alpha > 0.0 &&
                    cfg.output_smooth_low_link_alpha < a &&
                    cfg.output_smooth_low_link_threshold > 0 &&
                    cfg.output_smooth_low_link_window > 0 &&
                    cfg.output_smooth_low_link_count > 0 &&
                    output_smooth_history_count > 0) {
                    int low_link_frames = recent_int_count_lt(
                        output_smooth_linked_history, output_smooth_history_count,
                        output_smooth_history_pos, cfg.output_smooth_low_link_window,
                        cfg.output_smooth_low_link_threshold);
                    if (cfg.output_smooth_low_link_include_current &&
                        linked_points < cfg.output_smooth_low_link_threshold)
                        low_link_frames++;
                    if (low_link_frames >= cfg.output_smooth_low_link_count)
                        a = cfg.output_smooth_low_link_alpha;
                }
                double residual = vec3_dist(c, output_smooth_xyz);
                double stored_residual = residual;
                if (cfg.output_smooth_outlier_alpha > 0.0 &&
                    cfg.output_smooth_outlier_alpha < a &&
                    residual_k > 0.0 &&
                    output_smooth_residual_count > 0) {
                    double med = median_recent_values(output_smooth_residuals,
                                                      output_smooth_residual_count);
                    if (isfinite(med) && med > 0.0 &&
                        residual > residual_k * med) {
                        a = cfg.output_smooth_outlier_alpha;
                        if (output_smooth_unstable &&
                            cfg.output_smooth_unstable_cap_residual)
                            stored_residual = residual_k * med;
                    }
                }
                output_smooth_xyz[0] = a * c[0] + (1.0 - a) * output_smooth_xyz[0];
                output_smooth_xyz[1] = a * c[1] + (1.0 - a) * output_smooth_xyz[1];
                output_smooth_xyz[2] = a * c[2] + (1.0 - a) * output_smooth_xyz[2];
                output_smooth_residuals[output_smooth_residual_pos] = stored_residual;
                output_smooth_residual_pos =
                    (output_smooth_residual_pos + 1) % cfg.output_smooth_window;
                if (output_smooth_residual_count < cfg.output_smooth_window)
                    output_smooth_residual_count++;
            }
            out_c[0] = output_smooth_xyz[0];
            out_c[1] = output_smooth_xyz[1];
            out_c[2] = output_smooth_xyz[2];
        }
        if (cfg.output_smooth_unstable_window > 0 ||
            cfg.output_smooth_low_link_window > 0) {
            output_smooth_added_history[output_smooth_history_pos] = added;
            output_smooth_linked_history[output_smooth_history_pos] = linked_points;
            output_smooth_jump_history[output_smooth_history_pos] = trans_jump;
            output_smooth_history_pos = (output_smooth_history_pos + 1) % 64;
            if (output_smooth_history_count < 64)
                output_smooth_history_count++;
        }
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
                                                trans_jump,
                                                {c[0], c[1], c[2]},
                                                {out_c[0], out_c[1], out_c[2]}});
        profile_stat_vec_push(&profile, prof);
        profile.data[profile.size - 1].metrics += now_seconds() - t0;
        if ((frame_id + 1) % 10 == 0)
            printf("Frames=%d Pts=%d KF=%d Map=%d\n", frame_id + 1, pts, kf_db.size, map.size);
        memcpy(pgray, cblur, w * h);
        free(prev.corners.data);
        prev = curr;
        memset(&curr, 0, sizeof(curr));
        free(matches.data);
        free(anchor_matches.data);
        anchor_set_free(&cur_anchors);
        free(mask);
        frame_id++;
    }
    write_map_lifecycle_dump(diag.map_lifecycle_dump, &map, &map_lifecycle,
                             frame_id > 0 ? frame_id - 1 : 0);
    diagnostics_write_outputs(&cfg, &stats, &profile, pts, tri, now_seconds() - start);
    diagnostics_close_files(&diag);
    ffmpeg_close(cap);
    free(raw);
    image_gray_free(&pgray_img);
    image_gray_free(&cgray_img);
    image_gray_free(&cblur_img);
    free(prev.corners.data);
    lk_scratch_free(&lk_scratch);
    free(profile.data);
    free(stats.data);
    free(map_lifecycle.data);
    anchor_set_free(&anchors);
    kfdb_free(&kf_db);
    free(map.data);
    return 0;
}
