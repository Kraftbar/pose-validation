/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port, module M6: the map layer. See rd_map.h. */
#include "rd_map.h"
#include "rd_geom.h"
#include "rd_lie.h"
#include "rd_poisson.h"
#include "rd_ransac.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RD_PI 3.14159265358979323846

struct rd_map {
    size_t nframes, fcap; rd_frame** frames;      /* std::deque<std::unique_ptr<Frame>> */
    size_t ntracks, tcap; rd_track** tracks;      /* std::vector<std::unique_ptr<Track>> */
    size_t nid, idcap; rd_track** by_id;          /* std::map<size_t, Track*> track_id_map (ascending id) */
};

static rd_map_hooks G_h;
static uint64_t G_frame_id, G_track_id;        /* Identifiable<Frame> / <Track>::generate_id */
void rd_map_set_hooks(const rd_map_hooks* h) { if (h) G_h = *h; else memset(&G_h, 0, sizeof G_h); }
void rd_map_reset_ids(void) { G_frame_id = 0; G_track_id = 0; }

static void emit(int tag, const rd_map* m, const rd_frame* f, const rd_track* t, uint64_t a, uint64_t b, uint64_t c) {
    rd_map_event e;
    if (!G_h.event) return;
    memset(&e, 0, sizeof e);
    e.tag = tag; e.map = m; e.frame = f; e.track = t; e.a = a; e.b = b; e.c = c;
    G_h.event(G_h.ctx, &e);
}
static void* grow(void* p, size_t* cap, size_t need, size_t elem) {
    if (need <= *cap) return p;
    while (*cap < need) *cap = *cap ? 2 * *cap : 16;
    return realloc(p, *cap * elem);
}

rd_image* rd_image_retain(rd_image* im) { if (im) im->refs++; return im; }
void rd_image_release(rd_image* im) { if (im && --im->refs == 0 && im->destroy) im->destroy(im); }
void rd_imu_list_insert(rd_imu_sample** a, size_t* n, size_t* cap, size_t at, const rd_imu_sample* src, size_t count) {
    if (!count) return;
    *a = (rd_imu_sample*)grow(*a, cap, *n + count, sizeof(rd_imu_sample));
    memmove(*a + at + count, *a + at, (*n - at) * sizeof(rd_imu_sample));
    memcpy(*a + at, src, count * sizeof(rd_imu_sample));
    *n += count;
}
void rd_frame_sub_push(rd_frame* f, rd_frame* sub) {
    f->sub = (rd_frame**)grow(f->sub, &f->csub, f->nsub + 1, sizeof(rd_frame*));
    f->sub[f->nsub++] = sub;
}
rd_frame* rd_frame_sub_pop(rd_frame* f) { return f->nsub ? f->sub[--f->nsub] : NULL; }
rd_frame* rd_frame_sub_take(rd_frame* f, size_t i) {
    rd_frame* s = f->sub[i];
    memmove(f->sub + i, f->sub + i + 1, (f->nsub - i - 1) * sizeof(rd_frame*));
    f->nsub--;
    return s;
}

/* ------------------------------------------------------------------------------------------------------------------ Frame */
rd_frame* rd_frame_new(void) {
    rd_frame* f = (rd_frame*)calloc(1, sizeof *f);
    f->id = ++G_frame_id;
    f->pose_q.w = 1.0; f->cam_q.w = 1.0; f->imu_q.w = 1.0; f->delta_q.w = 1.0;
    rd_pi_reset(&f->preint); rd_pi_reset(&f->kpreint);
    emit(3, NULL, f, NULL, f->id, 0, 0);
    return f;
}
rd_frame* rd_frame_clone(const rd_frame* src) {
    rd_frame* f = (rd_frame*)calloc(1, sizeof *f);
    f->id = src->id;                                /* Identifiable(const Identifiable&) keeps the id */
    f->tags = src->tags;                            /* Tagged(frame) */
    emit(3, NULL, f, NULL, f->id, 1, (uint64_t)(uintptr_t)src);
    memcpy(f->K, src->K, sizeof f->K);
    f->pose_q = src->pose_q; memcpy(f->pose_p, src->pose_p, sizeof f->pose_p);
    f->cam_q = src->cam_q; memcpy(f->cam_p, src->cam_p, sizeof f->cam_p);
    f->imu_q = src->imu_q; memcpy(f->imu_p, src->imu_p, sizeof f->imu_p);
    f->delta_q = src->delta_q;
    f->t = src->t;
    f->image = rd_image_retain(src->image);
    memcpy(f->sqrt_inv_cov, src->sqrt_inv_cov, sizeof f->sqrt_inv_cov);
    f->motion = src->motion;
    f->preint = src->preint;
    rd_imu_list_insert(&f->data, &f->ndata, &f->cdata, 0, src->data, src->ndata);
    rd_pi_reset(&f->kpreint);
    f->nkp = src->nkp; f->cap = src->nkp;
    f->bearing = (double*)malloc(sizeof(double) * 3 * (src->nkp ? src->nkp : 1));
    if (src->nkp) memcpy(f->bearing, src->bearing, sizeof(double) * 3 * src->nkp);
    f->track = (rd_track**)calloc(src->nkp ? src->nkp : 1, sizeof(rd_track*));
    return f;
}
void rd_frame_free(rd_frame* f) {
    size_t i;
    if (!f) return;
    emit(4, NULL, f, NULL, 0, 0, 0);
    for (i = 0; i < f->nsub; ++i) rd_frame_free(f->sub[i]);   /* member destruction after the destructor body */
    rd_image_release(f->image);
    free(f->sub); free(f->data); free(f->kdata);
    free(f->bearing); free(f->track); free(f);
}
static void frame_reserve(rd_frame* f, size_t n) {
    if (n <= f->cap) return;
    while (f->cap < n) f->cap = f->cap ? 2 * f->cap : 64;
    f->bearing = (double*)realloc(f->bearing, sizeof(double) * 3 * f->cap);
    f->track = (rd_track**)realloc(f->track, sizeof(rd_track*) * f->cap);
}
void rd_frame_append_keypoint(rd_frame* f, const double b[3]) {
    frame_reserve(f, f->nkp + 1);
    memcpy(f->bearing + 3 * f->nkp, b, 3 * sizeof(double));
    f->track[f->nkp] = NULL;
    f->nkp++;
    emit(17, NULL, f, NULL, 0, 0, 0);
}
rd_track* rd_frame_get_track(rd_frame* f, size_t kp, rd_map* allocation_map) {
    if (!allocation_map) allocation_map = f->map;
    if (f->track[kp] == NULL) {
        rd_track* t = rd_map_create_track(allocation_map);
        rd_track_add_keypoint(t, f, kp);
    }
    return f->track[kp];
}
/* apply_k: (p0 / p2 * K(0,0) + K(0,2), p1 / p2 * K(1,1) + K(1,2)) */
void rd_apply_k(const double b[3], const double K[9], double px[2]) {
    px[0] = b[0] / b[2] * K[0] + K[6];
    px[1] = b[1] / b[2] * K[4] + K[7];
}
/* remove_k: ((x - K(0,2)) / K(0,0), (y - K(1,2)) / K(1,1), 1).normalized() */
void rd_remove_k(const double px[2], const double K[9], double b[3]) {
    double v[3];
    v[0] = (px[0] - K[6]) / K[0];
    v[1] = (px[1] - K[7]) / K[4];
    v[2] = 1.0;
    memcpy(b, v, sizeof v);
    ok_v3_normalized(v, b);
}

int rd_frame_detect_keypoints(rd_frame* f, rd_detect_fn detect, void* ctx) {
    double* px = (double*)malloc(sizeof(double) * 2 * (f->nkp ? f->nkp : 1));
    double* all = NULL;
    size_t i, n = 0;
    const size_t old = f->nkp;
    for (i = 0; i < f->nkp; ++i) rd_apply_k(f->bearing + 3 * i, f->K, px + 2 * i);
    if (!detect(ctx, f, px, f->nkp, &all, &n) || n < old) { free(px); free(all); return 0; }
    frame_reserve(f, n);
    for (i = old; i < n; ++i) { rd_remove_k(all + 2 * i, f->K, f->bearing + 3 * i); f->track[i] = NULL; }
    f->nkp = n;
    if (G_h.event) {
        rd_map_event e;
        memset(&e, 0, sizeof e);
        e.tag = 18; e.frame = f; e.a = old; e.b = n;
        G_h.event(G_h.ctx, &e);
    }
    free(px); free(all);
    return 1;
}

/* ------------------------------------------------------------------------------------------------------------------ sort */
/* introsort as in SGI STL / libstdc++'s std::sort (median of three moved to the first slot, unguarded partition, depth limit
 * 2 floor(log2 n), heap sort fallback, final insertion sort with a guarded first 16 and unguarded rest) */
typedef struct srt { unsigned char* b; size_t sz; int (*lt)(const void*, const void*); unsigned char tmp[128], tmp2[128]; } srt;
#define EL(s, i) ((s)->b + (size_t)(i) * (s)->sz)
static void s_swap(srt* s, long i, long j) { memcpy(s->tmp2, EL(s, i), s->sz); memcpy(EL(s, i), EL(s, j), s->sz); memcpy(EL(s, j), s->tmp2, s->sz); }
static void s_insertion(srt* s, long first, long last) {
    long i;
    if (first == last) return;
    for (i = first + 1; i != last; ++i) {
        memcpy(s->tmp, EL(s, i), s->sz);
        if (s->lt(s->tmp, EL(s, first))) {
            memmove(EL(s, first + 1), EL(s, first), (size_t)(i - first) * s->sz);
            memcpy(EL(s, first), s->tmp, s->sz);
        } else {
            long cur = i, nx = i - 1;
            while (s->lt(s->tmp, EL(s, nx))) { memcpy(EL(s, cur), EL(s, nx), s->sz); cur = nx; --nx; }
            memcpy(EL(s, cur), s->tmp, s->sz);
        }
    }
}
static void s_unguarded_insertion(srt* s, long first, long last) {
    long i;
    for (i = first; i != last; ++i) {
        long cur = i, nx = i - 1;
        memcpy(s->tmp, EL(s, i), s->sz);
        while (s->lt(s->tmp, EL(s, nx))) { memcpy(EL(s, cur), EL(s, nx), s->sz); cur = nx; --nx; }
        memcpy(EL(s, cur), s->tmp, s->sz);
    }
}
/* __adjust_heap with value v (in s->tmp) placed from hole `start` */
static void s_sift(srt* s, long base, long start, long len) {
    long top = start, child = start;
    while (child < (len - 1) / 2) {
        child = 2 * (child + 1);
        if (s->lt(EL(s, base + child), EL(s, base + child - 1))) --child;
        memcpy(EL(s, base + start), EL(s, base + child), s->sz); start = child;
    }
    if ((len & 1) == 0 && child == (len - 2) / 2) {
        child = 2 * (child + 1);
        memcpy(EL(s, base + start), EL(s, base + child - 1), s->sz); start = child - 1;
    }
    while (start > top) {
        const long parent = (start - 1) / 2;
        if (!s->lt(EL(s, base + parent), s->tmp)) break;
        memcpy(EL(s, base + start), EL(s, base + parent), s->sz); start = parent;
    }
    memcpy(EL(s, base + start), s->tmp, s->sz);
}
static void s_heapsort(srt* s, long first, long last) {
    long n = last - first, i;
    if (n < 2) return;
    for (i = (n - 2) / 2; ; --i) { memcpy(s->tmp, EL(s, first + i), s->sz); s_sift(s, first, i, n); if (i == 0) break; }
    while (last - first > 1) {
        --last;
        memcpy(s->tmp, EL(s, last), s->sz);
        memcpy(EL(s, last), EL(s, first), s->sz);
        s_sift(s, first, 0, last - first);
    }
}
static void s_median_to_first(srt* s, long r, long a, long b, long c) {
    if (s->lt(EL(s, a), EL(s, b))) {
        if (s->lt(EL(s, b), EL(s, c))) s_swap(s, r, b);
        else if (s->lt(EL(s, a), EL(s, c))) s_swap(s, r, c);
        else s_swap(s, r, a);
    } else if (s->lt(EL(s, a), EL(s, c))) s_swap(s, r, a);
    else if (s->lt(EL(s, b), EL(s, c))) s_swap(s, r, c);
    else s_swap(s, r, b);
}
static long s_partition(srt* s, long first, long last, long pivot) {
    for (;;) {
        while (s->lt(EL(s, first), EL(s, pivot))) ++first;
        --last;
        while (s->lt(EL(s, pivot), EL(s, last))) --last;
        if (!(first < last)) return first;
        s_swap(s, first, last);
        ++first;
    }
}
static void s_introsort(srt* s, long first, long last, long depth) {
    while (last - first > 16) {
        long cut;
        if (depth == 0) { s_heapsort(s, first, last); return; }
        --depth;
        s_median_to_first(s, first, first + 1, first + (last - first) / 2, last - 1);
        cut = s_partition(s, first + 1, last, first);
        s_introsort(s, cut, last, depth);
        last = cut;
    }
}
void rd_std_sort(void* base, size_t n, size_t size, int (*comp)(const void* a, const void* b)) {
    srt s;
    long lg = 0, k = (long)n;
    if (n < 2 || size > sizeof s.tmp) return;
    s.b = (unsigned char*)base; s.sz = size; s.lt = comp;
    while (k > 1) { k >>= 1; ++lg; }
    s_introsort(&s, 0, (long)n, 2 * lg);
    if ((long)n > 16) { s_insertion(&s, 0, 16); s_unguarded_insertion(&s, 16, (long)n); }
    else s_insertion(&s, 0, (long)n);
}

/* ------------------------------------------------------------------------------------------------------------------ track_keypoints */
typedef struct kp_len { size_t kp, len; } kp_len;
static int by_len_desc(const void* a, const void* b) { return ((const kp_len*)a)->len > ((const kp_len*)b)->len; }
static int dbl_lt(const void* a, const void* b) { return *(const double*)a < *(const double*)b; }

int rd_frame_track_keypoints(rd_frame* f, rd_frame* next, const rd_track_cfg* cfg, rd_lk_fn track, void* ctx) {
    const size_t n = f->nkp;
    double* curr = (double*)malloc(sizeof(double) * 2 * (n ? n : 1));
    double* nxt = (double*)calloc(2 * (n ? n : 1), sizeof(double));
    double* curr_h = (double*)malloc(sizeof(double) * 2 * (n ? n : 1));
    double* next_h = (double*)malloc(sizeof(double) * 2 * (n ? n : 1));
    double* next_b = (double*)malloc(sizeof(double) * 3 * (n ? n : 1));
    double* angles = (double*)malloc(sizeof(double) * (n ? n : 1));
    char* status = (char*)calloc(n ? n : 1, 1);
    char* mask = (char*)calloc(n ? n : 1, 1);
    kp_len* kl = (kp_len*)malloc(sizeof(kp_len) * (n ? n : 1));
    size_t i, na = 0, nk = 0;
    double E[9], R[9], misalignment;
    rd_poisson filter;
    int ok = 1;
    for (i = 0; i < n; ++i) rd_apply_k(f->bearing + 3 * i, f->K, curr + 2 * i);
    if (cfg->predict_keypoints) {
        /* delta_key_q = (camera.q_cs^* imu.q_cs next.delta.q next.imu.q_cs^* next.camera.q_cs)^* (left-associated products) */
        ok_quat a, b, c, d, cc = rd_quat_conj(f->cam_q), ni = rd_quat_conj(next->imu_q), dq;
        ok_quat_mul(&cc, &f->imu_q, &a);
        ok_quat_mul(&a, &next->delta_q, &b);
        ok_quat_mul(&b, &ni, &c);
        ok_quat_mul(&c, &next->cam_q, &d);
        dq = rd_quat_conj(d);
        for (i = 0; i < n; ++i) {
            double r[3];
            rd_quat_rotate(&dq, f->bearing + 3 * i, r);
            rd_apply_k(r, next->K, nxt + 2 * i);
        }
    }
    if (!track(ctx, f, next, curr, nxt, status, n)) ok = 0;
    for (i = 0; ok && i < n; ++i) {
        const double* bi = f->bearing + 3 * i;
        curr_h[2 * i] = bi[0] / bi[2]; curr_h[2 * i + 1] = bi[1] / bi[2];   /* hnormalized */
        rd_remove_k(nxt + 2 * i, next->K, next_b + 3 * i);
        next_h[2 * i] = next_b[3 * i] / next_b[3 * i + 2]; next_h[2 * i + 1] = next_b[3 * i + 1] / next_b[3 * i + 2];
    }
    if (ok) {
        rd_find_essential_matrix(n, curr_h, next_h, mask, 1.0, 0.999, 1000, 0, E);
        for (i = 0; i < n; ++i) if (!mask[i]) status[i] = 0;
        rd_find_rotation_matrix(n, f->bearing, next_b, mask, (RD_PI / 180.0) * cfg->rotation_ransac_threshold, 0.999, 1000, 0, R);
        for (i = 0; i < n; ++i)
            if (mask[i]) angles[na++] = rd_rotation_error(R, f->bearing + 3 * i, next_b + 3 * i) * 180 / RD_PI;
        rd_std_sort(angles, na, sizeof(double), dbl_lt);
        misalignment = na > 0 ? angles[na * 7 / 10] : 0;
        if (misalignment < cfg->rotation_misalignment_threshold) next->tags |= RD_TAG(RD_FT_NO_TRANSLATION);
        /* filter keypoints based on track length */
        for (i = 0; i < n; ++i) {
            if (status[i] == 0 || f->track[i] == NULL) continue;
            kl[nk].kp = i; kl[nk].len = f->track[i]->nref; nk++;
        }
        rd_std_sort(kl, nk, sizeof(kp_len), by_len_desc);
        rd_poisson_init(&filter, cfg->min_keypoint_distance);
        for (i = 0; i < nk; ++i) {
            const double* pt = nxt + 2 * kl[i].kp;
            const rd_track* t = f->track[kl[i].kp];
            if (rd_poisson_permit_point(&filter, pt) && (!t || !(t->tags & RD_TAG(RD_TT_TRASH)))) rd_poisson_preset_point(&filter, pt);
            else status[kl[i].kp] = 0;
        }
        rd_poisson_free(&filter);
        if (G_h.event) {
            rd_map_event e;
            memset(&e, 0, sizeof e);
            e.tag = 19; e.frame = f; e.a = n; e.c = (uint64_t)(uintptr_t)status;
            e.flag1 = (next->tags & RD_TAG(RD_FT_NO_TRANSLATION)) != 0;
            G_h.event(G_h.ctx, &e);
        }
        for (i = 0; i < n; ++i) {
            if (status[i]) {
                const size_t ni = next->nkp;
                rd_frame_append_keypoint(next, next_b + 3 * i);
                rd_track_add_keypoint(rd_frame_get_track(f, i, NULL), next, ni);
            }
        }
    }
    free(curr); free(nxt); free(curr_h); free(next_h); free(next_b); free(angles); free(status); free(mask); free(kl);
    return ok;
}

/* ------------------------------------------------------------------------------------------------------------------ Track */
static size_t ref_lower(const rd_track* t, uint64_t id) {
    size_t lo = 0, hi = t->nref;
    while (lo < hi) { const size_t mid = lo + (hi - lo) / 2; if (t->ref[mid].frame->id < id) lo = mid + 1; else hi = mid; }
    return lo;
}
size_t rd_track_keypoint_index(const rd_track* t, const rd_frame* f) {
    const size_t i = ref_lower(t, f->id);
    return i < t->nref && t->ref[i].frame->id == f->id ? t->ref[i].kp : RD_NIL;
}
void rd_track_add_keypoint(rd_track* t, rd_frame* f, size_t kp) {
    const size_t i = ref_lower(t, f->id);
    if (i < t->nref && t->ref[i].frame->id == f->id) {
        t->ref[i].kp = kp;                       /* operator[] on an existing key: the stored key (frame pointer) stays */
    } else {
        t->ref = (rd_kref*)grow(t->ref, &t->cap, t->nref + 1, sizeof(rd_kref));
        memmove(t->ref + i + 1, t->ref + i, (t->nref - i) * sizeof(rd_kref));
        t->ref[i].frame = f; t->ref[i].kp = kp;
        t->nref++;
    }
    f->track[kp] = t;
    if (G_h.prepare_add) G_h.prepare_add(G_h.ctx, t);
    if (t->tags & RD_TAG(RD_TT_TRIANGULATED)) t->life++;
    else t->life = 1;
    emit(15, NULL, f, t, kp, 0, 0);
}
static void obs_of(const rd_frame* f, size_t kp, rd_obs* o) {
    o->pose_q = f->pose_q; memcpy(o->pose_p, f->pose_p, sizeof o->pose_p);
    o->cam_q = f->cam_q; memcpy(o->cam_p, f->cam_p, sizeof o->cam_p);
    memcpy(o->keypoint, f->bearing + 3 * kp, sizeof o->keypoint);
}
static void map_recycle_track(rd_map* m, rd_track* t);
void rd_track_remove_keypoint(rd_track* t, rd_frame* f, int suicide_if_empty) {
    const size_t i = ref_lower(t, f->id);
    const size_t kp = t->ref[i].kp;            /* keypoint_refs.at(frame) */
    const int first = t->ref[0].frame == f;    /* frame == first_frame(): pointer comparison */
    int reanchor = 0;
    double landmark[3], inv_before = t->inv_depth;
    rd_map_event e;
    if (first && t->nref > 1 && G_h.prepare_reanchor) G_h.prepare_reanchor(G_h.ctx, t, f, t->ref[1].frame);
    if (first) {
        rd_obs o;
        inv_before = t->inv_depth;
        obs_of(t->ref[0].frame, t->ref[0].kp, &o);
        rd_track_get_landmark_point(&o, t->inv_depth, landmark);
    }
    f->track[kp] = NULL;
    memmove(t->ref + i, t->ref + i + 1, (t->nref - i - 1) * sizeof(rd_kref));
    t->nref--;
    if (t->nref > 0) {
        if (first) {
            rd_obs o;
            obs_of(t->ref[0].frame, t->ref[0].kp, &o);
            t->inv_depth = rd_track_set_landmark_point(&o, landmark);
            reanchor = 1;
        }
    } else {
        t->tags &= ~RD_TAG(RD_TT_VALID);
    }
    if (G_h.event) {
        memset(&e, 0, sizeof e);
        e.tag = 16; e.frame = f; e.track = t; e.a = kp; e.b = t->nref; e.flag1 = suicide_if_empty; e.flag2 = first;
        e.reanchor = reanchor; e.inv_before = inv_before; e.inv_after = t->inv_depth;
        if (reanchor) { e.new_first = t->ref[0].frame; e.new_kp = t->ref[0].kp; }
        G_h.event(G_h.ctx, &e);
    }
    if (t->nref == 0 && suicide_if_empty) map_recycle_track(t->map, t);
}

/* ------------------------------------------------------------------------------------------------------------------ Map */
rd_map* rd_map_new(void) {
    rd_map* m = (rd_map*)calloc(1, sizeof *m);
    emit(1, m, NULL, NULL, 0, 0, 0);
    return m;
}
void rd_map_free(rd_map* m) {
    size_t i;
    if (!m) return;
    emit(2, m, NULL, NULL, 0, 0, 0);
    for (i = 0; i < m->nframes; ++i) rd_frame_free(m->frames[i]);
    for (i = 0; i < m->ntracks; ++i) { free(m->tracks[i]->ref); free(m->tracks[i]); }
    free(m->frames); free(m->tracks); free(m->by_id); free(m);
}
size_t rd_map_frame_num(const rd_map* m) { return m->nframes; }
rd_frame* rd_map_get_frame(const rd_map* m, size_t index) { return m->frames[index]; }
size_t rd_map_track_num(const rd_map* m) { return m->ntracks; }
rd_track* rd_map_get_track(const rd_map* m, size_t index) { return m->tracks[index]; }

void rd_map_attach_frame(rd_map* m, rd_frame* f, size_t position) {
    const size_t at = position == RD_NIL ? m->nframes : position;
    f->map = m;
    m->frames = (rd_frame**)grow(m->frames, &m->fcap, m->nframes + 1, sizeof(rd_frame*));
    memmove(m->frames + at + 1, m->frames + at, (m->nframes - at) * sizeof(rd_frame*));
    m->frames[at] = f;
    m->nframes++;
    emit(5, m, f, NULL, position, m->nframes, 0);
}
static rd_frame* detach(rd_map* m, size_t index, int report) {
    rd_frame* f = m->frames[index];
    memmove(m->frames + index, m->frames + index + 1, (m->nframes - index - 1) * sizeof(rd_frame*));
    m->nframes--;
    f->map = NULL;
    if (report) emit(6, m, f, NULL, index, 0, 0);
    return f;
}
rd_frame* rd_map_detach_frame(rd_map* m, size_t index) { return detach(m, index, 1); }
void rd_map_untrack_frame(rd_map* m, rd_frame* f) {
    size_t i;
    emit(7, m, f, NULL, 0, 0, 0);
    for (i = 0; i < f->nkp; ++i)
        if (f->track[i]) rd_track_remove_keypoint(f->track[i], f, 1);
}
void rd_map_erase_frame(rd_map* m, size_t index) {
    rd_frame* f = m->frames[index];
    emit(8, m, f, NULL, index, 0, 0);
    rd_map_untrack_frame(m, f);
    rd_frame_free(rd_map_detach_frame(m, index));
}
void rd_map_marginalize_frame(rd_map* m, size_t index) {
    rd_frame* f;
    size_t i;
    if (G_h.marginalize) G_h.marginalize(G_h.ctx, m, index);
    f = m->frames[index];
    emit(9, m, f, NULL, index, 0, 0);
    for (i = 0; i < f->nkp; ++i)
        if (f->track[i]) rd_track_remove_keypoint(f->track[i], f, 1);
    rd_frame_free(detach(m, index, 0));          /* frames.erase(frames.begin() + index): no detach_frame call */
}
size_t rd_map_frame_index_by_id(const rd_map* m, uint64_t id) {
    size_t lo = 0, hi = m->nframes, r;
    while (lo < hi) { const size_t mid = lo + (hi - lo) / 2; if (m->frames[mid]->id < id) lo = mid + 1; else hi = mid; }
    if (lo == m->nframes) r = RD_NIL;
    else if (id < m->frames[lo]->id) r = RD_NIL;
    else r = lo;
    if (G_h.event) emit(10, m, NULL, NULL, id, r, 0);
    return r;
}
rd_track* rd_map_create_track(rd_map* m) {
    rd_track* t = (rd_track*)calloc(1, sizeof *t);
    size_t lo = 0, hi;
    t->id = ++G_track_id;
    t->tags = RD_TAG(RD_TT_STATIC);
    t->map_index = m->ntracks;
    t->map = m;
    m->by_id = (rd_track**)grow(m->by_id, &m->idcap, m->nid + 1, sizeof(rd_track*));
    hi = m->nid;
    while (lo < hi) { const size_t mid = lo + (hi - lo) / 2; if (m->by_id[mid]->id < t->id) lo = mid + 1; else hi = mid; }
    memmove(m->by_id + lo + 1, m->by_id + lo, (m->nid - lo) * sizeof(rd_track*));
    m->by_id[lo] = t; m->nid++;
    emit(11, m, NULL, t, t->map_index, 0, 0);
    m->tracks = (rd_track**)grow(m->tracks, &m->tcap, m->ntracks + 1, sizeof(rd_track*));
    m->tracks[m->ntracks++] = t;
    return t;
}
void rd_map_erase_track(rd_map* m, rd_track* t) {
    emit(12, m, NULL, t, 0, 0, 0);
    while (t->nref > 0) rd_track_remove_keypoint(t, t->ref[0].frame, 0);
    map_recycle_track(m, t);
}
void rd_map_prune_tracks(rd_map* m, int (*condition)(void* ctx, const rd_track* t), void* ctx) {
    rd_track** sel = (rd_track**)malloc(sizeof(rd_track*) * (m->ntracks ? m->ntracks : 1));
    size_t i, n = 0;
    for (i = 0; i < m->ntracks; ++i) if (condition(ctx, m->tracks[i])) sel[n++] = m->tracks[i];
    if (G_h.event) {
        rd_map_event e;
        memset(&e, 0, sizeof e);
        e.tag = 13; e.map = m; e.a = n; e.c = (uint64_t)(uintptr_t)sel;
        G_h.event(G_h.ctx, &e);
    }
    for (i = 0; i < n; ++i) rd_map_erase_track(m, sel[i]);
    free(sel);
}
rd_track* rd_map_get_track_by_id(const rd_map* m, uint64_t id) {
    size_t lo = 0, hi = m->nid;
    while (lo < hi) { const size_t mid = lo + (hi - lo) / 2; if (m->by_id[mid]->id < id) lo = mid + 1; else hi = mid; }
    return lo < m->nid && m->by_id[lo]->id == id ? m->by_id[lo] : NULL;
}
static void map_recycle_track(rd_map* m, rd_track* t) {
    rd_track* back = m->tracks[m->ntracks - 1];
    size_t lo = 0, hi = m->nid;
    emit(14, m, NULL, t, t->map_index, t->map_index != back->map_index ? back->id : 0, 0);
    if (t->map_index != back->map_index) {
        m->tracks[t->map_index] = back;          /* tracks[i].swap(tracks.back()) */
        m->tracks[m->ntracks - 1] = t;
        back->map_index = t->map_index;
    }
    while (lo < hi) { const size_t mid = lo + (hi - lo) / 2; if (m->by_id[mid]->id < t->id) lo = mid + 1; else hi = mid; }
    if (lo < m->nid && m->by_id[lo] == t) { memmove(m->by_id + lo, m->by_id + lo + 1, (m->nid - lo - 1) * sizeof(rd_track*)); m->nid--; }
    m->ntracks--;                                /* tracks.pop_back() destroys t */
    free(t->ref); free(t);
}

void rd_map_set_track_id(rd_map* m, rd_track* t, uint64_t id) {
    size_t i, k;
    for (i = 0; i < m->nid; ++i) if (m->by_id[i] == t) break;
    if (i < m->nid) { memmove(m->by_id + i, m->by_id + i + 1, (m->nid - i - 1) * sizeof(rd_track*)); m->nid--; }
    t->id = id;
    for (k = 0; k < m->nid && m->by_id[k]->id < id; ++k) {}
    memmove(m->by_id + k + 1, m->by_id + k, (m->nid - k) * sizeof(rd_track*));
    m->by_id[k] = t; m->nid++;
}
