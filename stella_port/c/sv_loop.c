/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD 2-Clause License
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* stella_vslam e445b545: module/loop_detector.cc, global_optimization_module.cc,
 * module/loop_bundle_adjuster.cc, optimize/graph_optimizer.cc, match/projection.cc,
 * match/fuse.cc, data/graph_node.cc, data/landmark.cc. See sv_loop.h.
 *
 * This file is compiled as one translation unit together with sv_mapping.c (it #includes it) because the
 * loop code reuses that module's file-local landmark / covisibility primitives (kf_get, lm_get,
 * lm_erase_observation, lm_replace, lm_connect, update_connections, detect_duplication, ...). */
#include "sv_mapping.c"
#include "sv_loop.h"
#include "sv_eigen_mat4.h"
#include "sv_g2o_sim3.h"
#include "sv_match_bow.h"

#define HAMMING_DIST_THR_HIGH 100u

/* ------------------------------------------------------------------ */
/* trace helpers                                                      */
/* ------------------------------------------------------------------ */
typedef struct tb_t {
    sv_tf f[96];
    int n;
} tb_t;

static sv_tf* tb_new(tb_t* b, char k) {
    sv_tf* t = &b->f[b->n++];
    memset(t, 0, sizeof(*t));
    t->k = k;
    return t;
}
static void tb_u(tb_t* b, unsigned long v) { tb_new(b, 'u')->i = (long)v; }
static void tb_i(tb_t* b, long v) { tb_new(b, 'i')->i = v; }
static void tb_d(tb_t* b, double v) { tb_new(b, 'd')->d = v; }
static void tb_ids(tb_t* b, const int* l, unsigned int n) {
    sv_tf* t = tb_new(b, 'L');
    t->l = l;
    t->n = n;
}
static void tb_assoc(tb_t* b, const int* l, unsigned int n) {
    sv_tf* t = tb_new(b, 'A');
    t->l = l;
    t->n = n;
}
static void tb_triples(tb_t* b, const int* l, unsigned int n) {
    sv_tf* t = tb_new(b, 'T');
    t->l = l;
    t->n = n;
}
static void tb_pairs(tb_t* b, const int* l, unsigned int n) { /* n pairs, l has 2n ints */
    sv_tf* t = tb_new(b, 'P');
    t->l = l;
    t->n = n;
}
static void tb_sim3(tb_t* b, const sv_sim3* s) {
    tb_d(b, s->r.x);
    tb_d(b, s->r.y);
    tb_d(b, s->r.z);
    tb_d(b, s->r.w);
    tb_d(b, s->t[0]);
    tb_d(b, s->t[1]);
    tb_d(b, s->t[2]);
    tb_d(b, s->s);
}
static void tb_vec3(tb_t* b, const double v[3]) {
    tb_d(b, v[0]);
    tb_d(b, v[1]);
    tb_d(b, v[2]);
}
/* column-major 3x3 / 4x4 printed row-major (util::lt_mat33 / lt_mat44) */
static void tb_mat33_cm(tb_t* b, const double m[9]) {
    int r, c;
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            tb_d(b, m[c * 3 + r]);
        }
    }
}
static void tb_mat44_cm(tb_t* b, const double m[16]) {
    int r, c;
    for (r = 0; r < 4; ++r) {
        for (c = 0; c < 4; ++c) {
            tb_d(b, m[c * 4 + r]);
        }
    }
}
static void tb_mat44_rm(tb_t* b, const double m[16]) {
    int i;
    for (i = 0; i < 16; ++i) {
        tb_d(b, m[i]);
    }
}
static void tb_emit(sv_loop* L, const char* tag, const tb_t* b) {
    if (L->trace) {
        L->trace(L->trace_user, tag, b->f, b->n);
    }
}
#define TB_INIT(b) \
    tb_t b;        \
    b.n = 0

static int* ids_to_int(const unsigned int* v, unsigned int n) {
    int* r = (int*)malloc((n ? n : 1) * sizeof(int));
    unsigned int i;
    for (i = 0; i < n; ++i) {
        r[i] = v[i] == SV_LOOP_NONE ? -1 : (int)v[i];
    }
    return r;
}

/* ------------------------------------------------------------------ */
/* context                                                            */
/* ------------------------------------------------------------------ */
void sv_loop_init(sv_loop* L, const sv_tr_config* cfg, sv_mapping* mp) {
    memset(L, 0, sizeof(*L));
    L->cfg = cfg;
    L->mp = mp;
    L->map = mp->map;
    L->num_final_matches_thr = 40;
    L->min_continuity = 3;
    L->reject_by_graph_distance = 0;
    L->min_distance_on_graph = 50;
    L->num_matches_thr = 20;
    L->num_matches_thr_brute_force = 0;
    L->num_optimized_inliers_thr = 20;
    L->top_n_covisibilities_to_search = 0;
    L->num_common_words_thr_ratio = 0.8f;
    L->thr_opt1 = 10;
    L->thr_a = 25;
    L->thr_b = 40;
    L->thr_neighbor_keyframes = 15;
    L->min_num_shared_lms_graph = 100;
    L->loop_ba_num_iter = 10;
    L->selected = -1;
    sv_sim3_identity(&L->sim3_world_to_curr);
}

static void prev_free(sv_loop* L) {
    unsigned int i;
    for (i = 0; i < L->n_prev; ++i) {
        free(L->prev[i].ids);
    }
    free(L->prev);
    L->prev = NULL;
    L->n_prev = 0;
}

void sv_loop_free(sv_loop* L) {
    unsigned int i;
    prev_free(L);
    for (i = 0; i < L->edges_cap; ++i) {
        free(L->loop_edges[i]);
    }
    free(L->loop_edges);
    free(L->loop_edges_n);
    free(L->flag_desc);
    free(L->flag_pred);
    for (i = 0; i < L->bow_cap; ++i) {
        if (L->bow_ready[i]) {
            sv_bow_vector_free(&L->bow[i]);
        }
    }
    free(L->bow);
    free(L->bow_ready);
    free(L->to_validate);
    free(L->match_cand);
    free(L->match_covis);
    memset(L, 0, sizeof(*L));
}

void sv_loop_clear_prev(sv_loop* L) { prev_free(L); }

void sv_loop_add_prev(sv_loop* L, unsigned int lead, unsigned int continuity, const unsigned int* ids, unsigned int n) {
    sv_loop_set* s;
    L->prev = (sv_loop_set*)realloc(L->prev, (L->n_prev + 1) * sizeof(sv_loop_set));
    s = &L->prev[L->n_prev++];
    s->lead = lead;
    s->continuity = continuity;
    s->n = n;
    s->ids = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
    if (n) {
        memcpy(s->ids, ids, n * sizeof(unsigned int));
    }
}

static void edges_ensure(sv_loop* L, unsigned int id) {
    if (id >= L->edges_cap) {
        const unsigned int nc = id + 64;
        L->loop_edges = (unsigned int**)realloc(L->loop_edges, nc * sizeof(unsigned int*));
        L->loop_edges_n = (unsigned int*)realloc(L->loop_edges_n, nc * sizeof(unsigned int));
        memset(L->loop_edges + L->edges_cap, 0, (nc - L->edges_cap) * sizeof(unsigned int*));
        memset(L->loop_edges_n + L->edges_cap, 0, (nc - L->edges_cap) * sizeof(unsigned int));
        L->edges_cap = nc;
    }
}

void sv_loop_set_loop_edges(sv_loop* L, unsigned int kf, const unsigned int* ids, unsigned int n) {
    edges_ensure(L, kf);
    free(L->loop_edges[kf]);
    L->loop_edges[kf] = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
    if (n) {
        memcpy(L->loop_edges[kf], ids, n * sizeof(unsigned int));
    }
    L->loop_edges_n[kf] = n;
}

unsigned int sv_loop_get_loop_edges(const sv_loop* L, unsigned int kf, const unsigned int** ids) {
    if (kf >= L->edges_cap) {
        *ids = NULL;
        return 0;
    }
    *ids = L->loop_edges[kf];
    return L->loop_edges_n[kf];
}

/* graph_node::add_loop_edge (id-ordered set, unique) */
static void add_loop_edge(sv_loop* L, unsigned int kf, unsigned int other) {
    unsigned int i, n;
    edges_ensure(L, kf);
    n = L->loop_edges_n[kf];
    for (i = 0; i < n; ++i) {
        if (L->loop_edges[kf][i] == other) {
            return;
        }
    }
    L->loop_edges[kf] = (unsigned int*)realloc(L->loop_edges[kf], (n + 1) * sizeof(unsigned int));
    L->loop_edges[kf][n] = other;
    L->loop_edges_n[kf] = n + 1;
    for (i = n; i > 0 && L->loop_edges[kf][i - 1] > L->loop_edges[kf][i]; --i) {
        const unsigned int t = L->loop_edges[kf][i];
        L->loop_edges[kf][i] = L->loop_edges[kf][i - 1];
        L->loop_edges[kf][i - 1] = t;
    }
}

/* landmark cache flags (default: valid) */
static void flags_ensure(sv_loop* L, unsigned int id) {
    if (id >= L->flag_cap) {
        const unsigned int nc = id + 4096;
        L->flag_desc = (unsigned char*)realloc(L->flag_desc, nc);
        L->flag_pred = (unsigned char*)realloc(L->flag_pred, nc);
        memset(L->flag_desc + L->flag_cap, 1, nc - L->flag_cap);
        memset(L->flag_pred + L->flag_cap, 1, nc - L->flag_cap);
        L->flag_cap = nc;
    }
}
static void flags_invalidate(sv_loop* L, unsigned int id) {
    flags_ensure(L, id);
    L->flag_desc[id] = 0;
    L->flag_pred[id] = 0;
}
static void lm_compute_descriptor(sv_loop* L, sv_tr_lm* lm) {
    flags_ensure(L, lm->id);
    sv_tr_lm_compute_descriptor(L->map, lm);
    L->flag_desc[lm->id] = 1;
}
static void lm_update_mean_normal(sv_loop* L, sv_tr_lm* lm) {
    flags_ensure(L, lm->id);
    sv_tr_lm_update_mean_normal_and_obs_scale_variance(L->cfg, L->map, lm);
    L->flag_pred[lm->id] = 1;
}
/* landmark::set_pos_in_world */
static void lm_set_pos(sv_loop* L, sv_tr_lm* lm, const double p[3]) {
    flags_ensure(L, lm->id);
    lm->pos_w[0] = p[0];
    lm->pos_w[1] = p[1];
    lm->pos_w[2] = p[2];
    L->flag_pred[lm->id] = 0;
}
/* landmark::connect_to_keyframe (add_observation clears both caches) */
static void lm_connect_l(sv_loop* L, sv_tr_kf* kf, sv_tr_lm* lm, unsigned int idx) {
    lm_connect(kf, lm, idx);
    flags_invalidate(L, lm->id);
}
/* landmark::erase_observation */
static void lm_erase_observation_l(sv_loop* L, sv_tr_lm* lm, unsigned int kf_id) {
    int found;
    lm_obs_pos(lm, kf_id, &found);
    lm_erase_observation(L->mp, lm, kf_id);
    if (found) {
        flags_invalidate(L, lm->id);
    }
}
/* landmark::replace(lm) : `old` merged into `nw` */
static void lm_replace_l(sv_loop* L, sv_tr_lm* old, sv_tr_lm* nw) {
    unsigned int before = nw->num_obs;
    lm_replace(L->mp, old, nw);
    if (nw->num_obs != before) {
        flags_invalidate(L, nw->id);
    }
}

static const sv_bow_vector* kf_bow(sv_loop* L, sv_tr_kf* kf) {
    if (kf->id >= L->bow_cap) {
        const unsigned int nc = kf->id + 64;
        L->bow = (sv_bow_vector*)realloc(L->bow, nc * sizeof(sv_bow_vector));
        L->bow_ready = (unsigned char*)realloc(L->bow_ready, nc);
        memset(L->bow + L->bow_cap, 0, (nc - L->bow_cap) * sizeof(sv_bow_vector));
        memset(L->bow_ready + L->bow_cap, 0, nc - L->bow_cap);
        L->bow_cap = nc;
    }
    if (!L->bow_ready[kf->id]) {
        sv_bow_feat_vector feat;
        memset(&feat, 0, sizeof(feat));
        memset(&L->bow[kf->id], 0, sizeof(sv_bow_vector));
        sv_bow_transform(L->cfg->vocab, kf->obs->desc, kf->obs->num_kp, 4, &L->bow[kf->id], &feat);
        sv_bow_feat_vector_free(&feat);
        L->bow_ready[kf->id] = 1;
    }
    return &L->bow[kf->id];
}

/* ------------------------------------------------------------------ */
/* id sets                                                            */
/* ------------------------------------------------------------------ */
static int cmp_uint(const void* a, const void* b) {
    const unsigned int x = *(const unsigned int*)a, y = *(const unsigned int*)b;
    return (x > y) - (x < y);
}

static int idset_has(const unsigned int* ids, unsigned int n, unsigned int id) {
    unsigned int lo = 0, hi = n;
    while (lo < hi) {
        const unsigned int mid = lo + (hi - lo) / 2;
        if (ids[mid] < id) {
            lo = mid + 1;
        }
        else {
            hi = mid;
        }
    }
    return lo < n && ids[lo] == id;
}

/* sorted unique copy (in place); returns the new length */
static unsigned int idset_normalize(unsigned int* ids, unsigned int n) {
    unsigned int i, k = 0;
    qsort(ids, n, sizeof(unsigned int), cmp_uint);
    for (i = 0; i < n; ++i) {
        if (k == 0 || ids[k - 1] != ids[i]) {
            ids[k++] = ids[i];
        }
    }
    return k;
}

/* graph_node::get_connected_keyframes(): keys of the connected map; an expired key is the null pointer
 * (SV_LOOP_NONE, sorts last, unique). Returns a malloc'ed sorted set. */
static unsigned int* connected_set(sv_loop* L, unsigned int kf_id, unsigned int* n_out) {
    sv_rbtree* t = conn_tree(L->mp, kf_id);
    unsigned int* v = (unsigned int*)malloc((t->count + 1) * sizeof(unsigned int));
    unsigned int n = 0;
    int nd;
    for (nd = sv_rb_first(t); nd >= 0; nd = sv_rb_next(t, nd)) {
        const unsigned int key = t->n[nd].key;
        v[n++] = sv_mapping_is_expired(L->mp, key) ? SV_LOOP_NONE : key;
    }
    *n_out = idset_normalize(v, n);
    return v;
}

/* graph_node::get_covisibilities(): visible ordered ids */
static unsigned int covisibilities(sv_loop* L, const sv_tr_kf* kf, unsigned int* out) {
    return top_n_covisibilities(L->mp, kf, 0xFFFFFFFFu, out);
}

/* get_covisibilities_over_min_num_shared_lms(min): ordered list, entries with count >= min (upper_bound with
 * std::greater on the ordered counts); expired entries do not count towards the bound */
static unsigned int covisibilities_over(sv_loop* L, const sv_tr_kf* kf, unsigned int min_shared, unsigned int* out) {
    unsigned int i, cnt = 0, bound = kf->n_covis;
    if (kf->n_covis == 0) {
        return 0;
    }
    for (i = 0; i < kf->n_covis; ++i) { /* first element strictly smaller than min_shared */
        if (kf->covis_w[i] < min_shared) {
            bound = i;
            break;
        }
    }
    for (i = 0; i < bound; ++i) {
        if (!covis_entry_visible(L->mp, kf->covis[i])) {
            continue;
        }
        out[cnt++] = kf->covis[i];
    }
    return cnt;
}

/* ------------------------------------------------------------------ */
/* loop_detector::detect_loop_candidates                              */
/* ------------------------------------------------------------------ */
static int sets_equal(const sv_loop_set* a, const sv_loop_set* b) {
    return a->n == b->n && (a->n == 0 || memcmp(a->ids, b->ids, a->n * sizeof(unsigned int)) == 0);
}

static int idset_has_unsorted(const unsigned int* ids, unsigned int n, unsigned int id) {
    unsigned int i;
    for (i = 0; i < n; ++i) {
        if (ids[i] == id) {
            return 1;
        }
    }
    return 0;
}

static int set_intersects(const unsigned int* a, unsigned int na, const unsigned int* b, unsigned int nb) {
    unsigned int i;
    for (i = 0; i < na; ++i) {
        if (idset_has(b, nb, a[i])) {
            return 1;
        }
    }
    return 0;
}

int sv_loop_detect(sv_loop* L, unsigned int cur_id, const unsigned int* db_ids, unsigned int n_db) {
    sv_mapping* mp = L->mp;
    sv_tr_kf* cur = kf_get(mp, (int)cur_id);
    unsigned int i, k, n_cov;
    unsigned int* covis;
    float min_score = 1.0f;
    unsigned int *reject = NULL, n_reject = 0;
    int ret = 0;

    free(L->to_validate);
    L->to_validate = NULL;
    L->n_to_validate = 0;
    if (!cur) {
        return 0;
    }
    {
        TB_INIT(b);
        tb_u(&b, cur_id);
        tb_u(&b, L->prev_loop_correct_keyfrm_id);
        tb_u(&b, 1);
        tb_u(&b, L->num_final_matches_thr);
        tb_u(&b, L->min_continuity);
        tb_u(&b, L->reject_by_graph_distance ? 1 : 0);
        tb_u(&b, (unsigned int)L->min_distance_on_graph);
        tb_u(&b, L->num_matches_thr);
        tb_u(&b, L->num_matches_thr_brute_force);
        tb_u(&b, L->num_optimized_inliers_thr);
        tb_u(&b, L->top_n_covisibilities_to_search);
        tb_d(&b, (double)L->num_common_words_thr_ratio);
        tb_emit(L, "DET", &b);
    }
    if (cur_id < L->prev_loop_correct_keyfrm_id + 10) {
        return 0;
    }

    /* 1-1. minimum score among the covisibilities */
    covis = (unsigned int*)malloc((cur->n_covis + 1) * sizeof(unsigned int));
    n_cov = covisibilities(L, cur, covis);
    {
        const sv_bow_vector* b1 = kf_bow(L, cur);
        for (i = 0; i < n_cov; ++i) {
            sv_tr_kf* c = kf_get(mp, (int)covis[i]);
            float score;
            if (!c) { /* covisibility->will_be_erased() (erased but not yet destroyed) */
                L->n_stale_covis++;
                continue;
            }
            score = (float)sv_bow_score(b1, kf_bow(L, c));
            if (score < min_score) {
                min_score = score;
            }
        }
    }
    free(covis);
    {
        TB_INIT(b);
        tb_d(&b, (double)min_score);
        tb_emit(L, "MINS", &b);
    }

    /* 1-2. keyframes to reject */
    if (!L->reject_by_graph_distance) {
        reject = connected_set(L, cur_id, &n_reject);
        reject = (unsigned int*)realloc(reject, (n_reject + 1) * sizeof(unsigned int));
        reject[n_reject++] = cur_id;
        n_reject = idset_normalize(reject, n_reject);
    }
    else {
        typedef struct tgt {
            unsigned int id;
            int dist;
        } tgt;
        tgt* stack = (tgt*)malloc((L->map->kf_cap + 2) * sizeof(tgt) * 2);
        int sp = 0;
        reject = (unsigned int*)malloc((L->map->kf_cap + 2) * sizeof(unsigned int));
        n_reject = 0;
        stack[sp].id = cur_id;
        stack[sp].dist = 0;
        ++sp;
        reject[n_reject++] = cur_id;
        while (sp > 0) {
            tgt tg = stack[--sp];
            sv_tr_kf* kf = kf_get(mp, (int)tg.id);
            if (!kf) {
                continue;
            }
            if (tg.dist + 1 < L->min_distance_on_graph) {
                /* search parent */
                if (kf->parent >= 0 && kf_get(mp, kf->parent) && !idset_has_unsorted(reject, n_reject, (unsigned int)kf->parent)) {
                    reject[n_reject++] = (unsigned int)kf->parent;
                    stack[sp].id = (unsigned int)kf->parent;
                    stack[sp].dist = tg.dist + 1;
                    ++sp;
                }
                /* loop edges */
                {
                    const unsigned int* le;
                    const unsigned int nle = sv_loop_get_loop_edges(L, kf->id, &le);
                    for (k = 0; k < nle; ++k) {
                        if (idset_has_unsorted(reject, n_reject, le[k])) {
                            continue;
                        }
                        reject[n_reject++] = le[k];
                        stack[sp].id = le[k];
                        stack[sp].dist = tg.dist + 1;
                        ++sp;
                    }
                }
                /* children */
                for (k = 0; k < kf->n_children; ++k) {
                    if (idset_has_unsorted(reject, n_reject, kf->children[k])) {
                        continue;
                    }
                    reject[n_reject++] = kf->children[k];
                    stack[sp].id = kf->children[k];
                    stack[sp].dist = tg.dist + 1;
                    ++sp;
                }
            }
        }
        free(stack);
        n_reject = idset_normalize(reject, n_reject);
    }

    /* 1-2. BoW database query (ascending id == the id-ordered candidate set of patch 0012) */
    {
        sv_bow_db* db = L->ext_db ? L->ext_db : sv_bow_db_create();
        sv_bow_db_keyframe* dbk = L->ext_db ? NULL : (sv_bow_db_keyframe*)calloc(n_db + 1, sizeof(sv_bow_db_keyframe));
        const sv_bow_db_keyframe** rej = (const sv_bow_db_keyframe**)malloc((n_reject + 1) * sizeof(*rej));
        unsigned int n_rej_obj = 0;
        sv_bow_db_result res;
        unsigned int* init = NULL;
        unsigned int n_init = 0;
        int* tr_rej;
        int* tr_init;
        memset(&res, 0, sizeof(res));
        if (L->ext_db) {
            /* persistent database: its content is already db_ids */
            for (k = 0; k < n_reject; ++k) {
                const sv_bow_db_keyframe* e = L->ext_dbk(L->ext_db_user, reject[k]);
                if (e) {
                    rej[n_rej_obj++] = e;
                }
            }
        }
        else {
            for (i = 0; i < n_db; ++i) {
                sv_tr_kf* kf = kf_get(mp, (int)db_ids[i]);
                if (!kf) {
                    continue;
                }
                dbk[i].id = kf->id;
                dbk[i].bow = kf_bow(L, kf);
                sv_bow_db_add(db, &dbk[i]);
            }
            for (k = 0; k < n_reject; ++k) {
                for (i = 0; i < n_db; ++i) {
                    if (dbk[i].bow && dbk[i].id == reject[k]) {
                        rej[n_rej_obj++] = &dbk[i];
                        break;
                    }
                }
            }
        }
        sv_bow_db_query(db, kf_bow(L, cur), min_score, L->num_common_words_thr_ratio, rej, n_rej_obj, &res);
        init = (unsigned int*)malloc((res.count + 1) * sizeof(unsigned int));
        for (i = 0; i < res.count; ++i) {
            if (res.matches[i].accepted) {
                init[n_init++] = res.matches[i].keyframe->id;
            }
        }
        tr_rej = ids_to_int(reject, n_reject);
        tr_init = ids_to_int(init, n_init);
        {
            TB_INIT(b);
            tb_ids(&b, tr_rej, n_reject);
            tb_emit(L, "REJ", &b);
        }
        {
            TB_INIT(b);
            tb_ids(&b, tr_init, n_init);
            tb_emit(L, "INIT", &b);
        }
        free(tr_rej);
        free(tr_init);
        sv_bow_db_result_free(&res);
        if (!L->ext_db) {
            sv_bow_db_destroy(db);
        }
        free(dbk);
        free(rej);

        if (n_init == 0) {
            prev_free(L); /* cont_detected_keyfrm_sets_.clear() */
            free(init);
            free(reject);
            return 0;
        }

        /* 2. find_continuously_detected_keyframe_sets */
        {
            sv_loop_set* curr = (sv_loop_set*)calloc(n_init * (L->n_prev + 1) + 1, sizeof(sv_loop_set));
            unsigned int n_curr = 0, ci, pi;
            unsigned char* flag = (unsigned char*)calloc(L->n_prev + 1, 1);
            int* canon = (int*)malloc((L->n_prev + 1) * sizeof(int));
            for (pi = 0; pi < L->n_prev; ++pi) { /* already_checked[prev.keyfrm_set_] : keyed by set content */
                canon[pi] = (int)pi;
                for (i = 0; i < pi; ++i) {
                    if (sets_equal(&L->prev[i], &L->prev[pi])) {
                        canon[pi] = canon[i];
                        break;
                    }
                }
            }
            for (ci = 0; ci < n_init; ++ci) {
                unsigned int nset;
                unsigned int* set = connected_set(L, init[ci], &nset);
                int init_needed = 1;
                for (pi = 0; pi < L->n_prev; ++pi) {
                    sv_loop_set* s;
                    if (flag[canon[pi]]) {
                        continue;
                    }
                    if (!set_intersects(L->prev[pi].ids, L->prev[pi].n, set, nset)) {
                        continue;
                    }
                    init_needed = 0;
                    s = &curr[n_curr++];
                    s->ids = (unsigned int*)malloc((nset ? nset : 1) * sizeof(unsigned int));
                    memcpy(s->ids, set, nset * sizeof(unsigned int));
                    s->n = nset;
                    s->lead = init[ci];
                    s->continuity = L->prev[pi].continuity + 1;
                    flag[canon[pi]] = 1;
                }
                if (init_needed) {
                    sv_loop_set* s = &curr[n_curr++];
                    s->ids = (unsigned int*)malloc((nset ? nset : 1) * sizeof(unsigned int));
                    memcpy(s->ids, set, nset * sizeof(unsigned int));
                    s->n = nset;
                    s->lead = init[ci];
                    s->continuity = 0;
                }
                free(set);
            }
            free(flag);
            free(canon);
            for (ci = 0; ci < n_curr; ++ci) {
                int* m = ids_to_int(curr[ci].ids, curr[ci].n);
                TB_INIT(b);
                tb_i(&b, (long)curr[ci].lead);
                tb_u(&b, curr[ci].continuity);
                tb_ids(&b, m, curr[ci].n);
                tb_emit(L, "CONT", &b);
                free(m);
            }

            /* 3. adopt the sets whose continuity reached min_continuity */
            {
                unsigned int* cand = (unsigned int*)malloc((n_curr + 1) * sizeof(unsigned int));
                unsigned int nc = 0;
                int* t3;
                for (ci = 0; ci < n_curr; ++ci) {
                    if (L->min_continuity <= curr[ci].continuity) {
                        cand[nc++] = curr[ci].lead;
                    }
                }
                nc = idset_normalize(cand, nc);
                t3 = ids_to_int(cand, nc);
                {
                    TB_INIT(b);
                    tb_ids(&b, t3, nc);
                    tb_emit(L, "TOVAL3", &b);
                }
                free(t3);

                /* 4. keep the sets for the next call */
                prev_free(L);
                L->prev = curr;
                L->n_prev = n_curr;

                /* 5. add top n covisibilities to the candidates */
                if (L->top_n_covisibilities_to_search > 0) {
                    unsigned int nc0 = nc;
                    for (ci = 0; ci < nc0; ++ci) {
                        sv_tr_kf* kf = kf_get(mp, (int)cand[ci]);
                        unsigned int* cv;
                        unsigned int ncv, j;
                        if (!kf) {
                            continue;
                        }
                        cv = (unsigned int*)malloc((kf->n_covis + 1) * sizeof(unsigned int));
                        ncv = top_n_covisibilities(mp, kf, L->top_n_covisibilities_to_search, cv);
                        cand = (unsigned int*)realloc(cand, (nc + ncv + 1) * sizeof(unsigned int));
                        for (j = 0; j < ncv; ++j) {
                            if (!idset_has(reject, n_reject, cv[j])) {
                                cand[nc++] = cv[j];
                            }
                        }
                        free(cv);
                    }
                    nc = idset_normalize(cand, nc);
                }
                {
                    int* t5 = ids_to_int(cand, nc);
                    TB_INIT(b);
                    tb_ids(&b, t5, nc);
                    tb_emit(L, "TOVAL5", &b);
                    free(t5);
                }
                L->to_validate = cand;
                L->n_to_validate = nc;
                ret = nc > 0;
            }
        }
        free(init);
    }
    free(reject);
    return ret;
}

/* ------------------------------------------------------------------ */
/* small geometry helpers (column-major)                              */
/* ------------------------------------------------------------------ */
static void kf_rot_trans(const sv_tr_kf* kf, double rot[9], double trans[3]) {
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot[c * 3 + r] = kf->pose_cw[c * 4 + r];
        }
    }
    trans[0] = kf->pose_cw[3 * 4 + 0];
    trans[1] = kf->pose_cw[3 * 4 + 1];
    trans[2] = kf->pose_cw[3 * 4 + 2];
}

static void pose_from_rt(const double rot[9], const double trans[3], double out[16]) { /* converter::to_eigen_pose */
    int r, c;
    for (c = 0; c < 4; ++c) {
        for (r = 0; r < 4; ++r) {
            out[c * 4 + r] = (r == c) ? 1.0 : 0.0;
        }
    }
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            out[c * 4 + r] = rot[c * 3 + r];
        }
    }
    out[3 * 4 + 0] = trans[0];
    out[3 * 4 + 1] = trans[1];
    out[3 * 4 + 2] = trans[2];
}

static void pose_to_se3_cm(const double pose_cm[16], sv_se3* out) {
    double rot[9];
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot[c * 3 + r] = pose_cm[c * 4 + r];
        }
    }
    sv_quat_from_mat3(rot, &out->q);
    out->t[0] = pose_cm[3 * 4 + 0];
    out->t[1] = pose_cm[3 * 4 + 1];
    out->t[2] = pose_cm[3 * 4 + 2];
    sv_se3_normalize_rotation(out);
}

static void se3_to_pose_cm(const sv_se3* pose, double out[16]) {
    double rot[9];
    sv_quat_to_mat3(&pose->q, rot);
    pose_from_rt(rot, pose->t, out);
}

/* converter::to_eigen_mat(g2o::Sim3): [scale * R | t] */
static void sim3_to_mat44(const sv_sim3* s, double out[16]) {
    double R[9];
    int i;
    sv_quat_to_mat3(&s->r, R);
    for (i = 0; i < 9; ++i) {
        R[i] = s->s * R[i];
    }
    pose_from_rt(R, s->t, out);
}

/* dot of two strided row blocks (Eigen non-vectorized redux: a0*b0 + (a1*b1 + a2*b2)) */
static double dot_row_blocks(const double a[3], const double b[3]) {
    return a[0] * b[0] + (a[1] * b[1] + a[2] * b[2]);
}

/* ------------------------------------------------------------------ */
/* pose_optimizer_g2o::optimize(cam_pose_cw, frm_obs, ..., landmarks, ...)                              */
/* ------------------------------------------------------------------ */
static unsigned int pose_optimize(sv_loop* L, const sv_tr_kf* cur, const double init_pose_cm[16], const int* lms,
                                  double pose_out[16], unsigned char* outlier) {
    const sv_tr_config* cfg = L->cfg;
    const unsigned int num_kp = cur->obs->num_kp;
    sv_pose_opt_edge* edges = (sv_pose_opt_edge*)malloc((num_kp ? num_kp : 1) * sizeof(sv_pose_opt_edge));
    unsigned int* edge_idx = (unsigned int*)malloc((num_kp ? num_kp : 1) * sizeof(unsigned int));
    unsigned int idx, n = 0, i, num_valid;
    sv_se3 pose;
    memset(outlier, 0, num_kp);
    for (idx = 0; idx < num_kp; ++idx) {
        const sv_tr_lm* lm;
        sv_pose_opt_edge* e;
        if (lms[idx] < 0) {
            continue;
        }
        lm = sv_tr_map_lm(L->map, lms[idx]);
        if (!lm) {
            continue;
        }
        e = &edges[n];
        edge_idx[n] = idx;
        e->pos_w[0] = lm->pos_w[0];
        e->pos_w[1] = lm->pos_w[1];
        e->pos_w[2] = lm->pos_w[2];
        e->obs[0] = (double)cur->obs->kp[idx].x;
        e->obs[1] = (double)cur->obs->kp[idx].y;
        e->inv_sigma_sq = (double)cfg->inv_level_sigma_sq[cur->obs->kp[idx].octave];
        e->fx = cfg->fx;
        e->fy = cfg->fy;
        e->cx = cfg->cx;
        e->cy = cfg->cy;
        e->level = 0;
        ++n;
    }
    pose_to_se3_cm(init_pose_cm, &pose);
    num_valid = sv_pose_optimizer_optimize(&pose, edges, (int)n, &cfg->pose_opt);
    if (n >= 5) {
        se3_to_pose_cm(&pose, pose_out);
        for (i = 0; i < n; ++i) {
            outlier[edge_idx[i]] = (unsigned char)(edges[i].level != 0);
        }
    }
    free(edges);
    free(edge_idx);
    return num_valid;
}

static unsigned int outlier_list(const unsigned char* flags, unsigned int n, int* out) {
    unsigned int i, k = 0;
    for (i = 0; i < n; ++i) {
        if (flags[i]) {
            out[k++] = (int)i;
        }
    }
    return k;
}

static void emit_opt(sv_loop* L, const char* tag, unsigned int num_valid, const double pose_cm[16], const unsigned char* outlier, unsigned int nkp) {
    int* ol = (int*)malloc((nkp ? nkp : 1) * sizeof(int));
    const unsigned int no = outlier_list(outlier, nkp, ol);
    TB_INIT(b);
    tb_u(&b, num_valid);
    if (num_valid > 0) {
        tb_mat44_cm(&b, pose_cm);
    }
    tb_ids(&b, ol, no);
    tb_emit(L, tag, &b);
    free(ol);
}

/* ------------------------------------------------------------------ */
/* match::projection                                                  */
/* ------------------------------------------------------------------ */
/* projection::match_frame_and_keyframe(cam_pose_cw, camera, frm_obs, orb_params, frm_landmarks, keyfrm,
 * already_matched_lms, margin, hamm_dist_thr) with check_orientation_ == false */
static unsigned int match_frame_and_keyframe_pose(sv_loop* L, const double pose_cm[16], const sv_tr_kf* cur, int* frm_lms,
                                                  const sv_tr_kf* cand, const unsigned int* already, unsigned int n_already,
                                                  float margin, unsigned int hamm_dist_thr) {
    const sv_tr_config* cfg = L->cfg;
    double rot_cw[9], trans_cw[3], t[3], cam_center[3];
    unsigned int num_matches = 0, idx;
    unsigned int* indices = (unsigned int*)malloc((cur->obs->num_kp ? cur->obs->num_kp : 1) * sizeof(unsigned int));
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot_cw[c * 3 + r] = pose_cm[c * 4 + r];
        }
    }
    trans_cw[0] = pose_cm[12];
    trans_cw[1] = pose_cm[13];
    trans_cw[2] = pose_cm[14];
    sv_mat3_mulv_lhs_transposed(rot_cw, trans_cw, t);
    cam_center[0] = -t[0];
    cam_center[1] = -t[1];
    cam_center[2] = -t[2];

    for (idx = 0; idx < cand->obs->num_kp; ++idx) {
        const sv_tr_lm* lm;
        double reproj[2], pos_w[3], cam_to_lm_vec[3], cam_to_lm_dist, max_d, min_d;
        float x_right;
        unsigned int pred_scale_level, n_idx, k;
        int min_level, max_level;
        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx = -1;
        if (cand->lm[idx] < 0) {
            continue;
        }
        lm = lm_get(L->mp, cand->lm[idx]);
        if (!lm) {
            continue;
        }
        if (idset_has(already, n_already, lm->id)) {
            continue;
        }
        pos_w[0] = lm->pos_w[0];
        pos_w[1] = lm->pos_w[1];
        pos_w[2] = lm->pos_w[2];
        if (!sv_tr_reproject_to_image(cfg, rot_cw, trans_cw, pos_w, reproj, &x_right)) {
            continue;
        }
        cam_to_lm_vec[0] = pos_w[0] - cam_center[0];
        cam_to_lm_vec[1] = pos_w[1] - cam_center[1];
        cam_to_lm_vec[2] = pos_w[2] - cam_center[2];
        cam_to_lm_dist = sv_vec3_norm(cam_to_lm_vec);
        max_d = 1.3 * (double)lm->max_valid_dist;
        min_d = (1.0 / 1.3) * (double)lm->min_valid_dist;
        if (cam_to_lm_dist < min_d || max_d < cam_to_lm_dist) {
            continue;
        }
        pred_scale_level = predict_scale_level(lm, (float)cam_to_lm_dist, (float)cfg->num_levels, cfg->log_scale_factor);
        min_level = (int)pred_scale_level - 1 > 0 ? (int)pred_scale_level - 1 : 0;
        max_level = (int)(cfg->num_levels - 1 < pred_scale_level + 1 ? cfg->num_levels - 1 : pred_scale_level + 1);
        n_idx = sv_frame_get_keypoints_in_cell(&cur->obs->grid, cur->obs->kp, (float)reproj[0], (float)reproj[1],
                                                margin * cfg->scale_factors[pred_scale_level], min_level, max_level,
                                                indices, cur->obs->num_kp);
        if (n_idx == 0) {
            continue;
        }
        for (k = 0; k < n_idx; ++k) {
            const unsigned int curr_idx = indices[k];
            unsigned int hamm_dist;
            if (frm_lms[curr_idx] >= 0) {
                continue;
            }
            hamm_dist = sv_tr_hamming(lm->desc, cur->obs->desc + (size_t)curr_idx * SV_TR_DESC_BYTES);
            if (hamm_dist < best_hamm_dist) {
                best_hamm_dist = hamm_dist;
                best_idx = (int)curr_idx;
            }
        }
        if (hamm_dist_thr < best_hamm_dist) {
            continue;
        }
        frm_lms[best_idx] = (int)lm->id;
        num_matches++;
    }
    free(indices);
    return num_matches;
}

/* projection::match_by_Sim3_transform(keyfrm, Sim3_cw (Mat44 with the scaled rotation), landmarks, matched, margin) */
static unsigned int match_by_sim3_transform(sv_loop* L, const sv_tr_kf* keyfrm, const double sim3_cm[16], const unsigned int* landmarks,
                                            unsigned int n_lms, int* matched, float margin) {
    const sv_tr_config* cfg = L->cfg;
    double s_rot[9], rot_cw[9], trans_cw[3], t[3], cam_center[3], s_cw;
    unsigned int num_matches = 0, li, i;
    unsigned int* already = (unsigned int*)malloc((keyfrm->obs->num_kp + 1) * sizeof(unsigned int));
    unsigned int n_already = 0;
    unsigned int* indices = (unsigned int*)malloc((keyfrm->obs->num_kp ? keyfrm->obs->num_kp : 1) * sizeof(unsigned int));
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            s_rot[c * 3 + r] = sim3_cm[c * 4 + r];
        }
    }
    {
        const double row0[3] = {s_rot[0], s_rot[3], s_rot[6]};
        s_cw = sqrt(dot_row_blocks(row0, row0));
    }
    for (i = 0; i < 9; ++i) {
        rot_cw[i] = s_rot[i] / s_cw;
    }
    for (i = 0; i < 3; ++i) {
        trans_cw[i] = sim3_cm[12 + i] / s_cw;
    }
    sv_mat3_mulv_lhs_transposed(rot_cw, trans_cw, t);
    cam_center[0] = -t[0];
    cam_center[1] = -t[1];
    cam_center[2] = -t[2];
    for (i = 0; i < keyfrm->obs->num_kp; ++i) {
        if (matched[i] >= 0) {
            already[n_already++] = (unsigned int)matched[i];
        }
    }
    n_already = idset_normalize(already, n_already);

    for (li = 0; li < n_lms; ++li) {
        const sv_tr_lm* lm = lm_get(L->mp, (int)landmarks[li]);
        double reproj[2], cam_to_lm_vec[3], cam_to_lm_dist, max_d, min_d;
        float x_right;
        unsigned int pred_scale_level, n_idx, k;
        int min_level, max_level;
        unsigned int best_dist = MAX_HAMMING_DIST;
        int best_idx = -1;
        if (!lm) { /* will_be_erased() */
            continue;
        }
        if (idset_has(already, n_already, lm->id)) {
            continue;
        }
        if (!sv_tr_reproject_to_image(cfg, rot_cw, trans_cw, lm->pos_w, reproj, &x_right)) {
            continue;
        }
        cam_to_lm_vec[0] = lm->pos_w[0] - cam_center[0];
        cam_to_lm_vec[1] = lm->pos_w[1] - cam_center[1];
        cam_to_lm_vec[2] = lm->pos_w[2] - cam_center[2];
        cam_to_lm_dist = sv_vec3_norm(cam_to_lm_vec);
        max_d = 1.3 * (double)lm->max_valid_dist;
        min_d = (1.0 / 1.3) * (double)lm->min_valid_dist;
        if (cam_to_lm_dist < min_d || max_d < cam_to_lm_dist) {
            continue;
        }
        if (sv_vec3_dot(cam_to_lm_vec, lm->mean_normal) < 0.5 * cam_to_lm_dist) {
            continue;
        }
        pred_scale_level = predict_scale_level(lm, (float)cam_to_lm_dist, (float)cfg->num_levels, cfg->log_scale_factor);
        min_level = (int)pred_scale_level - 1 > 0 ? (int)pred_scale_level - 1 : 0;
        max_level = (int)(cfg->num_levels - 1 < pred_scale_level + 1 ? cfg->num_levels - 1 : pred_scale_level + 1);
        n_idx = sv_frame_get_keypoints_in_cell(&keyfrm->obs->grid, keyfrm->obs->kp, (float)reproj[0], (float)reproj[1],
                                                margin * cfg->scale_factors[pred_scale_level], min_level, max_level,
                                                indices, keyfrm->obs->num_kp);
        if (n_idx == 0) {
            continue;
        }
        for (k = 0; k < n_idx; ++k) {
            const unsigned int idx = indices[k];
            unsigned int hamm_dist;
            if (matched[idx] >= 0) {
                continue;
            }
            hamm_dist = sv_tr_hamming(lm->desc, keyfrm->obs->desc + (size_t)idx * SV_TR_DESC_BYTES);
            if (hamm_dist < best_dist) {
                best_dist = hamm_dist;
                best_idx = (int)idx;
            }
        }
        if (HAMMING_DIST_THR_LOW < best_dist) {
            continue;
        }
        matched[best_idx] = (int)lm->id;
        ++num_matches;
    }
    free(already);
    free(indices);
    return num_matches;
}

/* projection::match_keyframes_mutually(keyfrm_1, keyfrm_2, matched_lms_in_keyfrm_1, s_12, rot_12, trans_12, margin) */
static unsigned int match_keyframes_mutually(sv_loop* L, const sv_tr_kf* k1, const sv_tr_kf* k2, int* matched1, float s_12,
                                             const double rot_12[9], const double trans_12[3], float margin) {
    const sv_tr_config* cfg = L->cfg;
    double rot_1w[9], trans_1w[3], rot_2w[9], trans_2w[3];
    double s_rot_12[9], s_rot_21[9], rot_12_t[9], trans_21[3], tmp3[3];
    const unsigned int n1 = k1->obs->num_kp, n2 = k2->obs->num_kp;
    unsigned char* already1 = (unsigned char*)calloc(n1 ? n1 : 1, 1);
    unsigned char* already2 = (unsigned char*)calloc(n2 ? n2 : 1, 1);
    int* m21 = (int*)malloc((n1 ? n1 : 1) * sizeof(int));
    int* m12 = (int*)malloc((n2 ? n2 : 1) * sizeof(int));
    unsigned int* indices = (unsigned int*)malloc(((n1 > n2 ? n1 : n2) + 1) * sizeof(unsigned int));
    unsigned int idx_1, idx_2, num_matches = 0, i;
    const double sd = (double)s_12;

    kf_rot_trans(k1, rot_1w, trans_1w);
    kf_rot_trans(k2, rot_2w, trans_2w);
    for (i = 0; i < 9; ++i) {
        s_rot_12[i] = sd * rot_12[i];
    }
    sv_mat3_transpose(rot_12, rot_12_t);
    for (i = 0; i < 9; ++i) {
        s_rot_21[i] = (1.0 / sd) * rot_12_t[i];
    }
    sv_mat3_mulv(s_rot_21, trans_12, tmp3);
    trans_21[0] = -tmp3[0];
    trans_21[1] = -tmp3[1];
    trans_21[2] = -tmp3[2];
    for (i = 0; i < n1; ++i) {
        m21[i] = -1;
    }
    for (i = 0; i < n2; ++i) {
        m12[i] = -1;
    }
    for (idx_1 = 0; idx_1 < n1; ++idx_1) {
        const sv_tr_lm* lm;
        int found, pos;
        if (matched1[idx_1] < 0) {
            continue;
        }
        lm = sv_tr_map_lm(L->map, matched1[idx_1]);
        if (!lm) {
            continue;
        }
        pos = lm_obs_pos(lm, k2->id, &found);
        if (found && (int)lm->obs_idx[pos] < (int)n2) {
            already1[idx_1] = 1;
            already2[lm->obs_idx[pos]] = 1;
        }
    }
    {
        double s_rot_21w[9], trans_21w[3], t2[3];
        sv_mat3_mul(s_rot_21, rot_1w, s_rot_21w);
        sv_mat3_mulv(s_rot_21, trans_1w, t2);
        for (i = 0; i < 3; ++i) {
            trans_21w[i] = t2[i] + trans_21[i];
        }
        for (idx_1 = 0; idx_1 < n1; ++idx_1) {
            const sv_tr_lm* lm;
            double pos_2[3], tt[3], reproj[2], cam_to_lm_dist, max_d, min_d;
            float x_right;
            unsigned int pred_scale_level, n_idx, k;
            int min_level, max_level;
            unsigned int best_hamm_dist = MAX_HAMMING_DIST;
            int best_idx_2 = -1;
            if (k1->lm[idx_1] < 0) {
                continue;
            }
            lm = lm_get(L->mp, k1->lm[idx_1]);
            if (!lm) {
                continue;
            }
            if (already1[idx_1]) {
                continue;
            }
            sv_mat3_mulv(s_rot_21w, lm->pos_w, tt);
            pos_2[0] = tt[0] + trans_21w[0];
            pos_2[1] = tt[1] + trans_21w[1];
            pos_2[2] = tt[2] + trans_21w[2];
            if (!sv_tr_reproject_to_image(cfg, s_rot_21w, trans_21w, lm->pos_w, reproj, &x_right)) {
                continue;
            }
            cam_to_lm_dist = sv_vec3_norm(pos_2);
            max_d = 1.3 * (double)lm->max_valid_dist;
            min_d = (1.0 / 1.3) * (double)lm->min_valid_dist;
            if (cam_to_lm_dist < min_d || max_d < cam_to_lm_dist) {
                continue;
            }
            pred_scale_level = predict_scale_level(lm, (float)cam_to_lm_dist, (float)cfg->num_levels, cfg->log_scale_factor);
            min_level = (int)pred_scale_level - 1 > 0 ? (int)pred_scale_level - 1 : 0;
            max_level = (int)(cfg->num_levels - 1 < pred_scale_level + 1 ? cfg->num_levels - 1 : pred_scale_level + 1);
            n_idx = sv_frame_get_keypoints_in_cell(&k2->obs->grid, k2->obs->kp, (float)reproj[0], (float)reproj[1],
                                                    margin * cfg->scale_factors[pred_scale_level], min_level, max_level,
                                                    indices, k2->obs->num_kp);
            if (n_idx == 0) {
                continue;
            }
            for (k = 0; k < n_idx; ++k) {
                const unsigned int j = indices[k];
                const unsigned int hamm_dist = sv_tr_hamming(lm->desc, k2->obs->desc + (size_t)j * SV_TR_DESC_BYTES);
                if (hamm_dist < best_hamm_dist) {
                    best_hamm_dist = hamm_dist;
                    best_idx_2 = (int)j;
                }
            }
            if (best_hamm_dist <= HAMMING_DIST_THR_HIGH) {
                m21[idx_1] = best_idx_2;
            }
        }
    }
    {
        double s_rot_12w[9], trans_12w[3], t2[3];
        sv_mat3_mul(s_rot_12, rot_2w, s_rot_12w);
        sv_mat3_mulv(s_rot_12, trans_2w, t2);
        for (i = 0; i < 3; ++i) {
            trans_12w[i] = t2[i] + trans_12[i];
        }
        for (idx_2 = 0; idx_2 < n2; ++idx_2) {
            const sv_tr_lm* lm;
            double pos_1[3], tt[3], reproj[2], cam_to_lm_dist, max_d, min_d;
            float x_right;
            unsigned int pred_scale_level, n_idx, k;
            int min_level, max_level;
            unsigned int best_hamm_dist = MAX_HAMMING_DIST;
            int best_idx_1 = -1;
            if (k2->lm[idx_2] < 0) {
                continue;
            }
            lm = lm_get(L->mp, k2->lm[idx_2]);
            if (!lm) {
                continue;
            }
            if (already2[idx_2]) {
                continue;
            }
            sv_mat3_mulv(s_rot_12w, lm->pos_w, tt);
            pos_1[0] = tt[0] + trans_12w[0];
            pos_1[1] = tt[1] + trans_12w[1];
            pos_1[2] = tt[2] + trans_12w[2];
            if (!sv_tr_reproject_to_image(cfg, s_rot_12w, trans_12w, lm->pos_w, reproj, &x_right)) {
                continue;
            }
            cam_to_lm_dist = sv_vec3_norm(pos_1);
            max_d = 1.3 * (double)lm->max_valid_dist;
            min_d = (1.0 / 1.3) * (double)lm->min_valid_dist;
            if (cam_to_lm_dist < min_d || max_d < cam_to_lm_dist) {
                continue;
            }
            pred_scale_level = predict_scale_level(lm, (float)cam_to_lm_dist, (float)cfg->num_levels, cfg->log_scale_factor);
            min_level = (int)pred_scale_level - 1 > 0 ? (int)pred_scale_level - 1 : 0;
            max_level = (int)(cfg->num_levels - 1 < pred_scale_level + 1 ? cfg->num_levels - 1 : pred_scale_level + 1);
            n_idx = sv_frame_get_keypoints_in_cell(&k1->obs->grid, k1->obs->kp, (float)reproj[0], (float)reproj[1],
                                                    margin * cfg->scale_factors[pred_scale_level], min_level, max_level,
                                                    indices, k1->obs->num_kp);
            if (n_idx == 0) {
                continue;
            }
            for (k = 0; k < n_idx; ++k) {
                const unsigned int j = indices[k];
                const unsigned int hamm_dist = sv_tr_hamming(lm->desc, k1->obs->desc + (size_t)j * SV_TR_DESC_BYTES);
                if (hamm_dist < best_hamm_dist) {
                    best_hamm_dist = hamm_dist;
                    best_idx_1 = (int)j;
                }
            }
            if (best_hamm_dist <= HAMMING_DIST_THR_HIGH) {
                m12[idx_2] = best_idx_1;
            }
        }
    }
    for (i = 0; i < n1; ++i) {
        const int j2 = m21[i];
        int j1;
        if (j2 < 0) {
            continue;
        }
        j1 = m12[j2];
        if (j1 == (int)i) {
            matched1[j1] = k2->lm[j2] >= 0 ? k2->lm[j2] : -1;
            ++num_matches;
        }
    }
    free(already1);
    free(already2);
    free(m21);
    free(m12);
    free(indices);
    return num_matches;
}

/* ------------------------------------------------------------------ */
/* transform_optimizer::optimize glue                                 */
/* ------------------------------------------------------------------ */
static unsigned int transform_optimize(sv_loop* L, const sv_tr_kf* k1, const sv_tr_kf* k2, int* matched, sv_sim3* sim3_12, float chi_sq) {
    const sv_tr_config* cfg = L->cfg;
    const unsigned int n1 = k1->obs->num_kp;
    sv_transform_match* ms = (sv_transform_match*)malloc((n1 ? n1 : 1) * sizeof(sv_transform_match));
    unsigned char* rej = (unsigned char*)calloc(n1 ? n1 : 1, 1);
    unsigned int n = 0, idx1, i, ret;
    int* triples = (int*)malloc((n1 ? n1 : 1) * 3 * sizeof(int));
    double rot_1w[9], trans_1w[3], rot_2w[9], trans_2w[3];
    sv_transform_camera cam;
    sv_sim3 mid;
    kf_rot_trans(k1, rot_1w, trans_1w);
    kf_rot_trans(k2, rot_2w, trans_2w);
    cam.fx = cfg->fx;
    cam.fy = cfg->fy;
    cam.cx = cfg->cx;
    cam.cy = cfg->cy;
    {
        TB_INIT(b);
        tb_u(&b, k1->id);
        tb_u(&b, k2->id);
        tb_sim3(&b, sim3_12);
        tb_d(&b, (double)chi_sq);
        tb_emit(L, "TIN", &b);
    }
    for (idx1 = 0; idx1 < n1; ++idx1) {
        const sv_tr_lm *lm_1, *lm_2;
        int found, pos;
        unsigned int idx2;
        sv_transform_match* m;
        if (matched[idx1] < 0) {
            continue;
        }
        lm_1 = k1->lm[idx1] >= 0 ? lm_get(L->mp, k1->lm[idx1]) : NULL;
        lm_2 = lm_get(L->mp, matched[idx1]);
        if (!lm_1 || !lm_2) {
            continue;
        }
        pos = lm_obs_pos(lm_2, k2->id, &found);
        if (!found) { /* idx2 < 0 */
            continue;
        }
        idx2 = lm_2->obs_idx[pos];
        m = &ms[n];
        m->idx1 = idx1;
        m->obs1[0] = (double)k1->obs->kp[idx1].x;
        m->obs1[1] = (double)k1->obs->kp[idx1].y;
        m->info1 = (double)cfg->inv_level_sigma_sq[k1->obs->kp[idx1].octave];
        m->obs2[0] = (double)k2->obs->kp[idx2].x;
        m->obs2[1] = (double)k2->obs->kp[idx2].y;
        m->info2 = (double)cfg->inv_level_sigma_sq[k2->obs->kp[idx2].octave];
        memcpy(m->pos_w_2, lm_2->pos_w, sizeof(double) * 3);
        memcpy(m->pos_w_1, lm_1->pos_w, sizeof(double) * 3);
        triples[3 * n + 0] = (int)idx1;
        triples[3 * n + 1] = (int)lm_1->id;
        triples[3 * n + 2] = (int)lm_2->id;
        ++n;
    }
    {
        /* TEDG is emitted before the first optimization; the early exit of the reference happens after TMID */
        TB_INIT(b);
        tb_u(&b, n);
        tb_triples(&b, triples, n);
        tb_emit(L, "TEDG", &b);
    }
    ret = sv_transform_optimize(&cam, &cam, rot_1w, trans_1w, rot_2w, trans_2w, ms, n, rej, sim3_12, chi_sq, 0, 10, &mid);
    {
        int* outl = (int*)malloc((n + 1) * sizeof(int));
        unsigned int no = 0;
        for (i = 0; i < n; ++i) {
            if (rej[i] == 1) {
                outl[no++] = (int)ms[i].idx1;
            }
        }
        TB_INIT(b);
        tb_sim3(&b, &mid);
        tb_ids(&b, outl, no);
        tb_emit(L, "TMID", &b);
        free(outl);
    }
    for (i = 0; i < n; ++i) {
        if (rej[i]) {
            matched[ms[i].idx1] = -1;
        }
    }
    free(ms);
    free(rej);
    free(triples);
    return ret;
}

/* ------------------------------------------------------------------ */
/* loop_detector::select_loop_candidate_via_Sim3                      */
/* ------------------------------------------------------------------ */
static int cmp_float(const void* a, const void* b) {
    const float x = *(const float*)a, y = *(const float*)b;
    return (x > y) - (x < y);
}

static void emit_cx(sv_loop* L, int code) {
    TB_INIT(b);
    tb_i(&b, code);
    tb_emit(L, "CX", &b);
}

static int select_loop_candidate(sv_loop* L, sv_tr_kf* cur) {
    sv_mapping* mp = L->mp;
    const sv_tr_config* cfg = L->cfg;
    const unsigned int nkp = cur->obs->num_kp;
    unsigned int ci;
    int* cm = (int*)malloc((nkp ? nkp : 1) * sizeof(int));
    unsigned char* outl = (unsigned char*)malloc(nkp ? nkp : 1);
    unsigned int* valid = (unsigned int*)malloc((nkp ? nkp : 1) * sizeof(unsigned int));
    uint64_t* toks_c = (uint64_t*)malloc((nkp ? nkp : 1) * sizeof(uint64_t));
    int ret = 0;

    for (ci = 0; ci < L->n_to_validate; ++ci) {
        const unsigned int cand_id = L->to_validate[ci];
        sv_tr_kf* cand = kf_get(mp, (int)cand_id);
        unsigned int i, num_matches = 0, n_valid = 0;
        sv_match_bow_view kv, cv;
        uint64_t* tok_cur;
        uint64_t* matched_tok;
        sv_loop_pnp pnp;
        double pose1[16], pose2[16], pose3[16];
        unsigned int nv1, nv2, nv3;
        {
            TB_INIT(b);
            tb_u(&b, cand_id);
            tb_u(&b, cand ? 0 : 1);
            tb_u(&b, L->thr_opt1);
            tb_u(&b, L->thr_a);
            tb_u(&b, L->thr_b);
            tb_emit(L, "CAND", &b);
        }
        if (!cand) { /* candidate->will_be_erased() */
            continue;
        }
        /* bow_matcher.match_keyframes(cur, candidate, matches) */
        if (sv_tr_obs_ensure_bow(cur->obs, cfg) != 0 || sv_tr_obs_ensure_bow(cand->obs, cfg) != 0) {
            continue;
        }
        tok_cur = (uint64_t*)calloc(nkp ? nkp : 1, sizeof(uint64_t));
        matched_tok = (uint64_t*)calloc(nkp ? nkp : 1, sizeof(uint64_t));
        for (i = 0; i < nkp; ++i) {
            tok_cur[i] = (cur->lm[i] >= 0 && lm_get(mp, cur->lm[i])) ? (uint64_t)cur->lm[i] + 1u : 0u;
        }
        for (i = 0; i < cand->obs->num_kp; ++i) {
            toks_c = (uint64_t*)realloc(toks_c, (cand->obs->num_kp + 1) * sizeof(uint64_t));
            toks_c[i] = (cand->lm[i] >= 0 && lm_get(mp, cand->lm[i])) ? (uint64_t)cand->lm[i] + 1u : 0u;
        }
        memset(&kv, 0, sizeof(kv));
        memset(&cv, 0, sizeof(cv));
        kv.count = nkp;
        kv.keypoints = cur->obs->kp;
        kv.descriptors = cur->obs->desc;
        kv.features = &cur->obs->bow_feat;
        kv.landmarks = tok_cur;
        cv.count = cand->obs->num_kp;
        cv.keypoints = cand->obs->kp;
        cv.descriptors = cand->obs->desc;
        cv.features = &cand->obs->bow_feat;
        cv.landmarks = toks_c;
        {
            uint32_t cnt = 0;
            sv_match_bow_keyframes(&kv, &cv, 0.75f, 0, matched_tok, &cnt);
            num_matches = cnt;
        }
        for (i = 0; i < nkp; ++i) {
            cm[i] = matched_tok[i] ? (int)(matched_tok[i] - 1u) : -1;
        }
        free(tok_cur);
        free(matched_tok);
        {
            TB_INIT(b);
            tb_u(&b, num_matches);
            tb_assoc(&b, cm, nkp);
            tb_emit(L, "BOW", &b);
        }
        if (num_matches < L->num_matches_thr) {
            emit_cx(L, 1);
            continue;
        }
        /* valid indices (non-null, not erased) */
        for (i = 0; i < nkp; ++i) {
            if (cm[i] >= 0 && lm_get(mp, cm[i])) {
                valid[n_valid++] = i;
            }
        }
        /* PnP RANSAC (injected) */
        memset(&pnp, 0, sizeof(pnp));
        if (L->pnp_ransac) {
            double* pb = (double*)malloc((n_valid + 1) * 3 * sizeof(double));
            double* pp = (double*)malloc((n_valid + 1) * 3 * sizeof(double));
            int* po = (int*)malloc((n_valid + 1) * sizeof(int));
            sv_tr_obs_ensure_bearings(cur->obs, cfg);
            for (i = 0; i < n_valid; ++i) {
                const sv_tr_lm* vlm = lm_get(mp, cm[valid[i]]);
                memcpy(pb + 3 * i, cur->obs->bearings + 3 * (size_t)valid[i], 3 * sizeof(double));
                memcpy(pp + 3 * i, vlm->pos_w, 3 * sizeof(double));
                po[i] = cur->obs->kp[valid[i]].octave;
            }
            if (L->pnp_ransac(L->pnp_ransac_user, pb, pp, po, n_valid, &pnp) != 0) {
                pnp.valid = 0;
            }
            free(pb);
            free(pp);
            free(po);
        }
        else if (!L->pnp || L->pnp(L->pnp_user, cand_id, n_valid, &pnp) != 0) {
            pnp.valid = 0;
        }
        {
            TB_INIT(b);
            int* in = NULL;
            tb_u(&b, pnp.valid ? 1 : 0);
            tb_u(&b, n_valid);
            if (pnp.valid) {
                unsigned int k;
                in = (int*)malloc((pnp.n_inliers + 1) * sizeof(int));
                for (k = 0; k < pnp.n_inliers; ++k) {
                    in[k] = (int)pnp.inliers[k];
                }
                tb_mat44_rm(&b, pnp.pose_rm);
                tb_ids(&b, in, pnp.n_inliers);
            }
            tb_emit(L, "PNP", &b);
            free(in);
        }
        if (!pnp.valid) {
            emit_cx(L, 3);
            continue;
        }
        {
            /* Set 2D-3D matches for the pose optimization: only the inliers of the solver */
            int* lms_in_cand = (int*)malloc((nkp ? nkp : 1) * sizeof(int));
            unsigned int* inlier_idx = (unsigned int*)malloc((pnp.n_inliers + 1) * sizeof(unsigned int));
            unsigned int k, n_already = 0, np_found;
            unsigned int* already = (unsigned int*)malloc((nkp + cur->obs->num_kp + 1) * sizeof(unsigned int));
            unsigned char *o1, *o2;
            double init_pose[16], nn = 0.0;
            (void)nn;
            for (i = 0; i < nkp; ++i) {
                lms_in_cand[i] = -1;
            }
            for (k = 0; k < pnp.n_inliers; ++k) {
                inlier_idx[k] = valid[pnp.inliers[k]];
                lms_in_cand[inlier_idx[k]] = cm[inlier_idx[k]];
            }
            memcpy(cm, lms_in_cand, nkp * sizeof(int));
            for (i = 0; i < 16; ++i) { /* row-major Mat44 -> column-major */
                init_pose[(i % 4) * 4 + (i / 4)] = pnp.pose_rm[i];
            }
            memset(pose1, 0, sizeof(pose1));
            memset(pose2, 0, sizeof(pose2));
            memset(pose3, 0, sizeof(pose3));
            /* Pose optimization 1 */
            nv1 = pose_optimize(L, cur, init_pose, cm, pose1, outl);
            emit_opt(L, "OPT1", nv1, pose1, outl, nkp);
            if ((int)nv1 < (int)L->thr_opt1) {
                emit_cx(L, 4);
                free(lms_in_cand);
                free(inlier_idx);
                free(already);
                continue;
            }
            /* already found landmarks: inliers that are not outliers of the pose optimization
             * (`lms_in_cand.at(idx) = nullptr` for outliers only touches the local copy) */
            for (k = 0; k < pnp.n_inliers; ++k) {
                if (outl[inlier_idx[k]]) {
                    continue;
                }
                already[n_already++] = (unsigned int)cm[inlier_idx[k]];
            }
            n_already = idset_normalize(already, n_already);
            /* Projection match based on the pre-optimized camera pose */
            np_found = match_frame_and_keyframe_pose(L, pose1, cur, cm, cand, already, n_already, 10.0f, 100);
            {
                TB_INIT(b);
                tb_u(&b, np_found);
                tb_u(&b, n_already);
                tb_assoc(&b, cm, nkp);
                tb_emit(L, "PRJ1", &b);
            }
            if (n_already + np_found < L->thr_a) {
                emit_cx(L, 5);
                free(lms_in_cand);
                free(inlier_idx);
                free(already);
                continue;
            }
            o1 = (unsigned char*)malloc(nkp ? nkp : 1);
            nv2 = pose_optimize(L, cur, pose1, cm, pose2, o1);
            emit_opt(L, "OPT2", nv2, pose2, o1, nkp);
            if (nv2 < L->thr_a) {
                emit_cx(L, 6);
                free(lms_in_cand);
                free(inlier_idx);
                free(already);
                free(o1);
                continue;
            }
            {
                unsigned int num_additional = match_frame_and_keyframe_pose(L, pose2, cur, cm, cand, already, n_already, 3.0f, 64);
                TB_INIT(b);
                tb_u(&b, num_additional);
                tb_assoc(&b, cm, nkp);
                tb_emit(L, "PRJ2", &b);
                if (nv2 + num_additional < L->thr_b) {
                    emit_cx(L, 7);
                    free(lms_in_cand);
                    free(inlier_idx);
                    free(already);
                    free(o1);
                    ret = 0;
                    goto done; /* `return false` */
                }
            }
            o2 = (unsigned char*)malloc(nkp ? nkp : 1);
            nv3 = pose_optimize(L, cur, pose2, cm, pose3, o2);
            emit_opt(L, "OPT3", nv3, pose3, o2, nkp);
            if (nv3 < L->thr_b) {
                emit_cx(L, 8);
                free(lms_in_cand);
                free(inlier_idx);
                free(already);
                free(o1);
                free(o2);
                ret = 0;
                goto done; /* `return false` */
            }
            for (i = 0; i < nkp; ++i) {
                if (o2[i]) {
                    cm[i] = -1;
                }
            }
            free(lms_in_cand);
            free(inlier_idx);
            free(already);
            free(o1);
            free(o2);
        }
        {
            /* scale references */
            double rot_1w_c[9], trans_1w_c[3], rot_cur[9], trans_cur[3], rot_cand[9], trans_cand[3], rot_cand_t[9];
            double rot_12[9], trans_12[3], tmp3[3];
            float* scales = (float*)malloc((nkp ? nkp : 1) * sizeof(float));
            unsigned int n_scales = 0, idx;
            float scale_12;
            unsigned int num_optimized_inliers;
            sv_sim3 sim3_12, sim3_cand;
            int r, c2;
            for (c2 = 0; c2 < 3; ++c2) {
                for (r = 0; r < 3; ++r) {
                    rot_1w_c[c2 * 3 + r] = pose3[c2 * 4 + r];
                }
            }
            trans_1w_c[0] = pose3[12];
            trans_1w_c[1] = pose3[13];
            trans_1w_c[2] = pose3[14];
            kf_rot_trans(cur, rot_cur, trans_cur);
            kf_rot_trans(cand, rot_cand, trans_cand);
            for (idx = 0; idx < nkp; ++idx) {
                const sv_tr_lm *lm_curr, *lm_cand;
                double t1[3], pos_1_in_cand[3], pos_1_in_curr[3];
                float norm_cand, norm_curr, cos_parallax;
                const float cos_parallax_thr = 0.99996192306f;
                if (cur->lm[idx] < 0 || cm[idx] < 0) {
                    continue;
                }
                lm_curr = lm_get(mp, cur->lm[idx]);
                lm_cand = lm_get(mp, cm[idx]);
                if (!lm_cand || !lm_curr) {
                    continue;
                }
                sv_mat3_mulv(rot_1w_c, lm_cand->pos_w, t1);
                pos_1_in_cand[0] = t1[0] + trans_1w_c[0];
                pos_1_in_cand[1] = t1[1] + trans_1w_c[1];
                pos_1_in_cand[2] = t1[2] + trans_1w_c[2];
                sv_mat3_mulv(rot_cur, lm_curr->pos_w, t1);
                pos_1_in_curr[0] = t1[0] + trans_cur[0];
                pos_1_in_curr[1] = t1[1] + trans_cur[1];
                pos_1_in_curr[2] = t1[2] + trans_cur[2];
                norm_cand = (float)sv_vec3_norm(pos_1_in_cand);
                norm_curr = (float)sv_vec3_norm(pos_1_in_curr);
                cos_parallax = (float)(sv_vec3_dot(pos_1_in_cand, pos_1_in_curr) / (double)(norm_cand * norm_curr));
                if (!(cos_parallax_thr < cos_parallax)) {
                    continue;
                }
                scales[n_scales++] = norm_curr / norm_cand;
            }
            if (n_scales < 1) {
                emit_cx(L, 9);
                free(scales);
                continue;
            }
            sv_mat3_transpose(rot_cand, rot_cand_t);
            sv_mat3_mul(rot_1w_c, rot_cand_t, rot_12);
            sv_mat3_mulv(rot_12, trans_cand, tmp3);
            trans_12[0] = -tmp3[0] + trans_1w_c[0];
            trans_12[1] = -tmp3[1] + trans_1w_c[1];
            trans_12[2] = -tmp3[2] + trans_1w_c[2];
            qsort(scales, n_scales, sizeof(float), cmp_float);
            scale_12 = scales[(n_scales - 1) / 2];
            free(scales);
            {
                TB_INIT(b);
                tb_u(&b, n_scales);
                tb_d(&b, (double)scale_12);
                tb_mat33_cm(&b, rot_12);
                tb_vec3(&b, trans_12);
                tb_emit(L, "SCL", &b);
            }
            {
                const unsigned int num_mutual = match_keyframes_mutually(L, cur, cand, cm, scale_12, rot_12, trans_12, 7.5f);
                TB_INIT(b);
                tb_u(&b, num_mutual);
                tb_assoc(&b, cm, nkp);
                tb_emit(L, "MUT", &b);
            }
            sv_sim3_from_rot(rot_12, trans_12, (double)scale_12, &sim3_12);
            num_optimized_inliers = transform_optimize(L, cur, cand, cm, &sim3_12, 10.0f);
            {
                TB_INIT(b);
                tb_u(&b, num_optimized_inliers);
                tb_sim3(&b, &sim3_12);
                tb_assoc(&b, cm, nkp);
                tb_emit(L, "TRF", &b);
            }
            if (num_optimized_inliers < L->num_optimized_inliers_thr) {
                emit_cx(L, 10);
                continue;
            }
            L->selected = (int)cand_id;
            sv_sim3_from_rot(rot_cand, trans_cand, 1.0, &sim3_cand);
            sv_sim3_mul(&sim3_12, &sim3_cand, &L->sim3_world_to_curr);
            {
                TB_INIT(b);
                tb_u(&b, cand_id);
                tb_sim3(&b, &L->sim3_world_to_curr);
                tb_emit(L, "ACC", &b);
            }
            free(L->match_cand);
            L->match_cand = (int*)malloc((nkp ? nkp : 1) * sizeof(int));
            memcpy(L->match_cand, cm, nkp * sizeof(int));
            L->n_match_cand = nkp;
            ret = 1;
            goto done;
        }
    }
done:
    free(cm);
    free(outl);
    free(valid);
    free(toks_c);
    return ret;
}

/* ------------------------------------------------------------------ */
/* loop_detector::validate_candidates                                 */
/* ------------------------------------------------------------------ */
int sv_loop_validate(sv_loop* L, unsigned int cur_id) {
    sv_mapping* mp = L->mp;
    sv_tr_kf* cur = kf_get(mp, (int)cur_id);
    sv_tr_kf* sel;
    unsigned int i, j, n_final = 0, nkp;
    unsigned int* covis_lms;
    unsigned int n_covis_lms = 0;
    unsigned char* seen;
    int found;
    L->selected = -1;
    if (!cur) {
        return 0;
    }
    nkp = cur->obs->num_kp;
    found = select_loop_candidate(L, cur);
    if (!found) {
        TB_INIT(b);
        tb_emit(L, "VNONE", &b);
        return 0;
    }
    sel = kf_get(mp, L->selected);
    /* 2. reproject the landmarks observed in the covisibilities of the selected candidate */
    {
        unsigned int* cov = (unsigned int*)malloc((sel->n_covis + 2) * sizeof(unsigned int));
        unsigned int ncov = covisibilities(L, sel, cov);
        cov[ncov++] = sel->id; /* cand_covisibilities.push_back(selected_candidate_) */
        seen = (unsigned char*)calloc(L->map->lm_cap ? L->map->lm_cap : 1, 1);
        covis_lms = (unsigned int*)malloc((1 + 1) * sizeof(unsigned int));
        {
            unsigned int cap = 1;
            for (i = 0; i < ncov; ++i) {
                const sv_tr_kf* k = kf_get(mp, (int)cov[i]);
                if (!k) {
                    continue;
                }
                for (j = 0; j < k->obs->num_kp; ++j) {
                    const sv_tr_lm* lm;
                    if (k->lm[j] < 0) {
                        continue;
                    }
                    lm = lm_get(mp, k->lm[j]);
                    if (!lm) {
                        continue;
                    }
                    if (seen[lm->id]) {
                        continue;
                    }
                    if (n_covis_lms == cap) {
                        cap *= 2;
                        covis_lms = (unsigned int*)realloc(covis_lms, cap * sizeof(unsigned int));
                    }
                    covis_lms[n_covis_lms++] = lm->id;
                    seen[lm->id] = 1;
                }
            }
        }
        free(cov);
        free(seen);
    }
    free(L->match_covis);
    L->match_covis = covis_lms;
    L->n_match_covis = n_covis_lms;
    {
        double m44[16];
        sim3_to_mat44(&L->sim3_world_to_curr, m44);
        match_by_sim3_transform(L, cur, m44, covis_lms, n_covis_lms, L->match_cand, 10.0f);
    }
    for (i = 0; i < nkp; ++i) {
        if (L->match_cand[i] >= 0) {
            ++n_final;
        }
    }
    {
        int* cl = (int*)malloc((n_covis_lms + 1) * sizeof(int));
        for (i = 0; i < n_covis_lms; ++i) {
            cl[i] = (int)covis_lms[i];
        }
        {
            TB_INIT(b);
            tb_ids(&b, cl, n_covis_lms);
            tb_emit(L, "VCOVIS", &b);
        }
        {
            TB_INIT(b);
            tb_u(&b, n_final);
            tb_assoc(&b, L->match_cand, nkp);
            tb_emit(L, "VFINAL", &b);
        }
        free(cl);
    }
    return L->num_final_matches_thr <= n_final;
}

/* ------------------------------------------------------------------ */
/* global_optimization_module::correct_loop                           */
/* ------------------------------------------------------------------ */
typedef struct id_sim3 {
    unsigned int id;
    sv_sim3 s;
} id_sim3;

static const id_sim3* find_sim3(const id_sim3* a, unsigned int n, unsigned int id) {
    unsigned int lo = 0, hi = n;
    while (lo < hi) {
        const unsigned int mid = lo + (hi - lo) / 2;
        if (a[mid].id < id) {
            lo = mid + 1;
        }
        else {
            hi = mid;
        }
    }
    return (lo < n && a[lo].id == id) ? &a[lo] : NULL;
}

/* graph_node::get_keyframes_from_root(): BFS over the id-ordered spanning children */
static unsigned int keyframes_from_root(sv_loop* L, unsigned int start_id, unsigned int* out) {
    const sv_tr_kf* k = kf_get(L->mp, (int)start_id);
    unsigned int head = 0, n = 0, i;
    while (k && !k->is_root && k->parent >= 0) {
        k = kf_get(L->mp, k->parent);
    }
    if (!k) {
        return 0;
    }
    out[n++] = k->id;
    while (head < n) {
        const sv_tr_kf* p = kf_get(L->mp, (int)out[head++]);
        for (i = 0; i < p->n_children; ++i) {
            out[n++] = p->children[i];
        }
    }
    return n;
}

static void mat44_mul_cm(const double a[16], const double b[16], double out[16]) {
    sv_mat4_mul(a, b, out);
}

typedef struct conn_edge_set {
    unsigned int kf;
    unsigned int* ids;
    unsigned int n;
} conn_edge_set;

/* global_optimization_module::extract_new_connections */
static conn_edge_set* extract_new_connections(sv_loop* L, const unsigned int* cov, unsigned int n_cov) {
    conn_edge_set* out = (conn_edge_set*)calloc(n_cov + 1, sizeof(conn_edge_set));
    unsigned int i, j, k;
    for (i = 0; i < n_cov; ++i) {
        sv_tr_kf* kf = kf_get(L->mp, (int)cov[i]);
        unsigned int* before = (unsigned int*)malloc((kf->n_covis + 1) * sizeof(unsigned int));
        unsigned int nb = covisibilities(L, kf, before);
        unsigned int nset;
        unsigned int* set;
        unsigned int nn = 0;
        update_connections(L->mp, kf);
        set = connected_set(L, kf->id, &nset);
        out[i].kf = kf->id;
        out[i].ids = (unsigned int*)malloc((nset + 1) * sizeof(unsigned int));
        for (j = 0; j < nset; ++j) {
            int drop = 0;
            for (k = 0; k < n_cov && !drop; ++k) {
                drop = (cov[k] == set[j]);
            }
            for (k = 0; k < nb && !drop; ++k) {
                drop = (before[k] == set[j]);
            }
            if (!drop) {
                out[i].ids[nn++] = set[j];
            }
        }
        out[i].n = nn;
        free(set);
        free(before);
    }
    /* the outer map is id-ordered */
    for (i = 1; i < n_cov; ++i) {
        conn_edge_set t = out[i];
        for (j = i; j > 0 && out[j - 1].kf > t.kf; --j) {
            out[j] = out[j - 1];
        }
        out[j] = t;
    }
    return out;
}

typedef struct pair_u {
    unsigned int a, b;
} pair_u;

/* optimize::graph_optimizer::optimize. `non_corrected` / `pre_corrected`: id-ordered Sim3 maps. */
static void graph_optimize(sv_loop* L, unsigned int loop_id, unsigned int cur_id, const id_sim3* non_corr, unsigned int n_non,
                           const id_sim3* pre_corr, unsigned int n_pre, const conn_edge_set* loop_conn, unsigned int n_conn,
                           const unsigned int* found_ids, const unsigned int* found_ref, unsigned int n_found) {
    sv_mapping* mp = L->mp;
    sv_tr_map* map = L->map;
    unsigned int* all = (unsigned int*)malloc((map->kf_cap + 1) * sizeof(unsigned int));
    unsigned int n_all = keyframes_from_root(L, cur_id, all), i, k;
    id_sim3* sim3_cw = (id_sim3*)calloc(map->kf_cap + 1, sizeof(id_sim3));
    unsigned char* has_cw = (unsigned char*)calloc(map->kf_cap + 1, 1);
    int* vidx = (int*)malloc((map->kf_cap + 1) * sizeof(int));
    unsigned int* order = (unsigned int*)malloc((n_all + 1) * sizeof(unsigned int));
    unsigned int n_order = 0;
    sv_s3_graph g;
    pair_u* inserted = (pair_u*)malloc((n_all * 32 + 64) * sizeof(pair_u));
    unsigned int n_ins = 0;
    unsigned int* all_lms = (unsigned int*)malloc((map->lm_cap + 1) * sizeof(unsigned int));
    unsigned char* lm_seen = (unsigned char*)calloc(map->lm_cap + 1, 1);
    unsigned int n_lms = 0;
    id_sim3* corr_wc = (id_sim3*)calloc(map->kf_cap + 1, sizeof(id_sim3));
    unsigned char* has_wc = (unsigned char*)calloc(map->kf_cap + 1, 1);
    const unsigned int min_shared = L->min_num_shared_lms_graph;

    for (i = 0; i < map->kf_cap; ++i) {
        vidx[i] = -1;
    }
    /* 2. Add vertices (landmark list first) */
    for (i = 0; i < n_all; ++i) {
        const sv_tr_kf* kf = kf_get(mp, (int)all[i]);
        for (k = 0; k < kf->obs->num_kp; ++k) {
            const sv_tr_lm* lm;
            if (kf->lm[k] < 0) {
                continue;
            }
            lm = lm_get(mp, kf->lm[k]);
            if (!lm || lm_seen[lm->id]) {
                continue;
            }
            lm_seen[lm->id] = 1;
            all_lms[n_lms++] = lm->id;
        }
    }
    sv_s3_graph_init(&g);
    g.use_terminate = 1;
    g.gain_threshold = 1e-3;
    g.fix_scale = 0;
    for (i = 0; i < n_all; ++i) {
        const sv_tr_kf* kf = kf_get(mp, (int)all[i]);
        const id_sim3* pc = find_sim3(pre_corr, n_pre, kf->id);
        sv_sim3 est;
        int fixed;
        if (pc) {
            est = pc->s;
        }
        else {
            double rot[9], trans[3];
            kf_rot_trans(kf, rot, trans);
            sv_sim3_from_rot(rot, trans, 1.0, &est);
        }
        sim3_cw[kf->id].id = kf->id;
        sim3_cw[kf->id].s = est;
        has_cw[kf->id] = 1;
        fixed = (kf->id == loop_id || kf->id == cur_id || kf->is_root);
        {
            TB_INIT(b);
            tb_u(&b, kf->id);
            tb_u(&b, fixed ? 1 : 0);
            tb_sim3(&b, &est);
            tb_emit(L, "GV", &b);
        }
        order[n_order++] = kf->id;
    }
    /* the C graph needs ascending vertex ids */
    qsort(order, n_order, sizeof(unsigned int), cmp_uint);
    for (i = 0; i < n_order; ++i) {
        const sv_tr_kf* kf = kf_get(mp, (int)order[i]);
        const int fixed = (kf->id == loop_id || kf->id == cur_id || kf->is_root);
        vidx[kf->id] = sv_s3_add_vertex(&g, kf->id, &sim3_cw[kf->id].s, fixed);
    }

    /* 3. Add edges */
#define INSERT_EDGE(id1, id2, S21)                                            \
    do {                                                                      \
        sv_sim3 s21__ = (S21);                                                \
        sv_s3_add_graph_edge(&g, vidx[(id1)], vidx[(id2)], &s21__);           \
        inserted[n_ins].a = (id1) < (id2) ? (id1) : (id2);                    \
        inserted[n_ins].b = (id1) < (id2) ? (id2) : (id1);                    \
        ++n_ins;                                                              \
        {                                                                     \
            TB_INIT(b__);                                                     \
            tb_u(&b__, (id1));                                                \
            tb_u(&b__, (id2));                                                \
            tb_sim3(&b__, &s21__);                                            \
            tb_emit(L, "GE", &b__);                                           \
        }                                                                     \
    } while (0)

    for (i = 0; i < n_conn; ++i) { /* loop edges over the number of shared landmarks threshold */
        const unsigned int id1 = loop_conn[i].kf;
        sv_sim3 s_w1;
        sv_sim3_inverse(&sim3_cw[id1].s, &s_w1);
        for (k = 0; k < loop_conn[i].n; ++k) {
            const unsigned int id2 = loop_conn[i].ids[k];
            sv_sim3 s21;
            if (!(id1 == cur_id && id2 == loop_id) && conn_weight(mp, id1, id2) < min_shared) {
                continue;
            }
            sv_sim3_mul(&sim3_cw[id2].s, &s_w1, &s21);
            INSERT_EDGE(id1, id2, s21);
        }
    }
    for (i = 0; i < n_all; ++i) { /* non-loop-connected edges */
        const sv_tr_kf* kf = kf_get(mp, (int)all[i]);
        const unsigned int id1 = kf->id;
        const id_sim3* n1 = find_sim3(non_corr, n_non, id1);
        sv_sim3 s_w1;
        const unsigned int* le;
        unsigned int nle, ncov, j;
        unsigned int* cov;
        sv_sim3_inverse(n1 ? &n1->s : &sim3_cw[id1].s, &s_w1);
        if (kf->parent >= 0 && kf_get(mp, kf->parent)) {
            const unsigned int id2 = (unsigned int)kf->parent;
            const id_sim3* n2 = find_sim3(non_corr, n_non, id2);
            sv_sim3 s21;
            sv_sim3_mul(n2 ? &n2->s : &sim3_cw[id2].s, &s_w1, &s21);
            INSERT_EDGE(id1, id2, s21);
        }
        nle = sv_loop_get_loop_edges(L, id1, &le);
        for (j = 0; j < nle; ++j) {
            const unsigned int id2 = le[j];
            const id_sim3* n2;
            sv_sim3 s21;
            if (id1 <= id2) {
                continue;
            }
            n2 = find_sim3(non_corr, n_non, id2);
            sv_sim3_mul(n2 ? &n2->s : &sim3_cw[id2].s, &s_w1, &s21);
            INSERT_EDGE(id1, id2, s21);
        }
        cov = (unsigned int*)malloc((kf->n_covis + 1) * sizeof(unsigned int));
        ncov = covisibilities_over(L, kf, min_shared, cov);
        for (j = 0; j < ncov; ++j) {
            const unsigned int id2 = cov[j];
            const id_sim3* n2;
            sv_sim3 s21;
            unsigned int q;
            int dup = 0;
            if (!(kf->parent >= 0)) { /* `!connected_keyfrm || !parent_node` */
                continue;
            }
            if (id2 == (unsigned int)kf->parent) {
                continue;
            }
            {
                const sv_tr_kf* other = kf_get(mp, (int)id2);
                int is_child = 0;
                for (q = 0; q < kf->n_children; ++q) {
                    is_child |= (kf->children[q] == id2);
                }
                if (is_child) {
                    continue;
                }
                if (idset_has(le, nle, id2)) {
                    continue;
                }
                if (!other) { /* connected_keyfrm->will_be_erased() */
                    continue;
                }
            }
            if (id1 <= id2) {
                continue;
            }
            for (q = 0; q < n_ins; ++q) {
                dup |= (inserted[q].a == (id1 < id2 ? id1 : id2) && inserted[q].b == (id1 < id2 ? id2 : id1));
            }
            if (dup) {
                continue;
            }
            n2 = find_sim3(non_corr, n_non, id2);
            sv_sim3_mul(n2 ? &n2->s : &sim3_cw[id2].s, &s_w1, &s21);
            INSERT_EDGE(id1, id2, s21);
        }
        free(cov);
    }
#undef INSERT_EDGE

    /* 4. Perform a pose graph optimization */
    sv_s3_initialize_optimization(&g);
    {
        const int iters = sv_s3_optimize(&g, 50);
        double chi = 0.0;
        for (k = 0; k < (unsigned int)g.n_active_edges; ++k) {
            chi += sv_s3_edge_chi2(&g.e[g.active_edges[k]]);
        }
        TB_INIT(b);
        tb_i(&b, iters);
        tb_d(&b, chi);
        tb_emit(L, "GOPT", &b);
    }

    /* 5. Update the camera poses and point-cloud */
    for (i = 0; i < n_all; ++i) {
        sv_tr_kf* kf = kf_get(mp, (int)all[i]);
        const sv_sim3* est = &g.v[vidx[kf->id]].est;
        const float s = (float)est->s;
        double R[9], trans[3], pose[16];
        {
            TB_INIT(b);
            tb_u(&b, kf->id);
            tb_sim3(&b, est);
            tb_emit(L, "GX", &b);
        }
        sv_quat_to_mat3(&est->r, R);
        trans[0] = est->t[0] / (double)s;
        trans[1] = est->t[1] / (double)s;
        trans[2] = est->t[2] / (double)s;
        pose_from_rt(R, trans, pose);
        sv_tr_kf_set_pose_cw(kf, pose);
        corr_wc[kf->id].id = kf->id;
        sv_sim3_inverse(est, &corr_wc[kf->id].s);
        has_wc[kf->id] = 1;
    }
    for (i = 0; i < n_lms; ++i) {
        sv_tr_lm* lm = lm_get(mp, (int)all_lms[i]);
        unsigned int ref_id = 0, q;
        double p1[3], p2[3];
        int have = 0;
        if (!lm) {
            continue;
        }
        for (q = 0; q < n_found; ++q) {
            if (found_ids[q] == lm->id) {
                ref_id = found_ref[q];
                have = 1;
                break;
            }
        }
        if (!have) {
            ref_id = (unsigned int)lm->ref_kf;
        }
        sv_sim3_map(&sim3_cw[ref_id].s, lm->pos_w, p1);
        sv_sim3_map(&corr_wc[ref_id].s, p1, p2);
        lm_set_pos(L, lm, p2);
        lm_update_mean_normal(L, lm);
    }
    sv_s3_graph_free(&g);
    free(all);
    free(sim3_cw);
    free(has_cw);
    free(vidx);
    free(order);
    free(inserted);
    free(all_lms);
    free(lm_seen);
    free(corr_wc);
    free(has_wc);
}

/* module::loop_bundle_adjuster::optimize */
static void loop_bundle_adjust(sv_loop* L, unsigned int cur_id) {
    sv_mapping* mp = L->mp;
    sv_tr_map* map = L->map;
    const sv_tr_config* cfg = L->cfg;
    unsigned int* keyfrms = (unsigned int*)malloc((map->kf_cap + 1) * sizeof(unsigned int));
    unsigned int n_kfs = keyframes_from_root(L, cur_id, keyfrms), i, k;
    sv_bav_view view;
    sv_bav_kf* vk;
    sv_bav_lm* vl;
    unsigned int nk = 0, nl = 0;
    sv_bav_result res;
    int flag = 0;
    double (*after_pose)[16] = (double(*)[16])calloc(map->kf_cap + 1, sizeof(double[16]));
    unsigned char* have_after = (unsigned char*)calloc(map->kf_cap + 1, 1);
    unsigned char* optimized_kf = (unsigned char*)calloc(map->kf_cap + 1, 1);
    double (*before_pose)[16] = (double(*)[16])calloc(map->kf_cap + 1, sizeof(double[16]));
    unsigned char* optimized_lm = (unsigned char*)calloc(map->lm_cap + 1, 1);
    double (*after_pos)[3] = (double(*)[3])calloc(map->lm_cap + 1, sizeof(double[3]));

    for (i = 0; i < map->kf_cap; ++i) {
        if (map->kfs[i] && map->kfs[i]->alive) {
            ++nk;
        }
    }
    for (i = 0; i < map->lm_cap; ++i) {
        if (map->lms[i] && map->lms[i]->alive) {
            ++nl;
        }
    }
    vk = (sv_bav_kf*)calloc(nk ? nk : 1, sizeof(sv_bav_kf));
    vl = (sv_bav_lm*)calloc(nl ? nl : 1, sizeof(sv_bav_lm));
    nk = 0;
    for (i = 0; i < map->kf_cap; ++i) {
        const sv_tr_kf* kf = map->kfs[i];
        unsigned int* slots;
        sv_bav_kp* kps;
        unsigned int nkp = 0;
        int r, c;
        if (!kf || !kf->alive) {
            continue;
        }
        vk[nk].id = kf->id;
        vk[nk].erased = 0;
        vk[nk].spanning_root = kf->is_root;
        for (r = 0; r < 4; ++r) {
            for (c = 0; c < 4; ++c) {
                vk[nk].pose_cw[r * 4 + c] = kf->pose_cw[c * 4 + r];
            }
        }
        vk[nk].has_slots = 1;
        vk[nk].n_slots = (int)kf->obs->num_kp;
        slots = (unsigned int*)malloc((kf->obs->num_kp ? kf->obs->num_kp : 1) * sizeof(unsigned int));
        kps = (sv_bav_kp*)malloc((kf->obs->num_kp ? kf->obs->num_kp : 1) * sizeof(sv_bav_kp));
        for (k = 0; k < kf->obs->num_kp; ++k) {
            const sv_tr_lm* lm = kf->lm[k] >= 0 ? map->lms[kf->lm[k]] : NULL;
            if (lm && lm->alive) {
                slots[k] = lm->id;
                kps[nkp].idx = k;
                kps[nkp].x = kf->obs->kp[k].x;
                kps[nkp].y = kf->obs->kp[k].y;
                kps[nkp].octave = kf->obs->kp[k].octave;
                ++nkp;
            }
            else {
                slots[k] = NONE_ID;
            }
        }
        vk[nk].slots = slots;
        vk[nk].n_kp = (int)nkp;
        vk[nk].kps = kps;
        ++nk;
    }
    nl = 0;
    for (i = 0; i < map->lm_cap; ++i) {
        const sv_tr_lm* lm = map->lms[i];
        if (!lm || !lm->alive) {
            continue;
        }
        vl[nl].id = lm->id;
        vl[nl].erased = 0;
        vl[nl].pos[0] = lm->pos_w[0];
        vl[nl].pos[1] = lm->pos_w[1];
        vl[nl].pos[2] = lm->pos_w[2];
        vl[nl].n_obs = (int)lm->num_obs;
        vl[nl].obs_kf = lm->obs_kf;
        vl[nl].obs_idx = lm->obs_idx;
        ++nl;
    }
    memset(&view, 0, sizeof(view));
    view.kfs = vk;
    view.n_kfs = (int)nk;
    view.lms = vl;
    view.n_lms = (int)nl;
    view.fx = cfg->fx;
    view.fy = cfg->fy;
    view.cx = cfg->cx;
    view.cy = cfg->cy;
    view.n_isq = (int)cfg->num_levels;
    view.isq = cfg->inv_level_sigma_sq;
    view.fixed_keyframe_id_threshold = map->fixed_keyframe_id_threshold;

    sv_bav_global_loop(&view, keyfrms, (int)n_kfs, L->loop_ba_num_iter, 0, &flag, &res);
    if (res.returned_ok) {
        /* results as maps: keyfrm_to_pose_cw_after_global_BA, optimized ids, lm_to_pos_w_after_global_BA */
        for (i = 0; i < (unsigned int)res.n_applied; ++i) {
            const unsigned int id = res.applied_kf[i];
            int r, c;
            for (r = 0; r < 4; ++r) {
                for (c = 0; c < 4; ++c) {
                    after_pose[id][c * 4 + r] = res.applied_pose[i][r * 4 + c];
                }
            }
            have_after[id] = 1;
            optimized_kf[id] = 1;
        }
        for (i = 0; i < (unsigned int)res.n_opt_lm; ++i) {
            optimized_lm[res.opt_lm[i]] = 1;
        }
        for (i = 0; i < (unsigned int)res.g.nv; ++i) {
            const sv_ba_vertex* v = &res.g.v[i];
            if (!v->is_landmark) {
                continue;
            }
            if (res.vtx_owner[i] < map->lm_cap && optimized_lm[res.vtx_owner[i]]) {
                memcpy(after_pos[res.vtx_owner[i]], v->pos, sizeof(double) * 3);
            }
        }

        /* update the camera pose along the spanning tree from the root */
        {
            unsigned int head = 0, n = 0;
            unsigned int* queue = (unsigned int*)malloc((map->kf_cap + 1) * sizeof(unsigned int));
            unsigned int root = keyfrms[0];
            queue[n++] = root;
            while (head < n) {
                sv_tr_kf* parent = kf_get(mp, (int)queue[head++]);
                double cam_pose_wp[16];
                memcpy(cam_pose_wp, parent->pose_wc, sizeof(cam_pose_wp));
                for (i = 0; i < parent->n_children; ++i) {
                    sv_tr_kf* child = kf_get(mp, (int)parent->children[i]);
                    if (!optimized_kf[child->id]) {
                        double cam_pose_cp[16];
                        mat44_mul_cm(child->pose_cw, cam_pose_wp, cam_pose_cp);
                        mat44_mul_cm(cam_pose_cp, after_pose[parent->id], after_pose[child->id]);
                        have_after[child->id] = 1;
                        optimized_kf[child->id] = 1;
                    }
                    queue[n++] = child->id;
                }
                memcpy(before_pose[parent->id], parent->pose_cw, sizeof(double) * 16);
                sv_tr_kf_set_pose_cw(parent, after_pose[parent->id]);
            }
            free(queue);
        }

        /* update the positions of the landmarks */
        {
            unsigned char* seen = (unsigned char*)calloc(map->lm_cap + 1, 1);
            unsigned int* lms = (unsigned int*)malloc((map->lm_cap + 1) * sizeof(unsigned int));
            unsigned int n_l = 0;
            for (i = 0; i < n_kfs; ++i) {
                const sv_tr_kf* kf = kf_get(mp, (int)keyfrms[i]);
                for (k = 0; k < kf->obs->num_kp; ++k) {
                    const sv_tr_lm* lm;
                    if (kf->lm[k] < 0) {
                        continue;
                    }
                    lm = lm_get(mp, kf->lm[k]);
                    if (!lm || seen[lm->id]) {
                        continue;
                    }
                    seen[lm->id] = 1;
                    lms[n_l++] = lm->id;
                }
            }
            for (i = 0; i < n_l; ++i) {
                sv_tr_lm* lm = lm_get(mp, (int)lms[i]);
                if (!lm) {
                    continue;
                }
                if (optimized_lm[lm->id]) {
                    lm_set_pos(L, lm, after_pos[lm->id]);
                }
                else {
                    const sv_tr_kf* ref = kf_get(mp, lm->ref_kf);
                    double rot_b[9], trans_b[3], pos_c[3], t1[3], rot_wc[9], trans_wc[3], p[3];
                    int r, c;
                    for (c = 0; c < 3; ++c) {
                        for (r = 0; r < 3; ++r) {
                            rot_b[c * 3 + r] = before_pose[ref->id][c * 4 + r];
                        }
                    }
                    trans_b[0] = before_pose[ref->id][12];
                    trans_b[1] = before_pose[ref->id][13];
                    trans_b[2] = before_pose[ref->id][14];
                    sv_mat3_mulv(rot_b, lm->pos_w, t1);
                    pos_c[0] = t1[0] + trans_b[0];
                    pos_c[1] = t1[1] + trans_b[1];
                    pos_c[2] = t1[2] + trans_b[2];
                    for (c = 0; c < 3; ++c) {
                        for (r = 0; r < 3; ++r) {
                            rot_wc[c * 3 + r] = ref->pose_wc[c * 4 + r];
                        }
                    }
                    trans_wc[0] = ref->pose_wc[12];
                    trans_wc[1] = ref->pose_wc[13];
                    trans_wc[2] = ref->pose_wc[14];
                    sv_mat3_mulv(rot_wc, pos_c, t1);
                    p[0] = t1[0] + trans_wc[0];
                    p[1] = t1[1] + trans_wc[1];
                    p[2] = t1[2] + trans_wc[2];
                    lm_set_pos(L, lm, p);
                }
                lm_update_mean_normal(L, lm);
            }
            free(seen);
            free(lms);
        }
    }
    sv_bav_result_free(&res);
    for (i = 0; i < nk; ++i) {
        free((void*)vk[i].slots);
        free((void*)vk[i].kps);
    }
    free(vk);
    free(vl);
    free(keyfrms);
    free(after_pose);
    free(have_after);
    free(optimized_kf);
    free(before_pose);
    free(optimized_lm);
    free(after_pos);
}

typedef struct repl_map {
    int* kv; /* pairs (from id, to id), unsorted */
    unsigned int n, cap;
} repl_map;

static void repl_put(repl_map* r, unsigned int from, unsigned int to) {
    unsigned int i;
    for (i = 0; i < r->n; ++i) {
        if ((unsigned int)r->kv[2 * i] == from) {
            r->kv[2 * i + 1] = (int)to;
            return;
        }
    }
    if (r->n == r->cap) {
        r->cap = r->cap ? r->cap * 2 : 64;
        r->kv = (int*)realloc(r->kv, r->cap * 2 * sizeof(int));
    }
    r->kv[2 * r->n] = (int)from;
    r->kv[2 * r->n + 1] = (int)to;
    r->n++;
}

static int cmp_pair_first(const void* a, const void* b) {
    const int x = ((const int*)a)[0], y = ((const int*)b)[0];
    return (x > y) - (x < y);
}

/* match::fuse::detect_duplication(...) with do_reprojection_matching = false (the default global_optimization_module::replace_duplicated_landmarks relies on) */
static void detect_duplication_no_reproj(sv_mapping* m, sv_tr_kf* kf, const double rot_cw[9], const double trans_cw[3],
                               sv_tr_lm* const* lms, unsigned int n_lms, float margin, dup_pair** dups, unsigned int* n_dups,
                               new_conn** ncs, unsigned int* n_ncs) {
    const sv_tr_config* cfg = m->cfg;
    double trans_wc[3], t[3];
    unsigned char* already = (unsigned char*)calloc(kf->obs->num_kp ? kf->obs->num_kp : 1, 1);
    unsigned int* indices = (unsigned int*)malloc((kf->obs->num_kp ? kf->obs->num_kp : 1) * sizeof(unsigned int));
    unsigned int li;
    *n_dups = 0;
    *dups = (dup_pair*)malloc((n_lms ? n_lms : 1) * sizeof(dup_pair));
    *n_ncs = 0;
    *ncs = (new_conn*)malloc((n_lms ? n_lms : 1) * sizeof(new_conn));

    sv_mat3_mulv_lhs_transposed(rot_cw, trans_cw, t); /* -rot_cw.transpose() * trans_cw */
    trans_wc[0] = -t[0];
    trans_wc[1] = -t[1];
    trans_wc[2] = -t[2];

    for (li = 0; li < n_lms; ++li) {
        sv_tr_lm* lm = lms[li];
        double reproj[2], cam_to_lm_vec[3], cam_to_lm_dist, max_d, min_d, margin_far, margin_near;
        float x_right;
        unsigned int pred_scale_level, n_idx, k;
        int min_level, max_level;
        unsigned int best_dist = MAX_HAMMING_DIST;
        int best_idx = -1;
        if (!lm) {
            continue;
        }
        if (!lm->alive) {
            continue;
        }
        if (lm_is_observed_in(lm, kf->id)) {
            continue;
        }
        if (!sv_tr_reproject_to_image(cfg, rot_cw, trans_cw, lm->pos_w, reproj, &x_right)) {
            continue;
        }
        cam_to_lm_vec[0] = lm->pos_w[0] - trans_wc[0];
        cam_to_lm_vec[1] = lm->pos_w[1] - trans_wc[1];
        cam_to_lm_vec[2] = lm->pos_w[2] - trans_wc[2];
        cam_to_lm_dist = sv_vec3_norm(cam_to_lm_vec);
        margin_far = 1.3;
        margin_near = 1.0 / margin_far;
        max_d = margin_far * (double)lm->max_valid_dist;
        min_d = margin_near * (double)lm->min_valid_dist;
        if (cam_to_lm_dist < min_d || max_d < cam_to_lm_dist) {
            continue;
        }
        if (sv_vec3_dot(cam_to_lm_vec, lm->mean_normal) < 0.5 * cam_to_lm_dist) {
            continue;
        }
        pred_scale_level = predict_scale_level(lm, (float)cam_to_lm_dist, (float)cfg->num_levels, cfg->log_scale_factor);
        min_level = (int)pred_scale_level - 1 > 0 ? (int)pred_scale_level - 1 : 0;
        max_level = (int)(cfg->num_levels - 1 < pred_scale_level + 1 ? cfg->num_levels - 1 : pred_scale_level + 1);
        n_idx = sv_frame_get_keypoints_in_cell(&kf->obs->grid, kf->obs->kp, (float)reproj[0], (float)reproj[1],
                                                margin * cfg->scale_factors[pred_scale_level], min_level, max_level,
                                                indices, kf->obs->num_kp);
        if (n_idx == 0) {
            continue;
        }
        for (k = 0; k < n_idx; ++k) {
            const unsigned int idx = indices[k];
            unsigned int hamm_dist;
            if (already[idx]) {
                continue;
            }
            hamm_dist = sv_tr_hamming(lm->desc, kf->obs->desc + (size_t)idx * SV_TR_DESC_BYTES);
            if (hamm_dist < best_dist) {
                best_dist = hamm_dist;
                best_idx = (int)idx;
            }
        }
        if (HAMMING_DIST_THR_LOW < best_dist) {
            continue;
        }
        already[best_idx] = 1;
        if (kf->lm[best_idx] >= 0) {
            sv_tr_lm* in_kf = m->map->lms[kf->lm[best_idx]];
            if (in_kf && in_kf->alive) {
                (*dups)[*n_dups].key = lm;
                (*dups)[*n_dups].val = in_kf;
                ++*n_dups;
            }
        }
        else {
            (*ncs)[*n_ncs].idx = (unsigned int)best_idx;
            (*ncs)[*n_ncs].lm = lm;
            ++*n_ncs;
        }
    }
    qsort(*dups, *n_dups, sizeof(dup_pair), cmp_dup); /* id-ordered std::map */
    free(already);
    free(indices);
}


/* global_optimization_module::replace_duplicated_landmarks */
static void replace_duplicated(sv_loop* L, sv_tr_kf* cur, const id_sim3* after, unsigned int n_after) {
    sv_mapping* mp = L->mp;
    repl_map rep;
    unsigned int idx, i, ni;
    sv_tr_lm** covis_lms = (sv_tr_lm**)malloc((L->n_match_covis + 1) * sizeof(sv_tr_lm*));
    memset(&rep, 0, sizeof(rep));

    for (idx = 0; idx < cur->obs->num_kp; ++idx) {
        sv_tr_lm* cm;
        sv_tr_lm* lm_in_curr;
        if (L->match_cand[idx] < 0) {
            continue;
        }
        cm = lm_get(mp, L->match_cand[idx]);
        if (!cm) {
            continue;
        }
        if (lm_is_observed_in(cm, cur->id)) {
            /* cur_keyfrm_->erase_landmark(cm): the slot holding cm; then cm->erase_observation(cur) */
            int found;
            const int pos = lm_obs_pos(cm, cur->id, &found);
            if (found) {
                cur->lm[cm->obs_idx[pos]] = SV_TR_NONE;
            }
            lm_erase_observation_l(L, cm, cur->id);
        }
        lm_in_curr = cur->lm[idx] >= 0 ? lm_get(mp, cur->lm[idx]) : NULL;
        if (lm_in_curr) {
            if (lm_in_curr->id != cm->id) {
                repl_put(&rep, lm_in_curr->id, cm->id);
                lm_replace_l(L, lm_in_curr, cm);
                if (!L->flag_desc[cm->id]) {
                    lm_compute_descriptor(L, cm);
                }
                if (!L->flag_pred[cm->id]) {
                    lm_update_mean_normal(L, cm);
                }
            }
        }
        else {
            lm_connect_l(L, cur, cm, idx);
            lm_update_mean_normal(L, cm);
            lm_compute_descriptor(L, cm);
        }
    }

    for (i = 0; i < L->n_match_covis; ++i) {
        covis_lms[i] = lm_get(mp, (int)L->match_covis[i]);
    }
    for (ni = 0; ni < n_after; ++ni) {
        sv_tr_kf* neighbor = kf_get(mp, (int)after[ni].id);
        double m44[16], s_rot[9], rot_cw[9], trans_cw[3], s_cw;
        dup_pair* dups = NULL;
        new_conn* ncs = NULL;
        unsigned int n_dups = 0, n_ncs = 0;
        int* tp;
        int r, c;
        if (!neighbor) {
            continue;
        }
        sim3_to_mat44(&after[ni].s, m44);
        for (c = 0; c < 3; ++c) {
            for (r = 0; r < 3; ++r) {
                s_rot[c * 3 + r] = m44[c * 4 + r];
            }
        }
        {
            const double row0[3] = {s_rot[0], s_rot[3], s_rot[6]};
            s_cw = sqrt(dot_row_blocks(row0, row0));
        }
        for (i = 0; i < 9; ++i) {
            rot_cw[i] = s_rot[i] / s_cw;
        }
        for (i = 0; i < 3; ++i) {
            trans_cw[i] = m44[12 + i] / s_cw;
        }
        detect_duplication_no_reproj(mp, neighbor, rot_cw, trans_cw, covis_lms, L->n_match_covis, 4.0f, &dups, &n_dups, &ncs, &n_ncs);
        qsort(ncs, n_ncs, sizeof(new_conn), cmp_newconn);
        {
            TB_INIT(b);
            int* pd = (int*)malloc((n_dups * 2 + 2) * sizeof(int));
            int* pn = (int*)malloc((n_ncs * 2 + 2) * sizeof(int));
            for (i = 0; i < n_dups; ++i) {
                pd[2 * i] = (int)dups[i].key->id;
                pd[2 * i + 1] = (int)dups[i].val->id;
            }
            for (i = 0; i < n_ncs; ++i) {
                pn[2 * i] = (int)ncs[i].idx;
                pn[2 * i + 1] = (int)ncs[i].lm->id;
            }
            tb_u(&b, neighbor->id);
            tb_pairs(&b, pd, n_dups);
            tb_pairs(&b, pn, n_ncs);
            tb_emit(L, "LFU", &b);
            free(pd);
            free(pn);
        }
        for (i = 0; i < n_ncs; ++i) {
            sv_tr_lm* lm = ncs[i].lm;
            if (lm_is_observed_in(lm, neighbor->id)) {
                L->n_dup_connect++;
            }
            lm_connect_l(L, neighbor, lm, ncs[i].idx);
            lm_update_mean_normal(L, lm);
            lm_compute_descriptor(L, lm);
        }
        for (i = 0; i < n_dups; ++i) {
            sv_tr_lm* lm_to_replace = dups[i].key;
            sv_tr_lm* lm_in_neighbor = dups[i].val;
            if (lm_to_replace->id != lm_in_neighbor->id) {
                repl_put(&rep, lm_to_replace->id, lm_in_neighbor->id);
                lm_replace_l(L, lm_to_replace, lm_in_neighbor);
                if (!L->flag_desc[lm_in_neighbor->id]) {
                    lm_compute_descriptor(L, lm_in_neighbor);
                }
                if (!L->flag_pred[lm_in_neighbor->id]) {
                    lm_update_mean_normal(L, lm_in_neighbor);
                }
            }
        }
        (void)tp;
        free(dups);
        free(ncs);
    }
    qsort(rep.kv, rep.n, 2 * sizeof(int), cmp_pair_first);
    {
        TB_INIT(b);
        tb_pairs(&b, rep.kv, rep.n);
        tb_emit(L, "LREP", &b);
    }
    if (L->replaced_hook) {
        L->replaced_hook(L->replaced_user, rep.kv, rep.n);
    }
    free(rep.kv);
    free(covis_lms);
}

int sv_loop_correct(sv_loop* L, unsigned int cur_id) {
    sv_mapping* mp = L->mp;
    sv_tr_map* map = L->map;
    sv_tr_kf* cur = kf_get(mp, (int)cur_id);
    sv_tr_kf* cand = kf_get(mp, L->selected);
    unsigned int i, j, n_nb = 0;
    unsigned int* nb;
    id_sim3 *before, *after;
    unsigned int *found_ids, *found_ref, n_found = 0;
    unsigned char* found_flag;
    conn_edge_set* conns;
    double cur_wc[16];
    int same_root;
    if (!cur || !cand) {
        return -1;
    }
    {
        /* both keyframes must share the spanning root */
        const sv_tr_kf *a = cur, *b = cand;
        while (a->parent >= 0 && !a->is_root && kf_get(mp, a->parent)) {
            a = kf_get(mp, a->parent);
        }
        while (b->parent >= 0 && !b->is_root && kf_get(mp, b->parent)) {
            b = kf_get(mp, b->parent);
        }
        same_root = (a->id == b->id);
    }
    {
        TB_INIT(t);
        tb_u(&t, cand->id);
        tb_u(&t, cur_id);
        tb_u(&t, same_root ? 1 : 0);
        tb_emit(L, "L0", &t);
    }
    if (!same_root) {
        return 1; /* "The feature to merge two spanning trees has not yet been implemented." */
    }

    /* 1. Sim3 of the covisibilities of the current keyframe */
    nb = (unsigned int*)malloc((cur->n_covis + 2) * sizeof(unsigned int));
    n_nb = covisibilities_over(L, cur, L->thr_neighbor_keyframes, nb);
    nb[n_nb++] = cur_id;
    {
        int* tn = ids_to_int(nb, n_nb);
        TB_INIT(t);
        tb_ids(&t, tn, n_nb);
        tb_emit(L, "LN", &t);
        free(tn);
    }
    before = (id_sim3*)malloc((n_nb + 1) * sizeof(id_sim3));
    after = (id_sim3*)malloc((n_nb + 1) * sizeof(id_sim3));
    memcpy(cur_wc, cur->pose_wc, sizeof(cur_wc));
    for (i = 0; i < n_nb; ++i) {
        sv_tr_kf* n = kf_get(mp, (int)nb[i]);
        double rot[9], trans[3], pc[16], rot_nc[9], trans_nc[3];
        sv_sim3 s_nc;
        int r, c;
        before[i].id = n->id;
        kf_rot_trans(n, rot, trans);
        sv_sim3_from_rot(rot, trans, 1.0, &before[i].s);
        mat44_mul_cm(n->pose_cw, cur_wc, pc);
        for (c = 0; c < 3; ++c) {
            for (r = 0; r < 3; ++r) {
                rot_nc[c * 3 + r] = pc[c * 4 + r];
            }
        }
        trans_nc[0] = pc[12];
        trans_nc[1] = pc[13];
        trans_nc[2] = pc[14];
        sv_sim3_from_rot(rot_nc, trans_nc, 1.0, &s_nc);
        after[i].id = n->id;
        sv_sim3_mul(&s_nc, &L->sim3_world_to_curr, &after[i].s);
    }
    /* id-ordered maps */
    for (i = 1; i < n_nb; ++i) {
        id_sim3 tb = before[i], ta = after[i];
        for (j = i; j > 0 && before[j - 1].id > tb.id; --j) {
            before[j] = before[j - 1];
            after[j] = after[j - 1];
        }
        before[j] = tb;
        after[j] = ta;
    }
    for (i = 0; i < n_nb; ++i) {
        TB_INIT(t);
        tb_u(&t, before[i].id);
        tb_sim3(&t, &before[i].s);
        tb_emit(L, "LB", &t);
    }
    for (i = 0; i < n_nb; ++i) {
        TB_INIT(t);
        tb_u(&t, after[i].id);
        tb_sim3(&t, &after[i].s);
        tb_emit(L, "LA", &t);
    }
    /* correct_covisibility_landmarks */
    found_ids = (unsigned int*)malloc((map->lm_cap + 1) * sizeof(unsigned int));
    found_ref = (unsigned int*)malloc((map->lm_cap + 1) * sizeof(unsigned int));
    found_flag = (unsigned char*)calloc(map->lm_cap + 1, 1);
    for (i = 0; i < n_nb; ++i) {
        sv_tr_kf* n = kf_get(mp, (int)after[i].id);
        sv_sim3 s_wn;
        sv_sim3_inverse(&after[i].s, &s_wn);
        for (j = 0; j < n->obs->num_kp; ++j) {
            sv_tr_lm* lm;
            double p1[3], p2[3];
            if (n->lm[j] < 0) {
                continue;
            }
            lm = lm_get(mp, n->lm[j]);
            if (!lm) {
                continue;
            }
            if (found_flag[lm->id]) {
                continue;
            }
            found_flag[lm->id] = 1;
            found_ids[n_found] = lm->id;
            found_ref[n_found] = n->id;
            ++n_found;
            sv_sim3_map(&before[i].s, lm->pos_w, p1);
            sv_sim3_map(&s_wn, p1, p2);
            lm_set_pos(L, lm, p2);
            lm_update_mean_normal(L, lm);
        }
    }
    /* correct_covisibility_keyframes */
    for (i = 0; i < n_nb; ++i) {
        sv_tr_kf* n = kf_get(mp, (int)after[i].id);
        double R[9], trans[3], pose[16];
        const double s_nw = after[i].s.s;
        sv_quat_to_mat3(&after[i].s.r, R);
        trans[0] = after[i].s.t[0] / s_nw;
        trans[1] = after[i].s.t[1] / s_nw;
        trans[2] = after[i].s.t[2] / s_nw;
        pose_from_rt(R, trans, pose);
        sv_tr_kf_set_pose_cw(n, pose);
    }
    {
        /* LF in ascending landmark id, LK in the neighbor order */
        unsigned int* order = (unsigned int*)malloc((n_found + 1) * sizeof(unsigned int));
        unsigned int q, nfo = 0;
        for (q = 0; q < map->lm_cap && nfo < n_found; ++q) {
            if (found_flag[q]) {
                order[nfo++] = q;
            }
        }
        for (q = 0; q < n_found; ++q) {
            const unsigned int id = order[q];
            unsigned int w;
            const sv_tr_lm* lm = lm_get(mp, (int)id);
            TB_INIT(t);
            tb_u(&t, id);
            for (w = 0; w < n_found; ++w) {
                if (found_ids[w] == id) {
                    tb_u(&t, found_ref[w]);
                    break;
                }
            }
            if (lm) {
                tb_vec3(&t, lm->pos_w);
            }
            tb_emit(L, "LF", &t);
        }
        free(order);
        for (i = 0; i < n_nb; ++i) {
            const sv_tr_kf* n = kf_get(mp, (int)after[i].id);
            TB_INIT(t);
            tb_u(&t, n->id);
            tb_mat44_cm(&t, n->pose_cw);
            tb_emit(L, "LK", &t);
        }
    }
    free(found_flag);

    /* 2. resolve duplications of landmarks caused by loop fusion */
    replace_duplicated(L, cur, after, n_nb);

    /* 3. extract the new connections created after loop fusion (covisibility order of curr_neighbors) */
    conns = extract_new_connections(L, nb, n_nb);
    for (i = 0; i < n_nb; ++i) {
        int* t = ids_to_int(conns[i].ids, conns[i].n);
        TB_INIT(b);
        tb_u(&b, conns[i].kf);
        tb_ids(&b, t, conns[i].n);
        tb_emit(L, "LNC", &b);
        free(t);
    }

    /* 4. pose graph optimization */
    graph_optimize(L, cand->id, cur_id, before, n_nb, after, n_nb, conns, n_nb, found_ids, found_ref, n_found);
    if (L->hook) {
        L->hook(L->hook_user, 1);
    }
    add_loop_edge(L, cand->id, cur_id);
    add_loop_edge(L, cur_id, cand->id);

    /* 5. loop BA */
    loop_bundle_adjust(L, cur_id);

    /* 6. post-processing */
    L->prev_loop_correct_keyfrm_id = cur_id;

    for (i = 0; i < n_nb; ++i) {
        free(conns[i].ids);
    }
    free(conns);
    free(found_ids);
    free(found_ref);
    free(before);
    free(after);
    free(nb);
    return 0;
}
