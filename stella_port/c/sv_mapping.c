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

/* stella_vslam e445b545: mapping_module.cc (mapping_with_new_keyframe,
 * store_new_keyframe, create_new_landmarks, triangulate_with_two_keyframes,
 * update_new_keyframe, fuse_landmark_duplication), module/local_map_cleaner.cc,
 * module/two_view_triangulator.{h,cc}, data/graph_node.cc, data/keyframe.cc
 * (prepare_for_erasing), data/landmark.cc, match/fuse.cc,
 * optimize/local_bundle_adjuster_g2o.cc (steps 7/8, applying the results).
 * See sv_mapping.h. */
#include "sv_mapping.h"
#include "sv_bundle_adjuster.h"
#include "sv_linalg.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define NONE_ID 0xFFFFFFFFu
#define HAMMING_DIST_THR_LOW 50u
#define MAX_HAMMING_DIST 256u

/* ------------------------------------------------------------------ */
/* small helpers                                                      */
/* ------------------------------------------------------------------ */
static sv_tr_kf* kf_get(sv_mapping* m, int id) {
    sv_tr_kf* k;
    if (id < 0 || (unsigned int)id >= m->map->kf_cap) {
        return NULL;
    }
    k = m->map->kfs[id];
    return (k && k->alive) ? k : NULL;
}

static sv_tr_lm* lm_get(sv_mapping* m, int id) {
    sv_tr_lm* l;
    if (id < 0 || (unsigned int)id >= m->map->lm_cap) {
        return NULL;
    }
    l = m->map->lms[id];
    return (l && l->alive) ? l : NULL;
}

static unsigned int count_alive_keyframes(const sv_mapping* m) {
    unsigned int i, n = 0;
    for (i = 0; i < m->map->kf_cap; ++i) {
        if (m->map->kfs[i] && m->map->kfs[i]->alive) {
            ++n;
        }
    }
    return n;
}

void sv_mapping_init(sv_mapping* m, const sv_tr_config* cfg, sv_tr_map* map) {
    unsigned int level;
    float scale_factor_at_level = 1.0f;
    memset(m, 0, sizeof(*m));
    m->cfg = cfg;
    m->map = map;
    m->min_num_shared_lms = 15;
    m->num_cov_gen = 20;
    m->num_cov_fuse = 20;
    m->baseline_dist_thr_ratio = 0.02;
    /* residual_rad_thr_(yaml["residual_deg_thr"].as<float>(0.2) * M_PI / 180.0) */
    m->residual_rad_thr = (float)(((double)0.2f * 3.14159265358979323846) / 180.0);
    m->observed_ratio_thr = 0.3;
    m->num_reliable_keyfrms = 2;
    m->redundant_obs_ratio_thr = 0.9;
    m->top_n_covis_to_search = 30;
    m->ba_first_iter = 5;
    m->ba_second_iter = 10;
    /* orb_params::calc_level_sigma_sq */
    m->level_sigma_sq[0] = 1.0f;
    for (level = 1; level < cfg->num_levels; ++level) {
        scale_factor_at_level = cfg->scale_factor * scale_factor_at_level;
        m->level_sigma_sq[level] = scale_factor_at_level * scale_factor_at_level;
    }
}

void sv_mapping_free(sv_mapping* m) {
    unsigned int i;
    for (i = 0; i < m->conn_cap; ++i) {
        if (m->conn[i].n) {
            sv_rb_free(&m->conn[i]);
        }
    }
    free(m->conn);
    free(m->expired);
    free(m->fresh);
    free(m->lm_first_kf);
    memset(m, 0, sizeof(*m));
}

void sv_mapping_trace_free(sv_mapping_trace* t) {
    unsigned int i;
    free(t->culled_lms);
    for (i = 0; i < t->n_tri; ++i) {
        free(t->tri[i].matches);
        free(t->tri[i].acc_ids);
        free(t->tri[i].acc_pos);
    }
    free(t->tri);
    free(t->replaced);
    free(t->culled_kfs);
    memset(t, 0, sizeof(*t));
}

void sv_mapping_set_lm_first_kf(sv_mapping* m, unsigned int lm_id, unsigned int first_kf) {
    if (lm_id >= m->lm_first_cap) {
        unsigned int nc = lm_id + 4096, i;
        m->lm_first_kf = (unsigned int*)realloc(m->lm_first_kf, nc * sizeof(unsigned int));
        for (i = m->lm_first_cap; i < nc; ++i) {
            m->lm_first_kf[i] = 0;
        }
        m->lm_first_cap = nc;
    }
    m->lm_first_kf[lm_id] = first_kf;
}

/* ------------------------------------------------------------------ */
/* connected_keyfrms_and_num_shared_lms_ (id_ordered_map<weak_ptr>)   */
/* ------------------------------------------------------------------ */
int sv_mapping_is_expired(const sv_mapping* m, unsigned int kf_id) {
    return kf_id < m->expired_cap && m->expired[kf_id];
}

void sv_mapping_set_expired(sv_mapping* m, unsigned int kf_id, int expired) {
    if (kf_id >= m->expired_cap) {
        unsigned int nc = kf_id + 64;
        m->expired = (unsigned char*)realloc(m->expired, nc);
        memset(m->expired + m->expired_cap, 0, nc - m->expired_cap);
        m->expired_cap = nc;
    }
    m->expired[kf_id] = (unsigned char)(expired != 0);
}

/* id_less<std::weak_ptr<keyframe>>: !a.expired() && (b.expired() || a.lock()->id_ < b.lock()->id_) */
static int conn_less(void* ctx, unsigned int a, unsigned int b) {
    const sv_mapping* m = (const sv_mapping*)ctx;
    return !sv_mapping_is_expired(m, a) && (sv_mapping_is_expired(m, b) || a < b);
}

static sv_rbtree* conn_tree(sv_mapping* m, unsigned int kf_id) {
    if (kf_id >= m->conn_cap) {
        unsigned int nc = kf_id + 64;
        m->conn = (sv_rbtree*)realloc(m->conn, nc * sizeof(sv_rbtree));
        memset(m->conn + m->conn_cap, 0, (nc - m->conn_cap) * sizeof(sv_rbtree));
        m->conn_cap = nc;
    }
    if (!m->conn[kf_id].n) {
        sv_rb_init(&m->conn[kf_id]);
    }
    return &m->conn[kf_id];
}

typedef struct wid {
    unsigned int w, id;
} wid;

/* std::sort(rbegin, rend, less_number_and_id_object_pairs) over pairs whose keyframe
 * pointer may be null (expired key): the forward sequence ends up sorted by count
 * descending, null pointers first within a count, then id descending */
static int cmp_wid_desc(const void* pa, const void* pb) {
    const wid* a = (const wid*)pa;
    const wid* b = (const wid*)pb;
    if (a->w != b->w) {
        return a->w > b->w ? -1 : 1;
    }
    if (a->id == NONE_ID || b->id == NONE_ID) {
        return (a->id == NONE_ID) == (b->id == NONE_ID) ? 0 : (a->id == NONE_ID ? -1 : 1);
    }
    if (a->id != b->id) {
        return a->id > b->id ? -1 : 1;
    }
    return 0;
}

static void set_ordered(sv_tr_kf* k, const wid* v, unsigned int n) {
    unsigned int i;
    k->covis = (unsigned int*)realloc(k->covis, (n ? n : 1) * sizeof(unsigned int));
    k->covis_w = (unsigned int*)realloc(k->covis_w, (n ? n : 1) * sizeof(unsigned int));
    for (i = 0; i < n; ++i) {
        k->covis[i] = v[i].id;
        k->covis_w[i] = v[i].w;
    }
    k->n_covis = n;
}

/* graph_node::update_covisibility_orders_impl: re-sort from the full map. `k` may be NULL for a
 * keyframe object that was erased from the map but is still alive (its list is never read). */
static void update_covisibility_orders(sv_mapping* m, unsigned int kf_id, sv_tr_kf* k) {
    sv_rbtree* t = conn_tree(m, kf_id);
    wid* v;
    unsigned int n = 0;
    int nd;
    if (!k) {
        return;
    }
    v = (wid*)malloc((t->count ? t->count : 1) * sizeof(wid));
    for (nd = sv_rb_first(t); nd >= 0; nd = sv_rb_next(t, nd)) {
        v[n].w = t->n[nd].val;
        v[n].id = sv_mapping_is_expired(m, t->n[nd].key) ? NONE_ID : t->n[nd].key; /* first.lock() == nullptr */
        ++n;
    }
    qsort(v, n, sizeof(wid), cmp_wid_desc);
    set_ordered(k, v, n);
    free(v);
}

/* ordered_covisibilities_ entry is visible iff its weak_ptr is not expired */
static int covis_entry_visible(const sv_mapping* m, unsigned int id) {
    return id != NONE_ID && !sv_mapping_is_expired(m, id);
}

/* graph_node::get_top_n_covisibilities(n) / get_covisibilities() (n = UINT_MAX): expired entries are
 * skipped and do not count. Returns the count. */
static unsigned int top_n_covisibilities(const sv_mapping* m, const sv_tr_kf* k, unsigned int n, unsigned int* out) {
    unsigned int i, cnt = 0;
    for (i = 0; i < k->n_covis; ++i) {
        if (cnt == n) {
            break;
        }
        if (!covis_entry_visible(m, k->covis[i])) {
            continue;
        }
        out[cnt++] = k->covis[i];
    }
    return cnt;
}

static unsigned int conn_weight(sv_mapping* m, unsigned int kf_id, unsigned int other) {
    sv_rbtree* t = conn_tree(m, kf_id);
    const int nd = sv_rb_find(t, other, conn_less, m);
    return nd >= 0 ? t->n[nd].val : 0;
}

unsigned int sv_mapping_covisibilities(sv_mapping* m, const sv_tr_kf* kf, unsigned int* ids, unsigned int* weights) {
    unsigned int n = top_n_covisibilities(m, kf, 0xFFFFFFFFu, ids), i;
    for (i = 0; i < n; ++i) {
        weights[i] = conn_weight(m, kf->id, ids[i]); /* get_num_shared_landmarks() */
    }
    return n;
}

unsigned int sv_mapping_dump_conn(sv_mapping* m, unsigned int kf_id, unsigned int* ids, unsigned int* weights, unsigned int n_max) {
    sv_rbtree* t = conn_tree(m, kf_id);
    unsigned int n = 0;
    int nd;
    for (nd = sv_rb_first(t); nd >= 0 && n < n_max; nd = sv_rb_next(t, nd)) {
        ids[n] = sv_mapping_is_expired(m, t->n[nd].key) ? NONE_ID : t->n[nd].key;
        weights[n] = t->n[nd].val;
        ++n;
    }
    return n;
}

static void add_connection(sv_mapping* m, unsigned int kf_id, unsigned int other, unsigned int num_shared) {
    sv_rbtree* t = conn_tree(m, kf_id);
    int nd = sv_rb_find(t, other, conn_less, m); /* count(keyfrm) */
    int need_update = 0;
    if (nd < 0) {
        nd = sv_rb_index(t, other, conn_less, m); /* connected[keyfrm] = num_shared_lms */
        t->n[nd].val = num_shared;
        need_update = 1;
    }
    else if (t->n[nd].val != num_shared) {
        t->n[nd].val = num_shared;
        need_update = 1;
    }
    if (need_update) {
        update_covisibility_orders(m, kf_id, kf_get(m, (int)kf_id));
    }
}

static void erase_connection(sv_mapping* m, unsigned int kf_id, unsigned int other) {
    sv_rbtree* t = conn_tree(m, kf_id);
    if (sv_rb_find(t, other, conn_less, m) >= 0) {
        sv_rb_erase_key(t, other, conn_less, m);
        update_covisibility_orders(m, kf_id, kf_get(m, (int)kf_id));
    }
}

static void erase_all_connections(sv_mapping* m, sv_tr_kf* k) {
    sv_rbtree* t = conn_tree(m, k->id);
    int nd;
    for (nd = sv_rb_first(t); nd >= 0; nd = sv_rb_next(t, nd)) {
        const unsigned int other = t->n[nd].key;
        if (sv_mapping_is_expired(m, other)) {
            continue; /* keyframe_and_num_shared_lms.first.expired() */
        }
        erase_connection(m, other, k->id);
    }
    sv_rb_clear(conn_tree(m, k->id));
    k->n_covis = 0;
}

static void kf_add_child(sv_tr_kf* p, unsigned int id) {
    unsigned int lo = 0, hi = p->n_children;
    while (lo < hi) {
        const unsigned int mid = lo + (hi - lo) / 2;
        if (p->children[mid] < id) {
            lo = mid + 1;
        }
        else {
            hi = mid;
        }
    }
    if (lo < p->n_children && p->children[lo] == id) {
        return;
    }
    p->children = (unsigned int*)realloc(p->children, (p->n_children + 1) * sizeof(unsigned int));
    memmove(p->children + lo + 1, p->children + lo, (p->n_children - lo) * sizeof(unsigned int));
    p->children[lo] = id;
    ++p->n_children;
}

static void kf_erase_child(sv_tr_kf* p, unsigned int id) {
    unsigned int i;
    for (i = 0; i < p->n_children; ++i) {
        if (p->children[i] == id) {
            memmove(p->children + i, p->children + i + 1, (p->n_children - i - 1) * sizeof(unsigned int));
            --p->n_children;
            return;
        }
    }
}

/* graph_node::update_connections(min_num_shared_lms) */
static void update_connections(sv_mapping* m, sv_tr_kf* kf) {
    sv_tr_map* map = m->map;
    unsigned int* cnt = (unsigned int*)calloc(map->kf_cap ? map->kf_cap : 1, sizeof(unsigned int));
    unsigned int idx, i, n_entries = 0;
    wid *entries, *pairs;
    unsigned int n_pairs = 0, max_num_shared_lms = 0;
    int nearest = -1;
    sv_rbtree* t;

    for (idx = 0; idx < kf->obs->num_kp; ++idx) {
        const sv_tr_lm* lm = lm_get(m, kf->lm[idx]);
        unsigned int o;
        if (!lm) {
            continue;
        }
        for (o = 0; o < lm->num_obs; ++o) {
            const sv_tr_kf* okf = kf_get(m, (int)lm->obs_kf[o]);
            if (!okf) {
                continue;
            }
            if (okf->parent == SV_TR_NONE && !okf->is_root) {
                continue;
            }
            if (okf->id == kf->id) {
                continue;
            }
            cnt[okf->id]++;
        }
    }
    for (i = 0; i < map->kf_cap; ++i) {
        if (cnt[i]) {
            ++n_entries;
        }
    }
    if (n_entries == 0) {
        free(cnt);
        return;
    }
    entries = (wid*)malloc(n_entries * sizeof(wid));
    pairs = (wid*)malloc((n_entries + 1) * sizeof(wid));
    n_entries = 0;
    for (i = 0; i < map->kf_cap; ++i) { /* ascending id */
        if (!cnt[i]) {
            continue;
        }
        entries[n_entries].id = i;
        entries[n_entries].w = cnt[i];
        ++n_entries;
    }
    for (i = 0; i < n_entries; ++i) {
        /* nearest_covisibility with greatest id_ is selected on ties */
        if (max_num_shared_lms <= entries[i].w) {
            max_num_shared_lms = entries[i].w;
            nearest = (int)entries[i].id;
        }
        if (m->min_num_shared_lms < entries[i].w) {
            pairs[n_pairs++] = entries[i];
        }
    }
    if (n_pairs == 0) {
        pairs[0].w = max_num_shared_lms;
        pairs[0].id = (unsigned int)nearest;
        n_pairs = 1;
    }
    for (i = 0; i < n_pairs; ++i) {
        add_connection(m, pairs[i].id, kf->id, pairs[i].w); /* covisibility->graph_node_->add_connection(owner, n) */
    }
    qsort(pairs, n_pairs, sizeof(wid), cmp_wid_desc);
    set_ordered(kf, pairs, n_pairs);

    /* connected_keyfrms_and_num_shared_lms_ = std::map(keyfrm_to_num_shared_lms.begin(), .end()) */
    t = conn_tree(m, kf->id);
    sv_rb_clear(t);
    for (i = 0; i < n_entries; ++i) {
        sv_rb_insert_at_end(t, entries[i].id, entries[i].w, conn_less, m);
    }

    if (kf->parent == SV_TR_NONE && !kf->is_root) {
        sv_tr_kf* p = kf_get(m, nearest);
        kf->parent = nearest;
        kf_add_child(p, kf->id);
    }
    free(entries);
    free(pairs);
    free(cnt);
}

/* ------------------------------------------------------------------ */
/* landmark / keyframe association primitives                          */
/* ------------------------------------------------------------------ */
static int lm_obs_pos(const sv_tr_lm* lm, unsigned int kf_id, int* found) {
    unsigned int lo = 0, hi = lm->num_obs;
    while (lo < hi) {
        const unsigned int mid = lo + (hi - lo) / 2;
        if (lm->obs_kf[mid] < kf_id) {
            lo = mid + 1;
        }
        else {
            hi = mid;
        }
    }
    *found = (lo < lm->num_obs && lm->obs_kf[lo] == kf_id);
    return (int)lo;
}

static int lm_is_observed_in(const sv_tr_lm* lm, unsigned int kf_id) {
    int found;
    lm_obs_pos(lm, kf_id, &found);
    return found;
}

/* landmark::add_observation (mono: one observation == 1) */
static void lm_add_observation(sv_tr_lm* lm, unsigned int kf_id, unsigned int idx) {
    int found;
    int pos = lm_obs_pos(lm, kf_id, &found);
    if (found) {
        lm->obs_idx[pos] = idx; /* release-build behaviour of observations_[keyfrm] = idx */
        lm->num_obs += 1;       /* num_observations_ += 1 (counter drifts from the map size upstream too) */
        return;
    }
    lm->obs_kf = (unsigned int*)realloc(lm->obs_kf, (lm->num_obs + 1) * sizeof(unsigned int));
    lm->obs_idx = (unsigned int*)realloc(lm->obs_idx, (lm->num_obs + 1) * sizeof(unsigned int));
    memmove(lm->obs_kf + pos + 1, lm->obs_kf + pos, (lm->num_obs - (unsigned int)pos) * sizeof(unsigned int));
    memmove(lm->obs_idx + pos + 1, lm->obs_idx + pos, (lm->num_obs - (unsigned int)pos) * sizeof(unsigned int));
    lm->obs_kf[pos] = kf_id;
    lm->obs_idx[pos] = idx;
    lm->num_obs += 1;
}

/* landmark::connect_to_keyframe */
static void lm_connect(sv_tr_kf* kf, sv_tr_lm* lm, unsigned int idx) {
    kf->lm[idx] = (int)lm->id;
    lm_add_observation(lm, kf->id, idx);
}

/* landmark::prepare_for_erasing */
static void lm_prepare_for_erasing(sv_mapping* m, sv_tr_lm* lm) {
    unsigned int i;
    if (!lm->alive) {
        return;
    }
    for (i = 0; i < lm->num_obs; ++i) {
        sv_tr_kf* k = kf_get(m, (int)lm->obs_kf[i]);
        if (k) {
            k->lm[lm->obs_idx[i]] = SV_TR_NONE;
        }
    }
    free(lm->obs_kf);
    free(lm->obs_idx);
    lm->obs_kf = NULL;
    lm->obs_idx = NULL;
    lm->num_obs = 0;
    lm->alive = 0;
    if (lm->id < m->map->lm_cap && m->map->lms[lm->id] == lm) {
        m->map->lms[lm->id] = NULL;
    }
}

/* landmark::erase_observation (mono) */
static void lm_erase_observation(sv_mapping* m, sv_tr_lm* lm, unsigned int kf_id) {
    int found;
    int pos = lm_obs_pos(lm, kf_id, &found);
    int discard = 0;
    if (!found) {
        return;
    }
    memmove(lm->obs_kf + pos, lm->obs_kf + pos + 1, (lm->num_obs - (unsigned int)pos - 1) * sizeof(unsigned int));
    memmove(lm->obs_idx + pos, lm->obs_idx + pos + 1, (lm->num_obs - (unsigned int)pos - 1) * sizeof(unsigned int));
    lm->num_obs -= 1;
    if (lm->num_obs == 0) {
        discard = 1;
    }
    else if (lm->ref_kf == (int)kf_id) {
        lm->ref_kf = (int)lm->obs_kf[0]; /* observations_.begin()->first */
    }
    if (discard) {
        lm_prepare_for_erasing(m, lm);
    }
}

/* landmark::replace(lm) : `old` is merged into `nw` */
static void lm_replace(sv_mapping* m, sv_tr_lm* old, sv_tr_lm* nw) {
    unsigned int n = old->num_obs, i;
    unsigned int* okf;
    unsigned int* oidx;
    if (nw->id == old->id) {
        return;
    }
    okf = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
    oidx = (unsigned int*)malloc((n ? n : 1) * sizeof(unsigned int));
    memcpy(okf, old->obs_kf, n * sizeof(unsigned int));
    memcpy(oidx, old->obs_idx, n * sizeof(unsigned int));
    lm_prepare_for_erasing(m, old);
    for (i = 0; i < n; ++i) {
        sv_tr_kf* k = kf_get(m, (int)okf[i]);
        if (!lm_is_observed_in(nw, okf[i])) {
            lm_connect(k, nw, oidx[i]);
        }
    }
    nw->num_observed += old->num_observed;
    nw->num_observable += old->num_observable;
    free(okf);
    free(oidx);
}

static void lm_refresh(sv_mapping* m, sv_tr_lm* lm) {
    sv_tr_lm_compute_descriptor(m->map, lm);
    sv_tr_lm_update_mean_normal_and_obs_scale_variance(m->cfg, m->map, lm);
}

/* ------------------------------------------------------------------ */
/* keyframe::prepare_for_erasing                                       */
/* ------------------------------------------------------------------ */
static void recover_spanning_connections(sv_mapping* m, sv_tr_kf* kf, sv_mapping_trace* tr) {
    unsigned int* cand = (unsigned int*)malloc((kf->n_children + 2) * sizeof(unsigned int));
    unsigned int n_cand = 0, i, j, k;
    const int parent = kf->parent;
    sv_tr_kf* pk = kf_get(m, parent);

    cand[n_cand++] = (unsigned int)parent;
    while (kf->n_children > 0) {
        int max_found = 0;
        unsigned int max_num = 0;
        int max_parent = -1, max_child = -1;
        for (i = 0; i < kf->n_children; ++i) {
            sv_tr_kf* ck = kf_get(m, (int)kf->children[i]);
            unsigned int inter_n = 0;
            unsigned int inter[64];
            unsigned int* child_cov;
            unsigned int n_child_cov;
            if (!ck) {
                continue;
            }
            /* extract_intersection(new_parent_candidates, child->get_covisibilities()) */
            child_cov = (unsigned int*)malloc((ck->n_covis ? ck->n_covis : 1) * sizeof(unsigned int));
            n_child_cov = top_n_covisibilities(m, ck, 0xFFFFFFFFu, child_cov);
            for (j = 0; j < n_cand; ++j) {
                for (k = 0; k < n_child_cov; ++k) {
                    if (cand[j] == child_cov[k] && inter_n < 64) {
                        inter[inter_n++] = cand[j];
                    }
                }
            }
            free(child_cov);
            for (j = 0; j < inter_n; ++j) {
                const unsigned int num = conn_weight(m, ck->id, inter[j]);
                if (max_num < num) {
                    max_num = num;
                    max_parent = (int)inter[j];
                    max_child = (int)ck->id;
                    max_found = 1;
                }
                else if (max_found && num == max_num && (int)inter[j] != max_parent && num > 0) {
                    tr->n_span_ties++;
                }
            }
        }
        if (max_found) {
            sv_tr_kf* mc = kf_get(m, max_child);
            sv_tr_kf* mp = kf_get(m, max_parent);
            /* change_spanning_parent */
            mc->parent = max_parent;
            kf_add_child(mp, mc->id);
            kf_erase_child(kf, mc->id);
            cand = (unsigned int*)realloc(cand, (n_cand + 1) * sizeof(unsigned int));
            cand[n_cand++] = mc->id;
        }
        else {
            break;
        }
    }
    /* set my parent as the new parent */
    for (i = 0; i < kf->n_children; ++i) {
        sv_tr_kf* ck = kf_get(m, (int)kf->children[i]);
        if (ck) {
            ck->parent = parent;
            kf_add_child(pk, ck->id);
        }
    }
    kf->n_children = 0;
    kf_erase_child(pk, kf->id);
    free(cand);
}

/* returns 1 iff the keyframe was erased */
static int kf_prepare_for_erasing(sv_mapping* m, sv_tr_kf* kf, sv_mapping_trace* tr) {
    unsigned int idx;
    if (kf->is_root) {
        return 0; /* "cannot erase the root node" */
    }
    if (m->is_protected && m->is_protected(m->hook_user, kf->id)) {
        return 0; /* cannot_be_erased_ */
    }
    for (idx = 0; idx < kf->obs->num_kp; ++idx) {
        sv_tr_lm* lm = lm_get(m, kf->lm[idx]);
        if (!lm) {
            continue;
        }
        lm_erase_observation(m, lm, kf->id);
        if (lm->alive) {
            lm_refresh(m, lm);
        }
    }
    erase_all_connections(m, kf);
    recover_spanning_connections(m, kf, tr);
    if (m->on_erase) {
        m->on_erase(m->hook_user, kf->id, kf->parent);
    }
    m->map->kfs[kf->id] = NULL;
    kf->alive = 0;
    return 1;
}

/* ------------------------------------------------------------------ */
/* local_map_cleaner                                                   */
/* ------------------------------------------------------------------ */
static void fresh_push(sv_mapping* m, unsigned int lm_id) {
    if (m->n_fresh == m->cap_fresh) {
        m->cap_fresh = m->cap_fresh ? m->cap_fresh * 2 : 1024;
        m->fresh = (unsigned int*)realloc(m->fresh, m->cap_fresh * sizeof(unsigned int));
    }
    m->fresh[m->n_fresh++] = lm_id;
}

static void trace_push_id(unsigned int** arr, unsigned int* n, unsigned int v) {
    *arr = (unsigned int*)realloc(*arr, (*n + 1) * sizeof(unsigned int));
    (*arr)[(*n)++] = v;
}

static void remove_invalid_landmarks(sv_mapping* m, unsigned int cur_id, sv_mapping_trace* tr) {
    unsigned int i = 0;
    while (i < m->n_fresh) {
        sv_tr_lm* lm = lm_get(m, (int)m->fresh[i]);
        enum { Valid, Invalid, NotClear } state = NotClear;
        if (!lm) {
            state = Valid; /* will_be_erased() */
        }
        else if ((double)((float)lm->num_observed / (float)lm->num_observable) < m->observed_ratio_thr) {
            state = Invalid;
        }
        else if (m->num_reliable_keyfrms + (m->fresh[i] < m->lm_first_cap ? m->lm_first_kf[m->fresh[i]] : 0u) < cur_id) {
            state = Valid;
        }
        if (state == Valid) {
            memmove(m->fresh + i, m->fresh + i + 1, (m->n_fresh - i - 1) * sizeof(unsigned int));
            --m->n_fresh;
        }
        else if (state == Invalid) {
            trace_push_id(&tr->culled_lms, &tr->n_culled_lms, lm->id);
            lm_prepare_for_erasing(m, lm);
            memmove(m->fresh + i, m->fresh + i + 1, (m->n_fresh - i - 1) * sizeof(unsigned int));
            --m->n_fresh;
        }
        else {
            ++i;
        }
    }
}

static void count_redundant_observations(sv_mapping* m, const sv_tr_kf* kf, unsigned int* num_valid, unsigned int* num_redundant) {
    const unsigned int num_better_obs_thr = 3;
    unsigned int idx;
    *num_valid = 0;
    *num_redundant = 0;
    for (idx = 0; idx < kf->obs->num_kp; ++idx) {
        const sv_tr_lm* lm = lm_get(m, kf->lm[idx]);
        int scale_level, obs_by_keyfrm_is_redundant = 0;
        unsigned int num_better_obs = 0, o;
        if (!lm) {
            continue;
        }
        ++*num_valid;
        if (lm->num_obs <= num_better_obs_thr) {
            continue;
        }
        scale_level = kf->obs->kp[idx].octave;
        for (o = 0; o < lm->num_obs; ++o) {
            const sv_tr_kf* ngh = kf_get(m, (int)lm->obs_kf[o]);
            int ngh_scale_level;
            if (ngh->id == kf->id) {
                continue;
            }
            ngh_scale_level = ngh->obs->kp[lm->obs_idx[o]].octave;
            if (ngh_scale_level <= scale_level + 1) {
                ++num_better_obs;
                if (num_better_obs_thr <= num_better_obs) {
                    obs_by_keyfrm_is_redundant = 1;
                    break;
                }
            }
        }
        if (obs_by_keyfrm_is_redundant) {
            ++*num_redundant;
        }
    }
}

static void remove_redundant_keyframes(sv_mapping* m, sv_tr_kf* cur, sv_mapping_trace* tr) {
    const unsigned int window_size_not_to_remove = 2;
    unsigned int n, i;
    unsigned int* cov;
    if (m->redundant_obs_ratio_thr < 0.0 || m->top_n_covis_to_search == 0) {
        return;
    }
    cov = (unsigned int*)malloc((cur->n_covis ? cur->n_covis : 1) * sizeof(unsigned int));
    n = top_n_covisibilities(m, cur, m->top_n_covis_to_search, cov);
    for (i = 0; i < n; ++i) {
        sv_tr_kf* c = kf_get(m, (int)cov[i]);
        unsigned int num_valid, num_redundant;
        if (!c) {
            continue;
        }
        if (c->is_root) {
            continue;
        }
        if (c->id <= cur->id && cur->id <= c->id + window_size_not_to_remove) {
            continue;
        }
        count_redundant_observations(m, c, &num_valid, &num_redundant);
        if (m->redundant_obs_ratio_thr <= (double)((float)num_redundant / (float)num_valid)) {
            trace_push_id(&tr->culled_kfs, &tr->n_culled_kfs, c->id);
            kf_prepare_for_erasing(m, c, tr);
        }
    }
    free(cov);
}

/* ------------------------------------------------------------------ */
/* landmark generation                                                 */
/* ------------------------------------------------------------------ */
static sv_tr_lm* new_lm_record(sv_mapping* m, unsigned int id) {
    sv_tr_lm* rec;
    if (m->alloc_lm) {
        rec = m->alloc_lm(m->alloc_user, id);
    }
    else {
        sv_tr_map* map = m->map;
        if (id >= map->lm_cap) {
            unsigned int nc = id + 4096, i;
            map->lms = (sv_tr_lm**)realloc(map->lms, nc * sizeof(sv_tr_lm*));
            for (i = map->lm_cap; i < nc; ++i) {
                map->lms[i] = NULL;
            }
            map->lm_cap = nc;
        }
        rec = (sv_tr_lm*)calloc(1, sizeof(sv_tr_lm));
    }
    free(rec->obs_kf);
    free(rec->obs_idx);
    memset(rec, 0, sizeof(*rec));
    return rec;
}

/* module::two_view_triangulator (mono) */
typedef struct two_view {
    const sv_tr_kf *k1, *k2;
    double rot_1w[9], rot_w1[9], trans_1w[3], center_1[3];
    double rot_2w[9], rot_w2[9], trans_2w[3], center_2[3];
    float ratio_factor, cos_rays_parallax_thr;
} two_view;

static void pose_split(const double pose[16], double rot[9], double t[3]) {
    int r, c;
    for (c = 0; c < 3; ++c) {
        for (r = 0; r < 3; ++r) {
            rot[c * 3 + r] = pose[c * 4 + r];
        }
    }
    t[0] = pose[12];
    t[1] = pose[13];
    t[2] = pose[14];
}

static void two_view_init(two_view* tv, const sv_tr_config* cfg, const sv_tr_kf* k1, const sv_tr_kf* k2, float rays_parallax_deg_thr) {
    tv->k1 = k1;
    tv->k2 = k2;
    pose_split(k1->pose_cw, tv->rot_1w, tv->trans_1w);
    sv_mat3_transpose(tv->rot_1w, tv->rot_w1);
    memcpy(tv->center_1, k1->trans_wc, sizeof(tv->center_1));
    pose_split(k2->pose_cw, tv->rot_2w, tv->trans_2w);
    sv_mat3_transpose(tv->rot_2w, tv->rot_w2);
    memcpy(tv->center_2, k2->trans_wc, sizeof(tv->center_2));
    /* 2.0f * std::max(scale_factor_1, scale_factor_2) */
    tv->ratio_factor = 2.0f * (cfg->scale_factor > cfg->scale_factor ? cfg->scale_factor : cfg->scale_factor);
    /* std::cos(rays_parallax_deg_thr * M_PI / 180.0) stored as float */
    tv->cos_rays_parallax_thr = (float)cos(((double)rays_parallax_deg_thr * 3.14159265358979323846) / 180.0);
}

/* check_depth_is_positive: rot_cw.block<1, 3>(2, 0).dot(pos_w) + trans_cw(2) */
static int check_depth_is_positive(const double pos_w[3], const double rot_cw[9], const double trans_cw[3]) {
    const double pos_z = (rot_cw[0 * 3 + 2] * pos_w[0] + (rot_cw[1 * 3 + 2] * pos_w[1] + rot_cw[2 * 3 + 2] * pos_w[2])) + trans_cw[2];
    return 0 < pos_z;
}

static int check_reprojection_error(const sv_tr_config* cfg, const double pos_w[3], const double rot_cw[9], const double trans_cw[3],
                                    const sv_keypoint* kp, float sigma_sq) {
    const float chi_sq_2D = 5.99146f;
    double reproj[2], ex, ey, sq;
    float x_right;
    sv_tr_reproject_to_image(cfg, rot_cw, trans_cw, pos_w, reproj, &x_right);
    ex = reproj[0] - (double)kp->x;
    ey = reproj[1] - (double)kp->y;
    sq = ex * ex + ey * ey;
    if ((double)(chi_sq_2D * sigma_sq) < sq) {
        return 0;
    }
    return 1;
}

static int check_scale_factors(const two_view* tv, const double pos_w[3], float sf1, float sf2) {
    double v1[3], v2[3], d1, d2, ratio_dists;
    float ratio_octave;
    v1[0] = pos_w[0] - tv->center_1[0];
    v1[1] = pos_w[1] - tv->center_1[1];
    v1[2] = pos_w[2] - tv->center_1[2];
    d1 = sv_vec3_norm(v1);
    v2[0] = pos_w[0] - tv->center_2[0];
    v2[1] = pos_w[1] - tv->center_2[1];
    v2[2] = pos_w[2] - tv->center_2[2];
    d2 = sv_vec3_norm(v2);
    if (d1 == 0 || d2 == 0) {
        return 0;
    }
    ratio_dists = d2 / d1;
    ratio_octave = sf1 / sf2;
    return (double)ratio_octave / ratio_dists < (double)tv->ratio_factor && ratio_dists / (double)ratio_octave < (double)tv->ratio_factor;
}

static int two_view_triangulate(sv_mapping* m, const two_view* tv, unsigned int idx_1, unsigned int idx_2, double pos_w[3]) {
    const sv_tr_config* cfg = m->cfg;
    const sv_keypoint* kp1 = &tv->k1->obs->kp[idx_1];
    const sv_keypoint* kp2 = &tv->k2->obs->kp[idx_2];
    const double* ray_c_1 = tv->k1->obs->bearings + 3 * (size_t)idx_1;
    const double* ray_c_2 = tv->k2->obs->bearings + 3 * (size_t)idx_2;
    double ray_w_1[3], ray_w_2[3], cos_rays_parallax;
    sv_mat3_mulv(tv->rot_w1, ray_c_1, ray_w_1);
    sv_mat3_mulv(tv->rot_w2, ray_c_2, ray_w_2);
    cos_rays_parallax = sv_vec3_dot(ray_w_1, ray_w_2);
    if (!(0.0 < cos_rays_parallax && cos_rays_parallax < (double)tv->cos_rays_parallax_thr)) {
        return 0; /* monocular: no stereo fallbacks */
    }
    sv_map_triangulate_poses(ray_c_1, ray_c_2, tv->k1->pose_cw, tv->k2->pose_cw, pos_w);
    if (!check_depth_is_positive(pos_w, tv->rot_1w, tv->trans_1w) || !check_depth_is_positive(pos_w, tv->rot_2w, tv->trans_2w)) {
        return 0;
    }
    if (!check_reprojection_error(cfg, pos_w, tv->rot_1w, tv->trans_1w, kp1, m->level_sigma_sq[kp1->octave])
        || !check_reprojection_error(cfg, pos_w, tv->rot_2w, tv->trans_2w, kp2, m->level_sigma_sq[kp2->octave])) {
        return 0;
    }
    if (!check_scale_factors(tv, pos_w, cfg->scale_factors[kp1->octave], cfg->scale_factors[kp2->octave])) {
        return 0;
    }
    return 1;
}

static void triangulate_with_two_keyframes(sv_mapping* m, sv_tr_kf* k1, sv_tr_kf* k2, const unsigned int (*matches)[2],
                                           unsigned int n_matches, sv_mapping_tri* trace) {
    two_view tv;
    unsigned int i;
    two_view_init(&tv, m->cfg, k1, k2, 1.0f);
    for (i = 0; i < n_matches; ++i) {
        const unsigned int idx_1 = matches[i][0], idx_2 = matches[i][1];
        double pos_w[3];
        sv_tr_lm* lm;
        if (!two_view_triangulate(m, &tv, idx_1, idx_2, pos_w)) {
            continue;
        }
        lm = new_lm_record(m, m->next_landmark_id);
        lm->id = m->next_landmark_id++;
        lm->alive = 1;
        lm->pos_w[0] = pos_w[0];
        lm->pos_w[1] = pos_w[1];
        lm->pos_w[2] = pos_w[2];
        lm->ref_kf = (int)k1->id;
        lm->num_observed = 1;
        lm->num_observable = 1;
        sv_mapping_set_lm_first_kf(m, lm->id, k1->id);
        lm_connect(k1, lm, idx_1);
        lm_connect(k2, lm, idx_2);
        lm_refresh(m, lm);
        m->map->lms[lm->id] = lm; /* map_db_->add_landmark */
        fresh_push(m, lm->id);
        trace->acc_ids = (unsigned int*)realloc(trace->acc_ids, (trace->n_acc + 1) * sizeof(unsigned int));
        trace->acc_pos = (double(*)[3])realloc(trace->acc_pos, (trace->n_acc + 1) * sizeof(double[3]));
        trace->acc_ids[trace->n_acc] = lm->id;
        memcpy(trace->acc_pos[trace->n_acc], pos_w, sizeof(double[3]));
        ++trace->n_acc;
    }
}

static void create_new_landmarks(sv_mapping* m, sv_tr_kf* cur, sv_mapping_trace* tr) {
    unsigned int n, i;
    unsigned int* cov = (unsigned int*)malloc((cur->n_covis ? cur->n_covis : 1) * sizeof(unsigned int));
    double cur_cam_center[3], cur_rot[9], cur_trans[3];
    n = top_n_covisibilities(m, cur, m->num_cov_gen, cov);
    memcpy(cur_cam_center, cur->trans_wc, sizeof(cur_cam_center));
    pose_split(cur->pose_cw, cur_rot, cur_trans);
    sv_tr_obs_ensure_bearings(cur->obs, m->cfg);
    for (i = 0; i < n; ++i) {
        sv_tr_kf* ngh = kf_get(m, (int)cov[i]);
        double ngh_center[3], baseline_vec[3], baseline_dist, ngh_rot[9], ngh_trans[3], E[9];
        float median_scale_in_ngh;
        unsigned int (*matches)[2] = NULL;
        unsigned int n_matches;
        sv_mapping_tri* t;
        if (!ngh) {
            tr->n_stale_neighbor++;
            continue;
        }
        memcpy(ngh_center, ngh->trans_wc, sizeof(ngh_center));
        baseline_vec[0] = ngh_center[0] - cur_cam_center[0];
        baseline_vec[1] = ngh_center[1] - cur_cam_center[1];
        baseline_vec[2] = ngh_center[2] - cur_cam_center[2];
        baseline_dist = sv_vec3_norm(baseline_vec);
        median_scale_in_ngh = sv_map_kf_median_depth(m->map, ngh, 1);
        if (baseline_dist < m->baseline_dist_thr_ratio * (double)median_scale_in_ngh) {
            continue;
        }
        pose_split(ngh->pose_cw, ngh_rot, ngh_trans);
        sv_map_create_E_21(ngh_rot, ngh_trans, cur_rot, cur_trans, E);
        n_matches = sv_map_match_for_triangulation(m->cfg, cur, ngh, E, m->residual_rad_thr, &matches);

        if (tr->n_tri == tr->cap_tri) {
            tr->cap_tri = tr->cap_tri ? tr->cap_tri * 2 : 32;
            tr->tri = (sv_mapping_tri*)realloc(tr->tri, tr->cap_tri * sizeof(sv_mapping_tri));
        }
        t = &tr->tri[tr->n_tri++];
        memset(t, 0, sizeof(*t));
        t->ngh_id = ngh->id;
        t->n_matches = n_matches;
        t->matches = matches;
        triangulate_with_two_keyframes(m, cur, ngh, (const unsigned int(*)[2])matches, n_matches, t);
    }
    free(cov);
}

/* ------------------------------------------------------------------ */
/* landmark fusion                                                     */
/* ------------------------------------------------------------------ */
typedef struct dup_pair {
    sv_tr_lm *key, *val;
} dup_pair;

typedef struct new_conn {
    unsigned int idx;
    sv_tr_lm* lm;
} new_conn;

typedef struct repl_pair {
    sv_tr_lm *from, *to;
} repl_pair;

static int cmp_dup(const void* a, const void* b) {
    const unsigned int ia = ((const dup_pair*)a)->key->id, ib = ((const dup_pair*)b)->key->id;
    return (ia > ib) - (ia < ib);
}

/* landmark::predict_scale_level(float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) */
static unsigned int predict_scale_level(const sv_tr_lm* lm, float cam_to_lm_dist, float num_scale_levels, float log_scale_factor) {
    const float ratio = lm->max_valid_dist / cam_to_lm_dist;
    const int pred_scale_level = (int)ceilf(logf(ratio) / log_scale_factor);
    if (pred_scale_level < 0) {
        return 0;
    }
    else if (num_scale_levels <= (float)(unsigned int)pred_scale_level) {
        return (unsigned int)(num_scale_levels - 1);
    }
    return (unsigned int)pred_scale_level;
}

/* match::fuse::detect_duplication(keyfrm, rot_cw, trans_cw, landmarks_to_check, margin, ..., do_reprojection_matching = true) */
static void detect_duplication(sv_mapping* m, sv_tr_kf* kf, const double rot_cw[9], const double trans_cw[3],
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
            const sv_keypoint* kp = &kf->obs->kp[idx];
            unsigned int hamm_dist;
            double ex, ey, reproj_error_sq;
            const float chi_sq_2D = 5.99146f;
            if (already[idx]) {
                continue;
            }
            ex = reproj[0] - (double)kp->x;
            ey = reproj[1] - (double)kp->y;
            reproj_error_sq = ex * ex + ey * ey;
            if ((double)chi_sq_2D < reproj_error_sq * (double)cfg->inv_level_sigma_sq[kp->octave]) {
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

static int cmp_newconn(const void* a, const void* b) {
    const unsigned int ia = ((const new_conn*)a)->idx, ib = ((const new_conn*)b)->idx;
    return (ia > ib) - (ia < ib);
}

static sv_tr_lm* resolve_replaced(const repl_pair* rp, unsigned int n, sv_tr_lm* lm) {
    unsigned int i;
    int again = 1;
    while (again) {
        again = 0;
        for (i = 0; i < n; ++i) {
            if (rp[i].from == lm) {
                lm = rp[i].to;
                again = 1;
                break;
            }
        }
    }
    return lm;
}

static int replaced_has(const repl_pair* rp, unsigned int n, const sv_tr_lm* lm) {
    unsigned int i;
    for (i = 0; i < n; ++i) {
        if (rp[i].from == lm) {
            return 1;
        }
    }
    return 0;
}

/* applies the results of one detect_duplication() call to `kf` */
static void apply_fuse(sv_mapping* m, sv_tr_kf* kf, dup_pair* dups, unsigned int n_dups, new_conn* ncs, unsigned int n_ncs,
                       repl_pair** replaced, unsigned int* n_replaced, sv_mapping_trace* tr) {
    unsigned int i;
    for (i = 0; i < n_dups; ++i) {
        sv_tr_lm* lm_to_replace = dups[i].key;
        sv_tr_lm* lm_in_keyfrm = dups[i].val;
        if (lm_to_replace->num_obs < lm_in_keyfrm->num_obs) {
            sv_tr_lm* tmp = lm_to_replace;
            lm_to_replace = lm_in_keyfrm;
            lm_in_keyfrm = tmp;
        }
        if (lm_to_replace->id != lm_in_keyfrm->id) {
            *replaced = (repl_pair*)realloc(*replaced, (*n_replaced + 1) * sizeof(repl_pair));
            (*replaced)[*n_replaced].from = lm_in_keyfrm;
            (*replaced)[*n_replaced].to = lm_to_replace;
            ++*n_replaced;
            lm_replace(m, lm_in_keyfrm, lm_to_replace);
            if (lm_to_replace->alive) {
                lm_refresh(m, lm_to_replace);
            }
        }
    }
    qsort(ncs, n_ncs, sizeof(new_conn), cmp_newconn);
    for (i = 0; i < n_ncs; ++i) {
        sv_tr_lm* lm = resolve_replaced(*replaced, *n_replaced, ncs[i].lm);
        if (!lm->alive) {
            tr->n_dup_connect++;
            continue;
        }
        if (lm_is_observed_in(lm, kf->id)) {
            tr->n_dup_connect++;
        }
        lm_connect(kf, lm, ncs[i].idx);
        lm_refresh(m, lm);
    }
    (void)replaced_has;
}

static void fuse_landmark_duplication(sv_mapping* m, sv_tr_kf* cur, const unsigned int* tgt, unsigned int n_tgt, sv_mapping_trace* tr) {
    repl_pair* replaced = NULL;
    unsigned int n_replaced = 0, i, k;
    sv_tr_map* map = m->map;
    double rot_cw[9], trans_cw[3];

    /* reproject the landmarks observed in the current keyframe to each of the targets */
    {
        sv_tr_lm** cur_landmarks = (sv_tr_lm**)malloc((cur->obs->num_kp ? cur->obs->num_kp : 1) * sizeof(sv_tr_lm*));
        for (k = 0; k < cur->obs->num_kp; ++k) {
            cur_landmarks[k] = cur->lm[k] >= 0 ? map->lms[cur->lm[k]] : NULL;
        }
        for (i = 0; i < n_tgt; ++i) {
            sv_tr_kf* tk = kf_get(m, (int)tgt[i]);
            dup_pair* dups;
            new_conn* ncs;
            unsigned int n_dups, n_ncs;
            if (!tk) {
                tr->n_stale_neighbor++;
                continue;
            }
            pose_split(tk->pose_cw, rot_cw, trans_cw);
            detect_duplication(m, tk, rot_cw, trans_cw, cur_landmarks, cur->obs->num_kp, 3.0f, &dups, &n_dups, &ncs, &n_ncs);
            apply_fuse(m, tk, dups, n_dups, ncs, n_ncs, &replaced, &n_replaced, tr);
            free(dups);
            free(ncs);
        }
        free(cur_landmarks);
    }

    /* reproject the landmarks observed in each of the targets to the current keyframe */
    {
        unsigned char* in_set = (unsigned char*)calloc(map->lm_cap ? map->lm_cap : 1, 1);
        sv_tr_lm** cand;
        unsigned int n_cand = 0;
        dup_pair* dups;
        new_conn* ncs;
        unsigned int n_dups, n_ncs;
        for (i = 0; i < n_tgt; ++i) {
            const sv_tr_kf* tk = kf_get(m, (int)tgt[i]);
            if (!tk) {
                continue;
            }
            for (k = 0; k < tk->obs->num_kp; ++k) {
                if (tk->lm[k] >= 0 && map->lms[tk->lm[k]] && map->lms[tk->lm[k]]->alive) {
                    in_set[tk->lm[k]] = 1;
                }
            }
        }
        for (k = 0; k < map->lm_cap; ++k) {
            n_cand += in_set[k];
        }
        cand = (sv_tr_lm**)malloc((n_cand ? n_cand : 1) * sizeof(sv_tr_lm*));
        n_cand = 0;
        for (k = 0; k < map->lm_cap; ++k) { /* std::set ordered by id */
            if (in_set[k]) {
                cand[n_cand++] = map->lms[k];
            }
        }
        pose_split(cur->pose_cw, rot_cw, trans_cw);
        detect_duplication(m, cur, rot_cw, trans_cw, cand, n_cand, 3.0f, &dups, &n_dups, &ncs, &n_ncs);
        apply_fuse(m, cur, dups, n_dups, ncs, n_ncs, &replaced, &n_replaced, tr);
        free(dups);
        free(ncs);
        free(cand);
        free(in_set);
    }

    /* trace: replaced_lms is an id-ordered map keyed by the replaced-away landmark */
    tr->replaced = (unsigned int(*)[2])malloc((n_replaced ? n_replaced : 1) * sizeof(unsigned int[2]));
    tr->n_replaced = n_replaced;
    for (i = 0; i < n_replaced; ++i) {
        tr->replaced[i][0] = replaced[i].from->id;
        tr->replaced[i][1] = replaced[i].to->id;
    }
    {
        /* insertion sort by key id */
        unsigned int a, b;
        for (a = 1; a < n_replaced; ++a) {
            unsigned int key[2];
            key[0] = tr->replaced[a][0];
            key[1] = tr->replaced[a][1];
            b = a;
            while (b > 0 && tr->replaced[b - 1][0] > key[0]) {
                tr->replaced[b][0] = tr->replaced[b - 1][0];
                tr->replaced[b][1] = tr->replaced[b - 1][1];
                --b;
            }
            tr->replaced[b][0] = key[0];
            tr->replaced[b][1] = key[1];
        }
    }
    free(replaced);
}

static void update_new_keyframe(sv_mapping* m, sv_tr_kf* cur, sv_mapping_trace* tr) {
    unsigned int* tgt = (unsigned int*)malloc((cur->n_covis ? cur->n_covis : 1) * sizeof(unsigned int));
    unsigned int n = top_n_covisibilities(m, cur, m->num_cov_fuse, tgt);
    fuse_landmark_duplication(m, cur, tgt, n, tr);
    free(tgt);
    /* the geometry refresh loop over cur's landmarks is a no-op here: every landmark that
     * lost its descriptor / prediction parameters was refreshed at the call site */
    update_connections(m, cur);
}

/* ------------------------------------------------------------------ */
/* local bundle adjustment glue                                        */
/* ------------------------------------------------------------------ */
static int run_local_ba(sv_mapping* m, sv_tr_kf* cur) {
    sv_tr_map* map = m->map;
    const sv_tr_config* cfg = m->cfg;
    sv_bav_view view;
    sv_bav_kf* vk;
    sv_bav_lm* vl;
    unsigned int nk = 0, nl = 0, i, k;
    sv_bav_result res;
    int flag = 0;

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
                vk[nk].pose_cw[r * 4 + c] = kf->pose_cw[c * 4 + r]; /* row-major */
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

    {
        unsigned int* covis = (unsigned int*)malloc((cur->n_covis ? cur->n_covis : 1) * sizeof(unsigned int));
        const unsigned int n_covis = top_n_covisibilities(m, cur, 0xFFFFFFFFu, covis); /* get_covisibilities() */
        sv_bav_local(&view, cur->id, covis, (int)n_covis, m->ba_first_iter, m->ba_second_iter, 0, &flag, &res);
        free(covis);
    }

    /* 8. Update the information */
    for (i = 0; i < (unsigned int)res.n_outliers; ++i) {
        sv_tr_kf* kf = kf_get(m, (int)res.outliers[i][0]);
        sv_tr_lm* lm = lm_get(m, (int)res.outliers[i][1]);
        int found, pos;
        if (!kf || !lm) {
            continue;
        }
        pos = lm_obs_pos(lm, kf->id, &found);
        if (found) {
            kf->lm[lm->obs_idx[pos]] = SV_TR_NONE; /* keyfrm->erase_landmark(lm) */
        }
        lm_erase_observation(m, lm, kf->id);
        if (lm->alive) {
            lm_refresh(m, lm);
        }
    }
    for (i = 0; i < (unsigned int)res.n_applied; ++i) {
        sv_tr_kf* kf = kf_get(m, (int)res.applied_kf[i]);
        double pose_cm[16];
        int r, c;
        for (r = 0; r < 4; ++r) {
            for (c = 0; c < 4; ++c) {
                pose_cm[c * 4 + r] = res.applied_pose[i][r * 4 + c];
            }
        }
        sv_tr_kf_set_pose_cw(kf, pose_cm);
    }
    for (i = 0; i < (unsigned int)res.g.nv; ++i) {
        const sv_ba_vertex* v = &res.g.v[i];
        sv_tr_lm* lm;
        if (!v->is_landmark) {
            continue;
        }
        lm = lm_get(m, (int)res.vtx_owner[i]);
        if (!lm) {
            continue;
        }
        lm->pos_w[0] = v->pos[0];
        lm->pos_w[1] = v->pos[1];
        lm->pos_w[2] = v->pos[2];
        sv_tr_lm_update_mean_normal_and_obs_scale_variance(cfg, map, lm);
    }
    sv_bav_result_free(&res);
    for (i = 0; i < nk; ++i) {
        free((void*)vk[i].slots);
        free((void*)vk[i].kps);
    }
    free(vk);
    free(vl);
    return 0;
}

/* ------------------------------------------------------------------ */
/* mapping_module::mapping_with_new_keyframe                           */
/* ------------------------------------------------------------------ */
int sv_mapping_step(sv_mapping* m, unsigned int cur_id, sv_mapping_trace* tr) {
    sv_tr_kf* cur = kf_get(m, (int)cur_id);
    unsigned int idx;
    if (!cur) {
        return -1;
    }
    memset(tr, 0, sizeof(*tr));
    tr->cur_id = cur_id;

    /* store_new_keyframe */
    sv_tr_obs_ensure_bow(cur->obs, m->cfg);
    for (idx = 0; idx < cur->obs->num_kp; ++idx) {
        if (cur->lm[idx] >= 0 && lm_get(m, cur->lm[idx])) {
            fresh_push(m, (unsigned int)cur->lm[idx]);
        }
    }
    update_connections(m, cur);
    m->map->num_keyframes = count_alive_keyframes(m);

    remove_invalid_landmarks(m, cur_id, tr);
    create_new_landmarks(m, cur, tr);
    update_new_keyframe(m, cur, tr);

    if (2 < count_alive_keyframes(m)) {
        tr->local_ba_invoked = 1;
        run_local_ba(m, cur);
    }
    remove_redundant_keyframes(m, cur, tr);
    m->map->num_keyframes = count_alive_keyframes(m);
    return 0;
}
