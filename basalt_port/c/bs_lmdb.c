/* SPDX-License-Identifier: BSD-3-Clause
 * LandmarkDatabase<float>, see bs_lmdb.h. Every function follows landmark_database.cpp statement by statement. */
#include <stdlib.h>
#include <string.h>
#include "bs_lmdb.h"

/* the C++ removeLandmarkHelper dereferences observations.find(host) == end() when the host entry was already erased by the removal of this
 * landmark's last observation (undefined behaviour, a crash in practice): never reached in the reference runs. The port counts it and survives. */
int bs_lmdb_ub = 0;

int bs_tcid_cmp(bs_tcid a, bs_tcid b) {
    if (a.frame_id == b.frame_id) return a.cam_id < b.cam_id ? -1 : (a.cam_id > b.cam_id ? 1 : 0);
    return a.frame_id < b.frame_id ? -1 : 1;
}

/* ---- sorted dynamic arrays (std::map / std::set replacements) */
static int ids_find(const int64_t* a, int n, int64_t k) {   /* lower bound index */
    int lo = 0, hi = n;
    while (lo < hi) { int m = (lo + hi) / 2; if (a[m] < k) lo = m + 1; else hi = m; }
    return lo;
}
static int obs_lower(const bs_obs* a, int n, bs_tcid k) {
    int lo = 0, hi = n;
    while (lo < hi) { int m = (lo + hi) / 2; if (bs_tcid_cmp(a[m].t, k) < 0) lo = m + 1; else hi = m; }
    return lo;
}
static int tgt_lower(const bs_tgt* a, int n, bs_tcid k) {
    int lo = 0, hi = n;
    while (lo < hi) { int m = (lo + hi) / 2; if (bs_tcid_cmp(a[m].t, k) < 0) lo = m + 1; else hi = m; }
    return lo;
}

static void free_keypoint(void* p) { bs_keypoint* k = (bs_keypoint*)p; free(k->obs); free(k); }
static void free_host(void* p) {
    bs_host* h = (bs_host*)p;
    for (int i = 0; i < h->n; i++) free(h->tgt[i].ids);
    free(h->tgt);
    free(h);
}

void bs_lmdb_init(bs_lmdb* db) {
    bs_htab_init(&db->kpts, BS_HK_U64);
    bs_htab_init(&db->observations, BS_HK_TCID);
}
void bs_lmdb_destroy(bs_lmdb* db) {
    bs_htab_destroy(&db->kpts, free_keypoint);
    bs_htab_destroy(&db->observations, free_host);
}

bs_keypoint* bs_lmdb_get_landmark(const bs_lmdb* db, int64_t id) {
    bs_hnode* n = bs_htab_find(&db->kpts, id, 0);
    return n ? (bs_keypoint*)n->val : NULL;
}
int bs_lmdb_landmark_exists(const bs_lmdb* db, int64_t id) { return bs_htab_find(&db->kpts, id, 0) != NULL; }
size_t bs_lmdb_num_landmarks(const bs_lmdb* db) { return db->kpts.nelem; }
const bs_host* bs_lmdb_host(const bs_lmdb* db, bs_tcid h) {
    bs_hnode* n = bs_htab_find(&db->observations, h.frame_id, (int64_t)h.cam_id);
    return n ? (const bs_host*)n->val : NULL;
}
int bs_lmdb_num_observations(const bs_lmdb* db) {
    int total = 0;
    for (bs_hnode* n = db->observations.before_begin.next; n; n = n->next) {
        const bs_host* h = (const bs_host*)n->val;
        for (int i = 0; i < h->n; i++) total += h->tgt[i].n;
    }
    return total;
}
int bs_lmdb_num_observations_lm(const bs_lmdb* db, int64_t id) { return bs_lmdb_get_landmark(db, id)->nobs; }

void bs_lmdb_add_landmark(bs_lmdb* db, int64_t id, const float dir[2], float inv_dist, bs_tcid host) {
    int ins;
    bs_hnode* n = bs_htab_insert(&db->kpts, id, 0, &ins);
    bs_keypoint* k;
    if (ins) {
        k = (bs_keypoint*)calloc(1, sizeof(bs_keypoint));
        k->id = id;
        n->val = k;
    } else {
        k = (bs_keypoint*)n->val;
    }
    k->direction[0] = dir[0]; k->direction[1] = dir[1];
    k->inv_dist = inv_dist;
    k->host = host;
}

int bs_lmdb_add_observation(bs_lmdb* db, bs_tcid target, int64_t kpt_id, const float pos[2]) {
    bs_keypoint* k = bs_lmdb_get_landmark(db, kpt_id);
    if (!k) return 0;
    /* it->second.obs[tcid_target] = o.pos */
    int i = obs_lower(k->obs, k->nobs, target);
    if (i < k->nobs && bs_tcid_cmp(k->obs[i].t, target) == 0) {
        k->obs[i].pos[0] = pos[0]; k->obs[i].pos[1] = pos[1];
    } else {
        if (k->nobs == k->cap) { k->cap = k->cap ? 2 * k->cap : 8; k->obs = (bs_obs*)realloc(k->obs, sizeof(bs_obs) * k->cap); }
        memmove(k->obs + i + 1, k->obs + i, sizeof(bs_obs) * (k->nobs - i));
        k->obs[i].t = target; k->obs[i].pos[0] = pos[0]; k->obs[i].pos[1] = pos[1];
        k->nobs++;
    }
    /* observations[host_kf_id][tcid_target].insert(kpt id) */
    int ins;
    bs_hnode* hn = bs_htab_insert(&db->observations, k->host.frame_id, (int64_t)k->host.cam_id, &ins);
    bs_host* h;
    if (ins) { h = (bs_host*)calloc(1, sizeof(bs_host)); hn->val = h; } else h = (bs_host*)hn->val;
    int ti = tgt_lower(h->tgt, h->n, target);
    if (!(ti < h->n && bs_tcid_cmp(h->tgt[ti].t, target) == 0)) {
        if (h->n == h->cap) { h->cap = h->cap ? 2 * h->cap : 4; h->tgt = (bs_tgt*)realloc(h->tgt, sizeof(bs_tgt) * h->cap); }
        memmove(h->tgt + ti + 1, h->tgt + ti, sizeof(bs_tgt) * (h->n - ti));
        memset(&h->tgt[ti], 0, sizeof(bs_tgt));
        h->tgt[ti].t = target;
        h->n++;
    }
    bs_tgt* tg = &h->tgt[ti];
    int ii = ids_find(tg->ids, tg->n, k->id);
    if (!(ii < tg->n && tg->ids[ii] == k->id)) {
        if (tg->n == tg->cap) { tg->cap = tg->cap ? 2 * tg->cap : 8; tg->ids = (int64_t*)realloc(tg->ids, sizeof(int64_t) * tg->cap); }
        memmove(tg->ids + ii + 1, tg->ids + ii, sizeof(int64_t) * (tg->n - ii));
        tg->ids[ii] = k->id;
        tg->n++;
    }
    return 1;
}

/* removeLandmarkObservationHelper(it, it2): erase the observation (target tcid) of keypoint k; returns the index of the element after it */
static int remove_obs_helper(bs_lmdb* db, bs_keypoint* k, int oi) {
    bs_hnode* hn = bs_htab_find(&db->observations, k->host.frame_id, (int64_t)k->host.cam_id);
    bs_host* h = (bs_host*)hn->val;
    int ti = tgt_lower(h->tgt, h->n, k->obs[oi].t);
    bs_tgt* tg = &h->tgt[ti];
    int ii = ids_find(tg->ids, tg->n, k->id);
    if (ii < tg->n && tg->ids[ii] == k->id) { memmove(tg->ids + ii, tg->ids + ii + 1, sizeof(int64_t) * (tg->n - ii - 1)); tg->n--; }
    if (tg->n == 0) {   /* host_it->second.erase(target_it) */
        free(tg->ids);
        memmove(h->tgt + ti, h->tgt + ti + 1, sizeof(bs_tgt) * (h->n - ti - 1));
        h->n--;
    }
    if (h->n == 0) { bs_htab_erase_node(&db->observations, hn); free_host(h); }
    memmove(k->obs + oi, k->obs + oi + 1, sizeof(bs_obs) * (k->nobs - oi - 1));
    k->nobs--;
    return oi;
}

/* removeLandmarkHelper(it): returns the node following the erased one */
static bs_hnode* remove_landmark_helper(bs_lmdb* db, bs_hnode* node) {
    bs_keypoint* k = (bs_keypoint*)node->val;
    bs_hnode* hn = bs_htab_find(&db->observations, k->host.frame_id, (int64_t)k->host.cam_id);
    bs_host* h;
    if (!hn) {
        bs_lmdb_ub++;
        free_keypoint(k);
        return bs_htab_erase_node(&db->kpts, node);
    }
    h = (bs_host*)hn->val;
    for (int oi = 0; oi < k->nobs; oi++) {
        int ti = tgt_lower(h->tgt, h->n, k->obs[oi].t);
        bs_tgt* tg = &h->tgt[ti];
        int ii = ids_find(tg->ids, tg->n, k->id);
        if (ii < tg->n && tg->ids[ii] == k->id) { memmove(tg->ids + ii, tg->ids + ii + 1, sizeof(int64_t) * (tg->n - ii - 1)); tg->n--; }
        if (tg->n == 0) {
            free(tg->ids);
            memmove(h->tgt + ti, h->tgt + ti + 1, sizeof(bs_tgt) * (h->n - ti - 1));
            h->n--;
        }
    }
    if (h->n == 0) { bs_htab_erase_node(&db->observations, hn); free_host(h); }
    free_keypoint(k);
    return bs_htab_erase_node(&db->kpts, node);
}

#define MIN_NUM_OBS 2

void bs_lmdb_remove_frame(bs_lmdb* db, int64_t frame) {
    for (bs_hnode* it = db->kpts.before_begin.next; it;) {
        bs_keypoint* k = (bs_keypoint*)it->val;
        for (int oi = 0; oi < k->nobs;) {
            if (k->obs[oi].t.frame_id == frame) oi = remove_obs_helper(db, k, oi);
            else oi++;
        }
        if (k->nobs < MIN_NUM_OBS) it = remove_landmark_helper(db, it);
        else it = it->next;
    }
}

static int in_set(const int64_t* a, int n, int64_t v) { for (int i = 0; i < n; i++) if (a[i] == v) return 1; return 0; }

void bs_lmdb_remove_keyframes(bs_lmdb* db, const int64_t* kfs, int nkf, const int64_t* poses, int np, const int64_t* states, int ns) {
    for (bs_hnode* it = db->kpts.before_begin.next; it;) {
        bs_keypoint* k = (bs_keypoint*)it->val;
        if (in_set(kfs, nkf, k->host.frame_id)) {
            it = remove_landmark_helper(db, it);
        } else {
            for (int oi = 0; oi < k->nobs;) {
                int64_t fid = k->obs[oi].t.frame_id;
                if (in_set(poses, np, fid) || in_set(states, ns, fid) || in_set(kfs, nkf, fid)) oi = remove_obs_helper(db, k, oi);
                else oi++;
            }
            if (k->nobs < MIN_NUM_OBS) it = remove_landmark_helper(db, it);
            else it = it->next;
        }
    }
}

void bs_lmdb_remove_landmark(bs_lmdb* db, int64_t id) {
    bs_hnode* n = bs_htab_find(&db->kpts, id, 0);
    if (n) remove_landmark_helper(db, n);
}

void bs_lmdb_remove_observations(bs_lmdb* db, int64_t id, const bs_tcid* obs, int n) {
    bs_hnode* node = bs_htab_find(&db->kpts, id, 0);
    if (!node) return;
    bs_keypoint* k = (bs_keypoint*)node->val;
    for (int oi = 0; oi < k->nobs;) {
        int found = 0;
        for (int j = 0; j < n; j++) if (bs_tcid_cmp(obs[j], k->obs[oi].t) == 0) { found = 1; break; }
        if (found) oi = remove_obs_helper(db, k, oi);
        else oi++;
    }
    if (k->nobs < MIN_NUM_OBS) remove_landmark_helper(db, node);
}

void bs_lmdb_backup(bs_lmdb* db) {
    for (bs_hnode* n = db->kpts.before_begin.next; n; n = n->next) {
        bs_keypoint* k = (bs_keypoint*)n->val;
        k->backup_direction[0] = k->direction[0]; k->backup_direction[1] = k->direction[1];
        k->backup_inv_dist = k->inv_dist;
    }
}
void bs_lmdb_restore(bs_lmdb* db) {
    for (bs_hnode* n = db->kpts.before_begin.next; n; n = n->next) {
        bs_keypoint* k = (bs_keypoint*)n->val;
        k->direction[0] = k->backup_direction[0]; k->direction[1] = k->backup_direction[1];
        k->inv_dist = k->backup_inv_dist;
    }
}
