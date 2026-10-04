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


/* stella_vslam e445b545: module/local_map_updater.cc (acquire_local_map with
 * keyframe_id_threshold == 0), type.h (greater_number_and_id_object_pairs).
 * The reference build is -DDETERMINISTIC=ON, so keyframe_to_num_shared_lms_t
 * is a std::map ordered by keyframe id, and the final ordering (count
 * descending, id ascending) is a total order: partial_sort / sort agree on
 * every element that is used. */
#include "sv_track.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
    unsigned int num;
    unsigned int id;
} shared_pair;

static int cmp_shared(const void* pa, const void* pb) {
    const shared_pair* a = (const shared_pair*)pa;
    const shared_pair* b = (const shared_pair*)pb;
    if (a->num != b->num) {
        return a->num > b->num ? -1 : 1;
    }
    if (a->id != b->id) {
        return a->id < b->id ? -1 : 1;
    }
    return 0;
}

static void push_kf(sv_tr_local_map* m, unsigned int id) {
    if (m->n_kfs == m->cap_kfs) {
        m->cap_kfs = m->cap_kfs ? m->cap_kfs * 2 : 64;
        m->kfs = (unsigned int*)realloc(m->kfs, m->cap_kfs * sizeof(unsigned int));
    }
    m->kfs[m->n_kfs++] = id;
}

static void push_lm(sv_tr_local_map* m, unsigned int id) {
    if (m->n_lms == m->cap_lms) {
        m->cap_lms = m->cap_lms ? m->cap_lms * 2 : 1024;
        m->lms = (unsigned int*)realloc(m->lms, m->cap_lms * sizeof(unsigned int));
    }
    m->lms[m->n_lms++] = id;
}

void sv_tr_local_map_free(sv_tr_local_map* m) {
    free(m->kfs);
    free(m->lms);
    memset(m, 0, sizeof(*m));
}

int sv_tr_acquire_local_map(const sv_tr_config* cfg, const sv_tr_map* map, const int* frm_lms,
                            unsigned int n_frm, sv_tr_local_map* out) {
    unsigned int* counts;
    unsigned char* found_kf;
    shared_pair* pairs;
    unsigned int n_pairs = 0, i, j, id, max_num_shared_lms = 0, n_first, n_second_start;
    unsigned char* found_lm;

    out->n_kfs = 0;
    out->n_lms = 0;
    out->nearest_covisibility = SV_TR_NONE;

    /* ---- find_local_keyframes ---- */
    counts = (unsigned int*)calloc(map->kf_cap ? map->kf_cap : 1, sizeof(unsigned int));
    for (i = 0; i < n_frm; ++i) {
        const sv_tr_lm* lm;
        if (frm_lms[i] < 0) {
            continue;
        }
        lm = sv_tr_map_lm(map, frm_lms[i]);
        if (!lm) { /* will_be_erased() */
            continue;
        }
        for (j = 0; j < lm->num_obs; ++j) {
            counts[lm->obs_kf[j]]++;
        }
    }
    pairs = (shared_pair*)malloc((map->kf_cap ? map->kf_cap : 1) * sizeof(shared_pair));
    for (id = 0; id < map->kf_cap; ++id) { /* std::map order: ascending id */
        if (counts[id]) {
            pairs[n_pairs].num = counts[id];
            pairs[n_pairs].id = id;
            ++n_pairs;
        }
    }
    free(counts);
    if (n_pairs == 0) {
        free(pairs);
        return 0;
    }
    qsort(pairs, n_pairs, sizeof(shared_pair), cmp_shared);

    found_kf = (unsigned char*)calloc(map->kf_cap ? map->kf_cap : 1, 1);

    /* find_first_local_keyframes */
    for (i = 0; i < n_pairs; ++i) {
        if (!sv_tr_map_kf(map, (int)pairs[i].id)) { /* will_be_erased() */
            continue;
        }
        push_kf(out, pairs[i].id);
        found_kf[pairs[i].id] = 1;
        if (max_num_shared_lms < pairs[i].num) {
            max_num_shared_lms = pairs[i].num;
            out->nearest_covisibility = (int)pairs[i].id;
        }
        if (cfg->max_num_local_keyfrms <= out->n_kfs) {
            break;
        }
    }
    free(pairs);
    n_first = out->n_kfs;
    n_second_start = out->n_kfs;

    /* find_second_local_keyframes */
#define SV_ADD_SECOND(KF_ID)                                                     \
    do {                                                                         \
        const int kid_ = (KF_ID);                                                \
        if (kid_ >= 0 && sv_tr_map_kf(map, kid_) && !found_kf[kid_]) {           \
            found_kf[kid_] = 1;                                                  \
            push_kf(out, (unsigned int)kid_);                                    \
        }                                                                        \
    } while (0)
#define SV_FULL() (cfg->max_num_local_keyfrms <= n_first + (out->n_kfs - n_second_start))
    for (i = 0; i < n_first; ++i) {
        const sv_tr_kf* kf;
        unsigned int taken = 0;
        if (SV_FULL()) {
            break;
        }
        kf = sv_tr_map_kf(map, (int)out->kfs[i]);
        /* covisibilities of the neighbor keyframe: get_top_n_covisibilities(10) */
        for (j = 0; j < kf->n_covis && taken < 10; ++j) {
            if (!sv_tr_map_kf(map, (int)kf->covis[j])) {
                continue; /* expired weak_ptr: skipped without counting */
            }
            ++taken;
            SV_ADD_SECOND((int)kf->covis[j]);
            if (SV_FULL()) {
                goto second_done;
            }
        }
        /* children of the spanning tree */
        for (j = 0; j < kf->n_children; ++j) {
            SV_ADD_SECOND((int)kf->children[j]);
            if (SV_FULL()) {
                goto second_done;
            }
        }
        /* parent of the spanning tree */
        SV_ADD_SECOND(kf->parent);
    }
second_done:
#undef SV_ADD_SECOND
#undef SV_FULL
    free(found_kf);

    /* ---- find_local_landmarks ---- */
    found_lm = (unsigned char*)calloc(map->lm_cap ? map->lm_cap : 1, 1);
    for (i = 0; i < n_frm; ++i) {
        if (frm_lms[i] < 0 || !sv_tr_map_lm(map, frm_lms[i])) {
            continue;
        }
        found_lm[frm_lms[i]] = 1;
    }
    for (i = 0; i < out->n_kfs; ++i) {
        const sv_tr_kf* kf = sv_tr_map_kf(map, (int)out->kfs[i]);
        for (j = 0; j < kf->obs->num_kp; ++j) {
            const int lid = kf->lm[j];
            if (lid < 0 || !sv_tr_map_lm(map, lid)) {
                continue;
            }
            if (found_lm[lid]) {
                continue;
            }
            found_lm[lid] = 1;
            push_lm(out, (unsigned int)lid);
        }
    }
    free(found_lm);
    return 1;
}
