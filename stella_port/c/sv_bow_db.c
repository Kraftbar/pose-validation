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
 *
 * C adaptation of stella_vslam e445b545 data/bow_database.cc. Only stella
 * source and its FBoW interface are used. Container order is represented by
 * sorted word buckets and insertion-ordered posting lists; result order is
 * explicitly canonicalized by ID (see header), not an STL emulation.
 */
#include "sv_bow_db.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    uint32_t word;
    const sv_bow_db_keyframe **keys;
    size_t count, capacity;
} bucket;
struct sv_bow_db { bucket *words; size_t count, capacity; };

static void *reserve(void *data, size_t *capacity, size_t need, size_t item)
{
    if (need <= *capacity) return data;
    size_t n = *capacity ? *capacity : 8;
    while (n < need) {
        if (n > SIZE_MAX / 2) { n = need; break; }
        n *= 2;
    }
    if (n > SIZE_MAX / item) return NULL;
    void *p = realloc(data, n * item);
    if (p) *capacity = n;
    return p;
}
static int valid_vector(const sv_bow_vector *v)
{
    if (!v || (v->count && !v->words)) return 0;
    for (uint32_t i = 1; i < v->count; ++i)
        if (v->words[i-1].word_id >= v->words[i].word_id) return 0;
    return 1;
}
static size_t lower_word(const sv_bow_db *db, uint32_t word)
{
    size_t lo = 0, hi = db->count;
    while (lo < hi) {
        size_t mid = lo + (hi-lo)/2;
        if (db->words[mid].word < word) lo = mid+1; else hi = mid;
    }
    return lo;
}
sv_bow_db *sv_bow_db_create(void) { return calloc(1, sizeof(sv_bow_db)); }
void sv_bow_db_clear(sv_bow_db *db)
{
    if (!db) return;
    for (size_t i = 0; i < db->count; ++i) free(db->words[i].keys);
    free(db->words); memset(db, 0, sizeof(*db));
}
void sv_bow_db_destroy(sv_bow_db *db) { sv_bow_db_clear(db); free(db); }

int sv_bow_db_add(sv_bow_db *db, const sv_bow_db_keyframe *kf)
{
    if (!db || !kf || !valid_vector(kf->bow)) return -1;
    uint32_t *created = kf->bow->count ? calloc(kf->bow->count, sizeof(uint32_t)) : NULL;
    if (kf->bow->count && !created) return -1;
    size_t n_created = 0;
    uint32_t i = 0;
    for (; i < kf->bow->count; ++i) {
        uint32_t word = kf->bow->words[i].word_id;
        size_t p = lower_word(db, word);
        if (p == db->count || db->words[p].word != word) {
            bucket *grown = reserve(db->words, &db->capacity, db->count+1, sizeof(bucket));
            if (!grown) goto fail;
            db->words = grown;
            memmove(db->words+p+1, db->words+p, (db->count-p)*sizeof(bucket));
            memset(db->words+p, 0, sizeof(bucket));
            db->words[p].word = word; ++db->count;
            created[n_created++] = word;
        }
        bucket *b = &db->words[p];
        const sv_bow_db_keyframe **grown = reserve(b->keys, &b->capacity, b->count+1, sizeof(*b->keys));
        if (!grown) goto fail;
        b->keys = grown;
        b->keys[b->count++] = kf;
    }
    free(created); return 0;
fail:
    for (uint32_t j = 0; j < i; ++j) --db->words[lower_word(db, kf->bow->words[j].word_id)].count;
    for (size_t j = 0; j < n_created; ++j) {
        size_t p = lower_word(db, created[j]);
        free(db->words[p].keys);
        memmove(db->words+p, db->words+p+1, (db->count-p-1)*sizeof(bucket));
        --db->count;
    }
    free(created); return -1;
}
int sv_bow_db_erase(sv_bow_db *db, const sv_bow_db_keyframe *kf)
{
    if (!db || !kf || !valid_vector(kf->bow)) return -1;
    for (uint32_t i = 0; i < kf->bow->count; ++i) {
        uint32_t word = kf->bow->words[i].word_id;
        size_t p = lower_word(db, word);
        if (p == db->count || db->words[p].word != word) continue;
        bucket *b = &db->words[p];
        for (size_t j = 0; j < b->count; ++j) if (b->keys[j]->id == kf->id) {
            memmove(b->keys+j, b->keys+j+1, (b->count-j-1)*sizeof(*b->keys));
            --b->count; break;
        }
    }
    return 0;
}
void sv_bow_db_result_free(sv_bow_db_result *r)
{
    if (!r) return;
    free(r->matches); memset(r, 0, sizeof(*r));
}
static int match_order(const void *a, const void *b)
{
    uint32_t x = ((const sv_bow_db_match *)a)->keyframe->id;
    uint32_t y = ((const sv_bow_db_match *)b)->keyframe->id;
    return (x > y) - (x < y);
}
int sv_bow_db_query(const sv_bow_db *db, const sv_bow_vector *v,
                    float min_score, float ratio,
                    const sv_bow_db_keyframe *const *reject, size_t n_reject,
                    sv_bow_db_result *result)
{
    if (!db || !result || !valid_vector(v) || (n_reject && !reject) ||
        !isfinite(min_score) || !isfinite(ratio) || ratio < 0) return -1;
    sv_bow_db_result r = {0};
    r.best_score = min_score;
    size_t capacity = 0;
    for (uint32_t i = 0; i < v->count; ++i) {
        size_t p = lower_word(db, v->words[i].word_id);
        if (p == db->count || db->words[p].word != v->words[i].word_id) continue;
        const bucket *b = &db->words[p];
        for (size_t j = 0; j < b->count; ++j) {
            const sv_bow_db_keyframe *kf = b->keys[j];
            size_t k;
            for (k = 0; k < n_reject; ++k) if (reject[k] == kf) break;
            if (k < n_reject) continue;
            for (k = 0; k < r.count; ++k) if (r.matches[k].keyframe == kf) break;
            if (k == r.count) {
                sv_bow_db_match *grown = reserve(r.matches, &capacity, r.count+1, sizeof(*r.matches));
                if (!grown) goto fail;
                r.matches = grown;
                memset(r.matches+k, 0, sizeof(*r.matches));
                r.matches[k].keyframe = kf; ++r.count;
            }
            if (r.matches[k].common_words == UINT32_MAX) goto fail;
            ++r.matches[k].common_words;
            if (r.max_common_words < r.matches[k].common_words)
                r.max_common_words = r.matches[k].common_words;
        }
    }
    if (r.count) {
        float threshold = ratio * r.max_common_words;
        if (!isfinite(threshold) || (double)threshold >= 4294967296.0) goto fail;
        r.min_common_words = (uint32_t)threshold;
    }
    if (r.count > 1) qsort(r.matches, r.count, sizeof(*r.matches), match_order);
    for (size_t i = 0; i < r.count; ++i) {
        sv_bow_db_match *m = &r.matches[i];
        if (r.min_common_words < m->common_words) {
            m->scored = 1;
            m->score = (float)sv_bow_score(v, m->keyframe->bow);
            if (min_score > m->score) continue;
            m->accepted = 1; ++r.accepted_count;
            if (r.best_score < m->score) r.best_score = m->score;
        }
    }
    sv_bow_db_result_free(result); *result = r; return 0;
fail:
    sv_bow_db_result_free(&r); return -1;
}
size_t sv_bow_db_num_words(const sv_bow_db *db) { return db ? db->count : 0; }
int sv_bow_db_word_at(const sv_bow_db *db, size_t i, sv_bow_db_word_view *v)
{
    if (!db || !v || i >= db->count) return -1;
    v->word_id = db->words[i].word; v->keyframes = db->words[i].keys;
    v->count = db->words[i].count; return 0;
}
