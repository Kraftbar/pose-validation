/* SPDX-License-Identifier: BSD-2-Clause
 * Derived from stella_vslam e445b545 data/bow_database.{h,cc}.
 * Copyright (c) 2019 AIST; Copyright (c) 2022 stella-cv.
 * Full notice in sv_bow_db.c. */
#ifndef SV_BOW_DB_H
#define SV_BOW_DB_H
#include "sv_bow.h"

typedef struct sv_bow_db sv_bow_db;
typedef struct {
    uint32_t id;
    const sv_bow_vector *bow;
} sv_bow_db_keyframe;

/* Caller-owned stable keyframe objects and sorted, unique-word BoW vectors.
 * Keep them alive and immutable while indexed. Distinct indexed objects must
 * have distinct IDs. Multiple insertions of the SAME object are allowed: upstream
 * appends duplicates, and erase removes only the first ID match per word.
 * Reject lists use object identity, just like canonical shared_ptr objects.
 * Single-threaded API; callers must serialize mutation and queries. */
sv_bow_db *sv_bow_db_create(void);
void sv_bow_db_clear(sv_bow_db *db);
void sv_bow_db_destroy(sv_bow_db *db);
int sv_bow_db_add(sv_bow_db *db, const sv_bow_db_keyframe *keyframe);
int sv_bow_db_erase(sv_bow_db *db, const sv_bow_db_keyframe *keyframe);

typedef struct {
    const sv_bow_db_keyframe *keyframe;
    uint32_t common_words;
    int scored, accepted;
    float score; /* float(sv_bow_score(...)), NOT the raw FBoW double */
} sv_bow_db_match;
typedef struct {
    sv_bow_db_match *matches; /* Owned rows; keyframe pointers remain borrowed. */
    size_t count, accepted_count;
    uint32_t max_common_words, min_common_words;
    float best_score;
} sv_bow_db_result;

/* Zero-initialize result before first use; query replaces its contents.
 * Entries are ID-sorted, NOT upstream's pointer-hashed iteration order.
 * Candidate membership and float score bits follow upstream exactly.
 * Integration must explicitly decide candidate traversal order: a caller
 * stopping at the first successful candidate can be order-sensitive.
 * min_score must be finite; ratio must be finite/nonnegative and its product
 * with max_common_words must fit uint32_t (upstream otherwise has UB).
 * Returns 0 on success, -1 on invalid input/allocation/count overflow.
 * On query error the previous result is left intact. */
int sv_bow_db_query(const sv_bow_db *db, const sv_bow_vector *query,
                    float min_score, float ratio,
                    const sv_bow_db_keyframe *const *reject, size_t n_reject,
                    sv_bow_db_result *result);
void sv_bow_db_result_free(sv_bow_db_result *result);

/* Read-only inverted-file inspection, ordered by word ID, including empty
 * buckets retained by erase. Views expire on the next database mutation. */
typedef struct {
    uint32_t word_id;
    const sv_bow_db_keyframe *const *keyframes;
    size_t count;
} sv_bow_db_word_view;
size_t sv_bow_db_num_words(const sv_bow_db *db);
int sv_bow_db_word_at(const sv_bow_db *db, size_t index,
                     sv_bow_db_word_view *view);
#endif
