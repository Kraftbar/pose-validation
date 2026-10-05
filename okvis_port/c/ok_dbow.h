/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * OKVIS2 pure-C port, module 7d (part 1): the DBoW2 bag-of-words place recognition database of Frontend::DBoW (vocabulary tree
 * transform, TemplatedDatabase::add / queryL1 with the L1 scoring object, Frontend::getFilteredDBoWResult).
 *
 * Derived from DBoW2 (Copyright (c) 2015, Dorian Galvez-Lopez, http://doriangalvez.com; modified BSD licence, full text in
 * okvis_port/LICENSES/dbow2-LICENSE.txt: TemplatedVocabulary::transform, BowVector, TemplatedDatabase, QueryResults, FBrisk
 * distance) and OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; Frontend.cpp, FBrisk.cpp).
 * DBoW2 clause 3: "The original author of the work must be notified of any redistribution of source code or in binary form."
 * This file is a source-level port that is NOT redistributed so far; before any redistribution of okvis_port (source or binary)
 * Dorian Galvez-Lopez must be notified (okvis_port/NOTICE, HANDOVER.md). Redistribution requires retaining these notices.
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only.
 *
 * ---- vocabulary payload (record 170 of place.bin, patch 0014; also the file written by tools/convert_okvis_vocabulary.py) ----
 *   u32 k, u32 L, u32 weighting (0 TF_IDF), u32 scoring (0 L1), u32 nnodes, u32 nwords,
 *   nnodes x { u32 id, u32 parent, u32 word_id, f64 weight, u32 nchildren, nchildren x u32, u32 desclen, desclen bytes }
 * ---- place.bin (patch 0014; framed like problem.bin) ----
 *   170 vocabulary (above)
 *   171 database add     u32 entry id, u64 frame id, u32 nfeatures, BowVector
 *   172 query            u32 which (0 main database), u64 frame id, u32 nfeatures, BowVector of the query, u32 database size,
 *                        u32 nresults, nresults x { u32 id, f64 score } (QueryResults as returned by query(..., -1)),
 *                        u32 nstate, nstate x { u64 state id, f64 score } (getFilteredDBoWResult output)
 *   173 verifyRecognisedPlace stages: see ok_place.h
 *   BowVector = u32 n, n x { u32 word id, f64 weight } (ascending word id)
 */
#ifndef OK_DBOW_H
#define OK_DBOW_H
#include <stddef.h>
#include <stdint.h>

typedef struct ok_dbow_node {
    uint32_t id, parent, word_id;
    double weight;
    int nchildren;
    uint32_t* children;
    unsigned char desc[48];
} ok_dbow_node;

typedef struct ok_dbow_voc {
    int k, L, weighting, scoring, nnodes, nwords;
    ok_dbow_node* nodes;                 /* indexed by node id */
} ok_dbow_voc;

typedef struct ok_dbow_bow { int n; uint32_t* id; double* w; } ok_dbow_bow;    /* ascending word ids; malloc'd */
typedef struct ok_dbow_result { uint32_t id; double score; } ok_dbow_result;

typedef struct ok_dbow_row { int n, cap; uint32_t* entry; double* weight; } ok_dbow_row;    /* inverted file row of one word */
typedef struct ok_dbow_db {
    const ok_dbow_voc* voc;
    int nentries;
    ok_dbow_row* rows;                   /* nwords rows */
    uint64_t* pose_ids; int npose, cappose;   /* Frontend::DBoW::poseIds: the multiframe id of every entry */
} ok_dbow_db;

/* the vocabulary payload (see above); returns 0 on success */
int ok_dbow_voc_parse(ok_dbow_voc* v, const unsigned char* p, size_t n);
int ok_dbow_voc_load(ok_dbow_voc* v, const char* path);
void ok_dbow_voc_free(ok_dbow_voc* v);

/* TemplatedVocabulary::transform(features, v) for the L1-normalised TF-IDF vocabulary; feat: nfeat x 48 bytes */
void ok_dbow_transform(const ok_dbow_voc* v, const unsigned char* feat, int nfeat, ok_dbow_bow* out);
void ok_dbow_bow_free(ok_dbow_bow* b);

void ok_dbow_db_init(ok_dbow_db* db, const ok_dbow_voc* v);
void ok_dbow_db_free(ok_dbow_db* db);
/* TemplatedDatabase::add(features) + poseIds.push_back(pose_id); returns the entry id */
int ok_dbow_db_add(ok_dbow_db* db, const unsigned char* feat, int nfeat, uint64_t pose_id);
/* std::sort(first, last) of QueryResults by operator< (Score ascending): the introsort of libstdc++, exposed for the oracle test */
void ok_dbow_sort_results(ok_dbow_result* r, int n);
/* TemplatedDatabase::queryL1(vec, ret, max_results = -1, max_id = -1): results sorted by std::sort order of the raw scores, then
 * Score = -Score / 2; *out malloc'd */
void ok_dbow_query(const ok_dbow_db* db, const ok_dbow_bow* q, ok_dbow_result** out, int* nout);
/* Frontend::getFilteredDBoWResult after the query: `orig` is the query result (score order); *state_ids / *scores malloc'd
 * (ascending entry id of the retained entries) */
void ok_dbow_filtered(const ok_dbow_db* db, const ok_dbow_result* orig, int norig, uint64_t** state_ids, double** scores, int* nout);

#endif
