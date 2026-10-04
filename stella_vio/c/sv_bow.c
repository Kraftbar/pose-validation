/* SPDX-License-Identifier: MIT */
/* Port of FBoW (external/candidates/stella_vslam/3rd/FBoW), used by
 * stella_vslam for BoW vocabulary transform. Original:
 *
 * The MIT License
 *
 * Copyright (c) 2017 Rafael Munoz-Salinas
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * stella-cv fork additions (2022) retain the same MIT license (see
 * external/candidates/stella_vslam/3rd/FBoW/LICENSE).
 */
#include "sv_bow.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* --- vocabulary header parsing --------------------------------------- */

/* Vocabulary::params byte offsets within the file (see sv_bow.h's format
 * comment): magic occupies [0:8), params occupies [8:128) with these
 * absolute offsets for its fields (natural x86-64 struct padding of
 * `char[50]; uint32_t,uint32_t; uint64_t x5; int32_t,int32_t; uint32_t`,
 * verified against external/candidates/orb_vocab.fbow's actual byte
 * layout by tools/dump_stella_frame_bow.py). Hardcoded (not read via a C
 * struct + memcpy of a matching type) so this parse does not silently
 * depend on the compiling toolchain reproducing the same padding. */
#define SV_BOW_OFF_ALIGMENT 60
#define SV_BOW_OFF_NBLOCKS 64
#define SV_BOW_OFF_DESC_SIZE_BYTES_WP 72
#define SV_BOW_OFF_BLOCK_SIZE_BYTES_WP 80
#define SV_BOW_OFF_FEATURE_OFF_START 88
#define SV_BOW_OFF_CHILD_OFF_START 96
#define SV_BOW_OFF_TOTAL_SIZE 104
#define SV_BOW_OFF_DESC_TYPE 112
#define SV_BOW_OFF_DESC_SIZE 116
#define SV_BOW_OFF_M_K 120
#define SV_BOW_HEADER_SIZE 128 /* 8 (magic) + 120 (sizeof(params)) */

static uint64_t rd_u64(const uint8_t* p) { uint64_t v; memcpy(&v, p, 8); return v; }
static uint32_t rd_u32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
static int32_t rd_i32(const uint8_t* p) { int32_t v; memcpy(&v, p, 4); return v; }

int sv_bow_load_memory(const uint8_t* buf, size_t len, sv_bow_vocab* vocab) {
    memset(vocab, 0, sizeof(*vocab));
    if (len < SV_BOW_HEADER_SIZE) return -1;
    if (rd_u64(buf) != 55824124ULL) return -1;

    vocab->aligment = rd_u32(buf + SV_BOW_OFF_ALIGMENT);
    vocab->nblocks = rd_u32(buf + SV_BOW_OFF_NBLOCKS);
    vocab->desc_size_bytes_wp = rd_u64(buf + SV_BOW_OFF_DESC_SIZE_BYTES_WP);
    vocab->block_size_bytes_wp = rd_u64(buf + SV_BOW_OFF_BLOCK_SIZE_BYTES_WP);
    vocab->feature_off_start = rd_u64(buf + SV_BOW_OFF_FEATURE_OFF_START);
    vocab->child_off_start = rd_u64(buf + SV_BOW_OFF_CHILD_OFF_START);
    vocab->total_size = rd_u64(buf + SV_BOW_OFF_TOTAL_SIZE);
    vocab->desc_type = rd_i32(buf + SV_BOW_OFF_DESC_TYPE);
    vocab->desc_size = rd_i32(buf + SV_BOW_OFF_DESC_SIZE);
    vocab->branching_k = rd_u32(buf + SV_BOW_OFF_M_K);

    if (len < (size_t)SV_BOW_HEADER_SIZE + vocab->total_size) return -1;
    if ((uint64_t)vocab->nblocks * vocab->block_size_bytes_wp != vocab->total_size) return -1;

    vocab->data = buf + SV_BOW_HEADER_SIZE;
    return 0;
}

/* --- block/node access (Vocabulary::Block / block_node_info, vocabulary.h) */

typedef struct { uint32_t id_or_childblock; float weight; } sv_bow_node_info;

static const uint8_t* get_block(const sv_bow_vocab* v, uint32_t block_id) {
    return v->data + (uint64_t)block_id * v->block_size_bytes_wp;
}
static uint16_t block_n(const uint8_t* block) { uint16_t n; memcpy(&n, block, 2); return n; }
static const uint8_t* block_feature(const sv_bow_vocab* v, const uint8_t* block, uint32_t i) {
    return block + v->feature_off_start + (uint64_t)i * v->desc_size_bytes_wp;
}
static void block_info(const sv_bow_vocab* v, const uint8_t* block, uint32_t i, sv_bow_node_info* out) {
    const uint8_t* p = block + v->child_off_start + (uint64_t)i * 8;
    memcpy(&out->id_or_childblock, p, 4);
    memcpy(&out->weight, p + 4, 4);
}
static int node_is_leaf(const sv_bow_node_info* n) { return (n->id_or_childblock & 0x80000000u) != 0; }
static uint32_t node_id_of(const sv_bow_node_info* n) { return n->id_or_childblock & 0x7FFFFFFFu; }

/* --- Hamming distance, L1_32bytes path (fbow.cpp): 4x uint64 XOR +
 * popcount. Implemented per-byte (popcount is invariant to word grouping
 * of an XOR of the same bytes -- it is just a sum of independent bit
 * counts), so this needs no assumption about host endianness. */
/* Per-byte bit counts of one 64-bit word (SWAR; every byte lane holds 0..8 afterwards). */
static uint64_t popcount_bytes64(uint64_t x) {
    x = x - ((x >> 1) & 0x5555555555555555ULL);
    x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
    return (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0FULL;
}
static unsigned int hamming32(const uint8_t* a, const uint8_t* b) {
    uint64_t wa[4], wb[4];
    memcpy(wa, a, 32);
    memcpy(wb, b, 32);
    /* 4 words x 8 (max count per byte lane) = 32 per lane, no lane overflow; the multiply sums the 8 lanes. */
    uint64_t c = popcount_bytes64(wa[0] ^ wb[0]) + popcount_bytes64(wa[1] ^ wb[1])
               + popcount_bytes64(wa[2] ^ wb[2]) + popcount_bytes64(wa[3] ^ wb[3]);
    return (unsigned int)((c * 0x0101010101010101ULL) >> 56);
}

/* --- growable builders used only during sv_bow_transform() ------------ */

typedef struct { sv_bow_word* arr; uint32_t count, cap; } build_words;
typedef struct { uint32_t node_id; uint32_t* idx; uint32_t count, cap; } build_node;
typedef struct { build_node* arr; uint32_t count, cap; } build_nodes;

static sv_bow_word* words_find_or_create(build_words* w, uint32_t word_id) {
    uint32_t i;
    for (i = 0; i < w->count; i++) {
        if (w->arr[i].word_id == word_id) return &w->arr[i];
    }
    if (w->count == w->cap) {
        uint32_t new_cap = w->cap ? w->cap * 2 : 16;
        sv_bow_word* p = (sv_bow_word*)realloc(w->arr, sizeof(sv_bow_word) * new_cap);
        if (!p) return NULL;
        w->arr = p;
        w->cap = new_cap;
    }
    w->arr[w->count].word_id = word_id;
    w->arr[w->count].weight = 0.0f;
    return &w->arr[w->count++];
}

static build_node* nodes_find_or_create(build_nodes* ns, uint32_t node_id) {
    uint32_t i;
    for (i = 0; i < ns->count; i++) {
        if (ns->arr[i].node_id == node_id) return &ns->arr[i];
    }
    if (ns->count == ns->cap) {
        uint32_t new_cap = ns->cap ? ns->cap * 2 : 16;
        build_node* p = (build_node*)realloc(ns->arr, sizeof(build_node) * new_cap);
        if (!p) return NULL;
        ns->arr = p;
        ns->cap = new_cap;
    }
    build_node* n = &ns->arr[ns->count++];
    n->node_id = node_id;
    n->idx = NULL;
    n->count = 0;
    n->cap = 0;
    return n;
}

static int node_push(build_node* n, uint32_t feature_idx) {
    if (n->count == n->cap) {
        uint32_t new_cap = n->cap ? n->cap * 2 : 8;
        uint32_t* p = (uint32_t*)realloc(n->idx, sizeof(uint32_t) * new_cap);
        if (!p) return -1;
        n->idx = p;
        n->cap = new_cap;
    }
    n->idx[n->count++] = feature_idx;
    return 0;
}

static void build_words_free(build_words* w) { free(w->arr); w->arr = NULL; w->count = w->cap = 0; }
static void build_nodes_free(build_nodes* ns) {
    uint32_t i;
    for (i = 0; i < ns->count; i++) free(ns->arr[i].idx);
    free(ns->arr);
    ns->arr = NULL;
    ns->count = ns->cap = 0;
}

static int cmp_word(const void* a, const void* b) {
    uint32_t x = ((const sv_bow_word*)a)->word_id;
    uint32_t y = ((const sv_bow_word*)b)->word_id;
    return (x > y) - (x < y);
}
static int cmp_node(const void* a, const void* b) {
    uint32_t x = ((const build_node*)a)->node_id;
    uint32_t y = ((const build_node*)b)->node_id;
    return (x > y) - (x < y);
}

int sv_bow_transform(const sv_bow_vocab* vocab, const uint8_t* descriptors,
                     unsigned int num_descriptors, unsigned int level,
                     sv_bow_vector* bow_vec_out, sv_bow_feat_vector* bow_feat_vec_out) {
    memset(bow_vec_out, 0, sizeof(*bow_vec_out));
    memset(bow_feat_vec_out, 0, sizeof(*bow_feat_vec_out));
    if (num_descriptors == 0) return 0;

    uint32_t k = vocab->branching_k;
    uint32_t nbits = 0;
    while ((1u << nbits) < k) nbits++;

    build_words words;
    build_nodes nodes;
    memset(&words, 0, sizeof(words));
    memset(&nodes, 0, sizeof(nodes));

    unsigned int feat_idx;
    for (feat_idx = 0; feat_idx < num_descriptors; feat_idx++) {
        const uint8_t* feat = descriptors + (size_t)feat_idx * 32;
        uint32_t block_id = 0;
        uint32_t cur_node = 0;
        uint32_t lvl = 0;

        for (;;) {
            const uint8_t* block = get_block(vocab, block_id);
            uint16_t n = block_n(block);
            unsigned int best_dist = 0xFFFFFFFFu;
            uint32_t best_idx = 0;
            uint16_t ci;
            for (ci = 0; ci < n; ci++) {
                unsigned int d = hamming32(feat, block_feature(vocab, block, ci));
                if (d < best_dist) {
                    best_dist = d;
                    best_idx = ci;
                }
            }

            if (lvl == level) {
                build_node* bn = nodes_find_or_create(&nodes, cur_node);
                if (!bn || node_push(bn, feat_idx) != 0) goto fail;
            }

            sv_bow_node_info info;
            block_info(vocab, block, best_idx, &info);

            if (node_is_leaf(&info)) {
                sv_bow_word* w = words_find_or_create(&words, node_id_of(&info));
                if (!w) goto fail;
                w->weight += info.weight;
                if (lvl < level) {
                    build_node* bn = nodes_find_or_create(&nodes, cur_node);
                    if (!bn || node_push(bn, feat_idx) != 0) goto fail;
                }
                break;
            }

            block_id = node_id_of(&info);
            cur_node = (cur_node << nbits) | best_idx;
            lvl++;
            if (block_id == 0) break; /* mirrors `while (!leaf && getId()!=0)` */
        }
    }

    /* L2 normalize (Vocabulary::transform, fbow.cpp): norm accumulated in
     * double from float squares; each weight then rescaled in double and
     * truncated back to float, matching `e.second *= inv_norm` on a
     * _float (float-backed) map value. */
    double norm = 0.0;
    uint32_t i;
    for (i = 0; i < words.count; i++) {
        float w = words.arr[i].weight;
        norm += (double)(w * w);
    }
    if (norm > 0.0) {
        double inv_norm = 1.0 / sqrt(norm);
        for (i = 0; i < words.count; i++) {
            words.arr[i].weight = (float)((double)words.arr[i].weight * inv_norm);
        }
    }

    qsort(words.arr, words.count, sizeof(sv_bow_word), cmp_word);
    qsort(nodes.arr, nodes.count, sizeof(build_node), cmp_node);

    bow_vec_out->words = words.arr;
    bow_vec_out->count = words.count;

    if (nodes.count) {
        bow_feat_vec_out->nodes = (sv_bow_feat_node*)malloc(sizeof(sv_bow_feat_node) * nodes.count);
        if (!bow_feat_vec_out->nodes) { build_nodes_free(&nodes); goto fail; }
        for (i = 0; i < nodes.count; i++) {
            bow_feat_vec_out->nodes[i].node_id = nodes.arr[i].node_id;
            bow_feat_vec_out->nodes[i].kp_indices = nodes.arr[i].idx;
            bow_feat_vec_out->nodes[i].count = nodes.arr[i].count;
        }
        bow_feat_vec_out->count = nodes.count;
        free(nodes.arr); /* ownership of each .idx moved above; only the array itself is freed */
    }

    return 0;

fail:
    build_words_free(&words);
    build_nodes_free(&nodes);
    return -1;
}

void sv_bow_vector_free(sv_bow_vector* v) {
    if (!v) return;
    free(v->words);
    v->words = NULL;
    v->count = 0;
}

void sv_bow_feat_vector_free(sv_bow_feat_vector* v) {
    if (!v) return;
    uint32_t i;
    for (i = 0; i < v->count; i++) free(v->nodes[i].kp_indices);
    free(v->nodes);
    v->nodes = NULL;
    v->count = 0;
}

double sv_bow_score(const sv_bow_vector* v1, const sv_bow_vector* v2) {
    uint32_t i1 = 0, i2 = 0;
    double score = 0.0;
    while (i1 < v1->count && i2 < v2->count) {
        uint32_t id1 = v1->words[i1].word_id;
        uint32_t id2 = v2->words[i2].word_id;
        if (id1 == id2) {
            /* fbow.cpp BoWVector::score(): `score += vi * wi` where vi,wi
             * are _float (float-backed) references -- the multiply happens
             * in float precision, only the += promotes to the double
             * accumulator. Computing the product in double instead (as a
             * first pass here did) rounds differently. */
            score += (double)(v1->words[i1].weight * v2->words[i2].weight);
            i1++;
            i2++;
        }
        else if (id1 < id2) {
            while (i1 < v1->count && v1->words[i1].word_id < id2) i1++;
        }
        else {
            while (i2 < v2->count && v2->words[i2].word_id < id1) i2++;
        }
    }
    if (score >= 1.0) {
        score = 1.0;
    }
    else {
        score = 1.0 - sqrt(1.0 - score);
    }
    return score;
}
