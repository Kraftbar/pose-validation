/* SPDX-License-Identifier: MIT */
#ifndef SV_BOW_H
#define SV_BOW_H

#include <stddef.h>
#include <stdint.h>

/* Port of FBoW (external/candidates/stella_vslam/3rd/FBoW, MIT, Copyright
 * (c) 2017 Rafael Munoz-Salinas + stella-cv 2022 fork) as used by
 * stella_vslam's data::bow_vocabulary_util (fbow::Vocabulary, BoWVector,
 * BoWFeatVector) for its ORB (CV_8UC1, 32-byte) vocabulary. See sv_bow.c
 * for the full MIT notice this port keeps.
 *
 * Binary vocabulary format (3rd/FBoW/src/fbow.cpp
 * Vocabulary::toStream/fromStream, include/fbow/vocabulary.h
 * Vocabulary::params/Block/block_node_info):
 *   [0:8)    uint64_t magic, must be 55824124
 *   [8:128)  Vocabulary::params, raw struct bytes (sizeof==120 on x86-64:
 *            char desc_name[50]; uint32_t aligment,nblocks; (pad 4)
 *            uint64_t desc_size_bytes_wp,block_size_bytes_wp,
 *            feature_off_start,child_off_start,total_size;
 *            int32_t desc_type,desc_size; uint32_t m_k; (pad to 8))
 *   [128:128+total_size) `nblocks` fixed-size blocks, each
 *            block_size_bytes_wp bytes:
 *              [0:2)   uint16_t N (valid node count, <= m_k)
 *              [2:4)   uint16_t isLeaf (block-level flag, unused by transform)
 *              [4:8)   uint32_t parent block id (unused by transform)
 *              [feature_off_start : feature_off_start + m_k*desc_size_bytes_wp)
 *                      m_k descriptors, desc_size_bytes_wp bytes each
 *                      (only the first desc_size bytes are real data; ORB:
 *                      desc_size==desc_size_bytes_wp==32, no padding)
 *              [child_off_start : child_off_start + m_k*8)
 *                      m_k block_node_info { uint32_t id_or_childblock;
 *                      float weight; } -- MSB of id_or_childblock set means
 *                      leaf (id = low 31 bits = word id); clear means
 *                      non-leaf (id = child block id).
 *
 * ORB vocabulary at external/candidates/orb_vocab.fbow: aligment=8, k
 * (branching factor, m_k)=10, desc_type=CV_8UC1(0), desc_size=32,
 * nblocks=110259 -- see runs/stella_port/reference_frame_bow/vocab_facts.txt
 * (produced by tools/dump_stella_frame_bow.py, which parses this same
 * header layout).
 *
 * File I/O stays out of this port (stdio-free, per the module-2 brief):
 * sv_bow_load_memory() takes an already-read memory buffer; the harness
 * does the fread().
 */

typedef struct sv_bow_vocab {
    const uint8_t* data;      /* buf + 128; NOT owned, caller keeps buf alive */
    uint32_t aligment;
    uint32_t nblocks;
    uint64_t desc_size_bytes_wp;
    uint64_t block_size_bytes_wp;
    uint64_t feature_off_start;
    uint64_t child_off_start;
    uint64_t total_size;
    int32_t desc_type;
    int32_t desc_size;
    uint32_t branching_k;
} sv_bow_vocab;

/* Parses the vocabulary header from `buf` (len bytes) and points
 * vocab->data at its block data (still inside `buf` -- caller must keep
 * `buf` alive as long as `vocab` and any sv_bow_transform() call using it
 * are in use). Returns 0 on success, -1 on bad signature / truncated
 * buffer. */
int sv_bow_load_memory(const uint8_t* buf, size_t len, sv_bow_vocab* vocab);

typedef struct sv_bow_word {
    uint32_t word_id;
    float weight;
} sv_bow_word;

/* Sorted ascending by word_id (std::map<uint32_t,_float> iteration order). */
typedef struct sv_bow_vector {
    sv_bow_word* words; /* owned, sv_bow_vector_free() releases it */
    uint32_t count;
} sv_bow_vector;

typedef struct sv_bow_feat_node {
    uint32_t node_id;
    uint32_t* kp_indices; /* owned */
    uint32_t count;
} sv_bow_feat_node;

/* Sorted ascending by node_id (std::map<uint32_t,vector<uint32_t>> order);
 * each node's kp_indices are in ascending descriptor-row order (push_back
 * order during the single left-to-right pass over descriptors). */
typedef struct sv_bow_feat_vector {
    sv_bow_feat_node* nodes; /* owned, sv_bow_feat_vector_free() releases it */
    uint32_t count;
} sv_bow_feat_vector;

/* Reproduces fbow::Vocabulary::transform(features, level, result, result2)
 * for CV_8UC1/32-byte (ORB) descriptors on an x86-64 host (i.e. the
 * L1_32bytes Hamming-distance path fbow.cpp's cpu::HW_x64 branch always
 * selects for this descriptor size) -- Vocabulary::_transform2's tree walk
 * plus the caller's final L2 normalization. `descriptors` is
 * num_descriptors rows of 32 bytes, row-major (same layout as module 1's
 * sv_orb_extract() output / stella's descriptors.tsv hex). `level` is the
 * BoWFeatVector capture depth (stella always calls this with level=4, see
 * data/bow_vocabulary_util.cc compute_bow()).
 *
 * Returns 0 on success, -1 on allocation failure. Caller must
 * sv_bow_vector_free()/sv_bow_feat_vector_free(). If num_descriptors==0,
 * both outputs are left empty (count=0, words/nodes=NULL), matching
 * fbow::Vocabulary::transform's early return. */
int sv_bow_transform(const sv_bow_vocab* vocab, const uint8_t* descriptors,
                     unsigned int num_descriptors, unsigned int level,
                     sv_bow_vector* bow_vec_out, sv_bow_feat_vector* bow_feat_vec_out);

void sv_bow_vector_free(sv_bow_vector* v);
void sv_bow_feat_vector_free(sv_bow_feat_vector* v);

/* fbow::BoWVector::score(): L2 similarity in [0,1], score=1 iff v1==v2's
 * nonzero support with equal weights (sum overlap >= 1, clamped). Inputs
 * must be sorted ascending by word_id (as sv_bow_transform() produces). */
double sv_bow_score(const sv_bow_vector* v1, const sv_bow_vector* v2);

#endif /* SV_BOW_H */
