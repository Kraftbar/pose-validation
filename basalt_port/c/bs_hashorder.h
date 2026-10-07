/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt port, module M6: model of the libstdc++ (GCC 13.3, `std::unordered_map` / `std::unordered_set`, unique keys) hash table
 * node order, for the three containers whose iteration order feeds float summation in the estimator (basalt_port/PLAN.md 4b):
 *   LandmarkDatabase::kpts            unordered_map<size_t, Keypoint>               key BS_HK_U64 (std::hash<size_t> = identity)
 *   LandmarkDatabase::observations    unordered_map<TimeCamId, map<...>>            key BS_HK_TCID (basalt hash_combine)
 *   unordered_set<int> unconnected_obs0 (measure())                                  key BS_HK_U64 (std::hash<int> = (size_t)(long)int)
 * Modelled exactly: one forward singly linked list, bucket array of "node before the first node of the bucket", a new node goes
 * after the bucket's before-node (head of bucket) or, for an empty bucket, at the head of the whole list; rehash re-inserts the
 * nodes in list order with the _Prime_rehash_policy (growth factor 2, max load 1.0, prime table of libstdc++.so); erase keeps the
 * order of the others. The hash code is recomputed (not cached): identical order. Validated by
 * basalt_port/reference_tools/bs_hashorder_test.cc against the real std::unordered_map at tolerance 0 (exact order).
 */
#ifndef BS_HASHORDER_H
#define BS_HASHORDER_H

#include <stddef.h>
#include <stdint.h>

enum { BS_HK_U64 = 0, BS_HK_TCID = 1 };

typedef struct bs_hnode {
    struct bs_hnode* next;
    int64_t k0, k1;   /* key: U64 uses k0 only (k1 = 0); TCID = {frame_id, cam_id} */
    void* val;        /* payload owned by the caller */
} bs_hnode;

typedef struct bs_htab {
    int kind;
    bs_hnode before_begin;      /* before_begin.next = first node */
    bs_hnode** buckets;         /* bucket -> node BEFORE the bucket's first node (or &before_begin) */
    size_t nbuckets;
    size_t nelem;
    size_t next_resize;
    bs_hnode* single_bucket;
} bs_htab;

void bs_htab_init(bs_htab* t, int kind);
/* frees all nodes; free_val(NULL-safe) is called for each val if non-NULL */
void bs_htab_destroy(bs_htab* t, void (*free_val)(void*));
size_t bs_hash_code(int kind, int64_t k0, int64_t k1);
bs_hnode* bs_htab_find(const bs_htab* t, int64_t k0, int64_t k1);
/* operator[] / insert: returns the node, *inserted = 1 when new (val = NULL) */
bs_hnode* bs_htab_insert(bs_htab* t, int64_t k0, int64_t k1, int* inserted);
/* erase(key): returns 1 if erased; the node is freed, val NOT freed (caller took it) */
int bs_htab_erase_key(bs_htab* t, int64_t k0, int64_t k1);
/* erase(iterator): returns the following node (the iterator erase() returns) */
bs_hnode* bs_htab_erase_node(bs_htab* t, bs_hnode* n);
/* _Prime_rehash_policy::_M_next_bkt (also updates policy.next_resize through the pointer if non-NULL) */
size_t bs_hash_next_bkt(size_t n, size_t* next_resize);

#endif
