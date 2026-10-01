/* SPDX-License-Identifier: BSD-2-Clause */
#ifndef SV_UMAP_ORDER_H
#define SV_UMAP_ORDER_H

/* Iteration-order model of `std::unordered_map<unsigned int, T>` as
 * implemented by the reference toolchain's libstdc++ (GCC 13.3), keys only.
 *
 * stella_vslam's local bundle adjuster keeps its local keyframes and local
 * landmarks in std::unordered_map<unsigned int, ...> and creates the g2o
 * vertices/edges by iterating them; the iteration order therefore decides
 * vertex ids, the Hessian block order and the edge insertion order, i.e.
 * the exact floating-point summation order of the BA. To reproduce it
 * without linking libstdc++, this file models the (observable, documented
 * as a "singly linked list of nodes grouped by bucket" container) behaviour:
 *   - hash of an unsigned key is the key itself, bucket = key % bucket_count;
 *   - an empty map has 1 bucket; the first insertion rehashes to 13, and a
 *     growth to N>next_resize elements rehashes to the smallest tabulated
 *     prime >= max(N+1, 2*bucket_count) (prime table below was MEASURED
 *     from the real library via rehash(n)/bucket_count(), n < 400000; for
 *     n < 14 the library uses a separate small table, also measured);
 *   - a new node goes to the front of its bucket's group, or, for an empty
 *     bucket, to the front of the whole list (the node that used to be first
 *     then becomes the bucket-begin of its own bucket);
 *   - a rehash re-inserts the existing nodes one by one in list order with
 *     the same rule;
 *   - erase unlinks the node (groups stay contiguous).
 * Verified against the real container on 60,000 random insert/erase
 * sequences by stella_port/reference_tools/umap_order_test.cc (0 mismatches,
 * see HANDOVER.md).
 * BSD-2 (behavioural model; no libstdc++ source is used).
 */

typedef struct sv_umap_order {
    unsigned int* keys;
    int* next;      /* node index of the next node in the list, -1 = end */
    int* alive;
    int first;      /* head of the list (before_begin.next), -1 = empty */
    int n_nodes;    /* nodes ever created (indices) */
    int cap;
    int count;      /* live elements */
    unsigned long nbkt;
    unsigned long next_resize;
    int* bucket_first; /* first node of each bucket's group, -1 if empty */
    int* bucket_prev;  /* node before that group (-1 = list head) */
} sv_umap_order;

void sv_umap_init(sv_umap_order* m);
void sv_umap_free(sv_umap_order* m);
/* operator[](key) on a missing key / emplace: returns 1 if a node was added. */
int sv_umap_insert(sv_umap_order* m, unsigned int key);
int sv_umap_contains(const sv_umap_order* m, unsigned int key);
/* erase(key): returns 1 if removed. */
int sv_umap_erase(sv_umap_order* m, unsigned int key);
/* iteration: for (i = m->first; i != -1; i = m->next[i]) use m->keys[i] */

#endif /* SV_UMAP_ORDER_H */
