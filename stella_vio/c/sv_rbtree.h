/* SPDX-License-Identifier: BSD-2-Clause
 * Copyright (c) 2019, AIST; Copyright (c) 2022, stella-cv (for the stella_vslam
 * behaviour this model reproduces). See sv_rbtree.c. */
#ifndef SV_RBTREE_H
#define SV_RBTREE_H

/* Behavioural model of the red-black tree behind std::map<K, V, Cmp> (unique
 * keys), used by the module-6 port to reproduce stella_vslam's
 * `id_ordered_map<std::weak_ptr<keyframe>, unsigned int>`
 * (data::graph_node::connected_keyfrms_and_num_shared_lms_).
 *
 * Why a tree model and not a sorted array: the map's comparator (id_less<weak_ptr>)
 * dereferences the keys and treats an EXPIRED key as +infinity, while the tree
 * shape was built when that key still compared by id. Once a keyframe object has
 * been destroyed while another keyframe's map still holds its weak_ptr (the
 * covisibility graph is not always symmetric), lookups for larger keys walk the
 * wrong way and fail (get_num_shared_landmarks() returns 0, add_connection()
 * inserts, erase_connection() does nothing). That is observable in the reference
 * dumps, so the port keeps the tree shape: same node relinking, rotations and
 * colouring as the standard SGI/libstdc++ algorithms (insert_and_rebalance,
 * rebalance_for_erase, _M_get_insert_unique_pos / _M_get_insert_hint_unique_pos,
 * _M_lower_bound / _M_upper_bound, equal_range, in-order increment/decrement).
 * Validated against the real std::map on random operation sequences with
 * changing expiry (stella_port/reference_tools/rbtree_test.cc).
 *
 * Node 0 is the header (parent = root, left = leftmost, right = rightmost);
 * -1 is the null pointer. Keys are unsigned ids; `less(ctx, a, b)` is the
 * user comparator (called with the searched key first or second exactly as the
 * standard algorithms do). */

typedef int (*sv_rb_less_fn)(void* ctx, unsigned int a, unsigned int b);

typedef struct sv_rb_node {
    int parent, left, right;
    int red;
    unsigned int key, val;
} sv_rb_node;

typedef struct sv_rbtree {
    sv_rb_node* n; /* n[0] == header */
    int cap;
    int used;      /* next unused slot */
    int free_head; /* singly linked through .left */
    unsigned int count;
} sv_rbtree;

void sv_rb_init(sv_rbtree* t);
void sv_rb_free(sv_rbtree* t);
void sv_rb_clear(sv_rbtree* t);

/* map::find / map::count : node index or -1 */
int sv_rb_find(const sv_rbtree* t, unsigned int key, sv_rb_less_fn less, void* ctx);
/* map::operator[] : node of `key`, inserted with val 0 if absent (lower_bound + emplace_hint) */
int sv_rb_index(sv_rbtree* t, unsigned int key, sv_rb_less_fn less, void* ctx);
/* map::erase(key) : number of erased nodes */
unsigned int sv_rb_erase_key(sv_rbtree* t, unsigned int key, sv_rb_less_fn less, void* ctx);
/* std::map(first, last) element insertion: _M_insert_unique_(end(), value) */
void sv_rb_insert_at_end(sv_rbtree* t, unsigned int key, unsigned int val, sv_rb_less_fn less, void* ctx);

/* in-order iteration: -1 when done */
int sv_rb_first(const sv_rbtree* t);
int sv_rb_next(const sv_rbtree* t, int node);

#endif /* SV_RBTREE_H */
