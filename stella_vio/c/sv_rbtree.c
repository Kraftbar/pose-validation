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

/* Behavioural model of a unique-key std::map red-black tree (the classic
 * insert-and-rebalance / rebalance-for-erase algorithms with the header node
 * holding root / leftmost / rightmost), see sv_rbtree.h. */
#include "sv_rbtree.h"
#include <stdlib.h>
#include <string.h>

#define HDR 0
#define NIL (-1)
#define N(i) (t->n[(i)])

void sv_rb_init(sv_rbtree* t) {
    memset(t, 0, sizeof(*t));
    t->cap = 16;
    t->n = (sv_rb_node*)calloc((size_t)t->cap, sizeof(sv_rb_node));
    t->used = 1;
    t->free_head = NIL;
    N(HDR).parent = NIL;
    N(HDR).left = HDR;
    N(HDR).right = HDR;
    N(HDR).red = 1;
}

void sv_rb_free(sv_rbtree* t) {
    free(t->n);
    memset(t, 0, sizeof(*t));
}

void sv_rb_clear(sv_rbtree* t) {
    t->used = 1;
    t->free_head = NIL;
    t->count = 0;
    N(HDR).parent = NIL;
    N(HDR).left = HDR;
    N(HDR).right = HDR;
    N(HDR).red = 1;
}

static int alloc_node(sv_rbtree* t, unsigned int key, unsigned int val) {
    int z;
    if (t->free_head != NIL) {
        z = t->free_head;
        t->free_head = N(z).left;
    }
    else {
        if (t->used == t->cap) {
            t->cap *= 2;
            t->n = (sv_rb_node*)realloc(t->n, (size_t)t->cap * sizeof(sv_rb_node));
        }
        z = t->used++;
    }
    N(z).key = key;
    N(z).val = val;
    N(z).parent = N(z).left = N(z).right = NIL;
    N(z).red = 0;
    return z;
}

static void free_node(sv_rbtree* t, int z) {
    N(z).left = t->free_head;
    t->free_head = z;
}

static int minimum(const sv_rbtree* t, int x) {
    while (N(x).left != NIL) {
        x = N(x).left;
    }
    return x;
}

static int maximum(const sv_rbtree* t, int x) {
    while (N(x).right != NIL) {
        x = N(x).right;
    }
    return x;
}

static int increment(const sv_rbtree* t, int x) {
    if (N(x).right != NIL) {
        x = N(x).right;
        while (N(x).left != NIL) {
            x = N(x).left;
        }
    }
    else {
        int y = N(x).parent;
        while (x == N(y).right) {
            x = y;
            y = N(y).parent;
        }
        if (N(x).right != y) {
            x = y;
        }
    }
    return x;
}

static int decrement(const sv_rbtree* t, int x) {
    if (x == HDR) {
        return N(HDR).right;
    }
    else if (N(x).left != NIL) {
        int y = N(x).left;
        while (N(y).right != NIL) {
            y = N(y).right;
        }
        return y;
    }
    else {
        int y = N(x).parent;
        while (x == N(y).left) {
            x = y;
            y = N(y).parent;
        }
        return y;
    }
}

int sv_rb_first(const sv_rbtree* t) {
    return t->count ? N(HDR).left : NIL;
}

int sv_rb_next(const sv_rbtree* t, int node) {
    const int nx = increment(t, node);
    return nx == HDR ? NIL : nx;
}

static void rotate_left(sv_rbtree* t, int x) {
    const int y = N(x).right;
    N(x).right = N(y).left;
    if (N(y).left != NIL) {
        N(N(y).left).parent = x;
    }
    N(y).parent = N(x).parent;
    if (x == N(HDR).parent) {
        N(HDR).parent = y;
    }
    else if (x == N(N(x).parent).left) {
        N(N(x).parent).left = y;
    }
    else {
        N(N(x).parent).right = y;
    }
    N(y).left = x;
    N(x).parent = y;
}

static void rotate_right(sv_rbtree* t, int x) {
    const int y = N(x).left;
    N(x).left = N(y).right;
    if (N(y).right != NIL) {
        N(N(y).right).parent = x;
    }
    N(y).parent = N(x).parent;
    if (x == N(HDR).parent) {
        N(HDR).parent = y;
    }
    else if (x == N(N(x).parent).right) {
        N(N(x).parent).right = y;
    }
    else {
        N(N(x).parent).left = y;
    }
    N(y).right = x;
    N(x).parent = y;
}

static void insert_and_rebalance(sv_rbtree* t, int insert_left, int x, int p) {
    N(x).parent = p;
    N(x).left = NIL;
    N(x).right = NIL;
    N(x).red = 1;
    if (insert_left) {
        N(p).left = x;
        if (p == HDR) {
            N(HDR).parent = x;
            N(HDR).right = x;
        }
        else if (p == N(HDR).left) {
            N(HDR).left = x;
        }
    }
    else {
        N(p).right = x;
        if (p == N(HDR).right) {
            N(HDR).right = x;
        }
    }
    while (x != N(HDR).parent && N(N(x).parent).red) {
        const int xpp = N(N(x).parent).parent;
        if (N(x).parent == N(xpp).left) {
            const int y = N(xpp).right;
            if (y != NIL && N(y).red) {
                N(N(x).parent).red = 0;
                N(y).red = 0;
                N(xpp).red = 1;
                x = xpp;
            }
            else {
                if (x == N(N(x).parent).right) {
                    x = N(x).parent;
                    rotate_left(t, x);
                }
                N(N(x).parent).red = 0;
                N(xpp).red = 1;
                rotate_right(t, xpp);
            }
        }
        else {
            const int y = N(xpp).left;
            if (y != NIL && N(y).red) {
                N(N(x).parent).red = 0;
                N(y).red = 0;
                N(xpp).red = 1;
                x = xpp;
            }
            else {
                if (x == N(N(x).parent).left) {
                    x = N(x).parent;
                    rotate_right(t, x);
                }
                N(N(x).parent).red = 0;
                N(xpp).red = 1;
                rotate_left(t, xpp);
            }
        }
    }
    N(N(HDR).parent).red = 0;
}

/* returns the node that was unlinked (z) */
static int rebalance_for_erase(sv_rbtree* t, int z) {
    int y = z, x = NIL, x_parent = NIL;
    if (N(y).left == NIL) {
        x = N(y).right;
    }
    else if (N(y).right == NIL) {
        x = N(y).left;
    }
    else {
        y = N(y).right;
        while (N(y).left != NIL) {
            y = N(y).left;
        }
        x = N(y).right;
    }
    if (y != z) {
        /* relink y in place of z; y is z's successor */
        N(N(z).left).parent = y;
        N(y).left = N(z).left;
        if (y != N(z).right) {
            x_parent = N(y).parent;
            if (x != NIL) {
                N(x).parent = N(y).parent;
            }
            N(N(y).parent).left = x; /* y must be a left child */
            N(y).right = N(z).right;
            N(N(z).right).parent = y;
        }
        else {
            x_parent = y;
        }
        if (N(HDR).parent == z) {
            N(HDR).parent = y;
        }
        else if (N(N(z).parent).left == z) {
            N(N(z).parent).left = y;
        }
        else {
            N(N(z).parent).right = y;
        }
        N(y).parent = N(z).parent;
        {
            const int c = N(y).red;
            N(y).red = N(z).red;
            N(z).red = c;
        }
        y = z; /* y now points to the node to be actually deleted */
    }
    else {
        x_parent = N(y).parent;
        if (x != NIL) {
            N(x).parent = N(y).parent;
        }
        if (N(HDR).parent == z) {
            N(HDR).parent = x;
        }
        else if (N(N(z).parent).left == z) {
            N(N(z).parent).left = x;
        }
        else {
            N(N(z).parent).right = x;
        }
        if (N(HDR).left == z) {
            if (N(z).right == NIL) {
                N(HDR).left = N(z).parent; /* makes leftmost == header if z == root */
            }
            else {
                N(HDR).left = minimum(t, x);
            }
        }
        if (N(HDR).right == z) {
            if (N(z).left == NIL) {
                N(HDR).right = N(z).parent; /* makes rightmost == header if z == root */
            }
            else {
                N(HDR).right = maximum(t, x);
            }
        }
    }
    if (!N(y).red) {
        while (x != N(HDR).parent && (x == NIL || !N(x).red)) {
            if (x == N(x_parent).left) {
                int w = N(x_parent).right;
                if (N(w).red) {
                    N(w).red = 0;
                    N(x_parent).red = 1;
                    rotate_left(t, x_parent);
                    w = N(x_parent).right;
                }
                if ((N(w).left == NIL || !N(N(w).left).red) && (N(w).right == NIL || !N(N(w).right).red)) {
                    N(w).red = 1;
                    x = x_parent;
                    x_parent = N(x_parent).parent;
                }
                else {
                    if (N(w).right == NIL || !N(N(w).right).red) {
                        N(N(w).left).red = 0;
                        N(w).red = 1;
                        rotate_right(t, w);
                        w = N(x_parent).right;
                    }
                    N(w).red = N(x_parent).red;
                    N(x_parent).red = 0;
                    if (N(w).right != NIL) {
                        N(N(w).right).red = 0;
                    }
                    rotate_left(t, x_parent);
                    break;
                }
            }
            else {
                /* same as above, with right <-> left */
                int w = N(x_parent).left;
                if (N(w).red) {
                    N(w).red = 0;
                    N(x_parent).red = 1;
                    rotate_right(t, x_parent);
                    w = N(x_parent).left;
                }
                if ((N(w).right == NIL || !N(N(w).right).red) && (N(w).left == NIL || !N(N(w).left).red)) {
                    N(w).red = 1;
                    x = x_parent;
                    x_parent = N(x_parent).parent;
                }
                else {
                    if (N(w).left == NIL || !N(N(w).left).red) {
                        N(N(w).right).red = 0;
                        N(w).red = 1;
                        rotate_left(t, w);
                        w = N(x_parent).left;
                    }
                    N(w).red = N(x_parent).red;
                    N(x_parent).red = 0;
                    if (N(w).left != NIL) {
                        N(N(w).left).red = 0;
                    }
                    rotate_right(t, x_parent);
                    break;
                }
            }
        }
        if (x != NIL) {
            N(x).red = 0;
        }
    }
    return y;
}

/* _M_lower_bound(x, y, k) */
static int lower_bound_from(const sv_rbtree* t, int x, int y, unsigned int k, sv_rb_less_fn less, void* ctx) {
    while (x != NIL) {
        if (!less(ctx, N(x).key, k)) {
            y = x;
            x = N(x).left;
        }
        else {
            x = N(x).right;
        }
    }
    return y;
}

/* _M_upper_bound(x, y, k) */
static int upper_bound_from(const sv_rbtree* t, int x, int y, unsigned int k, sv_rb_less_fn less, void* ctx) {
    while (x != NIL) {
        if (less(ctx, k, N(x).key)) {
            y = x;
            x = N(x).left;
        }
        else {
            x = N(x).right;
        }
    }
    return y;
}

int sv_rb_find(const sv_rbtree* t, unsigned int key, sv_rb_less_fn less, void* ctx) {
    const int j = lower_bound_from(t, N(HDR).parent, HDR, key, less, ctx);
    return (j == HDR || less(ctx, key, N(j).key)) ? NIL : j;
}

/* _M_get_insert_unique_pos: (*rx, *ry) as std::pair<_Base_ptr, _Base_ptr>; *ry == NIL means "key exists at *rx" */
static void get_insert_unique_pos(const sv_rbtree* t, unsigned int k, sv_rb_less_fn less, void* ctx, int* rx, int* ry) {
    int x = N(HDR).parent;
    int y = HDR;
    int comp = 1;
    int j;
    while (x != NIL) {
        y = x;
        comp = less(ctx, k, N(x).key);
        x = comp ? N(x).left : N(x).right;
    }
    j = y;
    if (comp) {
        if (j == N(HDR).left) { /* begin() */
            *rx = x;
            *ry = y;
            return;
        }
        j = decrement(t, j);
    }
    if (less(ctx, N(j).key, k)) {
        *rx = x;
        *ry = y;
        return;
    }
    *rx = j;
    *ry = NIL;
}

/* _M_get_insert_hint_unique_pos(pos, k) */
static void get_insert_hint_unique_pos(const sv_rbtree* t, int pos, unsigned int k, sv_rb_less_fn less, void* ctx, int* rx, int* ry) {
    if (pos == HDR) {
        if (t->count > 0 && less(ctx, N(N(HDR).right).key, k)) {
            *rx = NIL;
            *ry = N(HDR).right;
        }
        else {
            get_insert_unique_pos(t, k, less, ctx, rx, ry);
        }
    }
    else if (less(ctx, k, N(pos).key)) {
        /* first, try before... */
        int before = pos;
        if (pos == N(HDR).left) { /* begin() */
            *rx = N(HDR).left;
            *ry = N(HDR).left;
        }
        else {
            before = decrement(t, before);
            if (less(ctx, N(before).key, k)) {
                if (N(before).right == NIL) {
                    *rx = NIL;
                    *ry = before;
                }
                else {
                    *rx = pos;
                    *ry = pos;
                }
            }
            else {
                get_insert_unique_pos(t, k, less, ctx, rx, ry);
            }
        }
    }
    else if (less(ctx, N(pos).key, k)) {
        /* ... then try after. */
        int after = pos;
        if (pos == N(HDR).right) {
            *rx = NIL;
            *ry = N(HDR).right;
        }
        else {
            after = increment(t, after);
            if (less(ctx, k, N(after).key)) {
                if (N(pos).right == NIL) {
                    *rx = NIL;
                    *ry = pos;
                }
                else {
                    *rx = after;
                    *ry = after;
                }
            }
            else {
                get_insert_unique_pos(t, k, less, ctx, rx, ry);
            }
        }
    }
    else {
        /* equivalent keys */
        *rx = pos;
        *ry = NIL;
    }
}

/* _M_insert_node(x, p, z) */
static int insert_node(sv_rbtree* t, int x, int p, unsigned int key, unsigned int val, sv_rb_less_fn less, void* ctx) {
    const int z = alloc_node(t, key, val);
    const int insert_left = (x != NIL || p == HDR || less(ctx, key, N(p).key));
    insert_and_rebalance(t, insert_left, z, p);
    ++t->count;
    return z;
}

int sv_rb_index(sv_rbtree* t, unsigned int key, sv_rb_less_fn less, void* ctx) {
    const int i = lower_bound_from(t, N(HDR).parent, HDR, key, less, ctx);
    int rx, ry;
    if (i != HDR && !less(ctx, key, N(i).key)) {
        return i;
    }
    get_insert_hint_unique_pos(t, i, key, less, ctx, &rx, &ry);
    if (ry != NIL) {
        return insert_node(t, rx, ry, key, 0, less, ctx);
    }
    return rx;
}

void sv_rb_insert_at_end(sv_rbtree* t, unsigned int key, unsigned int val, sv_rb_less_fn less, void* ctx) {
    int rx, ry;
    get_insert_hint_unique_pos(t, HDR, key, less, ctx, &rx, &ry);
    if (ry != NIL) {
        insert_node(t, rx, ry, key, val, less, ctx);
    }
}

static void erase_aux(sv_rbtree* t, int pos) {
    const int y = rebalance_for_erase(t, pos);
    free_node(t, y);
    --t->count;
}

unsigned int sv_rb_erase_key(sv_rbtree* t, unsigned int key, sv_rb_less_fn less, void* ctx) {
    /* equal_range(key) */
    int x = N(HDR).parent, y = HDR;
    int first = HDR, last = HDR;
    const unsigned int old = t->count;
    int found = 0;
    while (x != NIL) {
        if (less(ctx, N(x).key, key)) {
            x = N(x).right;
        }
        else if (less(ctx, key, N(x).key)) {
            y = x;
            x = N(x).left;
        }
        else {
            int xu = x, yu = y;
            y = x;
            x = N(x).left;
            xu = N(xu).right;
            first = lower_bound_from(t, x, y, key, less, ctx);
            last = upper_bound_from(t, xu, yu, key, less, ctx);
            found = 1;
            break;
        }
    }
    if (!found) {
        first = y;
        last = y;
    }
    if (first == N(HDR).left && last == HDR) {
        sv_rb_clear(t);
    }
    else {
        while (first != last) {
            const int nx = increment(t, first);
            erase_aux(t, first);
            first = nx;
        }
    }
    return old - t->count;
}
