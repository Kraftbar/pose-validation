/* SPDX-License-Identifier: BSD-3-Clause
 * libstdc++ 13 unordered_map/unordered_set node-order model, see bs_hashorder.h. */
#include "bs_hashorder.h"
#include <stdlib.h>
#include <string.h>

/* _Prime_rehash_policy prime table (libstdc++.so, GCC 13.3: 249 entries above 13, dumped with _M_next_bkt) */
static const size_t k_primes[] = {
    17, 19, 23, 29, 31, 37, 41, 43,
    47, 53, 59, 61, 67, 71, 73, 79,
    83, 89, 97, 103, 109, 113, 127, 137,
    139, 149, 157, 167, 179, 193, 199, 211,
    227, 241, 257, 277, 293, 313, 337, 359,
    383, 409, 439, 467, 503, 541, 577, 619,
    661, 709, 761, 823, 887, 953, 1031, 1109,
    1193, 1289, 1381, 1493, 1613, 1741, 1879, 2029,
    2179, 2357, 2549, 2753, 2971, 3209, 3469, 3739,
    4027, 4349, 4703, 5087, 5503, 5953, 6427, 6949,
    7517, 8123, 8783, 9497, 10273, 11113, 12011, 12983,
    14033, 15173, 16411, 17749, 19183, 20753, 22447, 24281,
    26267, 28411, 30727, 33223, 35933, 38873, 42043, 45481,
    49201, 53201, 57557, 62233, 67307, 72817, 78779, 85229,
    92203, 99733, 107897, 116731, 126271, 136607, 147793, 159871,
    172933, 187091, 202409, 218971, 236897, 256279, 277261, 299951,
    324503, 351061, 379787, 410857, 444487, 480881, 520241, 562841,
    608903, 658753, 712697, 771049, 834181, 902483, 976369, 1056323,
    1142821, 1236397, 1337629, 1447153, 1565659, 1693859, 1832561, 1982627,
    2144977, 2320627, 2510653, 2716249, 2938679, 3179303, 3439651, 3721303,
    4026031, 4355707, 4712381, 5098259, 5515729, 5967347, 6456007, 6984629,
    7556579, 8175383, 8844859, 9569143, 10352717, 11200489, 12117689, 13109983,
    14183539, 15345007, 16601593, 17961079, 19431899, 21023161, 22744717, 24607243,
    26622317, 28802401, 31160981, 33712729, 36473443, 39460231, 42691603, 46187573,
    49969847, 54061849, 58488943, 63278561, 68460391, 74066549, 80131819, 86693767,
    93793069, 101473717, 109783337, 118773397, 128499677, 139022417, 150406843, 162723577,
    176048909, 190465427, 206062531, 222936881, 241193053, 260944219, 282312799, 305431229,
    330442829, 357502601, 386778277, 418451333, 452718089, 489790921, 529899637, 573292817,
    620239453, 671030513, 725980837, 785430967, 849749479, 919334987, 994618837, 1076067617,
    1164186217, 1259520799, 1362662261, 1474249943, 1594975441, 1725587117, 1866894511, 2019773507,
    2185171673, 2364114217, 2557710269, 2767159799, 2993761039, 3238918481, 3504151727, 3791104843,
    4101556399,
};
#define N_PRIMES (sizeof(k_primes) / sizeof(k_primes[0]))

size_t bs_hash_next_bkt(size_t n, size_t* next_resize) {
    static const unsigned char fast_bkt[] = {2, 2, 2, 3, 5, 5, 7, 7, 11, 11, 11, 11, 13, 13};
    size_t r;
    if (n < sizeof(fast_bkt)) {
        if (n == 0) { if (next_resize) *next_resize = 0; return 1; }
        r = fast_bkt[n];
    } else {
        size_t lo = 0, hi = N_PRIMES;   /* lower_bound: first prime >= n */
        while (lo < hi) { size_t m = (lo + hi) / 2; if (k_primes[m] < n) lo = m + 1; else hi = m; }
        if (lo == N_PRIMES) lo = N_PRIMES - 1;
        r = k_primes[lo];
    }
    if (next_resize) *next_resize = r;     /* floor(r * max_load_factor 1.0) */
    return r;
}

size_t bs_hash_code(int kind, int64_t k0, int64_t k1) {
    if (kind == BS_HK_U64) return (size_t)k0;
    size_t seed = 0;                        /* basalt::hash_combine, 64-bit variant */
    seed ^= (size_t)k0 + 0x9e3779b97f4a7c15ULL + (seed << 12) + (seed >> 4);
    seed ^= (size_t)k1 + 0x9e3779b97f4a7c15ULL + (seed << 12) + (seed >> 4);
    return seed;
}

static size_t bkt_of(const bs_htab* t, const bs_hnode* n, size_t nb) { return bs_hash_code(t->kind, n->k0, n->k1) % nb; }

void bs_htab_init(bs_htab* t, int kind) {
    memset(t, 0, sizeof(*t));
    t->kind = kind;
    t->nbuckets = 1;
    t->single_bucket = NULL;
    t->buckets = &t->single_bucket;
}

void bs_htab_destroy(bs_htab* t, void (*free_val)(void*)) {
    bs_hnode* n = t->before_begin.next;
    while (n) {
        bs_hnode* nx = n->next;
        if (free_val && n->val) free_val(n->val);
        free(n);
        n = nx;
    }
    if (t->buckets != &t->single_bucket) free(t->buckets);
    bs_htab_init(t, t->kind);
}

bs_hnode* bs_htab_find(const bs_htab* t, int64_t k0, int64_t k1) {
    size_t b = bs_hash_code(t->kind, k0, k1) % t->nbuckets;
    bs_hnode* prev = t->buckets[b];
    if (!prev) return NULL;
    for (bs_hnode* n = prev->next; n && bkt_of(t, n, t->nbuckets) == b; n = n->next)
        if (n->k0 == k0 && n->k1 == k1) return n;
    return NULL;
}

static void rehash(bs_htab* t, size_t nb) {
    bs_hnode** nbk = (nb == 1) ? &t->single_bucket : (bs_hnode**)calloc(nb, sizeof(bs_hnode*));
    if (nb == 1) t->single_bucket = NULL;
    bs_hnode* p = t->before_begin.next;
    t->before_begin.next = NULL;
    size_t bbegin_bkt = 0;
    while (p) {
        bs_hnode* next = p->next;
        size_t b = bkt_of(t, p, nb);
        if (!nbk[b]) {
            p->next = t->before_begin.next;
            t->before_begin.next = p;
            nbk[b] = &t->before_begin;
            if (p->next) nbk[bbegin_bkt] = p;
            bbegin_bkt = b;
        } else {
            p->next = nbk[b]->next;
            nbk[b]->next = p;
        }
        p = next;
    }
    if (t->buckets != &t->single_bucket) free(t->buckets);
    t->buckets = nbk;
    t->nbuckets = nb;
}

bs_hnode* bs_htab_insert(bs_htab* t, int64_t k0, int64_t k1, int* inserted) {
    bs_hnode* f = bs_htab_find(t, k0, k1);
    if (f) { if (inserted) *inserted = 0; return f; }
    if (inserted) *inserted = 1;
    bs_hnode* n = (bs_hnode*)calloc(1, sizeof(bs_hnode));
    n->k0 = k0; n->k1 = k1;
    /* _M_need_rehash(bkt_count, element_count, 1) */
    if (t->nelem + 1 > t->next_resize) {
        size_t lim = t->next_resize ? 0 : 11, want0 = t->nelem + 1 > lim ? t->nelem + 1 : lim;
        double min_bkts = (double)want0 / 1.0;
        if (min_bkts >= (double)t->nbuckets) {
            size_t want = (size_t)min_bkts + 1, g = t->nbuckets * 2;
            if (g > want) want = g;
            size_t nb = bs_hash_next_bkt(want, &t->next_resize);
            rehash(t, nb);
        } else {
            t->next_resize = t->nbuckets;
        }
    }
    size_t b = bs_hash_code(t->kind, k0, k1) % t->nbuckets;
    if (t->buckets[b]) {
        n->next = t->buckets[b]->next;
        t->buckets[b]->next = n;
    } else {
        n->next = t->before_begin.next;
        t->before_begin.next = n;
        if (n->next) t->buckets[bkt_of(t, n->next, t->nbuckets)] = n;
        t->buckets[b] = &t->before_begin;
    }
    t->nelem++;
    return n;
}

bs_hnode* bs_htab_erase_node(bs_htab* t, bs_hnode* n) {
    size_t b = bkt_of(t, n, t->nbuckets);
    bs_hnode* prev = t->buckets[b];
    while (prev->next != n) prev = prev->next;
    bs_hnode* next = n->next;
    if (prev == t->buckets[b]) {
        size_t next_bkt = next ? bkt_of(t, next, t->nbuckets) : 0;
        if (!next || next_bkt != b) {
            if (next) t->buckets[next_bkt] = t->buckets[b];
            if (&t->before_begin == t->buckets[b]) t->before_begin.next = next;
            t->buckets[b] = NULL;
        }
    } else if (next) {
        size_t next_bkt = bkt_of(t, next, t->nbuckets);
        if (next_bkt != b) t->buckets[next_bkt] = prev;
    }
    prev->next = next;
    free(n);
    t->nelem--;
    return next;
}

int bs_htab_erase_key(bs_htab* t, int64_t k0, int64_t k1) {
    bs_hnode* n = bs_htab_find(t, k0, k1);
    if (!n) return 0;
    bs_htab_erase_node(t, n);
    return 1;
}
