/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_umap_order.h (BSD-2 behavioural model of libstdc++ unordered_map order). */
#include "sv_umap_order.h"

#include <stdlib.h>
#include <string.h>

/* Bucket counts reachable through the prime rehash policy for n >= 14
 * (measured from the real library; see sv_umap_order.h). */
static const unsigned long k_primes[] = {
    17u, 19u, 23u, 29u, 31u, 37u, 41u, 43u, 47u, 53u, 59u, 61u,
    67u, 71u, 73u, 79u, 83u, 89u, 97u, 103u, 109u, 113u, 127u, 137u,
    139u, 149u, 157u, 167u, 179u, 193u, 199u, 211u, 227u, 241u, 257u, 277u,
    293u, 313u, 337u, 359u, 383u, 409u, 439u, 467u, 503u, 541u, 577u, 619u,
    661u, 709u, 761u, 823u, 887u, 953u, 1031u, 1109u, 1193u, 1289u, 1381u, 1493u,
    1613u, 1741u, 1879u, 2029u, 2179u, 2357u, 2549u, 2753u, 2971u, 3209u, 3469u, 3739u,
    4027u, 4349u, 4703u, 5087u, 5503u, 5953u, 6427u, 6949u, 7517u, 8123u, 8783u, 9497u,
    10273u, 11113u, 12011u, 12983u, 14033u, 15173u, 16411u, 17749u, 19183u, 20753u, 22447u, 24281u,
    26267u, 28411u, 30727u, 33223u, 35933u, 38873u, 42043u, 45481u, 49201u, 53201u, 57557u, 62233u,
    67307u, 72817u, 78779u, 85229u, 92203u, 99733u, 107897u, 116731u, 126271u, 136607u, 147793u, 159871u,
    172933u, 187091u, 202409u, 218971u, 236897u, 256279u, 277261u, 299951u, 324503u, 351061u, 379787u, 410857u,
};
#define K_NPRIMES (sizeof(k_primes) / sizeof(k_primes[0]))
/* n < 14: measured small table */
static const unsigned char k_small[14] = {2, 2, 2, 3, 5, 5, 7, 7, 11, 11, 11, 11, 13, 13};

static unsigned long next_bkt(unsigned long n) {
    if (n < 14) {
        return n == 0 ? 1 : k_small[n];
    }
    {
        size_t i;
        for (i = 0; i < K_NPRIMES; ++i) {
            if (k_primes[i] >= n) {
                return k_primes[i];
            }
        }
    }
    abort(); /* beyond the measured table (410857 buckets) */
}

void sv_umap_init(sv_umap_order* m) {
    memset(m, 0, sizeof(*m));
    m->first = -1;
    m->nbkt = 1;
    m->next_resize = 0;
    m->bucket_first = (int*)malloc(sizeof(int));
    m->bucket_prev = (int*)malloc(sizeof(int));
    m->bucket_first[0] = -1;
    m->bucket_prev[0] = -1;
}

void sv_umap_free(sv_umap_order* m) {
    free(m->keys);
    free(m->next);
    free(m->alive);
    free(m->bucket_first);
    free(m->bucket_prev);
    memset(m, 0, sizeof(*m));
}

int sv_umap_contains(const sv_umap_order* m, unsigned int key) {
    int i;
    for (i = m->first; i != -1; i = m->next[i]) {
        if (m->keys[i] == key) {
            return 1;
        }
    }
    return 0;
}

/* Insert node `n` (already allocated) at the position the bucket rule gives,
 * for a table of `nb` buckets; `bucket_first[b]` = first node of bucket b in
 * the current list or -1 (recomputed by callers that need it). */
static void list_insert(sv_umap_order* m, int n, unsigned long nb, int* bucket_first, int* bucket_prev) {
    const unsigned long b = m->keys[n] % nb;
    if (bucket_first[b] != -1) {
        /* insert right after the "before" node of the bucket == at the front of the group */
        const int prev = bucket_prev[b];
        if (prev == -1) {
            m->next[n] = m->first;
            m->first = n;
        } else {
            m->next[n] = m->next[prev];
            m->next[prev] = n;
        }
        bucket_first[b] = n;
    } else {
        /* empty bucket: front of the whole list */
        const int old_first = m->first;
        m->next[n] = old_first;
        m->first = n;
        bucket_first[b] = n;
        bucket_prev[b] = -1;
        if (old_first != -1) {
            /* the former first node's bucket now begins after n */
            bucket_prev[m->keys[old_first] % nb] = n;
        }
    }
}

/* Rebuild bucket_first / bucket_prev from the list (groups are contiguous). */
static void scan_buckets(const sv_umap_order* m, unsigned long nb, int* bucket_first, int* bucket_prev) {
    unsigned long b;
    int i, prev = -1;
    for (b = 0; b < nb; ++b) {
        bucket_first[b] = -1;
        bucket_prev[b] = -1;
    }
    for (i = m->first; i != -1; prev = i, i = m->next[i]) {
        b = m->keys[i] % nb;
        if (bucket_first[b] == -1) {
            bucket_first[b] = i;
            bucket_prev[b] = prev;
        }
    }
}

static void rehash(sv_umap_order* m, unsigned long new_nb) {
    /* re-insert every node one by one in current list order, into a table of new_nb buckets */
    int* bf = (int*)malloc(sizeof(int) * new_nb);
    int* bp = (int*)malloc(sizeof(int) * new_nb);
    unsigned long b;
    int p = m->first, nxt;
    for (b = 0; b < new_nb; ++b) {
        bf[b] = -1;
        bp[b] = -1;
    }
    m->first = -1;
    while (p != -1) {
        nxt = m->next[p];
        list_insert(m, p, new_nb, bf, bp);
        p = nxt;
    }
    free(m->bucket_first);
    free(m->bucket_prev);
    m->bucket_first = bf;
    m->bucket_prev = bp;
    m->nbkt = new_nb;
}

int sv_umap_insert(sv_umap_order* m, unsigned int key) {
    if (sv_umap_contains(m, key)) {
        return 0;
    }
    /* _M_need_rehash(n_bkt, n_elt = count, n_ins = 1) */
    if ((unsigned long)m->count + 1 > m->next_resize) {
        unsigned long min_bkts = (unsigned long)m->count + 1;
        if (m->next_resize == 0 && min_bkts < 11) {
            min_bkts = 11;
        }
        if (min_bkts >= m->nbkt) {
            unsigned long want = min_bkts + 1;
            if (want < m->nbkt * 2) {
                want = m->nbkt * 2;
            }
            {
                const unsigned long nb = next_bkt(want);
                rehash(m, nb);
                m->next_resize = nb; /* floor(nb * max_load_factor(1.0)) */
            }
        } else {
            m->next_resize = m->nbkt;
        }
    }
    if (m->n_nodes == m->cap) {
        m->cap = m->cap ? m->cap * 2 : 16;
        m->keys = (unsigned int*)realloc(m->keys, sizeof(unsigned int) * (size_t)m->cap);
        m->next = (int*)realloc(m->next, sizeof(int) * (size_t)m->cap);
        m->alive = (int*)realloc(m->alive, sizeof(int) * (size_t)m->cap);
    }
    {
        const int n = m->n_nodes++;
        m->keys[n] = key;
        m->alive[n] = 1;
        m->next[n] = -1;
        list_insert(m, n, m->nbkt, m->bucket_first, m->bucket_prev);
        m->count++;
    }
    return 1;
}

int sv_umap_erase(sv_umap_order* m, unsigned int key) {
    int i, prev = -1;
    for (i = m->first; i != -1; prev = i, i = m->next[i]) {
        if (m->keys[i] == key) {
            if (prev == -1) {
                m->first = m->next[i];
            } else {
                m->next[prev] = m->next[i];
            }
            m->alive[i] = 0;
            m->count--;
            scan_buckets(m, m->nbkt, m->bucket_first, m->bucket_prev);
            return 1;
        }
    }
    return 0;
}
