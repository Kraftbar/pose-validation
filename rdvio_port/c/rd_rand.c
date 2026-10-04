/* SPDX-License-Identifier: Apache-2.0 */
/* See rd_rand.h for provenance. */
#include "rd_rand.h"
#include <stdlib.h>
#include <string.h>

#define MINSTD_M 2147483647ULL
#define MINSTD_A 16807ULL

void rd_minstd_seed(rd_minstd* e, uint64_t value) {
    const uint64_t v = value % MINSTD_M;
    e->x = (v == 0) ? 1 : v;     /* c == 0 and seed == 0 -> 1 */
}
uint64_t rd_minstd_next(rd_minstd* e) {
    e->x = (MINSTD_A * e->x) % MINSTD_M;
    return e->x;
}

/* std::uniform_int_distribution<unsigned long>::operator()(urng, {a, b}) for the engine above (min 1, max 2147483646) */
uint64_t rd_uniform_int(rd_minstd* e, uint64_t a, uint64_t b) {
    const uint64_t urngmin = 1, urngmax = MINSTD_M - 1, urngrange = urngmax - urngmin;
    const uint64_t urange = b - a;
    uint64_t ret;
    if (urngrange > urange) {
        const uint64_t uerange = urange + 1;
        const uint64_t scaling = urngrange / uerange;
        const uint64_t past = uerange * scaling;
        do ret = rd_minstd_next(e) - urngmin; while (ret >= past);
        ret /= scaling;
    } else if (urngrange < urange) {
        uint64_t tmp;
        do {
            const uint64_t uerngrange = urngrange + 1;
            tmp = uerngrange * rd_uniform_int(e, 0, urange / uerngrange);
            ret = tmp + (rd_minstd_next(e) - urngmin);
        } while (ret > urange || ret < tmp);
    } else {
        ret = rd_minstd_next(e) - urngmin;
    }
    return ret + a;
}

void rd_lotbox_init(rd_lotbox* lb, size_t size) {
    size_t i;
    lb->cap = 0; lb->size = size;
    lb->lots = (size_t*)malloc(sizeof(size_t) * (size ? size : 1));
    for (i = 0; i < size; ++i) lb->lots[i] = i;
    rd_minstd_seed(&lb->dice, 1);   /* the C++ engine is seeded from std::random_device here; every RD-VIO use re-seeds explicitly */
}
void rd_lotbox_free(rd_lotbox* lb) { free(lb->lots); lb->lots = 0; }
void rd_lotbox_seed(rd_lotbox* lb, unsigned int value) { rd_minstd_seed(&lb->dice, value); }
void rd_lotbox_refill_all(rd_lotbox* lb) { lb->cap = 0; }
size_t rd_lotbox_draw_without_replacement(rd_lotbox* lb) {
    const size_t remaining = lb->size - lb->cap;
    if (remaining > 1) {
        const size_t j = (size_t)rd_uniform_int(&lb->dice, lb->cap, lb->size - 1);
        const size_t t = lb->lots[lb->cap]; lb->lots[lb->cap] = lb->lots[j]; lb->lots[j] = t;
        { const size_t result = lb->lots[lb->cap]; lb->cap++; return result; }
    } else if (remaining == 1) {
        lb->cap++;
        return lb->lots[lb->size - 1];
    }
    return (size_t)-1;
}

/* ---- glibc random_r TYPE_3 ---- */
static uint32_t g_ring[34];
static int g_pos;
void rd_glibc_srand(unsigned int seed) {
    int32_t r[344];
    int i;
    if (seed == 0) seed = 1;
    r[0] = (int32_t)seed;
    for (i = 1; i < 31; ++i) {
        const int32_t hi = r[i - 1] / 127773, lo = r[i - 1] % 127773;
        int32_t word = 16807 * lo - 2836 * hi;
        if (word < 0) word += 2147483647;
        r[i] = word;
    }
    for (i = 31; i < 34; ++i) r[i] = r[i - 31];
    for (i = 34; i < 344; ++i) r[i] = (int32_t)((uint32_t)r[i - 31] + (uint32_t)r[i - 3]);
    for (i = 0; i < 34; ++i) g_ring[i] = (uint32_t)r[344 - 34 + i];
    g_pos = 0;   /* g_ring[(g_pos + k) % 34] = r[310 + k] ; next output index 344 needs r[313] and r[341] */
}
int rd_glibc_rand(void) {
    /* r[n] = r[n-31] + r[n-3]; ring holds r[n-34 .. n-1] with oldest at g_pos */
    const uint32_t a = g_ring[(g_pos + 34 - 31) % 34];   /* r[n-31] */
    const uint32_t b = g_ring[(g_pos + 34 - 3) % 34];    /* r[n-3]  */
    const uint32_t v = a + b;
    /* ring position of r[n-34] is g_pos: overwrite it with r[n] */
    g_ring[g_pos] = v;
    g_pos = (g_pos + 1) % 34;
    return (int)(v >> 1);
}
