/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_rng.h for provenance/license (BSD-2, AIST 2019 + stella-cv 2022). */
#include "sv_rng.h"
#include <stdlib.h>
#include <string.h>

#define SV_MT_N 624
#define SV_MT_M 397
#define SV_MT_MATRIX_A 0x9908b0dfU
#define SV_MT_UPPER_MASK 0x80000000U
#define SV_MT_LOWER_MASK 0x7fffffffU

void sv_mt19937_seed(sv_mt19937* e, uint32_t seed) {
    e->state[0] = seed;
    for (unsigned int i = 1; i < SV_MT_N; ++i) {
        e->state[i] = (1812433253U * (e->state[i - 1] ^ (e->state[i - 1] >> 30)) + i);
    }
    e->idx = SV_MT_N;
}

void sv_mt19937_init_default(sv_mt19937* e) {
    sv_mt19937_seed(e, 5489U);
}

static void sv_mt19937_regen(sv_mt19937* e) {
    static const uint32_t mag01[2] = {0U, SV_MT_MATRIX_A};
    unsigned int kk;
    uint32_t* mt = e->state;
    for (kk = 0; kk < SV_MT_N - SV_MT_M; ++kk) {
        uint32_t y = (mt[kk] & SV_MT_UPPER_MASK) | (mt[kk + 1] & SV_MT_LOWER_MASK);
        mt[kk] = mt[kk + SV_MT_M] ^ (y >> 1) ^ mag01[y & 1U];
    }
    for (; kk < SV_MT_N - 1; ++kk) {
        uint32_t y = (mt[kk] & SV_MT_UPPER_MASK) | (mt[kk + 1] & SV_MT_LOWER_MASK);
        mt[kk] = mt[kk + (SV_MT_M - SV_MT_N)] ^ (y >> 1) ^ mag01[y & 1U];
    }
    {
        uint32_t y = (mt[SV_MT_N - 1] & SV_MT_UPPER_MASK) | (mt[0] & SV_MT_LOWER_MASK);
        mt[SV_MT_N - 1] = mt[SV_MT_M - 1] ^ (y >> 1) ^ mag01[y & 1U];
    }
    e->idx = 0;
}

uint32_t sv_mt19937_next(sv_mt19937* e) {
    uint32_t y;
    if (e->idx >= SV_MT_N) {
        sv_mt19937_regen(e);
    }
    y = e->state[e->idx++];
    y ^= (y >> 11);
    y ^= (y << 7) & 0x9d2c5680U;
    y ^= (y << 15) & 0xefc60000U;
    y ^= (y >> 18);
    return y;
}

/* libstdc++ uniform_int_distribution<T>::_S_nd<uint64_t> (Lemire's nearly
 * divisionless downscaling), the path always taken here since mt19937's
 * range is exactly 2^32-1 (see uniform_int_dist.h operator(), the
 * "__urngrange == __UINT32_MAX__" branch). `range` == __uerange (= b-a+1,
 * i.e. an exclusive draw count in [0,range)). */
static uint32_t sv_lemire_draw(sv_mt19937* e, uint32_t range) {
    uint64_t product = (uint64_t)sv_mt19937_next(e) * (uint64_t)range;
    uint32_t low = (uint32_t)product;
    if (low < range) {
        uint32_t threshold = (uint32_t)(0U - range) % range;
        while (low < threshold) {
            product = (uint64_t)sv_mt19937_next(e) * (uint64_t)range;
            low = (uint32_t)product;
        }
    }
    return (uint32_t)(product >> 32);
}

uint32_t sv_uniform_uint(sv_mt19937* e, uint32_t a, uint32_t b) {
    uint32_t urange = b - a; /* inclusive range width - 1 */
    uint32_t uerange = urange + 1; /* may wrap to 0 iff urange==0xFFFFFFFF */
    if (uerange == 0) {
        /* full 32-bit range: libstdc++'s __urngrange == __urange case,
         * direct passthrough (not exercised by stella's callers). */
        return sv_mt19937_next(e) + a;
    }
    return a + sv_lemire_draw(e, uerange);
}

/* 64-bit-range Lemire draw for shuffle's gen_two_uniform_ints, where the
 * product b0*b1 can exceed 32 bits for large containers (not the case for
 * stella's min_set_size in {4,8}, but implemented generally). Mirrors
 * uniform_int_distribution<uint64_t>::operator() for __urngrange ==
 * UINT32_MAX (mt19937): if __urange < 2^32 use the 32-bit-erange Lemire
 * path above; if it reaches into >=2^32 territory (not reachable since
 * b0*b1 with b0,b1 fitting a 32-bit container size is < 2^33 at most and
 * this codebase's containers never approach that), fall back to the
 * fallback two-division scaling from the same header for correctness. */
static uint64_t sv_uniform_uint64(sv_mt19937* e, uint64_t a, uint64_t b) {
    uint64_t urange = b - a;
    if (urange < 0xFFFFFFFFULL) {
        return a + sv_lemire_draw(e, (uint32_t)(urange + 1));
    }
    /* fallback path (2 divisions), per uniform_int_dist.h; urngrange ==
     * 0xFFFFFFFF here always (mt19937), so scaling == 1 when urange ==
     * 0xFFFFFFFF, i.e. direct passthrough. */
    if (urange == 0xFFFFFFFFULL) {
        return a + sv_mt19937_next(e);
    }
    {
        const uint64_t urngrange = 0xFFFFFFFFULL;
        const uint64_t uerange = urange + 1;
        const uint64_t scaling = urngrange / uerange;
        const uint64_t past = uerange * scaling;
        uint64_t ret;
        do {
            ret = sv_mt19937_next(e);
        } while (ret >= past);
        return a + ret / scaling;
    }
}

/* GCC libstdc++'s std::shuffle (bits/stl_algo.h `shuffle`), fast paired-swap
 * path (taken whenever urngrange/urange >= urange, i.e. n <= ~65536 here --
 * always true for stella's min_set_size in {4,8}) with generic fallback. */
static void sv_shuffle(uint32_t* v, uint32_t n, sv_mt19937* e) {
    const uint64_t urngrange = 0xFFFFFFFFULL;
    if (n == 0) {
        return;
    }
    if (urngrange / n >= n) {
        uint32_t i = 1;
        if ((n % 2) == 0) {
            uint32_t j = (uint32_t)sv_uniform_uint64(e, 0, 1);
            uint32_t tmp = v[1];
            v[1] = v[j];
            v[j] = tmp;
            i = 2;
        }
        while (i != n) {
            uint64_t swap_range = (uint64_t)i + 1;
            {
                uint64_t b0 = swap_range;
                uint64_t b1 = swap_range + 1;
                uint64_t val = sv_uniform_uint64(e, 0, b0 * b1 - 1);
                uint64_t first = val / b1;
                uint64_t second = val % b1;
                uint32_t tmp1;
                tmp1 = v[i];
                v[i] = v[(uint32_t)first];
                v[(uint32_t)first] = tmp1;
                ++i;
                tmp1 = v[i];
                v[i] = v[(uint32_t)second];
                v[(uint32_t)second] = tmp1;
                ++i;
            }
        }
        return;
    }
    /* fallback (not reached for stella's usage, n always tiny). */
    {
        uint32_t i;
        for (i = 1; i < n; ++i) {
            uint32_t j = sv_uniform_uint(e, 0, i);
            uint32_t tmp = v[i];
            v[i] = v[j];
            v[j] = tmp;
        }
    }
}

static int sv_cmp_u32(const void* pa, const void* pb) {
    uint32_t a = *(const uint32_t*)pa;
    uint32_t b = *(const uint32_t*)pb;
    return (a > b) - (a < b);
}

void sv_create_random_array(uint32_t size, uint32_t rand_min, uint32_t rand_max,
                             sv_mt19937* engine, uint32_t* out) {
    /* static_cast<size_t>(size * 1.2) in double arithmetic, as stella does. */
    size_t make_size = (size_t)((double)size * 1.2);
    size_t cap = make_size > size ? make_size : size;
    /* generous headroom: each fill-to-make_size round can grow the buffer
     * by up to make_size elements before dedup; size is always tiny
     * (min_set_size in {4,8}) so this never gets large in practice. */
    uint32_t* v = (uint32_t*)malloc(sizeof(uint32_t) * (cap + 16));
    size_t n = 0;

    for (;;) {
        while (n < make_size) {
            v[n++] = sv_uniform_uint(engine, rand_min, rand_max);
        }
        qsort(v, n, sizeof(uint32_t), sv_cmp_u32);
        {
            size_t unique_end = n ? 1 : 0;
            size_t i;
            for (i = 1; i < n; ++i) {
                if (v[i] != v[unique_end - 1]) {
                    v[unique_end++] = v[i];
                }
            }
            n = unique_end;
        }
        if (size < n) {
            n = size;
        }
        if (n == size) {
            break;
        }
    }

    sv_shuffle(v, size, engine);
    memcpy(out, v, sizeof(uint32_t) * size);
    free(v);
}
