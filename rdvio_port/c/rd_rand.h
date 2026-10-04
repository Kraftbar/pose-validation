/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M3a: random number generation used by RANSAC / PARSAC (rdvio_util/random.h, parsac.h).
 *
 *  - rd_minstd:   libstdc++ std::default_random_engine = minstd_rand0 (x <- 16807 x mod 2147483647), seed(value) as std::seed
 *                 (value mod m, 0 -> 1).
 *  - rd_uniform:  libstdc++ (GCC 13) std::uniform_int_distribution<unsigned long>::operator()(urng, param(a, b)) driven by minstd_rand0
 *                 (urngrange = 2147483645 is neither 2^32-1 nor 2^64-1: the "fallback case (2 divisions)" with rejection, and the
 *                 "upscaling" recursion if b - a exceeds the engine range; both are modelled).
 *  - rd_lotbox:   LotBox (shuffle-by-swap sampling without replacement, `refill_all`).
 *  - rd_glibc_*:  glibc srand()/rand() (TYPE_3 additive feedback generator, r[34] state, 310 discarded outputs), RAND_MAX = 2147483647.
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified. The libstdc++ and glibc
 * algorithms are re-derived from their documented behaviour and checked against the real library calls (rdvio_port/reference_tools/rd_m3_oracle.cc).
 */
#ifndef RD_RAND_H
#define RD_RAND_H
#include <stddef.h>
#include <stdint.h>

typedef struct rd_minstd { uint64_t x; } rd_minstd;
void rd_minstd_seed(rd_minstd* e, uint64_t value);
uint64_t rd_minstd_next(rd_minstd* e);                                  /* engine call, returns 1 .. 2147483646 */
uint64_t rd_uniform_int(rd_minstd* e, uint64_t a, uint64_t b);          /* std::uniform_int_distribution<size_t>(a, b)(e) */

typedef struct rd_lotbox {
    size_t cap, size;
    size_t* lots;     /* malloc'd, 0 .. size-1 */
    rd_minstd dice;
} rd_lotbox;
void rd_lotbox_init(rd_lotbox* lb, size_t size);                        /* LotBox(size) + (its UniformInteger starts unseeded; use seed) */
void rd_lotbox_free(rd_lotbox* lb);
void rd_lotbox_seed(rd_lotbox* lb, unsigned int value);
void rd_lotbox_refill_all(rd_lotbox* lb);
size_t rd_lotbox_draw_without_replacement(rd_lotbox* lb);

void rd_glibc_srand(unsigned int seed);
int rd_glibc_rand(void);
#endif
