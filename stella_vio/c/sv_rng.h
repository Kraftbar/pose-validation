/* SV_PORT_SOURCES: sv_rng.c
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Port of stella_vslam's util/random_array.{h,cc} (create_random_engine,
 * create_random_array<T>) -- std::mt19937 with libstdc++ (GCC 13)'s exact
 * uniform_int_distribution and std::shuffle algorithms, so that RANSAC
 * sampling in the ported homography/fundamental solvers draws the identical
 * index sequence as the reference build for use_fixed_seed=true (the
 * deterministic TUM config).
 *
 * BSD 2-Clause License
 *
 * Copyright (c) 2019,
 * National Institute of Advanced Industrial Science and Technology (AIST),
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * stella-cv fork additions (2022) retain the same BSD 2-Clause license.
 *
 * NOTE on libstdc++ provenance: mt19937's tempering constants/seeding and
 * uniform_int_distribution/shuffle's *algorithms* are pinned to GCC 13's
 * libstdc++ (/usr/include/c++/13/bits/{mersenne_twister.h,uniform_int_dist.h,
 * stl_algo.h}), which is GPL-3.0-or-later with GCC runtime exception -- not copied
 * here (this file contains no libstdc++ source text), only its documented
 * public algorithm (ISO C++ mt19937 is a standard, fully specified
 * generator; uniform_int_distribution's Lemire downscaling and shuffle's
 * paired-swap fast path are GCC-specific implementation choices
 * independently reimplemented from reading those headers, as instructed by
 * the porting task).
 */
#ifndef SV_RNG_H
#define SV_RNG_H

#include <stdint.h>

/* std::mt19937 (word_size=32, state_size=624, shift=397, mask_bits=31,
 * xor_mask=0x9908b0df, tempering u/d/s/b/t/c/l = 11/0xffffffff/7/
 * 0x9d2c5680/15/0xefc60000/18, initialization_multiplier=1812433253,
 * default_seed=5489). */
typedef struct sv_mt19937 {
    uint32_t state[624];
    unsigned int idx; /* next state[] slot to temper-and-return; 624 == "needs a regen pass" */
} sv_mt19937;

/* stella's create_random_engine(use_fixed_seed=true): default-constructed
 * std::mt19937(), i.e. seeded with the default seed 5489. */
void sv_mt19937_init_default(sv_mt19937* e);
void sv_mt19937_seed(sv_mt19937* e, uint32_t seed);

/* One raw engine draw: advances state, tempers, returns. */
uint32_t sv_mt19937_next(sv_mt19937* e);

/* std::uniform_int_distribution<T>(a,b)(engine) for 32-bit-or-narrower
 * unsigned ranges (T = unsigned int / int, as stella uses; b>=a, b-a <
 * 2^32). Implements libstdc++'s Lemire downscaling ("_S_nd") path, which is
 * the one always taken here since mt19937's range is exactly 2^32-1 and
 * every (a,b) stella uses is far narrower. */
uint32_t sv_uniform_uint(sv_mt19937* e, uint32_t a, uint32_t b);

/* stella's util::create_random_array<T>(size, rand_min, rand_max, engine):
 * `size` unique values drawn uniformly from [rand_min, rand_max], returned
 * in libstdc++ std::shuffle order. out must have room for `size` elements.
 * T is unsigned int (the only instantiation the solvers use). */
void sv_create_random_array(uint32_t size, uint32_t rand_min, uint32_t rand_max,
                             sv_mt19937* engine, uint32_t* out);

#endif /* SV_RNG_H */
