/* SPDX-License-Identifier: BSD-3-Clause */
/* Exact Hamming distance for the 48-byte BRISK descriptor. Integer lane counts only;
 * memcpy permits unaligned descriptors without violating C aliasing rules. */
#ifndef OK_HAMMING_H
#define OK_HAMMING_H
#include <stdint.h>
#include <string.h>

static inline int ok_hamming48(const unsigned char* a, const unsigned char* b) {
    int i, n = 0;
    for (i = 0; i < 48; i += 8) {
        uint64_t x, y;
        memcpy(&x, a + i, 8); memcpy(&y, b + i, 8);
        x ^= y;
        x -= (x >> 1) & UINT64_C(0x5555555555555555);
        x = (x & UINT64_C(0x3333333333333333)) + ((x >> 2) & UINT64_C(0x3333333333333333));
        x = (x + (x >> 4)) & UINT64_C(0x0f0f0f0f0f0f0f0f);
        n += (int)((x * UINT64_C(0x0101010101010101)) >> 56);
    }
    return n;
}
#endif
