/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M3c: PoissonDiskFilter<2> (rdvio_util/poisson_disk_filter.h, identical to rdvio_extra/poisson_disk_filter.h):
 * a minimum-distance filter over a sparse grid of cell size radius / sqrt(2), probing a (2*span+1)^2 neighbourhood (span = 2) with the
 * C++ loop's quirks (the window's first corner cell is skipped, one extra cell beyond the last row is probed; a cell stores only the LAST
 * point mapped to it). The unordered_map is only probed, never iterated, so any hash table reproduces it.
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified.
 *
 * Dump record layout (reference patch 0005-m3-ransac-dump.patch), channel "pois", native endian: a stream of operations
 *   u32 op, u64 filter id, then  op 0 (construct): f64 radius | op 1 (preset_point): f64 p[2] | op 2 (permit_point): f64 p[2], u32 result |
 *   op 3 (insert_point): f64 p[2], u32 result | op 4 (insert_points): u32 n, n x p[2], u32 kept, kept x p[2] | op 5 (clear) | op 6 (destroy)
 */
#ifndef RD_POISSON_H
#define RD_POISSON_H
#include <stddef.h>

typedef struct rd_poisson {
    double radius, radius_squared, grid_size;
    int grid_span;
    double* pts; size_t npts, cap;                 /* 2 doubles per point */
    int* keys; size_t* vals; unsigned char* used;  /* open-addressing table: key = (ix, iy) */
    size_t tcap, tcount;
} rd_poisson;

void rd_poisson_init(rd_poisson* f, double radius);
void rd_poisson_free(rd_poisson* f);
void rd_poisson_clear(rd_poisson* f);
void rd_poisson_preset_point(rd_poisson* f, const double p[2]);
int rd_poisson_permit_point(const rd_poisson* f, const double p[2]);
int rd_poisson_insert_point(rd_poisson* f, const double p[2]);
/* insert_points: candidates (n x 2) are filtered in place; returns the number kept (the C++ shrinks the vector to the accepted ones) */
size_t rd_poisson_insert_points(rd_poisson* f, double* candidates, size_t n);
#endif
