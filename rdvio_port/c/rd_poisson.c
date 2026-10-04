/* SPDX-License-Identifier: Apache-2.0 */
/* See rd_poisson.h for provenance. */
#include "rd_poisson.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static size_t hash_key(int ix, int iy) {
    size_t h = (size_t)(unsigned)ix * 2654435761u;
    h ^= (size_t)(unsigned)iy + 0x9e3779b9u + (h << 6) + (h >> 2);
    return h;
}

static void table_alloc(rd_poisson* f, size_t cap) {
    f->tcap = cap; f->tcount = 0;
    f->keys = (int*)calloc(cap * 2, sizeof(int));
    f->vals = (size_t*)calloc(cap, sizeof(size_t));
    f->used = (unsigned char*)calloc(cap, 1);
}

static size_t table_find(const rd_poisson* f, int ix, int iy) {   /* slot index or (size_t)-1 */
    size_t s = hash_key(ix, iy) & (f->tcap - 1);
    while (f->used[s]) {
        if (f->keys[2 * s] == ix && f->keys[2 * s + 1] == iy) return s;
        s = (s + 1) & (f->tcap - 1);
    }
    return (size_t)-1;
}

static void table_set(rd_poisson* f, int ix, int iy, size_t val) {
    size_t s;
    if ((f->tcount + 1) * 2 > f->tcap) {   /* grow */
        rd_poisson old = *f; size_t i;
        table_alloc(f, old.tcap * 2);
        for (i = 0; i < old.tcap; ++i) if (old.used[i]) table_set(f, old.keys[2 * i], old.keys[2 * i + 1], old.vals[i]);
        free(old.keys); free(old.vals); free(old.used);
    }
    s = hash_key(ix, iy) & (f->tcap - 1);
    while (f->used[s]) {
        if (f->keys[2 * s] == ix && f->keys[2 * s + 1] == iy) { f->vals[s] = val; return; }
        s = (s + 1) & (f->tcap - 1);
    }
    f->used[s] = 1; f->keys[2 * s] = ix; f->keys[2 * s + 1] = iy; f->vals[s] = val; f->tcount++;
}

void rd_poisson_init(rd_poisson* f, double radius) {
    f->radius = radius;
    f->radius_squared = radius * radius;
    f->grid_size = radius / sqrt(2.0);
    f->grid_span = (int)ceil(sqrt(2.0));
    f->pts = 0; f->npts = 0; f->cap = 0;
    table_alloc(f, 64);
}
void rd_poisson_free(rd_poisson* f) { free(f->pts); free(f->keys); free(f->vals); free(f->used); memset(f, 0, sizeof *f); }
void rd_poisson_clear(rd_poisson* f) {
    f->npts = 0;
    memset(f->used, 0, f->tcap); f->tcount = 0;
}

static void to_index(const rd_poisson* f, const double p[2], int idx[2]) {
    idx[0] = (int)floor(p[0] / f->grid_size);
    idx[1] = (int)floor(p[1] / f->grid_size);
}

static void push_point(rd_poisson* f, const double p[2]) {
    if (f->npts == f->cap) { f->cap = f->cap ? f->cap * 2 : 64; f->pts = (double*)realloc(f->pts, sizeof(double) * 2 * f->cap); }
    f->pts[2 * f->npts] = p[0]; f->pts[2 * f->npts + 1] = p[1]; f->npts++;
}

void rd_poisson_preset_point(rd_poisson* f, const double p[2]) {
    int idx[2];
    to_index(f, p, idx);
    table_set(f, idx[0], idx[1], f->npts);
    push_point(f, p);
}

static int test_point(const rd_poisson* f, const double p[2], int index[2]) {
    int ib[2], ie[2], ic[2];
    to_index(f, p, index);
    ib[0] = index[0] - f->grid_span; ib[1] = index[1] - f->grid_span;
    ie[0] = index[0] + f->grid_span; ie[1] = index[1] + f->grid_span;
    ic[0] = ib[0]; ic[1] = ib[1];
    while (ic[1] <= ie[1]) {
        size_t s;
        ic[0]++;
        if (ic[0] > ie[0]) { ic[0] = ib[0]; ic[1]++; }       /* for (i = 0; icurr[i] > iend[i] && i < dimension - 1; ++i) */
        s = table_find(f, ic[0], ic[1]);
        if (s != (size_t)-1) {
            const double* q = f->pts + 2 * f->vals[s];
            const double d0 = p[0] - q[0], d1 = p[1] - q[1];
            if (d0 * d0 + d1 * d1 < f->radius_squared) return 0;
        }
    }
    return 1;
}

int rd_poisson_permit_point(const rd_poisson* f, const double p[2]) { int index[2]; return test_point(f, p, index); }

int rd_poisson_insert_point(rd_poisson* f, const double p[2]) {
    int index[2];
    if (test_point(f, p, index)) { table_set(f, index[0], index[1], f->npts); push_point(f, p); return 1; }
    return 0;
}

size_t rd_poisson_insert_points(rd_poisson* f, double* candidates, size_t n) {
    size_t i, kept = 0;
    for (i = 0; i < n; ++i) {
        int index[2];
        const double p[2] = {candidates[2 * i], candidates[2 * i + 1]};
        if (test_point(f, p, index)) {
            table_set(f, index[0], index[1], f->npts); push_point(f, p);
            candidates[2 * kept] = p[0]; candidates[2 * kept + 1] = p[1]; kept++;
        }
    }
    return kept;
}
