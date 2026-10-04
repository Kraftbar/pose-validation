/* SPDX-License-Identifier: BSD-3-Clause */
/* OKVIS2 pure-C port, module 5c: ceres::Problem bookkeeping (program order). See ok_problem.h for the notices. */
#include "ok_problem.h"

#include <stdlib.h>
#include <string.h>

/* ---- u64 -> int open-addressing map with tombstones ---- */
static uint64_t mix(uint64_t k) { k ^= k >> 33; k *= 0xff51afd7ed558ccdULL; k ^= k >> 33; k *= 0xc4ceb9fe1a85ec53ULL; k ^= k >> 33; return k; }
static void map_init(ok_u64map* m) { memset(m, 0, sizeof *m); }
static void map_free(ok_u64map* m) { free(m->keys); free(m->vals); free(m->state); memset(m, 0, sizeof *m); }
static void map_put(ok_u64map* m, uint64_t k, int v);
static void map_rehash(ok_u64map* m, int cap) {
    ok_u64map n;
    int i;
    n.cap = cap; n.n = 0; n.used = 0;
    n.keys = (uint64_t*)calloc((size_t)cap, sizeof(uint64_t));
    n.vals = (int*)calloc((size_t)cap, sizeof(int));
    n.state = (unsigned char*)calloc((size_t)cap, 1);
    for (i = 0; i < m->cap; ++i)
        if (m->state[i] == 1) map_put(&n, m->keys[i], m->vals[i]);
    map_free(m);
    *m = n;
}
static int map_find(const ok_u64map* m, uint64_t k) {
    int i;
    if (m->cap == 0) return -1;
    i = (int)(mix(k) & (uint64_t)(m->cap - 1));
    for (;;) {
        if (m->state[i] == 0) return -1;
        if (m->state[i] == 1 && m->keys[i] == k) return m->vals[i];
        i = (i + 1) & (m->cap - 1);
    }
}
static void map_put(ok_u64map* m, uint64_t k, int v) {
    int i;
    if (m->cap == 0 || 2 * (m->used + 1) > m->cap) map_rehash(m, m->cap ? 2 * m->cap : 1024);
    i = (int)(mix(k) & (uint64_t)(m->cap - 1));
    for (;;) {
        if (m->state[i] != 1) { if (m->state[i] == 0) m->used++; m->state[i] = 1; m->keys[i] = k; m->vals[i] = v; m->n++; return; }
        if (m->keys[i] == k) { m->vals[i] = v; return; }
        i = (i + 1) & (m->cap - 1);
    }
}
static void map_del(ok_u64map* m, uint64_t k) {
    int i;
    if (m->cap == 0) return;
    i = (int)(mix(k) & (uint64_t)(m->cap - 1));
    for (;;) {
        if (m->state[i] == 0) return;
        if (m->state[i] == 1 && m->keys[i] == k) { m->state[i] = 2; m->n--; return; }
        i = (i + 1) & (m->cap - 1);
    }
}

/* ---- problem ---- */
void ok_problem_init(ok_problem* p) { memset(p, 0, sizeof *p); map_init(&p->pmap); map_init(&p->rmap); }
void ok_problem_free(ok_problem* p) {
    int i;
    for (i = 0; i < p->nparams; ++i) free(p->params[i].dep);
    free(p->params); free(p->free_p); free(p->resids); free(p->free_r); free(p->porder); free(p->rorder);
    map_free(&p->pmap); map_free(&p->rmap);
    memset(p, 0, sizeof *p);
}
int ok_problem_find_param(const ok_problem* p, uint64_t ptr) { return map_find(&p->pmap, ptr); }
int ok_problem_find_resid(const ok_problem* p, uint64_t ptr) { return map_find(&p->rmap, ptr); }

static int new_param_slot(ok_problem* p) {
    int s;
    if (p->nfree_p > 0) s = p->free_p[--p->nfree_p];
    else {
        if (p->nparams == p->capparams) {
            p->capparams = p->capparams ? 2 * p->capparams : 256;
            p->params = (ok_pb_param*)realloc(p->params, sizeof(ok_pb_param) * (size_t)p->capparams);
        }
        s = p->nparams++;
        memset(&p->params[s], 0, sizeof(ok_pb_param));
    }
    return s;
}
static int new_resid_slot(ok_problem* p) {
    int s;
    if (p->nfree_r > 0) s = p->free_r[--p->nfree_r];
    else {
        if (p->nresids == p->capresids) {
            p->capresids = p->capresids ? 2 * p->capresids : 1024;
            p->resids = (ok_pb_resid*)realloc(p->resids, sizeof(ok_pb_resid) * (size_t)p->capresids);
        }
        s = p->nresids++;
    }
    return s;
}
static void push_free(int** arr, int* n, int s) {
    *arr = (int*)realloc(*arr, sizeof(int) * (size_t)(*n + 1));
    (*arr)[(*n)++] = s;
}

int ok_problem_add_parameter_block(ok_problem* p, uint64_t ptr, int size) {
    int s = map_find(&p->pmap, ptr);
    ok_pb_param* b;
    if (s >= 0) return p->params[s].size == size ? s : -1;
    s = new_param_slot(p);
    b = &p->params[s];
    b->ptr = ptr; b->size = size; b->constant = 0; b->manifold = 0; b->alive = 1; b->ndep = 0;
    if (p->np == p->capp) { p->capp = p->capp ? 2 * p->capp : 256; p->porder = (int*)realloc(p->porder, sizeof(int) * (size_t)p->capp); }
    b->index = p->np;
    p->porder[p->np++] = s; /* program_->parameter_blocks_.push_back */
    map_put(&p->pmap, ptr, s);
    return s;
}

int ok_problem_set_manifold(ok_problem* p, uint64_t ptr, uint64_t manifold) {
    const int s = map_find(&p->pmap, ptr);
    if (s < 0) return 0;
    p->params[s].manifold = manifold;
    return 1;
}

static void dep_add(ok_pb_param* b, int rslot) {
    if (b->ndep == b->capdep) { b->capdep = b->capdep ? 2 * b->capdep : 8; b->dep = (int*)realloc(b->dep, sizeof(int) * (size_t)b->capdep); }
    b->dep[b->ndep++] = rslot;
}
static void dep_remove(ok_pb_param* b, int rslot) {
    int i;
    for (i = 0; i < b->ndep; ++i)
        if (b->dep[i] == rslot) { b->dep[i] = b->dep[--b->ndep]; return; }
}

int ok_problem_add_residual_block(ok_problem* p, uint64_t rb, uint64_t cost, uint64_t loss, int nb, const uint64_t* values) {
    int blk[OK_PB_MAXB], k, s;
    ok_pb_resid* r;
    if (nb < 1 || nb > OK_PB_MAXB) return -1;
    if (map_find(&p->rmap, rb) >= 0) return -1;
    for (k = 0; k < nb; ++k) { blk[k] = map_find(&p->pmap, values[k]); if (blk[k] < 0) return -1; }
    s = new_resid_slot(p);
    r = &p->resids[s];
    r->ptr = rb; r->cost = cost; r->loss = loss; r->nb = nb; r->alive = 1;
    for (k = 0; k < nb; ++k) r->blk[k] = blk[k];
    for (k = 0; k < nb; ++k) dep_add(&p->params[blk[k]], s); /* parameter_block->AddResidualBlock */
    if (p->nr == p->capr) { p->capr = p->capr ? 2 * p->capr : 1024; p->rorder = (int*)realloc(p->rorder, sizeof(int) * (size_t)p->capr); }
    r->index = p->nr;
    p->rorder[p->nr++] = s; /* program_->residual_blocks_.push_back */
    map_put(&p->rmap, rb, s);
    return s;
}

/* DeleteBlockInVector on the residual program vector */
static void internal_remove_residual(ok_problem* p, int s) {
    ok_pb_resid* r = &p->resids[s];
    int k, last;
    for (k = 0; k < r->nb; ++k) dep_remove(&p->params[r->blk[k]], s);
    map_del(&p->rmap, r->ptr);
    last = p->rorder[p->nr - 1];
    p->resids[last].index = r->index;
    p->rorder[r->index] = last;
    p->nr--;
    r->alive = 0;
    push_free(&p->free_r, &p->nfree_r, s);
}

int ok_problem_remove_residual_block(ok_problem* p, uint64_t rb) {
    const int s = map_find(&p->rmap, rb);
    if (s < 0) return 0;
    internal_remove_residual(p, s);
    return 1;
}

int ok_problem_remove_parameter_block(ok_problem* p, uint64_t ptr) {
    const int s = map_find(&p->pmap, ptr);
    ok_pb_param* b;
    int ndeps = 0, last;
    if (s < 0) return -1;
    b = &p->params[s];
    /* dependents in ascending program index (the reference iterates a pointer-keyed unordered_set) */
    while (b->ndep > 0) {
        int i, best = 0;
        for (i = 1; i < b->ndep; ++i)
            if (p->resids[b->dep[i]].index < p->resids[b->dep[best]].index) best = i;
        internal_remove_residual(p, b->dep[best]);
        ndeps++;
    }
    map_del(&p->pmap, ptr);
    last = p->porder[p->np - 1];
    p->params[last].index = b->index;
    p->porder[b->index] = last;
    p->np--;
    b->alive = 0;
    push_free(&p->free_p, &p->nfree_p, s);
    return ndeps;
}

int ok_problem_set_constant(ok_problem* p, uint64_t ptr, int constant) {
    const int s = map_find(&p->pmap, ptr);
    if (s < 0) return 0;
    p->params[s].constant = constant;
    return 1;
}

void ok_problem_program(const ok_problem* p, uint64_t* params, uint64_t* rbs) {
    int i;
    for (i = 0; i < p->np; ++i) params[i] = p->params[p->porder[i]].ptr;
    for (i = 0; i < p->nr; ++i) rbs[i] = p->resids[p->rorder[i]].ptr;
}
