/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port: ViGraph::optimise on the C graph. Builds the Ceres problem from the graph's Problem bookkeeping
 * (parameter and residual blocks in PROGRAM order, constant flags, manifolds, the terms by value), runs the module-4
 * solver (ok_sv_solve) and writes the solver's changes back into the graph: parameter values in place, the IMU terms'
 * re-integration state. The result bytes are the OPT-record result layout of patch 0010 (ok_vigraph.h), so the same
 * backend code can run on a logged solver output (replay) or on this native solve.
 * Derived from OKVIS2 (BSD-3-Clause, see okvis_port/NOTICE). C99, <stdint.h> <math.h> <stdlib.h> <string.h>. */
#include <stdlib.h>
#include <string.h>
#include "ok_solve.h"
#include "ok_vslam.h"

typedef struct pmap { uint64_t ptr; int idx; } pmap;
static int cmp_pmap(const void* a, const void* b) {
    const uint64_t x = ((const pmap*)a)->ptr, y = ((const pmap*)b)->ptr;
    return x < y ? -1 : (x > y);
}
static int pmap_find(const pmap* m, int n, uint64_t ptr) {
    int lo = 0, hi = n;
    while (lo < hi) { const int mid = (lo + hi) / 2; if (m[mid].ptr < ptr) lo = mid + 1; else hi = mid; }
    return (lo < n && m[lo].ptr == ptr) ? m[lo].idx : -1;
}

typedef struct bw { unsigned char* p; size_t n, cap; } bw;
static void bw_raw(bw* b, const void* d, size_t n) {
    if (b->n + n > b->cap) { b->cap = (b->n + n) * 2 + 256; b->p = (unsigned char*)realloc(b->p, b->cap); }
    memcpy(b->p + b->n, d, n); b->n += n;
}
static void bw_u32(bw* b, uint32_t v) { bw_raw(b, &v, 4); }
static void bw_u64(bw* b, uint64_t v) { bw_raw(b, &v, 8); }

typedef struct endinfo { int termination, iterations; } endinfo;
static void on_end(void* ctx, const ok_sv_end* e) { endinfo* x = (endinfo*)ctx; x->termination = e->termination_type; x->iterations = e->num_iterations; }

static int nres_of(int type) {
    switch (type) {
        case OK_SV_T_REPROJ: return 2;
        case OK_SV_T_IMU: return 15;
        case OK_SV_T_POSE: case OK_SV_T_RELPOSE: case OK_SV_T_TWOPOSE: case OK_SV_T_TWOPOSE_CONST: return 6;
        case OK_SV_T_SAB: return 9;
        case OK_SV_T_HPOINT: return 3;
        case OK_SV_T_GPS: return 3;
        default: return 0;
    }
}

int ok_vg_solve_native(ok_vg* g, int max_iter, unsigned char** res, size_t* rlen) {
    const ok_problem* pr = ok_vg_problem(g);
    ok_vg_blkref* bl = NULL; ok_vg_imuref* il = NULL;
    int nbl = ok_vg_blocks(g, &bl), nil = ok_vg_imu_links(g, &il);
    double* shadow = (double*)malloc(sizeof(double) * 9 * (size_t)(nbl ? nbl : 1));
    uint64_t* ihash = (uint64_t*)malloc(sizeof(uint64_t) * (size_t)(nil ? nil : 1));
    uint64_t *cp = (uint64_t*)malloc(8 * (size_t)(pr->np + 1)), *cr = (uint64_t*)malloc(8 * (size_t)(pr->nr + 1));
    ok_sv_problem sp;
    pmap* pm;
    ok_sv_hooks hooks;
    endinfo ei;
    bw out;
    int i, k, ok = 1, nch = 0;
    memset(&sp, 0, sizeof sp); memset(&out, 0, sizeof out); memset(&hooks, 0, sizeof hooks); memset(&ei, 0, sizeof ei);
    for (i = 0; i < nbl; ++i) { memset(&shadow[9 * i], 0, 72); memcpy(&shadow[9 * i], bl[i].b->x, 8 * (size_t)bl[i].b->size); }
    for (i = 0; i < nil; ++i) { unsigned char* sn; const size_t n = ok_vg_imu_snapshot(il[i].e, 0, 1, &sn); ihash[i] = ok_vg_fnv(sn, n); free(sn); }
    ok_problem_program(pr, cp, cr);
    sp.np = pr->np; sp.nr = pr->nr;
    sp.p = (ok_sv_param*)calloc((size_t)(sp.np ? sp.np : 1), sizeof(ok_sv_param));
    sp.r = (ok_sv_resid*)calloc((size_t)(sp.nr ? sp.nr : 1), sizeof(ok_sv_resid));
    pm = (pmap*)malloc(sizeof(pmap) * (size_t)(sp.np ? sp.np : 1));
    for (i = 0; i < sp.np; ++i) {
        ok_vg_blk* b = (ok_vg_blk*)(uintptr_t)cp[i];
        ok_sv_param* p = &sp.p[i];
        p->ptr = cp[i]; p->size = b->size;
        p->kind = b->size == 7 ? OK_SV_KIND_POSE : (b->size == 4 ? OK_SV_KIND_HPOINT : OK_SV_KIND_NONE);
        p->tangent = b->size == 7 ? 6 : (b->size == 4 ? 3 : b->size);
        if (b->kind == 3) { p->kind = OK_SV_KIND_POSE4; p->tangent = 4; }     /* OKVIS2-X T_GW: PoseManifold4d */
        p->constant = ok_vg_is_constant(g, cp[i]) > 0;
        p->x = b->x;                                    /* the solver updates the graph's block in place */
        p->index = -1;
        pm[i].ptr = cp[i]; pm[i].idx = i;
    }
    qsort(pm, (size_t)sp.np, sizeof(pmap), cmp_pmap);
    for (i = 0; i < sp.nr && ok; ++i) {
        ok_vg_resid d;
        ok_sv_resid* rb = &sp.r[i];
        if (!ok_vg_find_resid(g, cr[i], &d)) { ok = 0; break; }
        rb->ptr = cr[i]; rb->type = d.type; rb->loss = d.loss == 2 ? OK_SV_LOSS_CAUCHY3 : (d.loss ? OK_SV_LOSS_CAUCHY : OK_SV_LOSS_NONE); rb->nb = d.nb; rb->nres = nres_of(d.type);
        for (k = 0; k < d.nb; ++k) { rb->blk[k] = pmap_find(pm, sp.np, d.blk[k]); if (rb->blk[k] < 0) ok = 0; }
        switch (d.type) {
            case OK_SV_T_REPROJ: rb->term.reproj = *(const ok_reproj_err*)d.term; break;
            case OK_SV_T_IMU: ok_vg_imu_copy(&rb->term.imu, (const ok_imu_error*)d.term); break;
            case OK_SV_T_POSE: rb->term.pose = *(const ok_pose_err*)d.term; break;
            case OK_SV_T_SAB: rb->term.sab = *(const ok_sab_err*)d.term; break;
            case OK_SV_T_RELPOSE: rb->term.relpose = *(const ok_relpose_err*)d.term; break;
            case OK_SV_T_TWOPOSE: rb->term.tp = ((const ok_twopose*)d.term)->term; break;
            case OK_SV_T_TWOPOSE_CONST: rb->term.tp = *(const ok_tp_std*)d.term; rb->term.tp.is_computed = 1; break;
            case OK_SV_T_GPS: rb->term.gps = (ok_gps_async*)(void*)d.term; break;
            default: ok = 0; break;
        }
    }
    if (ok) {
        ok_sv_options* o = &sp.opt;
        o->linear_solver_type = ok_vg_solver_type(g); o->max_num_iterations = max_iter;
        o->function_tolerance = ok_vg_function_tolerance(g); o->gradient_tolerance = 1e-10; o->parameter_tolerance = 1e-8;
        o->initial_trust_region_radius = 1e4; o->max_trust_region_radius = 1e16; o->min_trust_region_radius = 1e-32;
        o->min_relative_decrease = 1e-3; o->min_lm_diagonal = 1e-6; o->max_lm_diagonal = 1e32;
        o->jacobi_scaling = 1; o->max_num_consecutive_invalid_steps = 5;
        hooks.ctx = &ei; hooks.on_end = on_end;
        ok_sv_solve(&sp, &hooks);
        /* the IMU terms carry their re-integration state back into the graph */
        for (i = 0; i < sp.nr; ++i) {
            ok_vg_resid d;
            if (sp.r[i].type != OK_SV_T_IMU) continue;
            ok_vg_find_resid(g, cr[i], &d);
            ok_vg_imu_copy((ok_imu_error*)(void*)d.term, &sp.r[i].term.imu);
        }
        /* result bytes: changed parameter blocks (OPT block order), changed IMU terms, termination, iterations */
        {   bw ch; uint32_t nci = 0; bw ci;
            memset(&ch, 0, sizeof ch); memset(&ci, 0, sizeof ci);
            for (i = 0; i < nbl; ++i)
                if (memcmp(bl[i].b->x, &shadow[9 * i], 8 * (size_t)bl[i].b->size) != 0) { bw_u32(&ch, (uint32_t)i); bw_raw(&ch, bl[i].b->x, 8 * (size_t)bl[i].b->size); nch++; }
            bw_u32(&out, (uint32_t)nch); bw_raw(&out, ch.p, ch.n);
            for (i = 0; i < nil; ++i) {
                unsigned char* sn; size_t n = ok_vg_imu_snapshot(il[i].e, 0, 1, &sn);
                const uint64_t h = ok_vg_fnv(sn, n);
                free(sn);
                if (h != ihash[i]) {
                    bw_u64(&ci, il[i].state_id); bw_u32(&ci, il[i].e->redo ? 1u : 0u); bw_u32(&ci, (uint32_t)il[i].e->redo_counter);
                    bw_raw(&ci, il[i].e->sb_ref, 72); nci++;
                }
            }
            bw_u32(&out, nci); bw_raw(&out, ci.p, ci.n);
            bw_u32(&out, (uint32_t)ei.termination); bw_u32(&out, (uint32_t)ei.iterations);
            free(ch.p); free(ci.p);
        }
    }
    for (i = 0; i < sp.nr; ++i) if (sp.r[i].type == OK_SV_T_IMU) ok_imu_error_free(&sp.r[i].term.imu);
    free(sp.p); free(sp.r); free(pm); free(bl); free(il); free(shadow); free(ihash); free(cp); free(cr);
    if (!ok) { free(out.p); return 0; }
    *res = out.p; *rlen = out.n;
    return 1;
}

int ok_vsb_solve_native(void* ctx, int graph, ok_vg* g, int max_iter, unsigned char** res, size_t* rlen) {
    (void)ctx; (void)graph;
    return ok_vg_solve_native(g, max_iter, res, rlen);
}
