/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 7d (part 2): verifyRecognisedPlace refinement. See ok_place.h for the notices. */
#include "ok_place.h"
#include "ok_err.h"
#include "ok_kin.h"
#include "ok_param.h"
#include "ok_solve.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

typedef struct place_end { int termination, iterations; double initial_cost, final_cost; } place_end;
static void on_end(void* ctx, const ok_sv_end* e) {
    place_end* x = (place_end*)ctx;
    x->termination = e->termination_type; x->iterations = e->num_iterations;
    x->initial_cost = e->initial_cost; x->final_cost = e->final_cost;
}

int ok_place_refine(const ok_place_term* terms, int nterms, int ncam, const double T_SC[][7], const double T0[7], int max_iters,
                    ok_place_refine_out* out) {
    ok_sv_problem pb;
    ok_sv_hooks hooks;
    place_end pe;
    double* px;                      /* parameter values: pose (7), ncam extrinsics (7), landmarks (4) */
    int* lm_block;                   /* block index of the landmark of each term */
    uint64_t* lm_ids;
    int nlm = 0, i, c, k;
    memset(&pb, 0, sizeof pb); memset(&hooks, 0, sizeof hooks); memset(&pe, 0, sizeof pe);
    hooks.on_end = on_end; hooks.ctx = &pe;
    px = (double*)calloc((size_t)(7 + 7 * ncam + 4 * (nterms ? nterms : 1)), sizeof(double));
    lm_block = (int*)calloc((size_t)(nterms ? nterms : 1), sizeof(int));
    lm_ids = (uint64_t*)calloc((size_t)(nterms ? nterms : 1), sizeof(uint64_t));
    pb.p = (ok_sv_param*)calloc((size_t)(1 + ncam + nterms), sizeof(ok_sv_param));
    pb.r = (ok_sv_resid*)calloc((size_t)(nterms ? nterms : 1), sizeof(ok_sv_resid));
    /* PoseParameterBlock(T_Sold_Snew, 1): the pose block is the free one */
    memcpy(px, T0, sizeof(double) * 7);
    pb.p[0].ptr = 1; pb.p[0].size = 7; pb.p[0].tangent = 6; pb.p[0].kind = OK_SV_KIND_POSE; pb.p[0].constant = 0; pb.p[0].x = px; pb.p[0].index = -1;
    pb.np = 1;
    for (c = 0; c < ncam; ++c) {         /* the extrinsics: SetParameterBlockConstant */
        ok_sv_param* p = &pb.p[pb.np];
        memcpy(px + 7 + 7 * c, T_SC[c], sizeof(double) * 7);
        p->ptr = (uint64_t)(2 + c); p->size = 7; p->tangent = 6; p->kind = OK_SV_KIND_POSE; p->constant = 1; p->x = px + 7 + 7 * c; p->index = -1;
        ++pb.np;
    }
    for (k = 0; k < nterms; ++k) {
        const ok_place_term* t = &terms[k];
        ok_sv_resid* rb = &pb.r[pb.nr];
        double info[4];
        int blk = -1;
        for (i = 0; i < nlm; ++i) if (lm_ids[i] == t->lm_id) { blk = lm_block[i]; break; }
        if (blk < 0) {                   /* a new HomogeneousPointParameterBlock (constant), added right before its first residual block */
            ok_sv_param* p = &pb.p[pb.np];
            double* x = px + 7 + 7 * ncam + 4 * nlm;
            memcpy(x, t->hp, sizeof(double) * 4);
            p->ptr = (uint64_t)(1000 + nlm); p->size = 4; p->tangent = 3; p->kind = OK_SV_KIND_HPOINT; p->constant = 1; p->x = x; p->index = -1;
            blk = pb.np; lm_ids[nlm] = t->lm_id; lm_block[nlm] = blk; ++nlm; ++pb.np;
        }
        info[0] = 64.0 / (t->size * t->size) * 1.0; info[1] = 64.0 / (t->size * t->size) * 0.0;
        info[2] = 64.0 / (t->size * t->size) * 0.0; info[3] = 64.0 / (t->size * t->size) * 1.0;
        rb->ptr = (uint64_t)(100000 + k); rb->type = OK_SV_T_REPROJ; rb->loss = OK_SV_LOSS_CAUCHY; rb->nb = 3; rb->nres = 2;
        rb->blk[0] = 0; rb->blk[1] = blk; rb->blk[2] = 1 + t->cam_idx;
        ok_reproj_err_init(&rb->term.reproj, t->cam, t->meas, info);
        ++pb.nr;
    }
    pb.opt.linear_solver_type = OK_SV_SPARSE_NORMAL_CHOLESKY;     /* Solver::Options defaults (EIGEN_SPARSE build) */
    pb.opt.max_num_iterations = max_iters;
    pb.opt.function_tolerance = 1e-6; pb.opt.gradient_tolerance = 1e-10; pb.opt.parameter_tolerance = 1e-8;
    pb.opt.initial_trust_region_radius = 1e4; pb.opt.max_trust_region_radius = 1e16; pb.opt.min_trust_region_radius = 1e-32;
    pb.opt.min_relative_decrease = 1e-3; pb.opt.min_lm_diagonal = 1e-6; pb.opt.max_lm_diagonal = 1e32;
    pb.opt.jacobi_scaling = 1; pb.opt.max_num_consecutive_invalid_steps = 5;
    pb.opt.strategy_lm = 1; pb.opt.cauchy_a = 3.0;
    ok_sv_solve(&pb, &hooks);
    /* T_Sold_Snew = pose->estimate(): PoseParameterBlock::estimate() is the TransformationCacheless that wraps the parameters
     * (no re-normalisation of the quaternion) */
    memcpy(out->T, px, sizeof(double) * 7);
    out->iterations = pe.iterations; out->termination = pe.termination; out->initial_cost = pe.initial_cost; out->final_cost = pe.final_cost;
    /* information: H += jacobianMinimal^T * jacobianMinimal for the terms with ||err|| <= 3 */
    memset(out->H, 0, sizeof out->H);
    out->additional_outliers = 0;
    for (k = 0; k < nterms; ++k) {
        const ok_sv_resid* rb = &pb.r[k];
        const double* pars[3];
        double err[2], jac[14], jacmin[12];
        double* jacs[3]; double* jacmins[3];
        pars[0] = px; pars[1] = pb.p[rb->blk[1]].x; pars[2] = pb.p[rb->blk[2]].x;
        jacs[0] = jac; jacs[1] = NULL; jacs[2] = NULL;
        jacmins[0] = jacmin; jacmins[1] = NULL; jacmins[2] = NULL;
        ok_reproj_err_evaluate(&rb->term.reproj, pars, err, jacs, jacmins);
        if (sqrt(err[0] * err[0] + err[1] * err[1]) > 3.0) out->additional_outliers++;
        else {
            /* EvaluateWithMinimalJacobians writes the 2 x 6 block ROW-major into `jacobianMinimal.data()`, a column-major
             * Eigen::Matrix<double,2,6> (upstream quirk): M(r, c) = buf[r + 2 c], and H += M^T M */
            for (c = 0; c < 6; ++c)
                for (i = 0; i < 6; ++i)
                    out->H[i + 6 * c] = out->H[i + 6 * c] + (jacmin[0 + 2 * i] * jacmin[0 + 2 * c] + jacmin[1 + 2 * i] * jacmin[1 + 2 * c]);
        }
    }
    free(px); free(lm_block); free(lm_ids); free(pb.p); free(pb.r);
    return 0;
}

