/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD-2-Clause, AIST 2019 / stella-cv 2022. See implementation notice. */
#ifndef SV_SOLVE_ESSENTIAL_RANSAC_H
#define SV_SOLVE_ESSENTIAL_RANSAC_H
#include "sv_rng.h"
#include "sv_solve_essential_5pt.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { int valid;float cost;double E[9];unsigned inliers; } sv_essential_result;
typedef void (*sv_essential_trace_fn)(void *user,unsigned iteration,const unsigned *indices,unsigned sample_size,const double *candidates,unsigned num_candidates,const float *costs,const unsigned *inlier_counts,const sv_essential_5pt_trace *minimal,const sv_essential_result *best);
/* Already-paired xyz bearings, n matches in upstream match order. Caller
 * supplies n mask bytes. Fixed seed by default; optional rng preserves the
 * state across calls. sample_size supports 5 or >=8; 6/7 are not implemented.
 * Returns 0 on normal completion (including no valid model), -1 on error.
 * trace is diagnostic only and may be NULL. Recompute preserves validity as
 * upstream does, even if the refined matrix has fewer inliers. */
int sv_essential_ransac(const double *b1,const double *b2,unsigned n,unsigned iterations,int recompute,unsigned sample_size,sv_mt19937 *rng,sv_essential_result *result,unsigned char *mask,sv_essential_trace_fn trace,void *user);
int sv_essential_nonminimal(const double *b1,const double *b2,unsigned n,double E[9]);
unsigned sv_essential_check_inliers(const double E[9],const double *b1,const double *b2,unsigned n,unsigned char *mask,float *cost);
#ifdef __cplusplus
}
#endif
#endif
