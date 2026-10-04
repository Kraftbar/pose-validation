/* SPDX-License-Identifier: BSD-2-Clause AND BSD-3-Clause */
/* BSD-2-Clause, AIST 2019 / stella-cv 2022. See sv_pnp.c. */
#ifndef SV_PNP_H
#define SV_PNP_H
#include "sv_rng.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Trace arrays are borrowed for the duration of the callback. Matrices are
 * column-major; point arrays consist of consecutive xyz triples. */
typedef void (*sv_pnp_trace_fn)(void*,const char*,const double*,unsigned);
int sv_pnp_compute_pose(const double *bearings,const double *points,unsigned count,
                        unsigned iterations,double rotation[9],double translation[3],
                        double *error,sv_pnp_trace_fn trace,void *user);
typedef struct {int valid;double rotation[9],translation[3],cost;unsigned inliers;} sv_pnp_result;
int sv_pnp_ransac(const double *bearings,const double *points,const int *octaves,unsigned count,
                  const float *scales,unsigned levels,unsigned min_inliers,unsigned iterations,
                  unsigned gn_iterations,int recompute,sv_mt19937 *rng,
                  sv_pnp_result *result,unsigned char *mask,sv_pnp_trace_fn trace,void *user);
#ifdef __cplusplus
}
#endif
#endif
