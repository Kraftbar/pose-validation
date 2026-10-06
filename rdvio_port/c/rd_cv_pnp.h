/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause */
#ifndef RD_CV_PNP_H
#define RD_CV_PNP_H
#include <stdlib.h>
/* RD-VIO callback. Input doubles are rounded to Point3f/Point2f first;
 * T is the float pose promoted to double, column-major. ctx is unused. */
void rd_cv_pnp6(void *ctx,const double X[6][3],const double x[6][2],double T[16]);
void rd_cv_pnp4(void *ctx,const double X[4][3],const double x[4][2],double T[16]);
typedef struct {void (*emit)(void *,const char *,const void *,size_t);void *user;} rd_cv_pnp_trace;
/* Diagnostic/general entry for n=4 or 6. Optional rvec/tvec are solvePnP's
 * double outputs before RD-VIO's float conversion. Returns 0 on invalid n
 * or null required buffers. Native degenerate results (including NaNs) kept. */
int rd_cv_pnp(int n,const double *X,const double *x,double T[16],double *rvec,double *tvec,const rd_cv_pnp_trace *trace);
#endif
