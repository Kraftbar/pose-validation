/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 * C adaptation of Eigen 3.4 FullPivLU.h and TriangularSolverMatrix.h.
 * Copyright (C) 2006-2009 Benoit Jacob; 2009 Gael Guennebaud. */
#ifndef SV_EIGEN_FULLPIVLU_H
#define SV_EIGEN_FULLPIVLU_H
#ifdef __cplusplus
extern "C" {
#endif
/* Square matrices of order <=10, column-major; SSE2 reference order. */
typedef struct {
    double a[100],maxpivot;
    int n,rank,nonzero,p[10],q[10];
} sv_eigen_fullpivlu;
int sv_eigen_fullpivlu_compute(const double *a,int n,sv_eigen_fullpivlu *lu);
/* out is n*(n-rank) column-major, or n zeros for full rank. */
int sv_eigen_fullpivlu_kernel(const sv_eigen_fullpivlu *lu,double *out);
/* b and out are n*cols; cols <=10. */
int sv_eigen_fullpivlu_solve(const sv_eigen_fullpivlu *lu,const double *b,int cols,double *out);
#ifdef __cplusplus
}
#endif
#endif
