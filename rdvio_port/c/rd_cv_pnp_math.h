/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause */
#ifndef RD_CV_PNP_MATH_H
#define RD_CV_PNP_MATH_H
/* Internal, row-major, m>=n, m,n<=12; U transposed is n*m. */
void rd_cv_pnp_svd(const double *a,int m,int n,double *w,double *ut,double *vt);
void rd_cv_pnp_solve(const double *a,int m,int n,const double *b,double *x);
void rd_cv_pnp_inverse(const double *a,int n,double *inverse);
void rd_cv_pnp_mtm(const double *a,int m,int n,double *ata);
void rd_cv_pnp_rodrigues_vector(const double R[9],double r[3]);
void rd_cv_pnp_rodrigues_float(const float r[3],float R[9]);
#endif
