/* SPDX-License-Identifier: MPL-2.0 */
/* MPL-2.0. Eigen 3.4 JacobiSVD / HouseholderQR specializations for PnP. */
#ifndef SV_EIGEN_PNP_H
#define SV_EIGEN_PNP_H
#ifdef __cplusplus
extern "C" {
#endif
void sv_pnp_eigen_mul3(const double*,const double*,double*);
void sv_pnp_eigen_mixed_mul3(const double*,const double*,double*);
/* Column-major full U (rows^2), V (cols^2), singular values (cols).
 * Supported shapes rows>=cols, rows<=12, cols<=12. */
int sv_pnp_eigen_svd(const double *a,int rows,int cols,double *u,double *v,double *s);
void sv_pnp_eigen_svd_solve(const double *a,int rows,int cols,const double *b,double *x);
void sv_pnp_eigen_qr_solve(const double a[24],const double b[6],double x[4]);
void sv_pnp_eigen_gram(const double *a,int rows,int cols,double *out);
#ifdef __cplusplus
}
#endif
#endif
