/* SPDX-License-Identifier: BSD-2-Clause AND MIT
 * Polynomial code (c) 2011 libmv authors.
 * stella_vslam adapter: BSD-2, AIST 2019 / stella-cv 2022. */
#ifndef SV_SOLVE_ESSENTIAL_5PT_H
#define SV_SOLVE_ESSENTIAL_5PT_H
#ifdef __cplusplus
extern "C" {
#endif
/* Matrices column-major; bearings are five consecutive xyz triples. */
typedef struct {
 double constraint[81],basis[36],polynomial[200],eliminated[100],action[100];
 double eigen_real[10],eigen_imag[10],vectors_real[100],vectors_imag[100];
 int rank,count;
} sv_essential_5pt_trace;
void sv_essential_5pt_polynomial(const double basis[36],double out[200]);
/* Returns number of real candidates (0..10), -1 on invalid/decomposition
 * failure. trace optional. Output holds up to 10 consecutive 3x3 matrices. */
int sv_essential_5pt(const double *bearing1,const double *bearing2,double out[90],sv_essential_5pt_trace *trace);
#ifdef __cplusplus
}
#endif
#endif
