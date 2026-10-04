/* SPDX-License-Identifier: MPL-2.0 */
/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. See https://mozilla.org/MPL/2.0/.
 * Copyright (C) 2008-2009 Gael Guennebaud; 2010,2012 Jitse Niesen.
 * Eigen 3.4 real EigenSolver specialization for 10x10, SSE2 order. */
#ifndef SV_EIGEN_EIGENSOLVER_H
#define SV_EIGEN_EIGENSOLVER_H
#ifdef __cplusplus
extern "C" {
#endif
/* Column-major matrices throughout. Independent stages exposed for traces. */
void sv_eigen_hessenberg10(const double a[100],double h[100],double q[100]);
int sv_eigen_realschur10(const double a[100],double t[100],double u[100]);
int sv_eigen_eigensolver10(const double a[100],double real[10],double imag[10],double vr[100],double vi[100]);
#ifdef __cplusplus
}
#endif
#endif
