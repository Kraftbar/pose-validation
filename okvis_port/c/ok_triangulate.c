/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 7b: triangulation::triangulateFast (okvis_frontend/src/stereo_triangulation.cpp).
 * Derived from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart Robotics Lab /
 * Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich; see
 * okvis_port/LICENSES/okvis2-BSD-3-Clause.txt) with the Eigen 3.4.0 evaluation-order models of ok_eigen.c (MPL-2.0).
 * Redistribution requires retaining these notices. C99, <math.h> <string.h> only. */
#include "ok_frontend.h"
#include <math.h>
#include <string.h>

static double dot3(const double a[3], const double b[3]) { return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]; }
static void sub3(const double a[3], const double b[3], double o[3]) { o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2]; }
static void add3(const double a[3], const double b[3], double o[3]) { o[0] = a[0] + b[0]; o[1] = a[1] + b[1]; o[2] = a[2] + b[2]; }
static void scale3(double s, const double a[3], double o[3]) { o[0] = s * a[0]; o[1] = s * a[1]; o[2] = s * a[2]; }
static void div3(const double a[3], double s, double o[3]) { o[0] = a[0] / s; o[1] = a[1] / s; o[2] = a[2] / s; }
static void nrm3(const double a[3], double o[3]) { double t[3]; memcpy(t, a, sizeof t); ok_v3_normalized(a, t); memcpy(o, t, sizeof t); }
static double norm3(const double a[3]) { return ok_v3_norm(a); }
static double stdmax(double a, double b) { return (a < b) ? b : a; }

/* ------------------------------------------------------------------------------------------------------------------
 * triangulation::triangulateFast (stereo_triangulation.cpp)
 * ---------------------------------------------------------------------------------------------------------------- */
void ok_fe_triangulate_fast(const double p1[3], const double e1[3], const double p2[3], const double e2[3], double sigma,
                            int* is_valid, int* is_parallel, double hp[4]) {
    double t12[3], b[2], A[4], Ai[4], lambda[2], det, invdet, c26, mid[3];
    *is_parallel = 0;
    *is_valid = 1;
    sub3(p2, p1, t12);
    b[0] = dot3(t12, e1);
    b[1] = dot3(t12, e2);
    A[0] = dot3(e1, e1);                       /* A(0, 0) */
    A[1] = dot3(e1, e2);                       /* A(1, 0) */
    A[2] = -A[1];                              /* A(0, 1) */
    A[3] = -dot3(e2, e2);                      /* A(1, 1) */
    /* computeInverseWithCheck(A_inverse, invertible, 1.0e-12): 2x2 closed form */
    det = A[0] * A[3] - A[1] * A[2];
    {
        const int invertible = fabs(det) > 1.0e-12;
        invdet = 1.0 / det;
        Ai[0] = A[3] * invdet; Ai[1] = -A[1] * invdet; Ai[2] = -A[2] * invdet; Ai[3] = A[0] * invdet;
        lambda[0] = Ai[0] * b[0] + Ai[2] * b[1];
        lambda[1] = Ai[1] * b[0] + Ai[3] * b[1];
        c26 = cos(2.6 * sigma);
        if (!invertible || lambda[0] < 0.01 || lambda[1] < 0.01) {
            double m[3], half[3], esum[3], s, dd[3], t[3];
            *is_parallel = 1;
            *is_valid = 1;
            scale3(0.5, t12, half);
            add3(p1, half, m);
            add3(e1, e2, esum);
            s = 40.0 * stdmax(0.01, norm3(t12));
            scale3(s, esum, t);
            add3(m, t, mid);
            hp[0] = mid[0]; hp[1] = mid[1]; hp[2] = mid[2]; hp[3] = 1.0;
            sub3(mid, p1, dd); nrm3(dd, dd);
            if (dot3(e1, dd) < c26) *is_valid = 0;
            sub3(mid, p2, dd); nrm3(dd, dd);
            if (dot3(e2, dd) < c26) *is_valid = 0;
            return;
        }
    }
    {
        double xm[3], xn[3], s[3], d1[3], d2[3], t[3];
        scale3(lambda[0], e1, t); add3(t, p1, xm);
        scale3(lambda[1], e2, t); add3(t, p2, xn);
        add3(xm, xn, s); div3(s, 2.0, mid);
        sub3(mid, p1, d1); nrm3(d1, d1);
        sub3(mid, p2, d2); nrm3(d2, d2);
        if (dot3(e1, d1) < c26) *is_valid = 0;
        if (dot3(e2, d2) < c26) *is_valid = 0;
        if (dot3(d2, d1) > cos(6.0 * sigma)) *is_parallel = 1;
        hp[0] = mid[0]; hp[1] = mid[1]; hp[2] = mid[2]; hp[3] = 1.0;
    }
}

