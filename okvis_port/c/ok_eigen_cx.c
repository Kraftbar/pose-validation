/* SPDX-License-Identifier: MPL-2.0 */
/* std::complex<double> arithmetic as compiled by the reference build (GCC 13, libstdc++, -O2 without -ffast-math, SSE2):
 * division = libgcc __divdc3 (Smith's ratio method with the Baudin-Smith range guards, GCC >= 12), multiplication = the plain
 * formula with the NaN-recovery branch (not needed for finite inputs), sqrt = glibc csqrt. Written from the published
 * algorithms (M. Baudin, R. L. Smith, "A robust complex division in Scilab", 2012); finite inputs only. */
#include <float.h>
#include <math.h>
#include "ok_eigen_eigsolver.h"

#define CX_RBIG (DBL_MAX / 2.0)
#define CX_RMIN (DBL_MIN)
#define CX_RMIN2 (DBL_EPSILON)
#define CX_RMINSCAL (1.0 / DBL_EPSILON)
#define CX_RMAX2 (CX_RBIG * CX_RMIN2)

void ok_cdiv(double a, double b, double c, double d, double* re, double* im) {
    double x, y, ratio, denom;
    if (fabs(c) < fabs(d)) {
        if (fabs(d) >= CX_RBIG) { a = a / 2; b = b / 2; c = c / 2; d = d / 2; }
        if (fabs(d) < CX_RMIN2) { a = a * CX_RMINSCAL; b = b * CX_RMINSCAL; c = c * CX_RMINSCAL; d = d * CX_RMINSCAL; }
        else if (((fabs(a) < CX_RMIN) && (fabs(b) < CX_RMAX2) && (fabs(d) < CX_RMAX2)) ||
                 ((fabs(b) < CX_RMIN) && (fabs(a) < CX_RMAX2) && (fabs(d) < CX_RMAX2))) {
            a = a * CX_RMINSCAL; b = b * CX_RMINSCAL; c = c * CX_RMINSCAL; d = d * CX_RMINSCAL;
        }
        ratio = c / d; denom = (c * ratio) + d;
        if (fabs(ratio) > CX_RMIN) { x = ((a * ratio) + b) / denom; y = ((b * ratio) - a) / denom; }
        else { x = ((c * (a / d)) + b) / denom; y = ((c * (b / d)) - a) / denom; }
    } else {
        if (fabs(c) >= CX_RBIG) { a = a / 2; b = b / 2; c = c / 2; d = d / 2; }
        if (fabs(c) < CX_RMIN2) { a = a * CX_RMINSCAL; b = b * CX_RMINSCAL; c = c * CX_RMINSCAL; d = d * CX_RMINSCAL; }
        else if (((fabs(a) < CX_RMIN) && (fabs(b) < CX_RMAX2) && (fabs(c) < CX_RMAX2)) ||
                 ((fabs(b) < CX_RMIN) && (fabs(a) < CX_RMAX2) && (fabs(c) < CX_RMAX2))) {
            a = a * CX_RMINSCAL; b = b * CX_RMINSCAL; c = c * CX_RMINSCAL; d = d * CX_RMINSCAL;
        }
        ratio = d / c; denom = (d * ratio) + c;
        if (fabs(ratio) > CX_RMIN) { x = ((b * ratio) + a) / denom; y = (b - (a * ratio)) / denom; }
        else { x = ((d * (b / c)) + a) / denom; y = (b - (d * (a / c))) / denom; }
    }
    *re = x; *im = y;
}

void ok_cmul(double a, double b, double c, double d, double* re, double* im) {
    const double ac = a * c, bd = b * d, ad = a * d, bc = b * c;
    *re = ac - bd; *im = ad + bc;
}

/* glibc csqrt for finite arguments with no extreme exponents: Re = sqrt((|z| + |x|) / 2), the sign of Im follows y;
 * the cancellation-free branch picks which component is computed first (x >= 0) */
void ok_csqrt(double a, double b, double* re, double* im) {
    double d, r, s;
    if (b == 0.0) {
        if (a < 0.0) { *re = 0.0; *im = copysign(sqrt(-a), b); }
        else { *re = fabs(sqrt(a)); *im = copysign(0.0, b); }
        return;
    }
    if (a == 0.0) {
        r = sqrt(0.5 * fabs(b));
        *re = r; *im = copysign(r, b);
        return;
    }
    d = hypot(a, b);
    if (a > 0.0) { r = sqrt(0.5 * (d + a)); s = 0.5 * (b / r); }
    else { s = sqrt(0.5 * (d - a)); r = fabs(0.5 * (b / s)); }
    *re = r; *im = copysign(s, b);
}
