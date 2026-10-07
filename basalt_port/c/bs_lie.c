/* SPDX-License-Identifier: MIT AND BSD-3-Clause AND MPL-2.0 */
/* Basalt port M1: Sophus SO3 / SE3 + basalt-headers Jacobians, float and double. See bs_lie.h. */
#include <math.h>
#include "bs_lie.h"

#define BS_XCAT(a, b) BS_CAT(a, b)
#define BS_CAT(a, b) a##b
#define BS_PI 3.14159265358979323846   /* M_PI (double) */

#define S float
#define SFX f
#define SQRTF sqrtf
#define SINF sinf
#define COSF cosf
#define ATAN2F atan2f
#define EPS 1e-5f                      /* Sophus::Constants<float>::epsilon() */
#define EPS_SQRT sqrtf(1e-5f)          /* Constants<float>::epsilonSqrt() */
#define PI2DIV ((S)(BS_PI * BS_PI))
#define DUMMY_PREC 1e-5f               /* NumTraits<float>::dummy_precision() */
#include "bs_lie_impl.h"
#undef S
#undef SFX
#undef SQRTF
#undef SINF
#undef COSF
#undef ATAN2F
#undef EPS
#undef EPS_SQRT
#undef PI2DIV
#undef DUMMY_PREC
#undef BN
#undef BJ
#undef Q
#undef SO3
#undef SE3
#undef E3
#undef EM3
#undef EM6
#undef EQ
#undef K

#define S double
#define SFX d
#define SQRTF sqrt
#define SINF sin
#define COSF cos
#define ATAN2F atan2
#define EPS 1e-10                      /* Constants<double>::epsilon() */
#define EPS_SQRT sqrt(1e-10)
#define PI2DIV ((S)(BS_PI * BS_PI))
#define DUMMY_PREC 1e-12
#include "bs_lie_impl.h"

void bs_so3_f_from_d(const bs_so3d* q, bs_so3f* o) {
  bs_quatf r = {(float)q->x, (float)q->y, (float)q->z, (float)q->w};
  bs_so3f_from_quat(&r, o);
}
void bs_so3_d_from_f(const bs_so3f* q, bs_so3d* o) {
  bs_quatd r = {(double)q->x, (double)q->y, (double)q->z, (double)q->w};
  bs_so3d_from_quat(&r, o);
}
void bs_se3_f_from_d(const bs_se3d* a, bs_se3f* o) {
  bs_so3_f_from_d(&a->so3, &o->so3);
  o->t[0] = (float)a->t[0]; o->t[1] = (float)a->t[1]; o->t[2] = (float)a->t[2];
}
void bs_se3_d_from_f(const bs_se3f* a, bs_se3d* o) {
  bs_so3_d_from_f(&a->so3, &o->so3);
  o->t[0] = (double)a->t[0]; o->t[1] = (double)a->t[1]; o->t[2] = (double)a->t[2];
}
