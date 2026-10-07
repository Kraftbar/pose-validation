/* SPDX-License-Identifier: MPL-2.0 */
/* Basalt port M1: Eigen small fixed-size kernels, float and double. See bs_eigenf.h. */
#include <math.h>
#include "bs_eigenf.h"

#define S float
#define SFX f
#define SQRTF sqrtf
#define BS_LEFT3 0
#define BS_PACKET4 1
#include "bs_eigenf_impl.h"
#undef S
#undef SFX
#undef SQRTF
#undef BS_LEFT3
#undef BS_PACKET4

#define S double
#define SFX d
#define SQRTF sqrt
#define BS_LEFT3 1
#define BS_PACKET4 0
#include "bs_eigenf_impl.h"
