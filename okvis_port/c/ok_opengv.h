/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 7c: the OpenGV pieces OKVIS2 uses.
 *
 * Derived from OpenGV (BSD-3-Clause, Copyright (c) 2013 Laurent Kneip, ANU; full text okvis_port/LICENSES/opengv-BSD-3-Clause.txt:
 * absolute_pose gp3p + Groebner module, relative_pose twopt_rotationOnly / fivept_stewenius, math arun / cayley, triangulation
 * triangulate2, sac Ransac / SampleConsensusProblem, sac_problems AbsolutePoseSacProblem / CentralRelativePoseSacProblem /
 * RotationOnlySacProblem) and from OKVIS2 (BSD-3-Clause, Copyright (c) 2015 Autonomous Systems Lab / ETH Zurich, 2020 Smart
 * Robotics Lab / Imperial College London, 2024 Smart Robotics Lab / Technical University of Munich;
 * okvis_frontend Frame{Absolute,RelativePose,RotationOnly}SacProblem), with the Eigen 3.4.0 evaluation-order models of ok_eigen.c /
 * ok_eigen_eigsolver.c / ok_eigen_svd.c (MPL-2.0) and the libstdc++ std::mt19937 / std::uniform_int_distribution<int> algorithms
 * (an ISO C++ standard generator; Lemire downscaling as selected by GCC 13's uniform_int_dist.h). Redistribution requires
 * retaining these notices (okvis_port/NOTICE).
 *
 * C99, <stdint.h> <math.h> <stdlib.h> <string.h> only.
 *
 * ---- record layouts of patch 0013 (ransac.bin, framed like problem.bin: u32 tag, u64 length, payload; native endian) ----
 *   163 absolute adapter  u32 kind (0 matchToMap GP3P, 3 verifyRecognisedPlace GP3P), u32 n,
 *                         n x { bearing[3], point[3], camOffset[3], camRotation[9] (column-major), sigmaAngle } f64
 *   164 relative adapter  u32 kind (1 rotation-only, 2 Stewenius), u32 n, n x { bearing1[3], bearing2[3], sigma1, sigma2 } f64
 *   162 result            as in ok_frontend.h
 * A 163 / 164 record is always followed by its 162 record.
 */
#ifndef OK_OPENGV_H
#define OK_OPENGV_H
#include <stdint.h>

/* FrameNoncentralAbsoluteAdapter / LoopclosureNoncentralAbsoluteAdapter data (n correspondences) */
typedef struct ok_og_abs {
    int n;
    const double* bearing;   /* n x 3 (unit) */
    const double* point;     /* n x 3 (world) */
    const double* offset;    /* n x 3 (camera offset, T_SC translation) */
    const double* rot;       /* n x 9 (camera rotation, column-major) */
    const double* sigma;     /* n */
} ok_og_abs;

/* FrameRelativeAdapter data */
typedef struct ok_og_rel {
    int n;
    const double* f1;        /* n x 3 */
    const double* f2;        /* n x 3 */
    const double* s1;        /* n */
    const double* s2;        /* n */
} ok_og_rel;

/* the observable state of an opengv::sac::Ransac after computeModel */
typedef struct ok_og_result {
    int iterations, ninliers;
    int* inliers;                   /* malloc'd by the callee, owned by the caller */
    int rows, cols;
    double model[16];               /* column-major (3x4 transformation / 3x3 rotation) */
    int degenerate;                 /* GP3P samples whose Groebner eigenproblem could not be solved (non-finite input, e.g. a sample
                                     * with a duplicated 3D point): the C++ then reads uninitialised EigenSolver storage (undefined
                                     * behaviour, not reproducible); the C code skips the sample. 0 for every other run. */
} ok_og_result;
extern long ok_og_eig_failures;     /* running count of such samples (ok_og_gp3p_main) */

/* ---- the std::mt19937 + std::uniform_int_distribution<int>(0, INT_MAX) generator of SampleConsensusProblem ---- */
typedef struct ok_og_rng { uint32_t state[624]; unsigned idx; } ok_og_rng;
void ok_og_rng_seed(ok_og_rng* e, uint32_t seed);
uint32_t ok_og_rng_next(ok_og_rng* e);            /* one mt19937 draw */
int ok_og_rng_rnd(ok_og_rng* e);                  /* uniform_int_distribution<int>(0, INT_MAX)(engine) */

/* ---- Ransac<Problem>::computeModel with threshold / max_iterations (probability 0.99, seed 12345, as the reference build) ----
 * Return value = the C++ bool (a model was found). out->inliers is malloc'd (NULL when empty). */
int ok_og_ransac_abs(const ok_og_abs* a, double threshold, int max_iterations, ok_og_result* out);      /* GP3P, sample size 4 */
int ok_og_ransac_rotation(const ok_og_rel* a, double threshold, int max_iterations, ok_og_result* out); /* sample size 2 */
int ok_og_ransac_stewenius(const ok_og_rel* a, double threshold, int max_iterations, ok_og_result* out);/* sample size 8 */

/* ---- building blocks (exposed for the oracle tests) ---- */
/* opengv::absolute_pose::gp3p: f / v / p are 3x3 column-major (bearing vectors already un-rotated, camera offsets, points);
 * sols receives up to 8 transformations (3x4 column-major); returns their number */
int ok_og_gp3p_main(const double f[9], const double v[9], const double p[9], double sols[8][12]);
/* AbsolutePoseSacProblem::computeModelCoefficients for GP3P (idx: 4 indices); returns the C++ bool */
int ok_og_abs_model(const ok_og_abs* a, const int idx[4], double model[12]);
void ok_og_abs_scores(const ok_og_abs* a, const double model[12], double* scores);
int ok_og_stewenius_essentials(const ok_og_rel* a, const int idx[5], double Ereal[10][9]);   /* fivept_stewenius, real parts; returns 10 (0 = solver failure) */
int ok_og_stewenius_model(const ok_og_rel* a, const int idx[8], double model[12]);
void ok_og_stewenius_scores(const ok_og_rel* a, const double model[12], double* scores);
int ok_og_rotation_model(const ok_og_rel* a, const int idx[2], double model[9]);
void ok_og_rotation_scores(const ok_og_rel* a, const double model[9], double* scores);

/* generated by tools/gen_okvis_gp3p.py */
void ok_og_gp3p_init(double* groebner, const double* f, const double* v, const double* p);
void ok_og_gp3p_compute(double* groebner);
void ok_og_stewenius_compose_a(const double* ee, double* a);   /* tools/gen_okvis_stewenius.py: 9 x 4 -> 10 x 20 */

#endif
