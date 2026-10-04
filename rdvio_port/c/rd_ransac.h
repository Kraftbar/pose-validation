/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M3b: RANSAC and PARSAC drivers over the geometry solvers
 * (rdvio_util/ransac.h, rdvio_util/parsac.h, rdvio_geometry/src/essential.cpp: find_essential_matrix[_parsac], find_rotation_matrix,
 * find_homography_matrix[_parsac], and the geometric error functions of essential.h / homography.h).
 *
 * NOT in this module: IMU_PARSAC + pnp.h (find_pnp_matrix_parsac_imu: needs OpenCV EPnP, module M8), the Poisson-disk filter (rd_poisson.c).
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified.
 * Contracts that mirror the C++ (including its undefined corners):
 *  - if no model is accepted (size < DoF, or no hypothesis with a positive inlier count) the C++ returns an UNINITIALISED matrix; the C
 *    port returns zeros. The inlier mask is exact in all cases.
 *  - PARSAC bins the second point set on a 20x20 grid over [-1,1)^2 without bounds checks (out-of-range bins read out of bounds in C++);
 *    the port requires |x|, |y| < 1 and aborts (returns -1) otherwise.
 *  - PARSAC keeps a persistent per-grid-bin confidence vector (`static std::vector<float> binConfidences(400, 0.5)` inside
 *    find_*_parsac): rd_parsac_state, one per matrix kind (essential / homography), initialised by rd_parsac_state_init.
 *
 * Dump record layouts (reference patch 0005-m3-ransac-dump.patch), native endian:
 *   fess.bin  : u32 n, f64 threshold, f64 confidence, u64 max_iteration, i32 seed, n x p1[2], n x p2[2], f64 E[9], n x u8 mask
 *   frot.bin  : u32 n, f64 threshold, f64 confidence, u64 max_iteration, i32 seed, n x p1[3], n x p2[3], f64 R[9], n x u8 mask
 *   fhom.bin  : as fess.bin with H
 *   pess.bin / phom.bin (parsac): as fess.bin, but with f32 conf_in[400] before the points and f32 conf_out[400] after the mask
 */
#ifndef RD_RANSAC_H
#define RD_RANSAC_H
#include <stddef.h>

double rd_essential_geometric_error(const double E[9], const double p1[2], const double p2[2]);
double rd_homography_geometric_error(const double H[9], const double p1[2], const double p2[2]);
double rd_rotation_error(const double R[9], const double p1[3], const double p2[3]);  /* acos((R * p1).dot(p2)) */

/* Points are plain arrays: p1[i] = &p1[2*i] (or 3*i for the rotation). Each function returns the inlier count and fills mask[n] (0/1). */
size_t rd_find_essential_matrix(size_t n, const double* p1, const double* p2, char* mask, double threshold, double confidence,
                                size_t max_iteration, int seed, double E[9]);
size_t rd_find_rotation_matrix(size_t n, const double* p1, const double* p2, char* mask, double threshold, double confidence,
                               size_t max_iteration, int seed, double R[9]);
size_t rd_find_homography_matrix(size_t n, const double* p1, const double* p2, char* mask, double threshold, double confidence,
                                 size_t max_iteration, int seed, double H[9]);

typedef struct rd_parsac_state { float conf[400]; } rd_parsac_state;
void rd_parsac_state_init(rd_parsac_state* s);   /* 400 x 0.5f */
/* Returns the inlier count, or (size_t)-1 if a second-set point lies outside (-1,1). */
size_t rd_find_essential_matrix_parsac(rd_parsac_state* st, size_t n, const double* p1, const double* p2, char* mask, double threshold,
                                       double confidence, size_t max_iteration, int seed, double E[9]);
size_t rd_find_homography_matrix_parsac(rd_parsac_state* st, size_t n, const double* p1, const double* p2, char* mask, double threshold,
                                        double confidence, size_t max_iteration, int seed, double H[9]);
#endif
