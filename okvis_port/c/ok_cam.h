/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/*
 * OKVIS2 pure-C port, module 2c: camera models (okvis_cv: PinholeCamera<RadialTangentialDistortion |
 * EquidistantDistortion | NoDistortion>) and NCameraSystem overlap computation.
 *
 * Derived from OKVIS2 (okvis_cv/include/okvis/cameras/{CameraBase,PinholeCamera,RadialTangentialDistortion,
 * EquidistantDistortion,NoDistortion,NCameraSystem}.hpp, implementation, src/NCameraSystem.cpp):
 *   Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *   Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *   Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *   BSD-3-Clause (see okvis_port/LICENSES/okvis2-BSD-3-Clause.txt). The evaluation-order models of the Eigen 3.4.0
 *   expressions (2x2 inverse, small products) are MPL-2.0 (Eigen, Copyright (C) Gael Guennebaud, Benoit Jacob and
 *   the Eigen authors). Redistribution requires retaining these notices; the names of ETH Zurich, Imperial College
 *   London and TUM may not be used to endorse derived products.
 *
 * C99, <math.h> <stdlib.h> <string.h> <stdint.h> only. Matrices are column-major. Not ported: RadialTangential8
 * (not used by any shipped EuRoC/TUM-VI config), image masks (the reference configs never set one: isMasked ==
 * !isInImage), undistort maps (OpenCV remap, unused by the estimator), the batch variants.
 * The "undistortion failed" console message of RadialTangentialDistortion::undistort is not reproduced.
 *
 * Camera dump records (patch 0005; native endian). Common header "cam":
 *   u32 distortion tag (0 radtan, 1 equidistant, 2 radtan8, 3 none), u32 w, u32 h, f64 fu fv cu cv, u32 nd, f64 d[nd]
 *   kin_proj.bin   : cam, f64 p[3], i32 status, u32 written, [f64 img[2]]                       project(p, &img)
 *   kin_projj.bin  : cam, f64 p[3], u32 has_intr, i32 status, u32 written,
 *                    [f64 img[2], f64 J[6] (2x3), (has_intr: u32 cols, f64 Ji[2*cols])]        project(p, &img, &J, &Ji)
 *   kin_projx.bin  : cam, f64 p[3], u32 np, f64 params[np], u32 has_pj, u32 has_intr, i32 status, u32 written,
 *                    [f64 img[2], (has_pj: f64 J[6]), (has_intr: u32 cols, f64 Ji[2*cols])]     projectWithExternalParameters
 *   kin_projh.bin  : cam, f64 p[4], i32 status, u32 written, [f64 img[2]]                       projectHomogeneous(p, &img)
 *   kin_projhj.bin : cam, f64 p[4], u32 has_intr, i32 status, u32 written,
 *                    [f64 img[2], f64 J[8] (2x4), (has_intr: u32 cols, f64 Ji[2*cols])]        projectHomogeneous(.., &J, &Ji)
 *   kin_projhx.bin : like kin_projx with a 4-vector point and J 2x4                             projectHomogeneousWithExternalParameters
 *   kin_back.bin   : cam, f64 ip[2], u32 ok, f64 dir[3]                                          backProject
 *   kin_backj.bin  : cam, f64 ip[2], u32 ok, f64 dir[3], f64 J[6] (3x2)                          backProject(.., &J)
 *   kin_backh.bin  : cam, f64 ip[2], u32 ok, f64 dir[4]                                          backProjectHomogeneous
 *   kin_backhj.bin : cam, f64 ip[2], u32 ok, f64 dir[4], f64 J[8] (4x2)
 *   kin_overlap.bin: u32 n, n x {u32 tag, u32 w, u32 h, f64 fu fv cu cv, u32 nd, f64 d[nd]},
 *                    n x {f64 T_SC coeffs[7], f64 C[9]}, n*n x {u32 overlaps, u32 w, u32 h, u8 mask[w*h] (row-major)}
 *                    (index [seenBy][camera])                                                     NCameraSystem::computeOverlaps
 */
#ifndef OK_CAM_H
#define OK_CAM_H

#include <stdint.h>
#include "ok_kin.h"

#define OK_CAM_RADTAN 0
#define OK_CAM_EQUIDISTANT 1
#define OK_CAM_NODIST 3
#define OK_CAM_MAX_DIST 4

typedef enum ok_proj_status {   /* okvis::cameras::ProjectionStatus */
    OK_PROJ_SUCCESSFUL = 0, OK_PROJ_OUTSIDE_IMAGE = 1, OK_PROJ_MASKED = 2, OK_PROJ_BEHIND = 3, OK_PROJ_INVALID = 4
} ok_proj_status;

typedef struct ok_cam {
    int dist;                      /* OK_CAM_* */
    int nd;                        /* number of distortion parameters (4, 4, 0) */
    int w, h;
    double fu, fv, cu, cv;
    double d[OK_CAM_MAX_DIST];     /* distortion parameters */
    double one_over_fu, one_over_fv;
} ok_cam;

/* PinholeCamera ctor / setIntrinsics (d may be NULL for OK_CAM_NODIST). nd is implied by dist. */
void ok_cam_init(ok_cam* c, int dist, int w, int h, double fu, double fv, double cu, double cv, const double* d);
int ok_cam_num_intrinsics(const ok_cam* c); /* 4 + nd */

/* All projections: status as in the C++ code. Outputs are written exactly where the C++ code writes them
 * (see the "written" flags of the dumps): img/J are untouched when the status is OK_PROJ_INVALID because |z| < 1e-12
 * (and, for the variants without Jacobian, also when the distortion model fails).
 * J: 2x3 (2x4 homogeneous) column-major or NULL where the C++ API takes a pointer; Ji: 2 x (4+nd) column-major or NULL. */
ok_proj_status ok_cam_project(const ok_cam* c, const double p[3], double img[2]);
ok_proj_status ok_cam_project_j(const ok_cam* c, const double p[3], double img[2], double J[6], double* Ji);
ok_proj_status ok_cam_project_ext(const ok_cam* c, const double p[3], const double* params, double img[2],
                                  double* J, double* Ji);
ok_proj_status ok_cam_project_h(const ok_cam* c, const double p[4], double img[2]);
ok_proj_status ok_cam_project_h_j(const ok_cam* c, const double p[4], double img[2], double J[8], double* Ji);
ok_proj_status ok_cam_project_h_ext(const ok_cam* c, const double p[4], const double* params, double img[2],
                                    double* J, double* Ji);

/* back-projection (direction with z = 1); return value = undistortion success */
int ok_cam_back_project(const ok_cam* c, const double ip[2], double dir[3]);
int ok_cam_back_project_j(const ok_cam* c, const double ip[2], double dir[3], double J[6] /* 3x2 */);
int ok_cam_back_project_h(const ok_cam* c, const double ip[2], double dir[4]);
int ok_cam_back_project_h_j(const ok_cam* c, const double ip[2], double dir[4], double J[8] /* 4x2 */);

/* PinholeCamera::initialiseCameraAwarenessMaps (the inputs of BRISK's camera-aware extraction): per pixel (u, v),
 * row-major, the normalised back-projected ray (zero if back-projection fails) as 3 floats and the 2x3 projection
 * Jacobian of that ray, row-major, as 6 floats. Upstream leaves a Jacobian uninitialised (cv::Mat) where the
 * projection fails (border pixels whose ray re-projects outside the image: 309 / 715 on EuRoC cam0 / cam1); the
 * reference's freshly mmap'd 8.7 MB buffer holds zeros there, which the port writes. Returns the number of such pixels. */
long ok_cam_awareness_maps(const ok_cam* c, float* rays /* h*w*3 */, float* jacobians /* h*w*6 */);

/* distortion layer (exposed for the random tests): params = k1,k2,p1,p2 (or NULL = the camera's own) */
int ok_dist_distort(const ok_cam* c, const double* params, const double u[2], double out[2], double J[4], double* Jp);
int ok_dist_undistort(const ok_cam* c, const double pd[2], double out[2]);
int ok_dist_undistort_j(const ok_cam* c, const double pd[2], double out[2], double J[4]);

/* ---- NCameraSystem::computeOverlaps ---- */
#define OK_NCAM_MAX 4
typedef struct ok_ncam {
    int n;
    ok_cam cam[OK_NCAM_MAX];
    ok_tf T_SC[OK_NCAM_MAX];        /* cached Transformation, i.e. C valid */
    uint8_t* mask[OK_NCAM_MAX][OK_NCAM_MAX];  /* [seenBy][camera], h x w row-major, malloc'ed */
    int overlaps[OK_NCAM_MAX][OK_NCAM_MAX];
} ok_ncam;

void ok_ncam_init(ok_ncam* s);
int ok_ncam_add(ok_ncam* s, const ok_cam* cam, const ok_tf* T_SC);   /* does not compute overlaps */
int ok_ncam_compute_overlaps(ok_ncam* s);                            /* 0 on success */
void ok_ncam_free(ok_ncam* s);

#endif
