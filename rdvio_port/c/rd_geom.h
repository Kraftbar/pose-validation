/* SPDX-License-Identifier: Apache-2.0 AND MPL-2.0 */
/*
 * RD-VIO pure-C port, module M2b: geometry layer (rdvio_geometry/{wahba.h, essential.{h,cpp}, homography.{h,cpp}, stereo.h}) and
 * Track::triangulate / triangulation_angle / landmark point helpers (rdvio_map/track.cpp).
 * NOT in this module: the RANSAC / PARSAC drivers (find_*_matrix*, M3) and pnp.h (needs OpenCV EPnP, M8).
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0; XRSLAM, Copyright 2022 XRSLAM Authors); translated to C99, modified.
 * Eigen 3.4.0 evaluation-order models: MPL-2.0. See rdvio_port/NOTICE. Column-major 3x3 matrices, points as plain arrays.
 *
 * Dump record layouts (reference patch 0004), native endian, f64 unless noted:
 *   wahba.bin : p1[2][3], p2[2][3], R[9]
 *   ess5.bin  : p1[5][2], p2[5][2], u32 count, count x E[9]
 *   hom4.bin  : p1[4][2], p2[4][2], H[9]
 *   decess.bin: E[9], R1[9], R2[9], T[3]
 *   dechom.bin: H[9], u32 ret, R1[9], R2[9], T1[3], T2[3], n1[3], n2[3]      (outputs are written even if ret == 0)
 *   tri2.bin  : P1[12], P2[12] (3x4 column-major), pt1[3], pt2[3], out[4]
 *   trin.bin  : u32 n, n x P[12], n x pt[3], out[4]
 *   trk.bin   : u32 n, n x {pose.q[4], pose.p[3], camera.q_cs[4], camera.p_cs[3], keypoint[3]} (map order), u32 valid, [valid: landmark[3]]
 */
#ifndef RD_GEOM_H
#define RD_GEOM_H
#include "rd_lie.h"

void rd_solve_rotation_2pt(const double p1[2][3], const double p2[2][3], double R[9]);

/* up to 10 solutions (EigenSolver<10x10> real eigenvalues); returns the count, or -1 if the real Schur iteration did not converge
 * (Eigen would return unspecified values there; not modelled) */
int rd_solve_essential_5pt(const double p1[5][2], const double p2[5][2], double E[10][9]);

void rd_decompose_essential(const double E[9], double R1[9], double R2[9], double T[3]);
int rd_decompose_homography(const double H[9], double R1[9], double R2[9], double T1[3], double T2[3], double n1[3], double n2[3]);
void rd_solve_homography_4pt(const double p1[4][2], const double p2[4][2], double H[9]);

void rd_triangulate_point2(const double P1[12], const double P2[12], const double pt1[3], const double pt2[3], double out[4]);
void rd_triangulate_point_n(int n, const double* Ps /* n x 12 */, const double* pts /* n x 3 */, double out[4]);

/* Frame::get_pose(camera): q = pose.q * cam.q_cs, p = pose.p + pose.q * cam.p_cs */
void rd_frame_get_pose(const ok_quat* pose_q, const double pose_p[3], const ok_quat* cam_q, const double cam_p[3], ok_quat* q, double p[3]);

typedef struct rd_obs { ok_quat pose_q; double pose_p[3]; ok_quat cam_q; double cam_p[3]; double keypoint[3]; } rd_obs;
/* Track::triangulate over the observations in map order; returns 1 and the landmark point if valid */
int rd_track_triangulate(int n, const rd_obs* obs, double landmark[3]);

/* Track::triangulation_angle(p) (max over observations of acos(n . nref), stableNormalized directions to the camera centres) */
double rd_track_triangulation_angle(int n, const rd_obs* obs, const double p[3]);
/* Track::get_landmark_point: first observation, inverse depth */
void rd_track_get_landmark_point(const rd_obs* first, double inv_depth, double out[3]);
/* Track::set_landmark_point: returns the new inverse depth */
double rd_track_set_landmark_point(const rd_obs* first, const double p[3]);
/*   tang.bin: u32 n, n x obs, p[3], angle        glp.bin: obs (first), inv_depth, out[3]        slp.bin: obs (first), p[3], new inv_depth */

#endif
