/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources.
 *
 * gf_geo.h : WGS-84 geodetic helpers. LLA (deg, deg, m) <-> ECEF <-> local ENU. C99, libm only.
 */
#ifndef GF_GEO_H
#define GF_GEO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Local tangent-plane (ENU) frame anchored at a geodetic origin. */
typedef struct gf_enu_frame {
    double lat0, lon0, h0;  /* origin, deg / deg / m (ellipsoidal height) */
    double ecef0[3];        /* origin in ECEF */
    double R[9];            /* row-major ECEF->ENU rotation */
} gf_enu_frame;

void gf_lla_to_ecef(double lat_deg, double lon_deg, double h, double ecef[3]);
/* Iterative (Bowring-style, converges to < 1e-9 m for terrestrial heights). */
void gf_ecef_to_lla(const double ecef[3], double *lat_deg, double *lon_deg, double *h);

void gf_enu_frame_init(gf_enu_frame *f, double lat0_deg, double lon0_deg, double h0);
void gf_ecef_to_enu(const gf_enu_frame *f, const double ecef[3], double enu[3]);
void gf_enu_to_ecef(const gf_enu_frame *f, const double enu[3], double ecef[3]);
void gf_lla_to_enu(const gf_enu_frame *f, double lat_deg, double lon_deg, double h, double enu[3]);
void gf_enu_to_lla(const gf_enu_frame *f, const double enu[3], double *lat_deg, double *lon_deg, double *h);

#ifdef __cplusplus
}
#endif
#endif
