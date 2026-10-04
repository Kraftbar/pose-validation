/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 pose-validation authors. Own code, no third-party sources. */
#include <math.h>
#include "gf_geo.h"

#define GF_WGS84_A  6378137.0
#define GF_WGS84_F  (1.0 / 298.257223563)
#define GF_PI       3.14159265358979323846

static double deg2rad(double d) { return d * (GF_PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / GF_PI); }

void gf_lla_to_ecef(double lat_deg, double lon_deg, double h, double e[3])
{
    const double e2 = GF_WGS84_F * (2.0 - GF_WGS84_F);
    double lat = deg2rad(lat_deg), lon = deg2rad(lon_deg);
    double sl = sin(lat), cl = cos(lat);
    double N = GF_WGS84_A / sqrt(1.0 - e2 * sl * sl);
    e[0] = (N + h) * cl * cos(lon);
    e[1] = (N + h) * cl * sin(lon);
    e[2] = (N * (1.0 - e2) + h) * sl;
}

void gf_ecef_to_lla(const double e[3], double *lat_deg, double *lon_deg, double *h)
{
    const double e2 = GF_WGS84_F * (2.0 - GF_WGS84_F);
    double p = sqrt(e[0] * e[0] + e[1] * e[1]);
    double lon = atan2(e[1], e[0]);
    double lat = atan2(e[2], p * (1.0 - e2));   /* start */
    double hh = 0.0;
    for (int it = 0; it < 8; ++it) {
        double sl = sin(lat);
        double N = GF_WGS84_A / sqrt(1.0 - e2 * sl * sl);
        hh = (fabs(lat) < 1.4) ? p / cos(lat) - N : e[2] / sl - N * (1.0 - e2);
        lat = atan2(e[2], p * (1.0 - e2 * N / (N + hh)));
    }
    *lat_deg = rad2deg(lat); *lon_deg = rad2deg(lon); *h = hh;
}

void gf_enu_frame_init(gf_enu_frame *f, double lat0, double lon0, double h0)
{
    double la = deg2rad(lat0), lo = deg2rad(lon0);
    double sa = sin(la), ca = cos(la), so = sin(lo), co = cos(lo);
    f->lat0 = lat0; f->lon0 = lon0; f->h0 = h0;
    gf_lla_to_ecef(lat0, lon0, h0, f->ecef0);
    /* rows: east, north, up */
    f->R[0] = -so;       f->R[1] = co;        f->R[2] = 0.0;
    f->R[3] = -sa * co;  f->R[4] = -sa * so;  f->R[5] = ca;
    f->R[6] = ca * co;   f->R[7] = ca * so;   f->R[8] = sa;
}

void gf_ecef_to_enu(const gf_enu_frame *f, const double e[3], double o[3])
{
    double d[3] = { e[0] - f->ecef0[0], e[1] - f->ecef0[1], e[2] - f->ecef0[2] };
    for (int i = 0; i < 3; ++i)
        o[i] = f->R[3 * i] * d[0] + f->R[3 * i + 1] * d[1] + f->R[3 * i + 2] * d[2];
}

void gf_enu_to_ecef(const gf_enu_frame *f, const double v[3], double e[3])
{
    for (int i = 0; i < 3; ++i)   /* R^T v */
        e[i] = f->ecef0[i] + f->R[i] * v[0] + f->R[3 + i] * v[1] + f->R[6 + i] * v[2];
}

void gf_lla_to_enu(const gf_enu_frame *f, double lat, double lon, double h, double enu[3])
{
    double e[3];
    gf_lla_to_ecef(lat, lon, h, e);
    gf_ecef_to_enu(f, e, enu);
}

void gf_enu_to_lla(const gf_enu_frame *f, const double enu[3], double *lat, double *lon, double *h)
{
    double e[3];
    gf_enu_to_ecef(f, enu, e);
    gf_ecef_to_lla(e, lat, lon, h);
}
