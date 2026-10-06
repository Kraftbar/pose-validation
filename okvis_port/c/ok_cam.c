/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 2c: camera models. See ok_cam.h for notices.
 * "Math on paper": one equation per line in the evaluation order of the reference build. */
#include "ok_cam.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define M2(a, i, j) (a)[(i) + 2 * (j)]

void ok_cam_init(ok_cam* c, int dist, int w, int h, double fu, double fv, double cu, double cv, const double* d) {
    int k;
    memset(c, 0, sizeof *c);
    c->dist = dist;
    c->nd = dist == OK_CAM_NODIST ? 0 : 4;
    c->w = w; c->h = h;
    c->fu = fu; c->fv = fv; c->cu = cu; c->cv = cv;
    for (k = 0; k < c->nd; ++k) c->d[k] = d[k];
    c->one_over_fu = 1.0 / fu;
    c->one_over_fv = 1.0 / fv;
}

int ok_cam_num_intrinsics(const ok_cam* c) { return 4 + c->nd; }

static int is_in_image(const ok_cam* c, const double ip[2]) {
    if (ip[0] < 0.0 || ip[1] < 0.0) return 0;
    if (ip[0] >= c->w || ip[1] >= c->h) return 0;
    return 1;
}

/* ------------------------------------------------------------------------------------------------ */
/* distortion models                                                                                 */
/* ------------------------------------------------------------------------------------------------ */

/* RadialTangentialDistortion::distort / distortWithExternalParameters. J (2x2) and Jp (2x4) optional. */
static int radtan_distort(const double* p, const double u[2], double out[2], double* J, double* Jp) {
    const double k1 = p[0], k2 = p[1], p1 = p[2], p2 = p[3];
    const double u0 = u[0];
    const double u1 = u[1];
    const double mx_u = u0 * u0;
    const double my_u = u1 * u1;
    const double mxy_u = u0 * u1;
    const double rho_u = mx_u + my_u;
    const double rad_dist_u = k1 * rho_u + k2 * rho_u * rho_u;
    out[0] = u0 + u0 * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho_u + 2.0 * mx_u);
    out[1] = u1 + u1 * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho_u + 2.0 * my_u);
    if (J) {
        M2(J, 0, 0) = 1 + rad_dist_u + k1 * 2.0 * mx_u + k2 * rho_u * 4 * mx_u + 2.0 * p1 * u1 + 6 * p2 * u0;
        M2(J, 1, 0) = k1 * 2.0 * u0 * u1 + k2 * 4 * rho_u * u0 * u1 + p1 * 2.0 * u0 + 2.0 * p2 * u1;
        M2(J, 0, 1) = M2(J, 1, 0);
        M2(J, 1, 1) = 1 + rad_dist_u + k1 * 2.0 * my_u + k2 * rho_u * 4 * my_u + 6 * p1 * u1 + 2.0 * p2 * u0;
    }
    if (J && Jp) {
        const double r2 = rho_u;
        const double r4 = r2 * r2;
        M2(Jp, 0, 0) = u0 * r2;
        M2(Jp, 0, 1) = u0 * r4;
        M2(Jp, 0, 2) = 2.0 * u0 * u1;
        M2(Jp, 0, 3) = r2 + 2.0 * u0 * u0;

        M2(Jp, 1, 0) = u1 * r2;
        M2(Jp, 1, 1) = u1 * r4;
        M2(Jp, 1, 2) = r2 + 2.0 * u1 * u1;
        M2(Jp, 1, 3) = 2.0 * u0 * u1;
    }
    return 1;
}

/* EquidistantDistortion::distort / distortWithExternalParameters. J (2x2) and Jp (2x4) optional. */
static int equi_distort(const double* p, const double u[2], double out[2], double* J, double* Jp) {
    const double k1 = p[0], k2 = p[1], k3 = p[2], k4 = p[3];
    const double u0 = u[0];
    const double u1 = u[1];
    const double r = sqrt(u0 * u0 + u1 * u1);
    const double theta = atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double thetad = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const double scaling = (r > 1e-8) ? thetad / r : 1.0;
    out[0] = scaling * u0;
    out[1] = scaling * u1;
    if (!J) return 1;
    if (r > 1e-8) {
        double t2, t3, t4, t6, t7, t8, t9, t11, t17, t18, t19, t20, t25;
        t2 = u0 * u0;
        t3 = u1 * u1;
        t4 = t2 + t3;
        t6 = atan(sqrt(t4));
        t7 = t6 * t6;
        t8 = 1.0 / sqrt(t4);
        t9 = t7 * t7;
        t11 = 1.0 / ((t2 + t3) + 1.0);
        t17 = (((k1 * t7 + k2 * t9) + k3 * t7 * t9) + k4 * (t9 * t9)) + 1.0;
        t18 = 1.0 / t4;
        t19 = 1.0 / sqrt(t4 * t4 * t4);
        t20 = t6 * t8 * t17;
        t25 = ((k2 * t6 * t7 * t8 * t11 * u1 * 4.0
            + k3 * t6 * t8 * t9 * t11 * u1 * 6.0)
            + k4 * t6 * t7 * t8 * t9 * t11 * u1 * 8.0)
            + k1 * t6 * t8 * t11 * u1 * 2.0;
        t4 = ((k2 * t6 * t7 * t8 * t11 * u0 * 4.0
            + k3 * t6 * t8 * t9 * t11 * u0 * 6.0)
            + k4 * t6 * t7 * t8 * t9 * t11 * u0 * 8.0)
            + k1 * t6 * t8 * t11 * u0 * 2.0;
        t7 = t11 * t17 * t18 * u0 * u1;
        M2(J, 0, 1) = (t7 + t6 * t8 * t25 * u0) - t6 * t17 * t19 * u0 * u1;
        M2(J, 1, 1) = ((t20 - t3 * t6 * t17 * t19) + t3 * t11 * t17 * t18) + t6 * t8 * t25 * u1;
        M2(J, 0, 0) = ((t20 - t2 * t6 * t17 * t19) + t2 * t11 * t17 * t18) + t6 * t8 * t4 * u0;
        M2(J, 1, 0) = (t7 + t6 * t8 * t4 * u1) - t6 * t17 * t19 * u0 * u1;
        if (Jp) {
            double s6, s2, s3, s8, s10;
            s6 = u0 * u0 + u1 * u1;
            s2 = atan(sqrt(s6));
            s3 = s2 * s2;
            s8 = s3 * s3;
            s6 = 1.0 / sqrt(s6);
            s10 = s8 * s8;
            M2(Jp, 0, 0) = s2 * s3 * s6 * u0;
            M2(Jp, 1, 0) = s2 * s3 * s6 * u1;
            M2(Jp, 0, 1) = s2 * s8 * s6 * u0;
            M2(Jp, 1, 1) = s2 * s8 * s6 * u1;
            M2(Jp, 0, 2) = s2 * s3 * s8 * s6 * u0;
            M2(Jp, 1, 2) = s2 * s3 * s8 * s6 * u1;
            M2(Jp, 0, 3) = s2 * s6 * s10 * u0;
            M2(Jp, 1, 3) = s2 * s6 * s10 * u1;
        }
    } else {
        if (Jp) memset(Jp, 0, 8 * sizeof(double));
        M2(J, 0, 0) = 1.0; M2(J, 1, 0) = 0.0; M2(J, 0, 1) = 0.0; M2(J, 1, 1) = 1.0; /* setIdentity */
    }
    return 1;
}

int ok_dist_distort(const ok_cam* c, const double* params, const double u[2], double out[2], double J[4], double* Jp) {
    const double* p = params ? params : c->d;
    switch (c->dist) {
    case OK_CAM_RADTAN: return radtan_distort(p, u, out, J, Jp);
    case OK_CAM_EQUIDISTANT: return equi_distort(p, u, out, J, Jp);
    default:
        out[0] = u[0]; out[1] = u[1];
        if (J) { M2(J, 0, 0) = 1.0; M2(J, 1, 0) = 0.0; M2(J, 0, 1) = 0.0; M2(J, 1, 1) = 1.0; }
        return 1;
    }
}

/* Eigen 2x2: determinant = a00*a11 - a10*a01, inverse via invdet = 1/det (InverseImpl.h size-2 helper) */
static void m2_inverse(const double a[4], double out[4]) {
    const double det = M2(a, 0, 0) * M2(a, 1, 1) - M2(a, 1, 0) * M2(a, 0, 1);
    const double invdet = 1.0 / det;
    const double temp = M2(a, 0, 0);
    M2(out, 0, 0) = M2(a, 1, 1) * invdet;
    M2(out, 1, 0) = -M2(a, 1, 0) * invdet;
    M2(out, 0, 1) = -M2(a, 0, 1) * invdet;
    M2(out, 1, 1) = temp * invdet;
}

/* du = (E^T E)^-1 * E^T * e : ((inv * E^T) * e), every product a plain 2-term sum */
static void gauss_newton_step(const double E[4], const double e[2], double du[2]) {
    double E2[4], inv[4], M[4];
    int i, j;
    for (j = 0; j < 2; ++j)
        for (i = 0; i < 2; ++i) M2(E2, i, j) = M2(E, 0, i) * M2(E, 0, j) + M2(E, 1, i) * M2(E, 1, j); /* E^T*E */
    m2_inverse(E2, inv);
    for (j = 0; j < 2; ++j)
        for (i = 0; i < 2; ++i) M2(M, i, j) = M2(inv, i, 0) * M2(E, j, 0) + M2(inv, i, 1) * M2(E, j, 1); /* inv*E^T */
    for (i = 0; i < 2; ++i) du[i] = M2(M, i, 0) * e[0] + M2(M, i, 1) * e[1];
}

/* undistort (x_bar iteration) for the models; n iterations, chi2 thresholds as in the three C++ variants */
static int undistort_impl(const ok_cam* c, const double pd[2], double out[2], double E[4], int n, double thresh) {
    double x_bar[2], x_tmp[2], e[2], du[2], chi2;
    int i, success = 0;
    x_bar[0] = pd[0]; x_bar[1] = pd[1];
    for (i = 0; i < n; i++) {
        ok_dist_distort(c, NULL, x_bar, x_tmp, E, NULL);
        e[0] = pd[0] - x_tmp[0];
        e[1] = pd[1] - x_tmp[1];
        gauss_newton_step(E, e, du);
        x_bar[0] += du[0];
        x_bar[1] += du[1];
        chi2 = e[0] * e[0] + e[1] * e[1];
        if (chi2 < thresh) success = 1;
        if (chi2 < 1e-15) { success = 1; break; }
    }
    out[0] = x_bar[0]; out[1] = x_bar[1];
    return success;
}

int ok_dist_undistort(const ok_cam* c, const double pd[2], double out[2]) {
    double E[4];
    if (c->dist == OK_CAM_NODIST) { out[0] = pd[0]; out[1] = pd[1]; return 1; }
    if (c->dist == OK_CAM_EQUIDISTANT) return undistort_impl(c, pd, out, E, 20, 1e-6);
    return undistort_impl(c, pd, out, E, 5, 1e-6);
}

int ok_dist_undistort_j(const ok_cam* c, const double pd[2], double out[2], double J[4]) {
    double E[4];
    int ok;
    if (c->dist == OK_CAM_NODIST) {
        out[0] = pd[0]; out[1] = pd[1];
        M2(J, 0, 0) = 1.0; M2(J, 1, 0) = 0.0; M2(J, 0, 1) = 0.0; M2(J, 1, 1) = 1.0;
        return 1;
    }
    ok = undistort_impl(c, pd, out, E, 5, c->dist == OK_CAM_EQUIDISTANT ? 1e-2 : 1e-4);
    m2_inverse(E, J); /* the Jacobian of the inverse map is the inverse Jacobian */
    return ok;
}

/* ------------------------------------------------------------------------------------------------ */
/* projection                                                                                        */
/* ------------------------------------------------------------------------------------------------ */

static ok_proj_status finish_status(const ok_cam* c, const double img[2], double z) {
    if (!is_in_image(c, img)) return OK_PROJ_OUTSIDE_IMAGE;
    if (z > 0.0) return OK_PROJ_SUCCESSFUL;
    return OK_PROJ_BEHIND;
}

ok_proj_status ok_cam_project(const ok_cam* c, const double p[3], double img[2]) {
    double pu[2], pd[2];
    double rz;
    if (fabs(p[2]) < 1.0e-12) return OK_PROJ_INVALID;
    rz = 1.0 / p[2];
    pu[0] = p[0] * rz;
    pu[1] = p[1] * rz;
    if (!ok_dist_distort(c, NULL, pu, pd, NULL, NULL)) return OK_PROJ_INVALID;
    img[0] = c->fu * pd[0] + c->cu;
    img[1] = c->fv * pd[1] + c->cv;
    return finish_status(c, img, p[2]);
}

/* shared body of project(.., &J, &Ji) and projectWithExternalParameters; params NULL = own */
static ok_proj_status project_jac(const ok_cam* c, const double p[3], const double* params, double img[2], double* J,
                                  double* Ji) {
    const double fu = params ? params[0] : c->fu;
    const double fv = params ? params[1] : c->fv;
    const double cu = params ? params[2] : c->cu;
    const double cv = params ? params[3] : c->cv;
    const double* dp = params ? params + 4 : c->d;
    double pu[2], pd[2], dJ[4], Jd[8];
    double rz, rz2;
    int ok, k, nd = c->nd;
    if (fabs(p[2]) < 1.0e-12) return OK_PROJ_INVALID;
    rz = 1.0 / p[2];
    rz2 = rz * rz;
    pu[0] = p[0] * rz;
    pu[1] = p[1] * rz;
    if (Ji) {
        ok = ok_dist_distort(c, dp, pu, pd, dJ, Jd);
        /* intrinsics Jacobian (2 x (4+nd)): [diag(pd) | I | diag(fu,fv) * Jd] */
        Ji[0 + 2 * 0] = pd[0]; Ji[1 + 2 * 0] = 0.0;
        Ji[0 + 2 * 1] = 0.0;   Ji[1 + 2 * 1] = pd[1];
        Ji[0 + 2 * 2] = 1.0;   Ji[1 + 2 * 2] = 0.0;
        Ji[0 + 2 * 3] = 0.0;   Ji[1 + 2 * 3] = 1.0;
        for (k = 0; k < nd; ++k) {
            Ji[0 + 2 * (4 + k)] = fu * M2(Jd, 0, k);
            Ji[1 + 2 * (4 + k)] = fv * M2(Jd, 1, k);
        }
    } else {
        ok = ok_dist_distort(c, dp, pu, pd, dJ, NULL);
    }
    if (J) {
        /* J(0,2) = -fu * (x*dJ00 + y*dJ01) * rz2 */
        M2(J, 0, 0) = fu * M2(dJ, 0, 0) * rz;
        M2(J, 0, 1) = fu * M2(dJ, 0, 1) * rz;
        J[0 + 2 * 2] = -fu * (p[0] * M2(dJ, 0, 0) + p[1] * M2(dJ, 0, 1)) * rz2;
        M2(J, 1, 0) = fv * M2(dJ, 1, 0) * rz;
        M2(J, 1, 1) = fv * M2(dJ, 1, 1) * rz;
        J[1 + 2 * 2] = -fv * (p[0] * M2(dJ, 1, 0) + p[1] * M2(dJ, 1, 1)) * rz2;
    }
    img[0] = fu * pd[0] + cu;
    img[1] = fv * pd[1] + cv;
    if (!ok) return OK_PROJ_INVALID;
    return finish_status(c, img, p[2]);
}

ok_proj_status ok_cam_project_j(const ok_cam* c, const double p[3], double img[2], double J[6], double* Ji) {
    return project_jac(c, p, NULL, img, J, Ji);
}

ok_proj_status ok_cam_project_ext(const ok_cam* c, const double p[3], const double* params, double img[2], double* J,
                                  double* Ji) {
    return project_jac(c, p, params, img, J, Ji);
}

ok_proj_status ok_cam_project_h(const ok_cam* c, const double p[4], double img[2]) {
    double head[3];
    if (p[3] < 0) { head[0] = -p[0]; head[1] = -p[1]; head[2] = -p[2]; }
    else { head[0] = p[0]; head[1] = p[1]; head[2] = p[2]; }
    return ok_cam_project(c, head, img);
}

static ok_proj_status project_h_jac(const ok_cam* c, const double p[4], const double* params, double img[2], double J[8],
                                    double* Ji) {
    double head[3], J3[6];
    ok_proj_status st;
    int k;
    if (p[3] < 0) { head[0] = -p[0]; head[1] = -p[1]; head[2] = -p[2]; }
    else { head[0] = p[0]; head[1] = p[1]; head[2] = p[2]; }
    for (k = 0; k < 6; ++k) J3[k] = 0.0; /* the C++ copies an uninitialised J3 for |z| < 1e-12; the dumps skip that case */
    st = project_jac(c, head, params, img, J3, Ji);
    if (J) {
        for (k = 0; k < 6; ++k) J[k] = J3[k];  /* topLeftCorner<2,3>; column-major 2x4: cols 0..2 are the first 6 */
        J[6] = 0.0; J[7] = 0.0;                /* bottomRightCorner<2,1> */
    }
    return st;
}

ok_proj_status ok_cam_project_h_j(const ok_cam* c, const double p[4], double img[2], double J[8], double* Ji) {
    return project_h_jac(c, p, NULL, img, J, Ji);
}

ok_proj_status ok_cam_project_h_ext(const ok_cam* c, const double p[4], const double* params, double img[2], double* J,
                                    double* Ji) {
    return project_h_jac(c, p, params, img, J, Ji);
}

/* ------------------------------------------------------------------------------------------------ */
/* back-projection                                                                                   */
/* ------------------------------------------------------------------------------------------------ */

int ok_cam_back_project(const ok_cam* c, const double ip[2], double dir[3]) {
    double p2[2], u[2];
    int ok;
    p2[0] = (ip[0] - c->cu) * c->one_over_fu;
    p2[1] = (ip[1] - c->cv) * c->one_over_fv;
    ok = ok_dist_undistort(c, p2, u);
    dir[0] = u[0];
    dir[1] = u[1];
    dir[2] = 1.0;
    return ok;
}

long ok_cam_awareness_maps(const ok_cam* c, float* rays, float* jacobians) {
    long failed = 0;
    int u, v, i;
    for (v = 0; v < c->h; ++v) {
        for (u = 0; u < c->w; ++u) {
            const size_t px = (size_t)v * (size_t)c->w + (size_t)u;
            double ip[2], ray[3], pt[2], J[6];   /* J column-major 2x3 */
            ip[0] = (double)u; ip[1] = (double)v;
            if (ok_cam_back_project(c, ip, ray)) {
                double n = (ray[0] * ray[0] + ray[1] * ray[1]) + ray[2] * ray[2];
                if (n > 0.0) { n = sqrt(n); ray[0] /= n; ray[1] /= n; ray[2] /= n; }   /* Eigen normalize() */
            } else {
                ray[0] = 0.0; ray[1] = 0.0; ray[2] = 0.0;
            }
            for (i = 0; i < 3; ++i) rays[3 * px + (size_t)i] = (float)ray[i];
            if (ok_cam_project_j(c, ray, pt, J, NULL) == OK_PROJ_SUCCESSFUL) {
                float* j = jacobians + 6 * px;
                j[0] = (float)J[0]; j[1] = (float)J[2]; j[2] = (float)J[4];
                j[3] = (float)J[1]; j[4] = (float)J[3]; j[5] = (float)J[5];
            } else {
                memset(jacobians + 6 * px, 0, 6 * sizeof(float));
                failed++;
            }
        }
    }
    return failed;
}

int ok_cam_back_project_j(const ok_cam* c, const double ip[2], double dir[3], double J[6]) {
    double p2[2], u[2], U[4], O[6];
    int ok, i, j;
    p2[0] = (ip[0] - c->cu) * c->one_over_fu;
    p2[1] = (ip[1] - c->cv) * c->one_over_fv;
    ok = ok_dist_undistort_j(c, p2, u, U);
    dir[0] = u[0];
    dir[1] = u[1];
    dir[2] = 1.0;
    /* outProjectJacobian (3x2) = zeros with (0,0) = 1/fu, (1,1) = 1/fv; J = O * U (3x2 * 2x2, plain 2-term sums) */
    for (i = 0; i < 6; ++i) O[i] = 0.0;
    O[0 + 3 * 0] = c->one_over_fu;
    O[1 + 3 * 1] = c->one_over_fv;
    for (j = 0; j < 2; ++j)
        for (i = 0; i < 3; ++i) J[i + 3 * j] = O[i + 3 * 0] * M2(U, 0, j) + O[i + 3 * 1] * M2(U, 1, j);
    return ok;
}

int ok_cam_back_project_h(const ok_cam* c, const double ip[2], double dir[4]) {
    double ray[3];
    const int ok = ok_cam_back_project(c, ip, ray);
    dir[0] = ray[0]; dir[1] = ray[1]; dir[2] = ray[2];
    dir[3] = 1.0;
    return ok;
}

int ok_cam_back_project_h_j(const ok_cam* c, const double ip[2], double dir[4], double J[8]) {
    double ray[3], J3[6];
    int k, j;
    const int ok = ok_cam_back_project_j(c, ip, ray, J3);
    dir[0] = ray[0]; dir[1] = ray[1]; dir[2] = ray[2];
    dir[3] = 1.0;
    for (j = 0; j < 2; ++j) {
        for (k = 0; k < 3; ++k) J[k + 4 * j] = J3[k + 3 * j];
        J[3 + 4 * j] = 0.0; /* bottomRightCorner<1,2> */
    }
    return ok;
}

/* ------------------------------------------------------------------------------------------------ */
/* NCameraSystem                                                                                     */
/* ------------------------------------------------------------------------------------------------ */

void ok_ncam_init(ok_ncam* s) { memset(s, 0, sizeof *s); }

int ok_ncam_add(ok_ncam* s, const ok_cam* cam, const ok_tf* T_SC) {
    if (s->n >= OK_NCAM_MAX) return -1;
    s->cam[s->n] = *cam;
    s->T_SC[s->n] = *T_SC;
    s->n++;
    return 0;
}

void ok_ncam_free(ok_ncam* s) {
    int a, b;
    for (a = 0; a < OK_NCAM_MAX; ++a)
        for (b = 0; b < OK_NCAM_MAX; ++b) { free(s->mask[a][b]); s->mask[a][b] = NULL; }
}

int ok_ncam_compute_overlaps(ok_ncam* s) {
    int seen, ci;
    ok_ncam_free(s);
    for (seen = 0; seen < s->n; ++seen) {
        for (ci = 0; ci < s->n; ++ci) {
            const ok_cam* camera = &s->cam[ci];
            const size_t sz = (size_t)camera->w * (size_t)camera->h;
            uint8_t* m = (uint8_t*)calloc(sz ? sz : 1, 1);
            if (!m) return -1;
            s->mask[seen][ci] = m;
            s->overlaps[seen][ci] = 0;
            if (ci == seen) {
                memset(m, 1, sz);
                s->overlaps[seen][ci] = 1;
            } else {
                const ok_cam* other = &s->cam[seen];
                ok_tf Tinv, T_Cother_C;
                int u, v, has = 0;
                ok_tf_inverse(&s->T_SC[seen], &Tinv, 1);
                ok_tf_mul(&Tinv, &s->T_SC[ci], &T_Cother_C, 1);
                for (u = 0; u < camera->w; ++u) {
                    for (v = 0; v < camera->h; ++v) {
                        double ip[2], ray_C[3], ray_Co[3], pt[2], ver[3];
                        ip[0] = (double)u; ip[1] = (double)v;
                        ok_cam_back_project(camera, ip, ray_C);
                        ok_m3_mulv(T_Cother_C.C, ray_C, ray_Co);
                        if (ok_cam_project(other, ray_Co, pt) == OK_PROJ_SUCCESSFUL) {
                            double n0[3], n1[3], dot;
                            ok_cam_back_project(other, pt, ver);
                            memcpy(n0, ray_Co, sizeof n0); /* normalized() returns the vector itself when |v|^2 == 0 */
                            memcpy(n1, ver, sizeof n1);
                            ok_v3_normalized(ray_Co, n0);
                            ok_v3_normalized(ver, n1);
                            dot = (n0[0] * n1[0] + n0[1] * n1[1]) + n0[2] * n1[2];
                            if (fabs(dot - 1.0) < 1.0e-10) {
                                m[(size_t)v * (size_t)camera->w + (size_t)u] = 1;
                                if (!has) s->overlaps[seen][ci] = 1;
                                has = 1;
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}
