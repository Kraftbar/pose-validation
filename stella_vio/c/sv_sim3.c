/* SPDX-License-Identifier: BSD-2-Clause */
/* g2o::Sim3 -- see sv_sim3.h (BSD-2, g2o; evaluation order via MPL-2.0 primitives). */
#include "sv_sim3.h"
#include "sv_eigen_lu3.h"
#include "sv_linalg.h"
#include <math.h>

#define M(a, i, j) (a)[(j) * 3 + (i)]

static void skew(const double v[3], double m[9]) {
    int i;
    for (i = 0; i < 9; ++i) {
        m[i] = 0.0;
    }
    M(m, 0, 1) = -v[2];
    M(m, 0, 2) = v[1];
    M(m, 1, 2) = -v[0];
    M(m, 1, 0) = v[2];
    M(m, 2, 0) = -v[1];
    M(m, 2, 1) = v[0];
}

static void delta_r(const double R[9], double v[3]) {
    v[0] = M(R, 2, 1) - M(R, 1, 2);
    v[1] = M(R, 0, 2) - M(R, 2, 0);
    v[2] = M(R, 1, 0) - M(R, 0, 1);
}

static void eye3(double I[9]) {
    int i;
    for (i = 0; i < 9; ++i) {
        I[i] = 0.0;
    }
    I[0] = I[4] = I[8] = 1.0;
}

void sv_sim3_identity(sv_sim3* out) {
    out->r.x = 0.0;
    out->r.y = 0.0;
    out->r.z = 0.0;
    out->r.w = 1.0;
    out->t[0] = out->t[1] = out->t[2] = 0.0;
    out->s = 1.0;
}

/* Sim3::normalizeRotation(): if (r.w() < 0) r.coeffs() *= -1; r.normalize(); */
static void normalize_rotation(sv_sim3* a) {
    if (a->r.w < 0.0) {
        a->r.x *= -1.0;
        a->r.y *= -1.0;
        a->r.z *= -1.0;
        a->r.w *= -1.0;
    }
    sv_quat_normalize(&a->r);
}

void sv_sim3_from_quat(const sv_quat* q, const double t[3], double s, sv_sim3* out) {
    out->r = *q;
    out->t[0] = t[0];
    out->t[1] = t[1];
    out->t[2] = t[2];
    out->s = s;
    normalize_rotation(out);
}

void sv_sim3_from_rot(const double R[9], const double t[3], double s, sv_sim3* out) {
    sv_quat q;
    sv_quat_from_mat3(R, &q);
    sv_sim3_from_quat(&q, t, s, out);
}

void sv_sim3_exp(const double update[7], sv_sim3* out) {
    double omega[3], upsilon[3], Omega[9], Omega2[9], I[9], R[9], W[9], tt[3];
    double sigma, theta, s, A, B, C;
    const double eps = 0.00001;
    int i;
    for (i = 0; i < 3; ++i) {
        omega[i] = update[i];
        upsilon[i] = update[i + 3];
    }
    sigma = update[6];
    theta = sv_vec3_norm(omega);
    skew(omega, Omega);
    s = exp(sigma);
    sv_mat3_mul(Omega, Omega, Omega2);
    eye3(I);
    if (fabs(sigma) < eps) {
        C = 1.0;
        if (theta < eps) {
            A = 1.0 / 2.0;
            B = 1.0 / 6.0;
            for (i = 0; i < 9; ++i) {
                R[i] = (I[i] + Omega[i]) + Omega2[i] / 2.0;
            }
        }
        else {
            const double theta2 = theta * theta;
            const double c1 = sin(theta) / theta;
            const double c2 = (1.0 - cos(theta)) / (theta * theta);
            A = (1.0 - cos(theta)) / theta2;
            B = (theta - sin(theta)) / (theta2 * theta);
            for (i = 0; i < 9; ++i) {
                R[i] = (I[i] + c1 * Omega[i]) + c2 * Omega2[i];
            }
        }
    }
    else {
        C = (s - 1.0) / sigma;
        if (theta < eps) {
            const double sigma2 = sigma * sigma;
            A = ((sigma - 1.0) * s + 1.0) / sigma2;
            B = ((0.5 * sigma2 - sigma + 1.0) * s - 1.0) / (sigma2 * sigma);
            for (i = 0; i < 9; ++i) {
                R[i] = (I[i] + Omega[i]) + Omega2[i] / 2.0;
            }
        }
        else {
            const double c1 = sin(theta) / theta;
            const double c2 = (1.0 - cos(theta)) / (theta * theta);
            const double a = s * sin(theta);
            const double b = s * cos(theta);
            const double theta2 = theta * theta;
            const double sigma2 = sigma * sigma;
            const double c = theta2 + sigma2;
            for (i = 0; i < 9; ++i) {
                R[i] = (I[i] + c1 * Omega[i]) + c2 * Omega2[i];
            }
            A = (a * sigma + (1.0 - b) * theta) / (theta * c);
            B = (C - ((b - 1.0) * sigma + a * theta) / (c)) * 1.0 / (theta2);
        }
    }
    sv_quat_from_mat3(R, &out->r);
    for (i = 0; i < 9; ++i) {
        W[i] = (A * Omega[i] + B * Omega2[i]) + C * I[i];
    }
    sv_mat3_mulv(W, upsilon, tt);
    out->t[0] = tt[0];
    out->t[1] = tt[1];
    out->t[2] = tt[2];
    out->s = s;
}

void sv_sim3_log(const sv_sim3* a, double out[7]) {
    double R[9], Omega[9], I[9], W[9], omega[3], upsilon[3], dR[3];
    double sigma = log(a->s), d, A, B, C;
    const double eps = 0.00001;
    const double s = a->s;
    int i;
    sv_quat_to_mat3(&a->r, R);
    d = 0.5 * (((M(R, 0, 0) + M(R, 1, 1)) + M(R, 2, 2)) - 1.0);
    eye3(I);
    if (fabs(sigma) < eps) {
        C = 1.0;
        if (d > 1.0 - eps) {
            delta_r(R, dR);
            for (i = 0; i < 3; ++i) {
                omega[i] = 0.5 * dR[i];
            }
            skew(omega, Omega);
            A = 1.0 / 2.0;
            B = 1.0 / 6.0;
        }
        else {
            const double theta = acos(d);
            const double theta2 = theta * theta;
            const double f = theta / (2.0 * sqrt(1.0 - d * d));
            delta_r(R, dR);
            for (i = 0; i < 3; ++i) {
                omega[i] = f * dR[i];
            }
            skew(omega, Omega);
            A = (1.0 - cos(theta)) / (theta2);
            B = (theta - sin(theta)) / (theta2 * theta);
        }
    }
    else {
        C = (s - 1.0) / sigma;
        if (d > 1.0 - eps) {
            const double sigma2 = sigma * sigma;
            delta_r(R, dR);
            for (i = 0; i < 3; ++i) {
                omega[i] = 0.5 * dR[i];
            }
            skew(omega, Omega);
            A = ((sigma - 1.0) * s + 1.0) / (sigma2);
            B = ((0.5 * sigma2 - sigma + 1.0) * s - 1.0) / (sigma2 * sigma);
        }
        else {
            const double theta = acos(d);
            const double f = theta / (2.0 * sqrt(1.0 - d * d));
            double theta2, aa, bb, cc;
            delta_r(R, dR);
            for (i = 0; i < 3; ++i) {
                omega[i] = f * dR[i];
            }
            skew(omega, Omega);
            theta2 = theta * theta;
            aa = s * sin(theta);
            bb = s * cos(theta);
            cc = theta2 + sigma * sigma;
            A = (aa * sigma + (1.0 - bb) * theta) / (theta * cc);
            B = (C - ((bb - 1.0) * sigma + aa * theta) / (cc)) * 1.0 / (theta2);
        }
    }
    {
        /* W = A * Omega + B * Omega * Omega + C * I  ==  (A*Omega + ((B*Omega)*Omega)) + C*I */
        double BO[9], BOO[9];
        for (i = 0; i < 9; ++i) {
            BO[i] = B * Omega[i];
        }
        sv_mat3_mul(BO, Omega, BOO);
        for (i = 0; i < 9; ++i) {
            W[i] = (A * Omega[i] + BOO[i]) + C * I[i];
        }
    }
    sv_lu3_solve(W, a->t, upsilon);
    for (i = 0; i < 3; ++i) {
        out[i] = omega[i];
        out[i + 3] = upsilon[i];
    }
    out[6] = sigma;
}

void sv_sim3_inverse(const sv_sim3* a, sv_sim3* out) {
    sv_quat conj;
    double st[3], tt[3];
    const double f = (-1.0) / a->s;
    conj.x = -a->r.x;
    conj.y = -a->r.y;
    conj.z = -a->r.z;
    conj.w = a->r.w;
    st[0] = f * a->t[0];
    st[1] = f * a->t[1];
    st[2] = f * a->t[2];
    sv_quat_map(&conj, st, tt);
    sv_sim3_from_quat(&conj, tt, 1.0 / a->s, out);
}

void sv_sim3_mul(const sv_sim3* a, const sv_sim3* b, sv_sim3* out) {
    sv_quat r;
    double rt[3], t[3];
    const double s = a->s * b->s;
    sv_quat_mul(&a->r, &b->r, &r);
    sv_quat_map(&a->r, b->t, rt);
    t[0] = a->s * rt[0] + a->t[0];
    t[1] = a->s * rt[1] + a->t[1];
    t[2] = a->s * rt[2] + a->t[2];
    out->r = r;
    out->t[0] = t[0];
    out->t[1] = t[1];
    out->t[2] = t[2];
    out->s = s;
}

void sv_sim3_map(const sv_sim3* a, const double xyz[3], double out[3]) {
    double rx[3];
    sv_quat_map(&a->r, xyz, rx);
    out[0] = a->s * rx[0] + a->t[0];
    out[1] = a->s * rx[1] + a->t[1];
    out[2] = a->s * rx[2] + a->t[2];
}

void sv_sim3_rotation_matrix(const sv_sim3* a, double R[9]) {
    sv_quat_to_mat3(&a->r, R);
}
