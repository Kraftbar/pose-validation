/* SPDX-License-Identifier: BSD-3-Clause
 * Port of basalt-headers imu/preintegration.h (IntegratedImuMeasurement<float>, BSD-3-Clause, (c) 2019 Usenko, Demmel),
 * imu/imu_types.h and the IMU factor of basalt/linearization/imu_block.hpp, with the Sophus 1.24.6 (MIT) SO3 / Basalt
 * sophus_utils.hpp Jacobian formulas they use. Private float copies of the Lie helpers (the M1 module bs_lie.h is independent).
 * Eigen 3.4.0 (MPL-2.0) evaluation-order rules applied (all measured against the real classes, basalt_port/reference_tools/bs_imu_test.cc):
 *   - 3-element reductions / 3x3 coefficient products (float: no SliceVectorization, packet is 4) are trees  a0 + (a1 + a2);
 *   - 9x9 / 9x3 products (rows+depth+cols >= 20) take the GEBP kernel: with float mr = 8, nr = 4, rows = 9 every output is ONE left
 *     fold over k starting from 0, written as `0 + 1*acc` (alpha = 1); the 1-row remainder of the triangular solve uses
 *     the scalar path (`res + (-1)*acc`);
 *   - column-major GEMV: per row one left fold from 0, then `res + alpha*acc`;  a 1 x k * k x 1 product is a `dot`: left fold without the 0;
 *   - SO3 constructed from a quaternion (every product, inverse) re-normalises: len = sqrtf((x*x+z*z)+(y*y+w*w)), coeffs /= len;
 *   - `a * b` of Matrix3f is evaluated coefficient-wise per product.
 * Every expression keeps the C++ association; float literals are explicit; sqrt/sin/cos/atan2 are the float libm functions. */
#include <float.h>
#include <math.h>
#include <string.h>
#include "bs_imu.h"

#ifdef BS_IMU_COV   /* branch coverage counters for the oracle (bs_imu_test built with -DBS_IMU_COV) */
unsigned long bs_imu_cov[16];
#define COV(i) (++bs_imu_cov[i])
#else
#define COV(i) ((void)0)
#endif
#define BS_EPS ((float)1e-5)  /* Sophus::Constants<float>::epsilon() */
#define BS_PI 3.14159265358979323846

/* ------------------------------------------------------------------ small dense helpers (column-major) */

static float sq3(const float a[3]) { return (a[0] * a[0]) + ((a[1] * a[1]) + (a[2] * a[2])); }

/* C = A*B, 3x3 float (coefficient-wise lazy product: a0b0 + (a1b1 + a2b2)) */
static void m3_mul(const float A[9], const float B[9], float C[9]) {
    int i, j;
    for (j = 0; j < 3; ++j)
        for (i = 0; i < 3; ++i) C[i + 3 * j] = A[i] * B[3 * j] + (A[i + 3] * B[1 + 3 * j] + A[i + 6] * B[2 + 3 * j]);
}

static void m3_mulv(const float A[9], const float v[3], float o[3]) {
    int i;
    for (i = 0; i < 3; ++i) o[i] = A[i] * v[0] + (A[i + 3] * v[1] + A[i + 6] * v[2]);
}

static void hat3(const float w[3], float H[9]) { /* SO3::hat, H(r,c) = H[r + 3c] */
    H[0] = 0.0f;  H[3] = -w[2]; H[6] = w[1];
    H[1] = w[2];  H[4] = 0.0f;  H[7] = -w[0];
    H[2] = -w[1]; H[5] = w[0];  H[8] = 0.0f;
}

static void cross3(const float a[3], const float b[3], float o[3]) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

/* dot product A(i,:)*B(:,j) as Eigen 3.4.0's float GEBP (SSE, mr = 8, nr = 4, no FMA) accumulates it, for rows in {1, 9} (rows % 8 in {0, 1}):
 *   rows [0, 8*(rows/8))        : 2-packet kernel, one left fold over k from 0;
 *   the remaining row(s)        : "process remaining rows, 1 at once": in a 4-column panel (j < 4*(cols/4)) the swapped-traits path keeps
 *                                 4 accumulators (k mod 4) over k < 4*(depth/4), C0 = (C0+C1)+(C2+C3), then the leftover k sequentially;
 *                                 in the leftover columns a scalar left fold. */
static float gebp_acc(const float* A, int ars, int acs, const float* B, int brs, int bcs, int i, int j, int rows, int cols, int depth) {
    const int peeled_mc2 = (rows / 8) * 8;
    int k;
    if (i < peeled_mc2 || j >= (cols / 4) * 4) {
        float acc = 0.0f;
        for (k = 0; k < depth; ++k) acc = acc + A[i * ars + k * acs] * B[k * brs + j * bcs];
        return acc;
    } else {
        float c[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        const int endk4 = (depth / 4) * 4;
        float t;
        for (k = 0; k < endk4; ++k) c[k & 3] = c[k & 3] + A[i * ars + k * acs] * B[k * brs + j * bcs];
        t = (c[0] + c[1]) + (c[2] + c[3]);
        for (k = endk4; k < depth; ++k) t = t + A[i * ars + k * acs] * B[k * brs + j * bcs];
        return t;
    }
}

/* same, for the statement form `dst = X * Y * Z.transpose()` (assignment, also inside a sum): Eigen solves the outer product as the transposed
 * problem dst^T = Z * (X*Y)^T, i.e. the remainder-row / swapped-chain pattern applies to the output COLUMN 8 (rows 0..7) instead of row 8 */
static void gemm_fold_T(int rows, int cols, int depth, const float* A, int ars, int acs, const float* B, int brs, int bcs, float* out) {
    int i, j;
    for (j = 0; j < cols; ++j)
        for (i = 0; i < rows; ++i) {
            const float acc = gebp_acc(B, bcs, brs, A, acs, ars, j, i, cols, rows, depth);  /* element (j,i) of (B^T)*(A^T) */
            out[i + rows * j] = 0.0f + 1.0f * acc;
        }
}

/* out(rows x cols, col-major) = 0 + 1 * A*B (GemmProduct into a zeroed destination); A(i,k) = A[i*ars + k*acs] */
static void gemm_fold(int rows, int cols, int depth, const float* A, int ars, int acs, const float* B, int brs, int bcs, float* out) {
    int i, j;
    for (j = 0; j < cols; ++j)
        for (i = 0; i < rows; ++i) {
            const float acc = gebp_acc(A, ars, acs, B, brs, bcs, i, j, rows, cols, depth);
            out[i + rows * j] = 0.0f + 1.0f * acc;
        }
}

/* ------------------------------------------------------------------ SO3<float> (quaternion x y z w) */

static void q_normalize(float q[4]) { /* SO3Base::normalize() */
    const float sq = (q[0] * q[0] + q[2] * q[2]) + (q[1] * q[1] + q[3] * q[3]);
    const float len = sqrtf(sq);
    q[0] /= len; q[1] /= len; q[2] /= len; q[3] /= len;
}

/* SO3 * SO3: QuaternionProduct then the SO3(quaternion) constructor normalises */
static void so3_mul(const float a[4], const float b[4], float o[4]) {
    float r[4];
    /* x y z w of a and b; formulas of SO3Base::QuaternionProduct(a, b) */
    r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    r[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    r[1] = a[3] * b[1] + a[1] * b[3] + a[2] * b[0] - a[0] * b[2];
    r[2] = a[3] * b[2] + a[2] * b[3] + a[0] * b[1] - a[1] * b[0];
    q_normalize(r);
    o[0] = r[0]; o[1] = r[1]; o[2] = r[2]; o[3] = r[3];
}

static void so3_inverse(const float q[4], float o[4]) { /* SO3(conjugate) -> normalised */
    float r[4];
    r[0] = -q[0]; r[1] = -q[1]; r[2] = -q[2]; r[3] = q[3];
    q_normalize(r);
    o[0] = r[0]; o[1] = r[1]; o[2] = r[2]; o[3] = r[3];
}

static void so3_exp(const float w[3], float q[4]) { /* SO3::expAndTheta, no normalisation */
    const float eps2 = BS_EPS * BS_EPS;
    const float theta_sq = sq3(w);
    float imag_factor, real_factor;
    if (theta_sq < eps2) {
        COV(0);
        const float theta_po4 = theta_sq * theta_sq;
        imag_factor = 0.5f - (float)(1.0 / 48.0) * theta_sq + (float)(1.0 / 3840.0) * theta_po4;
        real_factor = 1.0f - (float)(1.0 / 8.0) * theta_sq + (float)(1.0 / 384.0) * theta_po4;
    } else {
        COV(1);
        const float theta = sqrtf(theta_sq);
        const float half_theta = 0.5f * theta;
        const float sin_half_theta = sinf(half_theta);
        imag_factor = sin_half_theta / theta;
        real_factor = cosf(half_theta);
    }
    q[3] = real_factor;
    q[0] = imag_factor * w[0];
    q[1] = imag_factor * w[1];
    q[2] = imag_factor * w[2];
}

static void so3_log(const float q[4], float o[3]) { /* SO3::log (logAndTheta) */
    const float squared_n = q[0] * q[0] + (q[1] * q[1] + q[2] * q[2]);
    const float w = q[3];
    float t;
    if (squared_n < BS_EPS * BS_EPS) {
        COV(2);
        const float squared_w = w * w;
        t = 2.0f / w - (float)(2.0 / 3.0) * (squared_n) / (w * squared_w);
    } else {
        const float n = sqrtf(squared_n);
        COV(3); if (w < 0.0f) COV(4);
        const float atan_nbyw = (w < 0.0f) ? (float)atan2f(-n, -w) : (float)atan2f(n, w);
        t = 2.0f * atan_nbyw / n;
    }
    o[0] = t * q[0]; o[1] = t * q[1]; o[2] = t * q[2];
}

static void q_to_mat3(const float q[4], float R[9]) { /* Quaternion::toRotationMatrix; R(r,c) = R[r + 3c] */
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float tx = 2.0f * x, ty = 2.0f * y, tz = 2.0f * z;
    const float twx = tx * w, twy = ty * w, twz = tz * w;
    const float txx = tx * x, txy = ty * x, txz = tz * x;
    const float tyy = ty * y, tyz = tz * y, tzz = tz * z;
    R[0] = 1.0f - (tyy + tzz); R[3] = txy - twz;          R[6] = txz + twy;
    R[1] = txy + twz;          R[4] = 1.0f - (txx + tzz); R[7] = tyz - twx;
    R[2] = txz - twy;          R[5] = tyz + twx;          R[8] = 1.0f - (txx + tyy);
}

/* SO3 * Vector3 (rotate a point): uv = vec x p; uv += uv; p + w*uv + vec x uv */
static void so3_rotate(const float q[4], const float p[3], float o[3]) {
    float uv[3], c[3];
    const float v[3] = {q[0], q[1], q[2]};
    cross3(v, p, uv);
    uv[0] += uv[0]; uv[1] += uv[1]; uv[2] += uv[2];
    cross3(v, uv, c);
    o[0] = p[0] + q[3] * uv[0] + c[0];
    o[1] = p[1] + q[3] * uv[1] + c[1];
    o[2] = p[2] + q[3] * uv[2] + c[2];
}

/* ---- Jacobians (basalt sophus_utils.hpp), Scalar = float; `1 - cos` etc. are int-promoted-to-float */

static void jac_right(const float phi[3], float J[9]) { /* rightJacobianSO3 */
    const float n2 = sq3(phi);
    float H[9], H2[9];
    int e;
    hat3(phi, H);
    m3_mul(H, H, H2);
    for (e = 0; e < 9; ++e) J[e] = 0.0f;
    J[0] = J[4] = J[8] = 1.0f;
    if (n2 > BS_EPS) {
        COV(5);
        const float n = sqrtf(n2);
        const float n3 = n2 * n;
        const float c1 = 1.0f - cosf(n);
        const float c2 = n - sinf(n);
        for (e = 0; e < 9; ++e) J[e] = J[e] - (H[e] * c1) / n2;
        for (e = 0; e < 9; ++e) J[e] = J[e] + (H2[e] * c2) / n3;
    } else {
        COV(6);
        for (e = 0; e < 9; ++e) J[e] = J[e] - H[e] / 2.0f;
        for (e = 0; e < 9; ++e) J[e] = J[e] + H2[e] / 6.0f;
    }
}

/* sign = +1: rightJacobianInvSO3 (J = I + H/2 + ...), sign = -1: leftJacobianInvSO3 (J = I - H/2 + ...) */
static void jac_inv(const float phi[3], float J[9], int sign) {
    const float n2 = sq3(phi);
    float H[9], H2[9];
    int e;
    hat3(phi, H);
    m3_mul(H, H, H2);
    for (e = 0; e < 9; ++e) J[e] = 0.0f;
    J[0] = J[4] = J[8] = 1.0f;
    if (sign > 0) { for (e = 0; e < 9; ++e) J[e] = J[e] + H[e] / 2.0f; }
    else          { for (e = 0; e < 9; ++e) J[e] = J[e] - H[e] / 2.0f; }
    if (n2 > BS_EPS) {
        const float n = sqrtf(n2);
        if ((double)n < BS_PI - (double)sqrtf(BS_EPS)) {
            COV(7);
            const float f = 1.0f / n2 - (1.0f + cosf(n)) / (2.0f * n * sinf(n));
            for (e = 0; e < 9; ++e) J[e] = J[e] + H2[e] * f;
        } else {
            COV(8);
            const float d = (float)(BS_PI * BS_PI);
            for (e = 0; e < 9; ++e) J[e] = J[e] + H2[e] / d;
        }
    } else {
        COV(9);
        for (e = 0; e < 9; ++e) J[e] = J[e] + H2[e] / 12.0f;
    }
}

/* ------------------------------------------------------------------ states */

void bs_pvstate_default(bs_pvstate* s) {
    memset(s, 0, sizeof(*s));
    s->q[3] = 1.0f;
}

void bs_imu_init_default(bs_imu_meas* m) {
    memset(m, 0, sizeof(*m));
    bs_pvstate_default(&m->delta);
}

void bs_imu_init(bs_imu_meas* m, int64_t start_t_ns, const float bg[3], const float ba[3]) {
    memset(m, 0, sizeof(*m));
    bs_pvstate_default(&m->delta);
    m->start_t_ns = start_t_ns;
    memcpy(m->bias_gyro_lin, bg, 12);
    memcpy(m->bias_accel_lin, ba, 12);
}

/* ------------------------------------------------------------------ propagateState / integrate / predictState */

#define M9(a, r, c) ((a)[(r) + 9 * (c)])

void bs_imu_propagate_state(const bs_pvstate* curr, const bs_imudata* data, bs_pvstate* next, float* F, float* A, float* G) {
    const int64_t dt_ns = data->t_ns - curr->t_ns;
    const float dt = (float)dt_ns * (float)1e-9;
    float w_half[3], w_full[3], E[4], R2q[4], RR[9], aw[3], nq[4];
    int i, r, c;

    for (i = 0; i < 3; ++i) w_half[i] = (0.5f * dt) * data->gyro[i];
    so3_exp(w_half, E);
    so3_mul(curr->q, E, R2q);
    q_to_mat3(R2q, RR);
    m3_mulv(RR, data->accel, aw);

    for (i = 0; i < 3; ++i) w_full[i] = dt * data->gyro[i];
    so3_exp(w_full, E);
    so3_mul(curr->q, E, nq);

    {
        float np[3], nv[3];
        for (i = 0; i < 3; ++i) nv[i] = curr->v[i] + aw[i] * dt;
        for (i = 0; i < 3; ++i) np[i] = curr->p[i] + curr->v[i] * dt + ((0.5f * aw[i]) * dt) * dt;
        next->t_ns = data->t_ns;
        memcpy(next->q, nq, 16);
        memcpy(next->v, nv, 12);
        memcpy(next->p, np, 12);
    }

    if (F || A || G) {
        float naw[3], Hh[9];
        for (i = 0; i < 3; ++i) naw[i] = (-aw[i]) * dt;
        hat3(naw, Hh);
        if (F) {
            for (i = 0; i < 81; ++i) F[i] = 0.0f;
            for (i = 0; i < 9; ++i) M9(F, i, i) = 1.0f;
            for (i = 0; i < 3; ++i) M9(F, i, 6 + i) = dt;
            for (c = 0; c < 3; ++c)
                for (r = 0; r < 3; ++r) {
                    M9(F, 6 + r, 3 + c) = Hh[r + 3 * c];
                    M9(F, r, 3 + c) = (Hh[r + 3 * c] * dt) * 0.5f;
                }
        }
        if (A) {
            for (i = 0; i < 27; ++i) A[i] = 0.0f;
            for (c = 0; c < 3; ++c)
                for (r = 0; r < 3; ++r) {
                    A[r + 9 * c] = ((0.5f * RR[r + 3 * c]) * dt) * dt;
                    A[(6 + r) + 9 * c] = RR[r + 3 * c] * dt;
                }
        }
        if (G) {
            float Jr[9], Jr2[9], Rn[9], t1[9], t2[9], t3[9];
            for (i = 0; i < 27; ++i) G[i] = 0.0f;
            jac_right(w_full, Jr);
            jac_right(w_half, Jr2);
            q_to_mat3(nq, Rn);
            m3_mul(Rn, Jr, t1);
            for (c = 0; c < 3; ++c)
                for (r = 0; r < 3; ++r) G[(3 + r) + 9 * c] = t1[r + 3 * c] * dt;
            m3_mul(Hh, RR, t2);
            m3_mul(t2, Jr2, t3);
            for (c = 0; c < 3; ++c)
                for (r = 0; r < 3; ++r) G[(6 + r) + 9 * c] = (t3[r + 3 * c] * 0.5f) * dt;
            {
                const float hd = 0.5f * dt;
                for (c = 0; c < 3; ++c)
                    for (r = 0; r < 3; ++r) G[r + 9 * c] = hd * G[(6 + r) + 9 * c];
            }
        }
    }
}

void bs_imu_integrate(bs_imu_meas* m, const bs_imudata* data, const float accel_cov[3], const float gyro_cov[3]) {
    bs_imudata dc = *data;
    bs_pvstate nst;
    float F[81], A[27], G[27];
    float T1[81], P1[81], AD[27], P2[81], GD[27], P3[81], tba[27], tbg[27];
    int i, r, c;

    dc.t_ns -= m->start_t_ns;
    for (i = 0; i < 3; ++i) dc.accel[i] -= m->bias_accel_lin[i];
    for (i = 0; i < 3; ++i) dc.gyro[i] -= m->bias_gyro_lin[i];

    bs_pvstate_default(&nst);
    bs_imu_propagate_state(&m->delta, &dc, &nst, F, A, G);
    m->delta = nst;

    /* cov_ = F * cov_ * F.transpose() + A * accel_cov.asDiagonal() * A.transpose() + G * gyro_cov.asDiagonal() * G.transpose() */
    gemm_fold(9, 9, 9, F, 1, 9, m->cov, 1, 9, T1);             /* F * cov_ */
    gemm_fold_T(9, 9, 9, T1, 1, 9, F, 9, 1, P1);               /* (F*cov_) * F^T: assignment, outer product on the transposed problem */
    for (c = 0; c < 3; ++c)
        for (r = 0; r < 9; ++r) { AD[r + 9 * c] = A[r + 9 * c] * accel_cov[c]; GD[r + 9 * c] = G[r + 9 * c] * gyro_cov[c]; }
    gemm_fold_T(9, 9, 3, AD, 1, 9, A, 9, 1, P2);               /* (A*D) * A^T (depth 3: all patterns coincide) */
    gemm_fold_T(9, 9, 3, GD, 1, 9, G, 9, 1, P3);
    for (i = 0; i < 81; ++i) m->cov[i] = (P1[i] + P2[i]) + P3[i];
    m->sqrt_cov_inv_computed = 0;

    /* d_state_d_ba_ = -A + F * d_state_d_ba_ ;  d_state_d_bg_ = -G + F * d_state_d_bg_ */
    gemm_fold(9, 3, 9, F, 1, 9, m->d_state_d_ba, 1, 9, tba);
    gemm_fold(9, 3, 9, F, 1, 9, m->d_state_d_bg, 1, 9, tbg);
    for (i = 0; i < 27; ++i) { m->d_state_d_ba[i] = (-A[i]) + tba[i]; m->d_state_d_bg[i] = (-G[i]) + tbg[i]; }
}

void bs_imu_predict_state(const bs_imu_meas* m, const bs_pvstate* s0, const float g[3], bs_pvstate* s1) {
    const float dt = (float)m->delta.t_ns * (float)1e-9;
    float q1[4], Rdv[3], Rdp[3], v1[3], p1[3];
    int i;
    so3_mul(s0->q, m->delta.q, q1);
    so3_rotate(s0->q, m->delta.v, Rdv);
    so3_rotate(s0->q, m->delta.p, Rdp);
    for (i = 0; i < 3; ++i) v1[i] = s0->v[i] + g[i] * dt + Rdv[i];
    for (i = 0; i < 3; ++i) p1[i] = s0->p[i] + s0->v[i] * dt + ((0.5f * g[i]) * dt) * dt + Rdp[i];
    memcpy(s1->q, q1, 16);
    memcpy(s1->v, v1, 12);
    memcpy(s1->p, p1, 12);
}

/* ------------------------------------------------------------------ residual */

/* y(9) = M(9x3) * x(3): a (Large,1,Small) product is CoeffBasedProductMode (lazy): rows 0..7 are two 4-lane packets, ((p0 + p1) + p2) with no
 * leading 0; the last row 8 is a scalar coefficient, the redux tree p0 + (p1 + p2) */
static void lazy93(const float* M, const float x[3], float y[9]) {
    int i;
    for (i = 0; i < 8; ++i) {
        float acc = M[i] * x[0];
        acc = M[i + 9] * x[1] + acc;
        acc = M[i + 18] * x[2] + acc;
        y[i] = acc;
    }
    y[8] = M[8] * x[0] + (M[8 + 9] * x[1] + M[8 + 18] * x[2]);
}

void bs_imu_residual(const bs_imu_meas* m, const bs_pvstate* s0, const float g[3], const bs_pvstate* s1, const float curr_bg[3],
                     const float curr_ba[3], float res[9], float* d0, float* d1, float* dbg, float* dba) {
    const float dt = (float)m->delta.t_ns * (float)1e-9;
    float bg_diff[9], ba_diff[9], dbgv[3], dbav[3];
    float q0i[4], R0[9], x3[3], tmp[3], tmp2[3];
    int i, r, c;

    for (i = 0; i < 3; ++i) { dbgv[i] = curr_bg[i] - m->bias_gyro_lin[i]; dbav[i] = curr_ba[i] - m->bias_accel_lin[i]; }
    lazy93(m->d_state_d_bg, dbgv, bg_diff);
    lazy93(m->d_state_d_ba, dbav, ba_diff);

    so3_inverse(s0->q, q0i);
    q_to_mat3(q0i, R0);
    for (i = 0; i < 3; ++i) x3[i] = s1->p[i] - s0->p[i] - s0->v[i] * dt - ((0.5f * g[i]) * dt) * dt;
    m3_mulv(R0, x3, tmp);
    for (i = 0; i < 3; ++i) res[i] = tmp[i] - ((m->delta.p[i] + bg_diff[i]) + ba_diff[i]);

    {
        float E[4], a[4], b[4], c4[4], d4[4], q1i[4], w3[3] = {bg_diff[3], bg_diff[4], bg_diff[5]};
        so3_exp(w3, E);
        so3_mul(E, m->delta.q, a);
        so3_inverse(s1->q, q1i);
        so3_mul(a, q1i, b);
        so3_mul(b, s0->q, c4);
        (void)d4;
        so3_log(c4, w3);
        res[3] = w3[0]; res[4] = w3[1]; res[5] = w3[2];
    }

    for (i = 0; i < 3; ++i) x3[i] = s1->v[i] - s0->v[i] - g[i] * dt;
    m3_mulv(R0, x3, tmp2);
    for (i = 0; i < 3; ++i) res[6 + i] = tmp2[i] - ((m->delta.v[i] + bg_diff[6 + i]) + ba_diff[6 + i]);

    if (d0 || d1) {
        float J[9], h[9], t[9];
        const float rv[3] = {res[3], res[4], res[5]};
        jac_inv(rv, J, +1);
        if (d0) {
            for (i = 0; i < 81; ++i) d0[i] = 0.0f;
            for (c = 0; c < 3; ++c)
                for (r = 0; r < 3; ++r) {
                    M9(d0, r, c) = -R0[r + 3 * c];
                    M9(d0, r, 6 + c) = (-R0[r + 3 * c]) * dt;
                    M9(d0, 6 + r, 6 + c) = -R0[r + 3 * c];
                }
            hat3(tmp, h);
            m3_mul(h, R0, t);
            for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) M9(d0, r, 3 + c) = t[r + 3 * c];
            m3_mul(J, R0, t);
            for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) M9(d0, 3 + r, 3 + c) = t[r + 3 * c];
            hat3(tmp2, h);
            m3_mul(h, R0, t);
            for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) M9(d0, 6 + r, 3 + c) = t[r + 3 * c];
        }
        if (d1) {
            float nJ[9];
            for (i = 0; i < 81; ++i) d1[i] = 0.0f;
            for (i = 0; i < 9; ++i) nJ[i] = -J[i];
            m3_mul(nJ, R0, t);
            for (c = 0; c < 3; ++c)
                for (r = 0; r < 3; ++r) {
                    M9(d1, r, c) = R0[r + 3 * c];
                    M9(d1, 3 + r, 3 + c) = t[r + 3 * c];
                    M9(d1, 6 + r, 6 + c) = R0[r + 3 * c];
                }
        }
    }
    if (dba) {
        for (i = 0; i < 27; ++i) dba[i] = -m->d_state_d_ba[i];
    }
    if (dbg) {
        float Jl[9], blk[9], t[9];
        const float rv[3] = {res[3], res[4], res[5]};
        for (i = 0; i < 27; ++i) dbg[i] = -m->d_state_d_bg[i];
        jac_inv(rv, Jl, -1);
        for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) blk[r + 3 * c] = m->d_state_d_bg[(3 + r) + 9 * c];
        m3_mul(Jl, blk, t);
        for (c = 0; c < 3; ++c) for (r = 0; r < 3; ++r) dbg[(3 + r) + 9 * c] = t[r + 3 * c];
    }
}

/* ------------------------------------------------------------------ compute_sqrt_cov_inv (LDLT + triangular solve) */

#define L9(a, r, c) ((a)[(r) + 9 * (c)])

/* Eigen 3.4.0 ldlt_inplace<Lower>::unblocked on a copy of cov_ (9x9) */
static void ldlt9(float mat[81], int trans[9]) {
    const int n = 9;
    float temp[9];
    int k, i, r;
    for (k = 0; k < n; ++k) {
        int idx = k;
        float best = fabsf(L9(mat, k, k));
        int rs;
        float realAkk;
        for (i = k + 1; i < n; ++i) {
            const float v = fabsf(L9(mat, i, i));
            if (v > best) { best = v; idx = i; }
        }
        trans[k] = idx;
        if (k != idx) {
            COV(10);
            const int s = n - idx - 1;
            for (i = 0; i < k; ++i) { const float t = L9(mat, k, i); L9(mat, k, i) = L9(mat, idx, i); L9(mat, idx, i) = t; }
            for (i = 0; i < s; ++i) { const float t = L9(mat, idx + 1 + i, k); L9(mat, idx + 1 + i, k) = L9(mat, idx + 1 + i, idx); L9(mat, idx + 1 + i, idx) = t; }
            { const float t = L9(mat, k, k); L9(mat, k, k) = L9(mat, idx, idx); L9(mat, idx, idx) = t; }
            for (i = k + 1; i < idx; ++i) { const float t = L9(mat, i, k); L9(mat, i, k) = L9(mat, idx, i); L9(mat, idx, i) = t; }
        }
        rs = n - k - 1;
        if (k > 0) {
            float v;
            for (i = 0; i < k; ++i) temp[i] = L9(mat, i, i) * L9(mat, k, i);
            v = L9(mat, k, 0) * temp[0];
            for (i = 1; i < k; ++i) v = v + L9(mat, k, i) * temp[i];
            L9(mat, k, k) = L9(mat, k, k) - v;
            if (rs == 1) {
                float dot = L9(mat, k + 1, 0) * temp[0];
                for (i = 1; i < k; ++i) dot = dot + L9(mat, k + 1, i) * temp[i];
                L9(mat, k + 1, k) = L9(mat, k + 1, k) + (-1.0f) * dot;
            } else if (rs > 1) {
                for (r = 0; r < rs; ++r) {
                    float acc = 0.0f;
                    for (i = 0; i < k; ++i) acc = acc + L9(mat, k + 1 + r, i) * temp[i];
                    L9(mat, k + 1 + r, k) = acc * (-1.0f) + L9(mat, k + 1 + r, k);
                }
            }
        }
        realAkk = L9(mat, k, k);
        if (k == 0 && !(fabsf(realAkk) > 0.0f)) {
            COV(12);
            for (i = 0; i < n; ++i) trans[i] = i;
            return;
        }
        if (rs > 0 && fabsf(realAkk) > 0.0f)
            for (r = 0; r < rs; ++r) L9(mat, k + 1 + r, k) = L9(mat, k + 1 + r, k) / realAkk;
    }
}

/* Eigen 3.4.0 triangular_solve_matrix<OnTheLeft, UnitLower, ColMajor, ColMajor>, float (SmallPanelWidth = max(mr=8, nr=4) = 8), size 9:
 * one 8-wide panel solved column-by-column, then the 1-row remainder through the scalar GEBP path, then the 1x1 panel */
static void trisolve_unit_lower9(const float L[81], float S[81]) {
    int j, k, i3;
    for (k = 0; k < 8; ++k)
        for (j = 0; j < 9; ++j) {
            const float b = L9(S, k, j) * 1.0f;
            L9(S, k, j) = b;
            for (i3 = 0; i3 < 7 - k; ++i3) L9(S, k + 1 + i3, j) = L9(S, k + 1 + i3, j) - b * L9(L, k + 1 + i3, k);
        }
    for (j = 0; j < 9; ++j) {
        const float C0 = gebp_acc(&L9(L, 8, 0), 1, 9, S, 1, 9, 0, j, 1, 9, 8); /* gebp of 1 row x 8 deep x 9 columns */
        L9(S, 8, j) = C0 * (-1.0f) + L9(S, 8, j);
    }
}

/* test hooks (validated separately in bs_imu_test prim) */
void bs_imu_dbg_gemm(int rows, int cols, int depth, const float* A, const float* B, float* out) { gemm_fold(rows, cols, depth, A, 1, rows, B, 1, depth, out); }
void bs_imu_ldlt9(float mat[81], int trans[9]) { ldlt9(mat, trans); }
void bs_imu_trisolve_unit_lower9(const float L[81], float S[81]) { trisolve_unit_lower9(L, S); }

const float* bs_imu_sqrt_cov_inv(bs_imu_meas* m) {
    if (!m->sqrt_cov_inv_computed) {
        float mat[81], D_inv_sqrt[9];
        int trans[9], i, j;
        float* S = m->sqrt_cov_inv;
        memcpy(mat, m->cov, sizeof(mat));
        ldlt9(mat, trans);
        for (i = 0; i < 81; ++i) S[i] = 0.0f;
        for (i = 0; i < 9; ++i) S[i + 9 * i] = 1.0f;
        for (i = 0; i < 9; ++i)
            if (trans[i] != i)
                for (j = 0; j < 9; ++j) { const float t = L9(S, i, j); L9(S, i, j) = L9(S, trans[i], j); L9(S, trans[i], j) = t; }
        trisolve_unit_lower9(mat, S);
        for (i = 0; i < 9; ++i) {
            const float d = L9(mat, i, i);
            if (d < FLT_MIN) { COV(11); D_inv_sqrt[i] = 0.0f; }
            else D_inv_sqrt[i] = 1.0f / sqrtf(d);
        }
        for (j = 0; j < 9; ++j)
            for (i = 0; i < 9; ++i) L9(S, i, j) = D_inv_sqrt[i] * L9(S, i, j);
        m->sqrt_cov_inv_computed = 1;
    }
    return m->sqrt_cov_inv;
}

/* ------------------------------------------------------------------ ImuBlock::linearizeImu */

static float sq9(const float x[9]) { /* (cwiseAbs2 of a 9-vector).sum(): one 4-lane packet add, predux, then the odd coefficient */
    float s[9], l0, l1, l2, l3;
    int i;
    for (i = 0; i < 9; ++i) s[i] = x[i] * x[i];
    l0 = s[0] + s[4]; l1 = s[1] + s[5]; l2 = s[2] + s[6]; l3 = s[3] + s[7];
    return ((l0 + l2) + (l1 + l3)) + s[8];
}

float bs_imu_linearize(bs_imu_meas* m, const float g[3], const float gyro_bias_weight_sqrt[3], const float accel_bias_weight_sqrt[3],
                       const bs_pvb_with_lin* start, const bs_pvb_with_lin* end, float Jp[450], float r[15]) {
    const bs_pvbstate* st = start->linearized ? &start->cur : &start->lin;
    const bs_pvbstate* en = end->linearized ? &end->cur : &end->lin;
    const float* S;
    float res[9], d0[81], d1[81], dbg[27], dba[27], tmpv[9], tmpm[81];
    float imu_error, dt, gwdt[3], awdt[3], res_bg[3], res_ba[3], bg_error, ba_error;
    int i, rr, cc;

    for (i = 0; i < 450; ++i) Jp[i] = 0.0f;
    for (i = 0; i < 15; ++i) r[i] = 0.0f;

    bs_imu_residual(m, &start->lin.s, g, &end->lin.s, start->lin.bg, start->lin.ba, res, d0, d1, dbg, dba);
    if (start->linearized || end->linearized) bs_imu_residual(m, &st->s, g, &en->s, st->bg, st->ba, res, NULL, NULL, NULL, NULL);

    S = bs_imu_sqrt_cov_inv(m);
    gemm_fold(9, 1, 9, S, 1, 9, res, 1, 9, tmpv); /* get_sqrt_cov_inv() * res */
    imu_error = 0.5f * sq9(tmpv);

#define JP(row, col) Jp[(row) + 15 * (col)]
    gemm_fold(9, 9, 9, S, 1, 9, d0, 1, 9, tmpm);
    for (cc = 0; cc < 9; ++cc) for (rr = 0; rr < 9; ++rr) JP(rr, cc) = tmpm[rr + 9 * cc];
    gemm_fold(9, 9, 9, S, 1, 9, d1, 1, 9, tmpm);
    for (cc = 0; cc < 9; ++cc) for (rr = 0; rr < 9; ++rr) JP(rr, 15 + cc) = tmpm[rr + 9 * cc];
    gemm_fold(9, 3, 9, S, 1, 9, dbg, 1, 9, tmpm);
    for (cc = 0; cc < 3; ++cc) for (rr = 0; rr < 9; ++rr) JP(rr, 9 + cc) = tmpm[rr + 9 * cc];
    gemm_fold(9, 3, 9, S, 1, 9, dba, 1, 9, tmpm);
    for (cc = 0; cc < 3; ++cc) for (rr = 0; rr < 9; ++rr) JP(rr, 12 + cc) = tmpm[rr + 9 * cc];
    for (i = 0; i < 9; ++i) r[i] = tmpv[i];

    dt = (float)m->delta.t_ns * (float)1e-9;
    {
        const float sdt = sqrtf(dt);
        for (i = 0; i < 3; ++i) gwdt[i] = gyro_bias_weight_sqrt[i] / sdt;
        for (i = 0; i < 3; ++i) res_bg[i] = st->bg[i] - en->bg[i];
        for (i = 0; i < 3; ++i) { JP(9 + i, 9 + i) = gwdt[i]; JP(9 + i, 15 + 9 + i) = -gwdt[i]; }
        for (i = 0; i < 3; ++i) r[9 + i] = r[9 + i] + gwdt[i] * res_bg[i];
        { float p[3]; for (i = 0; i < 3; ++i) p[i] = gwdt[i] * res_bg[i]; bg_error = 0.5f * sq3(p); }

        for (i = 0; i < 3; ++i) awdt[i] = accel_bias_weight_sqrt[i] / sdt;
        for (i = 0; i < 3; ++i) res_ba[i] = st->ba[i] - en->ba[i];
        for (i = 0; i < 3; ++i) { JP(12 + i, 12 + i) = awdt[i]; JP(12 + i, 15 + 12 + i) = -awdt[i]; }
        for (i = 0; i < 3; ++i) r[12 + i] = r[12 + i] + awdt[i] * res_ba[i];
        { float p[3]; for (i = 0; i < 3; ++i) p[i] = awdt[i] * res_ba[i]; ba_error = 0.5f * sq3(p); }
    }
#undef JP
    return imu_error + bg_error + ba_error;
}
