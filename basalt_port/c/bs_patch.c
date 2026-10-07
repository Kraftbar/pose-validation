/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Basalt pure-C port, module M5 (part 1): SE2 optical-flow patch. See bs_patch.h for the layout and the executed-path note. */
#include "bs_patch.h"

#include <float.h>
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ patterns (patterns.h) */

static const int pattern52_raw[BS_PAT][2] = {
    {-3, 7},  {-1, 7},  {1, 7},   {3, 7},
    {-5, 5},  {-3, 5},  {-1, 5},  {1, 5},   {3, 5},  {5, 5},
    {-7, 3},  {-5, 3},  {-3, 3},  {-1, 3},  {1, 3},  {3, 3},  {5, 3},   {7, 3},
    {-7, 1},  {-5, 1},  {-3, 1},  {-1, 1},  {1, 1},  {3, 1},  {5, 1},   {7, 1},
    {-7, -1}, {-5, -1}, {-3, -1}, {-1, -1}, {1, -1}, {3, -1}, {5, -1},  {7, -1},
    {-7, -3}, {-5, -3}, {-3, -3}, {-1, -3}, {1, -3}, {3, -3}, {5, -3},  {7, -3},
    {-5, -5}, {-3, -5}, {-1, -5}, {1, -5},  {3, -5}, {5, -5},
    {-3, -7}, {-1, -7}, {1, -7},  {3, -7}};

int bs_pattern_init(float out[2 * BS_PAT], int pattern) {
    int i;
    float s;
    if (pattern == 52) s = 1.0f;
    else if (pattern == 51) s = 0.5f;
    else if (pattern == 50) s = 0.75f;
    else return 0;
    for (i = 0; i < BS_PAT; i++) {
        out[2 * i] = s * (float)pattern52_raw[i][0];      /* 0.5 / 0.75 * small int: exact in float */
        out[2 * i + 1] = s * (float)pattern52_raw[i][1];
    }
    return 1;
}

/* ------------------------------------------------------------------ Image: InBounds / interp / interpGrad (basalt-headers image.h) */

#define PX(im, x, y) ((float)(im)->p[(size_t)(y) * (size_t)(im)->pitch + (size_t)(x)])

/* InBounds(MatrixBase p, Scalar border): border <= p0 && p0 < ((int)w - border - 1) && same for y (float arithmetic) */
int bs_img_inbounds(const bs_imgv *im, float x, float y, float border) {
    const float offset = 1.0f;
    return border <= x && x < ((float)im->w - border - offset) && border <= y && y < ((float)im->h - border - offset);
}

float bs_img_interp(const bs_imgv *im, float x, float y) {
    const int ix = (int)x, iy = (int)y;
    const float dx = x - (float)ix, dy = y - (float)iy;
    const float ddx = 1.0f - dx, ddy = 1.0f - dy;
    return ddx * ddy * PX(im, ix, iy) + ddx * dy * PX(im, ix, iy + 1) + dx * ddy * PX(im, ix + 1, iy) + dx * dy * PX(im, ix + 1, iy + 1);
}

void bs_img_interp_grad(const bs_imgv *im, float x, float y, float o[3]) {
    const int ix = (int)x, iy = (int)y;
    const float dx = x - (float)ix, dy = y - (float)iy;
    const float ddx = 1.0f - dx, ddy = 1.0f - dy;
    const float px0y0 = PX(im, ix, iy), px1y0 = PX(im, ix + 1, iy), px0y1 = PX(im, ix, iy + 1), px1y1 = PX(im, ix + 1, iy + 1);
    const float pxm1y0 = PX(im, ix - 1, iy), pxm1y1 = PX(im, ix - 1, iy + 1);
    const float px2y0 = PX(im, ix + 2, iy), px2y1 = PX(im, ix + 2, iy + 1);
    const float px0ym1 = PX(im, ix, iy - 1), px1ym1 = PX(im, ix + 1, iy - 1);
    const float px0y2 = PX(im, ix, iy + 2), px1y2 = PX(im, ix + 1, iy + 2);
    float res_mx, res_px, res_my, res_py;
    o[0] = ddx * ddy * px0y0 + ddx * dy * px0y1 + dx * ddy * px1y0 + dx * dy * px1y1;
    res_mx = ddx * ddy * pxm1y0 + ddx * dy * pxm1y1 + dx * ddy * px0y0 + dx * dy * px0y1;
    res_px = ddx * ddy * px1y0 + ddx * dy * px1y1 + dx * ddy * px2y0 + dx * dy * px2y1;
    o[1] = 0.5f * (res_px - res_mx);
    res_my = ddx * ddy * px0ym1 + ddx * dy * px0y0 + dx * ddy * px1ym1 + dx * dy * px1y0;
    res_py = ddx * ddy * px0y1 + ddx * dy * px0y2 + dx * ddy * px1y1 + dx * dy * px1y2;
    o[2] = 0.5f * (res_py - res_my);
}

/* ------------------------------------------------------------------ Eigen 3.4.0 models (SSE, packet 4, no FMA) */

/* Eigen: J^T * J (3x52 * 52x3) is a GemmProduct (rows + depth + cols = 58 >= 20): GEBP with 3 remainder rows and 3 leftover columns = per coefficient
 * a scalar left fold from k = 0 (C0 = 0; C0 += a_k * b_k), then dst = 0 + 1 * C0. */
float bs_patch_dbg_dot52(const float *a, const float *b) {
    float acc = 0.0f;
    int k;
    for (k = 0; k < BS_PAT; k++) acc = acc + a[k] * b[k];
    return 1.0f * acc;
}

/* Eigen: H.ldlt() of a Matrix3f (ldlt_inplace<Lower>::unblocked, first-max diagonal pivoting, same code as bs_imu.c ldlt9 with n = 3) followed by
 * solveInPlace(Identity): P*b, unit-lower triangular_solve_matrix (one 3-wide panel, column oriented r -= b*l), D^-1 as a row division
 * (zero row when |D| <= FLT_MIN), unit-upper RowMajor solve (b = sum l*r from 0, other = (other - b) * 1), P^T. */
#define M3(a, r, c) ((a)[(r) + 3 * (c)])
void bs_patch_dbg_ldlt3_inverse(const float H[9], float Hinv[9]) {
    float m[9], temp[3];
    int tr[3], k, i, j, r, i3;
    memcpy(m, H, sizeof m);
    for (k = 0; k < 3; ++k) tr[k] = k;
    for (k = 0; k < 3; ++k) {
        int idx = k, rs;
        float best = fabsf(M3(m, k, k)), realAkk;
        for (i = k + 1; i < 3; ++i) { const float v = fabsf(M3(m, i, i)); if (v > best) { best = v; idx = i; } }
        tr[k] = idx;
        if (k != idx) {
            const int s = 3 - idx - 1;
            for (i = 0; i < k; ++i) { const float t = M3(m, k, i); M3(m, k, i) = M3(m, idx, i); M3(m, idx, i) = t; }
            for (i = 0; i < s; ++i) { const float t = M3(m, idx + 1 + i, k); M3(m, idx + 1 + i, k) = M3(m, idx + 1 + i, idx); M3(m, idx + 1 + i, idx) = t; }
            { const float t = M3(m, k, k); M3(m, k, k) = M3(m, idx, idx); M3(m, idx, idx) = t; }
            for (i = k + 1; i < idx; ++i) { const float t = M3(m, i, k); M3(m, i, k) = M3(m, idx, i); M3(m, idx, i) = t; }
        }
        rs = 3 - k - 1;
        if (k > 0) {
            float v;
            for (i = 0; i < k; ++i) temp[i] = M3(m, i, i) * M3(m, k, i);
            v = M3(m, k, 0) * temp[0];
            for (i = 1; i < k; ++i) v = v + M3(m, k, i) * temp[i];
            M3(m, k, k) = M3(m, k, k) - v;
            if (rs == 1) {
                float dot = M3(m, k + 1, 0) * temp[0];
                for (i = 1; i < k; ++i) dot = dot + M3(m, k + 1, i) * temp[i];
                M3(m, k + 1, k) = M3(m, k + 1, k) + (-1.0f) * dot;
            } else if (rs > 1) {
                for (r = 0; r < rs; ++r) {
                    float acc = 0.0f;
                    for (i = 0; i < k; ++i) acc = acc + M3(m, k + 1 + r, i) * temp[i];
                    M3(m, k + 1 + r, k) = acc * (-1.0f) + M3(m, k + 1 + r, k);
                }
            }
        }
        realAkk = M3(m, k, k);
        if (k == 0 && !(fabsf(realAkk) > 0.0f)) { for (i = 0; i < 3; ++i) tr[i] = i; break; }
        if (rs > 0 && fabsf(realAkk) > 0.0f)
            for (r = 0; r < rs; ++r) M3(m, k + 1 + r, k) = M3(m, k + 1 + r, k) / realAkk;
    }
    for (i = 0; i < 9; ++i) Hinv[i] = 0.0f;
    for (i = 0; i < 3; ++i) M3(Hinv, i, i) = 1.0f;
    for (k = 0; k < 3; ++k)      /* dst = P * b */
        if (tr[k] != k)
            for (j = 0; j < 3; ++j) { const float t = M3(Hinv, k, j); M3(Hinv, k, j) = M3(Hinv, tr[k], j); M3(Hinv, tr[k], j) = t; }
    for (k = 0; k < 3; ++k)      /* L^-1 (unit lower, column oriented) */
        for (j = 0; j < 3; ++j) {
            const float b = M3(Hinv, k, j) * 1.0f;
            M3(Hinv, k, j) = b;
            for (i3 = 0; i3 < 3 - k - 1; ++i3) M3(Hinv, k + 1 + i3, j) = M3(Hinv, k + 1 + i3, j) - b * M3(m, k + 1 + i3, k);
        }
    for (i = 0; i < 3; ++i) {    /* D^-1 */
        const float d = M3(m, i, i);
        for (j = 0; j < 3; ++j) M3(Hinv, i, j) = fabsf(d) > FLT_MIN ? M3(Hinv, i, j) / d : 0.0f;
    }
    for (k = 0; k < 3; ++k) {    /* L^-T: unit upper, RowMajor storage, rows 2, 1, 0 */
        const int ii = 2 - k;
        for (j = 0; j < 3; ++j) {
            float b = 0.0f;
            for (i3 = 0; i3 < k; ++i3) b += M3(m, ii + 1 + i3, ii) * M3(Hinv, ii + 1 + i3, j);
            M3(Hinv, ii, j) = (M3(Hinv, ii, j) - b) * 1.0f;
        }
    }
    for (k = 2; k >= 0; --k)     /* dst = P^T * dst */
        if (tr[k] != k)
            for (j = 0; j < 3; ++j) { const float t = M3(Hinv, k, j); M3(Hinv, k, j) = M3(Hinv, tr[k], j); M3(Hinv, tr[k], j) = t; }
}

/* Eigen: inc = -HJ * res (3x52 * 52 vector): per row a scalar left fold (rows are strided, not packet-vectorisable) */
void bs_patch_dbg_inc(const float HJ[3 * BS_PAT], const float res[BS_PAT], float inc[3]) {
    int r, k;
    for (r = 0; r < 3; r++) {
        float acc = (-HJ[r]) * res[0];
        for (k = 1; k < BS_PAT; k++) acc = acc + (-HJ[r + 3 * k]) * res[k];
        inc[r] = acc;
    }
}

/* ------------------------------------------------------------------ OpticalFlowPatch (patch.h) */

void bs_patch_set(bs_patch *pt, const bs_imgv *im, const float pat[2 * BS_PAT], const float pos[2]) {
    float J[BS_PAT * 3];     /* MatrixP3, column-major: J[i + 52*c] */
    float sum = 0.0f, gs[3] = {0.0f, 0.0f, 0.0f}, mean_inv, H[9], Hinv[9];
    int nv = 0, i, a, b, c;
    pt->pos[0] = pos[0];
    pt->pos[1] = pos[1];
    for (i = 0; i < BS_PAT; i++) {
        const float px = pos[0] + pat[2 * i], py = pos[1] + pat[2 * i + 1];
        const float jw02 = -pat[2 * i + 1], jw12 = pat[2 * i];      /* Jw_se2(0,2) = -pattern(1,i), Jw_se2(1,2) = pattern(0,i) */
        if (bs_img_inbounds(im, px, py, 2.0f)) {
            float vg[3];
            bs_img_interp_grad(im, px, py, vg);
            pt->data[i] = vg[0];
            sum += vg[0];
            /* J.row(i) = valGrad.tail<2>().transpose() * Jw_se2  (1x2 * 2x3 = [1 0 jw02; 0 1 jw12]) */
            J[i] = vg[1] * 1.0f + vg[2] * 0.0f;
            J[i + BS_PAT] = vg[1] * 0.0f + vg[2] * 1.0f;
            J[i + 2 * BS_PAT] = vg[1] * jw02 + vg[2] * jw12;
            for (c = 0; c < 3; c++) gs[c] += J[i + BS_PAT * c];
            nv++;
        } else {
            pt->data[i] = -1.0f;
        }
    }
    pt->mean = sum / (float)nv;
    mean_inv = (float)nv / sum;
    for (i = 0; i < BS_PAT; i++) {
        if (pt->data[i] >= 0.0f) {
            for (c = 0; c < 3; c++) J[i + BS_PAT * c] = J[i + BS_PAT * c] - (gs[c] * pt->data[i]) / sum;
            pt->data[i] = pt->data[i] * mean_inv;
        } else {
            for (c = 0; c < 3; c++) J[i + BS_PAT * c] = 0.0f;
        }
    }
    for (i = 0; i < BS_PAT * 3; i++) J[i] = J[i] * mean_inv;

    for (a = 0; a < 3; a++)
        for (b = 0; b < 3; b++) H[a + 3 * b] = bs_patch_dbg_dot52(&J[BS_PAT * a], &J[BS_PAT * b]);   /* J^T * J: lazy coefficient product, vectorised redux */
    bs_patch_dbg_ldlt3_inverse(H, Hinv);
    for (i = 0; i < BS_PAT; i++)
        for (a = 0; a < 3; a++)      /* Hinv * J^T: x0 + (x1 + x2) per coefficient */
            pt->HJ[a + 3 * i] = Hinv[a] * J[i] + (Hinv[a + 3] * J[i + BS_PAT] + Hinv[a + 6] * J[i + 2 * BS_PAT]);

    {
        int fin = 1;
        for (i = 0; i < 3 * BS_PAT; i++) fin &= isfinite(pt->HJ[i]) != 0;
        for (i = 0; i < BS_PAT; i++) fin &= isfinite(pt->data[i]) != 0;
        pt->valid = pt->mean > FLT_EPSILON && fin;
    }
}

int bs_patch_residual(const bs_patch *pt, const bs_imgv *im, const float tp[2 * BS_PAT], float res[BS_PAT]) {
    float sum = 0.0f;
    int nv = 0, nres = 0, i;
    for (i = 0; i < BS_PAT; i++) {
        if (bs_img_inbounds(im, tp[2 * i], tp[2 * i + 1], 2.0f)) {
            res[i] = bs_img_interp(im, tp[2 * i], tp[2 * i + 1]);
            sum += res[i];
            nv++;
        } else {
            res[i] = -1.0f;
        }
    }
    if (sum < FLT_EPSILON) {
        for (i = 0; i < BS_PAT; i++) res[i] = 0.0f;
        return 0;
    }
    for (i = 0; i < BS_PAT; i++) {
        if (res[i] >= 0.0f && pt->data[i] >= 0.0f) {
            const float val = res[i];
            res[i] = (float)nv * val / sum - pt->data[i];
            nres++;
        } else {
            res[i] = 0.0f;
        }
    }
    return nres > BS_PAT / 2;
}

/* ------------------------------------------------------------------ SE2 */

void bs_se2_exp_matrix(const float inc[3], float M[9]) {
    const float theta = inc[2];
    float c = cosf(theta), s = sinf(theta), len, sin_by, omc_by, tx, ty;
    len = hypotf(c, s);
    c = c / len;     /* SO2(real, imag) normalises: unit_complex /= hypot */
    s = s / len;
    if (fabsf(theta) < 1e-5f) {
        const float theta_sq = theta * theta;
        sin_by = 1.0f - (float)(1.0 / 6.0) * theta_sq;
        omc_by = 0.5f * theta - (float)(1.0 / 24.0) * theta * theta_sq;
    } else {
        sin_by = s / theta;
        omc_by = (1.0f - c) / theta;
    }
    tx = sin_by * inc[0] - omc_by * inc[1];
    ty = omc_by * inc[0] + sin_by * inc[1];
    M[0] = c; M[1] = s; M[2] = 0.0f;
    M[3] = -s; M[4] = c; M[5] = 0.0f;
    M[6] = tx; M[7] = ty; M[8] = 1.0f;
}

/* Transform<float,2,AffineCompact> *= Matrix3f: res.topRows(2) = affine() * other (2x3 * 3x3, lazy coefficient product, x0 + (x1 + x2)) */
void bs_affine_mul_assign(float m[6], const float M[9]) {
    float r[6];
    int i, j;
    for (j = 0; j < 3; j++)
        for (i = 0; i < 2; i++)
            r[i + 2 * j] = m[i] * M[3 * j] + (m[i + 2] * M[3 * j + 1] + m[i + 4] * M[3 * j + 2]);
    memcpy(m, r, sizeof r);
}

/* ------------------------------------------------------------------ trackPointAtLevel */

int bs_track_point_at_level(const bs_imgv *img2, const bs_patch *dp, const float pat[2 * BS_PAT], int max_iter, float tr[6]) {
    int valid = 1, it;
    for (it = 0; valid && it < max_iter; it++) {
        float tp[2 * BS_PAT], res[BS_PAT];
        int i;
        for (i = 0; i < BS_PAT; i++) {
            tp[2 * i] = tr[0] * pat[2 * i] + tr[2] * pat[2 * i + 1];
            tp[2 * i + 1] = tr[1] * pat[2 * i] + tr[3] * pat[2 * i + 1];
        }
        for (i = 0; i < BS_PAT; i++) { tp[2 * i] += tr[4]; tp[2 * i + 1] += tr[5]; }
        valid &= bs_patch_residual(dp, img2, tp, res);
        if (valid) {
            float inc[3], mx, M[9];
            bs_patch_dbg_inc(dp->HJ, res, inc);
            valid &= isfinite(inc[0]) && isfinite(inc[1]) && isfinite(inc[2]);
            mx = fabsf(inc[0]);
            if (fabsf(inc[1]) > mx) mx = fabsf(inc[1]);
            if (fabsf(inc[2]) > mx) mx = fabsf(inc[2]);
            valid &= mx < 1e6;
            if (valid) {
                bs_se2_exp_matrix(inc, M);
                bs_affine_mul_assign(tr, M);
                valid &= bs_img_inbounds(img2, tr[4], tr[5], 2.0f);
            }
        }
    }
    return valid;
}
