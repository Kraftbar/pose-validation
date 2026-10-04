/* SPDX-License-Identifier: BSD-3-Clause */
/* OKVIS2 pure-C port, module 6, pure helpers of ViSlamBackend: the sorted id sets (std::set<StateId>), the keypoint
 * overlap between two multiframes (ViSlamBackend::overlapFraction) with the OpenCV 4.6 filled cv::circle rasteriser
 * (BSD-3-Clause / Apache-2.0, see okvis_port/NOTICE), and the Eigen quaternion helpers of attemptLoopClosure
 * (MPL-2.0). Derived from OKVIS2 (BSD-3-Clause, see okvis_port/NOTICE). C99, <stdint.h> <math.h> <stdlib.h> <string.h>. */
#include "ok_vslam.h"
#include "ok_eigen.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int idset_lb(const ok_idset* s, uint64_t v) {
    int lo = 0, hi = s->n;
    while (lo < hi) { const int mid = (lo + hi) / 2; if (s->a[mid] < v) lo = mid + 1; else hi = mid; }
    return lo;
}
int ok_idset_has(const ok_idset* s, uint64_t v) { const int i = idset_lb(s, v); return i < s->n && s->a[i] == v; }
int ok_idset_add(ok_idset* s, uint64_t v) {
    const int i = idset_lb(s, v);
    if (i < s->n && s->a[i] == v) return 0;
    if (s->n == s->cap) { s->cap = s->cap ? 2 * s->cap : 8; s->a = (uint64_t*)realloc(s->a, sizeof(uint64_t) * (size_t)s->cap); }
    memmove(&s->a[i + 1], &s->a[i], sizeof(uint64_t) * (size_t)(s->n - i));
    s->a[i] = v; s->n++;
    return 1;
}
int ok_idset_del(ok_idset* s, uint64_t v) {
    const int i = idset_lb(s, v);
    if (i >= s->n || s->a[i] != v) return 0;
    memmove(&s->a[i], &s->a[i + 1], sizeof(uint64_t) * (size_t)(s->n - i - 1));
    s->n--;
    return 1;
}
void ok_idset_free(ok_idset* s) { free(s->a); s->a = NULL; s->n = s->cap = 0; }

/* ------------------------------------------------------------------------------------------------------------------
 * overlap between two multiframes (ViSlamBackend::overlapFraction) with OpenCV's filled cv::circle
 * ---------------------------------------------------------------------------------------------------------------- */
void ok_vsb_circle_filled(unsigned char* img, int rows, int cols, int cx, int cy, int radius) {
    int err = 0, dx = radius, dy = 0, plus = 1, minus = (radius << 1) - 1;
    const int inside = cx >= radius && cx < cols - radius && cy >= radius && cy < rows - radius;
    if (radius < 0) return;
#define HLINE(row, xl, xr) do { int xx_; for (xx_ = (xl); xx_ <= (xr); ++xx_) img[(size_t)(row) * (size_t)cols + (size_t)xx_] = 255; } while (0)
    while (dx >= dy) {
        int mask;
        const int y11 = cy - dy, y12 = cy + dy, y21 = cy - dx, y22 = cy + dx;
        int x11 = cx - dx, x12 = cx + dx, x21 = cx - dy, x22 = cx + dy;
        if (inside) {
            HLINE(y11, x11, x12);
            HLINE(y12, x11, x12);
            HLINE(y21, x21, x22);
            HLINE(y22, x21, x22);
        } else if (x11 < cols && x12 >= 0 && y21 < rows && y22 >= 0) {
            if (x11 < 0) x11 = 0;
            if (x12 > cols - 1) x12 = cols - 1;
            if ((unsigned)y11 < (unsigned)rows) HLINE(y11, x11, x12);
            if ((unsigned)y12 < (unsigned)rows) HLINE(y12, x11, x12);
            if (x21 < cols && x22 >= 0) {
                if (x21 < 0) x21 = 0;
                if (x22 > cols - 1) x22 = cols - 1;
                if ((unsigned)y21 < (unsigned)rows) HLINE(y21, x21, x22);
                if ((unsigned)y22 < (unsigned)rows) HLINE(y22, x21, x22);
            }
        }
        dy++;
        err += plus;
        plus += 2;
        mask = (err <= 0) - 1;
        err -= minus & mask;
        dx += mask;
        minus -= mask & 2;
    }
#undef HLINE
}

static int f2i(float v) { return (int)lrintf(v); }          /* cvRound(float) */

double ok_vsb_overlap(const ok_vsb_frame_view* fa, const ok_vsb_frame_view* fb, double kptradius) {
    const ok_vsb_frame_view* fr[2];
    ok_idset lms[2], matches;
    unsigned char* det[2][OK_VSB_MAXCAM];
    unsigned char* mat[2][OK_VSB_MAXCAM];
    double overlap[2];
    int f, im, k, i, j;
    fr[0] = fa; fr[1] = fb;
    memset(lms, 0, sizeof lms); memset(&matches, 0, sizeof matches);
    memset(det, 0, sizeof det); memset(mat, 0, sizeof mat);
    for (f = 0; f < 2; ++f)
        for (im = 0; im < fr[f]->ncam; ++im) {
            const ok_vsb_cam_view* c = &fr[f]->cam[im];
            int rows, cols, radius;
            double rad;
            if (c->images_cleared) continue;
            rows = c->rows / 10; cols = c->cols / 10;
            rad = (double)(rows < cols ? rows : cols) * kptradius;
            radius = (int)rad;
            det[f][im] = (unsigned char*)calloc((size_t)(rows * cols) + 1, 1);
            mat[f][im] = (unsigned char*)calloc((size_t)(rows * cols) + 1, 1);
            for (k = 0; k < c->nkp; ++k) {
                const float px = (float)((double)c->kp[3 * k] * 0.1), py = (float)((double)c->kp[3 * k + 1] * 0.1);
                ok_vsb_circle_filled(det[f][im], rows, cols, f2i(px), f2i(py), radius);
                if (c->lm[k] != 0) ok_idset_add(&lms[f], c->lm[k]);
            }
        }
    for (i = 0, j = 0; i < lms[0].n && j < lms[1].n;) {         /* std::set_intersection */
        if (lms[0].a[i] < lms[1].a[j]) ++i;
        else if (lms[1].a[j] < lms[0].a[i]) ++j;
        else { ok_idset_add(&matches, lms[0].a[i]); ++i; ++j; }
    }
    if (matches.n == 0) {
        for (f = 0; f < 2; ++f) for (im = 0; im < OK_VSB_MAXCAM; ++im) { free(det[f][im]); free(mat[f][im]); }
        ok_idset_free(&lms[0]); ok_idset_free(&lms[1]); ok_idset_free(&matches);
        return 0.0;
    }
    for (f = 0; f < 2; ++f)
        for (im = 0; im < fr[f]->ncam; ++im) {
            const ok_vsb_cam_view* c = &fr[f]->cam[im];
            int rows, cols, radius;
            double rad;
            if (c->images_cleared) continue;
            rows = c->rows / 10; cols = c->cols / 10;
            rad = (double)(rows < cols ? rows : cols) * kptradius;
            radius = (int)rad;
            for (k = 0; k < c->nkp; ++k)
                if (ok_idset_has(&matches, c->lm[k])) {
                    const float px = (float)((double)c->kp[3 * k] * 0.1), py = (float)((double)c->kp[3 * k + 1] * 0.1);
                    ok_vsb_circle_filled(mat[f][im], rows, cols, f2i(px), f2i(py), radius);
                }
        }
    for (f = 0; f < 2; ++f) {
        int inter = 0, uni = 0;
        for (im = 0; im < fr[f]->ncam; ++im) {
            const ok_vsb_cam_view* c = &fr[f]->cam[im];
            int p, np;
            if (c->images_cleared) continue;
            np = (c->rows / 10) * (c->cols / 10);
            for (p = 0; p < np; ++p) {
                if (mat[f][im][p] && det[f][im][p]) inter++;
                if (mat[f][im][p] || det[f][im][p]) uni++;
            }
        }
        overlap[f] = (double)inter / (double)uni;
    }
    for (f = 0; f < 2; ++f) for (im = 0; im < OK_VSB_MAXCAM; ++im) { free(det[f][im]); free(mat[f][im]); }
    ok_idset_free(&lms[0]); ok_idset_free(&lms[1]); ok_idset_free(&matches);
    return overlap[1] < overlap[0] ? overlap[1] : overlap[0];                 /* std::min(a, b) = (b < a) ? b : a */
}


/* MatrixBase::stableNorm() of a 3-vector whose first element is default-aligned (one kernel call over all of it) */
double ok_v3_stable_norm(const double v[3]) {
    double mx = fabs(v[0]), scale = 0.0, inv_scale = 1.0, ssq = 0.0;
    if (fabs(v[1]) > mx) mx = fabs(v[1]);
    if (fabs(v[2]) > mx) mx = fabs(v[2]);
    if (mx > scale) {
        const double r = scale / mx;
        const double tmp = 1.0 / mx;
        ssq = ssq * (r * r);
        if (tmp > 1.7976931348623157e308) { inv_scale = 1.7976931348623157e308; scale = 1.0 / inv_scale; }
        else if (mx > 1.7976931348623157e308) { inv_scale = 1.0; scale = mx; }
        else { scale = mx; inv_scale = tmp; }
    } else if (mx != mx) scale = mx;
    if (scale > 0.0) {
        const double a = v[0] * inv_scale, b = v[1] * inv_scale, c = v[2] * inv_scale;
        ssq += (a * a + b * b) + c * c;
    }
    return scale * sqrt(ssq);
}

/* ---- Eigen quaternion helpers of attemptLoopClosure (Eigen 3.4.0 AngleAxis.h / Quaternion.h) ---- */
void ok_quat_to_angle_axis(const ok_quat* q, double* angle, double axis[3]) {      /* AngleAxis::operator=(Quaternion) */
    double v[3], n;
    v[0] = q->x; v[1] = q->y; v[2] = q->z;
    n = ok_v3_norm(v);                                                                 /* q.vec().norm() */
    if (n < 2.220446049250313e-16) n = ok_v3_stable_norm(v);                          /* n = q.vec().stableNorm() */
    if (n != 0.0) {
        *angle = 2.0 * atan2(n, fabs(q->w));
        if (q->w < 0.0) n = -n;
        axis[0] = v[0] / n; axis[1] = v[1] / n; axis[2] = v[2] / n;
    } else { *angle = 0.0; axis[0] = 1.0; axis[1] = 0.0; axis[2] = 0.0; }
}
ok_quat ok_quat_from_angle_axis(double angle, const double axis[3]) {                  /* Quaternion::operator=(AngleAxis) */
    ok_quat q; const double ha = 0.5 * angle, sh = sin(ha);
    q.w = cos(ha); q.x = sh * axis[0]; q.y = sh * axis[1]; q.z = sh * axis[2];
    return q;
}
double ok_quat_angular_distance(const ok_quat* a, const ok_quat* b) {                  /* QuaternionBase::angularDistance */
    ok_quat bc, d; double dv[3];
    bc.x = -b->x; bc.y = -b->y; bc.z = -b->z; bc.w = b->w;
    ok_quat_mul(a, &bc, &d);
    dv[0] = d.x; dv[1] = d.y; dv[2] = d.z;
    return 2.0 * atan2(ok_v3_norm(dv), fabs(d.w));
}
