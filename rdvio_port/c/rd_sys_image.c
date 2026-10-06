/* SPDX-License-Identifier: Apache-2.0 */
/* RD-VIO pure-C port, module M8: OpenCvImage on rd_cv. See rd_sys_image.h. */
#include "rd_sys_image.h"
#include "rd_poisson.h"
#include <stdlib.h>
#include <string.h>

static int G_clahe_set; static double G_clip; static int G_tw, G_th;   /* static Ptr<CLAHE> s_clahe */
static int G_gftt_set; static size_t G_gftt_max;                        /* static Ptr<GFTTDetector> s_gftt */
void rd_sys_image_reset_statics(void) { G_clahe_set = 0; G_gftt_set = 0; }

static void destroy(rd_image* b) {
    rd_sys_image* im = (rd_sys_image*)b;
    if (im->have_pyr) rd_cv_free_pyramid(&im->pyr);
    free(im->image); free(im);
}
rd_sys_image* rd_sys_image_new(double t, const uint8_t* gray, int w, int h) {
    rd_sys_image* im = (rd_sys_image*)calloc(1, sizeof *im);
    im->base.refs = 1; im->base.t = t; im->base.destroy = destroy;
    im->w = w; im->h = h;
    im->image = (uint8_t*)malloc((size_t)w * (size_t)h);
    memcpy(im->image, gray, (size_t)w * (size_t)h);
    return im;
}
void rd_sys_image_preprocess(rd_sys_image* im, double clip_limit, int tiles_w, int tiles_h) {
    if (!G_clahe_set) { G_clahe_set = 1; G_clip = clip_limit; G_tw = tiles_w; G_th = tiles_h; }
    /* rd_cv_clahe is the CLAHE(6, 8x8) specialisation: the only parameters the RD-VIO configs use */
    rd_cv_clahe(im->image, im->w, im->h, im->image);
    if (im->have_pyr) { rd_cv_free_pyramid(&im->pyr); im->have_pyr = 0; }
    memset(&im->pyr, 0, sizeof im->pyr);
    im->have_pyr = rd_cv_build_pyramid(im->image, im->w, im->h, &im->pyr);
}
int rd_sys_image_detect(rd_sys_image* im, double** keypoints, size_t* n, size_t max_points, double keypoint_distance) {
    rd_cv_keypoint* kp = NULL;
    size_t nk = 0, i, kept, m = 0;
    double* cand;
    rd_poisson filter;
    if (!G_gftt_set) { G_gftt_set = 1; G_gftt_max = max_points; }
    if (!rd_cv_gftt(im->image, im->w, im->h, (int)G_gftt_max, 1, &kp, &nk)) return 0;
    if (nk == 0) { free(kp); return 1; }
    rd_cv_sort_keypoints(kp, nk);                       /* std::sort by response, descending */
    cand = (double*)malloc(sizeof(double) * 2 * nk);
    for (i = 0; i < nk; ++i) { cand[2 * i] = kp[i].pt.x; cand[2 * i + 1] = kp[i].pt.y; }
    free(kp);
    rd_poisson_init(&filter, keypoint_distance);
    for (i = 0; i < *n; ++i) rd_poisson_preset_point(&filter, *keypoints + 2 * i);
    kept = rd_poisson_insert_points(&filter, cand, nk);
    rd_poisson_free(&filter);
    for (i = 0; i < kept; ++i) {                       /* remove_if: the 20 px border */
        const double x = cand[2 * i], y = cand[2 * i + 1];
        if (x < 20 || y < 20 || x >= im->w - 20 || y >= im->h - 20) continue;
        cand[2 * m] = x; cand[2 * m + 1] = y; m++;
    }
    *keypoints = (double*)realloc(*keypoints, sizeof(double) * 2 * (*n + m + 1));
    memcpy(*keypoints + 2 * *n, cand, sizeof(double) * 2 * m);
    *n += m;
    free(cand);
    return 1;
}
int rd_sys_image_track(const rd_sys_image* im, const rd_sys_image* next_im, const double* curr, double* next, int have_next,
                       char* status, size_t n) {
    rd_cv_point *c, *nx, *rv;
    uint8_t *st, *rs;
    float* err;
    size_t i;
    memset(status, 0, n);
    if (!have_next) memset(next, 0, sizeof(double) * 2 * n);
    if (n == 0 || !next_im) return 1;
    c = (rd_cv_point*)malloc(sizeof *c * n); nx = (rd_cv_point*)malloc(sizeof *nx * n); rv = (rd_cv_point*)malloc(sizeof *rv * n);
    st = (uint8_t*)calloc(n, 1); rs = (uint8_t*)calloc(n, 1); err = (float*)calloc(n, sizeof(float));
    for (i = 0; i < n; ++i) {
        c[i].x = (float)curr[2 * i]; c[i].y = (float)curr[2 * i + 1];
        if (have_next) { nx[i].x = (float)next[2 * i]; nx[i].y = (float)next[2 * i + 1]; } else nx[i] = c[i];
        rv[i] = c[i];
    }
    rd_cv_lk(&im->pyr, &next_im->pyr, c, nx, st, err, n);           /* forward, OPTFLOW_USE_INITIAL_FLOW */
    rd_cv_lk(&next_im->pyr, &im->pyr, nx, rv, rs, err, n);          /* backward from the tracked points */
    rd_cv_track_status(c, nx, rv, st, rs, n, im->w, im->h);
    for (i = 0; i < n; ++i) {
        status[i] = (char)st[i];
        if (st[i]) { next[2 * i] = nx[i].x; next[2 * i + 1] = nx[i].y; }
    }
    free(c); free(nx); free(rv); free(st); free(rs); free(err);
    return 1;
}
void rd_sys_image_release_buffer(rd_sys_image* im) {
    free(im->image); im->image = NULL;
    if (im->have_pyr) { rd_cv_free_pyramid(&im->pyr); im->have_pyr = 0; }
}
