/* SPDX-License-Identifier: BSD-2-Clause */
/* See sv_solve_common.h (BSD-2, AIST 2019 + stella-cv 2022). */
#include "sv_solve_common.h"
#include <math.h>

void sv_solve_normalize(const float* pts_x, const float* pts_y, int n,
                         float* norm_x, float* norm_y, double transform[9]) {
    int i;
    float sumx = 0.0f, sumy = 0.0f;
    float mean_x, mean_y;
    float l1x = 0.0f, l1y = 0.0f;
    double mean_x_d, mean_y_d, l1x_d, l1y_d;

    /* std::accumulate, cv::Point2f addition (float), sequential. */
    for (i = 0; i < n; ++i) {
        sumx = sumx + pts_x[i];
        sumy = sumy + pts_y[i];
    }
    /* cv::Point2f / double(num_keypts): saturate_cast<float>(x/(double)n). */
    mean_x = (float)((double)sumx / (double)n);
    mean_y = (float)((double)sumy / (double)n);

    for (i = 0; i < n; ++i) {
        float nx = pts_x[i] - mean_x;
        float ny = pts_y[i] - mean_y;
        norm_x[i] = nx;
        norm_y[i] = ny;
        l1x = l1x + fabsf(nx);
        l1y = l1y + fabsf(ny);
    }
    l1x = (float)((double)l1x / (double)n);
    l1y = (float)((double)l1y / (double)n);

    for (i = 0; i < n; ++i) {
        norm_x[i] = norm_x[i] / l1x;
        norm_y[i] = norm_y[i] / l1y;
    }

    mean_x_d = (double)mean_x;
    mean_y_d = (double)mean_y;
    l1x_d = (double)l1x;
    l1y_d = (double)l1y;

    /* column-major 3x3: transform[col*3+row] */
#define T(r, c) transform[(c) * 3 + (r)]
    T(0, 0) = 1.0 / l1x_d;
    T(0, 1) = 0.0 / l1x_d;
    T(0, 2) = (-mean_x_d) / l1x_d;
    T(1, 0) = 0.0 / l1y_d;
    T(1, 1) = 1.0 / l1y_d;
    T(1, 2) = (-mean_y_d) / l1y_d;
    T(2, 0) = 0.0;
    T(2, 1) = 0.0;
    T(2, 2) = 1.0;
#undef T
}
