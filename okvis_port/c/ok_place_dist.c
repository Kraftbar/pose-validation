/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* OKVIS2 pure-C port, module 7d (part 2): the descriptor distinctiveness statistic of verifyRecognisedPlace. See ok_place.h. */
#include "ok_place.h"
#include <math.h>
/* The distinctiveness statistic of one camera (Eigen::Matrix<float, Dynamic, 384>, float arithmetic):
 *   stdev = ((M.rowwise() - M.colwise().mean()).colwise().squaredNorm() / (rows - 1)).cwiseSqrt();   return n * stdev.sum() */
float ok_place_distinctiveness(const unsigned char* desc, int n) {
    float mean[384], sq[384], stdev[384], sum;
    int i, j, b, c;
    for (j = 0; j < 384; ++j) {
        float s = 0.0f;
        b = j / 8; c = j % 8;
        for (i = 0; i < n; ++i) s = s + (((desc[48 * i + b] & (1 << c)) != 0) ? 1.0f : 0.0f);
        mean[j] = s / (float)n;
    }
    for (j = 0; j < 384; ++j) {
        float s = 0.0f;
        b = j / 8; c = j % 8;
        for (i = 0; i < n; ++i) {
            const float x = ((desc[48 * i + b] & (1 << c)) != 0) ? 1.0f : 0.0f;
            const float d = x - mean[j];
            s = (i == 0) ? d * d : s + d * d;
        }
        sq[j] = s;
    }
    for (j = 0; j < 384; ++j) stdev[j] = sqrtf(sq[j] / (float)(n - 1));
    {   /* stdev.sum(): Matrix<float,1,384>, linear vectorised redux, two 4-lane accumulators, then the horizontal add */
        float p0[4], p1[4];
        int idx, l;
        for (l = 0; l < 4; ++l) { p0[l] = stdev[l]; p1[l] = stdev[4 + l]; }
        for (idx = 8; idx < 384; idx += 8)
            for (l = 0; l < 4; ++l) { p0[l] = p0[l] + stdev[idx + l]; p1[l] = p1[l] + stdev[idx + 4 + l]; }
        for (l = 0; l < 4; ++l) p0[l] = p0[l] + p1[l];
        sum = (p0[0] + p0[2]) + (p0[1] + p0[3]);
    }
    return (float)n * sum;
}
