// Scratch debug tool (not part of the module-3 deliverable) to isolate a
// mismatch between sv_solve_homography.c and the real
// solve::homography_solver -- calls stella's own public
// solve::normalize()/homography_solver::compute_H_21() directly on a known
// 4-point minimal set from fr1_xyz attempt 0 iter 0.
#include "stella_vslam/solve/common.h"
#include "stella_vslam/solve/homography_solver.h"
#include "stella_vslam/util/converter.h"

#include <opencv2/core/types.hpp>
#include <cstdio>
#include <vector>

static void pr(const char* label, const stella_vslam::Mat33_t& m) {
    printf("%s:\n", label);
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r) {
            printf("%a%s", m(r, c), (r == 2 && c == 2) ? "\n" : ",");
        }
    }
}

int main() {
    // ref (frame 0) keypoints, all 99 matched (x,y) -- only the 4 minimal
    // set members matter for compute_H_21 but normalize() needs the FULL
    // matched set's centroid/deviation, so reproduce that too using the
    // same 99 points listed in matches.tsv for attempt 0. For this debug
    // we only need the transform matrices to be consistent with what
    // sv_solve_homography.c computes from the same 99 (x,y) pairs -- so
    // instead of retyping all 99, load them from the tsv file directly.
    FILE* mf = fopen("/tmp/att0_matches.tsv", "r");
    FILE* kf = fopen("/home/nybo/github/pose-validation/runs/stella_port/reference_dumps/fr1_xyz/keypoints.tsv", "r");
    // build keypoint lookup: frame(0/1) -> idx -> (x,y)
    std::vector<std::pair<float, float>> kp0(2000), kp1(2000);
    {
        char line[1024];
        fgets(line, sizeof(line), kf); // header
        while (fgets(line, sizeof(line), kf)) {
            int fi, ki, oct;
            double dummy;
            char xh[64], yh[64], ah[64], rh[64];
            int n = sscanf(line, "%d\t%d\t%lf\t%63s\t%lf\t%63s\t%d\t%lf\t%63s\t%lf\t%63s",
                          &fi, &ki, &dummy, xh, &dummy, yh, &oct, &dummy, ah, &dummy, rh);
            if (n != 11) continue;
            float x = strtof(xh, nullptr), y = strtof(yh, nullptr);
            if (fi == 0 && ki < 2000) kp0[ki] = {x, y};
            if (fi == 1 && ki < 2000) kp1[ki] = {x, y};
        }
    }
    std::vector<cv::KeyPoint> kpts1, kpts2;
    std::vector<std::pair<int, int>> matches;
    {
        char line[256];
        int i = 0;
        while (fgets(line, sizeof(line), mf)) {
            int aid, ri, ci;
            sscanf(line, "%d\t%d\t%d", &aid, &ri, &ci);
            kpts1.emplace_back(cv::Point2f(kp0[ri].first, kp0[ri].second), 1.0f);
            kpts2.emplace_back(cv::Point2f(kp1[ci].first, kp1[ci].second), 1.0f);
            matches.emplace_back(i, i);
            i++;
        }
    }
    printf("loaded %zu matches\n", matches.size());

    std::vector<cv::Point2f> norm1, norm2;
    stella_vslam::Mat33_t t1, t2;
    stella_vslam::solve::normalize(kpts1, norm1, t1);
    stella_vslam::solve::normalize(kpts2, norm2, t2);
    pr("transform1", t1);
    pr("transform2", t2);

    int idxs[4] = {80, 13, 82, 89};
    std::vector<cv::Point2f> ms1(4), ms2(4);
    for (int k = 0; k < 4; ++k) {
        ms1[k] = norm1[idxs[k]];
        ms2[k] = norm2[idxs[k]];
        printf("minset[%d]: (%a,%a) (%a,%a)\n", k, ms1[k].x, ms1[k].y, ms2[k].x, ms2[k].y);
    }

    stella_vslam::Mat33_t H;
    bool ok = stella_vslam::solve::homography_solver::compute_H_21(ms1, ms2, H);
    printf("compute_H_21 ok=%d\n", ok);
    pr("H_norm", H);

    stella_vslam::Mat33_t t2inv = t2.inverse();
    stella_vslam::Mat33_t H_sac = t2inv * H * t1;
    pr("H_sac", H_sac);

    // best_cost trajectory: fresh solver + find_via_ransac(K,false) for
    // K=1..100 (use_fixed_seed=true -> identical prefix draws every time).
    std::vector<cv::KeyPoint> kpts1b, kpts2b;
    for (size_t k = 0; k < kpts1.size(); ++k) { kpts1b.push_back(kpts1[k]); kpts2b.push_back(kpts2[k]); }
    for (int K = 1; K <= 100; ++K) {
        auto solver = stella_vslam::solve::homography_solver(kpts1b, kpts2b, matches, 1.0f, true);
        solver.find_via_ransac(K, false);
        printf("K=%d best_cost=%a valid=%d\n", K, solver.get_best_cost(), solver.solution_is_valid());
    }

    return 0;
}
