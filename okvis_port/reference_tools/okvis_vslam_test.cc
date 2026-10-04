// OK_PORT_TEST_SRC:
// OK_PORT_TEST_C: ok_vsb_geom.c ok_eigen.c
// Random-case, tolerance-0 comparison of the pure helpers of the C module ok_vsb_geom (module M6) against the REAL
// libraries, compiled with the reference flags (-O2 -DNDEBUG -ffp-contract=off -fno-fast-math, Eigen 3.4.0, OpenCV 4.6):
//   * ok_vsb_circle_filled vs cv::circle(img, centre, radius, 255, cv::FILLED) on random image sizes, centres (also far
//     outside the image) and radii (the rasteriser of ViSlamBackend::overlapFraction / trackingQuality);
//   * ok_vsb_overlap vs a verbatim transcription of ViSlamBackend::overlapFraction (cv::Mat, cv::circle,
//     cv::bitwise_and / bitwise_or / countNonZero, std::set_intersection, std::min) on random multiframes with 1-2
//     cameras, shared landmark ids, cleared images (empty Mats: the 0/0 = NaN case) and random keypoint radii;
//   * ok_quat_to_angle_axis / ok_quat_from_angle_axis / ok_quat_angular_distance vs Eigen::AngleAxisd(Quaterniond),
//     Eigen::Quaterniond(AngleAxisd) and QuaternionBase::angularDistance (random, tiny-vector, negative-w and
//     identity quaternions).
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <random>
#include <set>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>

extern "C" {
#include "../c/ok_vslam.h"
}

static long g_tot = 0, g_bad = 0;
static long g_kind[16];
static void chk(bool ok, int kind = 0) { g_tot++; if (!ok) { g_bad++; g_kind[kind]++; } }
static bool same_bits(double a, double b) { return std::memcmp(&a, &b, 8) == 0 || (std::isnan(a) && std::isnan(b)); }

static double ref_overlap(const ok_vsb_frame_view& fa, const ok_vsb_frame_view& fb, double kptradius_,
                          const std::vector<std::vector<cv::Mat>>& images) {
  // verbatim logic of ViSlamBackend::overlapFraction (okvis_ceres/src/ViSlamBackend.cpp), frames[f]->image(im) from `images`
  const size_t numFrames = size_t(fa.ncam);
  const ok_vsb_frame_view* frames[2] = {&fa, &fb};
  std::set<uint64_t> landmarks[2];
  std::vector<cv::Mat> detectionsImg[2];
  detectionsImg[0].resize(numFrames); detectionsImg[1].resize(numFrames);
  std::vector<cv::Mat> matchesImg[2];
  matchesImg[0].resize(numFrames); matchesImg[1].resize(numFrames);
  for (size_t f = 0; f < 2; ++f) {
    for (size_t im = 0; im < size_t(frames[f]->ncam); ++im) {
      if (images[f][im].empty()) continue;
      const int rows = images[f][im].rows / 10;
      const int cols = images[f][im].cols / 10;
      const double radius = double(std::min(rows, cols)) * kptradius_;
      detectionsImg[f].at(im) = cv::Mat::zeros(rows, cols, CV_8UC1);
      matchesImg[f].at(im) = cv::Mat::zeros(rows, cols, CV_8UC1);
      const size_t num = size_t(frames[f]->cam[im].nkp);
      for (size_t k = 0; k < num; ++k) {
        cv::KeyPoint keypoint;
        keypoint.pt = cv::Point2f(frames[f]->cam[im].kp[3 * k], frames[f]->cam[im].kp[3 * k + 1]);
        cv::circle(detectionsImg[f].at(im), keypoint.pt * 0.1, int(radius), cv::Scalar(255), cv::FILLED);
        uint64_t lmId = frames[f]->cam[im].lm[k];
        if (lmId != 0) landmarks[f].insert(lmId);
      }
    }
  }
  std::set<uint64_t> matches;
  std::set_intersection(landmarks[0].begin(), landmarks[0].end(), landmarks[1].begin(), landmarks[1].end(),
                        std::inserter(matches, matches.begin()));
  if (matches.size() == 0) return 0.0;
  for (size_t f = 0; f < 2; ++f) {
    for (size_t im = 0; im < size_t(frames[f]->ncam); ++im) {
      if (images[f][im].empty()) continue;
      const size_t num = size_t(frames[f]->cam[im].nkp);
      const int rows = images[f][im].rows / 10;
      const int cols = images[f][im].cols / 10;
      const double radius = double(std::min(rows, cols)) * kptradius_;
      for (size_t k = 0; k < num; ++k) {
        cv::KeyPoint keypoint;
        keypoint.pt = cv::Point2f(frames[f]->cam[im].kp[3 * k], frames[f]->cam[im].kp[3 * k + 1]);
        if (matches.count(frames[f]->cam[im].lm[k])) {
          cv::circle(matchesImg[f].at(im), keypoint.pt * 0.1, int(radius), cv::Scalar(255), cv::FILLED);
        }
      }
    }
  }
  double overlap[2];
  for (size_t f = 0; f < 2; ++f) {
    int intersectionCount = 0;
    int unionCount = 0;
    for (size_t im = 0; im < size_t(frames[f]->ncam); ++im) {
      if (images[f][im].empty()) continue;
      cv::Mat intersectionMask, unionMask;
      cv::bitwise_and(matchesImg[f].at(im), detectionsImg[f].at(im), intersectionMask);
      cv::bitwise_or(matchesImg[f].at(im), detectionsImg[f].at(im), unionMask);
      intersectionCount += cv::countNonZero(intersectionMask);
      unionCount += cv::countNonZero(unionMask);
    }
    overlap[f] = double(intersectionCount) / double(unionCount);
  }
  return std::min(overlap[0], overlap[1]);
}

int main() {
  std::mt19937_64 rng(20261004);
  auto U = [&](double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); };
  auto I = [&](int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); };

  // ---- cv::circle ----
  long n_circle = 0;
  for (int it = 0; it < 60000; ++it) {
    const int rows = I(1, 90), cols = I(1, 120), radius = I(0, 14);
    int cx, cy;
    if (it % 4 == 0) { cx = I(-30, cols + 30); cy = I(-30, rows + 30); } else { cx = I(0, cols - 1); cy = I(0, rows - 1); }
    cv::Mat ref = cv::Mat::zeros(rows, cols, CV_8UC1);
    cv::circle(ref, cv::Point(cx, cy), radius, cv::Scalar(255), cv::FILLED);
    std::vector<unsigned char> mine(size_t(rows) * size_t(cols), 0);
    ok_vsb_circle_filled(mine.data(), rows, cols, cx, cy, radius);
    chk(std::memcmp(mine.data(), ref.data, mine.size()) == 0);
    n_circle++;
  }
  std::printf("circle                %ld cases, %ld/%ld\n", n_circle, g_bad, g_tot);

  // ---- overlapFraction ----
  const long bad0 = g_bad, tot0 = g_tot;
  long n_ov = 0, n_nan = 0, n_zero = 0;
  for (int it = 0; it < 4000; ++it) {
    const int ncam = I(1, 2);
    const double kptradius = U(0.02, 0.2);
    const int rows = I(120, 600), cols = I(160, 800);
    ok_vsb_frame_view fr[2];
    std::vector<std::vector<float>> kps[2];
    std::vector<std::vector<uint64_t>> lms[2];
    std::vector<std::vector<cv::Mat>> images(2);
    std::memset(fr, 0, sizeof fr);
    const int pool = I(5, 60);
    for (int f = 0; f < 2; ++f) {
      fr[f].alive = 1; fr[f].ncam = ncam;
      kps[f].resize(size_t(ncam)); lms[f].resize(size_t(ncam)); images[f].resize(size_t(ncam));
      for (int c = 0; c < ncam; ++c) {
        const int nk = I(0, 80);
        kps[f][size_t(c)].resize(size_t(3 * nk + 1)); lms[f][size_t(c)].resize(size_t(nk + 1));
        for (int k = 0; k < nk; ++k) {
          kps[f][size_t(c)][size_t(3 * k)] = float(U(-5, cols + 5));
          kps[f][size_t(c)][size_t(3 * k + 1)] = float(U(-5, rows + 5));
          kps[f][size_t(c)][size_t(3 * k + 2)] = float(U(1, 40));
          lms[f][size_t(c)][size_t(k)] = (I(0, 3) == 0) ? 0 : uint64_t(I(1, pool));
        }
        const bool cleared = I(0, 4) == 0;
        fr[f].cam[c].rows = rows; fr[f].cam[c].cols = cols; fr[f].cam[c].nkp = nk; fr[f].cam[c].images_cleared = cleared;
        fr[f].cam[c].kp = kps[f][size_t(c)].data(); fr[f].cam[c].lm = lms[f][size_t(c)].data();
        images[f][size_t(c)] = cleared ? cv::Mat() : cv::Mat::zeros(rows, cols, CV_8UC1);
      }
    }
    const double ref = ref_overlap(fr[0], fr[1], kptradius, images);
    const double mine = ok_vsb_overlap(&fr[0], &fr[1], kptradius);
    chk(same_bits(ref, mine));
    n_ov++; if (std::isnan(ref)) n_nan++; if (ref == 0.0) n_zero++;
  }
  std::printf("overlapFraction       %ld cases (%ld NaN, %ld zero), %ld/%ld\n", n_ov, n_nan, n_zero, g_bad - bad0, g_tot - tot0);

  // ---- quaternion helpers ----
  const long bad1 = g_bad, tot1 = g_tot;
  for (int it = 0; it < 200000; ++it) {
    Eigen::Quaterniond q, p;
    switch (it % 6) {
      case 0: q = Eigen::Quaterniond(U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1)); break;
      case 1: q = Eigen::Quaterniond(1.0, U(-1e-3, 1e-3), U(-1e-3, 1e-3), U(-1e-3, 1e-3)); break;
      case 2: q = Eigen::Quaterniond(1.0, U(-1e-17, 1e-17), U(-1e-17, 1e-17), U(-1e-17, 1e-17)); break;
      case 3: q = Eigen::Quaterniond(-U(0.1, 1), U(-1, 1), U(-1, 1), U(-1, 1)); break;
      case 4: q = Eigen::Quaterniond::Identity(); break;
      default: q = Eigen::Quaterniond(U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1)).normalized(); break;
    }
    if (it % 6 != 2 && it % 6 != 4) q.normalize();
    p = Eigen::Quaterniond(U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1)).normalized();
    ok_quat cq; cq.x = q.x(); cq.y = q.y(); cq.z = q.z(); cq.w = q.w();
    ok_quat cp; cp.x = p.x(); cp.y = p.y(); cp.z = p.z(); cp.w = p.w();
    Eigen::AngleAxisd aa(q);
    double angle, axis[3];
    ok_quat_to_angle_axis(&cq, &angle, axis);
    chk(same_bits(aa.angle(), angle), 1 + it % 6);
    chk(same_bits(aa.axis()[0], axis[0]) && same_bits(aa.axis()[1], axis[1]) && same_bits(aa.axis()[2], axis[2]), 8);
    aa.angle() = aa.angle() * (1.0 / double(I(1, 9)));
    angle = aa.angle();
    const Eigen::Quaterniond qa(aa);
    const ok_quat cqa = ok_quat_from_angle_axis(angle, axis);
    chk(same_bits(qa.x(), cqa.x) && same_bits(qa.y(), cqa.y) && same_bits(qa.z(), cqa.z) && same_bits(qa.w(), cqa.w), 9);
    chk(same_bits(q.angularDistance(p), ok_quat_angular_distance(&cq, &cp)), 10);
    chk(same_bits(p.angularDistance(q), ok_quat_angular_distance(&cp, &cq)), 11);
  }
  std::printf("quaternion helpers    200000 cases, %ld/%ld\n", g_bad - bad1, g_tot - tot1);
  for (int k = 0; k < 16; ++k) if (g_kind[k]) std::printf("  kind %d: %ld\n", k, g_kind[k]);
  std::printf("okvis_vslam_test: %ld/%ld\n", g_bad, g_tot);
  return g_bad == 0 ? 0 : 1;
}
