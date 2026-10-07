// OK_PORT_TEST_C: ok_gps_init.c ok_kin.c ok_eigen.c
// Random + realistic tolerance-0 (memcmp) comparison of okvis_port/c/ok_gps_init.{h,c} against the OKVIS2-X source:
//   umeyamaTransform, estimateRigidRansac (std::mt19937(42) + std::uniform_int_distribution<int>) and the numeric core of
//   ViGraph::checkForGpsInit (centroid-free part: RANSAC / Umeyama T_GW, 4x4 yaw Hessian of Ei^T cov^-1 Ei, Matrix4d
//   inverse, yaw sigma in degrees), compiled with the reference flags (g++ -O2 -DNDEBUG -ffp-contract=off
//   -fno-fast-math, Eigen 3.4.0 SSE2, GCC 13 libstdc++). The two free functions are COPIED VERBATIM from
//   external/gnss/OKVIS2-X/okvis_ceres/src/ViGraph.cpp (between the BEGIN/END VERBATIM markers); checkForGpsInit is
//   transcribed with its members replaced by arguments (the gathering of propagated points, the Ceres refinement and the
//   logging are not part of the numeric core). Only logging is stubbed.
// Sensitivity: deliberately wrong variants (other RNG seed, `% n` instead of the distribution, 1/n instead of /n, a
// differently associated Hessian product, a pivoted-LU inverse) are evaluated on the reference side and MUST mismatch.
// Usage: okvis_gps_init_test [cases_per_seed [seed ...]]   (default 20000 cases, seeds 1 2 3 4)
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <vector>
#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/operators.hpp>
extern "C" {
#include "../c/ok_gps_init.h"
}

// ---- stubs for the logging of the X source ----
struct NullLog { template <class T> NullLog& operator<<(const T&) { return *this; } };
#define LOG(x) NullLog()
#define DLOG(x) NullLog()

namespace okvis {
using kinematics::Transformation;  // ViGraph.cpp is inside namespace okvis and says kinematics::Transformation

// ===== BEGIN VERBATIM: external/gnss/OKVIS2-X/okvis_ceres/src/ViGraph.cpp lines 106-231 =====
kinematics::Transformation umeyamaTransform(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& gpsPoints,
                                            const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& worldPoints)
{
  // align based on SVD: source http://nghiaho.com/?page_id=671
  if(gpsPoints.size() < 3 || (gpsPoints.size() != worldPoints.size())){
    LOG(ERROR) << "Umeyama cannot be computed!";
    return kinematics::Transformation::Identity();
  }

  // compute centroids (A <-> world, B <-> gps)
  Eigen::Vector3d centroidGps, centroidWorld;
  Eigen::MatrixXd gpsPtMatrix(3, gpsPoints.size());
  Eigen::MatrixXd worldPtMatrix(3, worldPoints.size());
  centroidGps.setZero();
  centroidWorld.setZero();
  gpsPtMatrix.setZero();
  worldPtMatrix.setZero();

  for(size_t i = 0; i < gpsPoints.size() ; ++i){
    gpsPtMatrix.col(i) = gpsPoints.at(i);
    worldPtMatrix.col(i) = worldPoints.at(i);

    centroidGps += gpsPoints.at(i);
    centroidWorld += worldPoints.at(i);

  }
  centroidGps /= gpsPoints.size();
  centroidWorld /= worldPoints.size();

  // build H matrix
  gpsPtMatrix.colwise() -= centroidGps;
  worldPtMatrix.colwise() -= centroidWorld;

  Eigen::Matrix3d H;
  H = worldPtMatrix * gpsPtMatrix.transpose();

  double A = H(0,1) - H(1,0);
  double B = H(0,0) + H(1,1);
  double theta = M_PI / 2.0 - std::atan2(B,A);
  Eigen::Matrix3d R_yaw;
  R_yaw.setZero();
  R_yaw(0,0)=std::cos(theta);
  R_yaw(0,1) = - std::sin(theta);
  R_yaw(1,0) = std::sin(theta);
  R_yaw(1,1) = std::cos(theta);
  R_yaw(2,2) = 1.0;

  // translation
  Eigen::Vector3d t = centroidGps - R_yaw * centroidWorld;

  // Full transformation
  Eigen::Matrix4d T;
  T.setIdentity();
  T.topLeftCorner<3,3>() = R_yaw;
  T.topRightCorner<3,1>() = t;

  return kinematics::Transformation(T);
}


struct RigidResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double inlier_ratio;
  std::vector<int> inliers;
};

RigidResult estimateRigidRansac(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& gpsPoints,
                                const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& worldPoints,
                                int iterations = 10,
                                size_t n_points = 6,
                                double inlierThreshold = 0.5,
                                double requiredInlierRatio = 0.7)
{
  if(gpsPoints.size() < 2*n_points) return RigidResult();
  assert(gpsPoints.size() == worldPoints.size());
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(0, gpsPoints.size() - 1);

  RigidResult best;
  best.inlier_ratio = 0.0;

  for (int it = 0; it < iterations; ++it) {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gpsSubset, worldSubset;
    std::vector<int> idxs;
    while (idxs.size() < n_points) {
      int idx = dist(rng);
      if (std::find(idxs.begin(), idxs.end(), idx) == idxs.end())
        idxs.push_back(idx);
    }
    for (int i : idxs) {
      gpsSubset.push_back(gpsPoints[i]);
      worldSubset.push_back(worldPoints[i]);
    }

    // Compute Umeyama Alignment
    kinematics::Transformation T_align = umeyamaTransform(gpsSubset, worldSubset);
    Eigen::Vector3d t = T_align.r();
    Eigen::Matrix3d R_yaw = T_align.C();

    // ---- Count inliers
    std::vector<int> inliers;
    for (size_t i = 0; i < gpsPoints.size(); ++i) {
      Eigen::Vector3d est = R_yaw * worldPoints[i] + t;
      double err = (gpsPoints[i] - est).norm();
      if (err < inlierThreshold)
        inliers.push_back(i);
    }

    double ratio = static_cast<double>(inliers.size()) / gpsPoints.size();

    if (ratio > best.inlier_ratio) {
      best.inlier_ratio = ratio;
      best.R = R_yaw;
      best.t = t;
      best.inliers = inliers;
    }

    // Optional early exit
    if (ratio > requiredInlierRatio)
      break;
  }

  return best;
}
// ===== END VERBATIM =====

// Numeric core of ViGraph::checkForGpsInit (lines 1050-1111 of the same file), members replaced by arguments.
// Returns 1 on the RANSAC rejection, else 0 with yawUncertainty set.
static int checkCore(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& gpsPoints,
                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& worldPoints,
                     const std::vector<Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3>> >& covariances,
                     bool robustGpsInit, okvis::kinematics::Transformation& T_GW, double* yaw_error, double* ratio_out)
{
  if(robustGpsInit){
    // RANSAC for robust initialization
    RigidResult ransac_init_result = estimateRigidRansac(gpsPoints, worldPoints, 20, 20, 4.0, 0.7);
    *ratio_out = ransac_init_result.inlier_ratio;
    if(ransac_init_result.inlier_ratio < 0.25){
      return 1;
    }
    Eigen::Matrix4d T_align;
    T_align.setIdentity();
    T_align.topLeftCorner<3,3>() = ransac_init_result.R;
    T_align.topRightCorner<3,1>() = ransac_init_result.t;
    T_GW.set(T_align);
  }
  else {
    T_GW = umeyamaTransform(gpsPoints, worldPoints);
  }

  // compute yaw uncertainty
  Eigen::Matrix<double,4,4> Hess;
  Hess.setZero();
  // Get all measurements so far as well as corresponding propagated poses
  Eigen::Matrix<double,3,4> Ei;
  Eigen::Matrix<double,4,4> tmp2;
  for(size_t i = 0; i < worldPoints.size(); ++i){

    Eigen::Vector3d pt = worldPoints.at(i);
    Ei.setZero();
    Ei.topLeftCorner<3,3>() = -Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,3,3> tmp = okvis::kinematics::crossMx(T_GW.C()*pt);
    Ei.topRightCorner<3,1>() = tmp.col(2);
    tmp2 = Ei.transpose() * covariances.at(i).inverse() * Ei;
    Hess = Hess + tmp2;
  }

  // invert hessian
  Eigen::Matrix<double,4,4> P = Hess.inverse();
  double yawUncertainty = std::sqrt(P(3,3)) / M_PI * 180.0;
  if(yaw_error) {
    *yaw_error = yawUncertainty;
  }
  return 0;
}

// ---- deliberately wrong variants (sensitivity) ----
// mode 1: rng seed 43; 2: idx = rng() % n; (copies of estimateRigidRansac with only that line changed)
static RigidResult ransacMut(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& gpsPoints,
                             const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& worldPoints,
                             int iterations, size_t n_points, double inlierThreshold, double requiredInlierRatio, int mode)
{
  if(gpsPoints.size() < 2*n_points) return RigidResult();
  std::mt19937 rng(mode == 1 ? 43 : 42);
  std::uniform_int_distribution<int> dist(0, gpsPoints.size() - 1);
  RigidResult best;
  best.inlier_ratio = 0.0;
  for (int it = 0; it < iterations; ++it) {
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> gpsSubset, worldSubset;
    std::vector<int> idxs;
    while (idxs.size() < n_points) {
      int idx = mode == 2 ? int(rng() % gpsPoints.size()) : dist(rng);
      if (std::find(idxs.begin(), idxs.end(), idx) == idxs.end()) idxs.push_back(idx);
    }
    for (int i : idxs) { gpsSubset.push_back(gpsPoints[i]); worldSubset.push_back(worldPoints[i]); }
    kinematics::Transformation T_align = umeyamaTransform(gpsSubset, worldSubset);
    Eigen::Vector3d t = T_align.r();
    Eigen::Matrix3d R_yaw = T_align.C();
    std::vector<int> inliers;
    for (size_t i = 0; i < gpsPoints.size(); ++i) {
      Eigen::Vector3d est = R_yaw * worldPoints[i] + t;
      double err = (gpsPoints[i] - est).norm();
      if (err < inlierThreshold) inliers.push_back(i);
    }
    double ratio = static_cast<double>(inliers.size()) / gpsPoints.size();
    if (ratio > best.inlier_ratio) { best.inlier_ratio = ratio; best.R = R_yaw; best.t = t; best.inliers = inliers; }
    if (ratio > requiredInlierRatio) break;
  }
  return best;
}
// mode 3: centroids with *= 1.0/n; mode 7: H accumulated as two interleaved (even / odd k) chains; otherwise the closed form
static kinematics::Transformation umeyamaMut(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& gpsPoints,
                                             const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& worldPoints, int mode)
{
  if(gpsPoints.size() < 3 || (gpsPoints.size() != worldPoints.size())) return kinematics::Transformation::Identity();
  Eigen::Vector3d centroidGps, centroidWorld;
  Eigen::MatrixXd gpsPtMatrix(3, gpsPoints.size());
  Eigen::MatrixXd worldPtMatrix(3, worldPoints.size());
  centroidGps.setZero(); centroidWorld.setZero();
  for(size_t i = 0; i < gpsPoints.size() ; ++i){
    gpsPtMatrix.col(i) = gpsPoints.at(i); worldPtMatrix.col(i) = worldPoints.at(i);
    centroidGps += gpsPoints.at(i); centroidWorld += worldPoints.at(i);
  }
  if (mode == 3) { centroidGps *= 1.0 / gpsPoints.size(); centroidWorld *= 1.0 / worldPoints.size(); }
  else { centroidGps /= gpsPoints.size(); centroidWorld /= worldPoints.size(); }
  gpsPtMatrix.colwise() -= centroidGps; worldPtMatrix.colwise() -= centroidWorld;
  Eigen::Matrix3d H; H = worldPtMatrix * gpsPtMatrix.transpose();
  if (mode == 7) {  // wrong order: two interleaved chains (even / odd k) added at the end
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
      double c = 0.0, d = 0.0;
      for (int k = 0; k < worldPtMatrix.cols(); ++k) { if (k & 1) d += worldPtMatrix(i, k) * gpsPtMatrix(j, k); else c += worldPtMatrix(i, k) * gpsPtMatrix(j, k); }
      H(i, j) = c + d;
    }
  }
  double A = H(0,1) - H(1,0), B = H(0,0) + H(1,1);
  double theta = M_PI / 2.0 - std::atan2(B,A);
  Eigen::Matrix3d R_yaw; R_yaw.setZero();
  R_yaw(0,0)=std::cos(theta); R_yaw(0,1) = - std::sin(theta); R_yaw(1,0) = std::sin(theta); R_yaw(1,1) = std::cos(theta); R_yaw(2,2) = 1.0;
  Eigen::Vector3d t = centroidGps - R_yaw * centroidWorld;
  Eigen::Matrix4d T; T.setIdentity(); T.topLeftCorner<3,3>() = R_yaw; T.topRightCorner<3,1>() = t;
  return kinematics::Transformation(T);
}
// mode 4: Ei^T * (cov^-1 * Ei); mode 5: pivoted-LU inverse of the Hessian; mode 6: cov.inverse() through partialPivLu
static double yawMut(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& worldPoints,
                     const std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& covariances,
                     const okvis::kinematics::Transformation& T_GW, int mode)
{
  Eigen::Matrix<double,4,4> Hess; Hess.setZero();
  Eigen::Matrix<double,3,4> Ei; Eigen::Matrix<double,4,4> tmp2;
  for(size_t i = 0; i < worldPoints.size(); ++i){
    Eigen::Vector3d pt = worldPoints.at(i);
    Ei.setZero(); Ei.topLeftCorner<3,3>() = -Eigen::Matrix3d::Identity();
    Eigen::Matrix<double,3,3> tmp = okvis::kinematics::crossMx(T_GW.C()*pt);
    Ei.topRightCorner<3,1>() = tmp.col(2);
    if(mode == 4) tmp2 = Ei.transpose() * (covariances.at(i).inverse() * Ei);
    else if(mode == 6) tmp2 = Ei.transpose() * covariances.at(i).partialPivLu().inverse() * Ei;
    else tmp2 = Ei.transpose() * covariances.at(i).inverse() * Ei;
    Hess = Hess + tmp2;
  }
  Eigen::Matrix<double,4,4> P = mode == 5 ? Eigen::Matrix4d(Hess.partialPivLu().inverse()) : Eigen::Matrix4d(Hess.inverse());
  return std::sqrt(P(3,3)) / M_PI * 180.0;
}
}  // namespace okvis

using namespace okvis;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> V3s;
typedef std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> M3s;

struct Sec { std::string name; long bad = 0, tot = 0; long bytes = 0; };
static std::vector<Sec> g_secs;
static Sec& sec(const char* n) {
  for (auto& s : g_secs) if (s.name == n) return s;
  g_secs.push_back(Sec{std::string(n)}); return g_secs.back();
}
static int cmpb(const char* n, const void* a, const void* b, size_t bytes) {
  Sec& s = sec(n); s.tot++; s.bytes += (long)bytes;
  if (std::memcmp(a, b, bytes)) { s.bad++; return 1; }
  return 0;
}
static int cmpi(const char* n, long a, long b) { Sec& s = sec(n); s.tot++; s.bytes += 8; if (a != b) { s.bad++; return 1; } return 0; }

static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N(double s = 1.0) { return std::normal_distribution<double>(0.0, s)(rng); }
static int RI(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

struct Case { V3s gps, world; M3s cov; int kind; };

static Eigen::Matrix3d randCov(double sigma, int style) {
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  if (style == 0) { C.diagonal() << sigma * sigma, sigma * sigma, sigma * sigma * U(1.0, 9.0); return C; }
  if (style == 1) { C.diagonal() << sigma * sigma * U(0.5, 3.0), sigma * sigma * U(0.5, 3.0), sigma * sigma * U(1.0, 9.0); return C; }
  Eigen::Matrix3d A; for (int i = 0; i < 9; ++i) A(i) = N(sigma);
  C = A * A.transpose(); C.diagonal().array() += sigma * sigma * 0.1;
  return C;
}

static Case genCase() {
  Case c; c.kind = RI(0, 7);
  int n;
  switch (c.kind) {
    case 3: n = RI(2, 16); break;
    case 5: n = RI(40, 300); break;
    case 6: n = RI(2, 60); break;
    default: n = RI(2, 300); break;
  }
  const double yaw = U(-M_PI, M_PI);
  const Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d tr(U(-100, 100), U(-100, 100), U(-30, 30));
  const double sigma = U(0.01, 0.05);
  const int style = RI(0, 2);
  Eigen::Vector3d p(U(-3, 3), U(-3, 3), U(0, 2)), v(N(0.5), N(0.5), N(0.1));
  const Eigen::Vector3d dir = Eigen::Vector3d(N(), N(), N() * 0.05).normalized();
  const double outlierFrac = (c.kind == 5) ? U(0.0, 0.7) : (RI(0, 3) == 0 ? U(0.0, 0.3) : 0.0);
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d w;
    if (c.kind == 0) w = Eigen::Vector3d(U(-10, 10), U(-10, 10), U(-5, 5));
    else if (c.kind == 2) w = p + dir * (0.2 * i) ;            // straight line (near-degenerate yaw), EuRoC-like step
    else if (c.kind == 4) w = (RI(0, 3) == 0 && i > 0) ? c.world[RI(0, i - 1)] : Eigen::Vector3d(N(1e3), N(1e3), N(1e3));
    else { v += Eigen::Vector3d(N(0.05), N(0.05), N(0.01)); p += v * 0.1; w = p; }   // smooth trajectory
    c.world.push_back(w);
    double s = sigma;
    Eigen::Vector3d g = Rz * w + tr + Eigen::Vector3d(N(s), N(s), N(s * 2.0));
    if (c.kind == 7) g = Eigen::Vector3d(U(-50, 50), U(-50, 50), U(-50, 50));  // garbage correspondences
    if (U(0, 1) < outlierFrac) g += Eigen::Vector3d(N(8.0), N(8.0), N(8.0));
    c.gps.push_back(g);
    if (c.kind == 4 && RI(0, 40) == 0) c.cov.push_back(Eigen::Matrix3d::Zero());           // singular covariance (inf/nan)
    else c.cov.push_back(randCov(s, style));
  }
  if (c.kind == 2 && RI(0, 1) == 0) for (auto& w : c.world) w = Eigen::Vector3d(w.x(), w.y(), 0.0) + Eigen::Vector3d(0, 0, 1.0);
  return c;
}

static void flat(const V3s& a, std::vector<double>& o) { o.clear(); for (auto& x : a) { o.push_back(x.x()); o.push_back(x.y()); o.push_back(x.z()); } }
static void flatc(const M3s& a, std::vector<double>& o) { o.clear(); for (auto& m : a) for (int k = 0; k < 9; ++k) o.push_back(m.data()[k]); }

static long tfDiffers(const kinematics::Transformation& T, const ok_tf& c) {
  const Eigen::Vector3d r = T.r(); const Eigen::Matrix3d C = T.C(); const Eigen::Quaterniond q = T.q();
  double qq[4] = {c.q.x, c.q.y, c.q.z, c.q.w};
  return std::memcmp(r.data(), c.r, 24) || std::memcmp(C.data(), c.C, 72) || std::memcmp(q.coeffs().data(), qq, 32);
}
static long cmpTf(const char* nm, const kinematics::Transformation& T, const ok_tf& c) {
  const Eigen::Vector3d r = T.r(); const Eigen::Matrix3d C = T.C(); const Eigen::Quaterniond q = T.q();
  double qq[4] = {c.q.x, c.q.y, c.q.z, c.q.w};
  int bad = cmpb(nm, r.data(), c.r, 24);
  bad += cmpb(nm, C.data(), c.C, 72);
  bad += cmpb(nm, q.coeffs().data(), qq, 32);
  return bad;
}

int main(int argc, char** argv) {
  long cases = argc > 1 ? atol(argv[1]) : 20000;
  std::vector<unsigned> seeds;
  for (int i = 2; i < argc; ++i) seeds.push_back((unsigned)atol(argv[i]));
  if (seeds.empty()) seeds = {1, 2, 3, 4};
  long mutCompared[8] = {0}, mutMismatch[8] = {0};
  for (unsigned seed : seeds) {
    rng.seed(seed * 7919u + 20261006u);
    // ---- building blocks ----
    {
      ok_mt19937 g; std::mt19937 r(seed + 100);
      ok_mt19937_seed(&g, seed + 100);
      for (int i = 0; i < 200000; ++i) { unsigned a = r(), b = ok_mt19937_next(&g); cmpb("mt19937 (1.6 MB)", &a, &b, 4); }
      for (int i = 0; i < 20000; ++i) {
        int lo = RI(0, 2) == 0 ? RI(-1000, 1000) : 0;
        int hi = lo + (RI(0, 3) == 0 ? RI(0, 2000000000) : RI(0, 400));
        if (RI(0, 50) == 0) hi = lo;
        std::uniform_int_distribution<int> d(lo, hi);
        int a = d(r), b = ok_uniform_int(&g, lo, hi);
        cmpb("uniform_int_distribution<int>", &a, &b, 4);
        // both generators advanced identically?
        unsigned x = r(), y = ok_mt19937_next(&g); cmpb("mt19937 after dist", &x, &y, 4);
      }
      std::uniform_int_distribution<int> d0(0, 0);
      int a = d0(r), b = ok_uniform_int(&g, 0, 0); cmpb("uniform_int_distribution<int>", &a, &b, 4);
    }
    for (int i = 0; i < 20000; ++i) {
      Eigen::Matrix3d m; for (int k = 0; k < 9; ++k) m(k) = N(RI(0, 1) ? 1.0 : 1e-3);
      if (RI(0, 20) == 0) m = Eigen::Matrix3d::Zero();
      Eigen::Matrix3d inv = m.inverse(); double o[9]; ok_gps_inverse3(m.data(), o);
      cmpb("Matrix3d::inverse", inv.data(), o, 72);
      Eigen::Matrix4d M; for (int k = 0; k < 16; ++k) M(k) = N(RI(0, 1) ? 1.0 : 1e3);
      if (RI(0, 3) == 0) { M = M * M.transpose(); }
      Eigen::Matrix4d inv4 = M.inverse(); double o4[16]; ok_gps_inverse4(M.data(), o4);
      cmpb("Matrix4d::inverse", inv4.data(), o4, 128);
    }
    // ---- the three functions on generated scenarios ----
    for (long ci = 0; ci < cases; ++ci) {
      Case c = genCase();
      const int n = (int)c.gps.size();
      std::vector<double> g, w, cv; flat(c.gps, g); flat(c.world, w); flatc(c.cov, cv);
      // umeyamaTransform
      { kinematics::Transformation T = umeyamaTransform(c.gps, c.world); ok_tf t; ok_gps_umeyama(n, g.data(), w.data(), &t); cmpTf("umeyamaTransform", T, t);
        if (ci < 3000) { for (int mode : {3, 7}) { kinematics::Transformation M = umeyamaMut(c.gps, c.world, mode); mutCompared[mode]++; mutMismatch[mode] += tfDiffers(M, t) ? 1 : 0; } } }
      // estimateRigidRansac: the default arguments, the call of checkForGpsInit, random parameters
      for (int pv = 0; pv < 3; ++pv) {
        int iters = 10; size_t np = 6; double thr = 0.5, req = 0.7;
        if (pv == 1) { iters = 20; np = 20; thr = 4.0; req = 0.7; }
        if (pv == 2) { iters = RI(1, 30); np = RI(2, 20); thr = U(0.01, 5.0); req = U(0.2, 1.0); }
        RigidResult rr = estimateRigidRansac(c.gps, c.world, iters, np, thr, req);
        ok_rigid_result cr; std::vector<int> ci2(n + 1);
        ok_gps_estimate_rigid_ransac(n, g.data(), w.data(), iters, (int)np, thr, req, &cr, ci2.data());
        // R and t of a `best` that never improved (ratio 0 after the loop) are UNINITIALISED in C++ (never read: the caller
        // rejects ratio < 0.25); the early return for too few points is a value-initialised (zero) RigidResult.
        if (rr.inlier_ratio != 0.0 || n < 2 * (int)np) { cmpb("ransac R,t", rr.R.data(), cr.R, 72); cmpb("ransac R,t", rr.t.data(), cr.t, 24); }
        cmpb("ransac ratio", &rr.inlier_ratio, &cr.inlier_ratio, 8);
        cmpi("ransac inlier count", (long)rr.inliers.size(), cr.n_inliers);
        if (!rr.inliers.empty()) cmpb("ransac inlier indices", rr.inliers.data(), ci2.data(), sizeof(int) * rr.inliers.size());
        if (pv == 1 && ci < 4000 && n >= 40) {
          for (int mode = 1; mode <= 2; ++mode) {
            RigidResult m = ransacMut(c.gps, c.world, iters, np, thr, req, mode);
            mutCompared[mode]++;
            if (std::memcmp(m.R.data(), cr.R, 72) || std::memcmp(m.t.data(), cr.t, 24) || m.inliers.size() != (size_t)cr.n_inliers) mutMismatch[mode]++;
          }
        }
      }
      // checkForGpsInit core, both modes
      for (int robust = 0; robust < 2; ++robust) {
        kinematics::Transformation T; double yaw = -1, ratio = -1;
        const int st = checkCore(c.gps, c.world, c.cov, robust != 0, T, &yaw, &ratio);
        ok_tf t; ok_tf_identity(&t); double cyaw = -1, cratio = -1;
        const int cst = ok_gps_init_core(n, g.data(), w.data(), cv.data(), robust, &t, &cyaw, &cratio);
        cmpi("checkForGpsInit status", st, cst);
        if (robust) cmpb("checkForGpsInit ransac ratio", &ratio, &cratio, 8);
        if (st == 0 && cst == 0) {
          cmpTf("checkForGpsInit T_GW", T, t);
          cmpb("checkForGpsInit yaw sigma [deg]", &yaw, &cyaw, 8);
          // Hessian alone
          Eigen::Matrix4d Hs; { Eigen::Matrix<double,3,4> Ei; Eigen::Matrix4d tmp2; Hs.setZero();
            for (size_t i = 0; i < c.world.size(); ++i) { Eigen::Vector3d pt = c.world[i]; Ei.setZero(); Ei.topLeftCorner<3,3>() = -Eigen::Matrix3d::Identity();
              Eigen::Matrix3d tmp = okvis::kinematics::crossMx(T.C()*pt); Ei.topRightCorner<3,1>() = tmp.col(2);
              tmp2 = Ei.transpose() * c.cov[i].inverse() * Ei; Hs = Hs + tmp2; } }
          double Hc[16]; ok_gps_yaw_hessian(n, w.data(), cv.data(), T.C().data(), Hc);
          Eigen::Matrix3d Cm = T.C(); ok_gps_yaw_hessian(n, w.data(), cv.data(), Cm.data(), Hc);
          cmpb("yaw Hessian sum", Hs.data(), Hc, 128);
          if (robust == 0 && ci < 4000) {
            for (int mode = 4; mode <= 6; ++mode) { double m = yawMut(c.world, c.cov, T, mode); mutCompared[mode]++; if (std::memcmp(&m, &cyaw, 8)) mutMismatch[mode]++; }
          }
        }
      }
    }
  }
  int fails = 0;
  long tb = 0;
  for (auto& s : g_secs) {
    std::printf("%-34s %9ld bytes %9ld cases mismatches %ld\n", s.name.c_str(), s.bytes, s.tot, s.bad);
    tb += s.bytes; if (s.bad) fails++;
  }
  const char* names[8] = {"", "ransac seed 43", "ransac rng()%n", "centroid *= 1/n", "Ei^T*(cov^-1*Ei)", "Hessian pivoted-LU inverse", "cov pivoted-LU inverse", "H as even/odd chains"};
  for (int m = 1; m <= 7; ++m)
    if (mutCompared[m]) { std::printf("sensitivity %-28s %ld / %ld mismatch (must be > 0)\n", names[m], mutMismatch[m], mutCompared[m]); if (!mutMismatch[m]) fails++; }
  std::printf("okvis_gps_init_test: %ld bytes compared, %d failing section(s)\n", tb, fails);
  return fails ? 1 : 0;
}
