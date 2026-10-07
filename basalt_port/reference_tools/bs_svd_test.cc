// Basalt port M9 oracle: Eigen 3.4.0 JacobiSVD<Matrix4f, ComputeFullV>, BundleAdjustmentBase<float>::triangulate and
// StereographicParam<float>::project (real classes, reference flags) vs basalt_port/c/bs_svd.c.  Tolerance 0 (memcmp of every output).
//   usage: bs_svd_test <seed> <cases_per_class>
#include <basalt/vi_estimator/ba_base.h>
#include <basalt/camera/stereographic_param.hpp>
#include <Eigen/SVD>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <random>
#include <string>

extern "C" {
#include "bs_svd.h"
}

using namespace Eigen;
static std::mt19937_64 g_rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(g_rng); }
static int I(int n) { return (int)(g_rng() % (uint64_t)n); }
struct Stat { long total = 0, bad = 0; };
static std::map<std::string, Stat> g_stats;
static std::map<std::string, long> g_cov;
static void chk(const std::string& name, const void* a, const void* b, size_t n, long cs) {
  Stat& s = g_stats[name];
  s.total++;
  if (memcmp(a, b, n)) { if (!s.bad) std::printf("first mismatch %s case %ld\n", name.c_str(), cs); s.bad++; }
}

static Matrix4f gen_matrix(int cls) {
  Matrix4f A;
  std::normal_distribution<double> N(0, 1);
  switch (cls) {
    case 0: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (float)N(g_rng); break;                           // gaussian
    case 1: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (float)(I(7) - 3); break;                          // small integers (ties, exact zeros)
    case 2: {                                                                                                  // rank deficient
      Vector4f u, v; for (int i = 0; i < 4; ++i) { u[i] = (float)N(g_rng); v[i] = (float)N(g_rng); }
      A = u * v.transpose(); if (I(2)) { Vector4f w, z; for (int i = 0; i < 4; ++i) { w[i] = (float)N(g_rng); z[i] = (float)N(g_rng); } A += w * z.transpose(); } break; }
    case 3: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (I(3) ? 0.f : (float)N(g_rng)); break;             // sparse
    case 4: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (float)(N(g_rng) * std::pow(10.0, U(-30, 30))); break; // dynamic range
    case 5: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (float)(N(g_rng) * 1e-38 * U(0.001, 1)); break;      // denormal range
    case 6: { A.setZero(); int k = I(4); A(k, I(4)) = (float)N(g_rng); if (I(2)) A(I(4), I(4)) = (float)N(g_rng); break; }
    case 7: { A.setIdentity(); for (int i = 0; i < 4; ++i) A(i, i) = (float)(I(3) ? 1 : N(g_rng)); A(I(4), I(4)) += (float)(N(g_rng) * 1e-8); break; }  // nearly diagonal
    case 8: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (float)(N(g_rng) * 1e20); break;
    default: for (int i = 0; i < 16; ++i) A(i % 4, i / 4) = (float)U(-1, 1); break;
  }
  return A;
}

static void cmp_svd(const std::string& tag, const Matrix4f& A, long cs) {
  JacobiSVD<Matrix4f> svd(A, ComputeFullV);
  float V[16], sv[4];
  int info = bs_jacobisvd4f(A.data(), V, sv);
  bool finite = A.allFinite();
  ++g_cov[tag + (finite ? " finite" : " non-finite")];
  if (!finite) return;   // the C++ leaves matrixV() uninitialised
  (void)info;
  chk(tag + ": V", svd.matrixV().data(), V, 64, cs);
  chk(tag + ": singular values", svd.singularValues().data(), sv, 16, cs);
}

static Matrix<float, 3, 1> unit3(const Vector3f& v) { return v / v.norm(); }

int main(int argc, char** argv) {
  const int seed = argc > 1 ? std::atoi(argv[1]) : 1;
  const int cases = argc > 2 ? std::atoi(argv[2]) : 2000;
  g_rng.seed(seed);
  for (int cls = 0; cls < 10; ++cls)
    for (int k = 0; k < cases; ++k) cmp_svd("random class " + std::to_string(cls), gen_matrix(cls), k);
  // non-finite inputs: the C returns InvalidInput (V undefined in C++)
  { Matrix4f A = gen_matrix(0); A(1, 2) = std::numeric_limits<float>::infinity(); cmp_svd("random class inf", A, 0);
    A(1, 2) = std::numeric_limits<float>::quiet_NaN(); cmp_svd("random class nan", A, 0); }

  // triangulation: realistic geometry (stereo baseline 0.11 m, forward / lateral motion, depth 0.3 .. 80 m, noisy bearings), plus degenerate ones
  std::normal_distribution<double> N(0, 1);
  const long tri_cases = (long)cases * 6;
  for (long k = 0; k < tri_cases; ++k) {
    const int kind = (int)(k % 6);
    Vector3f w(0, 0, 0), t(0, 0, 0);
    if (kind == 0) { t = Vector3f(0.11f, (float)N(g_rng) * 0.002f, (float)N(g_rng) * 0.002f); w = Vector3f((float)N(g_rng) * 0.01f, (float)N(g_rng) * 0.01f, (float)N(g_rng) * 0.01f); }  // stereo
    else if (kind == 1) { t = Vector3f((float)U(-1, 1), (float)U(-1, 1), (float)U(-1, 1)) * (float)U(0.05, 2.0); w = Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)) * 0.2f; }  // temporal
    else if (kind == 2) { t = Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)) * (float)U(1e-4, 0.1); w = Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)) * 0.01f; }  // tiny baseline
    else if (kind == 3) { t = Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)) * 5.f; w = Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)); }  // wide
    else if (kind == 4) { t = Vector3f(0, 0, (float)U(0.05, 1)); w = Vector3f(0, 0, 0); }   // pure forward (on the epipole: degenerate rows)
    else { t = Vector3f((float)U(-1, 1), 0, 0); w = Vector3f(0, (float)U(-0.1, 0.1), 0); }
    Sophus::SE3f T_0_1(Sophus::SO3f::exp(w), t);
    const Vector3f P((float)U(-20, 20) * 0.5f, (float)U(-10, 10) * 0.5f, (float)U(0.3, 80));          // point in frame 0
    Vector3f p1 = T_0_1.inverse() * P;
    Vector4f f0h, f1h; f0h.setZero(); f1h.setZero();
    Vector3f a = unit3(P), b = unit3(p1);
    if (k % 7 == 3) { a += Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)) * 1e-3f; a /= a.norm(); b += Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)) * 1e-3f; b /= b.norm(); }
    if (k % 11 == 5) { a = Vector3f((float)N(g_rng), (float)N(g_rng), (float)N(g_rng)); a /= a.norm(); }                              // inconsistent rays
    f0h.head<3>() = a; f1h.head<3>() = b;
    Vector4f ref = basalt::BundleAdjustmentBase<float>::triangulate(f0h.head<3>(), f1h.head<3>(), T_0_1);
    bs_se3f cT; cT.so3 = {T_0_1.so3().data()[0], T_0_1.so3().data()[1], T_0_1.so3().data()[2], T_0_1.so3().data()[3]};
    for (int i = 0; i < 3; ++i) cT.t[i] = T_0_1.translation()[i];
    float out[4];
    bs_triangulate_f(f0h.data(), f1h.data(), &cT, out);
    chk("triangulate kind " + std::to_string(kind), ref.data(), out, 16, k);
    ++g_cov["triangulate finite"]; if (!ref.array().isFinite().all()) { --g_cov["triangulate finite"]; ++g_cov["triangulate non-finite"]; }
    if (ref[3] > 0 && ref[3] < 3.0) ++g_cov["triangulate accepted (0 < inv_dist < 3)"];
    Vector2f sp = basalt::StereographicParam<float>::project(ref);
    float spc[2]; bs_stereographic_project_f(ref.data(), spc);
    chk("StereographicParam::project", sp.data(), spc, 8, k);
    // also the SVD matrix of this problem
    Matrix<float, 3, 4> P1, P2; P1.setIdentity(); P2 = T_0_1.inverse().matrix3x4();
    Matrix4f A; A.row(0) = f0h[0] * P1.row(2) - f0h[2] * P1.row(0); A.row(1) = f0h[1] * P1.row(2) - f0h[2] * P1.row(1);
    A.row(2) = f1h[0] * P2.row(2) - f1h[2] * P2.row(0); A.row(3) = f1h[1] * P2.row(2) - f1h[2] * P2.row(1);
    cmp_svd("triangulation matrix", A, k);
  }
  long total = 0, bad = 0;
  for (auto& kv : g_stats) { std::printf("%-44s %8ld comparisons %6ld mismatches\n", kv.first.c_str(), kv.second.total, kv.second.bad); total += kv.second.total; bad += kv.second.bad; }
  for (auto& kv : g_cov) std::printf("coverage: %-40s %ld\n", kv.first.c_str(), kv.second);
  std::printf("bs_svd: %ld/%ld\n", bad, total);
  return bad ? 1 : 0;
}
