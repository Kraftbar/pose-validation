// Oracle for basalt_port/c/bs_cam.{h,c}: the real basalt-headers DoubleSphereCamera<float/double> vs the C port, memcmp (tolerance 0).
// Build (reference flags): tools/check_basalt_port.py --modules m2   (g++ -std=c++17 -O2 -DNDEBUG -ffp-contract=off -fno-fast-math + gcc -c99 port)
// Usage: bs_cam_test <seed> <cases> [sens]
#include <basalt/camera/double_sphere_camera.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

extern "C" {
#include "bs_cam.h"
}

typedef std::mt19937_64 Rng;
static double urand(Rng& r, double a, double b) { return a + (b - a) * std::uniform_real_distribution<double>(0, 1)(r); }

static long g_total = 0, g_bad = 0, g_valid = 0, g_invalid = 0, g_nonfinite = 0;
static int g_verbose = 0;

// NaN *inputs* propagate the payload/sign of whichever operand the compiler happened to put first (not a model property):
// for those cases (g_nan_input) a NaN matches any NaN; every other case, including inf-inf = default NaN, is memcmp.
static bool g_nan_input = false;
static long g_nan_cases = 0;
template <class T> static bool same(const T* a, const T* b, int n) {
  if (!g_nan_input) return std::memcmp(a, b, sizeof(T) * n) == 0;
  for (int i = 0; i < n; ++i)
    if (std::memcmp(a + i, b + i, sizeof(T)) != 0 && !(std::isnan(a[i]) && std::isnan(b[i]))) return false;
  return true;
}

template <class T> struct Types;
template <> struct Types<float> { typedef bs_ds_f C; };
template <> struct Types<double> { typedef bs_ds_d C; };
static int c_project(const bs_ds_f& c, const float* p, float* o, float* j3, float* jp) { return bs_ds_project_f(&c, p, o, j3, jp); }
static int c_project(const bs_ds_d& c, const double* p, double* o, double* j3, double* jp) { return bs_ds_project_d(&c, p, o, j3, jp); }
static int c_unproject(const bs_ds_f& c, const float* p, float* o, float* j, float* jp) { return bs_ds_unproject_f(&c, p, o, j, jp); }
static int c_unproject(const bs_ds_d& c, const double* p, double* o, double* j, double* jp) { return bs_ds_unproject_d(&c, p, o, j, jp); }

// one case: check project (Vec4 argument, with and without Jacobians) and unproject (Vec4 result, with / without Jacobians)
template <class T>
static void run_case(const T par[6], const T p3[3], const T uv[2], int sens) {
  typedef Eigen::Matrix<T, 4, 1> Vec4;
  typedef Eigen::Matrix<T, 2, 1> Vec2;
  g_nan_input = std::isnan((double)p3[0]) || std::isnan((double)p3[1]) || std::isnan((double)p3[2]) || std::isnan((double)uv[0]) || std::isnan((double)uv[1]);
  g_nan_cases += g_nan_input;
  typename Types<T>::C cc;
  for (int i = 0; i < 6; ++i) cc.p[i] = par[i];
  if (sens) cc.p[0] = std::nextafter(cc.p[0], T(1e9));
  typename basalt::DoubleSphereCamera<T>::VecN pv;
  for (int i = 0; i < 6; ++i) pv[i] = par[i];
  basalt::DoubleSphereCamera<T> cam(pv);

  // ---- project, all three call shapes the code uses: (p, res), (p, res, &Jp), (p, res, &Jp, &Jparam)
  {
    Vec4 p;
    p << p3[0], p3[1], p3[2], T(1) / T(3);
    Vec2 r0, r1, r2;
    Eigen::Matrix<T, 2, 4> J1, J2;
    Eigen::Matrix<T, 2, 6> Jpar;
    bool v0 = cam.project(p, r0);
    bool v1 = cam.project(p, r1, &J1);
    bool v2 = cam.project(p, r2, &J2, &Jpar);
    T cr[2], cj[8], cj2[8], cjp[12], cr1[2], cr2[2];
    int cv0 = c_project(cc, p3, cr, nullptr, nullptr);
    int cv1 = c_project(cc, p3, cr1, cj, nullptr);
    int cv2 = c_project(cc, p3, cr2, cj2, cjp);
    bool ok = (int)v0 == cv0 && (int)v1 == cv1 && (int)v2 == cv2 && same(r0.data(), cr, 2) && same(r1.data(), cr1, 2) &&
              same(r2.data(), cr2, 2) && same(J1.data(), cj, 8) && same(J2.data(), cj2, 8) && same(Jpar.data(), cjp, 12);
    ++g_total;
    if (!ok) { ++g_bad; if (g_verbose && g_bad < 5) std::printf("project mismatch p=(%.9g %.9g %.9g)\n", (double)p3[0], (double)p3[1], (double)p3[2]); }
    (v0 ? g_valid : g_invalid)++;
    if (!std::isfinite((double)r0[0]) || !std::isfinite((double)r0[1])) ++g_nonfinite;
  }
  // ---- unproject: (p, res), (p, res, &J), (p, res, &J, &Jparam) and the param-only shape (p, res, nullptr, &Jparam)
  {
    Vec2 u;
    u << uv[0], uv[1];
    Vec4 q0, q1, q2, q3;
    Eigen::Matrix<T, 4, 2> J1, J2;
    Eigen::Matrix<T, 4, 6> Jp2, Jp3;
    q0.setConstant(T(7)); q1 = q0; q2 = q0; q3 = q0;
    bool v0 = cam.unproject(u, q0);
    bool v1 = cam.unproject(u, q1, &J1);
    bool v2 = cam.unproject(u, q2, &J2, &Jp2);
    bool v3 = cam.unproject(u, q3, (std::nullptr_t) nullptr, &Jp3);
    T cq0[4], cq1[4], cq2[4], cq3[4], cj1[8], cj2[8], cjp2[24], cjp3[24];
    int cv0 = c_unproject(cc, uv, cq0, nullptr, nullptr);
    int cv1 = c_unproject(cc, uv, cq1, cj1, nullptr);
    int cv2 = c_unproject(cc, uv, cq2, cj2, cjp2);
    int cv3 = c_unproject(cc, uv, cq3, nullptr, cjp3);
    bool ok = (int)v0 == cv0 && (int)v1 == cv1 && (int)v2 == cv2 && (int)v3 == cv3 && same(q0.data(), cq0, 4) && same(q1.data(), cq1, 4) &&
              same(q2.data(), cq2, 4) && same(q3.data(), cq3, 4) && same(J1.data(), cj1, 8) && same(J2.data(), cj2, 8) &&
              same(Jp2.data(), cjp2, 24) && same(Jp3.data(), cjp3, 24);
    ++g_total;
    if (!ok) { ++g_bad; if (g_verbose && g_bad < 5) std::printf("unproject mismatch uv=(%.9g %.9g)\n", (double)uv[0], (double)uv[1]); }
    (v0 ? g_valid : g_invalid)++;
    if (!std::isfinite((double)q0[0]) || !std::isfinite((double)q0[2])) ++g_nonfinite;
  }
}

template <class T> static T jitter(Rng& r, T v, int ulps) {
  int n = (int)(r() % (2 * ulps + 1)) - ulps;
  for (int i = 0; i < std::abs(n); ++i) v = std::nextafter(v, n > 0 ? std::numeric_limits<T>::infinity() : -std::numeric_limits<T>::infinity());
  return v;
}

template <class T> static void gen_params(Rng& r, int kind, T par[6]) {
  // kind 0: EuRoC cam0, 1: EuRoC cam1 (cast of the double calibration), 2: random plausible, 3: random wide (incl. alpha <= 0.5)
  static const double e0[6] = {349.7560023050409, 348.72454229977037, 365.89440762590149, 249.32995565708704, -0.2409573942178872, 0.566996899163044};
  static const double e1[6] = {361.6713883800533, 360.5856493689301, 379.40818394080869, 255.9772968522045, -0.21300835384809328, 0.5767008625037023};
  if (kind == 0 || kind == 1) {
    for (int i = 0; i < 6; ++i) par[i] = (T)(kind == 0 ? e0[i] : e1[i]);
  } else if (kind == 2) {
    par[0] = (T)urand(r, 250, 450); par[1] = (T)urand(r, 250, 450); par[2] = (T)urand(r, 300, 450); par[3] = (T)urand(r, 200, 300);
    par[4] = (T)urand(r, -0.6, 0.2); par[5] = (T)urand(r, 0.5, 0.75);
  } else {
    par[0] = (T)urand(r, 100, 900); par[1] = (T)urand(r, 100, 900); par[2] = (T)urand(r, 0, 800); par[3] = (T)urand(r, 0, 600);
    par[4] = (T)urand(r, -1, 1); par[5] = (T)urand(r, 0.05, 0.98);
  }
}

template <class T> static void run_all(uint64_t seed, long n, int sens) {
  Rng r(seed);
  for (long it = 0; it < n; ++it) {
    int kind = (int)(r() % 4);
    if (it % 3 == 0) kind = (int)(r() % 2);
    T par[6];
    gen_params<T>(r, kind, par);
    T p3[3], uv[2];
    int mode = (int)(r() % 8);
    const T xi = par[4], alpha = par[5];
    const T w1 = alpha > T(0.5) ? (T(1) - alpha) / alpha : alpha / (T(1) - alpha);
    const T w2 = (w1 + xi) / std::sqrt(T(2) * w1 * xi + xi * xi + T(1));
    if (mode <= 2) {  // direction in the field of view, depth 0.1 .. 60 m
      T az = (T)urand(r, 0, 2 * M_PI), el = (T)urand(r, 0, mode == 0 ? 1.2 : 2.6);
      T d = (T)std::exp(urand(r, std::log(0.1), std::log(60)));
      p3[0] = d * std::sin(el) * std::cos(az); p3[1] = d * std::sin(el) * std::sin(az); p3[2] = d * std::cos(el);
    } else if (mode == 3) {  // validity boundary z = -w2 * d1  <=>  z = -w2 r / sqrt(1 - w2^2), jittered by a few ulps
      T x = (T)urand(r, -5, 5), y = (T)urand(r, -5, 5);
      T rr = std::sqrt(x * x + y * y);
      T z = std::fabs(w2) < 1 ? -w2 * rr / std::sqrt(T(1) - w2 * w2) : (T)urand(r, -5, 5);
      p3[0] = x; p3[1] = y; p3[2] = jitter<T>(r, z, 6);
    } else if (mode == 4) {  // wide random box
      for (int i = 0; i < 3; ++i) p3[i] = (T)urand(r, -30, 30);
    } else if (mode == 5) {  // extreme magnitudes
      T s = (T)std::pow(10.0, urand(r, -20, 12));
      for (int i = 0; i < 3; ++i) p3[i] = (T)urand(r, -1, 1) * s;
    } else if (mode == 6) {  // special values
      const T sp[] = {T(0), T(-0.0), T(1), T(-1), std::numeric_limits<T>::min(), std::numeric_limits<T>::infinity(), -std::numeric_limits<T>::infinity(), std::numeric_limits<T>::quiet_NaN(), T(1e-30), T(1e30)};
      for (int i = 0; i < 3; ++i) p3[i] = sp[r() % 10];
    } else {  // near the optical axis / near origin
      p3[0] = (T)urand(r, -1e-3, 1e-3); p3[1] = (T)urand(r, -1e-3, 1e-3); p3[2] = (T)urand(r, -1, 1);
    }
    int umode = (int)(r() % 6);
    if (umode <= 2) {  // image area (EuRoC 752x480) plus margin
      uv[0] = (T)urand(r, -30, 782); uv[1] = (T)urand(r, -30, 510);
    } else if (umode == 3 && alpha > T(0.5)) {  // validity boundary r2 = 1 / (2 alpha - 1), jittered
      T rb = std::sqrt(T(1) / (T(2) * alpha - T(1)));
      T ang = (T)urand(r, 0, 2 * M_PI);
      rb = jitter<T>(r, rb, 8);
      uv[0] = par[0] * (rb * std::cos(ang)) + par[2]; uv[1] = par[1] * (rb * std::sin(ang)) + par[3];
    } else if (umode == 4) {
      uv[0] = (T)urand(r, -3000, 3000); uv[1] = (T)urand(r, -3000, 3000);
    } else {
      uv[0] = par[2] + (T)urand(r, -1e-2, 1e-2); uv[1] = par[3] + (T)urand(r, -1e-2, 1e-2);
    }
    run_case<T>(par, p3, uv, sens);
  }
}

// Replay of the real M0 FLOW dump (runs/basalt_port/dumps/<tag>/flow.bin; framing u32 tag, u64 bytes, payload; tag 1 = FLOW:
// i64 t_ns, u64 frame_counter, u32 ncam, ncam x {u32 w, u32 h, u64 hash}, ncam x {u32 n, n x {u64 id, f32 m[6] row-major 2x3}}).
// Every tracked keypoint position (= translation m[2], m[5]) of both EuRoC cameras goes through unproject (Vec4, no Jacobian, as in
// measure() and the epipolar filter) and then project back; real class vs C port, memcmp.
static int replay(const char* path) {
  FILE* f = std::fopen(path, "rb");
  if (!f) { std::perror(path); return 2; }
  static const double e[2][6] = {{349.7560023050409, 348.72454229977037, 365.89440762590149, 249.32995565708704, -0.2409573942178872, 0.566996899163044},
                                 {361.6713883800533, 360.5856493689301, 379.40818394080869, 255.9772968522045, -0.21300835384809328, 0.5767008625037023}};
  basalt::DoubleSphereCamera<float> cam[2];
  bs_ds_f cc[2];
  for (int c = 0; c < 2; ++c) {
    bs_ds_cast_f(&cc[c], e[c]);
    basalt::DoubleSphereCamera<float>::VecN pv;
    for (int i = 0; i < 6; ++i) pv[i] = (float)e[c][i];
    cam[c] = basalt::DoubleSphereCamera<float>(pv);
  }
  long nrec = 0, npts = 0, bad = 0, invalid = 0, pinvalid = 0;
  uint32_t tag; uint64_t len;
  while (std::fread(&tag, 4, 1, f) == 1 && std::fread(&len, 8, 1, f) == 1) {
    std::vector<unsigned char> buf(len);
    if (len && std::fread(buf.data(), 1, len, f) != len) { std::fprintf(stderr, "truncated\n"); return 2; }
    if (tag != 1) continue;
    size_t o = 8 + 8;
    uint32_t ncam; std::memcpy(&ncam, &buf[o], 4); o += 4;
    o += (size_t)ncam * 16;
    for (uint32_t c = 0; c < ncam; ++c) {
      uint32_t n; std::memcpy(&n, &buf[o], 4); o += 4;
      for (uint32_t k = 0; k < n; ++k) {
        float m[6]; std::memcpy(m, &buf[o + 8], 24); o += 32;
        float uv[2] = {m[2], m[5]};
        Eigen::Vector2f u(uv[0], uv[1]);
        Eigen::Vector4f q; bool v = cam[c].unproject(u, q);
        float cq[4]; int cv = bs_ds_unproject_f(&cc[c], uv, cq, nullptr, nullptr);
        // project the unprojected ray (times a depth) back, as the estimator does for a landmark
        float p3[3] = {q[0] * 2.5f, q[1] * 2.5f, q[2] * 2.5f};
        Eigen::Vector4f P(p3[0], p3[1], p3[2], 0.4f); Eigen::Vector2f r; Eigen::Matrix<float, 2, 4> J;
        bool pv = cam[c].project(P, r, &J);
        float cr[2], cj[8]; int cpv = bs_ds_project_f(&cc[c], p3, cr, cj, nullptr);
        bool ok = (int)v == cv && std::memcmp(q.data(), cq, 16) == 0 && (int)pv == cpv && std::memcmp(r.data(), cr, 8) == 0 && std::memcmp(J.data(), cj, 32) == 0;
        ++npts; bad += !ok; invalid += !v; pinvalid += !pv;
      }
    }
    ++nrec;
  }
  std::fclose(f);
  std::printf("bs_cam replay %s: %ld FLOW records, %ld keypoints (unproject invalid %ld, reprojection invalid %ld): %ld/%ld\n", path, nrec, npts, invalid, pinvalid, bad, npts);
  return bad ? 1 : 0;
}

int main(int argc, char** argv) {
  if (argc > 2 && std::strcmp(argv[1], "replay") == 0) return replay(argv[2]);
  uint64_t seed = argc > 1 ? strtoull(argv[1], 0, 10) : 1;
  long n = argc > 2 ? atol(argv[2]) : 20000;
  int sens = argc > 3 && std::strcmp(argv[3], "sens") == 0;
  g_verbose = 1;
  long tot0, bad0;
  run_all<float>(seed, n, sens);
  tot0 = g_total; bad0 = g_bad;
  std::printf("  float : %ld comparisons (%ld project + %ld unproject, each a 3-4 call-shape bundle incl. Jacobians), mismatches %ld\n", tot0, tot0 / 2, tot0 / 2, bad0);
  run_all<double>(seed + 1000003, n, sens);
  std::printf("  double: %ld comparisons, mismatches %ld\n", g_total - tot0, g_bad - bad0);
  std::printf("  valid %ld invalid %ld non-finite-output %ld (NaN-input cases, NaN-vs-NaN match: %ld)\n", g_valid, g_invalid, g_nonfinite, g_nan_cases);
  if (sens) { std::printf("sensitivity (fx perturbed by 1 ulp in the C port): %ld/%ld bundles differ\n", g_bad, g_total); return g_bad > 0 ? 0 : 1; }
  std::printf("bs_cam seed %llu: %ld/%ld\n", (unsigned long long)seed, g_bad, g_total);
  return g_bad ? 1 : 0;
}
