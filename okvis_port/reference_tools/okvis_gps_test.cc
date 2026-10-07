// OK_PORT_TEST_C: ok_gps.c ok_gps_init.c ok_err.c ok_param.c ok_kin.c ok_eigen.c ok_imu.c ok_time.c ok_cam.c ok_blas.c
// OK_PORT_TEST_LIBS: ceres
// Tolerance-0 (memcmp) comparison of okvis_port/c/ok_gps.{h,c} against the REAL OKVIS2-X classes
// (commit 38043e4: GpsErrorSynchronous, GpsErrorAsynchronous, PoseManifold4d; sources and headers of
// external/gnss/OKVIS2-X are compiled unmodified, with the compile flags and include set of the deterministic reference
// build, see runs/okvis2x_port/gps_leaf/build_oracle.sh: g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math, Eigen 3.4.0).
// The tool runner (tools/check_okvis_port.py) puts the OKVIS2 headers on the include path, not X's, so under it this file
// only prints SKIP; build and run it with runs/okvis2x_port/gps_leaf/build_oracle.sh.
// Sections: PoseManifold4d plus / minus / plusJacobian / minusJacobian (static functions and the ceres::Manifold virtuals,
// RightMultiplyByPlusJacobian for 1..40 rows), GpsErrorSynchronous (constructor, setInformation, residuals, full and
// minimal Jacobians of both blocks, pointer-combination semantics), GpsErrorAsynchronous (constructors, residuals, Jacobians of
// all three blocks, error(), the redo logic over call sequences, both static switches, applyPreInt).
// Inputs: random and EuRoC-like poses, near-degenerate / axis-aligned / unnormalised quaternions, IMU segments of 0-1 s at 200 Hz
// and 1 kHz with realistic measurements, GNSS stamps on and between IMU stamps, lever arm != 0, information matrices from diagonal
// to full SPD (and a few non-PD ones).
// Usage: okvis_gps_test [cases_per_seed [seed ...]]   (default 20000 cases, seeds 1 2 3 4); OKGPS_VERBOSE=1 prints mismatches.
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#if !__has_include(<okvis/ceres/GpsErrorSynchronous.hpp>)
int main() {
  std::printf("okvis_gps_test: SKIP (OKVIS2-X headers are not on the include path; run "
              "runs/okvis2x_port/gps_leaf/build_oracle.sh)\n");
  return 0;
}
#else

#include <okvis/Measurements.hpp>
#include <okvis/Parameters.hpp>
#include <okvis/Time.hpp>
#include <okvis/ceres/GpsErrorAsynchronous.hpp>
#include <okvis/ceres/GpsErrorSynchronous.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/kinematics/Transformation.hpp>
extern "C" {
#include "../c/ok_gps.h"
}

namespace {

struct Stat {
  uint64_t cases = 0, bytes = 0, mism = 0;
};
std::map<std::string, Stat> g_stats;
std::map<std::string, uint64_t> g_cov;  // coverage of the generated scenarios
std::map<std::string, uint64_t> g_where;  // "section:field[index]" -> mismatching doubles (histogram, verbose only)
bool g_verbose = false;

void compare(const char* sec, const char* field, const void* ref, const void* got, size_t nbytes) {
  Stat& s = g_stats[sec];
  s.bytes += nbytes;
  if (std::memcmp(ref, got, nbytes) != 0) {
    ++s.mism;
    if (g_verbose) {
      const double* a = static_cast<const double*>(ref);
      const double* b = static_cast<const double*>(got);
      for (size_t i = 0; i < nbytes / 8; ++i)
        if (std::memcmp(a + i, b + i, 8) != 0) {
          char key[160];
          std::snprintf(key, sizeof key, "%s:%s[%zu]", sec, field, i);
          if (++g_where[key] == 1)
            std::printf("  first mismatch %s: ref %.17g got %.17g\n", key, a[i], b[i]);
        }
    }
  }
}

struct Rng {
  std::mt19937_64 g;
  explicit Rng(uint64_t seed) : g(seed * 0x9E3779B97F4A7C15ull + 12345) {}
  double u(double a, double b) { return a + (b - a) * (double(g() >> 11) / 9007199254740992.0); }
  double n() {  // Box-Muller
    double a = u(1e-300, 1.0), b = u(0.0, 1.0);
    return std::sqrt(-2.0 * std::log(a)) * std::cos(6.283185307179586 * b);
  }
  int i(int a, int b) { return a + int(g() % uint64_t(b - a + 1)); }
  bool p(double pr) { return u(0, 1) < pr; }
};

const double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------- generators
void setQ(double* p, double x, double y, double z, double w) {
  p[3] = x; p[4] = y; p[5] = z; p[6] = w;
}
void genQuat(Rng& r, double* p, int mode) {
  Eigen::Quaterniond q;
  switch (mode) {
    case 0: {  // uniform
      Eigen::Vector4d v(r.n(), r.n(), r.n(), r.n());
      v.normalize();
      setQ(p, v[0], v[1], v[2], v[3]);
      return;
    }
    case 1: {  // EuRoC-like: any yaw, small roll / pitch
      q = Eigen::AngleAxisd(r.u(-kPi, kPi), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(0.15 * r.n(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(0.15 * r.n(), Eigen::Vector3d::UnitX());
      break;
    }
    case 2: {  // near-degenerate yaw / gimbal
      const double yaws[] = {0, kPi, -kPi, kPi / 2, -kPi / 2, 1e-9, -1e-9, kPi - 1e-9, -kPi + 1e-9, 3.0e-7};
      const double rps[] = {0, kPi / 2, -kPi / 2, 1e-8, -1e-8, kPi, 0.3, -0.3};
      double yaw = yaws[r.i(0, 9)], pi = rps[r.i(0, 7)], ro = rps[r.i(0, 7)];
      q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(ro, Eigen::Vector3d::UnitX());
      break;
    }
    case 3: {  // tiny rotation around identity (and its negative)
      double s = r.p(0.5) ? 1.0 : -1.0;
      Eigen::Vector4d v(1e-8 * r.n(), 1e-8 * r.n(), 1e-8 * r.n(), s);
      v.normalize();
      setQ(p, v[0], v[1], v[2], v[3]);
      return;
    }
    case 4: {  // not unit (parameter blocks are stored raw)
      Eigen::Vector4d v(r.n(), r.n(), r.n(), r.n());
      v.normalize();
      v *= r.u(0.3, 3.0);
      setQ(p, v[0], v[1], v[2], v[3]);
      return;
    }
    default: {  // axis aligned / exact
      const double h = std::sqrt(0.5);
      const double c[][4] = {{0, 0, 0, 1}, {0, 0, 0, -1}, {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, -1, 0},
                             {0.5, 0.5, 0.5, 0.5}, {-0.5, 0.5, -0.5, 0.5}, {h, 0, 0, h}, {0, h, 0, h}, {0, 0, h, h},
                             {0, 0, -h, h}, {0, 0, h, -h}};
      const double* v = c[r.i(0, 12)];
      setQ(p, v[0], v[1], v[2], v[3]);
      return;
    }
  }
  setQ(p, q.x(), q.y(), q.z(), q.w());
}
void genPose(Rng& r, double* p, bool world = false) {
  const int mode = r.p(0.45) ? 1 : r.p(0.3) ? 0 : r.i(2, 5);
  genQuat(r, p, mode);
  if (r.p(0.04)) { p[0] = p[1] = p[2] = 0.0; return; }
  if (r.p(0.04)) { p[0] = r.i(-3, 3); p[1] = r.i(-3, 3); p[2] = r.i(-3, 3); return; }
  const double sc = world ? std::pow(10.0, r.u(0, 6)) : (mode == 1 ? 5.0 : std::pow(10.0, r.u(-2, 3)));
  p[0] = sc * r.n(); p[1] = sc * r.n(); p[2] = (mode == 1 && !world) ? r.u(0.3, 2.5) : sc * r.n();
}
void genInfo(Rng& r, double* I) {  // 3x3 column-major information
  const int kind = r.p(0.5) ? 0 : r.p(0.6) ? 1 : r.p(0.7) ? 2 : r.p(0.2) ? 4 : 3;
  if (kind == 0) {  // diagonal, 1 cm ... 10 m
    for (int k = 0; k < 9; ++k) I[k] = 0.0;
    for (int d = 0; d < 3; ++d) { double s = std::pow(10.0, r.u(-2.3, 1.0)); I[d * 4] = 1.0 / (s * s); }
  } else if (kind == 1 || kind == 2) {  // full SPD
    double A[9];
    for (int k = 0; k < 9; ++k) A[k] = r.n() * (kind == 1 ? 1.0 : 30.0);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j <= i; ++j) {
        double s = 0;
        for (int k = 0; k < 3; ++k) s += A[k + 3 * i] * A[k + 3 * j];
        I[i + 3 * j] = I[j + 3 * i] = s;
      }
    for (int d = 0; d < 3; ++d) I[d * 4] += std::pow(10.0, r.u(-2, 3));
    if (r.p(0.03)) I[1] += 1e-3 * r.n();  // slightly non-symmetric
  } else if (kind == 4) {  // identity
    for (int k = 0; k < 9; ++k) I[k] = (k % 4 == 0) ? 1.0 : 0.0;
  } else {  // symmetric, possibly not positive definite
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j <= i; ++j) I[i + 3 * j] = I[j + 3 * i] = r.n() * 10.0;
  }
}
void genVec3(Rng& r, double* v, double s) {
  for (int k = 0; k < 3; ++k) v[k] = s * r.n();
}

struct ImuSeg {
  std::vector<ok_imu_meas> m;
  ok_imu_params prm;
  ok_time tk, tg;
};
ok_time toTime(uint64_t sec, uint64_t ns) {
  ok_time t;
  t.sec = uint32_t(sec + ns / 1000000000ull);
  t.nsec = uint32_t(ns % 1000000000ull);
  return t;
}
ImuSeg genImu(Rng& r) {
  ImuSeg s;
  const double scale = r.u(0.5, 2.0);
  s.prm.sigma_g_c = 12.0e-4 * scale;
  s.prm.sigma_a_c = 8.0e-3 * scale;
  s.prm.sigma_gw_c = 4.0e-6 * scale;
  s.prm.sigma_aw_c = 4.4e-5 * scale;
  s.prm.g = r.p(0.7) ? 9.81 : 9.80665;
  s.prm.g_max = r.p(0.9) ? 7.8 : 1.2;
  s.prm.a_max = r.p(0.9) ? 176.0 : 12.0;
  const uint64_t sec0 = r.p(0.6) ? 1403636579ull : (r.p(0.5) ? 5ull : 4000000000ull);
  const uint64_t period = r.p(0.7) ? 5000000ull : (r.p(0.5) ? 1000000ull : 10000000ull);  // 200 Hz, 1 kHz, 100 Hz
  const uint64_t ns0 = uint64_t(r.u(0, 999999999));
  const double len = r.p(0.08) ? 0.0 : (r.p(0.15) ? r.u(0, 0.05) : r.u(0, 1.0));
  const uint64_t lenns = uint64_t(len * 1e9);
  // stamps: first <= tk, last >= tg, optionally jittered
  const uint64_t margin = r.p(0.3) ? 0 : uint64_t(r.u(0, 1) * double(period));
  const uint64_t tkns = sec0 * 1000000000ull + ns0;  // absolute ns
  uint64_t t = tkns - margin;
  std::vector<uint64_t> st;
  st.push_back(t);
  const bool jit = r.p(0.5);
  const uint64_t tgns_target = tkns + lenns;
  while (t < tgns_target || st.size() < 2) {
    uint64_t dt = period;
    if (jit) dt = uint64_t(double(period) * r.u(0.7, 1.3));
    if (dt == 0) dt = 1;
    t += dt;
    st.push_back(t);
    if (st.size() > 1200) break;
  }
  if (r.p(0.06)) {  // boundary of the redo rule (n_meas < 50): exactly 49, 50 or 51 measurements
    const size_t N = size_t(r.i(49, 51));
    st.clear();
    uint64_t tt = tkns - margin;
    for (size_t j = 0; j < N; ++j) {
      st.push_back(tt);
      tt += jit ? uint64_t(double(period) * r.u(0.7, 1.3)) + 1 : period;
    }
  }
  // tg: exactly on a stamp, between, one ns off, or == tk
  uint64_t tgns;
  const double c = r.u(0, 1);
  if (st.size() <= 51 && st.size() >= 49 && st.back() >= tkns && c < 0.6) {  // boundary scenario: anywhere up to the last stamp
    tgns = tkns + uint64_t(r.u(0, 1) * double(st.back() - tkns));
  } else if (lenns == 0 && c < 0.5) tgns = tkns;
  else if (c < 0.35) {  // on a stamp (>= tk)
    size_t j = 0;
    while (j + 1 < st.size() && st[j] < tgns_target) ++j;
    tgns = st[j];
    if (tgns < tkns) tgns = tkns;
  } else if (c < 0.45) {  // one ns before/after a stamp
    size_t j = 0;
    while (j + 1 < st.size() && st[j] < tgns_target) ++j;
    tgns = st[j] - (r.p(0.5) ? 1 : 0);
    if (tgns < tkns) tgns = tkns;
  } else tgns = tgns_target;
  if (tgns > st.back()) tgns = st.back();
  if (tgns < tkns) tgns = tkns;
  s.tk = toTime(tkns / 1000000000ull, tkns % 1000000000ull);
  s.tg = toTime(tgns / 1000000000ull, tgns % 1000000000ull);
  // measurement values: gyro + accelerometer in a sensor frame whose z points up (acc ~ +g), motion and noise
  const double gyr_sc = r.p(0.8) ? 0.3 : 2.0;
  Eigen::Vector3d gbase(r.n() * gyr_sc, r.n() * gyr_sc, r.n() * gyr_sc);
  const Eigen::Vector3d down = Eigen::Vector3d(0.3 * r.n(), 0.3 * r.n(), 1.0).normalized() * s.prm.g;
  for (size_t j = 0; j < st.size(); ++j) {
    ok_imu_meas m;
    m.t = toTime(st[j] / 1000000000ull, st[j] % 1000000000ull);
    for (int k = 0; k < 3; ++k) {
      m.gyr[k] = gbase[k] * std::sin(0.01 * double(j) + k) + 0.02 * r.n();
      m.acc[k] = down[k] + 1.5 * r.n();
    }
    if (r.p(0.01)) m.gyr[r.i(0, 2)] = (r.p(0.5) ? 1 : -1) * r.u(5, 12);   // saturation
    if (r.p(0.005)) m.acc[r.i(0, 2)] = (r.p(0.5) ? 1 : -1) * r.u(100, 250);
    s.m.push_back(m);
  }
  return s;
}
void genSb(Rng& r, double* sb) {
  for (int k = 0; k < 3; ++k) sb[k] = 1.5 * r.n();
  for (int k = 3; k < 6; ++k) sb[k] = 0.02 * r.n();
  for (int k = 6; k < 9; ++k) sb[k] = 0.15 * r.n();
}

okvis::Time toOk(ok_time t) { return okvis::Time(t.sec, t.nsec); }
okvis::ImuMeasurementDeque toDeque(const ImuSeg& s) {
  okvis::ImuMeasurementDeque d;
  for (const auto& m : s.m)
    d.push_back(okvis::ImuMeasurement(
        okvis::Time(m.t.sec, m.t.nsec),
        okvis::ImuSensorReadings(Eigen::Vector3d(m.gyr[0], m.gyr[1], m.gyr[2]), Eigen::Vector3d(m.acc[0], m.acc[1], m.acc[2]))));
  return d;
}
okvis::ImuParameters toImuPrm(const ok_imu_params& p) {
  okvis::ImuParameters q;
  q.sigma_g_c = p.sigma_g_c; q.sigma_a_c = p.sigma_a_c; q.sigma_gw_c = p.sigma_gw_c; q.sigma_aw_c = p.sigma_aw_c;
  q.g = p.g; q.g_max = p.g_max; q.a_max = p.a_max;
  return q;
}

const unsigned char kSentinel = 0xA5;
struct Bufs {  // output buffers of one Evaluate call
  double res[3], j[3][27], jm[3][27];
  void fill() {
    std::memset(res, kSentinel, sizeof res);
    std::memset(j, kSentinel, sizeof j);
    std::memset(jm, kSentinel, sizeof jm);
  }
};
Eigen::Vector3d V3(const double* v) { return Eigen::Vector3d(v[0], v[1], v[2]); }
Eigen::Matrix3d M3(const double* m) { return Eigen::Map<const Eigen::Matrix3d>(m); }

// ---------------------------------------------------------------- manifold
void test_manifold(Rng& r, int cases) {
  okvis::ceres::PoseManifold4d pm;
  const ::ceres::Manifold& mf = pm;
  for (int c = 0; c < cases; ++c) {
    double x[7], y[7], d[4], a[7], b[7], ja[28], jb[28], m4a[4], m4b[4];
    genPose(r, x, r.p(0.2));
    for (int k = 0; k < 3; ++k) d[k] = std::pow(10.0, r.u(-9, 0.5)) * r.n();
    d[3] = r.p(0.1) ? r.u(-kPi, kPi) : std::pow(10.0, r.u(-9, 0.3)) * r.n();
    if (r.p(0.05)) d[3] = 0;
    if (r.p(0.03)) d[0] = d[1] = d[2] = 0;
    // sizes
    {
      int sz[2] = {pm.AmbientSize(), pm.TangentSize()}, sz2[2] = {mf.AmbientSize(), mf.TangentSize()}, mine[2] = {OK_POSE4_AMBIENT_SIZE, OK_POSE4_TANGENT_SIZE};
      compare("manifold4 sizes", "ambient/tangent", sz, mine, 8);
      compare("manifold4 sizes", "virtual", sz2, mine, 8);
    }
    // plus: static + virtual
    bool ok1 = okvis::ceres::PoseManifold4d::plus(x, d, a), ok2 = mf.Plus(x, d, b);
    double ok_out[7];
    int rc = ok_pose4_plus(x, d, ok_out);
    compare("manifold4 plus", "static", a, ok_out, 56);
    compare("manifold4 plus", "virtual", b, ok_out, 56);
    if (!ok1 || !ok2 || rc != 1) ++g_stats["manifold4 plus"].mism;
    // minus
    const int ymode = r.i(0, 4);
    if (ymode == 0) std::memcpy(y, a, sizeof y);
    else if (ymode == 1) std::memcpy(y, x, sizeof y);
    else if (ymode == 2) { genPose(r, y, r.p(0.2)); }
    else if (ymode == 3) { std::memcpy(y, a, sizeof y); for (int k = 3; k < 7; ++k) y[k] = -y[k]; }
    else { std::memcpy(y, x, sizeof y); y[0] += r.n(); y[3] = -y[3]; y[4] = -y[4]; y[5] = -y[5]; y[6] = -y[6]; }
    okvis::ceres::PoseManifold4d::minus(y, x, m4a);
    mf.Minus(y, x, m4b);
    double mo[4];
    ok_pose4_minus(y, x, mo);
    compare("manifold4 minus", "static", m4a, mo, 32);
    compare("manifold4 minus", "virtual", m4b, mo, 32);
    // plusJacobian / minusJacobian
    std::memset(ja, kSentinel, sizeof ja); std::memset(jb, kSentinel, sizeof jb);
    okvis::ceres::PoseManifold4d::plusJacobian(x, ja);
    mf.PlusJacobian(x, jb);
    double jo[28];
    std::memset(jo, kSentinel, sizeof jo);
    ok_pose4_plus_jacobian(x, jo);
    compare("manifold4 plusJacobian", "static", ja, jo, 224);
    compare("manifold4 plusJacobian", "virtual", jb, jo, 224);
    std::memset(ja, kSentinel, sizeof ja); std::memset(jb, kSentinel, sizeof jb);
    okvis::ceres::PoseManifold4d::minusJacobian(x, ja);
    mf.MinusJacobian(x, jb);
    std::memset(jo, kSentinel, sizeof jo);
    ok_pose4_minus_jacobian(x, jo);
    compare("manifold4 minusJacobian", "static", ja, jo, 224);
    compare("manifold4 minusJacobian", "virtual", jb, jo, 224);
    // RightMultiplyByPlusJacobian (the base-class default): rows 1..40 with random ambient matrices
    {
      const int rows = (c % 4 == 0) ? r.i(1, 40) : r.i(1, 16);
      std::vector<double> A(size_t(rows) * 7), o1(size_t(rows) * 4, 0.0), o2(size_t(rows) * 4, 0.0);
      for (auto& v : A) v = r.p(0.1) ? 0.0 : (r.p(0.5) ? r.n() : std::pow(10.0, r.u(-3, 3)) * r.n());
      std::memset(o1.data(), kSentinel, o1.size() * 8); std::memset(o2.data(), kSentinel, o2.size() * 8);
      mf.RightMultiplyByPlusJacobian(x, rows, A.data(), o1.data());
      ok_pose4_right_multiply(x, rows, A.data(), o2.data());
      compare("manifold4 RightMultiplyByPlusJacobian", "out", o1.data(), o2.data(), o1.size() * 8);
    }
    for (const char* s : {"manifold4 sizes", "manifold4 plus", "manifold4 minus", "manifold4 plusJacobian", "manifold4 minusJacobian",
                          "manifold4 RightMultiplyByPlusJacobian"})
      g_stats[s].cases++;
  }
}

// ---------------------------------------------------------------- synchronous
void test_sync(Rng& r, int cases) {
  for (int c = 0; c < cases; ++c) {
    double meas[3], lever[3], info[9], P0[7], P1[7];
    genVec3(r, meas, std::pow(10.0, r.u(-1, 3)));
    if (r.p(0.85)) { lever[0] = r.u(-0.3, 0.3); lever[1] = r.u(-0.3, 0.3); lever[2] = r.u(-0.3, 0.3); }
    else if (r.p(0.5)) { lever[0] = lever[1] = lever[2] = 0.0; }
    else genVec3(r, lever, 2.0);
    genInfo(r, info);
    genPose(r, P0);
    genPose(r, P1, true);
    if (r.p(0.4)) {  // measurement close to the prediction (small residual)
      Eigen::Quaterniond q0(P0[6], P0[3], P0[4], P0[5]), q1(P1[6], P1[3], P1[4], P1[5]);
      Eigen::Vector3d pred = q1.toRotationMatrix() * (V3(P0) + q0.toRotationMatrix() * V3(lever)) + V3(P1);
      for (int k = 0; k < 3; ++k) meas[k] = pred[k] + 0.02 * r.n();
    }
    okvis::GpsParameters gp;
    gp.r_SA = V3(lever);
    okvis::ceres::GpsErrorSynchronous real(7, V3(meas), M3(info), gp);
    ok_gps_sync cs;
    ok_gps_sync_init(&cs, meas, info, lever);
    compare("sync ctor", "measurement", real.measurement().data(), cs.meas, 24);
    compare("sync ctor", "information", real.information().data(), cs.info, 72);
    compare("sync ctor", "covariance", real.covariance().data(), cs.covariance, 72);
    if (r.p(0.15)) {  // setInformation again
      double info2[9];
      genInfo(r, info2);
      real.setInformation(M3(info2));
      ok_gps_sync_set_information(&cs, info2);
      compare("sync ctor", "information2", real.information().data(), cs.info, 72);
      compare("sync ctor", "covariance2", real.covariance().data(), cs.covariance, 72);
    }
    for (int rep = 0; rep < 2; ++rep) {
      const double* params[2] = {P0, P1};
      Bufs a, b;
      a.fill(); b.fill();
      // pointer pattern: jacobians NULL / entries NULL; minimal NULL / entries NULL
      const int pat = r.i(0, 9);
      double* ja[2] = {a.j[0], a.j[1]};
      double* jb[2] = {b.j[0], b.j[1]};
      double* jma[2] = {a.jm[0], a.jm[1]};
      double* jmb[2] = {b.jm[0], b.jm[1]};
      if (pat == 1) { ja[0] = jb[0] = nullptr; }
      if (pat == 2) { ja[1] = jb[1] = nullptr; }
      if (pat == 3) { jma[0] = jmb[0] = nullptr; }
      if (pat == 4) { jma[1] = jmb[1] = nullptr; }
      if (pat == 5) { ja[0] = jb[0] = nullptr; ja[1] = jb[1] = nullptr; }
      const bool useJac = pat != 6, useMin = rep == 0 && pat != 7;
      if (rep == 1) {  // the plain Evaluate entry point
        real.Evaluate(params, a.res, useJac ? ja : nullptr);
        ok_gps_sync_evaluate(&cs, params, b.res, useJac ? jb : nullptr, nullptr);
      } else {
        real.EvaluateWithMinimalJacobians(params, a.res, useJac ? ja : nullptr, useMin ? jma : nullptr);
        ok_gps_sync_evaluate(&cs, params, b.res, useJac ? jb : nullptr, useMin ? jmb : nullptr);
      }
      compare("sync residual", "res", a.res, b.res, 24);
      compare("sync jacobian T_WS (3x7)", "J0", a.j[0], b.j[0], 27 * 8);
      compare("sync jacobian T_GW (3x7)", "J1", a.j[1], b.j[1], 27 * 8);
      compare("sync minimal T_WS (3x6)", "Jm0", a.jm[0], b.jm[0], 27 * 8);
      compare("sync minimal T_GW (3x6)", "Jm1", a.jm[1], b.jm[1], 27 * 8);
      for (const char* s : {"sync residual", "sync jacobian T_WS (3x7)", "sync jacobian T_GW (3x7)", "sync minimal T_WS (3x6)",
                            "sync minimal T_GW (3x6)"})
        g_stats[s].cases++;
    }
    g_stats["sync ctor"].cases++;
  }
}

// ---------------------------------------------------------------- asynchronous
void test_async(Rng& r, int cases) {
  for (int c = 0; c < cases; ++c) {
    double meas[3], lever[3], info[9], sigma[3];
    ImuSeg seg = genImu(r);
    genVec3(r, meas, std::pow(10.0, r.u(-1, 3)));
    if (r.p(0.88)) { lever[0] = r.u(-0.3, 0.3); lever[1] = r.u(-0.3, 0.3); lever[2] = r.u(-0.3, 0.3); }
    else if (r.p(0.5)) { lever[0] = lever[1] = lever[2] = 0.0; }
    else genVec3(r, lever, 1.5);
    const bool sigmaCtor = r.p(0.3);
    for (int k = 0; k < 3; ++k) sigma[k] = std::pow(10.0, r.u(-2.3, 1.0));
    genInfo(r, info);
    okvis::GpsParameters gp;
    gp.r_SA = V3(lever);
    const bool cov = !r.p(0.15), always = r.p(0.1);
    okvis::ceres::GpsErrorAsynchronous::useImuCovariance = cov;
    okvis::ceres::GpsErrorAsynchronous::redoPropagationAlways = always;
    okvis::ImuMeasurementDeque dq = toDeque(seg);
    okvis::ImuParameters ip = toImuPrm(seg.prm);
    std::unique_ptr<okvis::ceres::GpsErrorAsynchronous> real;
    if (sigmaCtor)
      real.reset(new okvis::ceres::GpsErrorAsynchronous(V3(meas), sigma[0], sigma[1], sigma[2], dq, ip, toOk(seg.tk), toOk(seg.tg), gp));
    else
      real.reset(new okvis::ceres::GpsErrorAsynchronous(V3(meas), M3(info), dq, ip, toOk(seg.tk), toOk(seg.tg), gp));
    g_cov[seg.m.size() >= 50 ? "async n_meas >= 50" : "async n_meas < 50"]++;
    if (seg.m.size() >= 49 && seg.m.size() <= 51) g_cov["async n_meas in 49..51"]++;
    if (seg.tk.sec == seg.tg.sec && seg.tk.nsec == seg.tg.nsec) g_cov["async tk == tg"]++;
    else {
      bool on = false;
      for (const auto& m : seg.m) on |= (m.t.sec == seg.tg.sec && m.t.nsec == seg.tg.nsec);
      g_cov[on ? "async tg on an IMU stamp" : "async tg between IMU stamps"]++;
    }
    {
      bool sat = false;
      for (const auto& m : seg.m)
        for (int k = 0; k < 3; ++k) sat |= std::fabs(m.gyr[k]) > seg.prm.g_max || std::fabs(m.acc[k]) > seg.prm.a_max;
      if (sat) g_cov["async saturated measurement"]++;
    }
    if (!cov) g_cov["async useImuCovariance = false"]++;
    if (always) g_cov["async redoPropagationAlways = true"]++;
    if (lever[0] == 0 && lever[1] == 0 && lever[2] == 0) g_cov["async lever arm = 0"]++;
    ok_gps_async ca;
    if (sigmaCtor) ok_gps_async_init_sigma(&ca, meas, sigma, lever, seg.m.data(), seg.m.size(), &seg.prm, seg.tk, seg.tg);
    else ok_gps_async_init(&ca, meas, info, lever, seg.m.data(), seg.m.size(), &seg.prm, seg.tk, seg.tg);
    ca.use_imu_covariance = cov;
    ca.redo_always = always;
    compare("async ctor", "measurement", real->measurement().data(), ca.meas, 24);
    compare("async ctor", "information", real->information().data(), ca.info, 72);
    compare("async ctor", "covariance", real->covariance().data(), ca.covariance, 72);
    {
      double tt[2] = {0, 0}, tc[2] = {0, 0};
      uint32_t a[4] = {real->tk().sec, real->tk().nsec, real->tg().sec, real->tg().nsec};
      uint32_t b[4] = {ca.imu.t0.sec, ca.imu.t0.nsec, ca.imu.t1.sec, ca.imu.t1.nsec};
      (void)tt; (void)tc;
      compare("async ctor", "tk/tg", a, b, 16);
    }
    if (r.p(0.1)) {
      double info2[9];
      genInfo(r, info2);
      real->setInformation(M3(info2));
      ok_gps_async_set_information(&ca, info2);
      compare("async ctor", "information2", real->information().data(), ca.info, 72);
      compare("async ctor", "covariance2", real->covariance().data(), ca.covariance, 72);
    }
    // state before any evaluation: applyPreInt with the zero preintegration
    double P0[7], P2[7], SB[9];
    genPose(r, P0);
    genPose(r, P2, true);
    genSb(r, SB);
    if (r.p(0.4) && (P0[0] != 0 || true)) {
      // measurement near the prediction (small residuals) for realism
      Eigen::Quaterniond q0(P0[6], P0[3], P0[4], P0[5]);
      q0.normalize();
      Eigen::Quaterniond q2(P2[6], P2[3], P2[4], P2[5]);
      const double dt = (toOk(seg.tg) - toOk(seg.tk)).toSec();
      Eigen::Vector3d ant = V3(P0) + V3(SB) * dt + q0.toRotationMatrix() * V3(lever);
      Eigen::Vector3d pred = q2.toRotationMatrix() * ant + V3(P2);
      for (int k = 0; k < 3; ++k) meas[k] = pred[k] + 0.05 * r.n();
      real->setMeasurement(V3(meas));
      std::memcpy(ca.meas, meas, 24);
    }
    auto apply = [&](const char* tag) {
      double Pi[7], sbi[9];
      genPose(r, Pi);
      genSb(r, sbi);
      if (r.p(0.3)) std::memcpy(Pi, P0, sizeof Pi);
      if (r.p(0.3)) std::memcpy(sbi, SB, sizeof sbi);
      okvis::kinematics::Transformation Tin(V3(Pi), Eigen::Quaterniond(Pi[6], Pi[3], Pi[4], Pi[5])), Tout;
      real->applyPreInt(Tin, Eigen::Matrix<double, 9, 1>(sbi), Tout);
      ok_tf tin, tout;
      const ok_quat qq = {Pi[3], Pi[4], Pi[5], Pi[6]};
      ok_tf_from_rq(&tin, Pi, &qq, 1);
      ok_gps_async_apply_preint(&ca, &tin, sbi, &tout);
      double a[3 + 4 + 9], b[3 + 4 + 9];
      std::memcpy(a, Tout.r().data(), 24); std::memcpy(a + 3, Tout.q().coeffs().data(), 32); std::memcpy(a + 7, Tout.C().data(), 72);
      std::memcpy(b, tout.r, 24); b[3] = tout.q.x; b[4] = tout.q.y; b[5] = tout.q.z; b[6] = tout.q.w; std::memcpy(b + 7, tout.C, 72);
      compare(tag, "T_prop (r, q, C)", a, b, 128);
      g_stats[tag].cases++;
    };
    if (r.p(0.5)) apply("async applyPreInt (before redo)");
    // evaluation sequence on one object
    const int ncalls = r.i(1, 4);
    double sbcur[9], Pcur[7], P2cur[7];
    int prev_counter = 0;
    std::memcpy(sbcur, SB, sizeof sbcur); std::memcpy(Pcur, P0, sizeof Pcur); std::memcpy(P2cur, P2, sizeof P2cur);
    for (int call = 0; call < ncalls; ++call) {
      if (call > 0) {
        const int kind = r.i(0, 4);
        if (kind == 0) { /* identical */ }
        else if (kind == 1) { for (int k = 3; k < 6; ++k) sbcur[k] += 1e-5 * r.n(); for (int k = 6; k < 9; ++k) sbcur[k] += 1e-3 * r.n(); }
        else if (kind == 2) { for (int k = 3; k < 6; ++k) sbcur[k] += 3e-3 * r.n(); for (int k = 6; k < 9; ++k) sbcur[k] += 0.05 * r.n(); }
        else if (kind == 3) { genSb(r, sbcur); }
        else { genPose(r, Pcur); for (int k = 0; k < 3; ++k) sbcur[k] += 0.1 * r.n(); genPose(r, P2cur, true); }
      }
      const double* params[3] = {Pcur, sbcur, P2cur};
      Bufs a, b;
      a.fill(); b.fill();
      const int pat = r.i(0, 11);
      double* ja[3] = {a.j[0], a.j[1], a.j[2]};
      double* jb[3] = {b.j[0], b.j[1], b.j[2]};
      double* jma[3] = {a.jm[0], a.jm[1], a.jm[2]};
      double* jmb[3] = {b.jm[0], b.jm[1], b.jm[2]};
      if (pat >= 1 && pat <= 3) { ja[pat - 1] = jb[pat - 1] = nullptr; }
      if (pat >= 4 && pat <= 6) { jma[pat - 4] = jmb[pat - 4] = nullptr; }
      if (pat == 7) { for (int k = 0; k < 3; ++k) ja[k] = jb[k] = nullptr; }
      const bool useJac = pat != 8, useMin = pat != 9 && pat != 10;
      if (pat == 10) {
        real->Evaluate(params, a.res, useJac ? ja : nullptr);
        ok_gps_async_evaluate(&ca, params, b.res, useJac ? jb : nullptr, nullptr);
      } else {
        real->EvaluateWithMinimalJacobians(params, a.res, useJac ? ja : nullptr, useMin ? jma : nullptr);
        ok_gps_async_evaluate(&ca, params, b.res, useJac ? jb : nullptr, useMin ? jmb : nullptr);
      }
      compare("async residual", "res", a.res, b.res, 24);
      Eigen::Vector3d er = real->error();
      compare("async error()", "error", er.data(), ca.error, 24);
      compare("async jacobian T_WS(tk) (3x7)", "J0", a.j[0], b.j[0], 27 * 8);
      compare("async jacobian speed/bias (3x9)", "J1", a.j[1], b.j[1], 27 * 8);
      compare("async jacobian T_GW (3x7)", "J2", a.j[2], b.j[2], 27 * 8);
      compare("async minimal T_WS(tk) (3x6)", "Jm0", a.jm[0], b.jm[0], 27 * 8);
      compare("async minimal speed/bias (3x9)", "Jm1", a.jm[1], b.jm[1], 27 * 8);
      compare("async minimal T_GW (3x6)", "Jm2", a.jm[2], b.jm[2], 27 * 8);
      for (const char* s : {"async residual", "async error()", "async jacobian T_WS(tk) (3x7)", "async jacobian speed/bias (3x9)",
                            "async jacobian T_GW (3x7)", "async minimal T_WS(tk) (3x6)", "async minimal speed/bias (3x9)",
                            "async minimal T_GW (3x6)"})
        g_stats[s].cases++;
      if (call > 0 && ca.imu.redo_counter != prev_counter) g_cov["async call with re-preintegration (not first)"]++;
      if (call > 0 && ca.imu.redo_counter == prev_counter && ca.imu.redo && ca.imu.n_meas >= 50) g_cov["async call on a stale preintegration (redo pending, n >= 50)"]++;
      prev_counter = ca.imu.redo_counter;
      if (r.p(0.25)) apply("async applyPreInt (after evaluate)");
    }
    g_stats["async ctor"].cases++;
    ok_gps_async_free(&ca);
  }
  okvis::ceres::GpsErrorAsynchronous::useImuCovariance = true;
  okvis::ceres::GpsErrorAsynchronous::redoPropagationAlways = false;
}

}  // namespace

int main(int argc, char** argv) {
  int cases = 20000;
  std::vector<uint64_t> seeds = {1, 2, 3, 4};
  if (argc > 1) cases = std::atoi(argv[1]);
  if (argc > 2) { seeds.clear(); for (int i = 2; i < argc; ++i) seeds.push_back(std::strtoull(argv[i], nullptr, 10)); }
  g_verbose = std::getenv("OKGPS_VERBOSE") != nullptr;
  for (uint64_t seed : seeds) {
    Rng r(seed);
    test_manifold(r, cases);
    test_sync(r, cases);
    test_async(r, cases);
    std::printf("seed %llu done\n", (unsigned long long)seed);
    std::fflush(stdout);
  }
  uint64_t total_m = 0, total_b = 0;
  std::printf("%-44s %10s %14s %10s\n", "section", "cases", "bytes", "mismatches");
  for (const auto& kv : g_stats) {
    std::printf("%-44s %10llu %14llu %10llu\n", kv.first.c_str(), (unsigned long long)kv.second.cases,
                (unsigned long long)kv.second.bytes, (unsigned long long)kv.second.mism);
    total_m += kv.second.mism;
    total_b += kv.second.bytes;
  }
  std::printf("coverage:\n");
  for (const auto& kv : g_cov)
    if (kv.first.compare(0, 2, "__") != 0) std::printf("  %-64s %10llu\n", kv.first.c_str(), (unsigned long long)kv.second);
  std::printf("TOTAL bytes %llu mismatching comparisons %llu\n", (unsigned long long)total_b, (unsigned long long)total_m);
  return total_m == 0 ? 0 : 1;
}

#endif  // __has_include
