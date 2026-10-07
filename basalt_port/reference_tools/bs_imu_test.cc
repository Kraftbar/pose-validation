// Oracle for basalt_port/c/bs_imu.{h,c}: the real basalt-headers IntegratedImuMeasurement<float> (+ ImuBlock::linearizeImu) vs the C port,
// memcmp (tolerance 0). Reference flags: g++ -std=c++17 -O2 -DNDEBUG -ffp-contract=off -fno-fast-math (no -march), Eigen 3.4.0.
//   bs_imu_test <seed> <cases> [sens]          random + realistic IMU sequences (200 Hz, EuRoC magnitudes), biases, segment lengths
//   bs_imu_test replay <imu.bin>               M0 dump IMU_PREINT / IMU_PREDICT records: C port vs the dumped reference output, plus
//                                              real class vs C on the same inputs for the quantities the dump does not carry
//   bs_imu_test prim <seed> <cases>            Eigen primitives: 9x9 / 9x3 GEBP shapes, LDLT + triangular solve, gemv, 9-vector squaredNorm
// ODR hazard (PLAN.md 4d): preintegration.h calls an unqualified sqrt() in compute_sqrt_cov_inv<float>. It resolves to the float overload only if
// `using std::sqrt` in namespace basalt (double_sphere_camera.hpp) is visible at the definition, as in the linearisation translation units of
// the reference (the copy the linker keeps). Include it first and check the disassembly (sqrtss, no cvtss2sd) in tools/check_basalt_port.py.
#include <basalt/camera/double_sphere_camera.hpp>
#include <basalt/imu/preintegration.h>
#include <basalt/linearization/imu_block.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>

extern "C" {
#include "bs_imu.h"
}

using basalt::IntegratedImuMeasurement;
typedef float S;
typedef Eigen::Matrix<S, 3, 1> V3;
typedef Eigen::Matrix<S, 9, 9> M99;
typedef Eigen::Matrix<S, 9, 3> M93;
typedef Eigen::Matrix<S, 9, 1> V9;
typedef Sophus::SO3<S> SO3f;
typedef Sophus::SE3<S> SE3f;
typedef std::mt19937_64 Rng;

static double urand(Rng& r, double a, double b) { return a + (b - a) * std::uniform_real_distribution<double>(0, 1)(r); }
static double lurand(Rng& r, double a, double b) { return std::exp(urand(r, std::log(a), std::log(b))); }
static double nrand(Rng& r) { return std::normal_distribution<double>(0, 1)(r); }

#ifdef BS_IMU_COV
extern "C" unsigned long bs_imu_cov[16];
static void print_cov() {
  static const char* nm[] = {"exp small-angle", "exp regular", "log small", "log regular", "log w<0", "Jr regular", "Jr/Jl-inv-left small (<=eps)", "JInv regular", "JInv near pi",
                             "JInv small", "ldlt pivot swap", "D < FLT_MIN", "ldlt zero diagonal", "", "", ""};
  std::printf("  branch coverage:");
  for (int i = 0; i < 13; ++i) std::printf(" [%s %lu]", nm[i], bs_imu_cov[i]);
  std::printf("\n");
}
#else
static void print_cov() {}
#endif
// ---------------------------------------------------------------- bookkeeping
static std::map<std::string, long> g_cmp, g_bad;
static int g_verbose = 1;
static int g_dbg_cov = 0;
static bool eq(const void* a, const void* b, size_t n) { return std::memcmp(a, b, n) == 0; }
static void check(const char* name, const void* a, const void* b, size_t nbytes, long seed_case) {
  ++g_cmp[name];
  if (!eq(a, b, nbytes)) {
    if (++g_bad[name] <= 2 && g_verbose) {
      const float* fa = (const float*)a; const float* fb = (const float*)b;
      size_t i = 0; while (i < nbytes / 4 && std::memcmp(fa + i, fb + i, 4) == 0) ++i;
      std::printf("  MISMATCH %s case %ld first diff element %zu: cxx %.9g (0x%08x) vs c %.9g (0x%08x)\n", name, seed_case, i, fa[i],
                  *(const uint32_t*)(fa + i), fb[i], *(const uint32_t*)(fb + i));
    }
  }
}
static void report(const char* title) {
  long tc = 0, tb = 0;
  for (auto& kv : g_cmp) { tc += kv.second; tb += g_bad[kv.first]; std::printf("  %-22s %9ld compared, %ld mismatches\n", kv.first.c_str(), kv.second, g_bad[kv.first]); }
  std::printf("%s: %ld/%ld\n", title, tb, tc);
}

// ---------------------------------------------------------------- conversions C++ <-> C
static SO3f so3_from_raw(const float q[4]) {
  SO3f s;
  s = Eigen::Map<const SO3f>(q);  // raw copy of the unit quaternion (x y z w), no normalisation
  return s;
}
static void raw_from_so3(const SO3f& s, float q[4]) { std::memcpy(q, s.unit_quaternion().coeffs().data(), 16); }

static basalt::PoseVelState<S> to_cxx(const bs_pvstate& c) {
  basalt::PoseVelState<S> s;
  s.t_ns = c.t_ns;
  s.T_w_i.so3() = so3_from_raw(c.q);
  s.T_w_i.translation() = V3(c.p[0], c.p[1], c.p[2]);
  s.vel_w_i = V3(c.v[0], c.v[1], c.v[2]);
  return s;
}
static bs_pvstate to_c(const basalt::PoseVelState<S>& s) {
  bs_pvstate c;
  c.t_ns = s.t_ns;
  raw_from_so3(s.T_w_i.so3(), c.q);
  std::memcpy(c.p, s.T_w_i.translation().data(), 12);
  std::memcpy(c.v, s.vel_w_i.data(), 12);
  return c;
}
static basalt::PoseVelBiasState<S> to_cxx(const bs_pvbstate& c) {
  basalt::PoseVelBiasState<S> s;
  static_cast<basalt::PoseVelState<S>&>(s) = to_cxx(c.s);
  s.bias_gyro = V3(c.bg[0], c.bg[1], c.bg[2]);
  s.bias_accel = V3(c.ba[0], c.ba[1], c.ba[2]);
  return s;
}
static bs_pvbstate to_c(const basalt::PoseVelBiasState<S>& s) {
  bs_pvbstate c;
  c.s = to_c(static_cast<const basalt::PoseVelState<S>&>(s));
  std::memcpy(c.bg, s.bias_gyro.data(), 12);
  std::memcpy(c.ba, s.bias_accel.data(), 12);
  return c;
}

static void compare_meas(const char* tag, const IntegratedImuMeasurement<S>& a, const bs_imu_meas& b, long cs) {
  std::string t(tag);
  const auto& d = a.getDeltaState();
  bs_pvstate dc = to_c(d);
  check((t + ".delta_t").c_str(), &dc.t_ns, &b.delta.t_ns, 8, cs);
  check((t + ".delta_q").c_str(), dc.q, b.delta.q, 16, cs);
  check((t + ".delta_p").c_str(), dc.p, b.delta.p, 12, cs);
  check((t + ".delta_v").c_str(), dc.v, b.delta.v, 12, cs);
  check((t + ".cov").c_str(), a.get_cov().data(), b.cov, 324, cs);
  check((t + ".d_state_d_ba").c_str(), a.get_d_state_d_ba().data(), b.d_state_d_ba, 108, cs);
  check((t + ".d_state_d_bg").c_str(), a.get_d_state_d_bg().data(), b.d_state_d_bg, 108, cs);
}

// ---------------------------------------------------------------- random generators
struct Scenario {
  V3 bg_lin, ba_lin, acc_cov, gyr_cov;
  int64_t t0;
  std::vector<basalt::ImuData<S>> samples;  // already calibrated, increasing t
  int64_t tail_t;                           // frame time of the tail sample (may be == last sample t)
};

static Scenario gen_scenario(Rng& r) {
  Scenario sc;
  const int mode = (int)(r() % 4);
  const double gscale = mode == 0 ? 0.05 : mode == 1 ? 0.5 : mode == 2 ? 2.0 : 0.002;   // gyro rad/s (rest .. aggressive)
  const double ascale = mode == 0 ? 1.0 : mode == 1 ? 3.0 : mode == 2 ? 8.0 : 0.05;     // accel deviation from gravity
  for (int i = 0; i < 3; ++i) {
    sc.bg_lin[i] = (S)urand(r, -0.06, 0.06);
    sc.ba_lin[i] = (S)urand(r, -0.4, 0.4);
  }
  const double sa = lurand(r, 0.0015, 0.05), sg = lurand(r, 0.0001, 0.005);   // discrete-time noise std (EuRoC: ~0.04, ~0.002)
  for (int i = 0; i < 3; ++i) { sc.acc_cov[i] = (S)(sa * sa * urand(r, 0.5, 2)); sc.gyr_cov[i] = (S)(sg * sg * urand(r, 0.5, 2)); }
  if (r() % 16 == 0) { sc.acc_cov.setConstant((S)1e-3); sc.gyr_cov.setConstant((S)1e-6); }
  if (r() % 32 == 0) { for (int i = 0; i < 3; ++i) { sc.acc_cov[i] = (S)lurand(r, 1e-8, 1.0); sc.gyr_cov[i] = (S)lurand(r, 1e-12, 1e-2); } }
  sc.t0 = 1403636579763555584LL + (int64_t)(r() % 1000000000) * 1000;
  int n = (int)(r() % 4 == 0 ? 1 + r() % 4 : 5 + r() % 12);     // EuRoC: 200 Hz IMU at 20 Hz frames -> ~10 per segment
  if (r() % 40 == 0) n = 30 + (int)(r() % 60);
  int64_t t = sc.t0;
  Eigen::Matrix<double, 3, 1> g_w(0, 0, 9.81);
  double ax = urand(r, -1, 1), ay = urand(r, -1, 1), az = urand(r, -1, 1);
  for (int i = 0; i < n; ++i) {
    int64_t step = 5000000 + (int64_t)(r() % 2000001) - 1000000;   // 200 Hz with jitter
    if (r() % 32 == 0) step = 1 + (int64_t)(r() % 5000000);
    t += step;
    basalt::ImuData<S> d;
    d.t_ns = t;
    ax += urand(r, -0.3, 0.3); ay += urand(r, -0.3, 0.3); az += urand(r, -0.3, 0.3);
    for (int k = 0; k < 3; ++k) {
      d.gyro[k] = (S)(gscale * (r() % 8 == 0 ? nrand(r) : std::sin(0.7 * i + k) * 0.7 + nrand(r) * 0.2));
      d.accel[k] = (S)(ascale * (k == 0 ? ax : k == 1 ? ay : az) * 0.5 + (k == 2 ? 9.81 : 0) + nrand(r) * 0.05);
    }
    if (r() % 64 == 0) for (int k = 0; k < 3; ++k) d.gyro[k] = 0;
    sc.samples.push_back(d);
  }
  sc.tail_t = sc.samples.back().t_ns + (int64_t)(r() % 5000000);   // tail sample re-stamped to the frame time (shorter dt)
  if (r() % 4 == 0) sc.tail_t = sc.samples.back().t_ns;
  return sc;
}

static void rand_state(Rng& r, basalt::PoseVelBiasState<S>& s, bool raw_exp) {
  V3 w;
  const double ang = r() % 4 == 0 ? urand(r, 0, M_PI - 1e-3) : lurand(r, 1e-4, 3.0);
  Eigen::Vector3d ax(nrand(r), nrand(r), nrand(r));
  ax.normalize();
  w = (ax * ang).cast<S>();
  SO3f so = SO3f::exp(w);
  if (!raw_exp) so = SO3f(so.unit_quaternion());   // normalised copy
  s.T_w_i.so3() = so;
  for (int k = 0; k < 3; ++k) {
    s.T_w_i.translation()[k] = (S)urand(r, -15, 15);
    s.vel_w_i[k] = (S)urand(r, -4, 4);
    s.bias_gyro[k] = (S)urand(r, -0.06, 0.06);
    s.bias_accel[k] = (S)urand(r, -0.4, 0.4);
  }
}

// ImuBlock with exposed Jp / r
template <class Sc>
struct ImuBlockX : basalt::ImuBlock<Sc> {
  using basalt::ImuBlock<Sc>::ImuBlock;
  const Eigen::Matrix<Sc, -1, -1>& getJp() const { return this->Jp; }
  const Eigen::Matrix<Sc, -1, 1>& getR() const { return this->r; }
};

// one full pass over a scenario: integrate (every step compared), sqrt_cov_inv, predict, residual (+Jacobians), linearize
static void run_scenario(Rng& r, const Scenario& sc, long cs, int sens, bool cmp_steps) {
  IntegratedImuMeasurement<S> a(sc.t0, sc.bg_lin, sc.ba_lin);
  bs_imu_meas b;
  bs_imu_init(&b, sc.t0, sc.bg_lin.data(), sc.ba_lin.data());
  {
    IntegratedImuMeasurement<S> a0(sc.t0, sc.bg_lin, sc.ba_lin);
    compare_meas("init", a0, b, cs);
  }
  for (size_t i = 0; i < sc.samples.size(); ++i) {
    basalt::ImuData<S> d = sc.samples[i];
    if (i + 1 == sc.samples.size()) d.t_ns = sc.tail_t;
    bs_imudata cd;
    cd.t_ns = d.t_ns;
    std::memcpy(cd.accel, d.accel.data(), 12);
    std::memcpy(cd.gyro, d.gyro.data(), 12);
    if (sens && i == 0) cd.accel[0] = std::nextafter(cd.accel[0], 1e9f);
    if (g_dbg_cov) {  // replicate integrate()'s covariance statement with real Eigen on the real F, A, G, to bisect model errors
      basalt::ImuData<S> dcor = d; dcor.t_ns -= sc.t0; dcor.accel -= sc.ba_lin; dcor.gyro -= sc.bg_lin;
      basalt::PoseVelState<S> ns; M99 F; M93 A, G;
      IntegratedImuMeasurement<S>::propagateState(a.getDeltaState(), dcor, ns, &F, &A, &G);
      M99 covold = a.get_cov();
      M99 want = F * covold * F.transpose() + A * sc.acc_cov.asDiagonal() * A.transpose() + G * sc.gyr_cov.asDiagonal() * G.transpose();
      IntegratedImuMeasurement<S> a_copy = a;
      a_copy.integrate(d, sc.acc_cov, sc.gyr_cov);
      check("dbg.cov_statement_vs_class", want.data(), a_copy.get_cov().data(), 324, cs);
      bs_imu_meas bcopy = b;
      bs_imu_integrate(&bcopy, &cd, sc.acc_cov.data(), sc.gyr_cov.data());
      check("dbg.cov_statement_vs_c", want.data(), bcopy.cov, 324, cs);
      float cF[81], cA[27], cG[27]; bs_pvstate cn; bs_pvstate cdelta = to_c(a.getDeltaState()); bs_imudata cdd; cdd.t_ns = dcor.t_ns; std::memcpy(cdd.accel, dcor.accel.data(), 12); std::memcpy(cdd.gyro, dcor.gyro.data(), 12);
      bs_imu_propagate_state(&cdelta, &cdd, &cn, cF, cA, cG);
      check("dbg.F", F.data(), cF, 324, cs); check("dbg.A", A.data(), cA, 108, cs); check("dbg.G", G.data(), cG, 108, cs);
    }
    a.integrate(d, sc.acc_cov, sc.gyr_cov);
    bs_imu_integrate(&b, &cd, sc.acc_cov.data(), sc.gyr_cov.data());
    if (cmp_steps || i + 1 == sc.samples.size()) compare_meas("integrate", a, b, cs);
  }
  if (cs % 256 == 0) {   // lazy sqrt_cov_inv on a fresh measurement: all-zero covariance (pivot invalid at k = 0)
    IntegratedImuMeasurement<S> z(sc.t0, sc.bg_lin, sc.ba_lin);
    bs_imu_meas bz; bs_imu_init(&bz, sc.t0, sc.bg_lin.data(), sc.ba_lin.data());
    check("sqrt_cov_inv.zero_cov", z.get_sqrt_cov_inv().data(), bs_imu_sqrt_cov_inv(&bz), 324, cs);
  }
  // sqrt_cov_inv (lazy; the C++ copy is the float-sqrt one, see PLAN.md 4d)
  {
    const M99& sa = a.get_sqrt_cov_inv();
    const float* sb = bs_imu_sqrt_cov_inv(&b);
    check("sqrt_cov_inv", sa.data(), sb, 324, cs);
  }
  // predictState
  bool exact_pred = false;
  basalt::PoseVelBiasState<S> st0, st1;
  rand_state(r, st0, r() % 2);
  V3 g = (r() % 2) ? V3(0, 0, (S)-9.81) : V3((S)urand(r, -1, 1), (S)urand(r, -1, 1), (S)urand(r, -10, -9));
  {
    basalt::PoseVelBiasState<S> o = st0;
    o.t_ns = 12345;
    a.predictState(st0, g, o);
    bs_pvbstate c0 = to_c(st0), c1 = c0;
    c1.s.t_ns = 12345;
    bs_imu_predict_state(&b, &c0.s, g.data(), &c1.s);
    bs_pvbstate oc = to_c(o);
    check("predict.q", oc.s.q, c1.s.q, 16, cs);
    check("predict.p", oc.s.p, c1.s.p, 12, cs);
    check("predict.v", oc.s.v, c1.s.v, 12, cs);
    check("predict.rest", &oc.s.t_ns, &c1.s.t_ns, 8, cs);
    check("predict.bias", oc.bg, c1.bg, 24, cs);
    // end state close to the prediction (the linearisation regime) or arbitrary
    st1 = o;
    if (r() % 5) {
      V3 dw((S)(nrand(r) * lurand(r, 1e-4, 0.1)), (S)(nrand(r) * lurand(r, 1e-4, 0.1)), (S)(nrand(r) * lurand(r, 1e-4, 0.1)));
      st1.T_w_i.so3() = SO3f::exp(dw) * st1.T_w_i.so3();
      for (int k = 0; k < 3; ++k) { st1.T_w_i.translation()[k] += (S)(nrand(r) * lurand(r, 1e-4, 0.2)); st1.vel_w_i[k] += (S)(nrand(r) * lurand(r, 1e-4, 0.2)); }
    } else {
      rand_state(r, st1, r() % 2);
    }
    st1.t_ns = st0.t_ns + a.get_dt_ns();
    st1.bias_gyro = st0.bias_gyro + V3((S)(nrand(r) * 1e-3), (S)(nrand(r) * 1e-3), (S)(nrand(r) * 1e-3));
    st1.bias_accel = st0.bias_accel + V3((S)(nrand(r) * 1e-2), (S)(nrand(r) * 1e-2), (S)(nrand(r) * 1e-2));
    if (r() % 8 == 0) { st1 = o; st1.t_ns = st0.t_ns + a.get_dt_ns(); exact_pred = true; }   // zero rotation residual: the log small-angle branch
  }
  // residual: current biases around the linearisation biases (as in the estimator) and arbitrary ones; with / without Jacobians
  for (int rep = 0; rep < 2; ++rep) {
    V3 cbg0 = sc.bg_lin; V3 cba0 = sc.ba_lin;
    V3 cbg = sc.bg_lin + V3((S)(nrand(r) * (rep ? 0.05 : 1e-3)), (S)(nrand(r) * (rep ? 0.05 : 1e-3)), (S)(nrand(r) * (rep ? 0.05 : 1e-3)));
    V3 cba = sc.ba_lin + V3((S)(nrand(r) * (rep ? 0.3 : 1e-2)), (S)(nrand(r) * (rep ? 0.3 : 1e-2)), (S)(nrand(r) * (rep ? 0.3 : 1e-2)));
    if (exact_pred && rep == 0) { cbg = cbg0; cba = cba0; }
    M99 d0, d1;
    M93 dbg, dba;
    V9 res = a.residual(st0, g, st1, cbg, cba, &d0, &d1, &dbg, &dba);
    V9 resn = a.residual(st0, g, st1, cbg, cba);
    bs_pvbstate c0 = to_c(st0), c1 = to_c(st1);
    float cres[9], cresn[9], cd0[81], cd1[81], cdbg[27], cdba[27];
    bs_imu_residual(&b, &c0.s, g.data(), &c1.s, cbg.data(), cba.data(), cres, cd0, cd1, cdbg, cdba);
    bs_imu_residual(&b, &c0.s, g.data(), &c1.s, cbg.data(), cba.data(), cresn, NULL, NULL, NULL, NULL);
    if (g_verbose && !eq(res.data(), cres, 36) && g_bad["residual"] < 1) {
      S dtv = a.get_dt_ns() * S(1e-9);
      Eigen::Matrix3f R0e = st0.T_w_i.so3().inverse().matrix();
      V3 xe = st1.vel_w_i - st0.vel_w_i - g * dtv;
      V3 tmp2e = R0e * xe;
      float x3[3] = {xe[0], xe[1], xe[2]}, mine[3];
      for (int i = 0; i < 3; ++i) mine[i] = R0e(i, 0) * x3[0] + (R0e(i, 1) * x3[1] + R0e(i, 2) * x3[2]);
      std::printf("  dbg tmp2 eigen %.9g %.9g %.9g  tree %.9g %.9g %.9g ; x %.9g %.9g %.9g R0 row2 %.9g %.9g %.9g\n", tmp2e[0], tmp2e[1], tmp2e[2], mine[0], mine[1], mine[2], xe[0], xe[1], xe[2], R0e(2,0), R0e(2,1), R0e(2,2));
      std::printf("  dbg res %.9g %.9g %.9g | c %.9g %.9g %.9g\n", res[6], res[7], res[8], cres[6], cres[7], cres[8]);
      {
        const M93& D = a.get_d_state_d_ba(); V3 dd = cba - sc.ba_lin; float c0 = 0; for (int j = 0; j < 3; ++j) c0 = c0 + D(8, j) * dd[j];
        const M93& Dg = a.get_d_state_d_bg(); V3 dg = cbg - sc.bg_lin; float g0 = 0; for (int j = 0; j < 3; ++j) g0 = g0 + Dg(8, j) * dg[j];
        V9 bgdiff = Dg * dg; V9 badiff = D * dd;
        float dvv = a.getDeltaState().vel_w_i[2];
        float s1 = (dvv + g0) + c0, s2 = dvv + (g0 + c0);
        std::printf("  dbg model bad %.9g vs eigen %.9g ; bgd %.9g vs %.9g ; (dv+bg)+ba %.9g  dv+(bg+ba) %.9g ; res.. %.9g %.9g\n", c0, badiff[8], g0, bgdiff[8], s1, s2, tmp2e[2] - s1, tmp2e[2] - s2);
      }
      std::printf("  dbg dv %.9g bgd %.9g bad %.9g\n", a.getDeltaState().vel_w_i[2], (a.get_d_state_d_bg() * (cbg - sc.bg_lin))[8], (a.get_d_state_d_ba() * (cba - sc.ba_lin))[8]);
    }
    check("residual", res.data(), cres, 36, cs);
    check("residual.nojac", resn.data(), cresn, 36, cs);
    check("res.d_state0", d0.data(), cd0, 324, cs);
    check("res.d_state1", d1.data(), cd1, 324, cs);
    check("res.d_bg", dbg.data(), cdbg, 108, cs);
    check("res.d_ba", dba.data(), cdba, 108, cs);
    // partial Jacobian requests (calls with only some pointers)
    M99 e0; V9 resp = a.residual(st0, g, st1, cbg, cba, &e0, nullptr, nullptr, nullptr);
    float cp0[81], cpres[9];
    bs_imu_residual(&b, &c0.s, g.data(), &c1.s, cbg.data(), cba.data(), cpres, cp0, NULL, NULL, NULL);
    check("residual.only_d0", resp.data(), cpres, 36, cs);
    check("res.only_d0", e0.data(), cp0, 324, cs);
  }
  // linearizeImu through the real ImuBlock
  {
    basalt::PoseVelBiasStateWithLin<S> ws(st0.t_ns, st0.T_w_i, st0.vel_w_i, st0.bias_gyro, st0.bias_accel, (bool)(r() % 2));
    basalt::PoseVelBiasStateWithLin<S> we(st1.t_ns, st1.T_w_i, st1.vel_w_i, st1.bias_gyro, st1.bias_accel, (bool)(r() % 2));
    Eigen::Matrix<S, 15, 1> inc;
    for (int k = 0; k < 15; ++k) inc[k] = (S)(nrand(r) * (k < 6 ? 1e-2 : k < 9 ? 5e-2 : k < 12 ? 1e-3 : 1e-2));
    // a linearised state carries a current state = lin + delta (applyInc on a linearised state accumulates delta)
    basalt::PoseVelBiasStateWithLin<S> ws2 = ws, we2 = we;
    if (r() % 2) { ws2 = basalt::PoseVelBiasStateWithLin<S>(st0.t_ns, st0.T_w_i, st0.vel_w_i, st0.bias_gyro, st0.bias_accel, true); ws2.applyInc(inc); }
    for (int k = 0; k < 15; ++k) inc[k] = (S)(nrand(r) * (k < 6 ? 1e-2 : k < 9 ? 5e-2 : k < 12 ? 1e-3 : 1e-2));
    if (r() % 2) { we2 = basalt::PoseVelBiasStateWithLin<S>(st1.t_ns, st1.T_w_i, st1.vel_w_i, st1.bias_gyro, st1.bias_accel, true); we2.applyInc(inc); }
    basalt::AbsOrderMap aom;
    aom.abs_order_map[st0.t_ns] = std::make_pair(0, 15);
    aom.abs_order_map[st1.t_ns] = std::make_pair(15, 15);
    aom.items = 2; aom.total_size = 30;
    V3 gyro_w((S)(1.0 / lurand(r, 1e-4, 1e-2)), (S)(1.0 / lurand(r, 1e-4, 1e-2)), (S)(1.0 / lurand(r, 1e-4, 1e-2)));
    V3 accel_w((S)(1.0 / lurand(r, 1e-3, 1e-1)), (S)(1.0 / lurand(r, 1e-3, 1e-1)), (S)(1.0 / lurand(r, 1e-3, 1e-1)));
    basalt::ImuLinData<S> ild{g, gyro_w, accel_w, {}};
    // the state the block reads must have t_ns = start / start + dt; make the block's measurement start match the states
    IntegratedImuMeasurement<S> a2(st0.t_ns, sc.bg_lin, sc.ba_lin);
    bs_imu_meas b2;
    bs_imu_init(&b2, st0.t_ns, sc.bg_lin.data(), sc.ba_lin.data());
    for (size_t i = 0; i < sc.samples.size(); ++i) {
      basalt::ImuData<S> d = sc.samples[i];
      d.t_ns = st0.t_ns + (sc.samples[i].t_ns - sc.t0);
      if (i + 1 == sc.samples.size()) d.t_ns = st0.t_ns + (sc.tail_t - sc.t0);
      bs_imudata cd; cd.t_ns = d.t_ns; std::memcpy(cd.accel, d.accel.data(), 12); std::memcpy(cd.gyro, d.gyro.data(), 12);
      a2.integrate(d, sc.acc_cov, sc.gyr_cov);
      bs_imu_integrate(&b2, &cd, sc.acc_cov.data(), sc.gyr_cov.data());
    }
    st1.t_ns = st0.t_ns + a2.get_dt_ns();
    we2 = basalt::PoseVelBiasStateWithLin<S>(st1.t_ns, st1.T_w_i, st1.vel_w_i, st1.bias_gyro, st1.bias_accel, we2.isLinearized());
    if (we2.isLinearized()) we2.applyInc(inc);
    if (ws2.isLinearized() && ws2.getT_ns() != st0.t_ns) std::abort();
    aom.abs_order_map.clear();
    aom.abs_order_map[st0.t_ns] = std::make_pair(0, 15);
    aom.abs_order_map[st1.t_ns] = std::make_pair(15, 15);
    Eigen::aligned_map<int64_t, basalt::PoseVelBiasStateWithLin<S>> fs;
    fs[st0.t_ns] = ws2;
    fs[st1.t_ns] = we2;
    ImuBlockX<S> blk(&a2, &ild, aom);
    S err = blk.linearizeImu(fs);
    bs_pvb_with_lin cs0, cs1;
    cs0.linearized = ws2.isLinearized(); cs0.lin = to_c(ws2.getStateLin()); cs0.cur = to_c(ws2.getState());
    cs1.linearized = we2.isLinearized(); cs1.lin = to_c(we2.getStateLin()); cs1.cur = to_c(we2.getState());
    static float cJp[450], cr[15];
    float cerr = bs_imu_linearize(&b2, g.data(), gyro_w.data(), accel_w.data(), &cs0, &cs1, cJp, cr);
    check("linearize.Jp", blk.getJp().data(), cJp, 1800, cs);
    check("linearize.r", blk.getR().data(), cr, 60, cs);
    check("linearize.error", &err, &cerr, 4, cs);
    // the second use of the cached sqrt_cov_inv (as in the estimator, which linearises many times per integration)
    float cerr2 = bs_imu_linearize(&b2, g.data(), gyro_w.data(), accel_w.data(), &cs0, &cs1, cJp, cr);
    S err2 = blk.linearizeImu(fs);
    check("linearize.error2", &err2, &cerr2, 4, cs);
  }
}

// ---------------------------------------------------------------- M0 dump replay
struct Rd {
  const unsigned char* p;
  size_t o = 0;
  template <class T> T get() { T v; std::memcpy(&v, p + o, sizeof(T)); o += sizeof(T); return v; }
};

static int replay(const char* path) {
  FILE* f = std::fopen(path, "rb");
  if (!f) { std::perror(path); return 2; }
  uint32_t tag; uint64_t len;
  long npreint = 0, npredict = 0, nsamples = 0;
  bs_imu_meas last; bool have_last = false; int64_t last_start = 0;
  IntegratedImuMeasurement<S>* lastx = nullptr;
  std::vector<unsigned char> buf;
  while (std::fread(&tag, 4, 1, f) == 1 && std::fread(&len, 8, 1, f) == 1) {
    buf.resize(len);
    if (len && std::fread(buf.data(), 1, len, f) != len) { std::fprintf(stderr, "truncated\n"); return 2; }
    Rd rd{buf.data()};
    if (tag == 2) {  // IMU_PREINT
      int64_t t_start = rd.get<int64_t>(), t_end = rd.get<int64_t>();
      float bg[3], ba[3], acov[3], gcov[3];
      for (int i = 0; i < 3; ++i) bg[i] = rd.get<float>();
      for (int i = 0; i < 3; ++i) ba[i] = rd.get<float>();
      for (int i = 0; i < 3; ++i) acov[i] = rd.get<float>();
      for (int i = 0; i < 3; ++i) gcov[i] = rd.get<float>();
      uint32_t ns = rd.get<uint32_t>();
      bs_imu_meas b;
      bs_imu_init(&b, t_start, bg, ba);
      delete lastx;
      lastx = new IntegratedImuMeasurement<S>(t_start, V3(bg[0], bg[1], bg[2]), V3(ba[0], ba[1], ba[2]));
      for (uint32_t i = 0; i < ns; ++i) {
        bs_imudata d; d.t_ns = rd.get<int64_t>();
        for (int k = 0; k < 3; ++k) d.accel[k] = rd.get<float>();
        for (int k = 0; k < 3; ++k) d.gyro[k] = rd.get<float>();
        bs_imu_integrate(&b, &d, acov, gcov);
        basalt::ImuData<S> dx; dx.t_ns = d.t_ns; dx.accel = V3(d.accel[0], d.accel[1], d.accel[2]); dx.gyro = V3(d.gyro[0], d.gyro[1], d.gyro[2]);
        lastx->integrate(dx, V3(acov[0], acov[1], acov[2]), V3(gcov[0], gcov[1], gcov[2]));
        ++nsamples;
      }
      int64_t dt_ns = rd.get<int64_t>();
      float dq[4], dp[3], dv[3], cov[81], dba[27], dbg[27];
      for (int i = 0; i < 4; ++i) dq[i] = rd.get<float>();
      for (int i = 0; i < 3; ++i) dp[i] = rd.get<float>();
      for (int i = 0; i < 3; ++i) dv[i] = rd.get<float>();
      for (int i = 0; i < 81; ++i) cov[i] = rd.get<float>();
      for (int i = 0; i < 27; ++i) dba[i] = rd.get<float>();
      for (int i = 0; i < 27; ++i) dbg[i] = rd.get<float>();
      check("dump.dt_ns", &dt_ns, &b.delta.t_ns, 8, npreint);
      check("dump.delta_q", dq, b.delta.q, 16, npreint);
      check("dump.delta_p", dp, b.delta.p, 12, npreint);
      check("dump.delta_v", dv, b.delta.v, 12, npreint);
      check("dump.cov", cov, b.cov, 324, npreint);
      check("dump.d_state_d_ba", dba, b.d_state_d_ba, 108, npreint);
      check("dump.d_state_d_bg", dbg, b.d_state_d_bg, 108, npreint);
      compare_meas("replay.real_vs_c", *lastx, b, npreint);   // the real class on the same samples (also the sqrt_cov_inv below)
      {
        const M99& sa = lastx->get_sqrt_cov_inv();
        const float* sb = bs_imu_sqrt_cov_inv(&b);
        check("replay.sqrt_cov_inv", sa.data(), sb, 324, npreint);
      }
      (void)t_end;
      last = b; have_last = true; last_start = t_start;
      ++npreint;
    } else if (tag == 5) {  // IMU_PREDICT
      int64_t t0 = rd.get<int64_t>(), t1 = rd.get<int64_t>();
      float g[3], s0[16], s1[16];
      for (int i = 0; i < 3; ++i) g[i] = rd.get<float>();
      for (int i = 0; i < 16; ++i) s0[i] = rd.get<float>();
      for (int i = 0; i < 16; ++i) s1[i] = rd.get<float>();
      if (!have_last || t0 != last_start) { ++npredict; continue; }   // sampled dumps: the matching PREINT may be missing
      bs_pvbstate c0, c1;
      std::memset(&c0, 0, sizeof c0);
      std::memcpy(c0.s.q, s0, 16); std::memcpy(c0.s.p, s0 + 4, 12); std::memcpy(c0.s.v, s0 + 7, 12); std::memcpy(c0.bg, s0 + 10, 12); std::memcpy(c0.ba, s0 + 13, 12);
      c0.s.t_ns = t0;
      c1 = c0;   // next_state starts as a copy of state0 (measure())
      bs_imu_predict_state(&last, &c0.s, g, &c1.s);
      check("dump.predict.q", s1, c1.s.q, 16, npredict);
      check("dump.predict.p", s1 + 4, c1.s.p, 12, npredict);
      check("dump.predict.v", s1 + 7, c1.s.v, 12, npredict);
      check("dump.predict.bg", s1 + 10, c1.bg, 12, npredict);
      check("dump.predict.ba", s1 + 13, c1.ba, 12, npredict);
      // real class + residual on real states: state0 -> state1 with the dumped measurement (the optimiser's start point)
      {
        basalt::PoseVelBiasState<S> x0 = to_cxx(c0), x1 = to_cxx(c1);
        x1.t_ns = t1;
        V3 gg(g[0], g[1], g[2]);
        M99 d0, d1; M93 dbg, dba;
        V3 cbg = x0.bias_gyro, cba = x0.bias_accel;
        V9 res = lastx->residual(x0, gg, x1, cbg, cba, &d0, &d1, &dbg, &dba);
        float cres[9], cd0[81], cd1[81], cdbg[27], cdba[27];
        bs_imu_residual(&last, &c0.s, g, &c1.s, c0.bg, c0.ba, cres, cd0, cd1, cdbg, cdba);
        check("replay.residual", res.data(), cres, 36, npredict);
        check("replay.res.d_state0", d0.data(), cd0, 324, npredict);
        check("replay.res.d_state1", d1.data(), cd1, 324, npredict);
        check("replay.res.d_bg", dbg.data(), cdbg, 108, npredict);
        check("replay.res.d_ba", dba.data(), cdba, 108, npredict);
        // slightly displaced end state (as during the first LM steps)
        basalt::PoseVelBiasState<S> x1b = x1;
        x1b.T_w_i.translation()[0] += 0.0123f; x1b.vel_w_i[1] -= 0.0031f; x1b.T_w_i.so3() = SO3f::exp(V3(1e-3f, -2e-3f, 5e-4f)) * x1b.T_w_i.so3();
        V9 res2 = lastx->residual(x0, gg, x1b, cbg, cba, &d0, &d1, &dbg, &dba);
        bs_pvbstate c1b = to_c(x1b);
        bs_imu_residual(&last, &c0.s, g, &c1b.s, c0.bg, c0.ba, cres, cd0, cd1, cdbg, cdba);
        check("replay.residual.disp", res2.data(), cres, 36, npredict);
        check("replay.res.disp.d_state1", d1.data(), cd1, 324, npredict);
        check("replay.res.disp.d_bg", dbg.data(), cdbg, 108, npredict);
      }
      ++npredict;
    }
  }
  std::fclose(f);
  delete lastx;
  std::printf("replay %s: %ld IMU_PREINT records (%ld integrate calls), %ld IMU_PREDICT records\n", path, npreint, nsamples, npredict);
  report("bs_imu replay");
  long tb = 0;
  for (auto& kv : g_bad) tb += kv.second;
  return tb ? 1 : 0;
}

// ---------------------------------------------------------------- Eigen primitive probes (bisecting aids)
static int prim(uint64_t seed, long n) {
  Rng r(seed);
  for (long it = 0; it < n; ++it) {
    // random 9x9 symmetric matrices: covariance-like (large dynamic range) and plain SPD
    M99 X; for (int i = 0; i < 81; ++i) X.data()[i] = (S)nrand(r);
    M99 sc = M99::Identity(); for (int i = 0; i < 9; ++i) sc(i, i) = (S)lurand(r, 1e-4, 1e2);
    M99 C;
    switch (it % 3) {
      case 0: C = (sc * (X * X.transpose() * (S)0.01f + M99::Identity()) * sc).eval(); break;
      case 1: C = (X * X.transpose()).eval(); break;
      default: { C = (X + X.transpose()).eval(); for (int i = 0; i < 9; ++i) C(i, i) = (S)lurand(r, 0.1, 50); }
    }
    Eigen::LDLT<M99> ldlt(C);
    float cm[81]; int ct[9];
    std::memcpy(cm, C.data(), 324);
    bs_imu_ldlt9(cm, ct);
    // lower triangle (incl. diagonal) of matrixLDLT and the transpositions
    {
      float lo_a[45], lo_c[45]; int q = 0;
      for (int c = 0; c < 9; ++c) for (int rr = c; rr < 9; ++rr) { lo_a[q] = ldlt.matrixLDLT()(rr, c); lo_c[q] = cm[rr + 9 * c]; ++q; }
      check("prim.ldlt_lower", lo_a, lo_c, 180, it);
      int ta[9]; for (int i = 0; i < 9; ++i) ta[i] = ldlt.transpositionsP().indices()[i];
      check("prim.ldlt_transpositions", ta, ct, 36, it);
    }
    // triangular solve of a random right-hand side with the Eigen L (unit lower of matrixLDLT)
    {
      M99 B; for (int i = 0; i < 81; ++i) B.data()[i] = (S)nrand(r);
      M99 want = B;
      ldlt.matrixL().solveInPlace(want);
      float cb[81]; std::memcpy(cb, B.data(), 324);
      bs_imu_trisolve_unit_lower9(ldlt.matrixLDLT().data(), cb);
      check("prim.trisolve_unit_lower", want.data(), cb, 324, it);
    }
    // GEBP shapes of integrate() / linearizeImu(): every product statement form against the real Eigen
    {
      M93 B3; for (int i = 0; i < 27; ++i) B3.data()[i] = (S)nrand(r);
      Eigen::Matrix<S, 3, 9> B3t = B3.transpose();
      float o[81];
      { M99 w = X * C; bs_imu_dbg_gemm(9, 9, 9, X.data(), C.data(), o); check("prim.gemm_9x9x9", w.data(), o, 324, it); }
      { M99 w = X * X.transpose(); M99 Xt = X.transpose(); bs_imu_dbg_gemm(9, 9, 9, X.data(), Xt.data(), o); check("prim.gemm_9x9x9_T", w.data(), o, 324, it); }
      { M93 w = X * B3; bs_imu_dbg_gemm(9, 3, 9, X.data(), B3.data(), o); check("prim.gemm_9x9x3", w.data(), o, 108, it); }
      {  // the statement of integrate(): cov = F*cov*F^T + A*D*A^T + G*D*G^T, with random F (and F structured like propagateState's)
        M99 F = X;
        if (it % 2) { F.setIdentity(); for (int i = 0; i < 3; ++i) F(i, 6 + i) = (S)0.005; Eigen::Matrix3f H; H << 0, (S)-nrand(r), (S)nrand(r), (S)nrand(r), 0, (S)-nrand(r), (S)-nrand(r), (S)nrand(r), 0; F.block<3, 3>(6, 3) = H * 0.005f; F.block<3, 3>(0, 3) = F.block<3, 3>(6, 3) * 0.005f * 0.5f; }
        V3 d3((S)lurand(r, 1e-6, 1e-1), (S)lurand(r, 1e-6, 1e-1), (S)lurand(r, 1e-6, 1e-1));
        M99 cov = C;
        M99 w = F * cov * F.transpose() + B3 * d3.asDiagonal() * B3.transpose() + B3 * d3.asDiagonal() * B3.transpose();
        float t1[81], t2[81], t3[81], t4[81], P2[81]; M93 BD = B3 * d3.asDiagonal(); M99 Ft = F.transpose();
        bs_imu_dbg_gemm(9, 9, 9, F.data(), cov.data(), t1); bs_imu_dbg_gemm(9, 9, 9, t1, Ft.data(), t2);
        bs_imu_dbg_gemm(9, 9, 3, BD.data(), B3t.data(), P2);
        for (int i = 0; i < 81; ++i) t3[i] = (t2[i] + P2[i]) + P2[i];
        (void)t4;
        check(it % 2 ? "prim.cov_expr_structuredF" : "prim.cov_expr_randomF", w.data(), t3, 324, it);
        M99 w1 = F * cov; check("prim.F_times_cov", w1.data(), t1, 324, it);
        M99 w2 = F * cov * F.transpose(); check("prim.F_cov_Ft", w2.data(), t2, 324, it);
      }
      { M99 w = B3 * B3t; bs_imu_dbg_gemm(9, 9, 3, B3.data(), B3t.data(), o); check("prim.gemm_9x3x9", w.data(), o, 324, it); }
    }
  }
  return 0;
}

// diagnostic: per output element, which accumulation model reproduces Eigen for a statement form:
//   L = one left fold over k, N = native GEBP (remainder row / swapped 4-chain), T = the problem solved transposed (res^T = B^T A^T)
template <class MA, class MB, class MG>
static void model_table(const MG& got, const MA& A, const MB& B, long* cnt /* [3][rows*cols] flat: matches per model */) {
  const int rows = (int)A.rows(), cols = (int)B.cols(), depth = (int)A.cols();
  std::vector<float> a(rows * depth), b(depth * cols), bt(cols * depth), at(depth * rows), o1(rows * cols), o2(cols * rows);
  for (int i = 0; i < rows; ++i) for (int k = 0; k < depth; ++k) { a[i + rows * k] = A(i, k); at[k + depth * i] = A(i, k); }
  for (int k = 0; k < depth; ++k) for (int j = 0; j < cols; ++j) { b[k + depth * j] = B(k, j); bt[j + cols * k] = B(k, j); }
  bs_imu_dbg_gemm(rows, cols, depth, a.data(), b.data(), o1.data());
  bs_imu_dbg_gemm(cols, rows, depth, bt.data(), at.data(), o2.data());   // (B^T) * (A^T): element (j,i) = result (i,j)
  for (int j = 0; j < cols; ++j) for (int i = 0; i < rows; ++i) {
    float l = 0; for (int k = 0; k < depth; ++k) l = l + A(i, k) * B(k, j);
    l = 0.0f + 1.0f * l;
    float g = got(i, j), n = o1[i + rows * j], tt = o2[j + cols * i];
    const int e = i + rows * j, sz = rows * cols;
    cnt[0 * sz + e] += std::memcmp(&g, &l, 4) == 0;
    cnt[1 * sz + e] += std::memcmp(&g, &n, 4) == 0;
    cnt[2 * sz + e] += std::memcmp(&g, &tt, 4) == 0;
  }
}
static void print_table(const char* title, int rows, int cols, const std::vector<long>& cnt, long trials) {
  std::printf("%s\n", title);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const int e = i + rows * j, sz = rows * cols;
      bool l = cnt[e] == trials, n = cnt[sz + e] == trials, t = cnt[2 * sz + e] == trials;
      char c = (l && n && t) ? '.' : (l && n) ? 'a' : (l && t) ? 'b' : (n && t) ? 'c' : l ? 'L' : n ? 'N' : t ? 'T' : 'X';
      std::printf("%c", c);
    }
    std::printf("\n");
  }
}
typedef Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> MatX;
static int diag(uint64_t seed, long n) {
  Rng r(seed);
  struct Form { const char* name; int rows, cols; std::vector<long> cnt; };
  std::vector<Form> forms = {
    {"construct W = X*C", 9, 9, {}}, {"assign W = X*C", 9, 9, {}}, {"assign W = X*C*X^T (step 2, A = X*C)", 9, 9, {}},
    {"assign W = X*C*X^T + Y + Z (step 2)", 9, 9, {}}, {"assign W = -B + X*B3 (9x3)", 9, 3, {}}, {"assign W = X*B3 (9x3)", 9, 3, {}},
    {"assign W = (B3*d)*B3^T (depth 3)", 9, 9, {}}, {"Jp.block<9,9> = S*X", 9, 9, {}}, {"Jp.block<9,3> = S*B3", 9, 3, {}},
    {"construct W = X*C*X^T (step 2)", 9, 9, {}}};
  for (auto& f : forms) f.cnt.assign(3 * f.rows * f.cols, 0);
  for (long it = 0; it < n; ++it) {
    M99 X, C, Y; for (int i = 0; i < 81; ++i) { X.data()[i] = (S)nrand(r); C.data()[i] = (S)nrand(r); Y.data()[i] = (S)nrand(r); }
    M93 B3; for (int i = 0; i < 27; ++i) B3.data()[i] = (S)nrand(r);
    V3 d((S)nrand(r), (S)nrand(r), (S)nrand(r));
    M99 T = X * C; M99 Xt = X.transpose(); Eigen::Matrix<S, 3, 9> B3t = B3.transpose();
    { M99 W0 = X * C; model_table(W0, X, C, forms[0].cnt.data()); }
    { M99 W; W = X * C; model_table(W, X, C, forms[1].cnt.data()); }
    { M99 W; W = X * C * X.transpose(); model_table(W, T, Xt, forms[2].cnt.data()); }
    { M99 Z0 = M99::Zero(); M99 W; W = X * C * X.transpose() + Z0 + Z0; model_table(W, T, Xt, forms[3].cnt.data()); }
    { M93 W; W = -B3 + X * B3; M93 P = X * B3; M93 W2 = (-B3 + P); (void)W2; M93 Pn = W + B3;  /* recover P from W */
      // model of the product only: W = (-B3) + P -> P_model; compare the full statement result
      std::vector<float> a(81), b(27), o(27); for (int i = 0; i < 81; ++i) a[i] = X.data()[i]; for (int i = 0; i < 27; ++i) b[i] = B3.data()[i];
      (void)Pn;
      M93 gotp; gotp = W;  // statement result compared against (-B3)+model(P): build per model
      Eigen::Matrix<S, 9, 3> Lm, Nm, Tm; std::vector<float> o1(27), o2(27), bt(27), at(81);
      for (int k = 0; k < 9; ++k) for (int j = 0; j < 3; ++j) bt[j + 3 * k] = B3(k, j);
      for (int i = 0; i < 9; ++i) for (int k = 0; k < 9; ++k) at[k + 9 * i] = X(i, k);
      bs_imu_dbg_gemm(9, 3, 9, a.data(), b.data(), o1.data()); bs_imu_dbg_gemm(3, 9, 9, bt.data(), at.data(), o2.data());
      const int sz = 27;
      for (int j = 0; j < 3; ++j) for (int i = 0; i < 9; ++i) {
        float l = 0; for (int k = 0; k < 9; ++k) l = l + X(i, k) * B3(k, j); l = 0.0f + 1.0f * l;
        float g = W(i, j), nn = (-B3(i, j)) + o1[i + 9 * j], tt = (-B3(i, j)) + o2[j + 3 * i], ll = (-B3(i, j)) + l; const int e = i + 9 * j;
        forms[4].cnt[0 * sz + e] += std::memcmp(&g, &ll, 4) == 0; forms[4].cnt[1 * sz + e] += std::memcmp(&g, &nn, 4) == 0; forms[4].cnt[2 * sz + e] += std::memcmp(&g, &tt, 4) == 0;
      }
    }
    { M93 W; W = X * B3; model_table(W, X, B3, forms[5].cnt.data()); }
    { M99 W; W = B3 * d.asDiagonal() * B3.transpose(); M93 BD = B3 * d.asDiagonal(); model_table(W, BD, B3t, forms[6].cnt.data()); }
    { MatX Jp = MatX::Zero(15, 30); Jp.block<9, 9>(0, 0) = X * C; M99 W = Jp.block<9, 9>(0, 0); model_table(W, X, C, forms[7].cnt.data()); }
    { MatX Jp = MatX::Zero(15, 30); Jp.block<9, 3>(0, 9) = X * B3; M93 W = Jp.block<9, 3>(0, 9); model_table(W, X, B3, forms[8].cnt.data()); }
    { M99 W = X * C * X.transpose(); model_table(W, T, Xt, forms[9].cnt.data()); }
  }
  for (auto& f : forms) print_table((std::string(f.name) + "   [L left fold, N native gebp, T transposed problem; lower-case a: L=N, b: L=T, c: N=T, '.': all equal, X none]").c_str(), f.rows, f.cols, f.cnt, n);
  return 0;
}

int main(int argc, char** argv) {
  if (argc > 3 && std::strcmp(argv[1], "diag") == 0) return diag(strtoull(argv[2], 0, 10), atol(argv[3]));
  if (argc > 2 && std::strcmp(argv[1], "replay") == 0) return replay(argv[2]);
  if (argc > 3 && std::strcmp(argv[1], "prim") == 0) { int rc = prim(strtoull(argv[2], 0, 10), atol(argv[3])); report("bs_imu prim"); return rc; }
  uint64_t seed = argc > 1 ? strtoull(argv[1], 0, 10) : 1;
  long n = argc > 2 ? atol(argv[2]) : 20000;
  int sens = argc > 3 && std::strcmp(argv[3], "sens") == 0;
  g_dbg_cov = argc > 3 && std::strcmp(argv[3], "dbgcov") == 0;
  Rng r(seed);
  for (long it = 0; it < n; ++it) {
    Scenario sc = gen_scenario(r);
    run_scenario(r, sc, it, sens, it % 4 == 0);
  }
  long tb = 0;
  for (auto& kv : g_bad) tb += kv.second;
  if (sens) { std::printf("sensitivity (accel[0] of sample 0 perturbed by 1 ulp in the C port): %ld mismatching comparisons\n", tb); report("bs_imu sens"); return tb > 0 ? 0 : 1; }
  print_cov();
  char t[64]; std::snprintf(t, sizeof t, "bs_imu seed %llu", (unsigned long long)seed);
  report(t);
  return tb ? 1 : 0;
}
