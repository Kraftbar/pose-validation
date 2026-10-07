// Basalt port M1 oracle: real Sophus 1.24.6 / basalt-headers / Eigen 3.4.0 (reference flags) vs basalt_port/c/bs_lie.c.
// Tolerance 0 (memcmp of every output, so signed zeros and NaN payloads count).
//   usage: bs_lie_test <seed> <cases_per_function> [sens]
// Built and run by tools/check_basalt_port.py --modules m1 (also directly, see runs/basalt_port/m1/build.sh).
#include <basalt/imu/imu_types.h>
#include <basalt/utils/ba_utils.h>
#include <basalt/utils/sophus_utils.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <map>

extern "C" {
#include "bs_lie.h"
}

using namespace Eigen;

static std::mt19937_64 g_rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(g_rng); }
static int I(int n) { return (int)(g_rng() % (uint64_t)n); }

struct Stat { long total = 0, bad = 0; };
static std::map<std::string, Stat> g_stats;
static std::map<std::string, std::string> g_first;

static std::map<std::string, long> g_cov;   // branch coverage of the generated inputs
static void cov(const std::string& k, bool c) { if (c) g_cov[k]++; }

static void chk(const char* name, const void* a, const void* b, size_t n, const char* ctx = "") {
  Stat& s = g_stats[name];
  s.total++;
  if (memcmp(a, b, n)) {
    s.bad++;
    if (!g_first.count(name)) {
      std::string m = "first mismatch case " + std::to_string(s.total) + " " + ctx;
      const unsigned char *pa = (const unsigned char*)a, *pb = (const unsigned char*)b;
      (void)pa; (void)pb;
      g_first[name] = m;
    }
  }
}

// ---- C api adapters ----
template <class S> struct C;
#define ADAPT(S, X)                                                                                              \
  template <> struct C<S> {                                                                                      \
    using Q = bs_quat##X; using SO3 = bs_so3##X; using SE3 = bs_se3##X;                                          \
    static void exp(const S* w, SO3* o) { bs_so3##X##_exp(w, o); }                                              \
    static void log(const SO3* q, S* o) { bs_so3##X##_log(q, o); }                                              \
    static void mul(const SO3* a, const SO3* b, SO3* o) { bs_so3##X##_mul(a, b, o); }                           \
    static void inv(const SO3* a, SO3* o) { bs_so3##X##_inverse(a, o); }                                        \
    static void mat(const SO3* a, S* o) { bs_so3##X##_matrix(a, o); }                                           \
    static void act(const SO3* a, const S* p, S* o) { bs_so3##X##_act(a, p, o); }                               \
    static void hat(const S* w, S* o) { bs_so3##X##_hat(w, o); }                                                \
    static void vee(const S* m, S* o) { bs_so3##X##_vee(m, o); }                                                \
    static void from_quat(const Q* q, SO3* o) { bs_so3##X##_from_quat(q, o); }                                  \
    static void rj(const S* p, S* J) { bs_right_jacobian_so3##X(p, J); }                                        \
    static void rji(const S* p, S* J) { bs_right_jacobian_inv_so3##X(p, J); }                                   \
    static void lj(const S* p, S* J) { bs_left_jacobian_so3##X(p, J); }                                         \
    static void lji(const S* p, S* J) { bs_left_jacobian_inv_so3##X(p, J); }                                    \
    static void s_mul(const SE3* a, const SE3* b, SE3* o) { bs_se3##X##_mul(a, b, o); }                         \
    static void s_inv(const SE3* a, SE3* o) { bs_se3##X##_inverse(a, o); }                                      \
    static void s_act(const SE3* a, const S* p, S* o) { bs_se3##X##_act(a, p, o); }                             \
    static void s_mat(const SE3* a, S* o) { bs_se3##X##_matrix(a, o); }                                         \
    static void s_mat34(const SE3* a, S* o) { bs_se3##X##_matrix3x4(a, o); }                                    \
    static void s_adj(const SE3* a, S* o) { bs_se3##X##_adj(a, o); }                                            \
    static void incpose(const S* inc, SE3* T) { bs_inc_pose##X(inc, T); }                                       \
    static void relpose(const SE3* a, const SE3* b, const SE3* c, const SE3* d, S* dh, S* dt, SE3* o) {         \
      bs_compute_rel_pose##X(a, b, c, d, dh, dt, o);                                                             \
    }                                                                                                            \
    static int two(const S* a, const S* b, Q* o) { return bs_quat##X##_from_two_vectors(a, b, o); }              \
  };
ADAPT(float, f)
ADAPT(double, d)

// bs_eigenf kernels
#define EADAPT(S, X)                                                                                             \
  static S e_sqn3(const S* v, S) { return bs_v3##X##_sqn(v); }                                                   \
  static S e_dot3(const S* a, const S* b, S) { return bs_v3##X##_dot(a, b); }                                    \
  static void e_cross(const S* a, const S* b, S* o, S) { bs_v3##X##_cross(a, b, o); }                           \
  static int e_norm3(const S* a, S* o, S) { return bs_v3##X##_normalized(a, o); }                                \
  static S e_sqn4(const S* v, S) { return bs_q##X##_sqn(v); }                                                    \
  static void e_m3mul(const S* a, const S* b, S* o, S) { bs_m3##X##_mul(a, b, o); }                              \
  static void e_m3mulv(const S* a, const S* v, S* o, S) { bs_m3##X##_mulv(a, v, o); }                            \
  static void e_m3t(const S* a, S* o, S) { bs_m3##X##_transpose(a, o); }                                         \
  static void e_m6mul(const S* a, const S* b, S* o, S) { bs_m6##X##_mul(a, b, o); }
EADAPT(float, f)
EADAPT(double, d)

// ---- random inputs ----
template <class S> Matrix<S, 3, 1> rnd_dir() {
  Matrix<double, 3, 1> v;
  double n;
  do { v << U(-1, 1), U(-1, 1), U(-1, 1); n = v.norm(); } while (n < 1e-3 || n > 1);
  return (v / n).template cast<S>();
}
// rotation vector magnitude drawn from a mix of regimes (small-angle Taylor branches, regular, near pi)
template <class S> Matrix<S, 3, 1> rnd_phi(bool max_pi) {
  double eps = std::is_same<S, float>::value ? 1e-5 : 1e-10;
  double a;
  switch (I(12)) {
    case 0: return Matrix<S, 3, 1>::Zero();
    case 1: a = std::pow(10.0, U(-14, -8)); break;                    // double Taylor branch / float ~ 0
    case 2: a = eps * U(0.2, 5.0); break;                             // around exp()'s theta < eps switch
    case 3: a = std::sqrt(eps) * U(0.2, 5.0); break;                  // Jacobian switch phi_norm2 > eps  (float 3.2e-3)
    case 4: a = U(1e-4, 1e-2); break;                                 // realistic dt * gyro
    case 5: a = U(1e-3, 0.2); break;
    case 6: a = M_PI - std::pow(10.0, U(-8, -1)); break;              // near pi (Jr^-1 switch at pi - sqrt(eps))
    case 7: a = M_PI - std::sqrt(eps) * U(0.5, 1.5); break;
    case 8: a = M_PI; break;
    default: a = U(0, M_PI); break;
  }
  if (!max_pi && I(5) == 0) a = U(0, 12);
  return (rnd_dir<S>() * (S)a);
}
template <class S> Quaternion<S> rnd_quat_raw() {            // possibly unnormalised
  Matrix<double, 4, 1> q; q << U(-1, 1), U(-1, 1), U(-1, 1), U(-1, 1);
  double n = q.norm(); q /= n;
  switch (I(4)) { case 0: break; case 1: q *= 1.0 + U(-1e-6, 1e-6); break; case 2: q *= 1.0 + U(-1e-2, 1e-2); break; default: q *= U(0.5, 2); }
  return Quaternion<S>((S)q[3], (S)q[0], (S)q[1], (S)q[2]);
}
template <class S> Sophus::SO3<S> rnd_so3() {
  switch (I(5)) {
    case 0: return Sophus::SO3<S>();
    case 1: return Sophus::SO3<S>::exp(rnd_phi<S>(false) * (S)1e-3);   // near identity (log small-angle branch)
    case 2: return Sophus::SO3<S>::exp(rnd_phi<S>(false));
    default: return Sophus::SO3<S>(rnd_quat_raw<S>());
  }
}
template <class S> Matrix<S, 3, 1> rnd_vec(double sc = 5) { return Matrix<S, 3, 1>((S)U(-sc, sc), (S)U(-sc, sc), (S)U(-sc, sc)); }
template <class S> Sophus::SE3<S> rnd_se3() { return Sophus::SE3<S>(rnd_so3<S>(), rnd_vec<S>(10)); }

template <class S, class Q> void to_c(const Sophus::SO3<S>& r, Q* q) { memcpy(q, r.data(), sizeof(Q)); static_assert(sizeof(Q) == 4 * sizeof(S), "q"); }
template <class S, class T> void to_c(const Sophus::SE3<S>& r, T* t) {
  memcpy(&t->so3, r.so3().data(), 4 * sizeof(S));
  memcpy(t->t, r.translation().data(), 3 * sizeof(S));
}

#define CMP(name, a, b, n, ctx) chk(name, a, b, n, ctx)

template <class S> void test_all(const char* tn, long N, int sens);

template <class S> void test_all(const char* tn, long N) {
  using Cs = C<S>;
  using SO3 = Sophus::SO3<S>;
  using SE3 = Sophus::SE3<S>;
  using V3 = Matrix<S, 3, 1>;
  using M3 = Matrix<S, 3, 3>;
  using M6 = Matrix<S, 6, 6>;
  auto nm = [&](const char* f) { return std::string(tn) + " " + f; };
  for (long it = 0; it < N; it++) {
    // ---- SO3 ----
    V3 w = rnd_phi<S>(false);
    {
      const double eps = std::is_same<S, float>::value ? 1e-5 : 1e-10;
      S t2 = w.squaredNorm();
      cov(std::string(tn) + " exp Taylor branch (theta^2 < eps^2)", t2 < (S)eps * (S)eps);
      cov(std::string(tn) + " exp regular branch", !(t2 < (S)eps * (S)eps));
      cov(std::string(tn) + " Jr/Jl Taylor branch (phi^2 <= eps)", !(t2 > (S)eps));
      cov(std::string(tn) + " Jr/Jl regular branch", t2 > (S)eps);
    }
    SO3 e = SO3::exp(w);
    typename Cs::SO3 ce; Cs::exp(w.data(), &ce);
    CMP(nm("SO3::exp").c_str(), &ce, e.data(), sizeof(ce), "");

    SO3 a = rnd_so3<S>(), b = rnd_so3<S>();
    typename Cs::SO3 ca, cb, co;
    to_c(a, &ca); to_c(b, &cb);
    {
      { S sq = a.unit_quaternion().vec().squaredNorm(); const double eps = std::is_same<S, float>::value ? 1e-5 : 1e-10;
        cov(std::string(tn) + " log Taylor branch", sq < (S)eps * (S)eps); cov(std::string(tn) + " log regular w<0", !(sq < (S)eps * (S)eps) && a.unit_quaternion().w() < 0);
        cov(std::string(tn) + " log regular w>=0", !(sq < (S)eps * (S)eps) && !(a.unit_quaternion().w() < 0)); }
      S l[3]; Cs::log(&ca, l);
      V3 r = a.log();
      CMP(nm("SO3::log").c_str(), l, r.data(), sizeof(l), "");
      // log of a near-identity rotation built through exp (small-angle branch)
      SO3 s = SO3::exp(rnd_phi<S>(true) * (S)(std::pow(10.0, U(-6, 0))));
      typename Cs::SO3 cs; to_c(s, &cs);
      Cs::log(&cs, l); r = s.log();
      CMP(nm("SO3::log small").c_str(), l, r.data(), sizeof(l), "");
    }
    {
      Cs::mul(&ca, &cb, &co); SO3 r = a * b;
      CMP(nm("SO3 * SO3").c_str(), &co, r.data(), sizeof(co), "");
      Cs::inv(&ca, &co); SO3 ri = a.inverse();
      CMP(nm("SO3::inverse").c_str(), &co, ri.data(), sizeof(co), "");
      S m[9]; Cs::mat(&ca, m); M3 rm = a.matrix();
      CMP(nm("SO3::matrix").c_str(), m, rm.data(), sizeof(m), "");
      V3 p = rnd_vec<S>(); S o[3]; Cs::act(&ca, p.data(), o); V3 rp = a * p;
      CMP(nm("SO3 * Vec3").c_str(), o, rp.data(), sizeof(o), "");
      // raw quaternion into the constructor / setQuaternion
      Quaternion<S> raw = rnd_quat_raw<S>();
      typename Cs::Q cq = {raw.x(), raw.y(), raw.z(), raw.w()};
      Cs::from_quat(&cq, &co); SO3 rn; rn.setQuaternion(raw);
      CMP(nm("SO3 setQuaternion").c_str(), &co, rn.data(), sizeof(co), "");
      SO3 rc(raw);
      CMP(nm("SO3(Quaternion)").c_str(), &co, rc.data(), sizeof(co), "");
      S h[9]; Cs::hat(p.data(), h); M3 rh = SO3::hat(p);
      CMP(nm("SO3::hat").c_str(), h, rh.data(), sizeof(h), "");
      S vv[3]; Cs::vee(rh.data(), vv); V3 rv = SO3::vee(rh);
      CMP(nm("SO3::vee").c_str(), vv, rv.data(), sizeof(vv), "");
    }
    // ---- Jacobians ----
    {
      V3 phi = rnd_phi<S>(false);
      V3 phi_pi = rnd_phi<S>(true);
      S J[9]; M3 R;
      Cs::rj(phi.data(), J); Sophus::rightJacobianSO3(phi, R);
      CMP(nm("rightJacobianSO3").c_str(), J, R.data(), sizeof(J), "");
      Cs::lj(phi.data(), J); Sophus::leftJacobianSO3(phi, R);
      CMP(nm("leftJacobianSO3").c_str(), J, R.data(), sizeof(J), "");
      { S n2 = phi_pi.squaredNorm(); const double eps = std::is_same<S, float>::value ? 1e-5 : 1e-10;
        S pn = std::sqrt(n2); S es = std::sqrt((S)eps);
        cov(std::string(tn) + " JrInv/JlInv Taylor-0 branch", !(n2 > (S)eps));
        cov(std::string(tn) + " JrInv/JlInv regular branch", n2 > (S)eps && (double)pn < M_PI - (double)es);
        cov(std::string(tn) + " JrInv/JlInv pi branch", n2 > (S)eps && !((double)pn < M_PI - (double)es)); }
      Cs::rji(phi_pi.data(), J); Sophus::rightJacobianInvSO3(phi_pi, R);
      CMP(nm("rightJacobianInvSO3").c_str(), J, R.data(), sizeof(J), "");
      Cs::lji(phi_pi.data(), J); Sophus::leftJacobianInvSO3(phi_pi, R);
      CMP(nm("leftJacobianInvSO3").c_str(), J, R.data(), sizeof(J), "");
      // as used in residual(): J of log() of a rotation
      SO3 x = rnd_so3<S>(); V3 lg = x.log();
      Cs::rji(lg.data(), J); Sophus::rightJacobianInvSO3(lg, R);
      CMP(nm("rightJacobianInvSO3(log)").c_str(), J, R.data(), sizeof(J), "");
      Cs::lji(lg.data(), J); Sophus::leftJacobianInvSO3(lg, R);
      CMP(nm("leftJacobianInvSO3(log)").c_str(), J, R.data(), sizeof(J), "");
    }
    // ---- SE3 ----
    {
      SE3 A = rnd_se3<S>(), B = rnd_se3<S>();
      typename Cs::SE3 cA, cB, cO;
      to_c(A, &cA); to_c(B, &cB);
      Cs::s_mul(&cA, &cB, &cO); SE3 R = A * B;
      SE3 Rm = R;
      typename Cs::SE3 cR; to_c(Rm, &cR);
      CMP(nm("SE3 * SE3 ").c_str(), &cO, &cR, sizeof(cO), "");
      Cs::s_inv(&cA, &cO); SE3 Ri = A.inverse(); to_c(Ri, &cR);
      CMP(nm("SE3::inverse").c_str(), &cO, &cR, sizeof(cO), "");
      V3 p = rnd_vec<S>(); S o[3]; Cs::s_act(&cA, p.data(), o); V3 rp = A * p;
      CMP(nm("SE3 * Vec3").c_str(), o, rp.data(), sizeof(o), "");
      S m4[16]; Cs::s_mat(&cA, m4); Matrix<S, 4, 4> R4 = A.matrix();
      CMP(nm("SE3::matrix").c_str(), m4, R4.data(), sizeof(m4), "");
      S m34[12]; Cs::s_mat34(&cA, m34); Matrix<S, 3, 4> R34 = A.matrix3x4();
      CMP(nm("SE3::matrix3x4").c_str(), m34, R34.data(), sizeof(m34), "");
      S ad[36]; Cs::s_adj(&cA, ad); M6 RA = A.Adj();
      CMP(nm("SE3::Adj").c_str(), ad, RA.data(), sizeof(ad), "");
      // PoseState::incPose
      Matrix<S, 6, 1> inc; inc << rnd_vec<S>(0.1), rnd_phi<S>(false) * (S)std::pow(10.0, U(-4, 0));
      SE3 T = A; basalt::PoseState<S>::incPose(inc, T);
      typename Cs::SE3 cT = cA; Cs::incpose(inc.data(), &cT); to_c(T, &cR);
      CMP(nm("PoseState::incPose").c_str(), &cT, &cR, sizeof(cT), "");
      // computeRelPose
      SE3 Hh = rnd_se3<S>(), Ih = rnd_se3<S>(), Tt = rnd_se3<S>(), It = rnd_se3<S>();
      if (I(3) == 0) { Ih = SE3(SO3(), V3::Zero()); It = SE3(SO3::exp(rnd_phi<S>(true)), rnd_vec<S>(0.2)); }
      typename Cs::SE3 cH, cI, cTt, cIt, cRel;
      to_c(Hh, &cH); to_c(Ih, &cI); to_c(Tt, &cTt); to_c(It, &cIt);
      M6 dh, dt;
      SE3 rel = basalt::computeRelPose<S>(Hh, Ih, Tt, It, &dh, &dt);
      S cdh[36], cdt[36];
      Cs::relpose(&cH, &cI, &cTt, &cIt, cdh, cdt, &cRel);
      to_c(rel, &cR);
      CMP(nm("computeRelPose T").c_str(), &cRel, &cR, sizeof(cR), "");
      CMP(nm("computeRelPose d_rel_d_h").c_str(), cdh, dh.data(), sizeof(cdh), "");
      CMP(nm("computeRelPose d_rel_d_t").c_str(), cdt, dt.data(), sizeof(cdt), "");
      SE3 rel2 = basalt::computeRelPose<S>(Hh, Ih, Tt, It);
      Cs::relpose(&cH, &cI, &cTt, &cIt, nullptr, nullptr, &cRel); to_c(rel2, &cR);
      CMP(nm("computeRelPose (no J)").c_str(), &cRel, &cR, sizeof(cR), "");
      // the two one-sided variants used by ba_base.cpp / linearization (only d_rel_d_t)
      M6 dt2; SE3 rel3 = basalt::computeRelPose<S>(Hh, Ih, Tt, It, nullptr, &dt2);
      Cs::relpose(&cH, &cI, &cTt, &cIt, nullptr, cdt, &cRel);
      CMP(nm("computeRelPose d_rel_d_t only").c_str(), cdt, dt2.data(), sizeof(cdt), "");
      (void)rel3;
    }
    // ---- FromTwoVectors ----
    {
      V3 acc;
      if (I(2)) acc = V3((S)U(-1.5, 1.5), (S)U(-1.5, 1.5), (S)U(8.5, 10.5));     // EuRoC-like specific force
      else acc = rnd_vec<S>(12);
      V3 uz = V3::UnitZ();
      typename Cs::Q cq; int ok = Cs::two(acc.data(), uz.data(), &cq);
      if (ok) {
        Quaternion<S> q = Quaternion<S>::FromTwoVectors(acc, uz);
        CMP(nm("FromTwoVectors").c_str(), &cq, q.coeffs().data(), sizeof(cq), "");
        SE3 T; T.setQuaternion(q);
        typename Cs::SO3 co2; typename Cs::Q cq2 = cq; Cs::from_quat(&cq2, &co2);
        CMP(nm("FromTwoVectors+setQuaternion").c_str(), &co2, T.so3().data(), sizeof(co2), "");
      }
    }
  }
}

// direct tests of the small fixed-size kernels with dense random operands (the Lie call sites only multiply hat() matrices,
// whose zero diagonal hides the association order of M3*M3)
template <class S> void test_kernels(const char* tn, long N) {
  using Mx3 = Matrix<S, 3, 3>; using V3 = Matrix<S, 3, 1>; using M6 = Matrix<S, 6, 6>;
  auto nm = [&](const char* f) { return std::string(tn) + " " + f; };
  for (long it = 0; it < N; it++) {
    double sc = std::pow(10.0, U(-3, 3));
    Mx3 A, B; V3 v, w;
    for (int i = 0; i < 9; i++) { A(i) = (S)U(-sc, sc); B(i) = (S)U(-sc, sc); }
    for (int i = 0; i < 3; i++) { v(i) = (S)U(-sc, sc); w(i) = (S)U(-sc, sc); }
    if (I(8) == 0) v.setZero();
    Mx3 C = A * B; S o9[9]; e_m3mul(A.data(), B.data(), o9, (S)0);
    chk(nm("Matrix3 * Matrix3").c_str(), o9, C.data(), sizeof(o9));
    memcpy(o9, A.data(), sizeof(o9)); e_m3mul(o9, o9, o9, (S)0); Mx3 C2 = A * A;
    chk(nm("Matrix3 * Matrix3 (aliased A*A)").c_str(), o9, C2.data(), sizeof(o9));
    V3 y = A * v; S o3[3]; e_m3mulv(A.data(), v.data(), o3, (S)0);
    chk(nm("Matrix3 * Vector3").c_str(), o3, y.data(), sizeof(o3));
    Mx3 At = A.transpose(); e_m3t(A.data(), o9, (S)0);
    chk(nm("Matrix3::transpose").c_str(), o9, At.data(), sizeof(o9));
    S d = v.dot(w), od = e_dot3(v.data(), w.data(), (S)0);
    chk(nm("Vector3::dot").c_str(), &od, &d, sizeof(S));
    S q = v.squaredNorm(), oq = e_sqn3(v.data(), (S)0);
    chk(nm("Vector3::squaredNorm").c_str(), &oq, &q, sizeof(S));
    V3 cr = v.cross(w); e_cross(v.data(), w.data(), o3, (S)0);
    chk(nm("Vector3::cross").c_str(), o3, cr.data(), sizeof(o3));
    V3 nv = v.normalized(); e_norm3(v.data(), o3, (S)0);
    chk(nm("Vector3::normalized").c_str(), o3, nv.data(), sizeof(o3));
    Matrix<S, 4, 1> q4; for (int i = 0; i < 4; i++) q4(i) = (S)U(-sc, sc);
    S n4 = q4.squaredNorm(), on4 = e_sqn4(q4.data(), (S)0);
    chk(nm("Vector4::squaredNorm").c_str(), &on4, &n4, sizeof(S));
    S nn = q4.norm(), onn = std::sqrt(on4);
    chk(nm("Vector4::norm").c_str(), &onn, &nn, sizeof(S));
    M6 A6, B6; for (int i = 0; i < 36; i++) { A6(i) = (S)U(-sc, sc); B6(i) = (S)U(-sc, sc); if (I(6) == 0) A6(i) = 0; if (I(6) == 0) B6(i) = 0; }
    M6 C6 = A6 * B6; S o36[36]; e_m6mul(A6.data(), B6.data(), o36, (S)0);
    chk(nm("Matrix6 * Matrix6").c_str(), o36, C6.data(), sizeof(o36));
    M6 C6n = -A6 * B6; for (int i = 0; i < 36; i++) o36[i] = -A6(i);
    S o36b[36]; e_m6mul(o36, B6.data(), o36b, (S)0);
    chk(nm("-Matrix6 * Matrix6").c_str(), o36b, C6n.data(), sizeof(o36b));
  }
}

// casts double -> float and back (calib.T_i_c.cast<float>())
static void test_casts(long N) {
  for (long it = 0; it < N; it++) {
    Sophus::SE3d A = rnd_se3<double>();
    bs_se3d cA; to_c(A, &cA);
    Sophus::SE3f Rf = A.cast<float>();
    bs_se3f cf; bs_se3_f_from_d(&cA, &cf);
    bs_se3f rf; to_c(Rf, &rf);
    chk("SE3<d>::cast<f>", &cf, &rf, sizeof(cf));
    Sophus::SE3f F = rnd_se3<float>();
    bs_se3f cF; to_c(F, &cF);
    Sophus::SE3d Rd = F.cast<double>();
    bs_se3d cd; bs_se3_d_from_f(&cF, &cd);
    bs_se3d rd; to_c(Rd, &rd);
    chk("SE3<f>::cast<d>", &cd, &rd, sizeof(cd));
  }
}

// sensitivity: wrong-order alternatives of the Eigen rules must disagree with real Eigen often (proves the test can see them)
template <class S> void sensitivity(const char* tn, long N) {
  long d_sqn_left = 0, d_m3_left = 0, d_m3_tree = 0, d_q4_left = 0, d_m6_alllast = 0, d_pi = 0;
  for (long it = 0; it < N; it++) {
    Matrix<S, 3, 1> v = rnd_vec<S>(3);
    S r = v.squaredNorm();
    S left = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2], tree = v[0] * v[0] + (v[1] * v[1] + v[2] * v[2]);
    d_sqn_left += memcmp(&r, &left, sizeof(S)) != 0;
    (void)tree;
    Matrix<S, 3, 3> A = Matrix<S, 3, 3>::Random().eval() * (S)3, B = Matrix<S, 3, 3>::Random().eval() * (S)3, Cm = A * B;
    S l = (A(2, 0) * B(0, 2) + A(2, 1) * B(1, 2)) + A(2, 2) * B(2, 2);
    S t = A(2, 0) * B(0, 2) + (A(2, 1) * B(1, 2) + A(2, 2) * B(2, 2));
    d_m3_left += memcmp(&l, &Cm(2, 2), sizeof(S)) != 0;
    d_m3_tree += memcmp(&t, &Cm(2, 2), sizeof(S)) != 0;
    Matrix<S, 4, 1> q = Matrix<S, 4, 1>::Random().eval() * (S)3;
    S qs = q.squaredNorm(), ql = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
    d_q4_left += memcmp(&qs, &ql, sizeof(S)) != 0;
    Matrix<S, 6, 6> A6 = Matrix<S, 6, 6>::Random().eval() * (S)3, B6 = Matrix<S, 6, 6>::Random().eval() * (S)3, C6 = A6 * B6;
    S s6 = A6(5, 0) * B6(0, 0);
    for (int k = 1; k < 6; k++) s6 = s6 + A6(5, k) * B6(k, 0);
    d_m6_alllast += memcmp(&s6, &C6(5, 0), sizeof(S)) != 0;
  }
  printf("sensitivity %s (wrong rule disagrees with real Eigen in n of %ld): squaredNorm3 left-fold %ld, M3*M3(2,2) left %ld / tree %ld, squaredNorm4 left-fold %ld, M6*M6(5,0) left-fold %ld\n",
         tn, N, d_sqn_left, d_m3_left, d_m3_tree, d_q4_left, d_m6_alllast);
  (void)d_pi;
}

int main(int argc, char** argv) {
  uint64_t seed = argc > 1 ? strtoull(argv[1], 0, 10) : 1;
  long N = argc > 2 ? atol(argv[2]) : 20000;
  bool sens = argc > 3;
  g_rng.seed(seed);
  test_all<float>("float", N);
  test_all<double>("double", N);
  test_kernels<float>("float", N);
  test_kernels<double>("double", N);
  test_casts(N);
  if (sens) { sensitivity<float>("float", N); sensitivity<double>("double", N); }
  for (auto& kv : g_cov) printf("coverage %-52s %ld\n", kv.first.c_str(), kv.second);
  long tb = 0, tt = 0;
  for (auto& kv : g_stats) {
    printf("%-46s %ld/%ld%s%s\n", kv.first.c_str(), kv.second.bad, kv.second.total, g_first.count(kv.first) ? "  " : "", g_first.count(kv.first) ? g_first[kv.first].c_str() : "");
    tb += kv.second.bad; tt += kv.second.total;
  }
  printf("bs_lie_test seed %llu: %ld/%ld\n", (unsigned long long)seed, tb, tt);
  return tb ? 1 : 0;
}
