// Oracle for basalt_port/c/bs_vio_opt.{h,c} (module M7: LM loop of SqrtKeypointVioEstimator<float>::optimize + dense LDLT solve).
// Modes (argv[1]):  ldlt <seed> <cases>      random matrices vs the real Eigen::LDLT<Ref<MatX>> (factor, transpositions, solve), tolerance 0
//                   lambda <seed> <cases>    the lambda update expression of optimize() compiled by g++ vs the C expression
//                   optimize <seed> <cases>  random estimator states vs the real SqrtKeypointVioEstimator<float>::optimize() (private members
//                                            reached through explicit-instantiation access, never a macro hack)
//                   replay <dir> [max]       replay of a reference dump (m6.bin PROBLEM kind 0 + iter.bin OPT_BEGIN / ITER_STEP / OPT_END)
// The M6 oracle (bs_linabsqr_test.cc) is included for its random problem generator and the dump reader; its main() is renamed.
#include <basalt/camera/double_sphere_camera.hpp>
#define main bs_m6_main
#include "bs_linabsqr_test.cc"
#undef main
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/sqrt_keypoint_vio.h>
#include <Eigen/Cholesky>
#include <iostream>
#include <sstream>
#include <memory>
extern "C" {
#include "bs_vio_opt.h"
}

// ------------------------------------------------------------------------------------------------ access to the private members
template <class Tag, typename Tag::type M> struct Rob { friend typename Tag::type get(Tag) { return M; } };
#define BS_ACCESS(NAME, ...)                                                             \
  struct Tag_##NAME { typedef __VA_ARGS__ basalt::SqrtKeypointVioEstimator<S>::*type; friend type get(Tag_##NAME); }; \
  template struct Rob<Tag_##NAME, &basalt::SqrtKeypointVioEstimator<S>::NAME>;
typedef basalt::SqrtKeypointVioEstimator<S> Est;
BS_ACCESS(kf_ids, std::set<int64_t>)
BS_ACCESS(last_state_t_ns, int64_t)
BS_ACCESS(imu_meas, Eigen::aligned_map<int64_t, basalt::IntegratedImuMeasurement<S>>)
BS_ACCESS(num_points_kf, std::map<int64_t, int>)
BS_ACCESS(marg_data, basalt::MargLinData<S>)
BS_ACCESS(gyro_bias_sqrt_weight, Eigen::Matrix<S, 3, 1>)
BS_ACCESS(accel_bias_sqrt_weight, Eigen::Matrix<S, 3, 1>)
BS_ACCESS(opt_started, bool)
BS_ACCESS(take_kf, bool)
BS_ACCESS(frames_after_kf, int)
BS_ACCESS(lambda, S)
BS_ACCESS(min_lambda, S)
BS_ACCESS(max_lambda, S)
BS_ACCESS(lambda_vee, S)
BS_ACCESS(max_states, size_t)
BS_ACCESS(max_kfs, size_t)
BS_ACCESS(config, basalt::VioConfig)
#define P_(est, NAME) ((est).*get(Tag_##NAME()))

static basalt::VioConfig& cfg();
// ------------------------------------------------------------------------------------------------ ldlt
static void ldlt_cases(Rng& r, long cases) {
  for (long c = 0; c < cases; ++c) {
    int n;
    const int sc = (int)(r() % 10);
    if (sc < 3) n = 1 + (int)(r() % 12);
    else if (sc < 6) n = 13 + (int)(r() % 40);
    else if (sc < 9) n = 40 + (int)(r() % 60);
    else n = 100 + (int)(r() % 40);
    if (r() % 5 == 0) n = (r() % 2) ? 87 : 72;
    MatX A(n, n);
    const int cls = (int)(r() % 9);
    if (cls == 0 || cls == 1 || cls == 2) {            // SPD: J^T J + damping, mixed scales (like H + lambda diag)
      const int m = n + (int)(r() % (2 * n + 5));
      MatX J(m, n);
      for (int i = 0; i < m; ++i) for (int j = 0; j < n; ++j) J(i, j) = (r() % 4 == 0) ? 0.0f : (S)(nrand(r) * std::exp(urand(r, -3, 3) * (cls == 2 ? 1.0 : 0.3)));
      A = J.transpose() * J;
      for (int i = 0; i < n; ++i) A(i, i) += (S)std::exp(urand(r, -14, 2));
    } else if (cls == 3) {                              // generic symmetric (indefinite)
      for (int i = 0; i < n; ++i) for (int j = 0; j <= i; ++j) A(i, j) = A(j, i) = (S)(nrand(r) * std::exp(urand(r, -2, 2)));
    } else if (cls == 4) {                              // singular / rank deficient, zero rows, tied diagonal
      const int rk = 1 + (int)(r() % n);
      MatX J(rk, n);
      for (int i = 0; i < rk; ++i) for (int j = 0; j < n; ++j) J(i, j) = (r() % 3 == 0) ? 0.0f : (S)nrand(r);
      A = J.transpose() * J;
      if (r() % 2) for (int i = 0; i < n; ++i) A(i, i) = std::floor(A(i, i) * 4) / 4;   // exact ties in the pivot search
    } else if (cls == 5) {                              // banded / block diagonal with exact zeros
      A.setZero();
      for (int i = 0; i < n; ++i) { A(i, i) = (S)std::exp(urand(r, -3, 5)) * ((r() % 6 == 0) ? -1 : 1); for (int j = std::max(0, i - 3); j < i; ++j) A(i, j) = A(j, i) = (S)(nrand(r) * 0.3); }
    } else if (cls == 6) {                              // integers (ties, exact arithmetic paths)
      for (int i = 0; i < n; ++i) for (int j = 0; j <= i; ++j) A(i, j) = A(j, i) = (S)((int)(r() % 7) - 3);
    } else if (cls == 8) {                              // denormal diagonal: |d| <= FLT_MIN takes the pseudo-inverse branch of solve()
      const int m = n + (int)(r() % n + 2);
      MatX J(m, n);
      for (int i = 0; i < m; ++i) for (int j = 0; j < n; ++j) J(i, j) = (S)nrand(r);
      A = J.transpose() * J;
      const double sc = std::pow(10.0, urand(r, -41.5, -36.5));
      for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) A(i, j) = (S)((double)A(i, j) * sc);
      if (r() % 3 == 0) for (int i = 0; i < n; ++i) if (r() % 4 == 0) A(i, i) = 0.0f;
    } else {                                            // tiny / denormal / huge magnitudes
      for (int i = 0; i < n; ++i) for (int j = 0; j <= i; ++j) A(i, j) = A(j, i) = (S)(nrand(r) * std::pow(10.0, urand(r, -40, 38)));
    }
    if (r() % 25 == 0) A.setZero();
    if (r() % 40 == 0) { int z = (int)(r() % n); A.row(z).setZero(); A.col(z).setZero(); }
    VecX b(n);
    for (int i = 0; i < n; ++i) b[i] = (r() % 10 == 0) ? 0.0f : (S)(nrand(r) * std::exp(urand(r, -3, 3)));
    MatX Ae = A;
    Eigen::LDLT<Eigen::Ref<MatX>> ldlt(Ae);
    VecX xe = ldlt.solve(b);
    std::vector<float> Ac(A.data(), A.data() + (size_t)n * n);
    std::vector<int> tr(n);
    bs_ldlt_factor(Ac.data(), n, tr.data());
    VecX xc(n);
    bs_ldlt_solve(Ac.data(), n, tr.data(), b.data(), xc.data());
    g_shape[0] = n;
    ++g_cmp["ldlt: factor (matrixLDLT, whole storage)"];
    if (std::memcmp(Ac.data(), Ae.data(), 4 * (size_t)n * n)) { if (++g_bad["ldlt: factor (matrixLDLT, whole storage)"] <= 6) { int fi = -1; for (int q = 0; q < n * n; ++q) if (std::memcmp(&Ac[q], &Ae.data()[q], 4)) { fi = q; break; } std::printf("MISMATCH factor n=%d class %d first diff (%d,%d) c %.9g e %.9g\n", n, cls, fi % n, fi / n, Ac[fi], Ae.data()[fi]); } }
    ++g_cmp["ldlt: transpositions"];
    { bool ok = true; for (int i = 0; i < n; ++i) if (tr[i] != (int)ldlt.transpositionsP().indices()[i]) ok = false; if (!ok) ++g_bad["ldlt: transpositions"]; }
    ++g_cmp["ldlt: solve (x)"];
    // NaN payloads are not modelled (compiler dependent): treat NaN == NaN
    bool same = true;
    for (int i = 0; i < n; ++i) { const bool a = std::isnan(xe[i]), bb = std::isnan(xc[i]); if (a || bb) { if (a != bb) same = false; } else if (std::memcmp(&xe[i], &xc[i], 4)) same = false; }
    if (!same) { if (++g_bad["ldlt: solve (x)"] <= 3) std::printf("MISMATCH solve n=%d class %d\n", n, cls); }
  }
}

// ------------------------------------------------------------------------------------------------ the constructor's prior
static void prior_cases(Rng& r, long cases) {
  for (long c = 0; c < cases; ++c) {
    basalt::VioConfig vc = cfg();
    if (c > 0) { vc.vio_init_pose_weight = std::exp(urand(r, 0, 20)); vc.vio_init_ba_weight = std::exp(urand(r, -3, 6)); vc.vio_init_bg_weight = std::exp(urand(r, -3, 8)); }
    basalt::Calibration<double> cd;
    std::ostringstream sink; std::streambuf* ob = std::cout.rdbuf(sink.rdbuf());
    Est e(Eigen::Vector3d(0, 0, -9.81), cd, vc);
    std::cout.rdbuf(ob);
    bs_vio v; bs_vio_init(&v);
    const int64_t t = 1403636579763555584LL + (int64_t)(r() % 100000);
    bs_vio_init_marg_prior(&v, t, vc.vio_init_pose_weight, vc.vio_init_ba_weight, vc.vio_init_bg_weight);
    const basalt::MargLinData<S>& md = P_(e, marg_data);
    ++g_cmp["prior: constructor marg_data.H / b == bs_vio_init_marg_prior"];
    if (md.H.rows() != 15 || md.H.cols() != 15 || std::memcmp(md.H.data(), v.marg.H, 4 * 225) || std::memcmp(md.b.data(), v.marg.b, 4 * 15) || v.marg.total != 15 || v.marg.n != 1 || v.marg.item[0].start != 0 || v.marg.item[0].size != 15)
      ++g_bad["prior: constructor marg_data.H / b == bs_vio_init_marg_prior"];
    bs_vio_destroy(&v);
  }
}

// ------------------------------------------------------------------------------------------------ lambda update (the expressions of optimize())
static void lambda_cases(Rng& r, long cases) {
  for (long c = 0; c < cases; ++c) {
    S rd = (S)(nrand(r) * std::exp(urand(r, -6, 4)));
    if (r() % 20 == 0) rd = (S)(r() % 2 ? 1.0 : 0.5);
    S lambda = (S)std::exp(urand(r, -16, 5)), min_lambda = 1e-6f;
    const S l0 = lambda;
    // verbatim from sqrt_keypoint_vio.cpp
    lambda *= std::max<S>(S(1.0) / 3, 1 - std::pow<S>(2 * rd - 1, 3));
    lambda = std::max(min_lambda, lambda);
    // C (bs_vio_opt.c)
    float lc = l0;
    {
      const float third = 1.0f / 3;
      const float x = (float)(1 - pow((double)(2 * rd - 1), 3.0));
      lc *= (third < x) ? x : third;
      lc = (min_lambda < lc) ? lc : min_lambda;
    }
    ++g_cmp["lambda: accepted-step update"];
    if (std::memcmp(&lc, &lambda, 4)) { if (++g_bad["lambda: accepted-step update"] <= 3) std::printf("MISMATCH lambda rd=%.9g: %.9g vs %.9g\n", rd, lc, lambda); }
  }
}


// ------------------------------------------------------------------------------------------------ real estimator vs C (optimize)
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>
// The C++ asserts (BASALT_ASSERT, SOPHUS_ENSURE, .at()) abort the process, possibly inside a TBB body, so an abort cannot be caught in process.
// The scenarios the C port flags as "the C++ aborts here" are run in a forked child that must die with SIGABRT; the other scenarios run in the
// parent, where an unexpected abort ends the test with a diagnosis.
static void on_abort(int) {
  const char msg[] = "FATAL: the real class aborted on a scenario the C port did not flag (assertion mismatch)\n";
  ssize_t w = write(2, msg, sizeof msg - 1); (void)w;
  _exit(3);
}
template <class F>
static bool real_aborts_in_child(F f) {
  fflush(stdout); fflush(stderr);
  const pid_t pid = fork();
  if (pid == 0) { signal(SIGABRT, SIG_DFL); alarm(60); f(); _exit(0); }
  int status = 0;
  waitpid(pid, &status, 0);
  return WIFSIGNALED(status) && WTERMSIG(status) == SIGABRT;
}

static basalt::VioConfig& cfg() {
  static basalt::VioConfig c;
  static bool loaded = false;
  if (!loaded) { c.load("/home/nybo/github/pose-validation/external/vio/basalt_src/data/euroc_config.json"); loaded = true; }
  return c;
}

// move a C lmdb to another address (the hash tables point into themselves: before_begin / single_bucket)
static void htab_move(bs_htab* dst, bs_htab* src) {
  *dst = *src;
  if (src->buckets == &src->single_bucket) dst->buckets = &dst->single_bucket;
  for (size_t i = 0; i < dst->nbuckets; ++i) if (dst->buckets[i] == &src->before_begin) dst->buckets[i] = &dst->before_begin;
  src->buckets = nullptr; src->nbuckets = 0; src->nelem = 0; src->before_begin.next = nullptr; src->single_bucket = nullptr;
}
static void lmdb_move(bs_lmdb* dst, bs_lmdb* src) {
  bs_lmdb_destroy(dst);
  htab_move(&dst->kpts, &src->kpts);
  htab_move(&dst->observations, &src->observations);
  bs_lmdb_init(src);
}

static bool bits_eq(const float* a, const float* b, int n) {   // memcmp, NaN == NaN
  for (int i = 0; i < n; ++i) { const bool x = std::isnan(a[i]), y = std::isnan(b[i]); if (x || y) { if (x != y) return false; } else if (std::memcmp(&a[i], &b[i], 4)) return false; }
  return true;
}

// an estimator-shaped twin pair (real Est + C bs_vio) from an M6 Problem.  The prior is regenerated over a prefix of the full layout.
struct Twin {
  std::unique_ptr<Est> est;
  bs_vio v;
  basalt::Calibration<double> cd;
  Twin() { bs_vio_init(&v); }
  ~Twin() { bs_vio_destroy(&v); }
};

static bool make_twin(Rng& r, Problem& P, Twin& T, bool want_prior_all_poses, int max_extra = 1 << 20, int* np_out = nullptr) {
  auto& ba = P.ba;
  const int np = (int)ba.frame_poses.size(), ns = (int)ba.frame_states.size();
  // layout of the full aom: poses (ascending) then states
  std::vector<std::pair<int64_t, int>> layout;   // (t, size)
  for (auto& kv : ba.frame_poses) layout.push_back({kv.first, 6});
  for (auto& kv : ba.frame_states) layout.push_back({kv.first, 15});
  // prior: prefix of the layout covering all key frame poses (optimize() asserts the poses are in the prior order) and the first 0..ns states;
  // every prior item must be linearized (computeDelta asserts): unlinearized ones are linearized with setLinTrue (delta == 0 there)
  const int mx = std::max(0, std::min(max_extra, ns));
  int q = np + (int)(r() % 4 == 0 ? r() % (mx + 1) : std::min(1, mx)), cols = 0;
  for (int i = 0; i < q; ++i) cols += layout[i].second;
  for (int i = 0; i < q; ++i) {
    const int64_t t = layout[i].first;
    if (ba.frame_poses.count(t)) {
      auto& ps = ba.frame_poses.at(t);
      if (!ps.isLinearized()) {
        ps.setLinTrue();
        for (auto& c : P.cposes) if (c.t_ns == t) { c.linearized = 1; c.cur = c.lin; }
      }
    } else {
      auto& st = ba.frame_states.at(t);
      if (!st.isLinearized()) {
        st.setLinTrue();
        for (auto& c : P.cstates) if (c.t_ns == t) { c.s.linearized = 1; c.s.cur = c.s.lin; }
      }
    }
  }
  T.est.reset(new Est(P.g.cast<double>(), T.cd, cfg()));
  Est& e = *T.est;
  basalt::BundleAdjustmentBase<S>& base = e;
  base.frame_poses = ba.frame_poses; base.frame_states = ba.frame_states; base.lmdb = ba.lmdb;
  base.obs_std_dev = ba.obs_std_dev; base.huber_thresh = ba.huber_thresh; base.calib = ba.calib;
  P_(e, imu_meas) = P.imu;
  P_(e, gyro_bias_sqrt_weight) = P.gyro_sw; P_(e, accel_bias_sqrt_weight) = P.accel_sw;
  P_(e, opt_started) = true;
  basalt::MargLinData<S>& md = P_(e, marg_data);
  md = basalt::MargLinData<S>();
  md.is_sqrt = true;
  std::vector<bs_aom_item> items;
  int start = 0;
  for (int i = 0; i < q; ++i) {
    md.order.abs_order_map[layout[i].first] = std::make_pair(start, layout[i].second);
    items.push_back(bs_aom_item{layout[i].first, start, layout[i].second});
    start += layout[i].second;
  }
  md.order.total_size = cols; md.order.items = q;
  int rr = cols + (int)(r() % 12) - ((r() % 3 == 0) ? 3 : 0);
  if (rr < 1) rr = 1;
  if (rr + 2 * cols < 20) rr = 20 - 2 * cols;
  md.H.resize(rr, cols); md.b.resize(rr);
  const double hs = std::exp(urand(r, -3, 3));
  for (int i = 0; i < rr * cols; ++i) md.H.data()[i] = (S)(nrand(r) * hs);
  for (int i = 0; i < rr; ++i) md.b[i] = (S)(nrand(r) * hs);

  // C twin
  bs_vio& v = T.v;
  v.ba.obs_std_dev = ba.obs_std_dev; v.ba.huber_thresh = ba.huber_thresh;
  for (int c = 0; c < 2; ++c) { v.ba.T_i_c[c] = P.cba.T_i_c[c]; v.ba.cam[c] = P.cba.cam[c]; }
  for (auto& c : P.cposes) *bs_vio_pose_insert(&v, c.t_ns) = c;
  for (auto& c : P.cstates) *bs_vio_state_insert(&v, c.t_ns) = c;
  for (auto& c : P.cimu) *bs_vio_imu_insert(&v, c.start_t_ns) = c;
  lmdb_move(&v.ba.lmdb, &P.cba.lmdb);
  std::memcpy(v.g, P.g.data(), 12); std::memcpy(v.gyro_bias_sqrt_weight, P.gyro_sw.data(), 12); std::memcpy(v.accel_bias_sqrt_weight, P.accel_sw.data(), 12);
  bs_marg_data_set(&v.marg, items.data(), q, cols, rr, cols, md.H.data(), md.b.data());
  v.opt_started = 1;
  if (np_out) *np_out = np + ns;
  return true;
}

static bool compare_state(const char* tag, Est& e, bs_vio& v, long* nlm_out = nullptr) {
  basalt::BundleAdjustmentBase<S>& base = e;
  bool ok = true;
  auto fail = [&](const char* what) { if (++g_bad[std::string(tag) + ": " + what] <= 3) std::printf("MISMATCH %s: %s\n", tag, what); ok = false; };
  ++g_cmp[std::string(tag) + ": frame poses (lin, current, delta, flag)"];
  if ((int)base.frame_poses.size() != v.ba.n_poses) fail("frame poses (lin, current, delta, flag)");
  else {
    int i = 0;
    for (auto& kv : base.frame_poses) {
      const bs_frame_pose& c = v.ba.poses[i++];
      const bs_se3f lin = to_c(kv.second.getPoseLin()), cur = to_c(kv.second.getPose());
      const bs_se3f cc = c.linearized ? c.cur : c.lin;
      if (kv.first != c.t_ns || kv.second.isLinearized() != (bool)c.linearized || std::memcmp(&lin, &c.lin, sizeof lin) || std::memcmp(&cur, &cc, sizeof cur) || !bits_eq(kv.second.getDelta().data(), c.delta, 6)) { fail("frame poses (lin, current, delta, flag)"); break; }
    }
  }
  ++g_cmp[std::string(tag) + ": frame states (lin, current, delta, flag)"];
  if ((int)base.frame_states.size() != v.ba.n_states) fail("frame states (lin, current, delta, flag)");
  else {
    int i = 0;
    for (auto& kv : base.frame_states) {
      const bs_frame_state& c = v.ba.states[i++];
      const bs_pvbstate lin = to_c(kv.second.getStateLin()), cur = to_c(kv.second.getState());
      const bs_pvbstate cc = c.s.linearized ? c.s.cur : c.s.lin;
      if (kv.first != c.t_ns || kv.second.isLinearized() != (bool)c.s.linearized || !bits_eq((const float*)&lin.s.q, (const float*)&c.s.lin.s.q, 16 / 4 + 3 + 3) || std::memcmp(lin.s.q, c.s.lin.s.q, 16) || std::memcmp(lin.s.p, c.s.lin.s.p, 12) || std::memcmp(lin.s.v, c.s.lin.s.v, 12) || std::memcmp(lin.bg, c.s.lin.bg, 12) || std::memcmp(lin.ba, c.s.lin.ba, 12) ||
          std::memcmp(cur.s.q, cc.s.q, 16) || std::memcmp(cur.s.p, cc.s.p, 12) || std::memcmp(cur.s.v, cc.s.v, 12) || std::memcmp(cur.bg, cc.bg, 12) || std::memcmp(cur.ba, cc.ba, 12) || !bits_eq(kv.second.getDelta().data(), c.delta, 15)) { fail("frame states (lin, current, delta, flag)"); break; }
    }
  }
  ++g_cmp[std::string(tag) + ": landmark values (direction, inv_dist)"];
  {
    long nl = 0;
    if (base.lmdb.numLandmarks() != v.ba.lmdb.kpts.nelem) fail("landmark values (direction, inv_dist)");
    else for (const auto& kv : base.lmdb.getLandmarks()) {
      const bs_keypoint* k = bs_lmdb_get_landmark(&v.ba.lmdb, kv.first);
      ++nl;
      if (!k || !bits_eq(kv.second.direction.data(), k->direction, 2) || !bits_eq(&kv.second.inv_dist, &k->inv_dist, 1)) { fail("landmark values (direction, inv_dist)"); break; }
    }
    if (nlm_out) *nlm_out = nl;
  }
  return ok;
}

static void optimize_cases(Rng& r, long cases) {
  signal(SIGABRT, on_abort);
  long aborted = 0, ran = 0, steps = 0, accepted = 0, rejected = 0, retries = 0, conv = 0, term = 0, with_marg_rows = 0, big = 0;
  std::streambuf* cout_buf = std::cout.rdbuf();
  std::ostringstream sink;
  for (long sc = 0; sc < cases; ++sc) {
    Problem P;
    build_problem(r, P, (int)(r() % 4 == 0 ? 0 : 1));
    Twin T;
    std::cout.rdbuf(sink.rdbuf()); if (!getenv("BS_T_VERBOSE")) std::cerr.setstate(std::ios::failbit);
    const bool made = make_twin(r, P, T, false);
    std::cout.rdbuf(cout_buf); std::cerr.clear();
    if (!made) continue;
    Est& e = *T.est; bs_vio& v = T.v;
    // randomised LM configuration (identical on both sides)
    const int cfgc = (int)(r() % 6);
    double lam0 = 1e-4;
    if (cfgc == 1) lam0 = std::exp(urand(r, -9, 3));
    if (cfgc == 2) lam0 = std::exp(urand(r, -20, -3));
    const int maxit = (r() % 3 == 0) ? 1 + (int)(r() % 7) : 7;
    S minl = (r() % 6 == 0) ? (S)std::exp(urand(r, -14, -3)) : 1e-6f;
    S maxl = (r() % 6 == 0) ? (S)std::exp(urand(r, -2, 5)) : 1e2f;
    S vee = (r() % 4 == 0) ? (S)(2 << (r() % 4)) : 2.0f;
    if (r() % 6 == 0) { static const int pw[4] = {1, 3, 6, 10}; maxl = (S)lam0 * (S)(1 << pw[r() % 4]); }   // max_lambda exactly on the lambda sequence (the `>` test)
    P_(e, config).vio_lm_lambda_initial = lam0; P_(e, config).vio_max_iterations = maxit;
    P_(e, min_lambda) = minl; P_(e, max_lambda) = maxl; P_(e, lambda_vee) = vee;
    v.lm_lambda_initial = lam0; v.max_iterations = maxit; v.min_lambda = minl; v.max_lambda = maxl; v.lambda_vee = vee;
    if (cfgc == 4 || (cfgc == 3 && r() % 2)) {   // large observation std: tiny costs, f_diff around the 1e-6 function tolerance
      const S sd = (S)std::exp(urand(r, std::log(20.0), std::log(3000.0)));
      static_cast<basalt::BundleAdjustmentBase<S>&>(e).obs_std_dev = sd; v.ba.obs_std_dev = sd;
    }
    if (cfgc == 5 && (r() % 3 == 0)) {   // overflowing prior: H = J^T J becomes inf, the solve returns non-finite increments (retry path)
      basalt::MargLinData<S>& md = P_(e, marg_data);
      const S f = (r() % 2) ? 1e22f : 1e30f;
      for (int i = 0; i < md.H.size(); ++i) { md.H.data()[i] *= f; v.marg.H[i] *= f; }
    }
    if (!compare_state("optimize pre-state (twin construction)", e, v)) continue;
    if (v.marg.rows > 0) ++with_marg_rows;
    if (v.marg.total + 0 >= 60) ++big;
    // C first (the real call may abort on an invalid linearisation, the assertion of the C++)
    bs_opt_info info;
    bs_vio_optimize(&v, &info, nullptr, nullptr);
    bool real_aborted = false;
    std::cout.rdbuf(sink.rdbuf()); if (!getenv("BS_T_VERBOSE")) std::cerr.setstate(std::ios::failbit);
    if (info.invalid_linearization || info.nonfinite_increment) real_aborted = real_aborts_in_child([&] { e.optimize(); });
    else e.optimize();
    std::cout.rdbuf(cout_buf); std::cerr.clear();
    ++g_cmp["optimize: C reports invalid linearisation / non-finite increment <=> the real run aborts"];
    if (real_aborted != (bool)(info.invalid_linearization || info.nonfinite_increment)) { if (++g_bad["optimize: C reports invalid linearisation / non-finite increment <=> the real run aborts"] <= 3) std::printf("MISMATCH assert behaviour: real %d C %d/%d\n", (int)real_aborted, info.invalid_linearization, info.nonfinite_increment); }
    if (real_aborted) { ++aborted; continue; }
    ++ran;
    steps += info.steps; rejected += info.it_rejected; accepted += info.steps - info.it_rejected; retries += info.retries; conv += info.converged; term += info.terminated;
    compare_state("optimize", e, v);
    ++g_cmp["optimize: lambda, lambda_vee"];
    if (std::memcmp(&P_(e, lambda), &v.lambda, 4) || std::memcmp(&P_(e, lambda_vee), &v.lambda_vee, 4)) { if (++g_bad["optimize: lambda, lambda_vee"] <= 3) std::printf("MISMATCH lambda %.9g/%.9g vee %.9g/%.9g\n", P_(e, lambda), v.lambda, P_(e, lambda_vee), v.lambda_vee); }
    ++g_cmp["optimize: layout assert (marg order vs aom) never fires"];
    if (info.layout_error) ++g_bad["optimize: layout assert (marg order vs aom) never fires"];
  }
  std::printf("optimize scenarios: %ld cases, %ld ran to completion, %ld real assertion aborts (invalid linearisation), %ld LM steps (%ld accepted, %ld rejected iterations), %ld retries, %ld converged, %ld terminated, %ld with prior >= 60 cols\n",
              cases, ran, aborted, steps, accepted, rejected, retries, conv, term, big);
}


// ================================================================================================ dump replay (patches 0003 + 0004 + 0005)
struct RecStream {
  FILE* f = nullptr;
  explicit RecStream(const std::string& path) { f = std::fopen(path.c_str(), "rb"); }
  ~RecStream() { if (f) std::fclose(f); }
  bool next(Rec& r) {
    uint32_t tag; uint64_t len;
    if (!f || std::fread(&tag, 4, 1, f) != 1 || std::fread(&len, 8, 1, f) != 1) return false;
    r.tag = tag; r.b.resize(len);
    return len == 0 || std::fread(r.b.data(), 1, len, f) == len;
  }
};

struct ProbLm { int64_t id; float dir[2]; float inv; int64_t hf; uint64_t hc; uint32_t nobs; std::vector<bs_obs> obs; };
struct ProbRec {
  uint32_t kind = 0; int64_t t_ns = 0;
  std::vector<bs_frame_pose> poses; std::vector<bs_frame_state> states;
  std::vector<bs_aom_item> aom_items; int aom_total = 0;
  std::vector<bs_imu_meas> imus; std::vector<int64_t> imu_keys;
  float g[3], gw[3], aw[3];
  bool has_marg = false; std::vector<bs_aom_item> mg_items; int mg_total = 0, rows = 0, cols = 0; std::vector<float> mgH, mgb;
  std::vector<int64_t> used, lost; int n_used = -1, n_lost = -1;
  std::vector<ProbLm> lms;
};
static bool parse_problem(const Rec& r, const std::map<int64_t, std::pair<std::array<float, 3>, std::array<float, 3>>>& bias_lin, ProbRec& P) {
  Rd d(r.b);
  P.kind = d.get<uint32_t>(); P.t_ns = d.get<int64_t>();
  P.poses.resize(d.get<uint32_t>());
  for (auto& p : P.poses) { p.t_ns = d.get<int64_t>(); p.linearized = (int)d.get<uint32_t>(); d.arr((float*)&p.lin, 7); d.arr((float*)&p.cur, 7); d.arr(p.delta, 6); }
  P.states.resize(d.get<uint32_t>());
  for (auto& s : P.states) {
    s.t_ns = d.get<int64_t>(); s.s.linearized = (int)d.get<uint32_t>();
    float v[16]; d.arr(v, 16); std::memcpy(s.s.lin.s.q, v, 16); std::memcpy(s.s.lin.s.p, v + 4, 12); std::memcpy(s.s.lin.s.v, v + 7, 12); std::memcpy(s.s.lin.bg, v + 10, 12); std::memcpy(s.s.lin.ba, v + 13, 12);
    d.arr(v, 16); std::memcpy(s.s.cur.s.q, v, 16); std::memcpy(s.s.cur.s.p, v + 4, 12); std::memcpy(s.s.cur.s.v, v + 7, 12); std::memcpy(s.s.cur.bg, v + 10, 12); std::memcpy(s.s.cur.ba, v + 13, 12);
    s.s.lin.s.t_ns = s.t_ns; s.s.cur.s.t_ns = s.t_ns;
    d.arr(s.delta, 15);
  }
  P.aom_items.resize(d.get<uint32_t>());
  for (auto& a : P.aom_items) { a.t_ns = d.get<int64_t>(); a.start = (int)d.get<uint32_t>(); a.size = (int)d.get<uint32_t>(); }
  P.aom_total = (int)d.get<uint32_t>();
  uint32_t nimu = d.get<uint32_t>();
  P.imus.resize(nimu);
  for (uint32_t i = 0; i < nimu; ++i) {
    int64_t key = d.get<int64_t>(); int64_t st = d.get<int64_t>(); int64_t dt = d.get<int64_t>();
    auto bl = bias_lin.find(st);
    if (bl == bias_lin.end()) { std::printf("no BIAS_LIN for imu start %" PRId64 "\n", st); return false; }
    bs_imu_init(&P.imus[i], st, bl->second.first.data(), bl->second.second.data());
    P.imus[i].delta.t_ns = dt;
    d.arr(P.imus[i].delta.q, 4); d.arr(P.imus[i].delta.p, 3); d.arr(P.imus[i].delta.v, 3);
    d.arr(P.imus[i].cov, 81); d.arr(P.imus[i].d_state_d_ba, 27); d.arr(P.imus[i].d_state_d_bg, 27);
    P.imu_keys.push_back(key);
  }
  d.arr(P.g, 3); d.arr(P.gw, 3); d.arr(P.aw, 3);
  P.has_marg = d.get<uint32_t>() != 0;
  if (P.has_marg) {
    d.get<uint32_t>();
    P.mg_items.resize(d.get<uint32_t>());
    for (auto& a : P.mg_items) { a.t_ns = d.get<int64_t>(); a.start = (int)d.get<uint32_t>(); a.size = (int)d.get<uint32_t>(); }
    P.mg_total = (int)d.get<uint32_t>(); P.rows = (int)d.get<uint32_t>(); P.cols = (int)d.get<uint32_t>();
    P.mgH = d.vec<float>((size_t)P.rows * P.cols); P.mgb = d.vec<float>(P.rows);
  }
  { uint32_t n = d.get<uint32_t>(); if (n != 0xFFFFFFFFu) { P.used = d.vec<int64_t>(n); P.n_used = (int)n; } }
  { uint32_t n = d.get<uint32_t>(); if (n != 0xFFFFFFFFu) { P.lost.resize(n); for (auto& v : P.lost) v = (int64_t)d.get<uint64_t>(); P.n_lost = (int)n; } }
  uint32_t nl = d.get<uint32_t>();
  P.lms.resize(nl);
  for (auto& l : P.lms) {
    l.id = (int64_t)d.get<uint64_t>(); d.arr(l.dir, 2); l.inv = d.get<float>(); l.hf = d.get<int64_t>(); l.hc = d.get<uint64_t>(); l.nobs = d.get<uint32_t>();
    l.obs.resize(l.nobs);
    for (auto& o : l.obs) { o.t.frame_id = d.get<int64_t>(); o.t.cam_id = d.get<uint64_t>(); d.arr(o.pos, 2); }
  }
  return true;
}
// the C lmdb (replayed from the op log) must equal the dumped one; the dumped landmark values are installed
static bool lmdb_check_install(bs_lmdb& db, const ProbRec& P, const char* tag) {
  bool ok = (uint32_t)P.lms.size() == db.kpts.nelem;
  const bs_hnode* kn = db.kpts.before_begin.next;
  for (const auto& l : P.lms) {
    bs_keypoint* k = (ok && kn) ? (bs_keypoint*)kn->val : nullptr;
    if (!k || k->id != l.id || k->host.frame_id != l.hf || k->host.cam_id != l.hc || (uint32_t)k->nobs != l.nobs) ok = false;
    if (k && ok) for (uint32_t o = 0; o < l.nobs; ++o) if (k->obs[o].t.frame_id != l.obs[o].t.frame_id || k->obs[o].t.cam_id != l.obs[o].t.cam_id || std::memcmp(k->obs[o].pos, l.obs[o].pos, 8)) ok = false;
    if (k && ok) { k->direction[0] = l.dir[0]; k->direction[1] = l.dir[1]; k->inv_dist = l.inv; }
    if (kn) kn = kn->next;
  }
  ++g_cmp[std::string(tag) + ": C lmdb (op log) == dumped landmarks / observations, kpts order"];
  if (!ok) ++g_bad[std::string(tag) + ": C lmdb (op log) == dumped landmarks / observations, kpts order"];
  return ok;
}
static void vio_set_from_problem(bs_vio& v, const ProbRec& P) {
  free(v.ba.poses); free(v.ba.states); free(v.imu);
  v.ba.poses = nullptr; v.ba.states = nullptr; v.imu = nullptr; v.ba.n_poses = v.ba.n_states = v.n_imu = 0; v.cap_poses = v.cap_states = v.cap_imu = 0;
  for (auto& p : P.poses) *bs_vio_pose_insert(&v, p.t_ns) = p;
  for (auto& s : P.states) *bs_vio_state_insert(&v, s.t_ns) = s;
  for (auto& m : P.imus) *bs_vio_imu_insert(&v, m.start_t_ns) = m;
  std::memcpy(v.g, P.g, 12); std::memcpy(v.gyro_bias_sqrt_weight, P.gw, 12); std::memcpy(v.accel_bias_sqrt_weight, P.aw, 12);
  if (P.has_marg) bs_marg_data_set(&v.marg, P.mg_items.data(), (int)P.mg_items.size(), P.mg_total, P.rows, P.cols, P.mgH.data(), P.mgb.data());
}
static uint64_t vio_state_digest(const bs_vio& v) {
  std::vector<bs_frame_pose> p(v.ba.poses, v.ba.poses + v.ba.n_poses);
  std::vector<bs_frame_state> s(v.ba.states, v.ba.states + v.ba.n_states);
  return state_digest(p, s);
}
static void replay_calib(bs_vio& v) {
  static const double e[2][6] = {{349.7560023050409, 348.72454229977035, 365.89440762590147, 249.32995565708703, -0.2409573942178872, 0.566996899163044},
                                 {361.6713883800533, 360.5856493689301, 379.40818394080867, 255.9772968522045, -0.21300835384809327, 0.5767008625037023}};
  static const double T[2][7] = {{-0.016774788924641532, -0.068938940687127, 0.005139123188382424, -0.007239825785317818, 0.007541278561558601, 0.7017845426564943, 0.7123125505904486},
                                 {-0.01507436282032619, 0.0412627204046637, 0.00316287258752953, -0.0023360576185881624, 0.013000769689092388, 0.7024677108343111, 0.7115930283929829}};
  v.ba.obs_std_dev = 0.5f; v.ba.huber_thresh = 1.0f;
  for (int c = 0; c < 2; ++c) {
    bs_ds_cast_f(&v.ba.cam[c], e[c]);
    bs_quatd q = {T[c][3], T[c][4], T[c][5], T[c][6]};
    bs_se3d sd; bs_so3d_from_quat(&q, &sd.so3);
    sd.t[0] = T[c][0]; sd.t[1] = T[c][1]; sd.t[2] = T[c][2];
    bs_se3_f_from_d(&sd, &v.ba.T_i_c[c]);
  }
}
// apply one lmdb op-log record (tags 20..25) / bias-lin record (28) common to both replays
static bool apply_lmdb_op(bs_lmdb& db, const Rec& r) {
  Rd d(r.b);
  switch (r.tag) {
    case 20: { int64_t id = (int64_t)d.get<uint64_t>(); float dir[2]; d.arr(dir, 2); float inv = d.get<float>(); int64_t hf = d.get<int64_t>(); uint64_t hc = d.get<uint64_t>();
               bs_lmdb_add_landmark(&db, id, dir, inv, bs_tcid{hf, hc}); return true; }
    case 21: { int64_t f = d.get<int64_t>(); uint64_t c = d.get<uint64_t>(); int64_t id = (int64_t)d.get<uint64_t>(); float pos[2]; d.arr(pos, 2);
               if (!bs_lmdb_add_observation(&db, bs_tcid{f, c}, id, pos)) ++g_bad["replay: addObservation of a missing landmark"];
               return true; }
    case 22: bs_lmdb_remove_frame(&db, d.get<int64_t>()); return true;
    case 23: { uint32_t a = d.get<uint32_t>(); auto kf = d.vec<int64_t>(a); uint32_t b2 = d.get<uint32_t>(); auto po = d.vec<int64_t>(b2); uint32_t c = d.get<uint32_t>(); auto st = d.vec<int64_t>(c);
               bs_lmdb_remove_keyframes(&db, kf.data(), (int)a, po.data(), (int)b2, st.data(), (int)c); return true; }
    case 24: bs_lmdb_remove_landmark(&db, (int64_t)d.get<uint64_t>()); return true;
    case 25: { int64_t id = (int64_t)d.get<uint64_t>(); uint32_t n = d.get<uint32_t>(); std::vector<bs_tcid> v(n); for (auto& t : v) { t.frame_id = d.get<int64_t>(); t.cam_id = d.get<uint64_t>(); }
               bs_lmdb_remove_observations(&db, id, v.data(), (int)n); return true; }
  }
  return false;
}

struct IterStep { uint32_t it, j, flags; float lam0, lam1, error_total, after_vi, after_marg, l_diff, f_diff, rel_dec, norminf; uint32_t n; uint64_t hH, hb, hinc, st_pre, st_post, lm_post; };
struct OptEnd { uint32_t it, it_rej, conv, term; float lambda; uint64_t state_hash; };
struct OptBegin { float lambda; uint64_t state_hash; };
struct OptPre { float vee, minl, maxl; double lam_init; int32_t maxit; uint32_t started; };
struct OptPost { float vee, lambda; uint64_t lm_hash; };

struct StepCheck {
  const std::vector<IterStep>* steps; size_t idx = 0; const char* tag; int64_t t_ns;
  bs_vio* v; std::vector<float> Hrow;
  uint64_t lm_digest_pre = 0;
};
static void step_cb(void* ctx, const bs_opt_step* s) {
  StepCheck& c = *(StepCheck*)ctx;
  const std::string T = c.tag;
  if (c.idx >= c.steps->size()) { if (s->phase == 0) ++g_bad[T + ": C step has no dumped ITER_STEP"]; return; }
  const IterStep& d = (*c.steps)[c.idx];
  auto cmp = [&](const char* name, bool equal) { ++g_cmp[T + ": " + name]; if (!equal) { if (++g_bad[T + ": " + name] <= 3) std::printf("MISMATCH %s t=%" PRId64 " it %u j %u\n", name, c.t_ns, d.it, d.j); } };
  if (s->phase == 0) {
    cmp("step identity (it, j)", s->it == (int)d.it && s->j == (int)d.j);
    cmp("lambda before the step", !std::memcmp(&s->lambda_before, &d.lam0, 4));
    cmp("error_total (linearizeProblem)", !std::memcmp(&s->error_total, &d.error_total, 4));
    cmp("state digest before the step", vio_state_digest(*s->vio) == d.st_pre);
    cmp("hash(H), hash(b) of get_dense_H_b", hash_mat(s->H, s->n, s->n) == d.hH && hash_mat(s->b, s->n, 1) == d.hb && (int)d.n == s->n);
  } else {
    cmp("hash(inc) of the LDLT solution (negated)", hash_mat(s->inc, s->n, 1) == d.hinc);
    cmp("l_diff (backSubstitute)", !std::memcmp(&s->l_diff, &d.l_diff, 4));
    cmp("step_norminf", !std::memcmp(&s->step_norminf, &d.norminf, 4));
    cmp("after-update error (computeError + imu + bias)", !std::memcmp(&s->after_vi, &d.after_vi, 4));
    cmp("after-update marg prior error", !std::memcmp(&s->after_marg, &d.after_marg, 4));
    cmp("f_diff, relative_decrease", !std::memcmp(&s->f_diff, &d.f_diff, 4) && !std::memcmp(&s->relative_decrease, &d.rel_dec, 4));
    cmp("accept / reject decision (flags: accepted, step valid)", (uint32_t)((s->accepted ? 1u : 0u) | (s->step_valid ? 2u : 0u)) == d.flags);
    cmp("lambda after the decision", !std::memcmp(&s->lambda_after, &d.lam1, 4));
    cmp("state digest after applyInc", vio_state_digest(*s->vio) == d.st_post);
    cmp("landmark value digest after the step", lm_value_digest(s->vio->ba.lmdb) == d.lm_post);
    ++c.idx;
  }
}

static int replay7(const std::string& dir, long max_problems) {
  const std::string T7 = "replay opt";
  std::map<int64_t, std::vector<IterStep>> steps;
  std::map<int64_t, OptEnd> ends; std::map<int64_t, OptBegin> begins; std::map<int64_t, OptPre> pres; std::map<int64_t, OptPost> posts;
  long n_steps_dump = 0;
  {
    RecStream rs(dir + "/iter.bin"); Rec r;
    if (!rs.f) { std::printf("cannot read %s/iter.bin\n", dir.c_str()); return 2; }
    while (rs.next(r)) {
      Rd d(r.b);
      if (r.tag == 3) {
        IterStep s; int64_t t = d.get<int64_t>();
        s.it = d.get<uint32_t>(); s.j = d.get<uint32_t>(); s.flags = d.get<uint32_t>();
        s.lam0 = d.get<float>(); s.lam1 = d.get<float>(); s.error_total = d.get<float>(); s.after_vi = d.get<float>(); s.after_marg = d.get<float>();
        s.l_diff = d.get<float>(); s.f_diff = d.get<float>(); s.rel_dec = d.get<float>(); s.norminf = d.get<float>();
        s.n = d.get<uint32_t>(); s.hH = d.get<uint64_t>(); s.hb = d.get<uint64_t>(); s.hinc = d.get<uint64_t>(); s.st_pre = d.get<uint64_t>();
        s.st_post = d.get<uint64_t>(); s.lm_post = d.get<uint64_t>();
        steps[t].push_back(s); ++n_steps_dump;
      } else if (r.tag == 6) {
        OptEnd e; int64_t t = d.get<int64_t>(); e.it = d.get<uint32_t>(); e.it_rej = d.get<uint32_t>(); e.conv = d.get<uint32_t>(); e.term = d.get<uint32_t>(); e.lambda = d.get<float>(); e.state_hash = d.get<uint64_t>();
        ends[t] = e;
      } else if (r.tag == 7) {
        OptBegin b; int64_t t = d.get<int64_t>(); for (int i = 0; i < 7; ++i) d.get<uint32_t>(); b.lambda = d.get<float>(); b.state_hash = d.get<uint64_t>();
        begins[t] = b;
      }
    }
  }
  bool have78 = false;
  {
    RecStream rs(dir + "/m78.bin"); Rec r;
    if (rs.f) { have78 = true;
      while (rs.next(r)) {
        Rd d(r.b);
        if (r.tag == 40) { OptPre p; int64_t t = d.get<int64_t>(); p.vee = d.get<float>(); p.minl = d.get<float>(); p.maxl = d.get<float>(); p.lam_init = d.get<double>(); p.maxit = d.get<int32_t>(); p.started = d.get<uint32_t>(); pres[t] = p; }
        else if (r.tag == 41) { OptPost p; int64_t t = d.get<int64_t>(); p.vee = d.get<float>(); p.lambda = d.get<float>(); p.lm_hash = d.get<uint64_t>(); posts[t] = p; }
      }
    }
  }
  if (!have78) { std::printf("no m78.bin in %s (patch 0005 needed for lambda_vee)\n", dir.c_str()); return 2; }

  bs_vio v; bs_vio_init(&v); replay_calib(v);
  std::map<int64_t, std::pair<std::array<float, 3>, std::array<float, 3>>> bias_lin;
  long n_problem = 0, n_calls = 0, n_steps = 0, n_acc = 0, n_rej = 0, max_n = 0, max_np = 0, n_conv = 0, n_term = 0, n_multi_rej = 0;
  RecStream rs(dir + "/m6.bin"); Rec r;
  if (!rs.f) { std::printf("cannot read %s/m6.bin\n", dir.c_str()); return 2; }
  while (rs.next(r)) {
    if (r.tag >= 20 && r.tag <= 25) { apply_lmdb_op(v.ba.lmdb, r); continue; }
    if (r.tag == 28) { Rd d(r.b); int64_t t = d.get<int64_t>(); std::array<float, 3> bg, bac; d.arr(bg.data(), 3); d.arr(bac.data(), 3); bias_lin[t] = {bg, bac}; continue; }
    if (r.tag != 30) continue;
    if (max_problems >= 0 && n_calls >= max_problems) break;
    ProbRec P;
    if (!parse_problem(r, bias_lin, P)) return 2;
    if (!lmdb_check_install(v.ba.lmdb, P, "replay problem")) { ++n_problem; continue; }
    ++n_problem;
    if (P.kind != 0) continue;
    auto st = steps.find(P.t_ns);
    if (st == steps.end()) continue;
    auto pre = pres.find(P.t_ns);
    auto post = posts.find(P.t_ns);
    auto en = ends.find(P.t_ns);
    auto bg = begins.find(P.t_ns);
    if (pre == pres.end() || post == posts.end() || en == ends.end() || bg == begins.end()) { std::printf("missing OPT_PRE/POST/END/BEGIN for t=%" PRId64 "\n", P.t_ns); continue; }
    ++n_calls;
    vio_set_from_problem(v, P);
    v.opt_started = pre->second.started;
    v.lm_lambda_initial = pre->second.lam_init; v.min_lambda = pre->second.minl; v.max_lambda = pre->second.maxl; v.lambda_vee = pre->second.vee; v.max_iterations = pre->second.maxit;
    ++g_cmp[T7 + ": state digest at OPT_BEGIN"];
    if (vio_state_digest(v) != bg->second.state_hash) ++g_bad[T7 + ": state digest at OPT_BEGIN"];
    StepCheck sc; sc.steps = &st->second; sc.tag = "replay opt"; sc.t_ns = P.t_ns; sc.v = &v;
    bs_opt_info info;
    bs_vio_optimize(&v, &info, step_cb, &sc);
    ++g_cmp[T7 + ": number of LM steps"]; if ((size_t)info.steps != st->second.size()) { ++g_bad[T7 + ": number of LM steps"]; std::printf("MISMATCH step count t=%" PRId64 ": C %d dump %zu\n", P.t_ns, info.steps, st->second.size()); }
    ++g_cmp[T7 + ": OPT_END (it, it_rejected, converged, terminated)"];
    if (info.it != (int)en->second.it || info.it_rejected != (int)en->second.it_rej || info.converged != (int)en->second.conv || info.terminated != (int)en->second.term) ++g_bad[T7 + ": OPT_END (it, it_rejected, converged, terminated)"];
    ++g_cmp[T7 + ": OPT_END lambda, final state digest"];
    if (std::memcmp(&v.lambda, &en->second.lambda, 4) || vio_state_digest(v) != en->second.state_hash) ++g_bad[T7 + ": OPT_END lambda, final state digest"];
    ++g_cmp[T7 + ": lambda_vee after, landmark value digest after"];
    if (std::memcmp(&v.lambda_vee, &post->second.vee, 4) || lm_value_digest(v.ba.lmdb) != post->second.lm_hash) ++g_bad[T7 + ": lambda_vee after, landmark value digest after"];
    ++g_cmp[T7 + ": no assertion conditions (invalid linearisation, layout, non-finite)"];
    if (info.invalid_linearization || info.layout_error || info.nonfinite_increment) ++g_bad[T7 + ": no assertion conditions (invalid linearisation, layout, non-finite)"];
    n_steps += info.steps; n_rej += info.it_rejected; n_acc += info.steps - info.it_rejected; n_conv += info.converged; n_term += info.terminated;
    if (info.it_rejected >= 2) ++n_multi_rej;
    max_n = std::max<long>(max_n, v.marg.total + 0); max_np = std::max<long>(max_np, v.ba.n_poses);
  }
  bs_vio_destroy(&v);
  std::printf("replay7 %s: %ld PROBLEM records read, %ld optimize() calls replayed (dump holds %ld LM steps in %zu calls), %ld LM steps (%ld accepted, %ld rejected), %ld calls converged, %ld terminated by max lambda, %ld calls with >= 2 rejections, max prior cols %ld, max key frame poses %ld, bs_la_unsupported %d\n",
              dir.c_str(), n_problem, n_calls, n_steps_dump, steps.size(), n_steps, n_acc, n_rej, n_conv, n_term, n_multi_rej, max_n, max_np, bs_la_unsupported);
  return 0;
}

#ifndef BS_NO_MAIN
int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "ldlt";
  long seed = argc > 2 ? atol(argv[2]) : 1, cases = argc > 3 ? atol(argv[3]) : 1000;
  tbb::global_control tbb_one(tbb::global_control::max_allowed_parallelism, 1);
  Rng r(seed);
  if (mode == "ldlt") ldlt_cases(r, cases);
  if (mode == "lambda") lambda_cases(r, cases);
  if (mode == "prior") prior_cases(r, cases);
  if (mode == "optimize") optimize_cases(r, cases);
  if (mode == "replay") { int rc = replay7(argv[2], argc > 3 ? atol(argv[3]) : -1); if (rc) return rc; }
  long tot = 0, bad = 0;
  for (auto& kv : g_cmp) { std::printf("  %-44s %10ld compared %8ld mismatches\n", kv.first.c_str(), kv.second, g_bad[kv.first]); tot += kv.second; bad += g_bad[kv.first]; }
  std::printf("%s seed %ld cases %ld: %ld comparisons, %ld mismatches\n", mode.c_str(), seed, cases, tot, bad);
  return bad != 0;
}
#endif
