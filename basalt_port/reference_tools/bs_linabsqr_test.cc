// Oracle for basalt_port/c/bs_linabsqr.{h,c}, bs_lmdb.{h,c}.  See usage in main().
#include <basalt/camera/double_sphere_camera.hpp>
#include <basalt/imu/preintegration.h>
#include <basalt/linearization/imu_block.hpp>
#include <basalt/linearization/linearization_abs_qr.hpp>
#include <basalt/vi_estimator/ba_base.h>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <random>
#include <string>
#include <vector>
#include <array>
#include <algorithm>
#include <basalt/vi_estimator/sc_ba_base.h>
#include <basalt/linearization/linearization_base.hpp>
#include <basalt/utils/ba_utils.h>
#include <tbb/global_control.h>
extern "C" {
#include "bs_linabsqr.h"
#include "bs_lmdb.h"
}
using basalt::TimeCamId;
typedef float S;
typedef Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic> MatX;
typedef Eigen::Matrix<S, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RMatX;
typedef Eigen::Matrix<S, Eigen::Dynamic, 1> VecX;
typedef std::mt19937_64 Rng;
static double urand(Rng& r, double a, double b) { return a + (b - a) * std::uniform_real_distribution<double>(0, 1)(r); }
static double nrand(Rng& r) { return std::normal_distribution<double>(0, 1)(r); }
static std::map<std::string, long> g_cmp, g_bad;
static int g_shape[3];
static int g_ld = 0;
static void check(const char* name, const void* a, const void* b, size_t nbytes, long ctx) {
  ++g_cmp[name];
  if (std::memcmp(a, b, nbytes) != 0) {
    if (++g_bad[name] <= 3) {
      size_t i = 0;
      while (i < nbytes / 4 && std::memcmp((const char*)a + 4 * i, (const char*)b + 4 * i, 4) == 0) ++i;
      std::printf("MISMATCH %s ctx %ld shape m%d n%d k%d first diff idx %zu (row %zu col %zu ld %d)\n", name, ctx, g_shape[0], g_shape[1], g_shape[2], i, g_ld ? i % g_ld : 0, g_ld ? i / g_ld : 0, g_ld);
    }
  }
}
static S rv(Rng& r, int mode) {
  switch (mode) {
    case 0: return (S)nrand(r);
    case 1: return (S)(nrand(r) * std::exp(urand(r, -6, 6)));
    default: return (S)(std::round(nrand(r) * 4) / 4);
  }
}

// ---- dense primitives: the exact expressions of the linearisation ----
static void prim(Rng& r, long cases) {
  for (long c = 0; c < cases; ++c) {
    int mode = r() % 3;
    int m = 2 + r() % 99, n = 2 + r() % 99, k = 2 + r() % 99;
    g_shape[0] = m; g_shape[1] = n; g_shape[2] = k; g_ld = m;
    // gemm: C += J^T * J with J a (k x m) block of a RowMajor storage with padded columns (the landmark block)
    {
      int pad = ((m + 3) / 4) * 4;
      int ncols = pad + 4;
      RMatX stor(k, ncols);
      for (int i = 0; i < k; ++i) for (int j = 0; j < ncols; ++j) stor(i, j) = rv(r, mode);
      auto J = stor.block(0, 0, k, m);
      MatX H(m, m), H0;
      for (int i = 0; i < m * m; ++i) H.data()[i] = rv(r, mode);
      H0 = H;
      if (k + 2 * m >= 20) {
        H.noalias() += J.transpose() * J;
        MatX Hc = H0;
        bs_la_t_gemm(m, m, k, stor.data(), 1, ncols, stor.data(), ncols, 1, Hc.data(), m);
        check("gemm J^T*J +=", H.data(), Hc.data(), sizeof(S) * m * m, c);
      }
    }
    // gemm: dense col-major A (m x k) * B (k x n) assigned (dst zero + gemm) and accumulated
    {
      MatX A(m, k), B(k, n), C0(m, n);
      for (int i = 0; i < m * k; ++i) A.data()[i] = rv(r, mode);
      for (int i = 0; i < k * n; ++i) B.data()[i] = rv(r, mode);
      for (int i = 0; i < m * n; ++i) C0.data()[i] = rv(r, mode);
      if (k + m + n >= 20) {
        MatX C1 = A * B;
        MatX Cc = MatX::Zero(m, n);
        bs_la_t_gemm(m, n, k, A.data(), 1, m, B.data(), 1, k, Cc.data(), m);
        check("gemm A*B assign", C1.data(), Cc.data(), sizeof(S) * m * n, c);
        MatX C2 = C0;
        C2.noalias() += A.transpose().transpose() * B;
        MatX Cd = C0;
        bs_la_t_gemm(m, n, k, A.data(), 1, m, B.data(), 1, k, Cd.data(), m);
        check("gemm += A*B", C2.data(), Cd.data(), sizeof(S) * m * n, c);
        // A^T * B form with A (k x m) col-major: lhs row-major view
        MatX At(k, m);
        for (int i = 0; i < k * m; ++i) At.data()[i] = rv(r, mode);
        MatX C3 = At.transpose() * B;
        MatX Ce = MatX::Zero(m, n);
        bs_la_t_gemm(m, n, k, At.data(), k, 1, B.data(), 1, k, Ce.data(), m);
        check("gemm A^T*B assign", C3.data(), Ce.data(), sizeof(S) * m * n, c);
      }
    }
    // gemv col-major: y = A x  (evalTo) and y += A x, A dynamic col-major m x k
    {
      MatX A(m, k);
      VecX x(k), y0(m);
      for (int i = 0; i < m * k; ++i) A.data()[i] = rv(r, mode);
      for (int i = 0; i < k; ++i) x[i] = rv(r, mode);
      for (int i = 0; i < m; ++i) y0[i] = rv(r, mode);
      VecX y1 = A * x;
      VecX yc = VecX::Zero(m);
      bs_la_t_gemv(m, k, A.data(), 1, m, x.data(), 1, yc.data());
      check("gemv colmajor assign", y1.data(), yc.data(), sizeof(S) * m, c);
      VecX y2 = y0;
      y2.noalias() += A * x;
      VecX yd = y0;
      bs_la_t_gemv(m, k, A.data(), 1, m, x.data(), 1, yd.data());
      check("gemv colmajor +=", y2.data(), yd.data(), sizeof(S) * m, c);
      // J^T * r with J a RowMajor block (k x m): row-major-transposed = col-major view with stride ncols
      int ncols = ((m + 3) / 4) * 4 + 4;
      RMatX stor(k, ncols);
      for (int i = 0; i < k; ++i) for (int j = 0; j < ncols; ++j) stor(i, j) = rv(r, mode);
      auto J = stor.block(0, 0, k, m);
      VecX rr(k);
      for (int i = 0; i < k; ++i) rr[i] = rv(r, mode);
      VecX b0 = y0, b1 = y0;
      b0.noalias() += J.transpose() * rr;
      bs_la_t_gemv(m, k, stor.data(), 1, ncols, rr.data(), 1, b1.data());
      check("gemv J^T*r +=", b0.data(), b1.data(), sizeof(S) * m, c);
      // row-major kernel: y = Jrow (m x k RowMajor block) * x
      RMatX R(m, ((k + 3) / 4) * 4 + 4);
      for (int i = 0; i < R.rows(); ++i) for (int j = 0; j < R.cols(); ++j) R(i, j) = rv(r, mode);
      auto Rb = R.block(0, 0, m, k);
      VecX y3 = Rb * x;
      VecX ye = VecX::Zero(m);
      bs_la_t_gemv(m, k, R.data(), (int)R.cols(), 1, x.data(), 1, ye.data());
      check("gemv rowmajor assign", y3.data(), ye.data(), sizeof(S) * m, c);
      // row-major kernel through a transposed col-major matrix: y = A^T x'
      VecX xp(m);
      for (int i = 0; i < m; ++i) xp[i] = rv(r, mode);
      VecX y4 = A.transpose() * xp;
      VecX yf = VecX::Zero(k);
      bs_la_t_gemv(k, m, A.data(), m, 1, xp.data(), 1, yf.data());
      check("gemv A^T x (rowmajor kernel)", y4.data(), yf.data(), sizeof(S) * k, c);
    }
  }
}


// ---- LandmarkDatabase: random estimator-like operation sequences; contents and both iteration orders vs the real class ----
typedef basalt::LandmarkDatabase<float> RealDb;
static bool same_db(const RealDb& a, const bs_lmdb& b, long ctx) {
  bool ok = true;
  // kpts order + contents
  const bs_hnode* n = b.kpts.before_begin.next;
  size_t cnt = 0;
  for (const auto& kv : a.getLandmarks()) {
    if (!n) { ok = false; break; }
    const bs_keypoint* k = (const bs_keypoint*)n->val;
    const auto& v = kv.second;
    bool e = (int64_t)kv.first == n->k0 && k->id == (int64_t)kv.first && std::memcmp(v.direction.data(), k->direction, 8) == 0 && std::memcmp(&v.inv_dist, &k->inv_dist, 4) == 0 &&
             v.host_kf_id.frame_id == k->host.frame_id && v.host_kf_id.cam_id == k->host.cam_id && (int)v.obs.size() == k->nobs;
    if (e) {
      int i = 0;
      for (const auto& o : v.obs) {
        if (o.first.frame_id != k->obs[i].t.frame_id || o.first.cam_id != k->obs[i].t.cam_id || std::memcmp(o.second.data(), k->obs[i].pos, 8) != 0) { e = false; break; }
        ++i;
      }
    }
    if (!e) { ok = false; break; }
    n = n->next;
    ++cnt;
  }
  if (n || cnt != b.kpts.nelem || a.numLandmarks() != b.kpts.nelem) ok = false;
  ++g_cmp["lmdb kpts order+content"];
  if (b.kpts.nelem > g_cmp["(max landmarks in a db)"]) g_cmp["(max landmarks in a db)"] = b.kpts.nelem;
  if (b.observations.nelem > g_cmp["(max host frames in a db)"]) g_cmp["(max host frames in a db)"] = b.observations.nelem;
  if (!ok) { ++g_bad["lmdb kpts order+content"]; if (g_bad["lmdb kpts order+content"] <= 5) std::printf("MISMATCH lmdb kpts ctx %ld (sizes %zu/%zu)\n", ctx, a.numLandmarks(), b.kpts.nelem); }
  // observations order + contents
  bool ok2 = true;
  const bs_hnode* m = b.observations.before_begin.next;
  for (const auto& kv : a.getObservations()) {
    if (!m) { ok2 = false; break; }
    const bs_host* h = (const bs_host*)m->val;
    if ((int64_t)kv.first.frame_id != m->k0 || (int64_t)kv.first.cam_id != m->k1 || (int)kv.second.size() != h->n) { ok2 = false; break; }
    int ti = 0;
    for (const auto& tg : kv.second) {
      const bs_tgt& t = h->tgt[ti++];
      if (tg.first.frame_id != t.t.frame_id || tg.first.cam_id != t.t.cam_id || (int)tg.second.size() != t.n) { ok2 = false; break; }
      int ii = 0;
      for (auto id : tg.second) if ((int64_t)id != t.ids[ii++]) { ok2 = false; break; }
      if (!ok2) break;
    }
    if (!ok2) break;
    m = m->next;
  }
  if (m || a.getObservations().size() != b.observations.nelem || a.numObservations() != bs_lmdb_num_observations(&b)) ok2 = false;
  ++g_cmp["lmdb observations order+content"];
  if (!ok2) { ++g_bad["lmdb observations order+content"]; if (g_bad["lmdb observations order+content"] <= 5) std::printf("MISMATCH lmdb observations ctx %ld\n", ctx); }
  return ok && ok2;
}

static void lmdb_scenarios(Rng& r, long scenarios) {
  using basalt::TimeCamId;
  for (long sc = 0; sc < scenarios; ++sc) {
    RealDb a;
    bs_lmdb b;
    bs_lmdb_init(&b);
    int64_t t0 = 1403636579763555584LL + (int64_t)(r() % 100000) * 1000;
    int64_t dt = (r() % 2) ? 50000000LL : 33333333LL;
    int ncam = 1 + r() % 2;
    int nframes = 8 + r() % 60;
    long next_id = (long)(r() % 1000);
    std::vector<int64_t> frames;
    std::vector<long> live;       // landmark ids
    std::vector<int64_t> kf;      // key frames
    int64_t id_stride = 1 + r() % 3;
    int max_new = 5 + r() % 80;
    for (int f = 0; f < nframes; ++f) {
      int64_t ft = t0 + (int64_t)f * dt;
      frames.push_back(ft);
      // observe existing landmarks in the new frame
      std::vector<long> all = live;
      for (long id : all) {
        if (r() % 100 < 80) {
          for (int c = 0; c < ncam; ++c) if (r() % 100 < 85) {
            float pos[2] = {(float)urand(r, 0, 752), (float)urand(r, 0, 480)};
            basalt::KeypointObservation<float> o; o.kpt_id = (int)id; o.pos = Eigen::Vector2f(pos[0], pos[1]);
            a.addObservation(TimeCamId(ft, c), o);
            bs_lmdb_add_observation(&b, bs_tcid{ft, (uint64_t)c}, id, pos);
          }
        }
      }
      // new landmarks hosted by this frame (cam 0 mostly)
      int nnew = r() % (max_new + 1);
      for (int i = 0; i < nnew; ++i) {
        next_id += 1 + (long)(r() % id_stride);
        basalt::Keypoint<float> kp;
        kp.direction = Eigen::Vector2f((float)urand(r, -1, 1), (float)urand(r, -1, 1));
        kp.inv_dist = (float)urand(r, 0.01, 2);
        int hc = (r() % 8 == 0 && ncam > 1) ? 1 : 0;
        kp.host_kf_id = TimeCamId(ft, hc);
        a.addLandmark((int)next_id, kp);
        float dir[2] = {kp.direction[0], kp.direction[1]};
        bs_lmdb_add_landmark(&b, next_id, dir, kp.inv_dist, bs_tcid{ft, (uint64_t)hc});
        live.push_back(next_id);
        for (int c = 0; c < ncam; ++c) if (c == hc || r() % 100 < 70) {
          float pos[2] = {(float)urand(r, 0, 752), (float)urand(r, 0, 480)};
          basalt::KeypointObservation<float> o; o.kpt_id = (int)next_id; o.pos = Eigen::Vector2f(pos[0], pos[1]);
          a.addObservation(TimeCamId(ft, c), o);
          bs_lmdb_add_observation(&b, bs_tcid{ft, (uint64_t)c}, next_id, pos);
        }
      }
      if (r() % 3 == 0) kf.push_back(ft);
      same_db(a, b, sc * 1000 + f);
      // maintenance
      int action = r() % 8;
      if (action == 0 && frames.size() > 4) {                 // removeFrame on an old frame
        int64_t fr = frames[r() % (frames.size() - 2)];
        bs_lmdb_remove_frame(&b, fr);
        if (bs_lmdb_ub) { bs_lmdb_ub = 0; ++g_cmp["lmdb scenarios cut at C++ UB"]; goto scenario_end; }
        a.removeFrame(fr);
      } else if (action == 1 && frames.size() > 5) {          // marginalisation-like removeKeyframes
        std::set<int64_t> kfs, poses, states;
        std::vector<int64_t> vk, vp, vs;
        int nk = r() % 3, np = r() % 3, ns = r() % 3;
        for (int i = 0; i < nk; ++i) { int64_t x = frames[r() % frames.size()]; if (kfs.insert(x).second) vk.push_back(x); }
        for (int i = 0; i < np; ++i) { int64_t x = frames[r() % frames.size()]; if (poses.insert(x).second) vp.push_back(x); }
        for (int i = 0; i < ns; ++i) { int64_t x = frames[r() % frames.size()]; if (states.insert(x).second) vs.push_back(x); }
        bs_lmdb_remove_keyframes(&b, vk.data(), (int)vk.size(), vp.data(), (int)vp.size(), vs.data(), (int)vs.size());
        if (bs_lmdb_ub) { bs_lmdb_ub = 0; ++g_cmp["lmdb scenarios cut at C++ UB"]; goto scenario_end; }
        a.removeKeyframes(kfs, poses, states);
      } else if (action == 2 && !live.empty()) {              // removeLandmark (lost / outlier) incl. non-existing ids
        for (int i = 0, n = 1 + r() % 6; i < n; ++i) {
          long id = live[r() % live.size()];
          bs_lmdb_remove_landmark(&b, id);
          if (bs_lmdb_ub) { bs_lmdb_ub = 0; ++g_cmp["lmdb scenarios cut at C++ UB"]; goto scenario_end; }
          a.removeLandmark((int)id);
        }
      } else if (action == 3 && !live.empty()) {              // removeObservations
        long id = live[r() % live.size()];
        if (a.landmarkExists((int)id)) {
          std::set<TimeCamId> obs;
          std::vector<bs_tcid> vo;
          const auto& kp = a.getLandmark((int)id);
          for (const auto& o : kp.obs) if (r() % 3 == 0) { obs.insert(o.first); vo.push_back(bs_tcid{o.first.frame_id, o.first.cam_id}); }
          bs_lmdb_remove_observations(&b, id, vo.data(), (int)vo.size());
          if (bs_lmdb_ub) { bs_lmdb_ub = 0; ++g_cmp["lmdb scenarios cut at C++ UB"]; goto scenario_end; }
          a.removeObservations((int)id, obs);
        }
      }
      // keep the live list in sync with the db
      std::vector<long> nl;
      for (long id : live) if (a.landmarkExists((int)id)) nl.push_back(id);
      live = nl;
      if (!same_db(a, b, sc * 1000 + f + 500)) { bs_lmdb_destroy(&b); return; }
      // backup / restore round trip on landmark values
      if (r() % 10 == 0) {
        a.backup(); bs_lmdb_backup(&b);
        for (auto& kv : const_cast<Eigen::aligned_unordered_map<basalt::KeypointId, basalt::Keypoint<float>>&>(a.getLandmarks())) { kv.second.direction += Eigen::Vector2f(0.5f, -0.25f); kv.second.inv_dist += 1.0f; }
        for (bs_hnode* nn = b.kpts.before_begin.next; nn; nn = nn->next) { bs_keypoint* k = (bs_keypoint*)nn->val; k->direction[0] += 0.5f; k->direction[1] += -0.25f; k->inv_dist += 1.0f; }
        if (r() % 2) { a.restore(); bs_lmdb_restore(&b); }
        same_db(a, b, sc * 1000 + f + 700);
      }
    }
  scenario_end:
    bs_lmdb_destroy(&b);
  }
}

// ================================================================================================ full problem oracle
typedef Eigen::Matrix<S, 3, 1> V3;
typedef Sophus::SE3<S> SE3f;
typedef Sophus::SO3<S> SO3f;

static bs_se3f to_c(const SE3f& T) {
  bs_se3f c;
  std::memcpy(&c.so3, T.unit_quaternion().coeffs().data(), 16);
  std::memcpy(c.t, T.translation().data(), 12);
  return c;
}
static bs_pvbstate to_c(const basalt::PoseVelBiasState<S>& s) {
  bs_pvbstate c;
  c.s.t_ns = s.t_ns;
  std::memcpy(c.s.q, s.T_w_i.unit_quaternion().coeffs().data(), 16);
  std::memcpy(c.s.p, s.T_w_i.translation().data(), 12);
  std::memcpy(c.s.v, s.vel_w_i.data(), 12);
  std::memcpy(c.bg, s.bias_gyro.data(), 12);
  std::memcpy(c.ba, s.bias_accel.data(), 12);
  return c;
}

struct Problem {
  basalt::BundleAdjustmentBase<S> ba;       // the real estimator base: frame_poses, frame_states, lmdb, calib
  bs_ba cba;
  std::vector<bs_frame_pose> cposes;
  std::vector<bs_frame_state> cstates;
  basalt::AbsOrderMap aom;
  std::vector<bs_aom_item> caom_items;
  bs_aom caom;
  bool has_marg = false;
  basalt::MargLinData<S> marg;
  bs_marg_lin cmarg;
  std::vector<bs_aom_item> cmarg_items;
  bool has_imu = false;
  Eigen::aligned_map<int64_t, basalt::IntegratedImuMeasurement<S>> imu;   // the estimator's imu_meas map
  std::vector<bs_imu_meas> cimu;                                  // all, in key order
  std::vector<bs_imu_meas*> cimu_ptrs, cimu_sel;
  V3 g, gyro_sw, accel_sw;
  bs_imu_lin cild;
  std::vector<int64_t> used, lost;
  bool use_used = false, use_lost = false;
  Problem() { bs_ba_init(&cba); }
  ~Problem() { bs_ba_destroy(&cba); }
};

static SE3f rand_pose(Rng& r, double tr, double rot) {
  Eigen::Vector3d ax(nrand(r), nrand(r), nrand(r));
  ax.normalize();
  return SE3f(SO3f::exp((ax * urand(r, 0, rot)).cast<S>()), Eigen::Vector3d(nrand(r) * tr, nrand(r) * tr, nrand(r) * tr).cast<S>());
}

static int g_no_imu = 0, g_no_marg = 0;
static void build_problem(Rng& r, Problem& P, int scale_class) {
  using basalt::PoseStateWithLin;
  using basalt::PoseVelBiasStateWithLin;
  auto& ba = P.ba;
  const double e[2][6] = {{349.7560023050409, 348.72454229977037, 365.89440762590149, 249.32995565708704, -0.2409573942178872, 0.566996899163044},
                          {361.6713883800533, 360.5856493689301, 379.40818394080869, 255.9772968522045, -0.21300835384809328, 0.5767008625037023}};
  ba.obs_std_dev = (r() % 4 == 0) ? (S)urand(r, 0.2, 2.0) : 0.5f;
  ba.huber_thresh = (r() % 8 == 0) ? 0.0f : ((r() % 4 == 0) ? (S)urand(r, 0.3, 3.0) : 1.0f);
  for (int c = 0; c < 2; ++c) {
    basalt::DoubleSphereCamera<S>::VecN pv;
    for (int i = 0; i < 6; ++i) pv[i] = (S)e[c][i];
    basalt::GenericCamera<S> gc;
    gc.variant = basalt::DoubleSphereCamera<S>(pv);
    ba.calib.intrinsics.push_back(gc);
    bs_ds_cast_f(&P.cba.cam[c], e[c]);
  }
  ba.calib.T_i_c.push_back(SE3f(SO3f::exp(V3(1.57f, 0.01f, 0.0f)), V3(-0.017f, -0.069f, 0.005f)));
  ba.calib.T_i_c.push_back(SE3f(SO3f::exp(V3(1.55f, -0.01f, 0.02f)), V3(-0.015f, 0.041f, 0.003f)));
  for (int c = 0; c < 2; ++c) P.cba.T_i_c[c] = to_c(ba.calib.T_i_c[c]);
  P.cba.obs_std_dev = ba.obs_std_dev;
  P.cba.huber_thresh = ba.huber_thresh;

  // frames
  const int np = (int)(r() % (scale_class == 0 ? 3 : 8));              // key frame poses
  const int ns = 2 + (int)(r() % (scale_class == 0 ? 2 : 3));          // frame states (>= 2 for IMU)
  const int64_t t0 = 1403636579763555584LL + (int64_t)(r() % 100000) * 1000;
  const int64_t dt = 50000000LL;
  std::vector<int64_t> pose_t, state_t;
  int64_t tt = t0;
  for (int i = 0; i < np; ++i) { tt += dt * (1 + r() % 4); pose_t.push_back(tt); }
  for (int i = 0; i < ns; ++i) { tt += dt * (1 + r() % 2); state_t.push_back(tt); }
  SE3f T = rand_pose(r, 1.0, 0.5);
  std::vector<SE3f> pose_T;
  for (size_t i = 0; i < pose_t.size() + state_t.size(); ++i) { T = T * rand_pose(r, 0.15, 0.08); pose_T.push_back(T); }
  const int lin_mode = (int)(r() % 4);        // 0: all poses linearized, 1: random, 2: none, 3: all
  for (int i = 0; i < np; ++i) {
    bool lin = lin_mode == 0 || lin_mode == 3 || (lin_mode == 1 && r() % 2);
    PoseStateWithLin<S> ps(pose_t[i], pose_T[i], lin);
    if (r() % 3 != 0) { basalt::PoseState<S>::VecN inc; for (int k = 0; k < 6; ++k) inc[k] = (S)(nrand(r) * (r() % 2 ? 0.01 : 0.1)); ps.applyInc(inc); }
    ba.frame_poses[pose_t[i]] = ps;
    bs_frame_pose c;
    c.t_ns = pose_t[i]; c.linearized = ps.isLinearized(); c.lin = to_c(ps.getPoseLin()); c.cur = to_c(ps.getPose());
    std::memcpy(c.delta, ps.getDelta().data(), 24);
    P.cposes.push_back(c);
  }
  for (int i = 0; i < ns; ++i) {
    bool lin = lin_mode == 3 || (lin_mode == 1 && r() % 2) || (lin_mode == 0 && i + 1 < ns && r() % 2);
    V3 vel(nrand(r), nrand(r), nrand(r)), bg(urand(r, -0.05, 0.05), urand(r, -0.05, 0.05), urand(r, -0.05, 0.05)), bac(urand(r, -0.3, 0.3), urand(r, -0.3, 0.3), urand(r, -0.3, 0.3));
    PoseVelBiasStateWithLin<S> st(state_t[i], pose_T[np + i], vel, bg, bac, lin);
    if (r() % 3 != 0) { basalt::PoseVelBiasState<S>::VecN inc; for (int k = 0; k < 15; ++k) inc[k] = (S)(nrand(r) * (r() % 2 ? 0.005 : 0.05)); st.applyInc(inc); }
    ba.frame_states[state_t[i]] = st;
    bs_frame_state c;
    c.t_ns = state_t[i];
    c.s.linearized = st.isLinearized();
    c.s.lin = to_c(st.getStateLin());
    c.s.cur = to_c(st.getState());
    std::memcpy(c.delta, st.getDelta().data(), 60);
    P.cstates.push_back(c);
  }
  P.cba.poses = P.cposes.data(); P.cba.n_poses = (int)P.cposes.size();
  P.cba.states = P.cstates.data(); P.cba.n_states = (int)P.cstates.size();

  // IMU measurements between consecutive states
  P.g = V3(0, 0, -9.81f);
  P.gyro_sw = V3(1e4f, 1e4f, 1e4f) * (S)(r() % 2 ? 1.0 : urand(r, 0.3, 3));
  P.accel_sw = V3(1e3f, 1e3f, 1e3f) * (S)(r() % 2 ? 1.0 : urand(r, 0.3, 3));
  V3 acc_cov, gyr_cov;
  acc_cov.setConstant((S)std::pow(0.016 * std::sqrt(200.0), 2));
  gyr_cov.setConstant((S)std::pow(0.000282 * std::sqrt(200.0), 2));
  P.has_imu = ns >= 2 && (r() % 8 != 0) && !g_no_imu;
  if (P.has_imu) {
    for (int i = 0; i + 1 < ns; ++i) {
      const int64_t a = state_t[i], b = state_t[i + 1];
      const auto& sl = ba.frame_states.at(a).getStateLin();
      basalt::IntegratedImuMeasurement<S> m(a, sl.bias_gyro, sl.bias_accel);
      bs_imu_meas cm;
      bs_imu_init(&cm, a, sl.bias_gyro.data(), sl.bias_accel.data());
      int64_t t = a;
      while (t < b) {
        int64_t step = 5000000 + (int64_t)(r() % 2000001) - 1000000;
        t += step;
        if (t > b) t = b;
        basalt::ImuData<S> d;
        d.t_ns = t;
        for (int k = 0; k < 3; ++k) {
          d.gyro[k] = (S)(std::sin(0.01 * (double)(t / 1000000) + k) * 0.5 + nrand(r) * 0.05);
          d.accel[k] = (S)((k == 2 ? 9.81 : 0) + std::cos(0.02 * (double)(t / 1000000) + k) * 1.5 + nrand(r) * 0.1);
        }
        m.integrate(d, acc_cov, gyr_cov);
        bs_imudata cd; cd.t_ns = d.t_ns; std::memcpy(cd.accel, d.accel.data(), 12); std::memcpy(cd.gyro, d.gyro.data(), 12);
        bs_imu_integrate(&cm, &cd, acc_cov.data(), gyr_cov.data());
      }
      P.imu.emplace(a, m);
      P.cimu.push_back(cm);
    }
    for (auto& c : P.cimu) P.cimu_ptrs.push_back(&c);
  }

  // aom: poses first (ascending), then states; abs_order_map is keyed by time
  int total = 0;
  std::vector<int64_t> layout;
  for (auto t : pose_t) { P.aom.abs_order_map[t] = std::make_pair(total, 6); total += 6; P.aom.items++; layout.push_back(t); }
  for (auto t : state_t) { P.aom.abs_order_map[t] = std::make_pair(total, 15); total += 15; P.aom.items++; layout.push_back(t); }
  P.aom.total_size = total;
  // optionally take a pose frame out of the aom (marginalisation-like: observations to it are dropped)
  int64_t out_frame = -1;
  if (np >= 2 && r() % 3 == 0) {
    out_frame = pose_t[r() % np];
  }
  std::vector<int64_t> in_aom_frames, all_frames;
  for (auto t : pose_t) all_frames.push_back(t);
  for (auto t : state_t) all_frames.push_back(t);
  for (auto t : all_frames) if (t != out_frame) in_aom_frames.push_back(t);
  if (out_frame >= 0) {
    // rebuild a dense aom without the frame (keeps C++ invariants: offsets contiguous in layout order)
    P.aom = basalt::AbsOrderMap();
    total = 0;
    for (auto t : pose_t) if (t != out_frame) { P.aom.abs_order_map[t] = std::make_pair(total, 6); total += 6; P.aom.items++; }
    for (auto t : state_t) { P.aom.abs_order_map[t] = std::make_pair(total, 15); total += 15; P.aom.items++; }
    P.aom.total_size = total;
  }
  for (const auto& kv : P.aom.abs_order_map) P.caom_items.push_back(bs_aom_item{kv.first, kv.second.first, kv.second.second});
  P.caom.item = P.caom_items.data(); P.caom.n = (int)P.caom_items.size(); P.caom.total_size = (int)P.aom.total_size;

  // landmarks
  const int nlm = scale_class == 0 ? 3 + (int)(r() % 12) : (int)(r() % 220) + 3;
  long id = (long)(r() % 5000);
  const double noise_px = (r() % 3 == 0) ? 0.2 : ((r() % 2) ? 0.7 : 3.0);
  std::vector<int64_t> lm_ids;
  for (int l = 0; l < nlm; ++l) {
    id += 1 + (long)(r() % 3);
    basalt::Keypoint<S> kp;
    kp.direction = Eigen::Vector2f((S)urand(r, -0.9, 0.9), (S)urand(r, -0.9, 0.9));
    kp.inv_dist = (S)(r() % 20 == 0 ? urand(r, 0, 0.001) : urand(r, 0.05, 2.0));
    const int64_t host_t = in_aom_frames[r() % in_aom_frames.size()];
    kp.host_kf_id = TimeCamId(host_t, (r() % 4 == 0) ? 1 : 0);
    ba.lmdb.addLandmark((int)id, kp);
    const float dir[2] = {kp.direction[0], kp.direction[1]};
    bs_lmdb_add_landmark(&P.cba.lmdb, id, dir, kp.inv_dist, bs_tcid{host_t, kp.host_kf_id.cam_id});
    lm_ids.push_back(id);
    int added = 0;
    std::vector<TimeCamId> targets;
    targets.push_back(kp.host_kf_id);
    for (auto ft : all_frames) for (int c = 0; c < 2; ++c) if (!(ft == host_t && (size_t)c == kp.host_kf_id.cam_id) && r() % 100 < 45) targets.push_back(TimeCamId(ft, c));
    if (targets.size() < 2) targets.push_back(TimeCamId(all_frames[r() % all_frames.size()], 1 - kp.host_kf_id.cam_id));
    for (const auto& tc : targets) {
      // observation = projection of the landmark + noise (a sample of outliers)
      Eigen::Vector2f obs(0, 0);
      {
        Eigen::Matrix<S, 4, 4> Tth;
        if (tc == kp.host_kf_id) Tth.setIdentity();
        else {
          auto sh = ba.getPoseStateWithLin(kp.host_kf_id.frame_id), stt = ba.getPoseStateWithLin(tc.frame_id);
          Tth = basalt::computeRelPose<S>(sh.getPose(), ba.calib.T_i_c[kp.host_kf_id.cam_id], stt.getPose(), ba.calib.T_i_c[tc.cam_id]).matrix();
        }
        Eigen::Vector2f res;
        const auto& cam = std::get<basalt::DoubleSphereCamera<S>>(ba.calib.intrinsics[tc.cam_id].variant);
        bool v = basalt::linearizePoint<S, basalt::DoubleSphereCamera<S>>(Eigen::Vector2f::Zero(), kp, Tth, cam, res);
        if (v) obs = res; else obs = Eigen::Vector2f((S)urand(r, 0, 752), (S)urand(r, 0, 480));
        double sc = (r() % 15 == 0) ? 15.0 : noise_px;
        obs += Eigen::Vector2f((S)(nrand(r) * sc), (S)(nrand(r) * sc));
        if (r() % 60 == 0) obs = Eigen::Vector2f((S)urand(r, -100, 900), (S)urand(r, -100, 600));
      }
      basalt::KeypointObservation<S> o; o.kpt_id = (int)id; o.pos = obs;
      ba.lmdb.addObservation(tc, o);
      const float pos[2] = {obs[0], obs[1]};
      bs_lmdb_add_observation(&P.cba.lmdb, bs_tcid{tc.frame_id, tc.cam_id}, id, pos);
      ++added;
    }
  }

  // marginalisation prior over a prefix of the layout (items must be linearized)
  P.has_marg = (r() % 4 != 0) && !g_no_marg;
  if (P.has_marg) {
    int q = 0, cols = 0;
    std::vector<std::pair<int64_t, std::pair<int, int>>> pref;
    for (auto t : layout) {
      if (t == out_frame) continue;
      bool islin = ba.frame_poses.count(t) ? ba.frame_poses.at(t).isLinearized() : ba.frame_states.at(t).isLinearized();
      if (!islin) break;
      pref.push_back({t, P.aom.abs_order_map.at(t)});
      cols += P.aom.abs_order_map.at(t).second;
      ++q;
      if (r() % 3 == 0) break;
    }
    if (q == 0) P.has_marg = false;
    else {
      P.marg.is_sqrt = true;
      for (auto& it : pref) P.marg.order.abs_order_map[it.first] = it.second;
      P.marg.order.total_size = cols; P.marg.order.items = q;
      const int rows = cols + (int)(r() % 12) - ((r() % 3 == 0) ? 3 : 0);
      int rr = rows < 1 ? 1 : rows;
      if (rr + 2 * cols < 20) rr = 20 - 2 * cols;   // tiny products take Eigen's coefficient-based path (not modelled, not on the executed path)
      P.marg.H.resize(rr, cols); P.marg.b.resize(rr);
      const double hs = std::exp(urand(r, -3, 3));
      for (int i = 0; i < rr * cols; ++i) P.marg.H.data()[i] = (S)(nrand(r) * hs);
      for (int i = 0; i < rr; ++i) P.marg.b[i] = (S)(nrand(r) * hs);
      for (auto& kv : P.marg.order.abs_order_map) P.cmarg_items.push_back(bs_aom_item{kv.first, kv.second.first, kv.second.second});
      P.cmarg.order.item = P.cmarg_items.data(); P.cmarg.order.n = (int)P.cmarg_items.size(); P.cmarg.order.total_size = cols;
      P.cmarg.rows = rr; P.cmarg.cols = cols; P.cmarg.H = P.marg.H.data(); P.cmarg.b = P.marg.b.data();
    }
  }
  // lost landmarks / used frames (marginalisation flavour)
  if (r() % 3 == 0) {
    P.use_used = true;
    for (auto t : all_frames) if (r() % 3 == 0) P.used.push_back(t);
    if (r() % 2) { P.use_lost = true; for (auto lid : lm_ids) if (r() % 10 == 0) P.lost.push_back(lid); }
  } else if (r() % 6 == 0) {
    P.use_lost = true; for (auto lid : lm_ids) if (r() % 10 == 0) P.lost.push_back(lid);
  }
  // ild: pairs fully inside the aom
  P.cild.n = 0;
  if (P.has_imu) {
    for (size_t i = 0; i < P.cimu.size(); ++i) {
      const int64_t a = P.cimu[i].start_t_ns, b = a + P.cimu[i].delta.t_ns;
      if (P.aom.abs_order_map.count(a) && P.aom.abs_order_map.count(b)) P.cimu_sel.push_back(&P.cimu[i]);
    }
    std::memcpy(P.cild.g, P.g.data(), 12); std::memcpy(P.cild.gyro_bias_weight_sqrt, P.gyro_sw.data(), 12); std::memcpy(P.cild.accel_bias_weight_sqrt, P.accel_sw.data(), 12);
    P.cild.n = (int)P.cimu_sel.size(); P.cild.meas = P.cimu_sel.data();
  }
}

static void problem_scenarios(Rng& r, long scenarios, int verbose) {
  long invalid = 0, with_marg = 0, with_imu = 0, nblocks = 0, nobs_total = 0, dropped = 0, partial = 0;
  for (long sc = 0; sc < scenarios; ++sc) {
    Problem P;
    build_problem(r, P, (int)(r() % 4 == 0 ? 0 : 1));
    auto& ba = P.ba;
    if (P.has_marg) ++with_marg;
    if (P.cild.n > 0) ++with_imu;
    basalt::LinearizationBase<S, 6>::Options opt;
    opt.lb_options.huber_parameter = ba.huber_thresh;
    opt.lb_options.obs_std_dev = ba.obs_std_dev;
    opt.linearization_type = basalt::LinearizationType::ABS_QR;
    basalt::ImuLinData<S> ild = {P.g, P.gyro_sw, P.accel_sw, {}};
    for (auto& kv : P.imu) {
      const int64_t a = kv.second.get_start_t_ns(), b = a + kv.second.get_dt_ns();
      if (P.aom.abs_order_map.count(a) && P.aom.abs_order_map.count(b)) ild.imu_meas[kv.first] = &kv.second;
    }
    std::set<int64_t> used(P.used.begin(), P.used.end());
    std::unordered_set<basalt::KeypointId> lost;
    for (auto l : P.lost) lost.insert((basalt::KeypointId)l);
    basalt::LinearizationAbsQR<S, 6> lqr(&ba, P.aom, opt, P.has_marg ? &P.marg : nullptr, P.has_imu ? &ild : nullptr, P.use_used ? &used : nullptr,
                                         P.use_lost ? &lost : nullptr);
    bs_linabsqr* la = bs_la_create(&P.cba, &P.caom, P.has_marg ? &P.cmarg : nullptr, P.has_imu ? &P.cild : nullptr, P.use_used ? P.used.data() : nullptr,
                                   P.use_used ? (int)P.used.size() : -1, P.use_lost ? P.lost.data() : nullptr, P.use_lost ? (int)P.lost.size() : -1);
    long ctx = sc;
    // landmark order
    {
      std::vector<int64_t> exp;
      for (const auto& kv : ba.lmdb.getLandmarks()) {
        const auto& v = kv.second;
        if (P.use_used || P.use_lost) {
          if (P.use_used && used.count(v.host_kf_id.frame_id)) exp.push_back(kv.first);
          else if (P.use_lost && lost.count(kv.first)) exp.push_back(kv.first);
        } else exp.push_back(kv.first);
      }
      std::vector<int64_t> got;
      for (int i = 0; i < bs_la_num_landmark_blocks(la); ++i) got.push_back(bs_la_landmark_id(la, i));
      ++g_cmp["landmark_ids"];
      if (exp != got) { ++g_bad["landmark_ids"]; if (g_bad["landmark_ids"] < 4) std::printf("MISMATCH landmark_ids ctx %ld\n", ctx); }
      nblocks += (long)got.size();
    }
    bool valid_real = true;
    int valid_c = 1;
    S e_real = lqr.linearizeProblem(&valid_real);
    S e_c = bs_la_linearize_problem(la, &valid_c);
    check("lin.error", &e_real, &e_c, 4, ctx);
    int vr = valid_real, vc = valid_c;
    check("lin.valid", &vr, &vc, 4, ctx);
    if (!valid_real) ++invalid;
    lqr.performQR();
    bs_la_perform_qr(la);
    const int n = P.aom.total_size;
    MatX H; VecX b;
    lqr.get_dense_H_b(H, b);
    std::vector<S> cH((size_t)n * n), cb(n);
    bs_la_get_dense_H_b(la, cH.data(), cb.data());
    g_shape[0] = n; g_shape[1] = n; g_shape[2] = bs_la_num_landmark_blocks(la); g_ld = n;
    if ((int)H.rows() != n || (int)b.size() != n) { std::printf("size mismatch\n"); return; }
    check("H", H.data(), cH.data(), 4 * (size_t)n * n, ctx);
    g_ld = 1;
    check("b", b.data(), cb.data(), 4 * (size_t)n, ctx);
    MatX Q2Jp; VecX Q2r;
    lqr.get_dense_Q2Jp_Q2r(Q2Jp, Q2r);
    const int rows = bs_la_dense_Q2_rows(la);
    if ((int)Q2Jp.rows() != rows) { std::printf("Q2 rows mismatch %d vs %d\n", (int)Q2Jp.rows(), rows); ++g_bad["Q2Jp"]; }
    else {
      std::vector<S> cQ((size_t)rows * n), cr(rows);
      bs_la_get_dense_Q2Jp_Q2r(la, cQ.data(), cr.data());
      g_ld = rows;
      check("Q2Jp", Q2Jp.data(), cQ.data(), 4 * (size_t)rows * n, ctx);
      g_ld = 1;
      check("Q2r", Q2r.data(), cr.data(), 4 * (size_t)rows, ctx);
    }
    // estimator-side evaluation at the current state (before the update)
    {
      S er = 0; ba.computeError(er);
      S ec = bs_ba_compute_error(&P.cba);
      check("computeError", &er, &ec, 4, ctx);
    }
    if (P.has_marg) {
      S mr = 0; ba.computeMargPriorError(P.marg, mr);
      S mc = bs_ba_marg_prior_error(&P.cba, &P.cmarg);
      check("computeMargPriorError", &mr, &mc, 4, ctx);
      VecX inc(P.marg.H.cols());
      for (int i = 0; i < inc.size(); ++i) inc[i] = (S)(nrand(r) * 0.01);
      S cr_ = ba.computeMargPriorModelCostChange(P.marg, VecX(), inc);
      S cc_ = bs_ba_marg_prior_model_cost_change(&P.cba, &P.cmarg, inc.data());
      check("computeMargPriorModelCostChange", &cr_, &cc_, 4, ctx);
    }
    if (P.has_imu) {
      S ie = 0, be = 0, ae = 0;
      basalt::ScBundleAdjustmentBase<S>::computeImuError(P.aom, ie, be, ae, ba.frame_states, P.imu, P.gyro_sw.array().square(), P.accel_sw.array().square(), P.g);
      V3 gw = P.gyro_sw.array().square(), aw = P.accel_sw.array().square();
      S ci = 0, cb_ = 0, ca = 0;
      bs_ba_compute_imu_error(&P.cba, &P.caom, P.cimu_ptrs.data(), (int)P.cimu_ptrs.size(), P.g.data(), gw.data(), aw.data(), &ci, &cb_, &ca);
      S x[3] = {ie, be, ae}, y[3] = {ci, cb_, ca};
      check("computeImuError", x, y, 12, ctx);
    }
    // back substitution with a random pose increment
    VecX inc(n);
    const double isc = std::exp(urand(r, -7, -1));
    for (int i = 0; i < n; ++i) inc[i] = (S)(nrand(r) * isc);
    S ld_real = lqr.backSubstitute(inc);
    S ld_c = bs_la_back_substitute(la, inc.data());
    if (std::isnan(ld_real) && std::isnan(ld_c)) ++g_cmp["(l_diff NaN from a singular landmark R, any NaN accepted)"];   // NaN payload / sign depends on the compiler's operand order
    else check("backsub.l_diff", &ld_real, &ld_c, 4, ctx);
    {
      bool ok = true;
      const bs_hnode* nn = P.cba.lmdb.kpts.before_begin.next;
      for (const auto& kv : ba.lmdb.getLandmarks()) {
        const bs_keypoint* k = (const bs_keypoint*)nn->val;
        for (int q = 0; q < 3; ++q) {
          const S a = q < 2 ? kv.second.direction[q] : kv.second.inv_dist, c = q < 2 ? k->direction[q] : k->inv_dist;
          if (std::memcmp(&a, &c, 4) != 0) {
            if (std::isnan(a) && std::isnan(c)) ++g_cmp["(landmarks with NaN from a singular R, any NaN accepted)"];
            else ok = false;
          }
        }
        nn = nn->next;
      }
      ++g_cmp["backsub.landmarks"];
      if (!ok) { ++g_bad["backsub.landmarks"]; if (g_bad["backsub.landmarks"] < 4) std::printf("MISMATCH backsub.landmarks ctx %ld\n", ctx); }
    }
    {
      S er = 0; ba.computeError(er);
      S ec = bs_ba_compute_error(&P.cba);
      check("computeError (after update)", &er, &ec, 4, ctx);
    }
    if (P.has_marg) {
      VecX d1; ba.computeDelta(P.marg.order, d1);
      std::vector<S> d2(P.marg.order.total_size);
      bs_ba_compute_delta(&P.cba, &P.cmarg.order, d2.data());
      check("computeDelta", d1.data(), d2.data(), 4 * d2.size(), ctx);
    }
    for (const auto& kv : ba.lmdb.getLandmarks()) nobs_total += kv.second.obs.size();
    {   // coverage: valid projections, Huber-active residuals, observations dropped from the aom, same-frame other-camera observations
      for (const auto& kv : ba.lmdb.getLandmarks()) {
        const auto& kp = kv.second;
        for (const auto& o : kp.obs) {
          ++g_cmp["(coverage) observations"];
          if (!P.aom.abs_order_map.count(o.first.frame_id)) { ++g_cmp["(coverage) obs dropped from aom"]; continue; }
          if (o.first.frame_id == kp.host_kf_id.frame_id && o.first.cam_id != kp.host_kf_id.cam_id) ++g_cmp["(coverage) same frame, other camera"];
          Eigen::Matrix<S, 4, 4> Tth;
          if (o.first == kp.host_kf_id) Tth.setIdentity();
          else Tth = basalt::computeRelPose<S>(ba.getPoseStateWithLin(kp.host_kf_id.frame_id).getPose(), ba.calib.T_i_c[kp.host_kf_id.cam_id],
                                               ba.getPoseStateWithLin(o.first.frame_id).getPose(), ba.calib.T_i_c[o.first.cam_id]).matrix();
          Eigen::Vector2f res;
          const auto& cam = std::get<basalt::DoubleSphereCamera<S>>(ba.calib.intrinsics[o.first.cam_id].variant);
          if (basalt::linearizePoint<S, basalt::DoubleSphereCamera<S>>(o.second, kp, Tth, cam, res)) {
            ++g_cmp["(coverage) valid projections"];
            if (ba.huber_thresh > 0 && res.squaredNorm() > ba.huber_thresh * ba.huber_thresh) ++g_cmp["(coverage) Huber-active (res > threshold)"];
          }
        }
      }
      if (P.aom.total_size >= 48) ++g_cmp["(coverage) problems with n >= 48 (GEMM blocking heuristic)"];
      if (P.aom.total_size >= 87) ++g_cmp["(coverage) problems with n >= 87"];
    }
    bs_la_destroy(la);
    (void)dropped; (void)partial;
  }
  std::printf("problems: %ld scenarios, %ld with marg prior, %ld with IMU blocks, %ld landmark blocks, %ld observations, %ld numerically invalid\n", scenarios, with_marg, with_imu,
              nblocks, nobs_total, invalid);
}

// ---- statements of LandmarkBlockAbsDynamic::backSubstitute / performQRHouseholder on arbitrary row-major storage (the exact C++ statements) ----
static void prim_back(Rng& r, long cases) {
  typedef Eigen::Matrix<S, 3, 1> Vec3;
  for (long c = 0; c < cases; ++c) {
    const int mode = (int)(r() % 3);
    const int nobs = 2 + (int)(r() % 30);
    const int num_rows = 2 * nobs + 3;
    const int P = 6 + (int)(r() % 82);
    const int pad = (4 - P % 4) % 4, lm_idx = P + pad, res_idx = lm_idx + 3, nc = res_idx + 1;
    g_shape[0] = num_rows; g_shape[1] = nc; g_shape[2] = P;
    RMatX storage(num_rows, nc);
    for (int i = 0; i < num_rows; ++i) for (int j = 0; j < nc; ++j) storage(i, j) = rv(r, mode);
    for (int i = 0; i < 3; ++i) storage(i, lm_idx + i) = (S)((r() % 2 ? 1 : -1) * urand(r, 0.2, 3.0));
    for (int i = 1; i < 3; ++i) for (int j = 0; j < i; ++j) storage(i, lm_idx + j) = 0;      // upper triangular as after the QR (the lower part is not read)
    VecX pose_inc(P);
    for (int i = 0; i < P; ++i) pose_inc[i] = rv(r, mode) * 0.01f;
    const S l0 = (S)nrand(r);
    RMatX st2 = storage;
    {
      const auto Q1Jl = storage.template block<3, 3>(0, lm_idx).template triangularView<Eigen::Upper>();
      const auto Q1Jr = storage.col(res_idx).template head<3>();
      const auto Q1Jp = storage.topLeftCorner(3, P);
      Vec3 inc = -Q1Jl.solve(Q1Jr + Q1Jp * pose_inc);
      VecX QJinc = storage.topLeftCorner(num_rows - 3, P) * pose_inc;
      QJinc.template head<3>() += Q1Jl * inc;
      auto Qr = storage.col(res_idx).head(num_rows - 3);
      S l_diff = l0;
      l_diff -= QJinc.transpose() * (S(0.5) * QJinc + Qr);
      S dir[2] = {(S)nrand(r), (S)nrand(r)}, inv = (S)urand(r, -0.2, 1.5);
      S dir_c[2] = {dir[0], dir[1]}, inv_c = inv;
      S inc_c[3], q3_c[3];
      S l_c = bs_la_t_back_substitute(st2.data(), num_rows, nc, P, lm_idx, pose_inc.data(), l0, dir_c, &inv_c, inc_c, q3_c);
      dir[0] += inc[0]; dir[1] += inc[1]; inv = std::max(S(0), inv + inc[2]);
      check("stmt backSubstitute inc", inc.data(), inc_c, 12, c);
      check("stmt backSubstitute QJinc.head<3>", QJinc.data(), q3_c, 12, c);
      check("stmt backSubstitute l_diff", &l_diff, &l_c, 4, c);
      check("stmt backSubstitute landmark", dir, dir_c, 8, c);
      check("stmt backSubstitute inv_dist", &inv, &inv_c, 4, c);
    }
    // performQRHouseholder
    {
      RMatX a = storage, b2 = storage;
      if (r() % 6 == 0) {   // exact zero tails exercise the tau == 0 branch
        const int k = (int)(r() % 3);
        for (int i = k + 1; i < num_rows - 3; ++i) { a(i, lm_idx + k) = 0; b2(i, lm_idx + k) = 0; }
      }
      VecX tempVector1(nc);
      VecX tempVector2(num_rows - 3);
      for (size_t k = 0; k < 3; ++k) {
        size_t remainingRows = num_rows - k - 3;
        S beta, tau;
        a.col(lm_idx + k).segment(k, remainingRows).makeHouseholder(tempVector2, tau, beta);
        a.block(k, 0, remainingRows, nc).applyHouseholderOnTheLeft(tempVector2, tau, tempVector1.data());
      }
      bs_la_t_householder_qr(b2.data(), num_rows, nc, P, lm_idx);
      check("stmt performQRHouseholder", a.data(), b2.data(), 4 * (size_t)num_rows * nc, c);
    }
  }
}
static int aom_start(const bs_aom& a, int64_t t) { for (int i = 0; i < a.n; ++i) if (a.item[i].t_ns == t) return a.item[i].start; return -1; }

// ================================================================================================ replay of the M0 / M6 dumps
// Reads <dir>/m6.bin (lmdb op log, order digests, PROBLEM records), <dir>/iter.bin (OPT_BEGIN / ITER_STEP / OPT_END) and <dir>/marg.bin (MARG), all
// from one reference run with BASALT_PORT_DUMP_DIR, BASALT_PORT_DUMP_FULL=1, BASALT_PORT_M6=1 (patches 0003 + 0004).
#include <cinttypes>
struct Rec { uint32_t tag; std::vector<uint8_t> b; };
static bool read_records(const std::string& path, std::vector<Rec>& out) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  uint32_t tag; uint64_t len;
  while (std::fread(&tag, 4, 1, f) == 1 && std::fread(&len, 8, 1, f) == 1) {
    Rec r; r.tag = tag; r.b.resize(len);
    if (len && std::fread(r.b.data(), 1, len, f) != len) break;
    out.push_back(std::move(r));
  }
  std::fclose(f);
  return true;
}
struct Rd {
  const std::vector<uint8_t>& b; size_t o = 0;
  explicit Rd(const std::vector<uint8_t>& x) : b(x) {}
  template <class T> T get() { T v; std::memcpy(&v, &b[o], sizeof(T)); o += sizeof(T); return v; }
  template <class T> void arr(T* p, size_t n) { std::memcpy(p, &b[o], sizeof(T) * n); o += sizeof(T) * n; }
  template <class T> std::vector<T> vec(size_t n) { std::vector<T> v(n); arr(v.data(), n); return v; }
};

static uint64_t fnv_(const void* p, size_t n, uint64_t h) { const uint8_t* c = (const uint8_t*)p; for (size_t i = 0; i < n; i++) { h ^= c[i]; h *= 1099511628211ull; } return h; }
static const uint64_t FNV0 = 1469598103934665603ull;
template <class T> static uint64_t fnvT_(const T& v, uint64_t h) { return fnv_(&v, sizeof(T), h); }
static uint64_t hash_se3(const bs_se3f& T, uint64_t h) { h = fnv_(&T.so3, 16, h); return fnv_(T.t, 12, h); }
static uint64_t state_digest(const std::vector<bs_frame_pose>& poses, const std::vector<bs_frame_state>& states) {
  uint64_t h = FNV0;
  for (const auto& p : poses) {
    int64_t k = p.t_ns; h = fnvT_(k, h);
    uint8_t lin = p.linearized; h = fnvT_(lin, h);
    h = hash_se3(p.linearized ? p.cur : p.lin, h);
    h = hash_se3(p.lin, h);
    h = fnv_(p.delta, 24, h);
  }
  for (const auto& s : states) {
    int64_t k = s.t_ns; h = fnvT_(k, h);
    uint8_t lin = s.s.linearized; h = fnvT_(lin, h);
    const bs_pvbstate* two[2] = {s.s.linearized ? &s.s.cur : &s.s.lin, &s.s.lin};
    for (const bs_pvbstate* st : two) {
      bs_se3f T; std::memcpy(&T.so3, st->s.q, 16); std::memcpy(T.t, st->s.p, 12);
      h = hash_se3(T, h);
      h = fnv_(st->s.v, 12, h); h = fnv_(st->bg, 12, h); h = fnv_(st->ba, 12, h);
    }
    h = fnv_(s.delta, 60, h);
  }
  return h;
}
static uint64_t lm_value_digest(const bs_lmdb& db) {
  std::vector<const bs_keypoint*> ks;
  for (const bs_hnode* n = db.kpts.before_begin.next; n; n = n->next) ks.push_back((const bs_keypoint*)n->val);
  std::sort(ks.begin(), ks.end(), [](const bs_keypoint* a, const bs_keypoint* b) { return (uint64_t)a->id < (uint64_t)b->id; });
  uint64_t h = FNV0;
  for (const auto* k : ks) {
    uint64_t id = (uint64_t)k->id; int64_t hf = k->host.frame_id; uint64_t hc = k->host.cam_id;
    h = fnvT_(id, h); h = fnvT_(hf, h); h = fnvT_(hc, h);
    h = fnv_(k->direction, 8, h);
    h = fnvT_(k->inv_dist, h);
  }
  return h;
}
static uint64_t hash_mat(const float* d, int64_t r, int64_t c) { uint64_t h = FNV0; h = fnvT_(r, h); h = fnvT_(c, h); return fnv_(d, 4 * (size_t)(r * c), h); }

struct StepRec {
  int64_t t_ns; uint32_t it, j, flags; float lam0, lam1, error_total, after_vi, after_marg, l_diff, f_diff, rel_dec, norminf;
  uint32_t n; uint64_t hH, hb, hinc, st_pre, st_post, lm_post; bool full; std::vector<float> H, b, inc;
};
struct MargRec {
  uint32_t qrows = 0, qcols = 0; bool full = false; std::vector<float> Q2Jp, Q2r; uint64_t hQ = 0, hr = 0;
};

static int replay(const std::string& dir, long max_problems) {
  std::vector<Rec> m6, iter, marg;
  if (!read_records(dir + "/m6.bin", m6)) { std::printf("cannot read %s/m6.bin\n", dir.c_str()); return 2; }
  read_records(dir + "/iter.bin", iter);
  read_records(dir + "/marg.bin", marg);
  std::map<int64_t, std::vector<StepRec>> steps;
  for (const Rec& r : iter) {
    if (r.tag != 3) continue;
    Rd d(r.b); StepRec s;
    s.t_ns = d.get<int64_t>(); s.it = d.get<uint32_t>(); s.j = d.get<uint32_t>(); s.flags = d.get<uint32_t>();
    s.lam0 = d.get<float>(); s.lam1 = d.get<float>(); s.error_total = d.get<float>(); s.after_vi = d.get<float>(); s.after_marg = d.get<float>();
    s.l_diff = d.get<float>(); s.f_diff = d.get<float>(); s.rel_dec = d.get<float>(); s.norminf = d.get<float>();
    s.n = d.get<uint32_t>(); s.hH = d.get<uint64_t>(); s.hb = d.get<uint64_t>(); s.hinc = d.get<uint64_t>(); s.st_pre = d.get<uint64_t>();
    s.st_post = d.get<uint64_t>(); s.lm_post = d.get<uint64_t>(); s.full = d.get<uint32_t>() != 0;
    if (s.full) { s.H = d.vec<float>((size_t)s.n * s.n); s.b = d.vec<float>(s.n); s.inc = d.vec<float>(s.n); }
    steps[s.t_ns].push_back(std::move(s));
  }
  std::map<int64_t, MargRec> margs;
  for (const Rec& r : marg) {
    if (r.tag != 4) continue;
    Rd d(r.b); MargRec m;
    int64_t t = d.get<int64_t>(); d.get<int64_t>();
    d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>(); d.get<uint32_t>();
    uint32_t na = d.get<uint32_t>(); d.o += (size_t)na * 16;
    uint32_t nk = d.get<uint32_t>(); d.o += 4 * (size_t)nk;
    uint32_t nm = d.get<uint32_t>(); d.o += 4 * (size_t)nm;
    uint32_t nkm = d.get<uint32_t>(); d.o += 8 * (size_t)nkm;
    uint32_t nka = d.get<uint32_t>(); d.o += 8 * (size_t)nka;
    uint32_t prow = d.get<uint32_t>(), pcol = d.get<uint32_t>(); d.get<uint64_t>(); d.get<uint64_t>();
    m.qrows = d.get<uint32_t>(); m.qcols = d.get<uint32_t>(); m.hQ = d.get<uint64_t>(); m.hr = d.get<uint64_t>();
    uint32_t hnr = d.get<uint32_t>(), hnc = d.get<uint32_t>(); d.get<uint64_t>(); uint32_t nb = d.get<uint32_t>(); d.get<uint64_t>(); d.get<uint64_t>(); d.get<uint32_t>();
    m.full = d.get<uint32_t>() != 0;
    if (m.full) {
      d.o += 4 * ((size_t)prow * pcol + prow);
      m.Q2Jp = d.vec<float>((size_t)m.qrows * m.qcols); m.Q2r = d.vec<float>(m.qrows);
      (void)hnr; (void)hnc; (void)nb;
    }
    margs[t] = std::move(m);
  }

  // calibration of the EuRoC run (euroc_ds_calib.json, cast to float as calib.cast<float>())
  // generated from basalt_src/data/euroc_ds_calib.json by python repr (exact doubles)
  static const double e[2][6] = {{349.7560023050409, 348.72454229977035, 365.89440762590147, 249.32995565708703, -0.2409573942178872, 0.566996899163044},
                                 {361.6713883800533, 360.5856493689301, 379.40818394080867, 255.9772968522045, -0.21300835384809327, 0.5767008625037023}};
  static const double T[2][7] = {{-0.016774788924641532, -0.068938940687127, 0.005139123188382424, -0.007239825785317818, 0.007541278561558601, 0.7017845426564943, 0.7123125505904486},
                                 {-0.01507436282032619, 0.0412627204046637, 0.00316287258752953, -0.0023360576185881624, 0.013000769689092388, 0.7024677108343111, 0.7115930283929829}};
  bs_ba ba;
  bs_ba_init(&ba);
  ba.obs_std_dev = 0.5f; ba.huber_thresh = 1.0f;
  for (int c = 0; c < 2; ++c) {
    bs_ds_cast_f(&ba.cam[c], e[c]);
    bs_quatd q = {T[c][3], T[c][4], T[c][5], T[c][6]};
    bs_se3d sd; bs_so3d_from_quat(&q, &sd.so3);
    sd.t[0] = T[c][0]; sd.t[1] = T[c][1]; sd.t[2] = T[c][2];
    bs_se3_f_from_d(&sd, &ba.T_i_c[c]);
  }
  std::map<int64_t, std::pair<std::array<float, 3>, std::array<float, 3>>> bias_lin;
  long n_order = 0, n_unconn = 0, n_problem = 0;
  long st_max_n = 0, st_max_obs = 0, st_min_prior = 1 << 30, st_min_prior_cols = 1 << 30, st_max_blocks = 0, st_max_q2rows = 0, st_max_imu = 0, st_max_lm = 0;
  for (const Rec& r : m6) {
    Rd d(r.b);
    switch (r.tag) {
      case 20: { int64_t id = (int64_t)d.get<uint64_t>(); float dir[2]; d.arr(dir, 2); float inv = d.get<float>(); int64_t hf = d.get<int64_t>(); uint64_t hc = d.get<uint64_t>();
                 bs_lmdb_add_landmark(&ba.lmdb, id, dir, inv, bs_tcid{hf, hc}); break; }
      case 21: { int64_t f = d.get<int64_t>(); uint64_t c = d.get<uint64_t>(); int64_t id = (int64_t)d.get<uint64_t>(); float pos[2]; d.arr(pos, 2);
                 if (!bs_lmdb_add_observation(&ba.lmdb, bs_tcid{f, c}, id, pos)) ++g_bad["replay: addObservation of a missing landmark"]; break; }
      case 22: bs_lmdb_remove_frame(&ba.lmdb, d.get<int64_t>()); break;
      case 23: { uint32_t a = d.get<uint32_t>(); auto kf = d.vec<int64_t>(a); uint32_t b2 = d.get<uint32_t>(); auto po = d.vec<int64_t>(b2); uint32_t c = d.get<uint32_t>(); auto st = d.vec<int64_t>(c);
                 bs_lmdb_remove_keyframes(&ba.lmdb, kf.data(), (int)a, po.data(), (int)b2, st.data(), (int)c); break; }
      case 24: bs_lmdb_remove_landmark(&ba.lmdb, (int64_t)d.get<uint64_t>()); break;
      case 25: { int64_t id = (int64_t)d.get<uint64_t>(); uint32_t n = d.get<uint32_t>(); std::vector<bs_tcid> v(n); for (auto& t : v) { t.frame_id = d.get<int64_t>(); t.cam_id = d.get<uint64_t>(); }
                 bs_lmdb_remove_observations(&ba.lmdb, id, v.data(), (int)n); break; }
      case 26: {   // ORDER: the C model's kpts / observations iteration order vs the real unordered_map order at LinearizationBase::create
        d.get<uint32_t>(); uint32_t nl = d.get<uint32_t>(), nh = d.get<uint32_t>(); uint64_t hl = d.get<uint64_t>(), hh = d.get<uint64_t>();
        uint64_t cl = FNV0, ch = FNV0; uint32_t cnl = 0, cnh = 0;
        for (const bs_hnode* n = ba.lmdb.kpts.before_begin.next; n; n = n->next) { uint64_t id = (uint64_t)n->k0; cl = fnvT_(id, cl); ++cnl; }
        for (const bs_hnode* n = ba.lmdb.observations.before_begin.next; n; n = n->next) { int64_t f = n->k0; uint64_t c = (uint64_t)n->k1; ch = fnvT_(f, ch); ch = fnvT_(c, ch); ++cnh; }
        ++g_cmp["replay order: landmark (kpts) iteration order"]; if (cl != hl || cnl != nl) { if (++g_bad["replay order: landmark (kpts) iteration order"] <= 3) std::printf("MISMATCH kpts order at ORDER record %ld (n %u/%u)\n", n_order, cnl, nl); }
        ++g_cmp["replay order: host (observations) iteration order"]; if (ch != hh || cnh != nh) { if (++g_bad["replay order: host (observations) iteration order"] <= 3) std::printf("MISMATCH host order at ORDER record %ld\n", n_order); }
        ++n_order; break; }
      case 27: {   // UNCONN: unordered_set<int> iteration order
        uint32_t n = d.get<uint32_t>(); auto seq = d.vec<int32_t>(n); uint32_t m = d.get<uint32_t>(); auto ord = d.vec<int32_t>(m);
        bs_htab t; bs_htab_init(&t, BS_HK_U64);
        for (int32_t v : seq) { int ins; bs_htab_insert(&t, (int64_t)v, 0, &ins); }
        std::vector<int32_t> got;
        for (const bs_hnode* x = t.before_begin.next; x; x = x->next) got.push_back((int32_t)x->k0);
        bs_htab_destroy(&t, nullptr);
        ++g_cmp["replay order: unordered_set<int> unconnected_obs0"];
        if (got != ord) ++g_bad["replay order: unordered_set<int> unconnected_obs0"];
        ++n_unconn; break; }
      case 28: { int64_t t = d.get<int64_t>(); std::array<float, 3> bg, bac; d.arr(bg.data(), 3); d.arr(bac.data(), 3); bias_lin[t] = {bg, bac}; break; }
      case 30: {   // PROBLEM
        if (max_problems >= 0 && n_problem >= max_problems) break;
        const uint32_t kind = d.get<uint32_t>();
        const int64_t t_ns = d.get<int64_t>();
        std::vector<bs_frame_pose> poses(d.get<uint32_t>());
        for (auto& p : poses) { p.t_ns = d.get<int64_t>(); p.linearized = (int)d.get<uint32_t>(); d.arr((float*)&p.lin, 7); d.arr((float*)&p.cur, 7); d.arr(p.delta, 6); }
        std::vector<bs_frame_state> states(d.get<uint32_t>());
        for (auto& s : states) {
          s.t_ns = d.get<int64_t>(); s.s.linearized = (int)d.get<uint32_t>();
          float v[16]; d.arr(v, 16); std::memcpy(s.s.lin.s.q, v, 16); std::memcpy(s.s.lin.s.p, v + 4, 12); std::memcpy(s.s.lin.s.v, v + 7, 12); std::memcpy(s.s.lin.bg, v + 10, 12); std::memcpy(s.s.lin.ba, v + 13, 12);
          d.arr(v, 16); std::memcpy(s.s.cur.s.q, v, 16); std::memcpy(s.s.cur.s.p, v + 4, 12); std::memcpy(s.s.cur.s.v, v + 7, 12); std::memcpy(s.s.cur.bg, v + 10, 12); std::memcpy(s.s.cur.ba, v + 13, 12);
          s.s.lin.s.t_ns = s.t_ns; s.s.cur.s.t_ns = s.t_ns;
          d.arr(s.delta, 15);
        }
        std::vector<bs_aom_item> aom_items(d.get<uint32_t>());
        for (auto& a : aom_items) { a.t_ns = d.get<int64_t>(); a.start = (int)d.get<uint32_t>(); a.size = (int)d.get<uint32_t>(); }
        bs_aom aom; aom.item = aom_items.data(); aom.n = (int)aom_items.size(); aom.total_size = (int)d.get<uint32_t>();
        uint32_t nimu = d.get<uint32_t>();
        std::vector<bs_imu_meas> imus(nimu);
        std::vector<bs_imu_meas*> imup(nimu);
        for (uint32_t i = 0; i < nimu; ++i) {
          int64_t key = d.get<int64_t>(); int64_t st = d.get<int64_t>(); int64_t dt = d.get<int64_t>(); (void)key;
          auto bl = bias_lin.find(st);
          if (bl == bias_lin.end()) { std::printf("no BIAS_LIN for imu start %" PRId64 "\n", st); return 2; }
          bs_imu_init(&imus[i], st, bl->second.first.data(), bl->second.second.data());
          imus[i].delta.t_ns = dt;
          d.arr(imus[i].delta.q, 4); d.arr(imus[i].delta.p, 3); d.arr(imus[i].delta.v, 3);
          d.arr(imus[i].cov, 81); d.arr(imus[i].d_state_d_ba, 27); d.arr(imus[i].d_state_d_bg, 27);
          imup[i] = &imus[i];
        }
        bs_imu_lin ild; ild.n = (int)nimu; ild.meas = imup.data();
        d.arr(ild.g, 3); d.arr(ild.gyro_bias_weight_sqrt, 3); d.arr(ild.accel_bias_weight_sqrt, 3);
        bool has_marg = d.get<uint32_t>() != 0;
        bs_marg_lin mg; std::vector<bs_aom_item> mg_items; std::vector<float> mgH, mgb;
        if (has_marg) {
          bool is_sqrt = d.get<uint32_t>() != 0; (void)is_sqrt;
          mg_items.resize(d.get<uint32_t>());
          for (auto& a : mg_items) { a.t_ns = d.get<int64_t>(); a.start = (int)d.get<uint32_t>(); a.size = (int)d.get<uint32_t>(); }
          mg.order.item = mg_items.data(); mg.order.n = (int)mg_items.size(); mg.order.total_size = (int)d.get<uint32_t>();
          mg.rows = (int)d.get<uint32_t>(); mg.cols = (int)d.get<uint32_t>();
          mgH = d.vec<float>((size_t)mg.rows * mg.cols); mgb = d.vec<float>(mg.rows);
          mg.H = mgH.data(); mg.b = mgb.data();
        }
        std::vector<int64_t> used, lost; int n_used = -1, n_lost = -1;
        { uint32_t n = d.get<uint32_t>(); if (n != 0xFFFFFFFFu) { used = d.vec<int64_t>(n); n_used = (int)n; } }
        { uint32_t n = d.get<uint32_t>(); if (n != 0xFFFFFFFFu) { lost.resize(n); for (auto& v : lost) v = (int64_t)d.get<uint64_t>(); n_lost = (int)n; } }
        // landmarks: structure vs the replayed op log, values taken from the record
        uint32_t nl = d.get<uint32_t>();
        bool struct_ok = nl == ba.lmdb.kpts.nelem;
        const bs_hnode* kn = ba.lmdb.kpts.before_begin.next;
        for (uint32_t i = 0; i < nl; ++i) {
          int64_t id = (int64_t)d.get<uint64_t>(); float dir[2]; d.arr(dir, 2); float inv = d.get<float>(); int64_t hf = d.get<int64_t>(); uint64_t hc = d.get<uint64_t>(); uint32_t nobs = d.get<uint32_t>();
          bs_keypoint* k = (struct_ok && kn) ? (bs_keypoint*)kn->val : nullptr;
          if (!k || k->id != id || k->host.frame_id != hf || k->host.cam_id != hc || (uint32_t)k->nobs != nobs) struct_ok = false;
          for (uint32_t o = 0; o < nobs; ++o) {
            int64_t f = d.get<int64_t>(); uint64_t c = d.get<uint64_t>(); float pos[2]; d.arr(pos, 2);
            if (k && struct_ok && (k->obs[o].t.frame_id != f || k->obs[o].t.cam_id != c || std::memcmp(k->obs[o].pos, pos, 8) != 0)) struct_ok = false;
          }
          if (k && struct_ok) { k->direction[0] = dir[0]; k->direction[1] = dir[1]; k->inv_dist = inv; }
          if (kn) kn = kn->next;
        }
        ++g_cmp["replay problem: C lmdb (op log) == dumped landmarks / observations, kpts order"];
        if (!struct_ok) { ++g_bad["replay problem: C lmdb (op log) == dumped landmarks / observations, kpts order"]; std::printf("MISMATCH problem %ld (t %" PRId64 "): lmdb structure\n", n_problem, t_ns); ++n_problem; break; }
        ba.poses = poses.data(); ba.n_poses = (int)poses.size(); ba.states = states.data(); ba.n_states = (int)states.size();
        st_max_n = std::max<long>(st_max_n, aom.total_size); st_max_imu = std::max<long>(st_max_imu, nimu); st_max_lm = std::max<long>(st_max_lm, nl);
        if (has_marg) { st_min_prior = std::min<long>(st_min_prior, mg.rows + 2L * mg.cols); st_min_prior_cols = std::min<long>(st_min_prior_cols, mg.cols); }
        for (const bs_hnode* kn2 = ba.lmdb.kpts.before_begin.next; kn2; kn2 = kn2->next) st_max_obs = std::max<long>(st_max_obs, ((const bs_keypoint*)kn2->val)->nobs);
        bs_linabsqr* la = bs_la_create(&ba, &aom, has_marg ? &mg : nullptr, &ild, n_used >= 0 ? used.data() : nullptr, n_used, n_lost >= 0 ? lost.data() : nullptr, n_lost);
        if (kind == 0) {
          auto it = steps.find(t_ns);
          if (it == steps.end()) { std::printf("no ITER_STEP records for problem t=%" PRId64 "\n", t_ns); bs_la_destroy(la); break; }
          const int n = aom.total_size;
          float errv = 0;
          std::vector<float> H((size_t)n * n), b(n);
          std::vector<float> gw(3), aw(3);
          for (int i = 0; i < 3; ++i) { gw[i] = ild.gyro_bias_weight_sqrt[i] * ild.gyro_bias_weight_sqrt[i]; aw[i] = ild.accel_bias_weight_sqrt[i] * ild.accel_bias_weight_sqrt[i]; }
          for (const StepRec& s : it->second) {
            if (!s.full) continue;
            if (s.j == 0) {
              int valid = 1;
              errv = bs_la_linearize_problem(la, &valid);
              bs_la_perform_qr(la);
              ++g_cmp["replay opt: linearizeProblem valid"]; if (!valid) ++g_bad["replay opt: linearizeProblem valid"];
            }
            ++g_cmp["replay opt: error_total (linearizeProblem)"];
            if (std::memcmp(&errv, &s.error_total, 4)) { if (++g_bad["replay opt: error_total (linearizeProblem)"] <= 3) std::printf("MISMATCH error_total t=%" PRId64 " it %u j %u: %.9g vs dump %.9g\n", t_ns, s.it, s.j, errv, s.error_total); }
            ++g_cmp["replay opt: state digest before the step"];
            if (state_digest(poses, states) != s.st_pre) ++g_bad["replay opt: state digest before the step"];
            bs_la_get_dense_H_b(la, H.data(), b.data());
            ++g_cmp["replay opt: H (get_dense_H_b)"]; if ((int)s.n != n || std::memcmp(H.data(), s.H.data(), 4 * (size_t)n * n)) { if (++g_bad["replay opt: H (get_dense_H_b)"] <= 3) std::printf("MISMATCH H t=%" PRId64 " it %u j %u\n", t_ns, s.it, s.j); }
            ++g_cmp["replay opt: b (get_dense_H_b)"]; if ((int)s.n != n || std::memcmp(b.data(), s.b.data(), 4 * (size_t)n)) ++g_bad["replay opt: b (get_dense_H_b)"];
            ++g_cmp["replay opt: hash(H), hash(b) as dumped"]; if (hash_mat(H.data(), n, n) != s.hH || hash_mat(b.data(), n, 1) != s.hb) ++g_bad["replay opt: hash(H), hash(b) as dumped"];
            // backup, backSubstitute, applyInc
            std::vector<bs_frame_pose> poses0 = poses; std::vector<bs_frame_state> states0 = states;
            bs_lmdb_backup(&ba.lmdb);
            const float l_diff = bs_la_back_substitute(la, s.inc.data());
            ++g_cmp["replay opt: l_diff (backSubstitute)"]; if (std::memcmp(&l_diff, &s.l_diff, 4)) { if (++g_bad["replay opt: l_diff (backSubstitute)"] <= 3) std::printf("MISMATCH l_diff t=%" PRId64 " it %u j %u: %.9g vs dump %.9g\n", t_ns, s.it, s.j, l_diff, s.l_diff); }
            for (auto& p : poses) bs_pose_apply_inc(&p, &s.inc[aom_start(aom, p.t_ns)]);
            for (auto& st : states) bs_state_apply_inc(&st, &s.inc[aom_start(aom, st.t_ns)]);
            float ni = 0; for (int i = 0; i < n; ++i) { float a = std::fabs(s.inc[i]); if (a > ni) ni = a; }
            ++g_cmp["replay opt: step_norminf"]; if (std::memcmp(&ni, &s.norminf, 4)) ++g_bad["replay opt: step_norminf"];
            float vi = bs_ba_compute_error(&ba);
            float mp = has_marg ? bs_ba_marg_prior_error(&ba, &mg) : 0.0f;
            float ie, be, ae;
            std::vector<bs_imu_meas*> allimu = imup;
            bs_ba_compute_imu_error(&ba, &aom, allimu.data(), (int)allimu.size(), ild.g, gw.data(), aw.data(), &ie, &be, &ae);
            vi += ie + be + ae;
            ++g_cmp["replay opt: after-update error (computeError + imu + bias)"]; if (std::memcmp(&vi, &s.after_vi, 4)) { if (++g_bad["replay opt: after-update error (computeError + imu + bias)"] <= 3) std::printf("MISMATCH after_vi t=%" PRId64 " it %u j %u: %.9g vs dump %.9g\n", t_ns, s.it, s.j, vi, s.after_vi); }
            ++g_cmp["replay opt: after-update marg prior error"]; if (std::memcmp(&mp, &s.after_marg, 4)) ++g_bad["replay opt: after-update marg prior error"];
            const float after_total = vi + mp;
            const float f_diff = errv - after_total;
            const float rel = f_diff / l_diff;
            ++g_cmp["replay opt: f_diff, relative_decrease"]; if (std::memcmp(&f_diff, &s.f_diff, 4) || std::memcmp(&rel, &s.rel_dec, 4)) ++g_bad["replay opt: f_diff, relative_decrease"];
            ++g_cmp["replay opt: state digest after applyInc"]; if (state_digest(poses, states) != s.st_post) { if (++g_bad["replay opt: state digest after applyInc"] <= 3) std::printf("MISMATCH state_post t=%" PRId64 " it %u j %u\n", t_ns, s.it, s.j); }
            ++g_cmp["replay opt: landmark value digest after the step"]; if (lm_value_digest(ba.lmdb) != s.lm_post) { if (++g_bad["replay opt: landmark value digest after the step"] <= 3) std::printf("MISMATCH lm_post t=%" PRId64 " it %u j %u\n", t_ns, s.it, s.j); }
            if (!(s.flags & 1)) { poses = poses0; states = states0; bs_lmdb_restore(&ba.lmdb); }   // rejected: restore()
          }
        } else {
          auto mi = margs.find(t_ns);
          if (mi == margs.end() || !mi->second.full) { std::printf("no full MARG record for problem t=%" PRId64 "\n", t_ns); bs_la_destroy(la); ++n_problem; break; }
          const MargRec& m = mi->second;
          int valid = 1;
          bs_la_linearize_problem(la, &valid);
          bs_la_perform_qr(la);
          const int rows = bs_la_dense_Q2_rows(la);
          std::vector<float> Q((size_t)rows * aom.total_size), q(rows);
          bs_la_get_dense_Q2Jp_Q2r(la, Q.data(), q.data());
          ++g_cmp["replay marg: Q2Jp, Q2r (get_dense_Q2Jp_Q2r) vs MARG record"];
          bool ok = (int)m.qrows == rows && (int)m.qcols == aom.total_size && !std::memcmp(Q.data(), m.Q2Jp.data(), 4 * Q.size()) && !std::memcmp(q.data(), m.Q2r.data(), 4 * q.size());
          if (!ok) { if (++g_bad["replay marg: Q2Jp, Q2r (get_dense_Q2Jp_Q2r) vs MARG record"] <= 3) std::printf("MISMATCH marg Q2Jp/Q2r t=%" PRId64 " rows %d/%u\n", t_ns, rows, m.qrows); }
        }
        st_max_blocks = std::max<long>(st_max_blocks, bs_la_num_landmark_blocks(la));
        st_max_q2rows = std::max<long>(st_max_q2rows, bs_la_dense_Q2_rows(la));
        bs_la_destroy(la);
        ++n_problem;
        break; }
      default: break;
    }
  }
  bs_ba_destroy(&ba);
  std::printf("replay %s: %zu m6 records, %ld ORDER checks, %ld unconnected_obs0 checks, %ld PROBLEM records\n", dir.c_str(), m6.size(), n_order, n_unconn, n_problem);
  std::printf("replay sizes: max n (aom total) %ld, max observations of a landmark %ld, max landmarks in a problem %ld, max landmark blocks %ld, max Q2 rows %ld, max imu blocks %ld, min prior rows + 2*cols %ld, min prior cols %ld, bs_la_unsupported %d\n",
              st_max_n, st_max_obs, st_max_lm, st_max_blocks, st_max_q2rows, st_max_imu, st_min_prior, st_min_prior_cols, bs_la_unsupported);
  return 0;
}

int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "prim";
  long seed = argc > 2 ? atol(argv[2]) : 1, cases = argc > 3 ? atol(argv[3]) : 1000;
  tbb::global_control tbb_one(tbb::global_control::max_allowed_parallelism, 1);   // as the reference driver: TBB reductions run serially in index order
  Rng r(seed);
  {   // the GEBP blocking model hard-codes this machine's Eigen cache sizes
    const long l[3] = {(long)Eigen::l1CacheSize(), (long)Eigen::l2CacheSize(), (long)Eigen::l3CacheSize()};
    ++g_cmp["Eigen cache sizes == the C model's (l1 l2 l3)"];
    if (std::memcmp(l, bs_la_cache_sizes, sizeof l) != 0) { ++g_bad["Eigen cache sizes == the C model's (l1 l2 l3)"]; std::printf("MISMATCH Eigen cache sizes %ld %ld %ld vs model %ld %ld %ld\n", l[0], l[1], l[2], bs_la_cache_sizes[0], bs_la_cache_sizes[1], bs_la_cache_sizes[2]); }
  }
  if (getenv("BS_T_NOIMU")) g_no_imu = 1;
  if (getenv("BS_T_NOMARG")) g_no_marg = 1;
  if (mode == "prim") { prim(r, cases); prim_back(r, cases); }
  if (mode == "lmdb") lmdb_scenarios(r, cases);
  if (mode == "problem") problem_scenarios(r, cases, 1);
  if (mode == "replay") { int rc = replay(argv[2], argc > 3 ? atol(argv[3]) : -1); if (rc) return rc; }
  long tot = 0, bad = 0;
  for (auto& kv : g_cmp) { std::printf("  %-34s %10ld compared %8ld mismatches\n", kv.first.c_str(), kv.second, g_bad[kv.first]); tot += kv.second; bad += g_bad[kv.first]; }
  std::printf("%s seed %ld cases %ld: %ld comparisons, %ld mismatches\n", mode.c_str(), seed, cases, tot, bad);
  return bad != 0;
}
