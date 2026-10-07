// Oracle for basalt_port/c/bs_marg.{h,c} (module M8: marginalize() + MargHelper<float>::marginalizeHelperSqrtToSqrt).
// Modes (argv[1]):  helper <seed> <cases>        random Q2Jp / Q2r / index sets vs the real MargHelper<float>::marginalizeHelperSqrtToSqrt, tolerance 0
//                   marginalize <seed> <cases>   random estimator states vs the real SqrtKeypointVioEstimator<float>::marginalize()
//                   replay <dir> [max]           replay of a reference dump (patches 0003 + 0004 + 0005): every MARG record against the C marginalize()
// Reuses the estimator-twin machinery of bs_vio_opt_test.cc (its main() is renamed).
#define BS_NO_MAIN
#include "bs_vio_opt_test.cc"
#include <basalt/vi_estimator/marg_helper.h>
extern "C" {
#include "bs_marg.h"
}

// ------------------------------------------------------------------------------------------------ helper
static void helper_cases(Rng& r, long cases) {
  long rank_def = 0, big = 0, wide = 0, nonfinite = 0;
  for (long c = 0; c < cases; ++c) {
    // an aom: poses (6) then states (15), the marginalisation keeps / removes whole items or the vel / bias part of a state
    const int np = (int)(r() % 8), ns = 1 + (int)(r() % 4);
    std::vector<int> start, size;
    int total = 0;
    for (int i = 0; i < np; ++i) { start.push_back(total); size.push_back(6); total += 6; }
    for (int i = 0; i < ns; ++i) { start.push_back(total); size.push_back(15); total += 15; }
    std::set<int> keep, marg;
    const int mode = (int)(r() % 4);
    for (int i = 0; i < np + ns; ++i) {
      int kind = (int)(r() % 3);                         // 0 keep, 1 marg, 2 vel/bias split (states only)
      if (mode == 0) kind = (i == 0 || i == np) ? 1 : 0;
      if (size[i] == 6 && kind == 2) kind = (int)(r() % 2);
      for (int k = 0; k < size[i]; ++k) {
        const int idx = start[i] + k;
        if (kind == 0 || (kind == 2 && k < 6)) keep.insert(idx); else marg.insert(idx);
      }
    }
    if (marg.empty()) { marg.insert(*keep.begin()); keep.erase(keep.begin()); }
    if (keep.empty()) { keep.insert(*marg.rbegin()); marg.erase(std::prev(marg.end())); }
    int rows;
    const int rc = (int)(r() % 6);
    if (rc == 0) rows = 1 + (int)(r() % total);                          // fewer rows than columns
    else if (rc == 1) rows = total + (int)(r() % 40);
    else if (rc == 2) rows = 4 * total + (int)(r() % 400);
    else if (rc == 3) rows = (r() % 4 == 0) ? 1700 + (int)(r() % 900) : 300 + (int)(r() % 500);
    else rows = total + (int)(r() % (3 * total + 1));
    MatX Q(rows, total);
    VecX q(rows);
    const int cls = (int)(r() % 7);
    for (int j = 0; j < total; ++j)
      for (int i = 0; i < rows; ++i) {
        double v = nrand(r);
        if (cls == 1) v = (r() % 3 == 0) ? v : 0.0;                      // sparse
        if (cls == 2) v *= std::exp(urand(r, -6, 6) * 0.5);              // dynamic range
        if (cls == 3) v = (double)((int)(r() % 7) - 3);                  // integers
        Q(i, j) = (S)v;
      }
    if (cls == 4 || r() % 5 == 0) {                                     // rank deficiency: zero columns, repeated columns
      const int nz = 1 + (int)(r() % 6);
      for (int z = 0; z < nz; ++z) {
        const int j = (int)(r() % total);
        if (r() % 2) Q.col(j).setZero(); else Q.col(j) = Q.col((int)(r() % total)) * (S)((r() % 2) ? 1.0 : 0.5);
      }
      ++rank_def;
    }
    if (cls == 5) {                                                      // tiny entries around the rank threshold (sqrt(eps) = 3.45e-4)
      for (int j = 0; j < total; ++j) if (r() % 3 == 0) Q.col(j) *= (S)std::pow(10.0, urand(r, -6, -2));
    }
    if (cls == 6) { for (int i = 0; i < rows; ++i) for (int j = 0; j < total; ++j) if ((r() % 8) != 0 && ((i * 7 + j) % 11) < 8) Q(i, j) = 0.0f; }   // structured zeros
    if (r() % 40 == 0) Q.setZero();
    for (int i = 0; i < rows; ++i) q[i] = (r() % 6 == 0) ? 0.0f : (S)(nrand(r) * std::exp(urand(r, -3, 3)));
    if (r() % 60 == 0) { Q(r() % rows, r() % total) = std::numeric_limits<S>::infinity(); ++nonfinite; }
    MatX Qc = Q; VecX qc = q;
    MatX He; VecX be;
    basalt::MargHelper<S>::marginalizeHelperSqrtToSqrt(Qc, qc, keep, marg, He, be);
    std::vector<int> kv(keep.begin(), keep.end()), mv(marg.begin(), marg.end());
    std::vector<float> Cc(Q.data(), Q.data() + (size_t)rows * total), cq(q.data(), q.data() + rows);
    float* Hc; float* bc; int Hr;
    const int oob0 = bs_marg_oob;
    bs_marg_helper_sqrt_to_sqrt(Cc.data(), rows, total, cq.data(), kv.data(), (int)kv.size(), mv.data(), (int)mv.size(), &Hc, &Hr, &bc);
    if (bs_marg_oob != oob0) { ++g_cmp["helper: skipped, the C++ reads out of bounds (UB)"]; free(Hc); free(bc); continue; }
    ++g_cmp["helper: marg_sqrt_H (rows, cols, values)"];
    bool ok = (Hr == He.rows() && (int)kv.size() == He.cols() && bits_eq(Hc, He.data(), (int)(He.rows() * He.cols())));
    if (!ok) { if (++g_bad["helper: marg_sqrt_H (rows, cols, values)"] <= 4) std::printf("MISMATCH helper H rows %d cols %d class %d (C %d vs %ld)\n", rows, total, cls, Hr, (long)He.rows()); }
    ++g_cmp["helper: marg_sqrt_b"];
    if (!(Hr == be.size() && bits_eq(bc, be.data(), (int)be.size()))) ++g_bad["helper: marg_sqrt_b"];
    if (rows > 1500) ++big;
    if (rows < total) ++wide;
    free(Hc); free(bc);
  }
  std::printf("helper cases: %ld, %ld with injected rank deficiency, %ld with rows >= 1500, %ld with rows < cols, %ld with an infinite entry\n", cases, rank_def, big, wide, nonfinite);
}

// ------------------------------------------------------------------------------------------------ marginalize vs the real estimator

// debugging aid: capture the helper's inputs of the C marginalize and run the real MargHelper on them
static MatX g_hq; static VecX g_hr; static std::set<int> g_hkeep, g_hmarg; static bool g_hook_on = false;
static void helper_hook(const float* Q2Jp, int rows, int cols, const float* Q2r, const int* keep, int nkeep, const int* marg, int nmarg) {
  if (!g_hook_on) return;
  g_hq = Eigen::Map<const MatX>(Q2Jp, rows, cols); g_hr = Eigen::Map<const VecX>(Q2r, rows);
  g_hkeep = std::set<int>(keep, keep + nkeep); g_hmarg = std::set<int>(marg, marg + nmarg);
}
static long g_sc = 0;
static bool compare_marg_state(const char* tag, Est& e, bs_vio& v) {
  bool ok = compare_state(tag, e, v);
  basalt::BundleAdjustmentBase<S>& base = e;
  auto fail = [&](const char* what) { if (++g_bad[std::string(tag) + ": " + what] <= 3) std::printf("MISMATCH %s: %s\n", tag, what); ok = false; };
  ++g_cmp[std::string(tag) + ": kf_ids"];
  { const auto& k = P_(e, kf_ids); bool same = (int)k.size() == v.n_kf; int i = 0; for (auto t : k) if (same && t != v.kf_ids[i++]) same = false; if (!same) fail("kf_ids"); }
  ++g_cmp[std::string(tag) + ": imu_meas keys"];
  { const auto& m = P_(e, imu_meas); bool same = (int)m.size() == v.n_imu; int i = 0; for (auto& kv : m) if (same && kv.first != v.imu[i++].start_t_ns) same = false; if (!same) fail("imu_meas keys"); }
  ++g_cmp[std::string(tag) + ": marg_data (order, H, b)"];
  {
    const auto& md = P_(e, marg_data);
    bool same = md.order.total_size == (size_t)v.marg.total && (int)md.order.abs_order_map.size() == v.marg.n && md.H.rows() == v.marg.rows && md.H.cols() == v.marg.cols && md.b.size() == v.marg.rows;
    int i = 0;
    for (auto& kv : md.order.abs_order_map) { if (same && (kv.first != v.marg.item[i].t_ns || kv.second.first != v.marg.item[i].start || kv.second.second != v.marg.item[i].size)) same = false; ++i; }
    if (same && (!bits_eq(md.H.data(), v.marg.H, (int)md.H.size()) || !bits_eq(md.b.data(), v.marg.b, (int)md.b.size()))) same = false;
    if (!same) {
      fail("marg_data (order, H, b)");
      std::printf("  detail: scenario %ld real H %ldx%ld b %ld total %zu items %zu | C H %dx%d b %d total %d items %d\n", g_sc, (long)md.H.rows(), (long)md.H.cols(), (long)md.b.size(), md.order.total_size, md.order.abs_order_map.size(), v.marg.rows, v.marg.cols, v.marg.rows, v.marg.total, v.marg.n);
      if (md.H.rows() == v.marg.rows && md.H.cols() == v.marg.cols) {
        for (int q = 0; q < (int)md.H.size(); ++q) if (std::memcmp(&md.H.data()[q], &v.marg.H[q], 4)) { std::printf("  first H diff at (%d,%d): real %.9g C %.9g\n", q % v.marg.rows, q / v.marg.rows, md.H.data()[q], v.marg.H[q]); break; }
        for (int q = 0; q < (int)md.b.size(); ++q) if (std::memcmp(&md.b.data()[q], &v.marg.b[q], 4)) { std::printf("  first b diff at %d: real %.9g C %.9g\n", q, md.b.data()[q], v.marg.b[q]); break; }
      }
    }
  }
  ++g_cmp[std::string(tag) + ": lmdb iteration orders (kpts, observations), counts"];
  {
    bool same = base.lmdb.numLandmarks() == v.ba.lmdb.kpts.nelem && base.lmdb.getObservations().size() == v.ba.lmdb.observations.nelem && base.lmdb.numObservations() == bs_lmdb_num_observations(&v.ba.lmdb);
    if (same) {
      const bs_hnode* n = v.ba.lmdb.kpts.before_begin.next;
      for (const auto& kv : base.lmdb.getLandmarks()) { if (!n || (uint64_t)n->k0 != (uint64_t)kv.first) { same = false; break; } n = n->next; }
      const bs_hnode* h = v.ba.lmdb.observations.before_begin.next;
      for (const auto& kv : base.lmdb.getObservations()) { if (!h || h->k0 != kv.first.frame_id || (uint64_t)h->k1 != (uint64_t)kv.first.cam_id) { same = false; break; } h = h->next; }
    }
    if (!same) fail("lmdb iteration orders (kpts, observations), counts");
  }
  return ok;
}

static void marginalize_cases(Rng& r, long cases) {
  signal(SIGABRT, on_abort);
  long ran = 0, aborted = 0, skipped_ub = 0, skipped_oob = 0, not_triggered = 0, with_kfsel1 = 0, with_kfsel2 = 0, with_vb = 0, with_all = 0, with_lost = 0, with_kfs = 0, nonkf_pose = 0;
  std::streambuf* cout_buf = std::cout.rdbuf();
  std::ostringstream sink;
  for (long sc = 0; sc < cases; ++sc) {
    g_sc = sc;
    Problem P;
    build_problem(r, P, (int)(r() % 4 == 0 ? 0 : 1));
    Twin T;
    const int max_states = 2 + (int)(r() % 3), max_kfs = (r() % 2 == 0) ? 1 + (int)(r() % 7) : 7;
    const int idx_last = (int)P.ba.frame_states.size() - max_states + 1;   // index of last_state_to_marg: the prior covers at most the states before it (plus, rarely, it)
    std::cout.rdbuf(sink.rdbuf()); if (!getenv("BS_T_VERBOSE")) std::cerr.setstate(std::ios::failbit);
    const bool made = make_twin(r, P, T, true, std::max(0, idx_last) + (r() % 10 == 0 ? 1 : 0));
    std::cout.rdbuf(cout_buf); std::cerr.clear();
    if (!made) continue;
    Est& e = *T.est; bs_vio& v = T.v;
    basalt::BundleAdjustmentBase<S>& base = e;
    const int np = (int)base.frame_poses.size(), ns = (int)base.frame_states.size();
    P_(e, max_states) = max_states; P_(e, max_kfs) = max_kfs; v.max_states = max_states; v.max_kfs = max_kfs;
    const double ratio = (r() % 3 == 0) ? urand(r, 0.0, 1.5) : 0.1;
    P_(e, config).vio_kf_marg_feature_ratio = ratio; v.kf_marg_feature_ratio = ratio;
    // kf ids: a random subset of the poses (some poses stay non-kf) + the newest state + maybe one older state
    std::set<int64_t> kf;
    std::vector<int64_t> pose_t, state_t;
    for (auto& kv : base.frame_poses) pose_t.push_back(kv.first);
    for (auto& kv : base.frame_states) state_t.push_back(kv.first);
    for (auto t : pose_t) if (r() % 8 != 0) kf.insert(t); else ++nonkf_pose;
    kf.insert(state_t.back());
    if (ns >= 2 && r() % 4 != 0) kf.insert(state_t[(idx_last > 0 && r() % 4 != 0) ? r() % std::min(idx_last, ns - 1) : r() % (ns - 1)]);
    {   // states after the prior's states are not linearized in a real run (only the oldest state is, after being last_state_to_marg once)
      const int extra = v.marg.n - np;
      for (int i = std::max(extra, idx_last); i < ns; ++i) {
        auto& st = base.frame_states.at(state_t[i]);
        if (st.isLinearized()) { st.setLinFalse(); v.ba.states[i].s.linearized = 0; for (int k = 0; k < 15; ++k) v.ba.states[i].delta[k] = 0.0f; }
      }
    }
    // states older than last_state_to_marg are linearized in a real run (each was the oldest state once): so they become linearized poses
    for (int i = 0; i < std::min(idx_last, ns); ++i) if (r() % 10 != 0) {
      auto& st = base.frame_states.at(state_t[i]);
      if (!st.isLinearized()) { st.setLinTrue(); for (auto& c : v.ba.states[i].t_ns == state_t[i] ? std::vector<bs_frame_state*>{&v.ba.states[i]} : std::vector<bs_frame_state*>{}) { c->s.linearized = 1; c->s.cur = c->s.lin; } }
    }
    P_(e, kf_ids) = kf;
    for (auto t : kf) bs_vio_kf_insert(&v, t);
    std::map<int64_t, int> npk, npc;
    for (auto t : kf) { const int n = (r() % 30 == 0) ? 0 : 1 + (int)(r() % 300); npk[t] = n; bs_vio_npk_set(&v, t, n); }
    std::vector<bs_kf_count> cnpc;
    const double conn_frac = (r() % 3 == 0) ? 0.0 : urand(r, 0.0, 1.0);
    for (auto t : kf) if (r() % 100 < 100 * conn_frac + (r() % 4 == 0 ? 30 : 0)) { const int n = (int)(r() % 320 * (r() % 3 == 0 ? 0.1 : 1.0)); npc[t] = n; }
    if (r() % 5 == 0) npc[(int64_t)(r() % 1000)] = 5;                    // an entry for a non-kf id (never consulted)
    if (!getenv("BS_T_OLDSTREAM") && (int)(r() % 100) < (getenv("BS_T_DSO_PCT") ? atoi(getenv("BS_T_DSO_PCT")) : 30)) {   // force the DSO score path: every kf keeps more than the ratio of its points
      npc.clear();
      for (auto t : kf) npc[t] = (int)(npk[t] * (ratio < 1.0 ? ratio * urand(r, 1.05, 3.0) : 1.0) + 2);
    }
    for (auto& kv : npc) cnpc.push_back(bs_kf_count{kv.first, kv.second});
    P_(e, num_points_kf) = npk;
    std::vector<int64_t> lost;
    std::unordered_set<basalt::KeypointId> lost_set;
    {
      const int idx = std::max(0, (int)ns - max_states + 1);
      const int64_t last_t = state_t[std::min(idx, ns - 1)];
      const bool strict = r() % 12 != 0;       // usually only landmarks hosted inside the marginalisation aom are lost (the C++ asserts otherwise)
      if (r() % 3 != 0) for (const auto& kv : base.lmdb.getLandmarks()) if (r() % 100 < 12 && (!strict || kv.second.host_kf_id.frame_id <= last_t)) { lost.push_back(kv.first); lost_set.insert(kv.first); }
    }
    // the iteration order of the unordered_set is the order the C++ removes in: give the C the same order
    lost.clear(); for (auto id : lost_set) lost.push_back(id);
    if (r() % 10 < 6) {   // spread the key frame positions (the DSO score depends on their distances; random-walk poses make the argmin insensitive)
      const double sc = std::exp(urand(r, -2, 2));
      for (int i = 0; i < np; ++i) {
        basalt::PoseState<S>::VecN inc;
        for (int k = 0; k < 3; ++k) inc[k] = (S)(nrand(r) * sc);
        for (int k = 3; k < 6; ++k) inc[k] = (S)(nrand(r) * 0.01);
        base.frame_poses.at(pose_t[i]).applyInc(inc);
        bs_pose_apply_inc(&v.ba.poses[i], inc.data());
      }
    }
    if (!compare_marg_state("marginalize pre-state (twin construction)", e, v)) continue;

    // C first: UB scenarios of the lmdb (removeLandmarkHelper on a vanished host entry) are skipped, the real class would read an end() iterator
    const int ub0 = bs_lmdb_ub, oob0 = bs_marg_oob;
    g_hook_on = getenv("BS_T_HOOK") != nullptr; bs_marg_dbg_hook = helper_hook;
    bs_marg_result res;
    bs_vio_marginalize(&v, cnpc.data(), (int)cnpc.size(), lost.data(), (int)lost.size(), &res);
    if (bs_lmdb_ub != ub0) { ++skipped_ub; bs_marg_result_free(&res); continue; }
    if (bs_marg_oob != oob0) { ++skipped_oob; bs_marg_result_free(&res); continue; }   // the C++ helper reads Q2Jp out of bounds (undefined behaviour: the real value is garbage)
    bool real_aborted = false;
    std::cout.rdbuf(sink.rdbuf()); if (!getenv("BS_T_VERBOSE")) std::cerr.setstate(std::ios::failbit);
    if (res.layout_error) real_aborted = real_aborts_in_child([&] { e.marginalize(npc, lost_set); });
    else e.marginalize(npc, lost_set);
    std::cout.rdbuf(cout_buf); std::cerr.clear();
    if (getenv("BS_T_HOOK") && res.marginalized && !res.layout_error) {   // isolate: real helper on the C's helper inputs vs the C's helper
      MatX Hr; VecX br; MatX q = g_hq; VecX rr = g_hr;
      basalt::MargHelper<S>::marginalizeHelperSqrtToSqrt(q, rr, g_hkeep, g_hmarg, Hr, br);
      ++g_cmp["marginalize (hook): real helper on the C's inputs == C helper output"];
      if (!(Hr.rows() == v.marg.rows && Hr.cols() == v.marg.cols && std::memcmp(Hr.data(), v.marg.H, 4 * (size_t)Hr.size()) == 0)) { ++g_bad["marginalize (hook): real helper on the C's inputs == C helper output"]; std::printf("HOOK: helper differs in scenario %ld\n", sc);
        FILE* f = fopen("/tmp/claude-1000/-home-nybo-github-pose-validation/4ce8902b-4ba7-41fb-b22a-8d144ecc43f3/scratchpad/hook_case.bin", "wb");
        int hdr[4] = {(int)g_hq.rows(), (int)g_hq.cols(), (int)g_hkeep.size(), (int)g_hmarg.size()};
        fwrite(hdr, 4, 4, f); fwrite(g_hq.data(), 4, (size_t)g_hq.size(), f); fwrite(g_hr.data(), 4, (size_t)g_hr.size(), f);
        std::vector<int> kk(g_hkeep.begin(), g_hkeep.end()), mm(g_hmarg.begin(), g_hmarg.end()); fwrite(kk.data(), 4, kk.size(), f); fwrite(mm.data(), 4, mm.size(), f); fclose(f);
        int nd = 0; for (int q = 0; q < Hr.size(); ++q) if (std::memcmp(&Hr.data()[q], &v.marg.H[q], 4)) { if (nd++ < 6) std::printf("  H[%d]: real %.9g C %.9g\n", q, Hr.data()[q], v.marg.H[q]); } std::printf("  %d of %ld entries differ; b: real %.9g C %.9g\n", nd, (long)Hr.size(), br[0], v.marg.b[0]); }
    }
    ++g_cmp["marginalize: C layout / assertion condition <=> the real run aborts"];
    if (real_aborted != (bool)res.layout_error) { if (++g_bad["marginalize: C layout / assertion condition <=> the real run aborts"] <= 4) std::printf("MISMATCH abort behaviour: real %d C %d (np %d ns %d max_kfs %d max_states %d)\n", (int)real_aborted, res.layout_error, np, ns, max_kfs, max_states); }
    if (real_aborted) { ++aborted; bs_marg_result_free(&res); continue; }
    if (!res.marginalized) ++not_triggered;
    else {
      ++ran;
      if (res.n_kfs > 0) ++with_kfs;
      if (res.n_states_vb > 0) ++with_vb;
      if (res.n_states_all > 0) ++with_all;
      if (!lost.empty()) ++with_lost;
      if (res.n_kf_all - res.n_kfs > 0 && res.n_kfs > 0) ++with_kfsel1;
      if (res.n_kfs >= 2) ++with_kfsel2;
    }
    compare_marg_state("marginalize", e, v);
    bs_marg_result_free(&res);
  }
  std::printf("C counters: %ld marginalizations, %ld kfs chosen by the feature ratio, %ld by the DSO score, %ld rank-deficient columns\n", bs_marg_stats[0], bs_marg_stats[1], bs_marg_stats[2], bs_marg_stats[3]);
  std::printf("marginalize scenarios: %ld cases, %ld marginalized, %ld not triggered, %ld real assertion aborts (all matched by the C), %ld skipped (lmdb UB), %ld skipped (helper reads Q2Jp out of bounds: UB), %ld with kfs_to_marg, %ld with >= 2 kfs_to_marg, %ld with vel/bias states, %ld with removed states, %ld with lost landmarks, %ld non-kf poses seen\n",
              cases, ran, not_triggered, aborted, skipped_ub, skipped_oob, with_kfs, with_kfsel2, with_vb, with_all, with_lost, nonkf_pose);
}

// ================================================================================================ dump replay (every MARG record)
struct MargDump {
  int64_t t_ns, last_state_to_marg; uint32_t path, is_lin_sqrt, is_sqrt, aom_total, aom_items, n_poses_to_marg, n_states_all, n_states_vb, n_kfs, n_kf_all;
  std::vector<bs_aom_item> aom; std::vector<int32_t> keep, marg; std::vector<int64_t> kfs, kf_all;
  uint32_t prow, pcol; uint64_t hpH, hpb; uint32_t qrows, qcols; uint64_t hQ, hr; uint32_t orows, ocols; uint64_t hHnew; uint32_t nb; uint64_t hbnew, hfinal; uint32_t order_total; bool full;
  std::vector<float> priorH, priorb, Hnew, bnew, bfinal;
};
struct MargIn { uint32_t max_states, max_kfs; double ratio; std::vector<int64_t> kf; std::vector<std::pair<int64_t, int32_t>> conn, npk; std::vector<int64_t> imu, prev, poses, states; };
struct MargOut {
  std::vector<int64_t> kf; std::vector<std::pair<int64_t, uint32_t>> poses, states; std::vector<int64_t> imu, prev; std::vector<bs_aom_item> order; uint32_t order_total, hrows, hcols; uint64_t hH, hb, sdig, lmo, hos, lmv; uint32_t nlm, nobs;
};
static bool read_i64s(Rd& d, std::vector<int64_t>& v) { uint32_t n = d.get<uint32_t>(); v = d.vec<int64_t>(n); return true; }
static void read_pairs(Rd& d, std::vector<std::pair<int64_t, int32_t>>& v) { uint32_t n = d.get<uint32_t>(); v.resize(n); for (auto& p : v) { p.first = d.get<int64_t>(); p.second = d.get<int32_t>(); } }
static void read_lin(Rd& d, std::vector<std::pair<int64_t, uint32_t>>& v) { uint32_t n = d.get<uint32_t>(); v.resize(n); for (auto& p : v) { p.first = d.get<int64_t>(); p.second = d.get<uint32_t>(); } }

static int replay8(const std::string& dir, long max_problems) {
  const std::string T8 = "replay marg";
  std::map<int64_t, MargDump> margs; std::map<int64_t, MargIn> ins; std::map<int64_t, MargOut> outs;
  {
    RecStream rs(dir + "/marg.bin"); Rec r;
    if (!rs.f) { std::printf("cannot read %s/marg.bin\n", dir.c_str()); return 2; }
    while (rs.next(r)) {
      if (r.tag != 4) continue;
      Rd d(r.b); MargDump m;
      m.t_ns = d.get<int64_t>(); m.last_state_to_marg = d.get<int64_t>();
      m.path = d.get<uint32_t>(); m.is_lin_sqrt = d.get<uint32_t>(); m.is_sqrt = d.get<uint32_t>(); m.aom_total = d.get<uint32_t>(); m.aom_items = d.get<uint32_t>();
      m.n_poses_to_marg = d.get<uint32_t>(); m.n_states_all = d.get<uint32_t>(); m.n_states_vb = d.get<uint32_t>(); m.n_kfs = d.get<uint32_t>(); m.n_kf_all = d.get<uint32_t>();
      m.aom.resize(d.get<uint32_t>()); for (auto& a : m.aom) { a.t_ns = d.get<int64_t>(); a.start = (int)d.get<uint32_t>(); a.size = (int)d.get<uint32_t>(); }
      m.keep = d.vec<int32_t>(d.get<uint32_t>()); m.marg = d.vec<int32_t>(d.get<uint32_t>());
      read_i64s(d, m.kfs); read_i64s(d, m.kf_all);
      m.prow = d.get<uint32_t>(); m.pcol = d.get<uint32_t>(); m.hpH = d.get<uint64_t>(); m.hpb = d.get<uint64_t>();
      m.qrows = d.get<uint32_t>(); m.qcols = d.get<uint32_t>(); m.hQ = d.get<uint64_t>(); m.hr = d.get<uint64_t>();
      m.orows = d.get<uint32_t>(); m.ocols = d.get<uint32_t>(); m.hHnew = d.get<uint64_t>(); m.nb = d.get<uint32_t>(); m.hbnew = d.get<uint64_t>(); m.hfinal = d.get<uint64_t>(); m.order_total = d.get<uint32_t>();
      m.full = d.get<uint32_t>() != 0;
      if (m.full) {
        m.priorH = d.vec<float>((size_t)m.prow * m.pcol); m.priorb = d.vec<float>(m.prow);
        d.o += 4 * ((size_t)m.qrows * m.qcols + m.qrows);       // Q2Jp, Q2r: validated by M6
        m.Hnew = d.vec<float>((size_t)m.orows * m.ocols); m.bnew = d.vec<float>(m.nb); m.bfinal = d.vec<float>(m.orows);
        if (d.o != r.b.size()) { std::printf("MARG record layout: %zu trailing bytes\n", r.b.size() - d.o); return 2; }
      }
      margs[m.t_ns] = std::move(m);
    }
  }
  {
    RecStream rs(dir + "/m78.bin"); Rec r;
    if (!rs.f) { std::printf("no m78.bin in %s (patch 0005 needed)\n", dir.c_str()); return 2; }
    while (rs.next(r)) {
      Rd d(r.b);
      if (r.tag == 42) {
        MargIn m; int64_t t = d.get<int64_t>(); m.max_states = d.get<uint32_t>(); m.max_kfs = d.get<uint32_t>(); m.ratio = d.get<double>();
        read_i64s(d, m.kf); read_pairs(d, m.conn); read_pairs(d, m.npk); read_i64s(d, m.imu); read_i64s(d, m.prev); read_i64s(d, m.poses); read_i64s(d, m.states);
        ins[t] = std::move(m);
      } else if (r.tag == 43) {
        MargOut m; int64_t t = d.get<int64_t>();
        read_i64s(d, m.kf); read_lin(d, m.poses); read_lin(d, m.states); read_i64s(d, m.imu); read_i64s(d, m.prev);
        m.order.resize(d.get<uint32_t>()); for (auto& a : m.order) { a.t_ns = d.get<int64_t>(); a.start = (int)d.get<uint32_t>(); a.size = (int)d.get<uint32_t>(); }
        m.order_total = d.get<uint32_t>(); m.hrows = d.get<uint32_t>(); m.hcols = d.get<uint32_t>(); m.hH = d.get<uint64_t>(); m.hb = d.get<uint64_t>(); m.sdig = d.get<uint64_t>();
        m.lmo = d.get<uint64_t>(); m.hos = d.get<uint64_t>(); m.lmv = d.get<uint64_t>(); m.nlm = d.get<uint32_t>(); m.nobs = d.get<uint32_t>();
        outs[t] = std::move(m);
      }
    }
  }

  bs_vio v; bs_vio_init(&v); replay_calib(v);
  std::map<int64_t, std::pair<std::array<float, 3>, std::array<float, 3>>> bias_lin;
  long n_problem = 0, n_calls = 0, n_none = 0, max_q2 = 0, n_kfsel = 0, n_vb = 0, n_lost = 0, n_multi_kf = 0;
  bool have_pre_order = false; uint64_t pre_hl = 0, pre_hh = 0; uint32_t pre_nl = 0, pre_nh = 0;
  bool expect_removals = false; std::vector<int64_t> rm_expect_lost; size_t rm_lost_pos = 0; bs_marg_result cur; memset(&cur, 0, sizeof cur);
  RecStream rs(dir + "/m6.bin"); Rec r;
  if (!rs.f) { std::printf("cannot read %s/m6.bin\n", dir.c_str()); return 2; }
  auto cmp = [&](const char* name, bool ok, int64_t t) { ++g_cmp[T8 + ": " + name]; if (!ok) { if (++g_bad[T8 + ": " + name] <= 3) std::printf("MISMATCH %s t=%" PRId64 "\n", name, t); } };
  while (rs.next(r)) {
    if (expect_removals && r.tag == 23) {          // removeKeyframes of this marginalisation: compare, do not apply (the C marginalize already did)
      Rd d(r.b); uint32_t a = d.get<uint32_t>(); auto kf = d.vec<int64_t>(a); uint32_t b2 = d.get<uint32_t>(); auto po = d.vec<int64_t>(b2); uint32_t c = d.get<uint32_t>(); auto st = d.vec<int64_t>(c);
      auto eq = [](const std::vector<int64_t>& x, const int64_t* y, int n) { return (int)x.size() == n && (n == 0 || !std::memcmp(x.data(), y, 8 * (size_t)n)); };
      cmp("removeKeyframes(kfs_to_marg, poses_to_marg, states_to_marg_all) arguments", eq(kf, cur.kfs_to_marg, cur.n_kfs) && eq(po, cur.poses_to_marg, cur.n_poses_to_marg) && eq(st, cur.states_to_marg_all, cur.n_states_all), 0);
      continue;
    }
    if (expect_removals && r.tag == 24) {          // removeLandmark(lost) in the unordered_set order
      Rd d(r.b); const int64_t id = (int64_t)d.get<uint64_t>();
      cmp("removeLandmark sequence (lost_landmaks iteration order)", rm_lost_pos < rm_expect_lost.size() && rm_expect_lost[rm_lost_pos] == id, 0);
      ++rm_lost_pos;
      continue;
    }
    if (expect_removals && r.tag == 26 && have_pre_order) {   // LinearizationBase::create of the marginalisation: the order BEFORE the removals
      Rd d(r.b); d.get<uint32_t>(); uint32_t nl = d.get<uint32_t>(), nh = d.get<uint32_t>(); uint64_t hl = d.get<uint64_t>(), hh = d.get<uint64_t>();
      cmp("lmdb kpts / host iteration order at the marginalisation's create (before the removals)", pre_hl == hl && pre_nl == nl && pre_hh == hh && pre_nh == nh, 0);
      have_pre_order = false;
      continue;
    }
    if (expect_removals) { cmp("removeLandmark count", rm_lost_pos == rm_expect_lost.size(), 0); expect_removals = false; bs_marg_result_free(&cur); }
    if (r.tag >= 20 && r.tag <= 25) { apply_lmdb_op(v.ba.lmdb, r); continue; }
    if (r.tag == 26) {
      Rd d(r.b); d.get<uint32_t>(); uint32_t nl = d.get<uint32_t>(), nh = d.get<uint32_t>(); uint64_t hl = d.get<uint64_t>(), hh = d.get<uint64_t>();
      uint64_t cl = FNV0, ch = FNV0; uint32_t cnl = 0, cnh = 0;
      for (const bs_hnode* n = v.ba.lmdb.kpts.before_begin.next; n; n = n->next) { uint64_t id = (uint64_t)n->k0; cl = fnvT_(id, cl); ++cnl; }
      for (const bs_hnode* n = v.ba.lmdb.observations.before_begin.next; n; n = n->next) { int64_t f = n->k0; uint64_t c = (uint64_t)n->k1; ch = fnvT_(f, ch); ch = fnvT_(c, ch); ++cnh; }
      cmp("lmdb kpts / host iteration order at LinearizationBase::create (after the C removals)", cl == hl && cnl == nl && ch == hh && cnh == nh, 0);
      continue;
    }
    if (r.tag == 28) { Rd d(r.b); int64_t t = d.get<int64_t>(); std::array<float, 3> bg, bac; d.arr(bg.data(), 3); d.arr(bac.data(), 3); bias_lin[t] = {bg, bac}; continue; }
    if (r.tag != 30) continue;
    if (max_problems >= 0 && n_calls >= max_problems) break;
    ProbRec P;
    if (!parse_problem(r, bias_lin, P)) return 2;
    ++n_problem;
    if (!lmdb_check_install(v.ba.lmdb, P, "replay problem")) continue;
    if (P.kind != 1) continue;
    auto mi = margs.find(P.t_ns); auto ii = ins.find(P.t_ns); auto oi = outs.find(P.t_ns);
    if (mi == margs.end() || ii == ins.end() || oi == outs.end() || !mi->second.full) { std::printf("missing MARG / MARG_IN / MARG_OUT for t=%" PRId64 "\n", P.t_ns); continue; }
    const MargDump& M = mi->second; const MargIn& I = ii->second; const MargOut& O = oi->second;
    ++n_calls;
    // ---- the estimator state at the entry of marginalize(): poses / states / prior from the PROBLEM record, the rest from MARG_IN
    vio_set_from_problem(v, P);
    for (int64_t k : I.imu) {      // imu_meas entries the aom filter dropped (their end is not in the aom): placeholders that cannot be selected
      if (!bs_vio_imu_insert(&v, k)) continue;
    }
    for (int i = 0; i < v.n_imu; ++i) { bool dumped = false; for (const auto& m : P.imus) if (m.start_t_ns == v.imu[i].start_t_ns) dumped = true; if (!dumped) { v.imu[i].delta.t_ns = 1; } }
    // the dumped imu entries were selected by "start and end in the aom": placeholders get dt = 1 ns, which is never an aom member
    v.n_kf = 0; for (int64_t t : I.kf) bs_vio_kf_insert(&v, t);
    v.n_npk = 0; for (auto& p : I.npk) bs_vio_npk_set(&v, p.first, p.second);
    v.max_states = (int)I.max_states; v.max_kfs = (int)I.max_kfs; v.kf_marg_feature_ratio = I.ratio; v.opt_started = 1; v.last_state_t_ns = P.t_ns;
    std::vector<bs_kf_count> conn; for (auto& p : I.conn) conn.push_back(bs_kf_count{p.first, p.second});
    ++g_cmp[T8 + ": prior in (PROBLEM record == MARG record H, b)"];
    if (P.has_marg && (P.rows != (int)M.prow || P.cols != (int)M.pcol || std::memcmp(P.mgH.data(), M.priorH.data(), 4 * M.priorH.size()) || std::memcmp(P.mgb.data(), M.priorb.data(), 4 * M.priorb.size()))) ++g_bad[T8 + ": prior in (PROBLEM record == MARG record H, b)"];
    {   // the iteration orders of the C lmdb at the time the C++ builds its LinearizationBase (before the removals)
      pre_hl = FNV0; pre_hh = FNV0; pre_nl = pre_nh = 0;
      for (const bs_hnode* n = v.ba.lmdb.kpts.before_begin.next; n; n = n->next) { uint64_t id = (uint64_t)n->k0; pre_hl = fnvT_(id, pre_hl); ++pre_nl; }
      for (const bs_hnode* n = v.ba.lmdb.observations.before_begin.next; n; n = n->next) { int64_t f = n->k0; uint64_t c = (uint64_t)n->k1; pre_hh = fnvT_(f, pre_hh); pre_hh = fnvT_(c, pre_hh); ++pre_nh; }
      have_pre_order = true;
    }
    bs_marg_result res;
    bs_vio_marginalize(&v, conn.data(), (int)conn.size(), P.lost.data(), (int)P.lost.size(), &res);
    cur = res; expect_removals = true; rm_expect_lost = P.lost; rm_lost_pos = 0;
    const int64_t t = P.t_ns;
    cmp("marginalize ran (MARG record exists) and no assertion condition", res.marginalized && !res.layout_error, t);
    if (!res.marginalized) { ++n_none; continue; }
    cmp("last_state_to_marg", res.last_state_to_marg == M.last_state_to_marg, t);
    cmp("aom (items, offsets, total)", (int)M.aom.size() == res.aom_n && res.aom_total == (int)M.aom_total && (M.aom.empty() || !std::memcmp(M.aom.data(), res.aom, sizeof(bs_aom_item) * M.aom.size())) && M.aom_items == (uint32_t)res.aom_n, t);
    cmp("kf_ids before, kfs_to_marg (selection)", (int)M.kfs.size() == res.n_kfs && (res.n_kfs == 0 || !std::memcmp(M.kfs.data(), res.kfs_to_marg, 8 * (size_t)res.n_kfs)) && (int)M.kf_all.size() == res.n_kf_all && !std::memcmp(M.kf_all.data(), res.kf_ids_all, 8 * (size_t)res.n_kf_all), t);
    cmp("set sizes (poses_to_marg, states all / vel_bias)", M.n_poses_to_marg == (uint32_t)res.n_poses_to_marg && M.n_states_all == (uint32_t)res.n_states_all && M.n_states_vb == (uint32_t)res.n_states_vb, t);
    cmp("idx_to_keep, idx_to_marg", (int)M.keep.size() == res.n_keep && (int)M.marg.size() == res.n_marg && !std::memcmp(M.keep.data(), res.idx_to_keep, 4 * (size_t)res.n_keep) && !std::memcmp(M.marg.data(), res.idx_to_marg, 4 * (size_t)res.n_marg), t);
    cmp("Q2Jp rows x cols (size)", (int)M.qrows == res.q2_rows && (int)M.qcols == res.aom_total, t);
    cmp("marg_H_new (marginalizeHelperSqrtToSqrt), rows x cols and values", (int)M.orows == v.marg.rows && (int)M.ocols == v.marg.cols && !std::memcmp(M.Hnew.data(), v.marg.H, 4 * M.Hnew.size()) && hash_mat(v.marg.H, v.marg.rows, v.marg.cols) == M.hHnew, t);
    cmp("marg_b_new", (int)M.nb == res.n_b_new && !std::memcmp(M.bnew.data(), res.b_new, 4 * M.bnew.size()) && hash_mat(res.b_new, res.n_b_new, 1) == M.hbnew, t);
    cmp("final marg_data.b (after - H * delta)", !std::memcmp(M.bfinal.data(), v.marg.b, 4 * M.bfinal.size()) && hash_mat(v.marg.b, v.marg.rows, 1) == M.hfinal && M.order_total == (uint32_t)v.marg.total, t);
    // MARG_OUT
    cmp("kf_ids after", (int)O.kf.size() == v.n_kf && (v.n_kf == 0 || !std::memcmp(O.kf.data(), v.kf_ids, 8 * (size_t)v.n_kf)), t);
    {
      bool ok = (int)O.poses.size() == v.ba.n_poses;
      for (int i = 0; ok && i < v.ba.n_poses; ++i) if (O.poses[i].first != v.ba.poses[i].t_ns || O.poses[i].second != (uint32_t)v.ba.poses[i].linearized) ok = false;
      cmp("frame_poses keys and linearized flags after", ok, t);
      ok = (int)O.states.size() == v.ba.n_states;
      for (int i = 0; ok && i < v.ba.n_states; ++i) if (O.states[i].first != v.ba.states[i].t_ns || O.states[i].second != (uint32_t)v.ba.states[i].s.linearized) ok = false;
      cmp("frame_states keys and linearized flags after", ok, t);
      ok = (int)O.imu.size() == v.n_imu;
      for (int i = 0; ok && i < v.n_imu; ++i) if (O.imu[i] != v.imu[i].start_t_ns) ok = false;
      cmp("imu_meas keys after (erase of the marginalised states)", ok, t);
      // prev_opt_flow_res: erase(states_to_marg_all), erase(poses_to_marg) of the keys at entry
      std::set<int64_t> prev(I.prev.begin(), I.prev.end());
      for (int i = 0; i < res.n_states_all; ++i) prev.erase(res.states_to_marg_all[i]);
      for (int i = 0; i < res.n_poses_to_marg; ++i) prev.erase(res.poses_to_marg[i]);
      ok = prev.size() == O.prev.size() && std::equal(prev.begin(), prev.end(), O.prev.begin());
      cmp("prev_opt_flow_res keys after (from states_to_marg_all / poses_to_marg)", ok, t);
    }
    cmp("marg order (items, offsets, total), prior rows x cols, hash(H), hash(b)", (int)O.order.size() == v.marg.n && !std::memcmp(O.order.data(), v.marg.item, sizeof(bs_aom_item) * O.order.size()) && O.order_total == (uint32_t)v.marg.total &&
                                                                                      O.hrows == (uint32_t)v.marg.rows && O.hcols == (uint32_t)v.marg.cols && O.hH == hash_mat(v.marg.H, v.marg.rows, v.marg.cols) && O.hb == hash_mat(v.marg.b, v.marg.rows, 1), t);
    cmp("state digest (frame_poses + frame_states) after", O.sdig == vio_state_digest(v), t);
    {
      uint64_t cl = FNV0, ch = FNV0;
      for (const bs_hnode* n = v.ba.lmdb.kpts.before_begin.next; n; n = n->next) { uint64_t id = (uint64_t)n->k0; cl = fnvT_(id, cl); }
      for (const bs_hnode* n = v.ba.lmdb.observations.before_begin.next; n; n = n->next) { int64_t f = n->k0; uint64_t c = (uint64_t)n->k1; ch = fnvT_(f, ch); ch = fnvT_(c, ch); }
      cmp("lmdb after removeKeyframes / removeLandmark: kpts order, host order, landmark values, counts", O.lmo == cl && O.hos == ch && O.lmv == lm_value_digest(v.ba.lmdb) && O.nlm == v.ba.lmdb.kpts.nelem && (int)O.nobs == bs_lmdb_num_observations(&v.ba.lmdb), t);
    }
    max_q2 = std::max<long>(max_q2, res.q2_rows);
    if (res.n_kfs > 0) ++n_kfsel;
    if (res.n_kfs > 1) ++n_multi_kf;
    if (res.n_states_vb > 0) ++n_vb;
    if (!P.lost.empty()) ++n_lost;
  }
  if (expect_removals) bs_marg_result_free(&cur);
  bs_vio_destroy(&v);
  std::printf("replay8 C counters: %ld marginalizations, %ld kfs chosen by the feature ratio, %ld by the DSO score, %ld rank-deficient columns\n", bs_marg_stats[0], bs_marg_stats[1], bs_marg_stats[2], bs_marg_stats[3]);
  std::printf("replay8 %s: %ld PROBLEM records read, %ld marginalizations replayed (%zu MARG records in the dump), %ld with kfs_to_marg (%ld with >= 2), %ld with vel/bias states, %ld with lost landmarks, max Q2 rows %ld, bs_marg_oob %d, bs_lmdb_ub %d, bs_la_unsupported %d\n",
              dir.c_str(), n_problem, n_calls, margs.size(), n_kfsel, n_multi_kf, n_vb, n_lost, max_q2, bs_marg_oob, bs_lmdb_ub, bs_la_unsupported);
  return 0;
}

int main(int argc, char** argv) {
  std::string mode = argc > 1 ? argv[1] : "helper";
  long seed = argc > 2 ? atol(argv[2]) : 1, cases = argc > 3 ? atol(argv[3]) : 1000;
  tbb::global_control tbb_one(tbb::global_control::max_allowed_parallelism, 1);
  Rng r(seed);
  if (mode == "helper") helper_cases(r, cases);
  if (mode == "marginalize") marginalize_cases(r, cases);
  if (mode == "replay") { int rc = replay8(argv[2], argc > 3 ? atol(argv[3]) : -1); if (rc) return rc; }
  long tot = 0, bad = 0;
  for (auto& kv : g_cmp) { std::printf("  %-60s %10ld compared %8ld mismatches\n", kv.first.c_str(), kv.second, g_bad[kv.first]); tot += kv.second; bad += g_bad[kv.first]; }
  std::printf("%s seed %ld cases %ld: %ld comparisons, %ld mismatches\n", mode.c_str(), seed, cases, tot, bad);
  return bad != 0;
}
