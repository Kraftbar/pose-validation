// Bit-exactness replay of stella_port/c/sv_g2o_sim3.c against the REAL g2o + stella_vslam classes:
//
//  A. pose graph -- optimize::graph_optimizer's problem: internal::sim3::shot_vertex, graph_opt_edge, Levenberg,
//     BlockSolver_7_3 + g2o::LinearSolverEigen (the solver STELLA_PORT_EIGEN_SOLVER=1 selects in the reference),
//     terminate_action(1e-3), optimize(50); random graphs (chains, extra loop / covisibility edges in both
//     id orders so that transposed Hessian blocks occur, fixed vertices, fix_scale on/off).
//  B. transform optimizer -- optimize::transform_optimizer::optimize's problem: one transform_vertex,
//     perspective_forward/backward_reproj_edge pairs with Huber kernels, BlockSolverX + LinearSolverEigen,
//     optimize(5), outlier rejection at chi_sq = 10, optimize(10), inlier count.
//
// Compared bit for bit: final vertex estimates, number of iterations, chi2, inlier flags / count.
// Build + run: python3 tools/dump_stella_loop.py --build-tools ; or by hand, see tools/dump_stella_loop.py.
#include "stella_vslam/type.h"
#include "stella_vslam/optimize/terminate_action.h"
#include "stella_vslam/optimize/internal/sim3/shot_vertex.h"
#include "stella_vslam/optimize/internal/sim3/graph_opt_edge.h"
#include "stella_vslam/optimize/internal/sim3/transform_vertex.h"
#include "stella_vslam/optimize/internal/sim3/forward_reproj_edge.h"
#include "stella_vslam/optimize/internal/sim3/backward_reproj_edge.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <random>

extern "C" {
#include "../c/sv_g2o_sim3.h"
}

using namespace stella_vslam;

static std::mt19937_64 rng(4242);
static double urand(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double nrand(double sd) { return std::normal_distribution<double>(0.0, sd)(rng); }
static int irand(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

static bool same(double a, double b) { return std::memcmp(&a, &b, 8) == 0 || (a == 0.0 && b == 0.0); }

static void to_c(const g2o::Sim3& a, sv_sim3* c) {
    c->r.x = a.rotation().x();
    c->r.y = a.rotation().y();
    c->r.z = a.rotation().z();
    c->r.w = a.rotation().w();
    for (int i = 0; i < 3; ++i) c->t[i] = a.translation()(i);
    c->s = a.scale();
}
static bool eq(const g2o::Sim3& a, const sv_sim3& c) {
    return same(a.rotation().x(), c.r.x) && same(a.rotation().y(), c.r.y) && same(a.rotation().z(), c.r.z) &&
           same(a.rotation().w(), c.r.w) && same(a.translation()(0), c.t[0]) && same(a.translation()(1), c.t[1]) &&
           same(a.translation()(2), c.t[2]) && same(a.scale(), c.s);
}
static g2o::Sim3 random_sim3(double rot, double trans, double logscale) {
    Vec7_t u;
    Vec3_t ax(nrand(1), nrand(1), nrand(1));
    ax.normalize();
    u << ax * urand(0, rot), nrand(trans), nrand(trans), nrand(trans), nrand(logscale);
    return g2o::Sim3(u);
}

// ------------------------------------------------------------------------------------------------
static long graph_bad = 0, graph_cases = 0, graph_iters = 0;

static void run_graph_case(int case_idx) {
    const int nv = irand(4, 36);
    const bool fix_scale = (case_idx % 5 == 4);
    std::vector<unsigned int> ids(nv);
    unsigned int id = irand(0, 3);
    for (int i = 0; i < nv; ++i) {
        ids[i] = id;
        id += irand(1, 3);
    }
    std::vector<g2o::Sim3> gt(nv), init(nv);
    for (int i = 0; i < nv; ++i) {
        gt[i] = i == 0 ? g2o::Sim3() : random_sim3(0.5, 0.5, fix_scale ? 0.0 : 0.05) * gt[i - 1];
        init[i] = g2o::Sim3(Vec7_t(Vec7_t::Zero() + Vec7_t::Random() * 0.0)) * gt[i];
        Vec7_t n;
        n << nrand(0.02), nrand(0.02), nrand(0.02), nrand(0.05), nrand(0.05), nrand(0.05), fix_scale ? 0.0 : nrand(0.02);
        init[i] = g2o::Sim3(n) * gt[i];
    }
    std::vector<char> fixed(nv, 0);
    fixed[0] = 1;
    if (case_idx % 3 == 0) fixed[nv - 1] = 1;
    // edges: (a, b) index pairs, each with measurement Sim3_ba = T_b T_a^-1 (perturbed)
    std::vector<std::pair<int, int>> pairs;
    for (int i = 1; i < nv; ++i) {
        const int parent = irand(std::max(0, i - 3), i - 1);
        pairs.push_back(irand(0, 3) == 0 ? std::make_pair(parent, i) : std::make_pair(i, parent));
    }
    const int extra = irand(0, nv);
    for (int k = 0; k < extra; ++k) {
        const int a = irand(0, nv - 1), b = irand(0, nv - 1);
        if (a != b) pairs.emplace_back(a, b);
    }
    std::vector<g2o::Sim3> meas;
    for (auto& p : pairs) {
        Vec7_t n;
        n << nrand(0.005), nrand(0.005), nrand(0.005), nrand(0.01), nrand(0.01), nrand(0.01), fix_scale ? 0.0 : nrand(0.005);
        meas.push_back(g2o::Sim3(n) * (gt[p.second] * gt[p.first].inverse()));
    }

    // ---- real g2o ----
    auto linear_solver = stella_vslam::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
    auto block_solver = stella_vslam::make_unique<g2o::BlockSolver_7_3>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    g2o::SparseOptimizer optimizer;
    auto terminateAction = new optimize::terminate_action;
    terminateAction->setGainThreshold(1e-3);
    optimizer.addPostIterationAction(terminateAction);
    optimizer.setAlgorithm(algorithm);
    std::vector<int> order(nv);
    for (int i = 0; i < nv; ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), rng);
    std::vector<optimize::internal::sim3::shot_vertex*> vtx(nv);
    for (int oi : order) {
        auto v = new optimize::internal::sim3::shot_vertex();
        v->setEstimate(init[oi]);
        v->setFixed(fixed[oi] != 0);
        v->setId(ids[oi]);
        v->fix_scale_ = fix_scale;
        optimizer.addVertex(v);
        vtx[oi] = v;
    }
    for (size_t k = 0; k < pairs.size(); ++k) {
        auto edge = new optimize::internal::sim3::graph_opt_edge();
        edge->setVertex(0, vtx[pairs[k].first]);
        edge->setVertex(1, vtx[pairs[k].second]);
        edge->setMeasurement(meas[k]);
        edge->information() = MatRC_t<7, 7>::Identity();
        optimizer.addEdge(edge);
    }
    optimizer.initializeOptimization();
    const int iters = optimizer.optimize(50);

    // ---- port ----
    sv_s3_graph g;
    sv_s3_graph_init(&g);
    g.fix_scale = fix_scale;
    g.use_terminate = 1;
    g.gain_threshold = 1e-3;
    std::vector<int> sorted(nv);
    for (int i = 0; i < nv; ++i) sorted[i] = i;
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) { return ids[a] < ids[b]; });
    std::vector<int> cidx(nv);
    for (int k = 0; k < nv; ++k) {
        sv_sim3 c;
        to_c(init[sorted[k]], &c);
        cidx[sorted[k]] = sv_s3_add_vertex(&g, ids[sorted[k]], &c, fixed[sorted[k]]);
    }
    for (size_t k = 0; k < pairs.size(); ++k) {
        sv_sim3 c;
        to_c(meas[k], &c);
        sv_s3_add_graph_edge(&g, cidx[pairs[k].first], cidx[pairs[k].second], &c);
    }
    sv_s3_initialize_optimization(&g);
    const int citers = sv_s3_optimize(&g, 50);

    bool ok = (iters == citers);
    for (int i = 0; i < nv && ok; ++i) ok = ok && eq(vtx[i]->estimate(), g.v[cidx[i]].est);
    // chi2 after the last iteration (stored errors)
    const double chi_real = optimizer.activeChi2();
    double chi_c = 0.0;
    for (int k = 0; k < g.n_active_edges; ++k) chi_c += sv_s3_edge_chi2(&g.e[g.active_edges[k]]);
    ok = ok && same(chi_real, chi_c);
    ++graph_cases;
    graph_iters += iters;
    if (!ok) {
        if (graph_bad < 5) std::printf("MISMATCH graph case %d (nv=%d edges=%zu iters real=%d c=%d)\n", case_idx, nv, pairs.size(), iters, citers);
        ++graph_bad;
    }
    sv_s3_graph_free(&g);
}

// ------------------------------------------------------------------------------------------------
static long tr_bad = 0, tr_cases = 0, tr_outliers = 0;

static void random_rot(Mat33_t& R, double angle) {
    Vec3_t ax(nrand(1), nrand(1), nrand(1));
    ax.normalize();
    R = Eigen::AngleAxisd(urand(0, angle), ax).toRotationMatrix();
}

static void run_transform_case(int case_idx) {
    const double fx = 517.3 + nrand(20), fy = 516.5 + nrand(20), cx = 318.6, cy = 255.3;
    const bool fix_scale = (case_idx % 6 == 5);
    Mat33_t R1, R2;
    random_rot(R1, 1.0);
    random_rot(R2, 1.0);
    const Vec3_t t1(nrand(1), nrand(1), nrand(1)), t2(nrand(1), nrand(1), nrand(1));
    const g2o::Sim3 s12_true = random_sim3(0.15, 0.1, fix_scale ? 0.0 : 0.1);
    const int n = irand(12, 90);
    const double out_frac = (case_idx % 4 == 0) ? 0.35 : 0.1;
    struct M {
        sv_transform_match c;
        Vec2_t obs1, obs2;
        Vec3_t p2w, p1w;
        float info1, info2;
    };
    std::vector<M> ms(n);
    for (int i = 0; i < n; ++i) {
        M& m = ms[i];
        const Vec3_t X(nrand(1.5), nrand(1.5), urand(2, 6));
        const Vec3_t pos_2 = R2 * X + t2;
        const Vec3_t pos_1 = s12_true.map(pos_2);
        const bool bad = urand(0, 1) < out_frac;
        m.obs1 = Vec2_t(fx * pos_1(0) / pos_1(2) + cx + nrand(bad ? 25 : 0.7), fy * pos_1(1) / pos_1(2) + cy + nrand(bad ? 25 : 0.7));
        m.obs2 = Vec2_t(fx * pos_2(0) / pos_2(2) + cx + nrand(bad ? 25 : 0.7), fy * pos_2(1) / pos_2(2) + cy + nrand(bad ? 25 : 0.7));
        m.p2w = X;
        m.p1w = R1.transpose() * (pos_1 - t1);
        static const float tab[8] = {1.f, 1.f / 1.44f, 1.f / 2.0736f, 1.f / 2.985984f, 1.f / 4.29981696f, 1.f / 6.1917364f, 1.f / 8.916100f, 1.f / 12.839185f};
        m.info1 = tab[irand(0, 7)];
        m.info2 = tab[irand(0, 7)];
        m.c.idx1 = i;
        for (int k = 0; k < 2; ++k) {
            m.c.obs1[k] = m.obs1(k);
            m.c.obs2[k] = m.obs2(k);
        }
        m.c.info1 = m.info1;
        m.c.info2 = m.info2;
        for (int k = 0; k < 3; ++k) {
            m.c.pos_w_2[k] = m.p2w(k);
            m.c.pos_w_1[k] = m.p1w(k);
        }
    }
    Vec7_t noise;
    noise << nrand(0.03), nrand(0.03), nrand(0.03), nrand(0.05), nrand(0.05), nrand(0.05), fix_scale ? 0.0 : nrand(0.03);
    const g2o::Sim3 init = g2o::Sim3(noise) * s12_true;
    const float chi_sq = 10.0f;
    const float sqrt_chi_sq = std::sqrt(chi_sq);
    const unsigned int num_iter = 10;

    // ---- real (a transcription of transform_optimizer::optimize with the camera / keyframe lookups removed) ----
    auto linear_solver = stella_vslam::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = stella_vslam::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);
    auto vtx = new optimize::internal::sim3::transform_vertex();
    vtx->setId(0);
    vtx->setEstimate(init);
    vtx->setFixed(false);
    vtx->fix_scale_ = fix_scale;
    vtx->rot_1w_ = R1;
    vtx->trans_1w_ = t1;
    vtx->rot_2w_ = R2;
    vtx->trans_2w_ = t2;
    optimizer.addVertex(vtx);
    std::vector<optimize::internal::sim3::base_forward_reproj_edge*> e12(n);
    std::vector<optimize::internal::sim3::base_backward_reproj_edge*> e21(n);
    for (int i = 0; i < n; ++i) {
        auto f = new optimize::internal::sim3::perspective_forward_reproj_edge();
        f->setMeasurement(ms[i].obs1);
        f->setInformation(Mat22_t::Identity() * ms[i].info1);
        f->pos_w_ = ms[i].p2w;
        f->fx_ = fx;
        f->fy_ = fy;
        f->cx_ = cx;
        f->cy_ = cy;
        f->setVertex(0, vtx);
        auto hk = new g2o::RobustKernelHuber();
        hk->setDelta(sqrt_chi_sq);
        f->setRobustKernel(hk);
        e12[i] = f;
        auto b = new optimize::internal::sim3::perspective_backward_reproj_edge();
        b->setMeasurement(ms[i].obs2);
        b->setInformation(Mat22_t::Identity() * ms[i].info2);
        b->pos_w_ = ms[i].p1w;
        b->fx_ = fx;
        b->fy_ = fy;
        b->cx_ = cx;
        b->cy_ = cy;
        b->setVertex(0, vtx);
        auto hk2 = new g2o::RobustKernelHuber();
        hk2->setDelta(sqrt_chi_sq);
        b->setRobustKernel(hk2);
        e21[i] = b;
        optimizer.addEdge(f);
        optimizer.addEdge(b);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(5);
    const g2o::Sim3 mid_real = vtx->estimate();
    std::vector<unsigned char> rej_real(n, 0);
    unsigned int num_outliers = 0;
    for (int i = 0; i < n; ++i) {
        if (e12[i]->chi2() < chi_sq && e21[i]->chi2() < chi_sq) continue;
        rej_real[i] = 1;
        e12[i]->setLevel(1);
        e21[i]->setLevel(1);
        ++num_outliers;
    }
    unsigned int inl_real = 0;
    g2o::Sim3 fin_real = init;
    bool early = false;
    if ((unsigned)n - num_outliers < 10) {
        early = true;
    }
    else {
        optimizer.initializeOptimization();
        optimizer.optimize(num_iter);
        for (int i = 0; i < n; ++i) {
            if (rej_real[i]) continue;
            if (chi_sq < e12[i]->chi2() || chi_sq < e21[i]->chi2()) {
                rej_real[i] = 2;
                continue;
            }
            ++inl_real;
        }
        fin_real = vtx->estimate();
    }
    tr_outliers += num_outliers;

    // ---- port ----
    std::vector<sv_transform_match> cm(n);
    for (int i = 0; i < n; ++i) cm[i] = ms[i].c;
    sv_transform_camera cam = {fx, fy, cx, cy};
    double c_R1[9], c_R2[9], c_t1[3], c_t2[3];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            c_R1[c * 3 + r] = R1(r, c);
            c_R2[c * 3 + r] = R2(r, c);
        }
        c_t1[r] = t1(r);
        c_t2[r] = t2(r);
    }
    sv_sim3 csim, cmid;
    to_c(init, &csim);
    std::vector<unsigned char> rej_c(n, 0);
    const unsigned int inl_c = sv_transform_optimize(&cam, &cam, c_R1, c_t1, c_R2, c_t2, cm.data(), n, rej_c.data(), &csim,
                                                     chi_sq, fix_scale, num_iter, &cmid);
    bool ok = (inl_c == (early ? 0u : inl_real)) && eq(mid_real, cmid) && (early || eq(fin_real, csim)) && (early || eq(init, csim) || true);
    for (int i = 0; i < n && ok; ++i) ok = ok && rej_real[i] == rej_c[i];
    if (early) ok = ok && eq(init, csim);
    ++tr_cases;
    if (!ok) {
        if (tr_bad < 5) std::printf("MISMATCH transform case %d (n=%d outliers=%u inl real=%u c=%u early=%d)\n", case_idx, n, num_outliers, inl_real, inl_c, (int)early);
        ++tr_bad;
    }
}

int main(int argc, char** argv) {
    const int ng = argc > 1 ? std::atoi(argv[1]) : 400;
    const int nt = argc > 2 ? std::atoi(argv[2]) : 400;
    for (int i = 0; i < ng; ++i) run_graph_case(i);
    for (int i = 0; i < nt; ++i) run_transform_case(i);
    std::printf("replay_sim3_opt: pose graph %ld/%ld cases differ (%ld LM iterations in total); transform optimizer %ld/%ld differ (%ld rejected matches)\n",
                graph_bad, graph_cases, graph_iters, tr_bad, tr_cases, tr_outliers);
    return (graph_bad || tr_bad) ? 1 : 0;
}
