// stella_port reference tool, module 4b part 2: re-solve captured BA graphs
// with the REAL g2o LinearSolverEigen (SimplicialLLT + AMDOrdering).
//
// Why: global_bundle_adjuster uses g2o::LinearSolverCSparse upstream, which
// the port deliberately replaces by the SimplicialLLT path (no CSparse). The
// local BA Schur systems of fr1_xyz / fr1_desk are block-dense (every
// keyframe pair shares landmarks), so AMD returns the identity there; only
// the big loop-BA calls on the final map (54 keyframes on fr1_desk, 1125 of
// 1485 block entries) get a non-trivial AMD permutation. To validate the
// port's permuted-AMD + sparse-LLT path end to end against the real library
// (rather than only against CSparse), this tool rebuilds each captured
// global graph (kind 1 and kind 2 calls from ba_calls.bin: the same
// vertices/edges the real function built, in the same order) with stella's
// own header-only vertex/edge classes, solves it with the real
// g2o::LinearSolverEigen + OptimizationAlgorithmLevenberg + stella's
// terminate_action, and writes the per-iteration robust chi2 / lambda /
// Levenberg trials and the final vertex estimates to
//   <dir>/ba_eigen_replay.bin
// which check_sv_g2o_ba.c compares bit for bit against the port (tolerance
// 0), and against the CSparse-produced reference to quantify the
// CSparse-vs-Eigen deviation.
//
// Clean-room: g2o core/solver_eigen (BSD) and stella_vslam (BSD-2) only.
//
// usage: replay_ba_eigen <dir containing ba_calls.bin>

#include "stella_vslam/type.h"
#include "stella_vslam/optimize/terminate_action.h"
#include "stella_vslam/optimize/internal/landmark_vertex.h"
#include "stella_vslam/optimize/internal/se3/shot_vertex.h"
#include "stella_vslam/optimize/internal/se3/perspective_reproj_edge.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

using namespace stella_vslam;
using namespace stella_vslam::optimize;

struct Rd {
    const unsigned char* p;
    const unsigned char* end;
    uint32_t u32() { uint32_t v; std::memcpy(&v, p, 4); p += 4; return v; }
    int32_t i32() { return (int32_t)u32(); }
    float f32() { float v; std::memcpy(&v, p, 4); p += 4; return v; }
    double f64() { double v; std::memcpy(&v, p, 8); p += 8; return v; }
    void skip(size_t n) { p += n; }
};

struct RVert { uint32_t id, is_lm, fixed; double init[7]; };
struct REdge { uint32_t lm_vtx, kf_vtx; double obs[2], isq, delta; uint32_t has_kernel; };

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "usage: replay_ba_eigen <dir>\n";
        return 1;
    }
    const std::string dir = argv[1];
    std::ifstream in(dir + "/ba_calls.bin", std::ios::binary);
    std::vector<unsigned char> buf((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    Rd r{buf.data(), buf.data() + buf.size()};
    if (r.u32() != 0x32434142u) {
        std::cerr << "bad magic\n";
        return 2;
    }
    const uint32_t n_calls = r.u32();
    std::ofstream out(dir + "/ba_eigen_replay.bin", std::ios::binary);
    auto w32 = [&](uint32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); };
    auto wi32 = [&](int32_t v) { out.write(reinterpret_cast<const char*>(&v), 4); };
    auto w64 = [&](double v) { out.write(reinterpret_cast<const char*>(&v), 8); };
    w32(0x31525245u);
    std::streampos count_pos = out.tellp();
    w32(0);
    uint32_t n_written = 0;

    for (uint32_t ci = 0; ci < n_calls; ++ci) {
        const uint32_t kind = r.u32();
        const uint32_t call_index = r.u32();
        r.skip(4 * 4); // curr_keyfrm_id, fixed_thr, num_first, num_second  (re-read num_first below)
        // rewind the two fields we need: layout is fixed, so parse by offsets
        const unsigned char* base = r.p - 16;
        uint32_t num_first;
        std::memcpy(&num_first, base + 8, 4);
        double gain = r.f64();
        const uint32_t use_huber = r.u32();
        r.u32(); // fix_markers
        r.u32(); // flag_given
        r.u32(); // flag_in
        r.u32(); // use_additional
        const uint32_t has_markers = r.u32();
        const uint32_t ran = r.u32();
        r.u32(); // returned_ok
        const double fx = r.f64(), fy = r.f64(), cx = r.f64(), cy = r.f64();
        r.skip(4 * r.u32()); // isq
        r.skip(4 * r.u32()); // order kf
        r.skip(4 * r.u32()); // order lm
        const uint32_t nk = r.u32();
        for (uint32_t i = 0; i < nk; ++i) {
            r.skip(12 + 128);
            r.u32(); // has_slots
            r.skip(4 * r.u32());
            r.skip(16 * r.u32());
        }
        const uint32_t nl = r.u32();
        for (uint32_t i = 0; i < nl; ++i) {
            r.skip(8 + 24);
            r.skip(8 * r.u32());
        }
        std::vector<RVert> verts(r.u32());
        for (auto& v : verts) {
            v.id = r.u32(); v.is_lm = r.u32(); v.fixed = r.u32(); r.u32();
            for (int j = 0; j < 7; ++j) v.init[j] = r.f64();
            r.skip(56); // final
        }
        std::vector<REdge> edges(r.u32());
        for (auto& e : edges) {
            e.lm_vtx = r.u32(); e.kf_vtx = r.u32(); r.skip(12);
            e.obs[0] = r.f64(); e.obs[1] = r.f64(); e.isq = r.f64(); e.delta = r.f64(); e.has_kernel = r.u32();
            r.skip(8 * 3 + 8); // chi2, err[2], depth+level
        }
        // stages, outliers, applied, opt: skip
        const uint32_t ns = r.u32();
        for (uint32_t s = 0; s < ns; ++s) {
            r.skip(16);
            const uint32_t nlv = r.u32();
            r.skip(nlv + (4 - nlv % 4) % 4);
            r.skip(24 * r.u32());
        }
        r.skip(8 * r.u32());
        { const uint32_t na = r.u32(); r.skip((size_t)na * (4 + 128)); }
        r.skip(4 * r.u32());

        if (kind == 0 || has_markers || !ran) {
            continue;
        }

        // ---- rebuild with the real g2o and solve with LinearSolverEigen
        std::unique_ptr<g2o::BlockSolverBase> block_solver;
        auto linear_solver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
        block_solver = std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
        auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));
        g2o::SparseOptimizer optimizer;
        auto terminateAction = new terminate_action;
        terminateAction->setGainThreshold(gain);
        optimizer.addPostIterationAction(terminateAction);
        optimizer.setAlgorithm(algorithm);

        std::vector<g2o::OptimizableGraph::Vertex*> vptr;
        for (const auto& v : verts) {
            if (v.is_lm) {
                auto* lv = new internal::landmark_vertex();
                lv->setId(v.id);
                lv->setEstimate(Vec3_t(v.init[0], v.init[1], v.init[2]));
                lv->setFixed(v.fixed != 0);
                lv->setMarginalized(true);
                optimizer.addVertex(lv);
                vptr.push_back(lv);
            } else {
                auto* sv = new internal::se3::shot_vertex();
                sv->setId(v.id);
                Eigen::Quaterniond q(v.init[3], v.init[0], v.init[1], v.init[2]);
                // exact SE3Quat with the dumped (already normalized) coefficients
                g2o::SE3Quat est;
                est.setRotation(q);
                est.setTranslation(Vec3_t(v.init[4], v.init[5], v.init[6]));
                sv->setEstimate(est);
                sv->setFixed(v.fixed != 0);
                optimizer.addVertex(sv);
                vptr.push_back(sv);
            }
        }
        std::map<uint32_t, g2o::OptimizableGraph::Vertex*> by_id;
        for (size_t i = 0; i < verts.size(); ++i) by_id[verts[i].id] = vptr[i];
        for (const auto& e : edges) {
            auto* edge = new internal::se3::mono_perspective_reproj_edge();
            edge->setMeasurement(Vec2_t(e.obs[0], e.obs[1]));
            edge->setInformation(Mat22_t::Identity() * e.isq);
            edge->fx_ = fx;
            edge->fy_ = fy;
            edge->cx_ = cx;
            edge->cy_ = cy;
            edge->setVertex(0, by_id.at(e.lm_vtx));
            edge->setVertex(1, by_id.at(e.kf_vtx));
            if (e.has_kernel) {
                auto* hk = new g2o::RobustKernelHuber();
                hk->setDelta(e.delta);
                edge->setRobustKernel(hk);
            }
            optimizer.addEdge(edge);
        }
        (void)use_huber;

        struct It { double chi2, lambda; int32_t lev; };
        std::vector<It> its;
        // record after each iteration via a tiny action registered AFTER terminate_action
        // (both are in a std::set<HyperGraphAction*>; order is by address, so recompute the errors
        // ourselves and read the state the same way terminate_action leaves it)
        struct Rec : public g2o::HyperGraphAction {
            g2o::OptimizationAlgorithmLevenberg* a;
            std::vector<It>* v;
            g2o::HyperGraphAction* operator()(const g2o::HyperGraph* graph, Parameters* parameters) override {
                auto* opt = const_cast<g2o::SparseOptimizer*>(static_cast<const g2o::SparseOptimizer*>(graph));
                auto* p = static_cast<g2o::HyperGraphAction::ParametersIteration*>(parameters);
                if (p->iteration >= 0) {
                    opt->computeActiveErrors();
                    v->push_back({opt->activeRobustChi2(), a->currentLambda(), a->levenbergIteration()});
                }
                return this;
            }
        } rec;
        rec.a = algorithm;
        rec.v = &its;
        optimizer.addPostIterationAction(&rec);

        optimizer.initializeOptimization();
        const int ret = optimizer.optimize(num_first);

        w32(call_index);
        wi32(ret);
        w32((uint32_t)its.size());
        for (const auto& it : its) {
            w64(it.chi2);
            w64(it.lambda);
            wi32(it.lev);
        }
        w32((uint32_t)verts.size());
        for (size_t i = 0; i < verts.size(); ++i) {
            double est[7] = {};
            if (verts[i].is_lm) {
                auto* lv = static_cast<internal::landmark_vertex*>(vptr[i]);
                est[0] = lv->estimate()(0); est[1] = lv->estimate()(1); est[2] = lv->estimate()(2);
            } else {
                auto* sv = static_cast<internal::se3::shot_vertex*>(vptr[i]);
                const g2o::SE3Quat& e = sv->estimate();
                est[0] = e.rotation().x(); est[1] = e.rotation().y(); est[2] = e.rotation().z(); est[3] = e.rotation().w();
                est[4] = e.translation()(0); est[5] = e.translation()(1); est[6] = e.translation()(2);
            }
            for (int j = 0; j < 7; ++j) w64(est[j]);
        }
        ++n_written;
        (void)kind;
    }
    out.seekp(count_pos);
    w32(n_written);
    std::cerr << "replay_ba_eigen: " << n_written << " global calls re-solved with real LinearSolverEigen\n";
    return 0;
}
