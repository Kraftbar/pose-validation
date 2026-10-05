// RD-VIO M4 oracle (reference tooling, never distributed): drives the REAL Ceres 2.2.0 (the patched copy with the rdvio_port solve hooks,
// rdvio_port/reference/ceres_patches/0006) on RANDOM RD-VIO-shaped problems and writes solve.bin in the layout of rdvio_port/c/rd_solve.h.
// The problems mimic the structure rdvio::Solver builds (frames q(4/3, quaternion manifold) p v bg ba (3), inverse depths (1), reprojection-like
// rows (2) with Cauchy(1) loss, 15-row IMU-like factors, a dense marginalization-like factor over all frames but the last, rotation priors,
// pose-only problems, constant blocks, unreferenced blocks) but every cost function is a SYNTHETIC smooth function of its parameters
// (affine + quadratic, analytic ambient Jacobians): every evaluation is recorded as an ORACLE record (type 6) and replayed by
// rdvio_port/c/check_rd_solve.c, so what is verified is the solver itself (reduction, Schur ordering, AMD of the Schur columns, lexicographic
// residual order, Jacobian assembly with the quaternion manifold and the Cauchy corrector, Jacobi scaling, Dogleg, Schur elimination into the
// block-sparse reduced system, Eigen SimplicialLDLT, user-state updates) against the real library on structures the MH_01 dump never reaches.
// usage: rd_m4_oracle <outdir> [seed] [count]
#include <ceres/ceres.h>
#include <ceres/rdvio_port_solve_hooks.h>
#include <rdvio/estimation/ceres/quaternion_parameterization.h>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace rp = rdvio_port_solve;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }

// r_j = c_j + s_j + 0.25 u_j^2, s = sum_b W_b x_b, u = sum_b V_b x_b; ambient Jacobians (row-major nres x size)
struct Synth : ceres::CostFunction {
    std::vector<int> sizes;
    std::vector<std::vector<double>> W, V;
    std::vector<double> c;
    Synth(int nres, const std::vector<int> &bs, double scale) : sizes(bs) {
        set_num_residuals(nres);
        *mutable_parameter_block_sizes() = bs;
        c.resize(nres);
        for (auto &v : c) v = scale * N();
        for (size_t b = 0; b < bs.size(); ++b) {
            W.emplace_back(nres * bs[b]);
            V.emplace_back(nres * bs[b]);
            for (auto &v : W.back()) v = scale * N() * (I(0, 3) == 0 ? 0.0 : 1.0);
            for (auto &v : V.back()) v = 0.1 * N();
        }
    }
    bool Evaluate(const double *const *x, double *r, double **jac) const override {
        const int n = num_residuals();
        for (int j = 0; j < n; ++j) {
            double s = c[j], u = 0;
            for (size_t b = 0; b < sizes.size(); ++b)
                for (int i = 0; i < sizes[b]; ++i) {
                    s += W[b][j * sizes[b] + i] * x[b][i];
                    u += V[b][j * sizes[b] + i] * x[b][i];
                }
            r[j] = s + 0.25 * u * u;
            if (jac)
                for (size_t b = 0; b < sizes.size(); ++b)
                    if (jac[b])
                        for (int i = 0; i < sizes[b]; ++i)
                            jac[b][j * sizes[b] + i] = W[b][j * sizes[b] + i] + 0.5 * u * V[b][j * sizes[b] + i];
        }
        return true;
    }
};

struct Frame { double q[4], p[3], v[3], bg[3], ba[3]; };

static void write_problem(ceres::Problem &pr, ceres::LossFunction *cauchy, ceres::Manifold *quat, const ceres::Solver::Options &o, uint64_t id, int level) {
    rp::g.solve_id = id;
    rp::g.level = level;
    rp::g.oracle.clear();
    {
        rp::Buf b;
        b.u64(id); b.u32((uint32_t)level);
        b.u32((uint32_t)o.linear_solver_type); b.u32((uint32_t)o.trust_region_strategy_type); b.u32((uint32_t)o.dogleg_type);
        b.u32((uint32_t)o.max_num_iterations); b.u32((uint32_t)o.num_threads);
        b.f64(o.function_tolerance); b.f64(o.gradient_tolerance); b.f64(o.parameter_tolerance);
        b.f64(o.initial_trust_region_radius); b.f64(o.max_trust_region_radius); b.f64(o.min_trust_region_radius);
        b.f64(o.min_relative_decrease); b.f64(o.min_lm_diagonal); b.f64(o.max_lm_diagonal);
        b.u32(o.jacobi_scaling); b.u32(o.use_nonmonotonic_steps); b.u32((uint32_t)o.max_num_consecutive_invalid_steps);
        b.u32((uint32_t)o.linear_solver_ordering_type); b.u32((uint32_t)o.sparse_linear_algebra_library_type);
        b.u32((uint32_t)o.dense_linear_algebra_library_type); b.u32(o.linear_solver_ordering ? 1u : 0u);
        b.u32((uint32_t)o.callbacks.size()); b.u32(o.use_inner_iterations); b.u32(o.dynamic_sparsity);
        b.u32((uint32_t)o.minimizer_type); b.u32((uint32_t)o.preconditioner_type); b.u32(o.use_explicit_schur_complement);
        b.u32((uint32_t)o.max_num_refinement_iterations); b.u32(o.use_mixed_precision_solves);
        b.u32(o.update_state_every_iteration); b.f64(o.max_solver_time_in_seconds); b.u32((uint32_t)o.num_threads);
        rp::g.write(rp::R_SOLVE, b);
    }
    std::vector<double *> pbs;
    pr.GetParameterBlocks(&pbs);
    std::unordered_map<const double *, uint32_t> pidx;
    rp::Buf b;
    b.u32((uint32_t)pbs.size());
    for (size_t i = 0; i < pbs.size(); ++i) {
        double *q = pbs[i];
        pidx[q] = (uint32_t)i;
        const int sz = pr.ParameterBlockSize(q), ts = pr.ParameterBlockTangentSize(q);
        const ceres::Manifold *m = pr.GetManifold(q);
        b.u64((uint64_t)(uintptr_t)q); b.u32((uint32_t)sz); b.u32((uint32_t)ts);
        b.u32(m == nullptr ? 0u : (m == quat ? 1u : 3u));
        b.u32(pr.IsParameterBlockConstant(q) ? 1u : 0u);
        b.f64n(q, (size_t)sz);
    }
    std::vector<ceres::ResidualBlockId> rbs;
    pr.GetResidualBlocks(&rbs);
    b.u32((uint32_t)rbs.size());
    for (auto rb : rbs) {
        const ceres::CostFunction *cf = pr.GetCostFunctionForResidualBlock(rb);
        const ceres::LossFunction *lf = pr.GetLossFunctionForResidualBlock(rb);
        std::vector<double *> blocks;
        pr.GetParameterBlocksForResidualBlock(rb, &blocks);
        rp::g.oracle.insert(cf);
        b.u64((uint64_t)(uintptr_t)rb); b.u32(6);
        b.u32(lf == nullptr ? 0u : (lf == cauchy ? 1u : 2u));
        b.u32((uint32_t)blocks.size());
        for (double *q : blocks) b.u32(pidx.at(q));
        b.u32((uint32_t)cf->num_residuals());
        b.u64(0);
    }
    rp::g.write(rp::R_PROBLEM, b);
    rp::g.armed = true;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: rd_m4_oracle <outdir> [seed] [count]\n"); return 2; }
    const std::string dir = argv[1];
    rng.seed(argc > 2 ? strtoull(argv[2], nullptr, 10) : 1);
    const int count = argc > 3 ? atoi(argv[3]) : 200;
    rp::g.f = fopen((dir + "/solve.bin").c_str(), "wb");
    rp::g.enabled = rp::g.f != nullptr;
    if (!rp::g.enabled) return 1;
    std::unique_ptr<ceres::Manifold> quat = std::make_unique<rdvio::QuaternionParameterization>();
    std::unique_ptr<ceres::LossFunction> cauchy = std::make_unique<ceres::CauchyLoss>(1.0);
    for (int t = 0; t < count; ++t) {
        ceres::Problem::Options po;
        po.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        po.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        po.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        ceres::Problem pr(po);
        std::vector<std::unique_ptr<Synth>> costs;
        const int kind = I(0, 9);  // 0: pose-only, 1: pose + motion priors, else window-like
        const int F = kind == 0 ? 1 : (kind == 1 ? I(1, 2) : I(2, 7));
        const int L = kind <= 1 ? 0 : (I(0, 5) == 0 ? 0 : I(1, 24));
        std::vector<std::unique_ptr<Frame>> fr;
        std::vector<std::unique_ptr<double>> lm;
        const double sc = U(0.3, 1.5);
        for (int i = 0; i < F; ++i) {
            fr.emplace_back(new Frame);
            Frame &f = *fr.back();
            double n = 0;
            for (int k = 0; k < 4; ++k) { f.q[k] = N(); n += f.q[k] * f.q[k]; }
            for (int k = 0; k < 4; ++k) f.q[k] /= std::sqrt(n);
            for (int k = 0; k < 3; ++k) { f.p[k] = N(); f.v[k] = N(); f.bg[k] = 0.05 * N(); f.ba[k] = 0.05 * N(); }
            pr.AddParameterBlock(f.q, 4, quat.get());
            pr.AddParameterBlock(f.p, 3);
            if (kind != 0) { pr.AddParameterBlock(f.v, 3); pr.AddParameterBlock(f.bg, 3); pr.AddParameterBlock(f.ba, 3); }
            if (I(0, 3) == 0 && i == 0) { pr.SetParameterBlockConstant(f.q); pr.SetParameterBlockConstant(f.p); }
            if (kind != 0 && I(0, 5) == 0) { pr.SetParameterBlockConstant(f.v); pr.SetParameterBlockConstant(f.bg); pr.SetParameterBlockConstant(f.ba); }
        }
        for (int l = 0; l < L; ++l) { lm.emplace_back(new double(U(0.2, 2.0))); pr.AddParameterBlock(lm.back().get(), 1); }
        auto add = [&](int nres, std::vector<double *> blocks, std::vector<int> sizes, bool loss) {
            costs.emplace_back(new Synth(nres, sizes, sc));
            pr.AddResidualBlock(costs.back().get(), loss ? cauchy.get() : nullptr, blocks);
        };
        if (kind == 0) {  // pose-only (RPP-like) problem: rows all 2, e = q, f = p
            const int n = I(2, 20);
            for (int k = 0; k < n; ++k) add(2, {fr[0]->q, fr[0]->p}, {4, 3}, true);
        } else if (kind == 1) {  // localize_newframe-like: IMU-prior (frame i-1 live, frame i variable) + priors
            for (int i = 0; i < F; ++i) {
                const int n = I(1, 12);
                for (int k = 0; k < n; ++k) add(2, {fr[i]->q, fr[i]->p}, {4, 3}, true);
                add(15, {fr[i]->q, fr[i]->p, fr[i]->v, fr[i]->bg, fr[i]->ba}, {4, 3, 3, 3, 3}, false);
            }
        } else {
            for (int l = 0; l < L; ++l) {
                const int r = I(0, F - 1);
                const int nobs = I(1, std::min(F - 1, 5));
                for (int k = 0; k < nobs; ++k) {
                    int tg = I(0, F - 2);
                    if (tg >= r) ++tg;   // never the reference frame itself (duplicate blocks are illegal)
                    add(2, {fr[tg]->q, fr[tg]->p, fr[r]->q, fr[r]->p, lm[l].get()}, {4, 3, 4, 3, 1}, true);
                }
            }
            for (int i = 0; i + 1 < F; ++i)
                if (I(0, 4) != 0)
                    add(15, {fr[i]->q, fr[i]->p, fr[i]->v, fr[i]->bg, fr[i]->ba, fr[i + 1]->q, fr[i + 1]->p, fr[i + 1]->v, fr[i + 1]->bg, fr[i + 1]->ba},
                        {4, 3, 3, 3, 3, 4, 3, 3, 3, 3}, false);
            if (F >= 3 && I(0, 2) == 0) {  // marginalization-like: all blocks of the first F-1 frames
                std::vector<double *> bl;
                std::vector<int> sz;
                for (int i = 0; i + 1 < F; ++i) {
                    bl.push_back(fr[i]->q); bl.push_back(fr[i]->p); bl.push_back(fr[i]->v); bl.push_back(fr[i]->bg); bl.push_back(fr[i]->ba);
                    for (int s : {4, 3, 3, 3, 3}) sz.push_back(s);
                }
                add(15 * (F - 1), bl, sz, false);
            }
            for (int i = 0; i < F; ++i)
                if (I(0, 3) == 0) add(2, {fr[i]->q}, {4}, true);                        // rotation prior
            for (int i = 0; i < F; ++i)
                if (I(0, 3) == 0) add(2, {fr[i]->q, fr[i]->p}, {4, 3}, true);           // reprojection prior
        }
        if (pr.NumResidualBlocks() == 0) continue;
        ceres::Solver::Options o;
        o.linear_solver_type = ceres::SPARSE_SCHUR;
        o.trust_region_strategy_type = ceres::DOGLEG;
        o.max_num_iterations = I(2, 30);
        o.max_solver_time_in_seconds = 1e6;
        o.num_threads = 1;
        o.minimizer_progress_to_stdout = false;
        o.update_state_every_iteration = true;
        write_problem(pr, cauchy.get(), quat.get(), o, (uint64_t)t, (t & 1) ? 1 : 2);
        ceres::Solver::Summary summary;
        ceres::Solve(o, &pr, &summary);
        rp::g.armed = false;
        rp::g.oracle.clear();
    }
    fclose(rp::g.f);
    return 0;
}
