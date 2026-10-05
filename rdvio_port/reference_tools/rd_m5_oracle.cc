// RD-VIO M5 oracle (reference tooling, never distributed): drives the REAL rdvio classes (Map, Frame, Track, PreIntegrator,
// CeresReprojectionErrorFactor, CeresPreIntegrationErrorFactor, CeresMarginalizationFactor) on RANDOM windows and chains of marginalisations.
// The marginalize() inputs / outputs are written by the instrumentation of patch 0007 itself (run this program with RDVIO_PORT_DUMP_DIR=<dir>
// RDVIO_PORT_DUMP_EVERY=marg=1 RDVIO_PORT_MARG_FULL_EVERY=n -> marg.bin, layout rdvio_port/c/rd_marg.h); this program additionally writes
// eval.bin: random CeresMarginalizationFactor::Evaluate calls (random Jacobian masks, factor state included).
// (with probability 1/4 per step a marginalize(index > 0) call on the factor is inserted before the real one: the port is generic in the victim index.)
// A chain is: random window of nf frames (2..13) with tracks, factor constructed from the map, then `steps` times { perturb the states (like a
// solver update), random keyframe preintegrations, a few Evaluate calls, Map::marginalize_frame(0), append a new frame that continues some tracks
// and starts new ones }, i.e. the same factor object is marginalised repeatedly exactly as sliding_window_tracker does.
// usage: rd_m5_oracle <outdir> [seed] [chains] [steps]
#include <ceres/rdvio_port_solve_hooks.h>
#include <rdvio/estimation/ceres/marginalization_factor.h>
#include <rdvio/estimation/ceres/preintegration_factor.h>
#include <rdvio/estimation/ceres/reprojection_factor.h>
#include <rdvio/geometry/lie_algebra.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/map.h>
#include <rdvio/map/track.h>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>
using namespace rdvio;
namespace rp = rdvio_port_solve;

#include "rd_world.h"
using namespace rdw;
static void put_factor(FILE *f, const CeresMarginalizationFactor &fac) {
    rp::Buf b;
    fac.port_payload(b);
    W(f, b.b.data(), b.b.size());
}

// one random Evaluate call -> eval.bin
static void eval_record(FILE *fe, const CeresMarginalizationFactor &fac) {
    const std::vector<Frame *> &fr = fac.linearization_frames();
    const int nff = (int)fr.size(), n = nff * ES_SIZE;
    std::vector<std::vector<double>> prm(5 * nff);
    std::vector<const double *> pp(5 * nff);
    for (int i = 0; i < nff; ++i) {
        Frame *f = fr[i];
        quaternion q = (f->pose.q * rquat(0.03)).normalized();
        const double s = I(0, 4) == 0 ? 0.0 : 1.0;   // sometimes exactly the current state
        if (s == 0.0) q = f->pose.q;
        prm[5 * i + 0].assign(q.coeffs().data(), q.coeffs().data() + 4);
        vector<3> p = f->pose.p + rvec(0.05 * s), v = f->motion.v + rvec(0.05 * s), bg = f->motion.bg + rvec(0.005 * s), ba = f->motion.ba + rvec(0.01 * s);
        prm[5 * i + 1].assign(p.data(), p.data() + 3);
        prm[5 * i + 2].assign(v.data(), v.data() + 3);
        prm[5 * i + 3].assign(bg.data(), bg.data() + 3);
        prm[5 * i + 4].assign(ba.data(), ba.data() + 3);
    }
    for (int k = 0; k < 5 * nff; ++k) pp[k] = prm[k].data();
    const bool have_jac = I(0, 3) != 0;
    uint64_t mask = 0;
    std::vector<std::vector<double>> jac(5 * nff);
    std::vector<double *> jp(5 * nff, nullptr);
    if (have_jac) {
        const int mode = I(0, 3);   // 0 all, 1 random, 2 only some frames, 3 none
        for (int k = 0; k < 5 * nff; ++k) {
            bool on = mode == 0 ? true : (mode == 1 ? I(0, 1) == 1 : (mode == 2 ? (k / 5) % 2 == 0 : false));
            if (on) {
                jac[k].assign((size_t)n * (k % 5 == 0 ? 4 : 3), -7.0);   // garbage that Evaluate must overwrite
                jp[k] = jac[k].data();
                mask |= 1ull << k;
            }
        }
    }
    std::vector<double> res((size_t)n, -9.0);
    fac.Evaluate(pp.data(), res.data(), have_jac ? jp.data() : nullptr);
    Wu(fe, (uint32_t)nff);
    put_factor(fe, fac);
    for (int k = 0; k < 5 * nff; ++k) W(fe, prm[k].data(), 8 * prm[k].size());
    Wu(fe, have_jac ? 1u : 0u);
    Wu64(fe, mask);
    W(fe, res.data(), 8 * res.size());
    for (int k = 0; k < 5 * nff; ++k)
        if (mask & (1ull << k)) W(fe, jac[k].data(), 8 * jac[k].size());
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: rd_m5_oracle <outdir> [seed] [chains] [steps]\n"); return 2; }
    const std::string dir = argv[1];
    rng.seed(argc > 2 ? strtoull(argv[2], nullptr, 10) : 1);
    const int chains = argc > 3 ? atoi(argv[3]) : 30;
    const int steps = argc > 4 ? atoi(argv[4]) : 4;
    FILE *fe = fopen((dir + "/eval.bin").c_str(), "wb");
    if (!fe) return 1;
    for (int c = 0; c < chains; ++c) {
        Map map;
        const int nf = (c % 5 == 0) ? 13 : I(2, 13);   // every 5th chain at the full window size (165 x 165 priors)
        const int ntr = I(0, 5) == 0 ? I(0, 3) : I(8, 40);
        for (int i = 0; i < nf; ++i) {
            const bool kf = I(0, 9) != 0;
            Frame *f = push_frame(map, kf, 0.75, i == 0 ? ntr : I(0, 6));
            (void)f;
        }
        map.marginalization_factor = std::make_unique<CeresMarginalizationFactor>(&map);
        for (int s = 0; s < steps; ++s) {
            for (size_t i = 0; i < map.frame_num(); ++i) {
                perturb(map.get_frame(i), 1.0);
                if (i > 0 && I(0, 1) == 0) rand_preint(map.get_frame(i)->keyframe_preintegration);
            }
            auto *fac = static_cast<CeresMarginalizationFactor *>(map.marginalization_factor.get());
            for (int k = 0; k < 3; ++k) eval_record(fe, *fac);
            if (map.frame_num() < 2) break;
            if (map.frame_num() > 2 && I(0, 3) == 0)   // never done by the real code (index 0 only): direct call of the factor with another victim, the map keeps the frame
                fac->marginalize((size_t)I(1, (int)map.frame_num() - 1));
            map.marginalize_frame(0);
            push_frame(map, I(0, 9) != 0, 0.8, I(0, 8));
        }
        auto *fac = static_cast<CeresMarginalizationFactor *>(map.marginalization_factor.get());
        for (int k = 0; k < 3; ++k) eval_record(fe, *fac);
    }
    fclose(fe);
    return 0;
}
