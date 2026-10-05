// Random RD-VIO windows built with the REAL rdvio classes (Map, Frame, Track, PreIntegrator): shared by rd_m5_oracle.cc and rd_m4_oracle.cc
// (reference tooling, never distributed). Requires the includes of those programs and `rng`, `U`, `N`, `I` defined here.
#pragma once
#include <rdvio/estimation/ceres/marginalization_factor.h>
#include <rdvio/estimation/ceres/preintegration_factor.h>
#include <rdvio/estimation/ceres/reprojection_factor.h>
#include <rdvio/geometry/lie_algebra.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/map.h>
#include <rdvio/map/track.h>
#include <memory>
#include <random>
#include <vector>
namespace rdw {
using namespace rdvio;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static vector<3> rvec(double s) { return vector<3>(N() * s, N() * s, N() * s); }
static quaternion rquat(double spread) {
    quaternion q(N(), N(), N(), N());
    if (spread < 1e8) q.coeffs() << N() * spread, N() * spread, N() * spread, 1.0;
    return q.normalized();
}
static vector<3> rbearing() {
    vector<3> v(N() * 0.4, N() * 0.3, 1.0);
    return v.normalized();
}
static matrix<15> rspd(double scale) {
    matrix<15> G;
    for (int i = 0; i < 15; ++i)
        for (int j = 0; j < 15; ++j) G(i, j) = N();
    matrix<15> M = G * G.transpose() * scale;
    M += matrix<15>::Identity() * scale * 0.05;
    return M;
}
static void rand_preint(PreIntegrator &p) {
    p.delta.t = U(0.02, 0.3);
    p.delta.q = rquat(0.05);
    p.delta.p = rvec(0.05);
    p.delta.v = rvec(0.2);
    p.delta.cov = rspd(U(1e-6, 1e-3));
    for (matrix<3> *m : {&p.jacobian.dq_dbg, &p.jacobian.dp_dbg, &p.jacobian.dp_dba, &p.jacobian.dv_dbg, &p.jacobian.dv_dba})
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) (*m)(i, j) = N() * 0.01;
    p.compute_sqrt_inv_cov();
}
static void rand_state(Frame *f) {
    f->pose.q = rquat(1e9);
    f->pose.p = rvec(1.0);
    f->motion.v = rvec(0.5);
    f->motion.bg = rvec(0.02);
    f->motion.ba = rvec(0.05);
}
static void perturb(Frame *f, double s) {
    f->pose.q = (f->pose.q * rquat(0.02 * s)).normalized();
    f->pose.p += rvec(0.02 * s);
    f->motion.v += rvec(0.02 * s);
    f->motion.bg += rvec(0.002 * s);
    f->motion.ba += rvec(0.005 * s);
}

static void W(FILE *f, const void *p, size_t n) { fwrite(p, 1, n, f); }
static void Wu(FILE *f, uint32_t v) { W(f, &v, 4); }
static void Wu64(FILE *f, uint64_t v) { W(f, &v, 8); }

static std::unique_ptr<Frame> new_frame(bool keyframe) {
    auto f = std::make_unique<Frame>();
    rand_state(f.get());
    f->camera.q_cs = rquat(0.1);
    f->camera.p_cs = rvec(0.05);
    f->imu.q_cs = rquat(0.1);
    f->imu.p_cs = rvec(0.05);
    f->sqrt_inv_cov << U(0.5, 3.0) * 100, N() * 3, N() * 3, U(0.5, 3.0) * 100;
    f->sqrt_inv_cov(1, 0) = f->sqrt_inv_cov(0, 1);   // keep it symmetric-ish, as noise/focal is
    if (keyframe) f->tag(FT_KEYFRAME) = true;
    rand_preint(f->keyframe_preintegration);
    return f;
}

// new keypoint of `track` in `frame`
static void observe(Frame *frame, Track *track) {
    frame->append_keypoint(rbearing());
    track->add_keypoint(frame, frame->keypoint_num() - 1);
}
static Track *new_track(Map &map, Frame *frame, bool valid) {
    Track *t = map.create_track();
    t->tag(TT_VALID) = valid;
    t->tag(TT_TRIANGULATED) = true;
    t->landmark.inv_depth = U(0.05, 2.0);
    observe(frame, t);
    return t;
}
// attach a new frame at the end of the map: continue tracks with probability p, start `nnew` new tracks
static Frame *push_frame(Map &map, bool keyframe, double p_continue, int nnew) {
    std::unique_ptr<Frame> f = new_frame(keyframe);
    Frame *fp = f.get();
    map.attach_frame(std::move(f));
    std::vector<Track *> live;
    for (size_t k = 0; k < map.track_num(); ++k) live.push_back(map.get_track(k));
    for (Track *t : live)
        if (!t->has_keypoint(fp) && t->keypoint_num() > 0 && U(0, 1) < p_continue) observe(fp, t);
    for (int k = 0; k < nnew; ++k) new_track(map, fp, I(0, 9) != 0);
    return fp;
}


// factor over the first nf-1 frames of a random window of nf frames, after `chain` marginalisations (the map is left with nf frames)
static inline void make_chain(Map &map, int nf, int ntr, int chain) {
    for (int i = 0; i < nf; ++i) push_frame(map, I(0, 9) != 0, 0.75, i == 0 ? ntr : I(0, 6));
    map.marginalization_factor = std::make_unique<CeresMarginalizationFactor>(&map);
    for (int s = 0; s < chain; ++s) {
        for (size_t i = 0; i < map.frame_num(); ++i) {
            perturb(map.get_frame(i), 1.0);
            if (i > 0 && I(0, 1) == 0) rand_preint(map.get_frame(i)->keyframe_preintegration);
        }
        map.marginalize_frame(0);
        push_frame(map, I(0, 9) != 0, 0.8, I(0, 8));
    }
}
} // namespace rdw
