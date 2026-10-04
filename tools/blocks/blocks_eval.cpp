// SPDX-License-Identifier: BSD-3-Clause (harness; links PoseLib BSD-3-Clause and stella_vio C sources BSD-2-Clause, read-only)
// Standalone harness: PoseLib vs stella_vio solvers on real ORB correspondences (pairs.txt from make_pairs.py).
//   blocks_eval pairs.txt out.csv [min_matches=50]
// pair file: PAIR seq i j gap / K fx fy cx cy / GT R21(row-major 9) t21(3) / G g1 g2 gt1 gt2 (unit 'up' vectors in cam1, cam2; measured, ground truth)
//            / N n / n lines: u1 v1 u2 v2 depth1 octave1 octave2     (pixels, undistorted; x2 = R21 x1 + t21)
// Relative pose solvers: ST_init (stella_vio sv_init_try_monocular, seeds=1), ST_init4 (4 seeds), PL_5pt (PoseLib estimate_relative_pose),
//   PL_up3_real / PL_up3_gt / PL_up3_gt3deg (PoseLib relpose_upright_3pt in an own MSAC loop, gravity from calibrated accelerometer / ground truth /
//   ground truth + 3 deg random error), PL_H (PoseLib estimate_homography + stella's homography decompose + cheirality).
// PnP solvers (2D in view 2, 3D from depth of view 1, outlier fractions 0/.3/.5/.7/.85 injected): ST_pnp30, ST_pnp100 (sv_pnp_ransac), PL_p3p
//   (estimate_absolute_pose), PL_up2p_real / PL_up2p_gt (up2p in an own MSAC loop + PoseLib refinement).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <chrono>
#include <Eigen/Dense>
#include "PoseLib/poselib.h"
extern "C" {
#include "sv_types.h"
#include "sv_rng.h"
#include "sv_triangulate.h"
#include "sv_init.h"
#include "sv_pnp.h"
#include "sv_solve_homography.h"
}
using namespace Eigen;
using Clock = std::chrono::steady_clock;
static double now_us() { return std::chrono::duration<double, std::micro>(Clock::now().time_since_epoch()).count(); }

struct Match { double u1, v1, u2, v2, z1; int o1, o2; };
struct Pair { std::string seq; int i, j, gap; double fx, fy, cx, cy; Matrix3d R; Vector3d t; Vector3d g1, g2, gt1, gt2; std::vector<Match> m; };

static bool read_pairs(const char* path, std::vector<Pair>& out) {
    FILE* f = fopen(path, "r"); if (!f) return false;
    char tok[64];
    while (fscanf(f, "%63s", tok) == 1) {
        if (strcmp(tok, "PAIR")) return false;
        Pair p; char seq[64];
        if (fscanf(f, "%63s %d %d %d", seq, &p.i, &p.j, &p.gap) != 4) return false; p.seq = seq;
        fscanf(f, "%*s %lf %lf %lf %lf", &p.fx, &p.fy, &p.cx, &p.cy);
        double a[12]; fscanf(f, "%*s"); for (int k = 0; k < 12; ++k) fscanf(f, "%lf", a + k);
        for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c) p.R(r, c) = a[r * 3 + c];
        p.t = Vector3d(a[9], a[10], a[11]);
        double g[12]; fscanf(f, "%*s"); for (int k = 0; k < 12; ++k) fscanf(f, "%lf", g + k);
        p.g1 = Vector3d(g[0], g[1], g[2]); p.g2 = Vector3d(g[3], g[4], g[5]); p.gt1 = Vector3d(g[6], g[7], g[8]); p.gt2 = Vector3d(g[9], g[10], g[11]);
        int n; fscanf(f, "%*s %d", &n); p.m.resize(n);
        for (int k = 0; k < n; ++k) { Match& m = p.m[k]; fscanf(f, "%lf %lf %lf %lf %lf %d %d", &m.u1, &m.v1, &m.u2, &m.v2, &m.z1, &m.o1, &m.o2); }
        out.push_back(std::move(p));
    }
    fclose(f); return true;
}

static double ang_deg(const Vector3d& a, const Vector3d& b) { return std::acos(std::min(1.0, std::max(-1.0, a.normalized().dot(b.normalized())))) * 180.0 / M_PI; }
static double rot_err_deg(const Matrix3d& R, const Matrix3d& Rg) {
    double c = ((R.transpose() * Rg).trace() - 1.0) * 0.5; return std::acos(std::min(1.0, std::max(-1.0, c))) * 180.0 / M_PI;
}
static Vector3d bearing(const Pair& p, double u, double v) { return Vector3d((u - p.cx) / p.fx, (v - p.cy) / p.fy, 1.0).normalized(); }
static Vector2d norm2(const Pair& p, double u, double v) { return Vector2d((u - p.cx) / p.fx, (v - p.cy) / p.fy); }

// ----- stella-style acceptance test (triangulate the inliers; >= 50 valid points, 1 deg parallax at the 50th point; reprojection error < 4 px in view 2)
struct Check { int nvalid = 0; double par_deg = 0; bool accepted = false; };
static Check check_pose(const Pair& p, const Matrix3d& R, const Vector3d& t, const std::vector<char>& inl) {
    Check c; std::vector<double> cosp;
    for (size_t k = 0; k < p.m.size(); ++k) {
        if (!inl[k]) continue;
        Vector3d x1 = bearing(p, p.m[k].u1, p.m[k].v1), x2 = bearing(p, p.m[k].u2, p.m[k].v2);
        Matrix<double, 3, 2> A; A.col(0) = R * x1; A.col(1) = -x2;
        Vector2d l = (A.transpose() * A).ldlt().solve(-A.transpose() * t);
        if (!(l(0) > 0 && l(1) > 0)) continue;
        Vector3d X1 = l(0) * x1, X2 = R * X1 + t;
        if (X2(2) <= 0) continue;
        double e = std::hypot(p.fx * X2(0) / X2(2) + p.cx - p.m[k].u2, p.fy * X2(1) / X2(2) + p.cy - p.m[k].v2);
        if (e > 4.0) continue;
        ++c.nvalid; Vector3d v2 = R.transpose() * X2; cosp.push_back(X1.normalized().dot(v2.normalized()));
    }
    if (c.nvalid) { std::sort(cosp.begin(), cosp.end()); size_t idx = std::min<size_t>(cosp.size() - 1, 50); c.par_deg = std::acos(std::min(1.0, cosp[idx])) * 180.0 / M_PI; }
    c.accepted = c.nvalid >= 50 && c.par_deg >= 1.0;
    return c;
}

struct Res { bool ran = false, ok = false; Matrix3d R = Matrix3d::Identity(); Vector3d t = Vector3d::Zero(); int ninl = 0; double us = 0; Check chk; };

static double sampson_px(const Matrix3d& E, const Vector3d& x1, const Vector3d& x2, double f) {
    Vector3d Ex1 = E * x1, Etx2 = E.transpose() * x2; double num = x2.dot(Ex1); double den = Ex1(0) * Ex1(0) + Ex1(1) * Ex1(1) + Etx2(0) * Etx2(0) + Etx2(1) * Etx2(1);
    return den > 0 ? std::fabs(num) / std::sqrt(den) * f : 1e9;
}
static Matrix3d skew(const Vector3d& t) { Matrix3d s; s << 0, -t(2), t(1), t(2), 0, -t(0), -t(1), t(0), 0; return s; }

// ---------- own MSAC loop around PoseLib's upright minimal solver (3 points, gravity from the accelerometer)
static Res relpose_upright(const Pair& p, const Vector3d& g1, const Vector3d& g2, double thr_px, int max_it, uint64_t seed) {
    Res r; r.ran = true; double t0 = now_us();
    size_t n = p.m.size(); std::vector<Vector3d> b1(n), b2(n); std::vector<Vector2d> n1(n), n2(n);
    double f = 0.5 * (p.fx + p.fy);
    for (size_t k = 0; k < n; ++k) { b1[k] = bearing(p, p.m[k].u1, p.m[k].v1); b2[k] = bearing(p, p.m[k].u2, p.m[k].v2); n1[k] = norm2(p, p.m[k].u1, p.m[k].v1); n2[k] = norm2(p, p.m[k].u2, p.m[k].v2); }
    std::mt19937_64 rng(seed); double best = 1e30; poselib::CameraPose bp; std::vector<char> binl(n, 0); int it = 0, need = max_it;
    for (; it < need && it < max_it; ++it) {
        int idx[3]; do { for (int s = 0; s < 3; ++s) idx[s] = (int)(rng() % n); } while (idx[0] == idx[1] || idx[0] == idx[2] || idx[1] == idx[2]);
        std::vector<Vector3d> x1 = { b1[idx[0]], b1[idx[1]], b1[idx[2]] }, x2 = { b2[idx[0]], b2[idx[1]], b2[idx[2]] };
        poselib::CameraPoseVector poses; poselib::relpose_upright_3pt(x1, x2, g1, g2, &poses);
        for (auto& ps : poses) {
            Matrix3d E = skew(ps.t) * ps.R(); double sc = 0; int ni = 0;
            for (size_t k = 0; k < n; ++k) { double e = sampson_px(E, b1[k], b2[k], f); if (e < thr_px) { sc += e * e; ++ni; } else sc += thr_px * thr_px; }
            if (sc < best) { best = sc; bp = ps; need = std::min(max_it, std::max(50, (int)(std::log(1 - 0.9999) / std::log(1 - std::pow(std::max(1e-3, ni / (double)n), 3.0)))));
            }
        }
    }
    if (best > 1e29) { r.us = now_us() - t0; return r; }
    // inliers + refinement (general relative pose refinement, PoseLib)
    Matrix3d E = skew(bp.t) * bp.R(); std::vector<Vector2d> i1, i2;
    for (size_t k = 0; k < n; ++k) if (sampson_px(E, b1[k], b2[k], f) < thr_px) { i1.push_back(n1[k]); i2.push_back(n2[k]); binl[k] = 1; }
    if (i1.size() >= 8) { poselib::BundleOptions bo; bo.loss_type = poselib::BundleOptions::LossType::CAUCHY; bo.loss_scale = thr_px / f; bo.max_iterations = 25; poselib::refine_relpose(i1, i2, &bp, bo);
        E = skew(bp.t) * bp.R(); std::fill(binl.begin(), binl.end(), 0); r.ninl = 0;
        for (size_t k = 0; k < n; ++k) if (sampson_px(E, b1[k], b2[k], f) < thr_px) { binl[k] = 1; ++r.ninl; } }
    else r.ninl = (int)i1.size();
    r.us = now_us() - t0; r.R = bp.R(); r.t = bp.t; r.ok = true; r.chk = check_pose(p, r.R, r.t, binl);
    return r;
}

static Res run_poselib_relpose(const Pair& p, double thr_px, size_t max_it = 1000, size_t min_it = 100) {
    Res r; r.ran = true; size_t n = p.m.size();
    std::vector<poselib::Point2D> x1(n), x2(n);
    for (size_t k = 0; k < n; ++k) { x1[k] = Vector2d(p.m[k].u1, p.m[k].v1); x2[k] = Vector2d(p.m[k].u2, p.m[k].v2); }
    poselib::Camera cam("PINHOLE", { p.fx, p.fy, p.cx, p.cy }, 640, 480);
    poselib::RelativePoseOptions opt; opt.ransac.max_iterations = max_it; opt.ransac.min_iterations = min_it; opt.ransac.seed = 1; opt.max_error = thr_px;
    poselib::CameraPose pose; std::vector<char> inl; double t0 = now_us();
    poselib::RansacStats st = poselib::estimate_relative_pose(x1, x2, cam, cam, opt, &pose, &inl);
    r.us = now_us() - t0; r.ninl = (int)st.num_inliers; r.ok = st.num_inliers >= 8; r.R = pose.R(); r.t = pose.t;
    if (r.ok) r.chk = check_pose(p, r.R, r.t, inl);
    r.ran = true; return r;
}

static Res run_poselib_homography(const Pair& p, double thr_px) {
    Res r; r.ran = true; size_t n = p.m.size();
    std::vector<poselib::Point2D> x1(n), x2(n);
    for (size_t k = 0; k < n; ++k) { x1[k] = Vector2d(p.m[k].u1, p.m[k].v1); x2[k] = Vector2d(p.m[k].u2, p.m[k].v2); }
    poselib::HomographyOptions opt; opt.ransac.max_iterations = 1000; opt.ransac.min_iterations = 100; opt.ransac.seed = 1; opt.max_error = thr_px;
    Matrix3d H; std::vector<char> inl; double t0 = now_us();
    poselib::RansacStats st = poselib::estimate_homography(x1, x2, opt, &H, &inl);
    // decompose with stella's decompose (column-major), pick the hypothesis with most valid triangulated inliers
    double cam[9] = { p.fx, 0, 0, 0, p.fy, 0, p.cx, p.cy, 1 }, H21[9];
    for (int rr = 0; rr < 3; ++rr) for (int cc = 0; cc < 3; ++cc) H21[cc * 3 + rr] = H(rr, cc);
    double rots[8][9], trs[8][3], nrm[8][3];
    if (st.num_inliers >= 8 && sv_solve_homography_decompose(H21, cam, cam, rots, trs, nrm)) {
        int bestn = -1;
        for (int h = 0; h < 8; ++h) {
            Matrix3d R; for (int rr = 0; rr < 3; ++rr) for (int cc = 0; cc < 3; ++cc) R(rr, cc) = rots[h][cc * 3 + rr];
            Vector3d t(trs[h][0], trs[h][1], trs[h][2]);
            Check c = check_pose(p, R, t, inl);
            if (c.nvalid > bestn) { bestn = c.nvalid; r.R = R; r.t = t; r.chk = c; r.ok = true; }
        }
    }
    r.us = now_us() - t0; r.ninl = (int)st.num_inliers; return r;
}

static Res run_stella_init(const Pair& p, unsigned seeds, bool refine = false) {
    Res r; r.ran = true; size_t n = p.m.size();
    std::vector<sv_keypoint> k1(n), k2(n); std::vector<double> b1(3 * n), b2(3 * n); std::vector<int> matched(n);
    sv_camera_perspective cam; cam.fx = p.fx; cam.fy = p.fy; cam.cx = p.cx; cam.cy = p.cy; cam.focal_x_baseline = 0; cam.min_x = 0; cam.max_x = 640; cam.min_y = 0; cam.max_y = 480;
    for (size_t k = 0; k < n; ++k) {
        k1[k] = { (float)p.m[k].u1, (float)p.m[k].v1, 1, 0, 0, p.m[k].o1 }; k2[k] = { (float)p.m[k].u2, (float)p.m[k].v2, 1, 0, 0, p.m[k].o2 };
        sv_camera_convert_point_to_bearing(&cam, k1[k].x, k1[k].y, &b1[3 * k]); sv_camera_convert_point_to_bearing(&cam, k2[k].x, k2[k].y, &b2[3 * k]); matched[k] = (int)k;
    }
    double camm[9] = { p.fx, 0, 0, 0, p.fy, 0, p.cx, p.cy, 1 };
    sv_init_params ip; ip.num_ransac_iters = 100; ip.min_num_valid_pts = 50; ip.min_num_triangulated_pts = 50; ip.parallax_deg_thr = 1.0f; ip.reproj_err_thr = 4.0f; ip.num_seeds = seeds; ip.par_frac = 0;
    sv_init_attempt_result ar; memset(&ar, 0, sizeof ar);
    std::vector<int> m2(n); std::vector<unsigned char> ih(n), jf(n), is_tri(n, 0); std::vector<double> tp(3 * n);
    ar.matched_2_in_1 = m2.data(); ar.inlier_h = ih.data(); ar.inlier_f = jf.data(); ar.triangulated_pts = tp.data(); ar.is_triangulated = is_tri.data();
    double t0 = now_us();
    sv_init_try_monocular(k1.data(), (unsigned)n, b1.data(), k2.data(), (unsigned)n, b2.data(), matched.data(), &cam, &cam, camm, camm, &ip, &ar);
    r.us = now_us() - t0;
    r.ok = ar.verdict == SV_INIT_SUCCESS;     // stella's own accept decision
    if (r.ok) {
        for (int rr = 0; rr < 3; ++rr) for (int cc = 0; cc < 3; ++cc) r.R(rr, cc) = ar.rot_ref_to_cur[cc * 3 + rr];
        r.t = Vector3d(ar.trans_ref_to_cur[0], ar.trans_ref_to_cur[1], ar.trans_ref_to_cur[2]);
        const unsigned char* mask = ar.model_chosen == SV_INIT_MODEL_H ? ar.inlier_h : ar.inlier_f; int c = 0; for (size_t k = 0; k < n; ++k) c += mask[k]; r.ninl = c;
        r.chk.accepted = true; r.chk.nvalid = (int)ar.hyps[ar.selected_hyp].num_valid_pts;
        if (refine) {   // PoseLib Sampson refinement of stella's pose on stella's own inliers (no new minimal solver)
            std::vector<Vector2d> i1, i2; for (size_t k = 0; k < n; ++k) if (mask[k]) { i1.push_back(norm2(p, p.m[k].u1, p.m[k].v1)); i2.push_back(norm2(p, p.m[k].u2, p.m[k].v2)); }
            if (i1.size() >= 8) { poselib::CameraPose ps(r.R, r.t.normalized()); poselib::BundleOptions bo; bo.loss_type = poselib::BundleOptions::LossType::CAUCHY; bo.loss_scale = 1.5 / (0.5 * (p.fx + p.fy)); bo.max_iterations = 25;
                poselib::refine_relpose(i1, i2, &ps, bo); r.R = ps.R(); r.t = ps.t; }
            r.us = now_us() - t0;
        }
    }
    return r;
}

// ---------- PnP
struct PnPIn { std::vector<Vector2d> uv; std::vector<Vector3d> X; std::vector<int> oct; };
struct PRes { bool ok = false; Matrix3d R; Vector3d t; int ninl = 0; double us = 0; };

static PRes pnp_stella(const Pair& p, const PnPIn& in, unsigned iters) {
    PRes r; size_t n = in.uv.size(); if (n < 10) return r;
    std::vector<double> b(3 * n), P(3 * n); std::vector<int> oct(n); std::vector<unsigned char> mask(n);
    for (size_t k = 0; k < n; ++k) { Vector3d bb = bearing(p, in.uv[k](0), in.uv[k](1)); for (int c = 0; c < 3; ++c) { b[3 * k + c] = bb(c); P[3 * k + c] = in.X[k](c); } oct[k] = std::min(7, std::max(0, in.oct[k])); }
    float scales[8]; for (int i = 0; i < 8; ++i) scales[i] = (float)std::pow(1.2, i);
    sv_pnp_result res; memset(&res, 0, sizeof res); double t0 = now_us();
    int rc = sv_pnp_ransac(b.data(), P.data(), oct.data(), (unsigned)n, scales, 8, 10, iters, 10, 0, NULL, &res, mask.data(), NULL, NULL);
    r.us = now_us() - t0;
    if (rc == 0 && res.valid) { r.ok = true; r.ninl = (int)res.inliers; for (int rr = 0; rr < 3; ++rr) for (int cc = 0; cc < 3; ++cc) r.R(rr, cc) = res.rotation[cc * 3 + rr]; r.t = Vector3d(res.translation[0], res.translation[1], res.translation[2]); }
    return r;
}
static PRes pnp_poselib(const Pair& p, const PnPIn& in, double thr_px) {
    PRes r; size_t n = in.uv.size(); if (n < 6) return r;
    std::vector<poselib::Point2D> x(n); std::vector<poselib::Point3D> X(n);
    for (size_t k = 0; k < n; ++k) { x[k] = in.uv[k]; X[k] = in.X[k]; }
    poselib::AbsolutePoseOptions opt; opt.ransac.max_iterations = 1000; opt.ransac.min_iterations = 100; opt.ransac.seed = 1; opt.max_error = thr_px;
    poselib::Image img; img.camera = poselib::Camera("PINHOLE", { p.fx, p.fy, p.cx, p.cy }, 640, 480); std::vector<char> inl; double t0 = now_us();
    poselib::RansacStats st = poselib::estimate_absolute_pose(x, X, opt, &img, &inl);
    r.us = now_us() - t0; r.ok = st.num_inliers >= 10; r.ninl = (int)st.num_inliers; r.R = img.pose.R(); r.t = img.pose.t; return r;
}
static PRes pnp_up2p(const Pair& p, const PnPIn& in, const Vector3d& gcam, const Vector3d& gworld, double thr_px, uint64_t seed) {
    PRes r; size_t n = in.uv.size(); if (n < 10) return r; double t0 = now_us();
    std::vector<Vector3d> b(n); for (size_t k = 0; k < n; ++k) b[k] = bearing(p, in.uv[k](0), in.uv[k](1));
    std::mt19937_64 rng(seed); double best = 1e30; poselib::CameraPose bp; int need = 1000, it = 0;
    auto err = [&](const poselib::CameraPose& ps, size_t k) { Vector3d Xc = ps.R() * in.X[k] + ps.t; if (Xc(2) <= 0) return 1e9; return std::hypot(p.fx * Xc(0) / Xc(2) + p.cx - in.uv[k](0), p.fy * Xc(1) / Xc(2) + p.cy - in.uv[k](1)); };
    for (; it < need; ++it) {
        size_t i0 = rng() % n, i1; do { i1 = rng() % n; } while (i1 == i0);
        std::vector<Vector3d> x = { b[i0], b[i1] }, X = { in.X[i0], in.X[i1] }; poselib::CameraPoseVector poses; poselib::up2p(x, X, gcam, gworld, &poses);
        for (auto& ps : poses) { double sc = 0; int ni = 0; for (size_t k = 0; k < n; ++k) { double e = err(ps, k); if (e < thr_px) { sc += e * e; ++ni; } else sc += thr_px * thr_px; }
            if (sc < best) { best = sc; bp = ps; need = std::min(1000, std::max(50, (int)(std::log(1 - 0.9999) / std::log(1 - std::pow(std::max(1e-3, ni / (double)n), 2.0))))); } }
    }
    if (best > 1e29) { r.us = now_us() - t0; return r; }
    std::vector<Vector2d> xi; std::vector<Vector3d> Xi; for (size_t k = 0; k < n; ++k) if (err(bp, k) < thr_px) { xi.push_back(norm2(p, in.uv[k](0), in.uv[k](1))); Xi.push_back(in.X[k]); }
    if (xi.size() >= 6) { poselib::BundleOptions bo; bo.loss_type = poselib::BundleOptions::LossType::CAUCHY; bo.loss_scale = thr_px / (0.5 * (p.fx + p.fy)); bo.max_iterations = 25; poselib::bundle_adjust(xi, Xi, &bp, bo); }
    int c = 0; for (size_t k = 0; k < n; ++k) c += err(bp, k) < thr_px;
    r.us = now_us() - t0; r.ok = c >= 10; r.ninl = c; r.R = bp.R(); r.t = bp.t; return r;
}

static Vector3d perturb(const Vector3d& g, double deg, std::mt19937_64& rng) {
    std::normal_distribution<double> nd(0, 1); Vector3d ax(nd(rng), nd(rng), nd(rng)); ax -= ax.dot(g) * g; ax.normalize();
    return (AngleAxisd(deg * M_PI / 180.0, ax) * g).normalized();
}

int main(int argc, char** argv) {
    if (argc < 3) return 1;
    std::vector<Pair> pairs; if (!read_pairs(argv[1], pairs)) { fprintf(stderr, "bad pairs file\n"); return 2; }
    size_t minm = argc > 3 ? atoi(argv[3]) : 50;
    FILE* fo = fopen(argv[2], "w");
    fprintf(fo, "task,seq,gap,i,j,nmatch,solver,param,ran,ok,accepted,rot_err,dir_err,pos_err,ninl,nvalid,us\n");
    int idx = 0;
    for (auto& p : pairs) {
        ++idx; if (p.m.size() < minm) continue;
        std::mt19937_64 grng(1000 + idx);
        Vector3d tdir = p.t.norm() > 1e-6 ? p.t.normalized() : Vector3d(0, 0, 1);
        Vector3d g1gt3 = perturb(p.gt1, 3.0, grng), g2gt3 = perturb(p.gt2, 3.0, grng);
        struct Job { const char* name; Res r; };
        std::vector<Job> jobs;
        jobs.push_back({ "ST_init", run_stella_init(p, 1) });
        jobs.push_back({ "ST_init4", run_stella_init(p, 4) });
        jobs.push_back({ "ST_init4_refined", run_stella_init(p, 4, true) });
        jobs.push_back({ "PL_5pt", run_poselib_relpose(p, 1.5) });
        jobs.push_back({ "PL_5pt_100it", run_poselib_relpose(p, 1.5, 100, 100) });
        jobs.push_back({ "PL_H", run_poselib_homography(p, 2.0) });
        jobs.push_back({ "PL_up3_real", relpose_upright(p, p.g1, p.g2, 1.5, 1000, 11) });
        jobs.push_back({ "PL_up3_gt", relpose_upright(p, p.gt1, p.gt2, 1.5, 1000, 12) });
        jobs.push_back({ "PL_up3_gt3deg", relpose_upright(p, g1gt3, g2gt3, 1.5, 1000, 13) });
        for (auto& j : jobs) {
            const Res& r = j.r; double re = r.ok ? rot_err_deg(r.R, p.R) : -1, de = r.ok ? std::min(ang_deg(r.t, tdir), 180.0) : -1;
            fprintf(fo, "rel,%s,%d,%d,%d,%zu,%s,0,%d,%d,%d,%.4f,%.4f,0,%d,%d,%.1f\n", p.seq.c_str(), p.gap, p.i, p.j, p.m.size(), j.name, r.ran, r.ok, r.chk.accepted, re, de, r.ninl, r.chk.nvalid, r.us);
        }
        // PnP with injected outliers
        for (double frac : { 0.0, 0.3, 0.5, 0.7, 0.85 }) {
            PnPIn in; std::mt19937_64 rng(5000 + idx * 7 + (int)(frac * 100)); std::uniform_real_distribution<double> ux(0, 640), uy(0, 480);
            for (auto& m : p.m) {
                if (!(m.z1 > 0.2 && m.z1 < 6.0)) continue;
                Vector3d X1 = bearing(p, m.u1, m.v1) / bearing(p, m.u1, m.v1)(2) * m.z1;   // point in view 1 frame (z = depth)
                Vector2d uv(m.u2, m.v2); if ((double)(rng() % 1000) / 1000.0 < frac) uv = Vector2d(ux(rng), uy(rng));
                in.uv.push_back(uv); in.X.push_back(X1); in.oct.push_back(m.o2);
            }
            if (in.uv.size() < 20) continue;
            struct PJob { const char* name; PRes r; };
            std::vector<PJob> pj;
            pj.push_back({ "ST_pnp30", pnp_stella(p, in, 30) });
            pj.push_back({ "ST_pnp100", pnp_stella(p, in, 100) });
            pj.push_back({ "PL_p3p", pnp_poselib(p, in, 2.5) });
            pj.push_back({ "PL_up2p_real", pnp_up2p(p, in, p.g2, p.g1, 2.5, 21) });
            pj.push_back({ "PL_up2p_gt", pnp_up2p(p, in, p.gt2, p.gt1, 2.5, 22) });
            for (auto& j : pj) {
                const PRes& r = j.r; double re = -1, pe = -1;
                if (r.ok) { re = rot_err_deg(r.R, p.R); Vector3d c = -r.R.transpose() * r.t, cg = -p.R.transpose() * p.t; pe = (c - cg).norm(); }
                fprintf(fo, "pnp,%s,%d,%d,%d,%zu,%s,%.2f,1,%d,%d,%.4f,0,%.4f,%d,%zu,%.1f\n", p.seq.c_str(), p.gap, p.i, p.j, p.m.size(), j.name, frac, r.ok, r.ok, re, pe, r.ninl, in.uv.size(), r.us);
            }
        }
    }
    fclose(fo); return 0;
}
