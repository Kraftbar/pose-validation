// RD-VIO M2 oracle (reference tooling, never distributed): drives the REAL rdvio classes (CeresReprojectionErrorFactor / PriorFactor,
// CeresRotationPriorFactor, solve_essential_5pt, solve_homography_4pt, solve_rotation_2pt, decompose_*, triangulate_point, Track::triangulate)
// with random and edge inputs and writes records in the layouts of rdvio_port/c/rd_factor.h / rd_geom.h, so check_rd_m2.c replays them bitwise.
// Built by tools/check_rdvio_port.py against the reference build. usage: rd_m2_oracle <outdir> [seed] [count]
#include <rdvio/estimation/ceres/reprojection_factor.h>
#include <rdvio/estimation/ceres/rotation_factor.h>
#include <rdvio/geometry/essential.h>
#include <rdvio/geometry/homography.h>
#include <rdvio/geometry/stereo.h>
#include <rdvio/geometry/wahba.h>
#include <rdvio/map/frame.h>
#include <rdvio/map/map.h>
#include <rdvio/map/track.h>
#include <random>
#include <cstdio>
#include <string>
using namespace rdvio;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static void W(FILE *f, const void *p, size_t n) { fwrite(p, 1, n, f); }
static void Wd(FILE *f, double v) { W(f, &v, 8); }
static void Wu(FILE *f, uint32_t v) { W(f, &v, 4); }
static quaternion rquat() {
    int m = I(0, 9);
    if (m == 0) return quaternion::Identity();
    quaternion q(N(), N(), N(), N());
    if (m == 1) q.coeffs() << N() * 1e-3, N() * 1e-3, N() * 1e-3, 1.0;
    return q.normalized();
}
static vector<3> rvec(double s) { return vector<3>(N() * s, N() * s, N() * s); }
static vector<3> rbearing() {
    int m = I(0, 11);
    vector<3> v;
    switch (m) {
    case 0: v << 0, 0, 1; break;
    case 1: v << 1, 0, 0; break;
    case 2: v << 0, 1, 0; break;
    case 3: v << 0.5, 0.5, std::sqrt(0.5); break;                 // |x| == |y| tie in the basis axis choice
    case 4: v << N() * 0.05, N() * 0.05, 1.0; break;               // near optical axis
    case 5: v << std::sqrt(1.0 / 3), std::sqrt(1.0 / 3), std::sqrt(1.0 / 3); break;
    default: v << N() * 0.6, N() * 0.5, 1.0; break;               // pinhole-ish
    }
    return v.normalized();
}
static matrix<2> rsic() {
    matrix<2> m;
    int k = I(0, 3);
    double f = U(100, 600);
    if (k == 0) m << f, 0, 0, f;
    else m << f * U(0.5, 2), U(-50, 50) * (k == 1), U(-50, 50) * (k == 1), f * U(0.5, 2);
    return m;
}

// ---------------- factors ----------------
static void factors(FILE *fe, FILE *fr, int cnt) {
    static const int SZ[5] = {4, 3, 4, 3, 1};
    Map map;
    for (int n = 0; n < cnt; ++n) {
        Frame fref, ftgt;   // fref first => smaller id => first_keypoint
        fref.camera.q_cs = rquat(); fref.camera.p_cs = rvec(I(0, 2) == 0 ? 0.0 : 0.1);
        ftgt.camera = fref.camera;
        if (I(0, 2) == 0) { ftgt.camera.q_cs = rquat(); ftgt.camera.p_cs = rvec(0.1); }
        ftgt.sqrt_inv_cov = rsic(); fref.sqrt_inv_cov = ftgt.sqrt_inv_cov;
        vector<3> zr = rbearing(), zt = (I(0, 3) == 0) ? zr : rbearing();
        if (I(0, 6) == 0) zt = (zr + rvec(0.01)).normalized();
        fref.append_keypoint(zr); ftgt.append_keypoint(zt);
        Track *tr = map.create_track();
        tr->add_keypoint(&fref, 0); tr->add_keypoint(&ftgt, 0);
        quaternion qt = rquat(), qr = rquat();
        vector<3> pt = rvec(2), pr = rvec(2);
        if (I(0, 4) == 0) pr = pt + rvec(0.1);
        double invd = (I(0, 20) == 0) ? U(1e-4, 1e-2) : U(0.02, 5.0);
        uint32_t hasj = I(0, 5) != 0, mask = hasj ? (I(0, 3) == 0 ? (uint32_t)I(0, 31) : 31u) : 0;
        bool prior = I(0, 4) == 0;
        double P[15];  // q_tgt p_tgt q_ref p_ref invd
        const int o[5] = {0, 4, 7, 11, 14};
        for (int i = 0; i < 4; ++i) { P[o[0] + i] = qt.coeffs()(i); P[o[2] + i] = qr.coeffs()(i); }
        for (int i = 0; i < 3; ++i) { P[o[1] + i] = pt(i); P[o[3] + i] = pr(i); }
        P[o[4]] = invd;
        double res[2], jb[5][8], *jj[5];
        if (prior) {
            mask &= 3;
            fref.pose.q = qr; fref.pose.p = pr; tr->landmark.inv_depth = invd;
            for (int k = 0; k < 5; ++k) jj[k] = (k < 2 && (mask >> k & 1)) ? jb[k] : nullptr;
            CeresReprojectionPriorFactor f(&ftgt, tr);
            double *jp[2] = {jj[0], jj[1]};
            f.Evaluate(std::array<const double *, 2>{P + o[0], P + o[1]}.data(), res, hasj ? jp : nullptr);
        } else {
            for (int k = 0; k < 5; ++k) jj[k] = (mask >> k & 1) ? jb[k] : nullptr;
            const double *pp[5] = {P + o[0], P + o[1], P + o[2], P + o[3], P + o[4]};
            CeresReprojectionErrorFactor f(&ftgt, tr);
            f.Evaluate(pp, res, hasj ? jj : nullptr);
        }
        Wu(fe, hasj); Wu(fe, mask); Wu(fe, prior ? 1 : 0);
        W(fe, P, 15 * 8);
        W(fe, zt.data(), 24); W(fe, zr.data(), 24);
        W(fe, fref.camera.q_cs.coeffs().data(), 32); W(fe, fref.camera.p_cs.data(), 24);
        W(fe, ftgt.camera.q_cs.coeffs().data(), 32); W(fe, ftgt.camera.p_cs.data(), 24);
        W(fe, ftgt.sqrt_inv_cov.data(), 32); W(fe, res, 16);
        for (int k = 0; k < 5; ++k) if (hasj && (mask >> k & 1)) W(fe, jb[k], 2 * SZ[k] * 8);
        tr->remove_keypoint(&ftgt, false); tr->remove_keypoint(&fref, false);
        map.erase_track(tr);
    }
    // rotation prior
    for (int n = 0; n < cnt / 2; ++n) {
        Frame fref, ftgt;
        fref.camera.q_cs = rquat(); fref.camera.p_cs = rvec(I(0, 2) == 0 ? 0.0 : 0.1);
        ftgt.camera = fref.camera;
        if (I(0, 2) == 0) { ftgt.camera.q_cs = rquat(); ftgt.camera.p_cs = rvec(0.1); }
        ftgt.sqrt_inv_cov = rsic(); fref.sqrt_inv_cov = ftgt.sqrt_inv_cov;
        vector<3> zr = rbearing(), zt = rbearing();
        fref.append_keypoint(zr); ftgt.append_keypoint(zt);
        Track *tr = map.create_track();
        tr->add_keypoint(&fref, 0); tr->add_keypoint(&ftgt, 0);
        fref.pose.q = rquat(); fref.pose.p = rvec(1);
        quaternion qt = (I(0, 2) == 0) ? rquat() : (fref.pose.q * Eigen::AngleAxisd(0.05, vector<3>(rvec(1).normalized()))).normalized();
        uint32_t hasj = I(0, 4) != 0;
        double res[2], jb[8];
        CeresRotationPriorFactor f(&ftgt, tr);
        const double *pp[1] = {qt.coeffs().data()};
        double *jp[1] = {jb};
        f.Evaluate(pp, res, hasj ? jp : nullptr);
        Wu(fr, hasj);
        W(fr, qt.coeffs().data(), 32); W(fr, fref.pose.q.coeffs().data(), 32);
        W(fr, zt.data(), 24); W(fr, zr.data(), 24);
        W(fr, fref.camera.q_cs.coeffs().data(), 32); W(fr, fref.camera.p_cs.data(), 24);
        W(fr, ftgt.camera.q_cs.coeffs().data(), 32); W(fr, ftgt.camera.p_cs.data(), 24);
        W(fr, ftgt.sqrt_inv_cov.data(), 32); W(fr, res, 16);
        if (hasj) W(fr, jb, 64);
        tr->remove_keypoint(&ftgt, false); tr->remove_keypoint(&fref, false);
        map.erase_track(tr);
    }
}


// ---------------- geometry ----------------
static matrix<3> hatm(const vector<3> &v) { matrix<3> m; m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0; return m; }
static void w_obs(FILE *f, const Frame &fr, size_t idx) {
    W(f, fr.pose.q.coeffs().data(), 32); W(f, fr.pose.p.data(), 24); W(f, fr.camera.q_cs.coeffs().data(), 32); W(f, fr.camera.p_cs.data(), 24);
    W(f, fr.get_keypoint(idx).data(), 24);
}
static void geometry(FILE *fw, FILE *fe5, FILE *fh4, FILE *fde, FILE *fdh, FILE *ft2, FILE *ftn, FILE *ftk, FILE *fta, FILE *fgl, FILE *fsl, int cnt) {
    // Wahba
    for (int n = 0; n < cnt; ++n) {
        std::array<vector<3>, 2> a, b;
        quaternion R = (I(0, 5) == 0) ? quaternion::Identity() : rquat();
        double noise = (I(0, 3) == 0) ? 0.0 : U(1e-6, 0.05);
        for (int i = 0; i < 2; ++i) { a[i] = rbearing(); b[i] = (R * a[i] + rvec(noise)).normalized(); }
        if (I(0, 15) == 0) b[1] = b[0];
        if (I(0, 15) == 0) a[1] = a[0];
        matrix<3> r = solve_rotation_2pt(a, b);
        for (int i = 0; i < 2; ++i) W(fw, a[i].data(), 24);
        for (int i = 0; i < 2; ++i) W(fw, b[i].data(), 24);
        W(fw, r.data(), 72);
    }
    // essential 5pt
    for (int n = 0; n < cnt; ++n) {
        std::array<vector<2>, 5> a, b;
        quaternion R = (I(0, 4) == 0) ? (quaternion::Identity()) : (quaternion(Eigen::AngleAxisd(U(0, 0.5), rvec(1).normalized())));
        vector<3> t = rvec(1).normalized() * U(0.05, 1.0);
        if (I(0, 20) == 0) t = vector<3>(0, 0, 0.3);
        double noise = (I(0, 3) == 0) ? 0.0 : U(1e-6, 2e-3);
        int mode = I(0, 9);
        for (int i = 0; i < 5; ++i) {
            if (mode == 0) { a[i] = vector<2>(N() * 0.5, N() * 0.5); b[i] = vector<2>(N() * 0.5, N() * 0.5); continue; }
            vector<3> X(N() * 1.0, N() * 1.0, U(2.0, 10.0));
            vector<3> Y = R * X + t;
            a[i] = X.hnormalized(); b[i] = Y.hnormalized();
            a[i] += vector<2>(N() * noise, N() * noise); b[i] += vector<2>(N() * noise, N() * noise);
        }
        if (mode == 1) for (int i = 0; i < 5; ++i) { a[i](1) = 0.1 * a[i](0); b[i](1) = 0.1 * b[i](0); }   // collinear
        auto sols = solve_essential_5pt(a, b);
        for (int i = 0; i < 5; ++i) W(fe5, a[i].data(), 16);
        for (int i = 0; i < 5; ++i) W(fe5, b[i].data(), 16);
        Wu(fe5, (uint32_t)sols.size());
        for (auto &E : sols) W(fe5, E.data(), 72);
    }
    // homography 4pt
    for (int n = 0; n < cnt; ++n) {
        std::array<vector<2>, 4> a, b;
        matrix<3> H;
        quaternion R = (I(0, 5) == 0) ? quaternion::Identity() : quaternion(Eigen::AngleAxisd(U(0, 0.5), rvec(1).normalized()));
        vector<3> t = rvec(0.3), nn = vector<3>(0, 0, 1) + rvec(0.1);
        H = R.matrix() + t * nn.transpose();
        double noise = (I(0, 3) == 0) ? 0.0 : U(1e-6, 2e-3);
        int mode = I(0, 9);
        for (int i = 0; i < 4; ++i) {
            a[i] = vector<2>(N() * 0.5, N() * 0.5);
            b[i] = (H * a[i].homogeneous()).hnormalized() + vector<2>(N() * noise, N() * noise);
            if (mode == 0) b[i] = vector<2>(N() * 0.5, N() * 0.5);
            if (mode == 1) b[i] = a[i] + vector<2>(0.1, -0.2);
        }
        matrix<3> h = solve_homography_4pt(a, b);
        for (int i = 0; i < 4; ++i) W(fh4, a[i].data(), 16);
        for (int i = 0; i < 4; ++i) W(fh4, b[i].data(), 16);
        W(fh4, h.data(), 72);
    }
    // decompose essential
    for (int n = 0; n < cnt; ++n) {
        matrix<3> E;
        int mode = I(0, 5);
        if (mode == 0) { E = matrix<3>::Zero(); for (int i = 0; i < 9; ++i) E(i / 3, i % 3) = N(); }
        else { quaternion R = rquat(); vector<3> t = rvec(1).normalized() * U(0.1, 2.0); E = hatm(t) * R.matrix(); }
        matrix<3> R1, R2; vector<3> T;
        decompose_essential(E, R1, R2, T);
        W(fde, E.data(), 72); W(fde, R1.data(), 72); W(fde, R2.data(), 72); W(fde, T.data(), 24);
    }
    // decompose homography
    for (int n = 0; n < cnt; ++n) {
        matrix<3> H;
        int mode = I(0, 5);
        quaternion R = rquat();
        if (mode == 0) { H = R.matrix() * U(0.5, 2.0); }                                          // pure rotation, scaled
        else if (mode == 1) { H = R.matrix() + rvec(1e-4).asDiagonal().toDenseMatrix(); }       // nearly pure rotation
        else if (mode == 2) { for (int i = 0; i < 9; ++i) H(i / 3, i % 3) = N(); }
        else { vector<3> t = rvec(U(0.05, 1.0)), nn = (vector<3>(0, 0, 1) + rvec(0.3)).normalized(); H = (quaternion(Eigen::AngleAxisd(U(0, 0.5), rvec(1).normalized())).matrix() + t * nn.transpose()) * U(0.5, 2.0); }
        matrix<3> R1 = matrix<3>::Zero(), R2 = matrix<3>::Zero(); vector<3> T1 = vector<3>::Zero(), T2 = vector<3>::Zero(), n1 = vector<3>::Zero(), n2 = vector<3>::Zero();
        bool ret = decompose_homography(H, R1, R2, T1, T2, n1, n2);
        W(fdh, H.data(), 72); Wu(fdh, ret ? 1 : 0); W(fdh, R1.data(), 72); W(fdh, R2.data(), 72); W(fdh, T1.data(), 24); W(fdh, T2.data(), 24); W(fdh, n1.data(), 24); W(fdh, n2.data(), 24);
    }
    // triangulate (two views)
    for (int n = 0; n < cnt; ++n) {
        matrix<3, 4> P1, P2;
        quaternion R = rquat(); vector<3> t = rvec(1);
        P1 << matrix<3>::Identity(), vector<3>::Zero();
        P2 << R.matrix(), t;
        vector<3> X(N() * 2, N() * 2, U(2, 20));
        vector<3> a = X.normalized() , b = (R * X + t).normalized();
        if (I(0, 2) > 0) { a = X; a /= a.z(); b = R * X + t; b /= b.z(); a += rvec(1e-3); b += rvec(1e-3); a(2) = 1; b(2) = 1; }
        vector<4> h = triangulate_point(P1, P2, a, b);
        W(ft2, P1.data(), 96); W(ft2, P2.data(), 96); W(ft2, a.data(), 24); W(ft2, b.data(), 24); W(ft2, h.data(), 32);
    }
    // triangulate (n views)
    for (int n = 0; n < cnt; ++n) {
        int k = I(1, 8);
        std::vector<matrix<3, 4>> Ps(k); std::vector<vector<3>> pts(k);
        vector<3> X(N() * 2, N() * 2, U(2, 20));
        for (int i = 0; i < k; ++i) {
            quaternion R = rquat(); vector<3> t = rvec(1); Ps[i] << R.matrix(), t;
            pts[i] = (R * X + t).normalized() + rvec(I(0, 2) ? 1e-3 : 0.0); pts[i].normalize();
        }
        vector<4> h = triangulate_point(Ps, pts);
        Wu(ftn, (uint32_t)k); for (auto &P : Ps) W(ftn, P.data(), 96); for (auto &p : pts) W(ftn, p.data(), 24); W(ftn, h.data(), 32);
    }
    // Track::triangulate and friends
    Map map;
    for (int n = 0; n < cnt; ++n) {
        int k = I(2, 7);
        std::vector<std::unique_ptr<Frame>> fr;
        Track *tr = map.create_track();
        vector<3> X(N() * 3, N() * 3, U(2, 20));
        quaternion cq = rquat(); vector<3> cp = rvec(0.1);
        for (int i = 0; i < k; ++i) {
            fr.emplace_back(new Frame());
            Frame &f = *fr.back();
            f.camera.q_cs = cq; f.camera.p_cs = cp;
            f.pose.q = rquat(); f.pose.p = rvec(1);
            if (I(0, 3) > 0) { f.pose.q = (quaternion(Eigen::AngleAxisd(0.1 * i, vector<3>(0, 1, 0)))).normalized(); f.pose.p = vector<3>(0.3 * i, 0, 0) + rvec(0.01); }
            PoseState cam = f.get_pose(f.camera);
            vector<3> y = cam.q.conjugate() * (X - cam.p);
            if (I(0, 20) == 0) y = -y;                                        // behind the camera => invalid
            vector<3> kp = (y + rvec(I(0, 2) ? 0.01 : 0.0)).normalized();
            f.append_keypoint(kp);
            tr->add_keypoint(&f, 0);
        }
        auto lm = tr->triangulate();
        Wu(ftk, (uint32_t)k); for (int i = 0; i < k; ++i) w_obs(ftk, *fr[i], 0);
        Wu(ftk, lm ? 1 : 0); if (lm) W(ftk, lm->data(), 24);
        // aux
        vector<3> p = lm ? *lm : X;
        if (I(0, 6) == 0) p = rvec(5);
        double invd = U(0.02, 3.0);
        tr->landmark.inv_depth = invd;
        double ang = tr->triangulation_angle(p);
        vector<3> gp = tr->get_landmark_point();
        tr->set_landmark_point(p);
        double sd = tr->landmark.inv_depth;
        Wu(fta, (uint32_t)k); for (int i = 0; i < k; ++i) w_obs(fta, *fr[i], 0);
        W(fta, p.data(), 24); Wd(fta, ang);
        w_obs(fgl, *fr[0], 0); Wd(fgl, invd); W(fgl, gp.data(), 24);
        w_obs(fsl, *fr[0], 0); W(fsl, p.data(), 24); Wd(fsl, sd);
        for (int i = k - 1; i >= 0; --i) tr->remove_keypoint(fr[i].get(), false);
        map.erase_track(tr);
    }
}

int main(int argc, char **argv) {
    std::string dir = argv[1];
    rng.seed(argc > 2 ? atoll(argv[2]) : 1);
    int cnt = argc > 3 ? atoi(argv[3]) : 3000;
    FILE *fe = fopen((dir + "/rpe.bin").c_str(), "wb"), *fr = fopen((dir + "/rot.bin").c_str(), "wb");
    factors(fe, fr, cnt);
    fclose(fe); fclose(fr);
    {
        FILE *a = fopen((dir + "/wahba.bin").c_str(), "wb"), *b = fopen((dir + "/ess5.bin").c_str(), "wb"), *c = fopen((dir + "/hom4.bin").c_str(), "wb"),
             *d = fopen((dir + "/decess.bin").c_str(), "wb"), *e = fopen((dir + "/dechom.bin").c_str(), "wb"), *f = fopen((dir + "/tri2.bin").c_str(), "wb"),
             *g = fopen((dir + "/trin.bin").c_str(), "wb"), *h = fopen((dir + "/trk.bin").c_str(), "wb"), *i = fopen((dir + "/tang.bin").c_str(), "wb"),
             *j = fopen((dir + "/glp.bin").c_str(), "wb"), *k = fopen((dir + "/slp.bin").c_str(), "wb");
        geometry(a, b, c, d, e, f, g, h, i, j, k, cnt);
        fclose(a); fclose(b); fclose(c); fclose(d); fclose(e); fclose(f); fclose(g); fclose(h); fclose(i); fclose(j); fclose(k);
    }
    return 0;
}
