// RD-VIO M1 oracle (reference tooling, never distributed): drives the REAL rdvio classes (PreIntegrator, CeresPreIntegrationErrorFactor /
// PriorFactor, QuaternionParameterization) with random inputs and writes records in the layouts of rdvio_port/c/rd_imu.h, plus
// incr.bin (one PreIntegrator::increment) and sqrt.bin (compute_sqrt_inv_cov), so check_rd_imu.c can replay them bit for bit.
// Built by tools/check_rdvio_port.py against the reference build (Eigen 3.4.0, Ceres 2.2.0, -O2 -ffp-contract=off -fno-fast-math).
// usage: rd_imu_oracle <outdir> [seed] [count]
#include <rdvio/estimation/ceres/preintegration_factor.h>
#include <rdvio/estimation/ceres/quaternion_parameterization.h>
#include <rdvio/estimation/preintegrator.h>
#include <rdvio/map/frame.h>
#include <random>
#include <cstdio>
#include <string>
using namespace rdvio;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static quaternion rquat() {
    int m = I(0, 9);
    if (m == 0) return quaternion::Identity();
    if (m == 1) { quaternion q(0, 0, 0, 1); q.coeffs() << 0.0, 0.0, std::sin(1e-9), 1.0; return q.normalized(); }
    quaternion q(N(), N(), N(), N());
    if (m == 2) { q.coeffs() << N() * 1e-3, N() * 1e-3, N() * 1e-3, 1.0; }
    return q.normalized();
}
static vector<3> rvec(double s) { return vector<3>(N() * s, N() * s, N() * s); }
static void W(FILE* f, const void* p, size_t n) { fwrite(p, 1, n, f); }
static void Wd(FILE* f, double v) { W(f, &v, 8); }
static void Wu(FILE* f, uint32_t v) { W(f, &v, 4); }
static void put_state(FILE* f, const PreIntegrator& p) {
    Wd(f, p.delta.t); W(f, p.delta.q.coeffs().data(), 32); W(f, p.delta.p.data(), 24); W(f, p.delta.v.data(), 24);
    W(f, p.delta.cov.data(), 1800); W(f, p.delta.sqrt_inv_cov.data(), 1800);
    W(f, p.jacobian.dq_dbg.data(), 72); W(f, p.jacobian.dp_dbg.data(), 72); W(f, p.jacobian.dp_dba.data(), 72);
    W(f, p.jacobian.dv_dbg.data(), 72); W(f, p.jacobian.dv_dba.data(), 72);
}
static matrix<15> rspd(double scale) {
    matrix<15> G; for (int i = 0; i < 15; ++i) for (int j = 0; j < 15; ++j) G(i, j) = N();
    matrix<15> M = G * G.transpose() * scale; M += matrix<15>::Identity() * scale * 0.05; return M;
}
static void rand_noise(PreIntegrator& p) {
    auto d = [](double s) { matrix<3> m = matrix<3>::Zero(); for (int i = 0; i < 3; ++i) m(i, i) = s * U(0.5, 2.0); return m; };
    p.cov_w = d(U(1e-9, 1e-5)); p.cov_a = d(U(1e-6, 1e-3)); p.cov_bg = d(U(1e-12, 1e-8)); p.cov_ba = d(U(1e-8, 1e-5));
}
static void rand_state(PreIntegrator& p) {
    p.delta.t = U(0, 0.3); p.delta.q = rquat(); p.delta.p = rvec(0.05); p.delta.v = rvec(0.2);
    p.delta.cov = rspd(1e-4); p.delta.sqrt_inv_cov = matrix<15>::Zero();
    for (matrix<3>* m : {&p.jacobian.dq_dbg, &p.jacobian.dp_dbg, &p.jacobian.dp_dba, &p.jacobian.dv_dbg, &p.jacobian.dv_dba})
        for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) (*m)(i, j) = N() * 0.01;
}
int main(int argc, char **argv) {
    std::string dir = argv[1];
    rng.seed(argc > 2 ? atoll(argv[2]) : 1);
    int cnt = argc > 3 ? atoi(argv[3]) : 3000;
    FILE *fi = fopen((dir + "/incr.bin").c_str(), "wb"), *fs = fopen((dir + "/sqrt.bin").c_str(), "wb"), *fp = fopen((dir + "/plus.bin").c_str(), "wb"),
         *fe = fopen((dir + "/pie.bin").c_str(), "wb"), *fg = fopen((dir + "/integ.bin").c_str(), "wb");
    // ---- increment ----
    for (int n = 0; n < cnt; ++n) {
        PreIntegrator p; rand_noise(p); rand_state(p);
        double dt = (I(0, 20) == 0) ? U(0, 1e-7) : U(0.001, 0.02);
        if (I(0, 50) == 0) dt = 0.0;
        ImuData d; d.t = 0; d.w = rvec(I(0, 5) == 0 ? 1e-7 : 1.0); d.a = rvec(3.0); d.a.z() += 9.8;
        if (I(0, 30) == 0) d.w.setZero();
        vector<3> bg = rvec(0.01), ba = rvec(0.05);
        uint32_t flags = I(0, 3);
        PreIntegrator in = p;
        p.increment(dt, d, bg, ba, flags & 1, (flags >> 1) & 1);
        Wd(fi, dt); Wd(fi, d.t); W(fi, d.w.data(), 24); W(fi, d.a.data(), 24); W(fi, bg.data(), 24); W(fi, ba.data(), 24); Wu(fi, flags);
        W(fi, in.cov_w.data(), 72); W(fi, in.cov_a.data(), 72); W(fi, in.cov_bg.data(), 72); W(fi, in.cov_ba.data(), 72);
        put_state(fi, in); put_state(fi, p);
    }
    // ---- sqrt_inv_cov ----
    for (int n = 0; n < cnt / 3; ++n) {
        PreIntegrator p; p.reset(); p.delta.cov = rspd(std::pow(10.0, U(-8, -1)));
        W(fs, p.delta.cov.data(), 1800); p.compute_sqrt_inv_cov(); W(fs, p.delta.sqrt_inv_cov.data(), 1800);
    }
    // ---- plus ----
    for (int n = 0; n < cnt; ++n) {
        quaternion q = rquat(); vector<3> dq = rvec(I(0, 3) == 0 ? 1e-6 : 0.01); if (I(0, 40) == 0) dq.setZero();
        double out[4]; QuaternionParameterization().Plus(q.coeffs().data(), dq.data(), out);
        W(fp, q.coeffs().data(), 32); W(fp, dq.data(), 24); W(fp, out, 32);
    }
    // ---- integrate ----
    for (int n = 0; n < cnt / 6; ++n) {
        PreIntegrator p; rand_noise(p);
        int m = I(0, 12); double t = 100.0 + U(0, 10);
        for (int k = 0; k < m; ++k) { ImuData d; d.t = t; t += U(0.002, 0.012); d.w = rvec(0.5); d.a = rvec(2.0); d.a.z() += 9.8; p.data.push_back(d); }
        double tend = t + U(0, 0.01);
        vector<3> bg = rvec(0.01), ba = rvec(0.05);
        uint32_t flags = I(0, 3); if (I(0, 2) > 0) flags = 3;
        rand_state(p);  // stale state must not matter (reset)
        Wu(fg, (uint32_t)p.data.size()); Wu(fg, flags); Wd(fg, tend); W(fg, bg.data(), 24); W(fg, ba.data(), 24);
        W(fg, p.cov_w.data(), 72); W(fg, p.cov_a.data(), 72); W(fg, p.cov_bg.data(), 72); W(fg, p.cov_ba.data(), 72);
        for (auto &d : p.data) { Wd(fg, d.t); W(fg, d.w.data(), 24); W(fg, d.a.data(), 24); }
        bool ret = p.integrate(tend, bg, ba, flags & 1, (flags >> 1) & 1);
        Wu(fg, ret ? 1 : 0);
        if (ret) put_state(fg, p);
    }
    // ---- pre-integration error / prior factor ----
    for (int n = 0; n < cnt / 2; ++n) {
        Frame fi_, fj_;
        fi_.imu.q_cs = rquat(); fi_.imu.p_cs = rvec(I(0, 2) == 0 ? 0.0 : 0.1);
        fj_.imu.q_cs = fi_.imu.q_cs; fj_.imu.p_cs = fi_.imu.p_cs;
        if (I(0, 3) == 0) { fj_.imu.q_cs = rquat(); fj_.imu.p_cs = rvec(0.1); }
        fi_.motion.bg = rvec(0.01); fi_.motion.ba = rvec(0.05);
        PreIntegrator pre; rand_noise(pre); rand_state(pre);
        pre.delta.t = U(0.005, 0.4);
        pre.delta.sqrt_inv_cov = Eigen::LLT<matrix<15, 15>>(rspd(1e-4).inverse()).matrixL().transpose();
        static const int SZ[10] = {4, 3, 3, 3, 3, 4, 3, 3, 3, 3};
        quaternion qi = rquat(), qj = (I(0, 3) == 0) ? qi : (qi * expmap(rvec(0.05))).normalized();
        if (I(0, 15) == 0) qj = rquat();
        const double *pp[10];
        // blocks: q_i p_i v_i bg_i ba_i q_j p_j v_j bg_j ba_j at offsets 0,4,7,10,13,16,20,23,26,29 (32 doubles)
        double P[32];
        const int o[10] = {0, 4, 7, 10, 13, 16, 20, 23, 26, 29};
        for (int i = 0; i < 4; ++i) { P[o[0] + i] = qi.coeffs()(i); P[o[5] + i] = qj.coeffs()(i); }
        for (int i = 0; i < 3; ++i) {
            P[o[1] + i] = N() * 2; P[o[2] + i] = N() * 0.5; P[o[3] + i] = fi_.motion.bg(i) + N() * 1e-3; P[o[4] + i] = fi_.motion.ba(i) + N() * 1e-2;
            P[o[6] + i] = P[o[1] + i] + N() * 0.2; P[o[7] + i] = P[o[2] + i] + N() * 0.2; P[o[8] + i] = P[o[3] + i] + N() * 1e-3; P[o[9] + i] = P[o[4] + i] + N() * 1e-3;
        }
        for (int k = 0; k < 10; ++k) pp[k] = P + o[k];
        double res[15]; double jbuf[10][60]; double *jj[10];
        uint32_t hasj = I(0, 5) != 0, mask = hasj ? (I(0, 3) == 0 ? (uint32_t)I(0, 1023) : 1023u) : 0;
        bool prior = I(0, 4) == 0;
        for (int k = 0; k < 10; ++k) jj[k] = (mask >> k & 1) ? jbuf[k] : nullptr;
        if (prior) mask &= 0x3e0;  // prior factor only exposes blocks 5..9
        if (prior) {
            // the prior factor reads frame_i's own pose/motion as the i-side parameters
            fi_.pose.q = qi; fi_.pose.p = Eigen::Map<const vector<3>>(P + o[1]); fi_.motion.v = Eigen::Map<const vector<3>>(P + o[2]);
            fi_.motion.bg = Eigen::Map<const vector<3>>(P + o[3]); fi_.motion.ba = Eigen::Map<const vector<3>>(P + o[4]);
            for (int k = 0; k < 5; ++k) jj[k] = nullptr;
            CeresPreIntegrationPriorFactor f(&fi_, &fj_, pre);
            const double *pr[5] = {P + o[5], P + o[6], P + o[7], P + o[8], P + o[9]};
            double *pj5[5] = {jj[5], jj[6], jj[7], jj[8], jj[9]};
            f.Evaluate(pr, res, hasj ? pj5 : nullptr);
        } else {
            CeresPreIntegrationErrorFactor f(&fi_, &fj_, pre);
            f.Evaluate(pp, res, hasj ? jj : nullptr);
        }
        // record in dump layout (prior: the dump shows the 10 blocks the inner factor saw; here fi_ holds them)
        Wu(fe, hasj); Wu(fe, mask); Wu(fe, prior ? 1 : 0);
        for (int k = 0; k < 10; ++k) W(fe, P + o[k], SZ[k] * 8);
        W(fe, fi_.imu.q_cs.coeffs().data(), 32); W(fe, fi_.imu.p_cs.data(), 24); W(fe, fj_.imu.q_cs.coeffs().data(), 32); W(fe, fj_.imu.p_cs.data(), 24);
        W(fe, fi_.motion.bg.data(), 24); W(fe, fi_.motion.ba.data(), 24);
        Wd(fe, pre.delta.t); W(fe, pre.delta.q.coeffs().data(), 32); W(fe, pre.delta.p.data(), 24); W(fe, pre.delta.v.data(), 24);
        W(fe, pre.jacobian.dq_dbg.data(), 72); W(fe, pre.jacobian.dp_dbg.data(), 72); W(fe, pre.jacobian.dp_dba.data(), 72);
        W(fe, pre.jacobian.dv_dbg.data(), 72); W(fe, pre.jacobian.dv_dba.data(), 72);
        W(fe, pre.delta.sqrt_inv_cov.data(), 1800); W(fe, res, 120);
        for (int k = 0; k < 10; ++k) if (hasj && (mask >> k & 1)) W(fe, jbuf[k], 15 * SZ[k] * 8);
    }
    fclose(fi); fclose(fs); fclose(fp); fclose(fe); fclose(fg);
    return 0;
}
