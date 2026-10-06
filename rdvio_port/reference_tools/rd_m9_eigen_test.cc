// RD-VIO M9 Eigen oracle test (reference tooling): the REAL Eigen 3.4.0 (external/vio/deps, -O2 -DNDEBUG -ffp-contract=off
// -fno-fast-math) against the C models of rdvio_port/c: FullPivHouseholderQR<MatrixXd>::solve (rd_qr_fullpiv_solve),
// JacobiSVD<Matrix3d>(FullU|FullV)::solve (rd_svd3_solve), Quaternion::FromTwoVectors (rd_quat_from_two_vectors),
// Matrix4d::inverse (rd_m4_inverse), the sliding-window tracker's predict_RT / F / epipolar distance expressions.
// Tolerance 0 (memcmp). usage: rd_m9_eigen_test [seed] [count]. Last line "m9eigen: <mismatches>/<compared>".
#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
extern "C" {
#include "../c/rd_qr.h"
#include "../c/rd_sys_eigen.h"
#include "../c/rd_lie.h"
#include "../../okvis_port/c/ok_eigen.h"
}
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static long bad = 0, tot = 0, bad_qr = 0, bad_svd = 0, bad_q = 0, bad_inv = 0, bad_rt = 0, bad_f = 0, bad_epi = 0, bad_pnp = 0;
static void cmp(const double *a, const double *b, int n, long &cat, const char *what, long idx) {
    for (int i = 0; i < n; ++i) {
        tot++;
        if (std::memcmp(&a[i], &b[i], 8)) {
            bad++; cat++;
            if (bad <= 10) std::printf("  %s case %ld: element %d: eigen %.17g C %.17g\n", what, idx, i, a[i], b[i]);
        }
    }
}
/* the initializer's A of solve_gravity_scale_velocity: (N-1)*6 x (4+3N) with -0.0 off-diagonals */
static Eigen::MatrixXd init_like(int nf, int refine) {
    const int cols = (refine ? 3 : 4) + 3 * nf;
    Eigen::MatrixXd A((nf - 1) * 6, cols);
    A.setZero();
    Eigen::Matrix<double, 3, 2> Tg = Eigen::Matrix<double, 3, 2>::Random();
    for (int j = 1; j < nf; ++j) {
        const int i = j - 1;
        const double dt = U(0.2, 0.3);
        Eigen::Vector3d dp(N(), N(), N());
        if (!refine) {
            A.block<3, 3>(i * 6, 0) = -0.5 * dt * dt * Eigen::Matrix3d::Identity();
            A.block<3, 1>(i * 6, 3) = dp;
            A.block<3, 3>(i * 6, 4 + i * 3) = -dt * Eigen::Matrix3d::Identity();
            A.block<3, 3>(i * 6 + 3, 0) = -dt * Eigen::Matrix3d::Identity();
            A.block<3, 3>(i * 6 + 3, 4 + i * 3) = -Eigen::Matrix3d::Identity();
            A.block<3, 3>(i * 6 + 3, 4 + j * 3) = Eigen::Matrix3d::Identity();
        } else {
            A.block<3, 2>(i * 6, 0) = -0.5 * dt * dt * Tg;
            A.block<3, 1>(i * 6, 2) = dp;
            A.block<3, 3>(i * 6, 3 + i * 3) = -dt * Eigen::Matrix3d::Identity();
            A.block<3, 2>(i * 6 + 3, 0) = -dt * Tg;
            A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -Eigen::Matrix3d::Identity();
            A.block<3, 3>(i * 6 + 3, 3 + j * 3) = Eigen::Matrix3d::Identity();
        }
    }
    return A;
}
int main(int argc, char **argv) {
    const unsigned long long seed = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 1;
    const long count = argc > 2 ? std::atol(argv[2]) : 20000;
    rng.seed(seed);
    long nqr = 0, nsvd = 0, nq = 0;
    for (long c = 0; c < count; ++c) {
        /* ---- FullPivHouseholderQR solve ---- */
        {
            Eigen::MatrixXd A;
            const int kind = I(0, 5);
            if (kind <= 1) A = init_like(8, kind);
            else if (kind == 2) A = init_like(I(2, 10), I(0, 1));
            else { A.resize(I(1, 50), I(1, 50)); for (int i = 0; i < A.size(); ++i) A(i) = N() * std::pow(10.0, I(-3, 3)); }
            if (I(0, 7) == 0 && A.cols() > 1) A.col(I(0, (int)A.cols() - 1)) = A.col(0) * U(-2, 2);   /* rank deficiency */
            if (I(0, 9) == 0) for (int i = 0; i < A.size(); ++i) if (I(0, 3) == 0) A(i) = 0.0;
            Eigen::VectorXd b(A.rows());
            for (int i = 0; i < b.size(); ++i) b(i) = N();
            Eigen::VectorXd xe = A.fullPivHouseholderQr().solve(b);
            Eigen::VectorXd xc(A.cols());
            rd_qr_fullpiv_solve(A.data(), (int)A.rows(), (int)A.cols(), b.data(), xc.data());
            cmp(xe.data(), xc.data(), (int)A.cols(), bad_qr, "fullpiv", c);
            nqr++;
        }
        /* ---- JacobiSVD<Matrix3d> solve (the gyro-bias normal equations: sums of J^T J) ---- */
        {
            Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();
            const int k = I(0, 9);
            if (k == 0) { A.setZero(); b << N(), N(), N(); }
            else if (k == 1) { Eigen::Vector3d v(N(), N(), N()); A = v * v.transpose(); b << N(), N(), N(); }
            else for (int j = 0; j < 7; ++j) { Eigen::Matrix3d J = Eigen::Matrix3d::Random() * U(0.01, 1); A += J.transpose() * J; b += J.transpose() * Eigen::Vector3d(N(), N(), N()); }
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d xe = svd.solve(b), xc;
            rd_svd3_solve(A.data(), b.data(), xc.data());
            cmp(xe.data(), xc.data(), 3, bad_svd, "svd3", c);
            nsvd++;
        }
        /* ---- Matrix4d::inverse (predict_RT: rigid transforms, plus general matrices) ---- */
        {
            Eigen::Matrix4d M;
            if (I(0, 1)) {
                M.setIdentity();
                Eigen::Quaterniond q(N(), N(), N(), N()); q.normalize();
                M.block<3, 3>(0, 0) = q.toRotationMatrix();
                M.block<3, 1>(0, 3) = Eigen::Vector3d(N(), N(), N());
            } else for (int i = 0; i < 16; ++i) M(i) = N() * std::pow(10.0, I(-2, 2));
            Eigen::Matrix4d ie = M.inverse(), ic;
            rd_m4_inverse(M.data(), ic.data());
            cmp(ie.data(), ic.data(), 16, bad_inv, "inverse4", c);
        }
        /* ---- predict_RT (sliding_window_tracker.cpp) ---- */
        {
            Eigen::Quaterniond q[4];
            Eigen::Vector3d p[4];
            for (int k = 0; k < 4; ++k) { q[k] = Eigen::Quaterniond(N(), N(), N(), N()); q[k].normalize(); p[k] = Eigen::Vector3d(N(), N(), N()); }
            Eigen::Matrix4d Pwc = Eigen::Matrix4d::Identity(), PwI = Eigen::Matrix4d::Identity(), Pwi = Eigen::Matrix4d::Identity(), Pwj = Eigen::Matrix4d::Identity();
            Pwc.block<3, 3>(0, 0) = q[0].toRotationMatrix(); Pwc.block<3, 1>(0, 3) = p[0];
            PwI.block<3, 3>(0, 0) = q[1].toRotationMatrix(); PwI.block<3, 1>(0, 3) = p[1];
            Pwi.block<3, 3>(0, 0) = q[2].toRotationMatrix(); Pwi.block<3, 1>(0, 3) = p[2];
            Pwj.block<3, 3>(0, 0) = q[3].toRotationMatrix(); Pwj.block<3, 1>(0, 3) = p[3];
            Eigen::Matrix4d Pji = Pwj.inverse() * Pwi;
            Eigen::Matrix4d P = (Pwc.inverse() * PwI * Pji * PwI.inverse() * Pwc);
            double M[4][16], inv[16], a[16], b[16];
            Eigen::Matrix4d *src[4] = {&Pwc, &PwI, &Pwi, &Pwj};
            for (int k = 0; k < 4; ++k) std::memcpy(M[k], src[k]->data(), sizeof M[k]);
            rd_m4_inverse(M[3], inv); rd_m4_mul(inv, M[2], b);                 /* Pji */
            rd_m4_inverse(M[0], inv); rd_m4_mul(inv, M[1], a);                 /* Pwc^-1 PwI */
            rd_m4_mul(a, b, a);
            rd_m4_inverse(M[1], inv); rd_m4_mul(a, inv, a);
            rd_m4_mul(a, M[0], a);
            cmp(P.data(), a, 16, bad_rt, "predict_RT", c);
        }
        /* ---- F = K^T^-1 E K^-1 (E = [t]x R) and the epipolar distance ---- */
        {
            Eigen::Matrix3d K = Eigen::Matrix3d::Identity(), K2 = Eigen::Matrix3d::Identity(), R, tx = Eigen::Matrix3d::Zero();
            if (I(0, 2)) { K(0, 0) = U(300, 600); K(1, 1) = U(300, 600); K(0, 2) = U(200, 400); K(1, 2) = U(200, 300); K2 = K; }
            else { for (int i = 0; i < 9; ++i) { K(i) = N(); K2(i) = N(); } }
            Eigen::Quaterniond q(N(), N(), N(), N()); q.normalize();
            R = q.toRotationMatrix();
            Eigen::Vector3d t(N(), N(), N());
            tx(0, 1) = -t(2); tx(0, 2) = t(1); tx(1, 0) = t(2); tx(1, 2) = -t(0); tx(2, 0) = -t(1); tx(2, 1) = t(0);
            Eigen::Matrix3d E = tx * R;
            Eigen::Matrix3d F = K2.transpose().inverse() * E * K.inverse();
            double Ec[9], Kti[9], Ki[9], Fc[9];
            ok_m3_mul(tx.data(), R.data(), Ec);
            rd_inverse3_t(K2.data(), Kti); rd_inverse3(K.data(), Ki);
            rd_m3_mul_tinv(Kti, Ec, Fc); ok_m3_mul(Fc, Ki, Fc);
            cmp(F.data(), Fc, 9, bad_f, "F", c);
            for (int k = 0; k < 4; ++k) {
                Eigen::Vector2d p1(U(0, 752), U(0, 480)), p2(U(0, 752), U(0, 480));
                Eigen::Vector3d l = F * p1.homogeneous();
                double de = std::abs(p2.homogeneous().transpose() * l) / l.segment<2>(0).norm();
                Eigen::Matrix3d Ft = F.transpose();
                Eigen::Vector3d l2 = Ft * p2.homogeneous();
                double de2 = std::abs(p1.homogeneous().transpose() * l2) / l2.segment<2>(0).norm();
                double dc = rd_epipolar_dist(F.data(), p1.data(), p2.data()), dc2 = rd_epipolar_dist(Ft.data(), p2.data(), p1.data());
                cmp(&de, &dc, 1, bad_epi, "epipolar", c); cmp(&de2, &dc2, 1, bad_epi, "epipolar_t", c);
            }
        }
        /* ---- pnp_reproject_error (pnp.h) ---- */
        {
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            Eigen::Quaterniond q(N(), N(), N(), N()); q.normalize();
            T.block<3, 3>(0, 0) = q.toRotationMatrix();
            T.block<3, 1>(0, 3) = Eigen::Vector3d(N(), N(), N());
            if (I(0, 3) == 0) for (int i = 0; i < 12; ++i) T(i % 3, i / 3) = (double)(float)N();   /* float-converted like solve_pnp_6pt */
            for (int k = 0; k < 4; ++k) {
                Eigen::Vector3d P(N() * 3, N() * 3, U(0.5, 20));
                Eigen::Vector2d p(U(-1, 1), U(-1, 1));
                double e = (p - (T.block<3, 3>(0, 0) * P + T.block<3, 1>(0, 3)).hnormalized()).squaredNorm();
                double cc = rd_pnp_reproject_error(T.data(), P.data(), p.data());
                cmp(&e, &cc, 1, bad_pnp, "pnp_reproject_error", c);
            }
        }
        /* ---- FromTwoVectors ---- */
        {
            Eigen::Vector3d a(N(), N(), N() + (I(0, 1) ? 0 : 9.8)), g(0, 0, -9.80665);
            if (I(0, 1)) g = Eigen::Vector3d(N(), N(), N());
            Eigen::Quaterniond qe = Eigen::Quaterniond::FromTwoVectors(a, g);
            ok_quat qc;
            if (rd_quat_from_two_vectors(a.data(), g.data(), &qc)) {
                double v[4] = {qc.x, qc.y, qc.z, qc.w};
                cmp(qe.coeffs().data(), v, 4, bad_q, "from_two_vectors", c);
                nq++;
            }
        }
    }
    std::printf("  fullpiv QR solve: %ld cases, %ld mismatching values\n", nqr, bad_qr);
    std::printf("  svd3 solve: %ld cases, %ld mismatching values\n", nsvd, bad_svd);
    std::printf("  FromTwoVectors: %ld cases, %ld mismatching values\n", nq, bad_q);
    std::printf("  Matrix4d::inverse: %ld cases, %ld mismatching values\n", count, bad_inv);
    std::printf("  predict_RT: %ld cases, %ld mismatching values; F: %ld mismatching; epipolar distance: %ld mismatching; pnp_reproject_error: %ld mismatching\n", count, bad_rt, bad_f, bad_epi, bad_pnp);
    std::printf("m9eigen: %ld/%ld\n", bad, tot);
    return bad == 0 && tot > 0 ? 0 : 1;
}
