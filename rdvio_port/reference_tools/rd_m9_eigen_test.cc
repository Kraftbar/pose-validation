// RD-VIO M9 Eigen oracle test (reference tooling): the REAL Eigen 3.4.0 (external/vio/deps, -O2 -DNDEBUG -ffp-contract=off
// -fno-fast-math) against the C models of rdvio_port/c: FullPivHouseholderQR<MatrixXd>::solve (rd_qr_fullpiv_solve),
// JacobiSVD<Matrix3d>(FullU|FullV)::solve (rd_svd3_solve), Quaternion::FromTwoVectors (rd_quat_from_two_vectors).
// Tolerance 0 (memcmp). usage: rd_m9_eigen_test [seed] [count]. Last line "m9eigen: <mismatches>/<compared>".
#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
extern "C" {
#include "../c/rd_qr.h"
#include "../c/rd_sys_eigen.h"
}
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static double N() { return std::normal_distribution<double>(0, 1)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
static long bad = 0, tot = 0, bad_qr = 0, bad_svd = 0, bad_q = 0;
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
    std::printf("m9eigen: %ld/%ld\n", bad, tot);
    return bad == 0 && tot > 0 ? 0 : 1;
}
