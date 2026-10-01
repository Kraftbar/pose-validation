// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// Standalone (header-only Eigen from external/eigen, no stella_vslam link)
// ground-truth generator for stella_port/c/sv_eigen_svd.{h,c}. Builds N x 9
// coefficient matrices the same way homography_solver/fundamental_solver/
// essential_solver do (from normalized point correspondences, including
// near-degenerate sets) and 3x3 matrices (random, plus repeated/zero
// singular value cases), runs real Eigen::JacobiSVD<...>(ComputeFullU|
// ComputeFullV) on each, and writes inputs + every output the port checks
// as hex doubles (%a -- bit-exact, portable round-trip via strtod).
//
// Build with the SAME flags the reference build used (see
// runs/stella_port/reference_build/provenance.json): -O2 -ffp-contract=off
// -fno-fast-math, no -march=native (baseline x86-64 SSE2).
#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

static void put_hex(FILE *f, double v) { fprintf(f, "%a\n", v); }

static void dump_Nx9(FILE *f, const Eigen::MatrixXd &A, int N) {
    fprintf(f, "NX9 %d\n", N);
    for (int c = 0; c < 9; ++c)
        for (int r = 0; r < N; ++r) put_hex(f, A(r, c));

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::MatrixXd &V = svd.matrixV();
    for (int c = 0; c < 9; ++c)
        for (int r = 0; r < 9; ++r) put_hex(f, V(r, c));
    Eigen::VectorXd sv = Eigen::VectorXd::Zero(9);
    sv.head(svd.singularValues().size()) = svd.singularValues();
    for (int i = 0; i < 9; ++i) put_hex(f, sv(i));
    fprintf(f, "RANK %d\n", (int)svd.rank());
}

static void dump_3x3(FILE *f, const Eigen::Matrix3d &A) {
    fprintf(f, "MAT3\n");
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) put_hex(f, A(r, c));

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix3d &U = svd.matrixU();
    const Eigen::Matrix3d &V = svd.matrixV();
    const Eigen::Vector3d &sv = svd.singularValues();
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) put_hex(f, U(r, c));
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r) put_hex(f, V(r, c));
    for (int i = 0; i < 3; ++i) put_hex(f, sv(i));
}

// Builds an N x 9 coefficient matrix the way fundamental_solver /
// essential_solver do: A.row(i) = [x2*p1h ; y2*p1h ; p1h] for a homogeneous
// point p1h = (x1,y1,1). (homography_solver interleaves two rows per point
// with a different layout, but exercises the same JacobiSVD<MatrixXd> shape
// and QR preconditioning path -- N there is just 2*num_points.)
static Eigen::MatrixXd build_Nx9(std::mt19937 &rng, int N, bool degenerate) {
    std::normal_distribution<double> nd(0.0, 1.0);
    Eigen::MatrixXd A(N, 9);
    double line_a = nd(rng), line_b = nd(rng), line_c = nd(rng);
    for (int i = 0; i < N; ++i) {
        double x1, y1;
        if (degenerate) {
            // near-collinear point set (all near a random line) -> A is
            // close to rank-deficient, exercising the QR pivoting /
            // near-zero-pivot and near-equal-singular-value code paths.
            double t = nd(rng);
            x1 = t;
            y1 = (fabs(line_b) > 1e-9) ? (-line_a * t - line_c) / line_b + 1e-9 * nd(rng) : nd(rng);
        } else {
            x1 = nd(rng);
            y1 = nd(rng);
        }
        double x2 = nd(rng), y2 = nd(rng);
        double p0 = x1, p1 = y1, p2 = 1.0;
        A(i, 0) = x2 * p0; A(i, 1) = x2 * p1; A(i, 2) = x2 * p2;
        A(i, 3) = y2 * p0; A(i, 4) = y2 * p1; A(i, 5) = y2 * p2;
        A(i, 6) = p0;      A(i, 7) = p1;      A(i, 8) = p2;
    }
    return A;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s out.txt\n", argv[0]);
        return 1;
    }
    FILE *f = fopen(argv[1], "w");
    if (!f) { perror("fopen"); return 1; }

    std::mt19937 rng(20260924u);

    int shapes[] = {8, 9, 16, 100, 500};
    for (int N : shapes) {
        for (int trial = 0; trial < 6; ++trial) {
            Eigen::MatrixXd A = build_Nx9(rng, N, false);
            dump_Nx9(f, A, N);
        }
        for (int trial = 0; trial < 3; ++trial) {
            Eigen::MatrixXd A = build_Nx9(rng, N, true);
            dump_Nx9(f, A, N);
        }
    }
    // Also exercise homography_solver's row layout directly (2*num_points
    // rows built from two stacked equations per correspondence) for a
    // couple of point counts, since its A.rows() = 2*num_points differs
    // slightly in construction (though not in the JacobiSVD shape/path).
    {
        std::normal_distribution<double> nd(0.0, 1.0);
        for (int num_points : {4, 8, 50, 250}) {
            int N = 2 * num_points;
            Eigen::MatrixXd A(N, 9);
            for (int i = 0; i < num_points; ++i) {
                double x1 = nd(rng), y1 = nd(rng), x2 = nd(rng), y2 = nd(rng);
                Eigen::RowVector3d p1h(x1, y1, 1.0);
                A.block<1, 3>(2 * i, 0).setZero();
                A.block<1, 3>(2 * i, 3) = -p1h;
                A.block<1, 3>(2 * i, 6) = y2 * p1h;
                A.block<1, 3>(2 * i + 1, 0) = p1h;
                A.block<1, 3>(2 * i + 1, 3).setZero();
                A.block<1, 3>(2 * i + 1, 6) = -x2 * p1h;
            }
            dump_Nx9(f, A, N);
        }
    }

    // 3x3 cases: random, near-singular, repeated singular values (scaled
    // orthogonal), exact rank-1 and rank-2, zero matrix.
    {
        std::normal_distribution<double> nd(0.0, 1.0);
        for (int trial = 0; trial < 20; ++trial) {
            Eigen::Matrix3d A;
            for (int i = 0; i < 9; ++i) A(i % 3, i / 3) = nd(rng);
            dump_3x3(f, A);
        }
        for (int trial = 0; trial < 5; ++trial) {
            Eigen::Matrix3d Q = Eigen::Matrix3d::Random();
            Eigen::HouseholderQR<Eigen::Matrix3d> qr(Q);
            Eigen::Matrix3d Uo = qr.householderQ();
            double s = std::abs(nd(rng)) + 0.1;
            Eigen::Matrix3d A = Uo * (s * Eigen::Matrix3d::Identity());
            dump_3x3(f, A); // repeated (equal) singular values
        }
        {
            Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
            dump_3x3(f, A);
        }
        for (int trial = 0; trial < 5; ++trial) {
            Eigen::Vector3d u = Eigen::Vector3d::Random(), v = Eigen::Vector3d::Random();
            Eigen::Matrix3d A = u * v.transpose(); // rank-1: two zero singular values
            dump_3x3(f, A);
        }
        for (int trial = 0; trial < 5; ++trial) {
            Eigen::Matrix3d A = Eigen::Matrix3d::Random();
            A.col(2) = A.col(0) + A.col(1); // exactly rank <= 2
            dump_3x3(f, A);
        }
    }

    fclose(f);
    fprintf(stderr, "wrote %s\n", argv[1]);
    return 0;
}
