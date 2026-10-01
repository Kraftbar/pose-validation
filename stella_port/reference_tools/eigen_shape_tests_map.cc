// Bit-exactness self-test of the Eigen kernels used by the module-6 mapping
// port against REAL Eigen 3.4, in the exact expression contexts stella_vslam's
// mapping code uses:
//   M1  JacobiSVD<Matrix4d> full U/V     (solve::triangulator::triangulate, poses)
//   M2  rot_cw.block<1,3>(2,0).dot(pos_w) + trans_cw(2)   (check_depth_is_positive)
//   M3  create_E_21: rot_21 = rot_2w * rot_1w.transpose();
//       trans_21 = -rot_21 * trans_1w + trans_2w; E = skew(trans_21) * rot_21
// Random data. Build (repo root):
//   gcc -std=c99 -O2 -ffp-contract=off -c stella_port/c/sv_eigen_svd.c stella_port/c/sv_eigen_qr.c stella_port/c/sv_linalg.c
//   g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -Iexternal/eigen stella_port/reference_tools/eigen_shape_tests_map.cc \
//       sv_eigen_svd.o sv_eigen_qr.o sv_linalg.o -lm -o eigen_shape_tests_map
#include <Eigen/Core>
#include <Eigen/SVD>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
#include "../c/sv_eigen_svd.h"
#include "../c/sv_linalg.h"
}
typedef Eigen::Matrix<double, 4, 4> M44;
typedef Eigen::Matrix<double, 3, 3> M33;
typedef Eigen::Matrix<double, 3, 1> V3;

static bool same(const double* a, const double* b, int n) { return std::memcmp(a, b, sizeof(double) * n) == 0; }

__attribute__((noinline)) static void svd4(const M44& A, M44& U, M44& V, Eigen::Vector4d& s) {
    Eigen::JacobiSVD<M44> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    U = svd.matrixU();
    V = svd.matrixV();
    s = svd.singularValues();
}
__attribute__((noinline)) static double depth_z(const M33& rot_cw, const V3& pos_w, const V3& trans_cw) {
    return rot_cw.block<1, 3>(2, 0).dot(pos_w) + trans_cw(2);
}
__attribute__((noinline)) static M33 make_E(const M33& rot_1w, const V3& trans_1w, const M33& rot_2w, const V3& trans_2w) {
    const M33 rot_21 = rot_2w * rot_1w.transpose();
    const V3 trans_21 = -rot_21 * trans_1w + trans_2w;
    M33 skew;
    skew << 0, -trans_21(2), trans_21(1),
        trans_21(2), 0, -trans_21(0),
        -trans_21(1), trans_21(0), 0;
    return skew * rot_21;
}

int main() {
    std::mt19937_64 rng(12345);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    long bad[4] = {0, 0, 0, 0};
    const long N = 300000;
    for (long it = 0; it < N; ++it) {
        // M1: triangulation-like matrix
        M44 A;
        for (int i = 0; i < 16; ++i) A(i) = u(rng) * (it % 3 == 0 ? 1e-3 : 1.0);
        if (it % 2) { A.row(3) = A.row(0) * 0.5 + A.row(1); }  // rank-deficient-ish
        M44 U, V;
        Eigen::Vector4d s;
        svd4(A, U, V, s);
        double cU[16], cV[16], cs[4];
        sv_eigen_jacobisvd_4x4(A.data(), cU, cV, cs);
        if (!same(cU, U.data(), 16) || !same(cV, V.data(), 16) || !same(cs, s.data(), 4)) ++bad[0];
        // M2
        M33 R;
        for (int i = 0; i < 9; ++i) R(i) = u(rng);
        V3 p, t;
        for (int i = 0; i < 3; ++i) { p(i) = u(rng) * 3; t(i) = u(rng); }
        double d = depth_z(R, p, t);
        double rowz[3] = {R.data()[2], R.data()[5], R.data()[8]};
        double c1 = (rowz[0] * p(0) + (rowz[1] * p(1) + rowz[2] * p(2))) + t(2);
        double c2 = ((rowz[0] * p(0) + rowz[1] * p(1)) + rowz[2] * p(2)) + t(2);
        if (std::memcmp(&d, &c1, 8) != 0) ++bad[1];
        if (std::memcmp(&d, &c2, 8) != 0) ++bad[3];  // L variant, informational
        // M3
        M33 R1, R2;
        for (int i = 0; i < 9; ++i) { R1(i) = u(rng); R2(i) = u(rng); }
        V3 t1, t2;
        for (int i = 0; i < 3; ++i) { t1(i) = u(rng); t2(i) = u(rng); }
        M33 E = make_E(R1, t1, R2, t2);
        double r1t[9], r21[9], mt[3], t21[3], skew[9], Ec[9];
        sv_mat3_transpose(R1.data(), r1t);
        sv_mat3_mul(R2.data(), r1t, r21);
        sv_mat3_mulv(r21, t1.data(), mt);
        for (int i = 0; i < 3; ++i) t21[i] = -mt[i] + t2(i);
        skew[0] = 0; skew[3] = -t21[2]; skew[6] = t21[1];
        skew[1] = t21[2]; skew[4] = 0; skew[7] = -t21[0];
        skew[2] = -t21[1]; skew[5] = t21[0]; skew[8] = 0;
        sv_mat3_mul(skew, r21, Ec);
        if (!same(E.data(), Ec, 9)) ++bad[2];
    }
    std::printf("M1 svd4 mismatches %ld/%ld\n", bad[0], N);
    std::printf("M2 strided-row dot (R variant a0+(a1+a2)) mismatches %ld/%ld; L variant mismatches %ld\n", bad[1], N, bad[3]);
    std::printf("M3 create_E_21 mismatches %ld/%ld\n", bad[2], N);
    return (bad[0] | bad[1] | bad[2]) ? 1 : 0;
}
