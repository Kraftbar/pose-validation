// Bit-exactness self-test of the Eigen evaluation-order kernels used by the
// module-5 tracking port (sv_eigen_mat4.c, sv_linalg.c) against REAL Eigen
// 3.4, in the exact expression contexts stella_vslam's tracking code uses:
//   T1  Mat44_t c = a * b                     (velocity * pose, pose * pose_wc, ...)
//   T2  trans_wc = -rot_cw.transpose() * trans_cw   (data::frame::set_pose_cw)
//   T3  trans_wc = -rot_wc * trans_cw               (data::keyframe::set_pose_cw, rot_wc materialized)
//   T4  pos_c = rot_cw * pos_w + trans_cw           (camera::perspective::reproject_to_image)
//   T5  (a - b).norm(), a.dot(b)                    (can_observe, keyframe_inserter)
// Random data, 300k cases each. Build (repo root):
//   gcc -std=c99 -O2 -ffp-contract=off -c stella_port/c/sv_linalg.c stella_port/c/sv_eigen_mat4.c
//   g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -Iexternal/eigen \
//       stella_port/reference_tools/eigen_shape_tests_track.cc sv_linalg.o sv_eigen_mat4.o -lm -o eigen_shape_tests_track
#include <Eigen/Core>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
#include "../c/sv_eigen_mat4.h"
#include "../c/sv_linalg.h"
}
typedef Eigen::Matrix<double, 4, 4> M44;
typedef Eigen::Matrix<double, 3, 3> M33;
typedef Eigen::Matrix<double, 3, 1> V3;

static bool same(const double* a, const double* b, int n) { return std::memcmp(a, b, sizeof(double) * n) == 0; }

// deliberately non-inlined so each expression is evaluated like a
// function-local statement in stella's code (arguments live in memory)
__attribute__((noinline)) static M44 mul44(const M44& a, const M44& b) { M44 c = a * b; return c; }
__attribute__((noinline)) static V3 f_t2(const M33& rot_cw, const V3& t) { V3 r = -rot_cw.transpose() * t; return r; }
__attribute__((noinline)) static V3 f_t3(const M33& rot_wc, const V3& t) { V3 r = -rot_wc * t; return r; }
__attribute__((noinline)) static V3 f_t4(const M33& rot_cw, const V3& p, const V3& t) { const V3 r = rot_cw * p + t; return r; }
__attribute__((noinline)) static double f_norm(const V3& a, const V3& b) { return (a - b).norm(); }
__attribute__((noinline)) static double f_dot(const V3& a, const V3& b) { return a.dot(b); }

int main() {
    std::mt19937 rng(777);
    std::uniform_real_distribution<double> ud(-3.0, 3.0);
    const int N = 300000;
    long bad[8] = {0};
    for (int t = 0; t < N; ++t) {
        M44 A, B;
        M33 R;
        V3 v, w, u;
        for (int i = 0; i < 16; ++i) { A.data()[i] = ud(rng); B.data()[i] = ud(rng); }
        for (int i = 0; i < 9; ++i) R.data()[i] = ud(rng);
        for (int i = 0; i < 3; ++i) { v(i) = ud(rng); w(i) = ud(rng); u(i) = ud(rng); }

        { M44 c = mul44(A, B); double o[16]; sv_mat4_mul(A.data(), B.data(), o); if (!same(c.data(), o, 16)) ++bad[0]; }
        {
            V3 e = f_t2(R, v); double o[3]; sv_mat3_mulv_lhs_transposed(R.data(), v.data(), o);
            o[0] = -o[0]; o[1] = -o[1]; o[2] = -o[2];
            if (!same(e.data(), o, 3)) ++bad[1];
        }
        {
            V3 e = f_t3(R, v); double o[3]; sv_mat3_mulv(R.data(), v.data(), o);
            o[0] = -o[0]; o[1] = -o[1]; o[2] = -o[2];
            if (!same(e.data(), o, 3)) ++bad[2];
        }
        {
            V3 e = f_t4(R, v, w); double o[3]; sv_mat3_mulv(R.data(), v.data(), o);
            o[0] = o[0] + w(0); o[1] = o[1] + w(1); o[2] = o[2] + w(2);
            if (!same(e.data(), o, 3)) ++bad[3];
        }
        {
            double e = f_norm(v, w);
            double d[3] = {v(0) - w(0), v(1) - w(1), v(2) - w(2)};
            double o = sqrt((d[0] * d[0] + d[1] * d[1]) + d[2] * d[2]);
            if (!same(&e, &o, 1)) ++bad[4];
            double e2 = f_dot(v, u);
            double o2 = (v(0) * u(0) + v(1) * u(1)) + v(2) * u(2);
            if (!same(&e2, &o2, 1)) ++bad[5];
        }
    }
    printf("T1 mat44*mat44: %ld/%d\nT2 -R^T t: %ld\nT3 -R t: %ld\nT4 R p + t: %ld\nT5 norm: %ld dot: %ld\n",
           bad[0], N, bad[1], bad[2], bad[3], bad[4], bad[5]);
    return 0;
}
