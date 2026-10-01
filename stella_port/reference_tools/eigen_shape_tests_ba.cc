// Bit-exactness self-test of the Eigen evaluation-order kernels in
// stella_port/c/sv_g2o_ba.c (sv_ba_k_*) against REAL Eigen 3.4 / real g2o
// helper templates, in the exact expression contexts g2o's BlockSolver uses:
//   sv_ba_k_axpy3     : g2o::internal::axpy<Matrix3d>  (SparseBlockMatrixDiagonal::multiply)
//   sv_ba_k_atxpy63   : g2o::internal::atxpy<Matrix<6,3>> (SparseBlockMatrixCCS::rightMultiply)
//   sv_ba_k_bdinv     : PoseLandmarkMatrixType BDinv = (*Bi) * Dinv
//   sv_ba_k_bb        : Map<Matrix<6,1>> Bb; Bb.noalias() += (*Bi) * db
//   sv_ba_k_schur_sub : (*Hi1i2).noalias() -= BDinv * Bj->transpose()
// plus Matrix3d::inverse() and Matrix3d*Vector3d used for Dinv / db, and the
// binary-edge quadratic-form products (depth-2 shapes). Segment offsets in
// the Map<VectorX> tests are varied so any runtime-alignment dependence would
// show. Random data, several 100k cases each. Build (repo root):
//   gcc -std=c99 -O2 -ffp-contract=off -c stella_port/c/sv_g2o_ba.c stella_port/c/sv_linalg.c ... (see below)
//   g++ -O2 -DNDEBUG -ffp-contract=off -fno-fast-math -Iexternal/eigen -Iexternal/candidates/deps/root/usr/include \
//       stella_port/reference_tools/eigen_shape_tests_ba.cc /tmp/ba_objs/*.o -lm -o /tmp/eigen_shape_tests_ba
#include <Eigen/Core>
#include <Eigen/LU>
#include <g2o/core/matrix_operations.h>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
#include "../c/sv_g2o_ba.h"
#include "../c/sv_linalg.h"
}
typedef Eigen::Matrix<double, 6, 3> M63;
typedef Eigen::Matrix<double, 3, 3> M33;
typedef Eigen::Matrix<double, 6, 6> M66;
typedef Eigen::Matrix<double, 6, 1> V6;
typedef Eigen::Matrix<double, 3, 1> V3;

static bool same(const double* a, const double* b, int n) { return std::memcmp(a, b, sizeof(double) * n) == 0; }

int main() {
    std::mt19937 rng(4242);
    std::uniform_real_distribution<double> ud(-3.0, 3.0);
    const int N = 300000;
    long bad_axpy = 0, bad_atxpy = 0, bad_bdinv = 0, bad_bb = 0, bad_schur = 0, bad_inv = 0, bad_mulv = 0, bad_q = 0;
    alignas(16) double buf[64];
    for (int t = 0; t < N; ++t) {
        M63 B;
        M33 D;
        for (int i = 0; i < 18; ++i) B.data()[i] = ud(rng);
        for (int i = 0; i < 9; ++i) D.data()[i] = ud(rng);
        double xs[16], ys[16];
        for (int i = 0; i < 16; ++i) { xs[i] = ud(rng); ys[i] = ud(rng); }
        const int off = t % 4;  // segment offsets 0..3 doubles into an aligned buffer

        // ---- axpy<Matrix3d>: y.segment<3>(off) += D * x.segment<3>(off)
        {
            std::memcpy(buf, ys, sizeof(ys));
            double ref[16];
            std::memcpy(ref, ys, sizeof(ys));
            Eigen::Map<Eigen::VectorXd> yv(buf, 16);
            Eigen::Map<const Eigen::VectorXd> xv(xs, 16);
            g2o::internal::axpy<M33>(D, xv, off, yv, off);
            std::memcpy(ref, buf, sizeof(ref));
            double y3[16];
            std::memcpy(y3, ys, sizeof(ys));
            sv_ba_k_axpy3(D.data(), xs + off, y3 + off);
            if (!same(ref, y3, 16)) ++bad_axpy;
        }
        // ---- atxpy<Matrix<6,3>>: y.segment<3>(off) += B^T * x.segment<6>(off)
        {
            std::memcpy(buf, ys, sizeof(ys));
            Eigen::Map<Eigen::VectorXd> yv(buf, 16);
            Eigen::Map<const Eigen::VectorXd> xv(xs, 16);
            g2o::internal::atxpy<M63>(B, xv, off, yv, off);
            double y3[16];
            std::memcpy(y3, ys, sizeof(ys));
            double Br[6][3];
            for (int r = 0; r < 6; ++r) for (int c = 0; c < 3; ++c) Br[r][c] = B(r, c);
            sv_ba_k_atxpy63(Br, xs + off, y3 + off);
            if (!same(buf, y3, 16)) ++bad_atxpy;
        }
        double Br[6][3];
        for (int r = 0; r < 6; ++r) for (int c = 0; c < 3; ++c) Br[r][c] = B(r, c);
        // ---- BDinv = B * Dinv
        {
            M63 BD = B * D;
            double out[6][3];
            sv_ba_k_bdinv(Br, D.data(), out);
            double ref[6][3];
            for (int r = 0; r < 6; ++r) for (int c = 0; c < 3; ++c) ref[r][c] = BD(r, c);
            if (!same(&ref[0][0], &out[0][0], 18)) ++bad_bdinv;
        }
        // ---- Bb.noalias() += B * db  (Map<Matrix<6,1>>)
        {
            V3 db;
            db << xs[0], xs[1], xs[2];
            std::memcpy(buf, ys, sizeof(ys));
            Eigen::Map<V6> Bb(buf + (off & 1 ? 0 : 2), 6);  // 16B-aligned or not, both ok for the type
            Bb.noalias() += B * db;
            double y6[16];
            std::memcpy(y6, ys, sizeof(ys));
            double* yp = y6 + (off & 1 ? 0 : 2);
            sv_ba_k_bb(Br, db.data(), yp);
            if (!same(buf, y6, 16)) ++bad_bb;
        }
        // ---- H.noalias() -= BDinv * Bj^T
        {
            M63 BD = B * D;
            M63 Bj;
            for (int i = 0; i < 18; ++i) Bj.data()[i] = ud(rng);
            M66 H;
            for (int i = 0; i < 36; ++i) H.data()[i] = ud(rng);
            M66 Href = H;
            Href.noalias() -= BD * Bj.transpose();
            double Hm[6][6], BDr[6][3], Bjr[6][3];
            for (int r = 0; r < 6; ++r) { for (int c = 0; c < 6; ++c) Hm[r][c] = H(r, c); for (int c = 0; c < 3; ++c) { BDr[r][c] = BD(r, c); Bjr[r][c] = Bj(r, c); } }
            sv_ba_k_schur_sub(Hm, BDr, Bjr);
            bool ok = true;
            for (int r = 0; r < 6; ++r) for (int c = 0; c < 6; ++c) ok = ok && std::memcmp(&Href(r, c), &Hm[r][c], 8) == 0;
            if (!ok) ++bad_schur;
        }
        // ---- Dinv = D.inverse();  db = Dinv * db
        {
            M33 Dinv = D.inverse();
            double mine[9];
            sv_mat3_inverse(D.data(), mine);
            if (!same(Dinv.data(), mine, 9)) ++bad_inv;
            V3 db;
            db << xs[3], xs[4], xs[5];
            V3 r = Dinv * db;
            double o[3];
            sv_mat3_mulv(Dinv.data(), db.data(), o);
            if (!same(r.data(), o, 3)) ++bad_mulv;
        }
        // ---- quadratic-form depth-2 shapes: AtO(3x2)=A^T(3x2)*omega(2x2); H3 += AtO*A;
        //      b3 += A^T*we ; Hpl(6x3) += B2^T*AtO^T
        {
            Eigen::Matrix<double, 2, 3> A;
            Eigen::Matrix<double, 2, 6> B2;
            for (int i = 0; i < 6; ++i) A.data()[i] = ud(rng);
            for (int i = 0; i < 12; ++i) B2.data()[i] = ud(rng);
            const double w = std::fabs(ud(rng)) + 0.1, rho1 = 0.3 + std::fabs(ud(rng)) * 0.1;
            Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * w;
            Eigen::Matrix2d omega = rho1 * info;
            Eigen::Vector2d err(ud(rng), ud(rng));
            Eigen::Vector2d omega_r = -info * err;
            omega_r *= rho1;
            Eigen::Matrix<double, 3, 2> AtO = A.transpose() * omega;
            M33 H3 = M33::Zero();
            V3 b3 = V3::Zero();
            Eigen::Matrix<double, 6, 3> Hpl = Eigen::Matrix<double, 6, 3>::Zero();
            b3.noalias() += A.transpose() * omega_r;
            H3.noalias() += AtO * A;
            Hpl.noalias() += B2.transpose() * AtO.transpose();
            // mine (same as sv_g2o_ba.c build_system)
            const double omega_diag = rho1 * w;
            const double we0 = (-(w * err(0))) * rho1, we1 = (-(w * err(1))) * rho1;
            double AtO0[3], AtO1[3], H3m[3][3], b3m[3], Hplm[6][3];
            for (int r = 0; r < 3; ++r) { AtO0[r] = A(0, r) * omega_diag; AtO1[r] = A(1, r) * omega_diag; }
            for (int r = 0; r < 3; ++r) {
                b3m[r] = 0.0 + (A(0, r) * we0 + A(1, r) * we1);
                for (int c = 0; c < 3; ++c) H3m[r][c] = 0.0 + (AtO0[r] * A(0, c) + AtO1[r] * A(1, c));
            }
            for (int r = 0; r < 6; ++r) for (int c = 0; c < 3; ++c) Hplm[r][c] = 0.0 + (B2(0, r) * AtO0[c] + B2(1, r) * AtO1[c]);
            bool ok = true;
            for (int r = 0; r < 3; ++r) { ok = ok && std::memcmp(&b3(r), &b3m[r], 8) == 0; for (int c = 0; c < 3; ++c) ok = ok && std::memcmp(&H3(r, c), &H3m[r][c], 8) == 0; }
            for (int r = 0; r < 6; ++r) for (int c = 0; c < 3; ++c) ok = ok && std::memcmp(&Hpl(r, c), &Hplm[r][c], 8) == 0;
            if (!ok) ++bad_q;
        }
    }
    std::printf("eigen_shape_tests_ba: %d cases each; mismatches: axpy3 %ld, atxpy63 %ld, bdinv %ld, bb %ld, schur_sub %ld, inverse3 %ld, mat3*vec3 %ld, quadform(depth-2 shapes) %ld\n",
                N, bad_axpy, bad_atxpy, bad_bdinv, bad_bb, bad_schur, bad_inv, bad_mulv, bad_q);
    return (bad_axpy | bad_atxpy | bad_bdinv | bad_bb | bad_schur | bad_inv | bad_mulv | bad_q) ? 1 : 0;
}
