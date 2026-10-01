// Scratch debug tool (not part of the module-3 deliverable): bisects the
// hyp.rot residual by feeding the dumped, bit-exact F21 (attempt 1,
// fr1_xyz, ref_frame=1 cur_frame=2 -- the first attempt with a hyp.rot
// mismatch) into the REAL library's fundamental_solver::decompose() path,
// printing every intermediate (E21, each JacobiSVD's U/sv/V, W, the
// U*W*Vt/U*Wt*Vt products, det sign flips, trans extraction/normalize,
// final R/t) as hex. Uses stella's own public static
// fundamental_solver::decompose/essential_solver::decompose/normalize
// (all public) plus direct Eigen::JacobiSVD calls (same as those
// functions use internally) to expose intermediates decompose() itself
// doesn't return.
#include "stella_vslam/type.h"
#include "stella_vslam/solve/fundamental_solver.h"
#include "stella_vslam/solve/essential_solver.h"
#include "stella_vslam/camera/perspective.h"

#include <Eigen/SVD>
#include <cstdio>
#include <cmath>

using namespace stella_vslam;

static std::string h(double v) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%a", v);
    return std::string(buf);
}
static void pr3(const char* label, const Mat33_t& m) {
    printf("%s:\n", label);
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            printf("  (%d,%d)=%s\n", r, c, h(m(r, c)).c_str());
}
static void prv(const char* label, const Vec3_t& v) {
    printf("%s: %s %s %s\n", label, h(v(0)).c_str(), h(v(1)).c_str(), h(v(2)).c_str());
}

int main() {
    Mat33_t F21;
    F21 << 0x1.e2fd8dfd6fe4ep-25, 0x1.2938510f84dffp-15, -0x1.b8e096afa03f4p-8,
           -0x1.2971aad2868f2p-15, 0x1.d5656d6ebfffep-25, 0x1.d85f4b904d438p-7,
           0x1.a6f83ac326a3bp-8, -0x1.de0f28ca9aba8p-7, 0x1.14136b94a37a8p-3;
    // (F21 dumped column-major: col0={row0,row1,row2}, col1={...}, col2={...})

    const double fx = 517.306408, fy = 516.469215, cx = 318.643040, cy = 255.313989;
    Mat33_t cam;
    cam << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    pr3("F21", F21);

    // essential_solver::create_E_21-equivalent used by fundamental_solver::decompose:
    // E_21 = cam_matrix_2.transpose() * F_21 * cam_matrix_1
    const Mat33_t E21 = cam.transpose() * F21 * cam;
    pr3("E21", E21);

    // essential_solver::decompose(E21, ...) body, replicated to expose intermediates:
    const Eigen::JacobiSVD<Mat33_t> svd(E21, Eigen::ComputeFullU | Eigen::ComputeFullV);
    pr3("svd.U", svd.matrixU());
    prv("svd.sv", svd.singularValues());
    pr3("svd.V", svd.matrixV());

    Vec3_t trans = svd.matrixU().col(2);
    prv("trans_raw(U.col(2))", trans);
    trans.normalize();
    prv("trans_normalized", trans);

    Mat33_t W = Mat33_t::Zero();
    W(0, 1) = -1;
    W(1, 0) = 1;
    W(2, 2) = 1;
    pr3("W", W);

    Mat33_t rot_1 = svd.matrixU() * W * svd.matrixV().transpose();
    pr3("rot_1_pre_detcheck", rot_1);
    printf("det(rot_1)=%s\n", h(rot_1.determinant()).c_str());
    if (rot_1.determinant() < 0) {
        rot_1 *= -1;
        printf("rot_1 flipped\n");
    }
    pr3("rot_1_final", rot_1);

    Mat33_t rot_2 = svd.matrixU() * W.transpose() * svd.matrixV().transpose();
    pr3("rot_2_pre_detcheck", rot_2);
    printf("det(rot_2)=%s\n", h(rot_2.determinant()).c_str());
    if (rot_2.determinant() < 0) {
        rot_2 *= -1;
        printf("rot_2 flipped\n");
    }
    pr3("rot_2_final", rot_2);

    // Cross-check against the real public API end-to-end.
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    solve::fundamental_solver::decompose(F21, cam, cam, init_rots, init_transes);
    printf("=== fundamental_solver::decompose() public output ===\n");
    for (size_t i = 0; i < init_rots.size(); ++i) {
        char label[32];
        std::snprintf(label, sizeof(label), "rots[%zu]", i);
        pr3(label, init_rots[i]);
        std::snprintf(label, sizeof(label), "transes[%zu]", i);
        prv(label, init_transes[i]);
    }
    return 0;
}
