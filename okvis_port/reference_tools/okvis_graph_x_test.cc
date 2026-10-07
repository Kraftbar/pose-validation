// OK_PORT_TEST_C: ok_graph.c ok_cam.c ok_err.c ok_param.c ok_kin.c ok_eigen.c ok_dense.c ok_blas.c
// OKVIS2-X ViGraph::updateLandmarks quality kernel: ((dirs.colwise() - dirs.rowwise().mean()).square().rowwise().sum())
// .sqrt().norm() over a 3 x o Array of unit directions (upstream expression verbatim), real Eigen 3.4.0 vs
// ok_graph_dir_std_quality. Tolerance 0 (memcmp). Last line "okvis_graph_x_test: <mismatches>/<compared>".
#include <Eigen/Dense>
#include <cstdio>
#include <cstring>
#include <random>
extern "C" {
double ok_graph_dir_std_quality(const double* dirs, int o);
}
int main(int argc, char** argv) {
    long bad = 0, tot = 0;
    for (unsigned seed = 1; seed <= 4; ++seed) {
        std::mt19937_64 rng(seed);
        std::normal_distribution<double> N(0, 1);
        std::uniform_int_distribution<int> On(0, 70), K(0, 3);
        for (int c = 0; c < 20000; ++c) {
            const int num = On(rng);
            Eigen::Array<double, 3, Eigen::Dynamic> dirs(3, num);
            Eigen::Vector3d base(N(rng), N(rng), N(rng));
            base.normalize();
            const double spread = std::pow(10.0, -(double)K(rng) - 1.0);
            for (int i = 0; i < num; ++i) {
                Eigen::Vector3d d = base + spread * Eigen::Vector3d(N(rng), N(rng), N(rng));
                dirs.col(i) = d.normalized();
            }
            const int o = num == 0 ? 0 : std::uniform_int_distribution<int>(0, num)(rng);   /* inliers: the first o columns */
            Eigen::Array<double, 3, Eigen::Dynamic> dirso(3, o);
            dirso = dirs.topLeftCorner(3, o);
            Eigen::Vector3d std_dev = ((dirso.colwise() - dirso.rowwise().mean()).square().rowwise().sum()).sqrt();
            const double qe = std_dev.norm();
            const double qc = ok_graph_dir_std_quality(dirso.data(), o);
            ++tot;
            if (std::memcmp(&qe, &qc, 8)) { if (++bad <= 5) std::printf("  seed %u case %d o %d: eigen %.17g C %.17g\n", seed, c, o, qe, qc); }
        }
    }
    std::printf("okvis_graph_x_test: %ld/%ld\n", bad, tot);
    return bad == 0 ? 0 : 1;
}
