// RD-VIO reference tooling: RD-VIO's own solve_pnp_6pt (rdvio_geometry/include/rdvio/geometry/pnp.h: OpenCV EPnP + Rodrigues in
// float) behind the C callback of rdvio_port/c/rd_imu_parsac.h, so the C system (rdvio_c_euroc built with -DRD_PNP_OPENCV)
// runs the native IMU-PARSAC with the real minimal solver until module M7b ports EPnP. Never part of the C port.
#include <rdvio/geometry/pnp.h>
extern "C" void rd_pnp6_opencv(void *ctx, const double X[6][3], const double x[6][2], double T[16]) {
    (void)ctx;
    std::array<rdvio::vector<3>, 6> Xs;
    std::array<rdvio::vector<2>, 6> xs;
    for (int i = 0; i < 6; ++i) { Xs[i] = rdvio::vector<3>(X[i][0], X[i][1], X[i][2]); xs[i] = rdvio::vector<2>(x[i][0], x[i][1]); }
    const rdvio::matrix<4> M = rdvio::solve_pnp_6pt(Xs, xs)[0];
    for (int i = 0; i < 16; ++i) T[i] = M.data()[i];   // column-major
}
