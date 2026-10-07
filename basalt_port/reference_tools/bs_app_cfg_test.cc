// Basalt port M9 oracle: config / calibration json loading.  The real cereal readers (VioConfig::load, Calibration<double> via cereal JSONInputArchive,
// exactly as basalt_ref_driver.cpp does) vs bs_app_cfg_load of basalt_port/c/bs_app.c; every value compared bitwise.
//   usage: bs_app_cfg_test <config.json> <calib.json>   -> last line "bs_app_cfg: <mismatches>/<total>"
#include <basalt/calibration/calibration.hpp>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/vio_config.h>

#include <cstdio>
#include <cstring>
#include <fstream>

extern "C" {
#include "bs_app.h"
}

static long g_total = 0, g_bad = 0;
template <class A, class B>
static void chk(const char* what, const A& a, const B& b) {
  ++g_total;
  if (sizeof(A) != sizeof(B) || std::memcmp(&a, &b, sizeof(A))) { ++g_bad; std::printf("MISMATCH %s\n", what); }
}

int main(int argc, char** argv) {
  if (argc < 3) return 2;
  basalt::VioConfig vc;
  vc.load(argv[1]);
  basalt::Calibration<double> calib;
  {
    std::ifstream is(argv[2], std::ios::binary);
    cereal::JSONInputArchive ar(is);
    ar(calib);
  }
  bs_app_cfg c;
  char err[256];
  if (bs_app_cfg_load(&c, argv[1], argv[2], err, sizeof err)) { std::printf("C loader failed: %s\n", err); return 1; }
  chk("max_states", (int)vc.vio_max_states, c.max_states);
  chk("max_kfs", (int)vc.vio_max_kfs, c.max_kfs);
  chk("min_frames_after_kf", (int)vc.vio_min_frames_after_kf, c.min_frames_after_kf);
  chk("max_iterations", (int)vc.vio_max_iterations, c.max_iterations);
  chk("marg_lost_landmarks", (int)vc.vio_marg_lost_landmarks, c.marg_lost_landmarks);
  chk("new_kf_keypoints_thresh (float member)", vc.vio_new_kf_keypoints_thresh, (float)c.new_kf_keypoints_thresh);
  chk("obs_std_dev", vc.vio_obs_std_dev, c.obs_std_dev);
  chk("obs_huber_thresh", vc.vio_obs_huber_thresh, c.obs_huber_thresh);
  chk("min_triangulation_dist", vc.vio_min_triangulation_dist, c.min_triangulation_dist);
  chk("kf_marg_feature_ratio", vc.vio_kf_marg_feature_ratio, c.kf_marg_feature_ratio);
  chk("lm_lambda_initial", vc.vio_lm_lambda_initial, c.lm_lambda_initial);
  chk("lm_lambda_min", vc.vio_lm_lambda_min, c.lm_lambda_min);
  chk("lm_lambda_max", vc.vio_lm_lambda_max, c.lm_lambda_max);
  chk("init_pose_weight", vc.vio_init_pose_weight, c.init_pose_weight);
  chk("init_ba_weight", vc.vio_init_ba_weight, c.init_ba_weight);
  chk("init_bg_weight", vc.vio_init_bg_weight, c.init_bg_weight);
  for (int cam = 0; cam < 2; ++cam) {
    Eigen::VectorXd p = calib.intrinsics[cam].getParam();
    for (int k = 0; k < 6; ++k) chk("intrinsics", p[k], c.intr[cam][k]);
    const double* q = calib.T_i_c[cam].so3().data();   // x y z w, exactly as stored (cereal writes the raw coefficients)
    const double* t = calib.T_i_c[cam].translation().data();
    for (int k = 0; k < 3; ++k) chk("T_i_c t", t[k], c.T_i_c[cam][k]);
    for (int k = 0; k < 4; ++k) chk("T_i_c q", q[k], c.T_i_c[cam][3 + k]);
  }
  for (int k = 0; k < 9; ++k) chk("calib_accel_bias", calib.calib_accel_bias.getParam()[k], c.accel_bias_full[k]);
  for (int k = 0; k < 12; ++k) chk("calib_gyro_bias", calib.calib_gyro_bias.getParam()[k], c.gyro_bias_full[k]);
  for (int k = 0; k < 3; ++k) {
    chk("accel_noise_std", calib.accel_noise_std[k], c.accel_noise_std[k]);
    chk("gyro_noise_std", calib.gyro_noise_std[k], c.gyro_noise_std[k]);
    chk("accel_bias_std", calib.accel_bias_std[k], c.accel_bias_std[k]);
    chk("gyro_bias_std", calib.gyro_bias_std[k], c.gyro_bias_std[k]);
  }
  chk("imu_update_rate", calib.imu_update_rate, c.imu_update_rate);
  std::printf("bs_app_cfg: %ld/%ld\n", g_bad, g_total);
  return g_bad ? 1 : 0;
}
