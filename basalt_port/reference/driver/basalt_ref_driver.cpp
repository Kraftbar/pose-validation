// Headless, deterministic Basalt VIO driver for basalt_port (derived from basalt src/vio.cpp, BSD-3, with every GUI/Pangolin
// path removed). Same pipeline as vio.cpp: feed_images + feed_imu push into the optical-flow / estimator queues; the optical
// flow thread and the estimator thread each consume FIFO; results come back through out_state_queue. TBB parallelism is
// forced to 1 and OpenCV threading to 0, so every floating-point reduction runs in a fixed order.
// Usage: basalt_ref_driver --dataset-path D --cam-calib C.json --config-path CFG.json --out traj.tum
//          [--use-imu 1] [--use-double 0] [--max-frames N]
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>

#include <opencv2/core.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/global_control.h>

#include <basalt/calibration/calibration.hpp>
#include <basalt/io/dataset_io.h>
#include <basalt/optical_flow/optical_flow.h>
#include <basalt/serialization/headers_serialization.h>
#include <basalt/utils/vio_config.h>
#include <basalt/vi_estimator/vio_estimator.h>

int main(int argc, char** argv) {
  std::string dataset_path, calib_path, config_path, out_path = "trajectory.tum";
  bool use_imu = true, use_double = false;
  size_t max_frames = 0;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i], v = argv[i + 1];
    if (k == "--dataset-path") dataset_path = v;
    else if (k == "--cam-calib") calib_path = v;
    else if (k == "--config-path") config_path = v;
    else if (k == "--out") out_path = v;
    else if (k == "--use-imu") use_imu = std::stoi(v);
    else if (k == "--use-double") use_double = std::stoi(v);
    else if (k == "--max-frames") max_frames = std::stoul(v);
    else { std::cerr << "unknown option " << k << std::endl; return 2; }
  }
  tbb::global_control tbb_gc(tbb::global_control::max_allowed_parallelism, 1);
  cv::setNumThreads(0);

  basalt::VioConfig vio_config;
  if (!config_path.empty()) {
    vio_config.load(config_path);
    vio_config.vio_enforce_realtime = false;  // dataset mode: never drop frames
  }
  basalt::Calibration<double> calib;
  {
    std::ifstream is(calib_path, std::ios::binary);
    if (!is.is_open()) { std::cerr << "cannot open " << calib_path << std::endl; return 2; }
    cereal::JSONInputArchive ar(is);
    ar(calib);
  }
  basalt::DatasetIoInterfacePtr dataset_io = basalt::DatasetIoFactory::getDatasetIo("euroc");
  dataset_io->read(dataset_path);
  basalt::VioDatasetPtr ds = dataset_io->get_data();

  basalt::OpticalFlowBase::Ptr opt_flow = basalt::OpticalFlowFactory::getOpticalFlow(vio_config, calib);
  basalt::VioEstimatorBase::Ptr vio =
      basalt::VioEstimatorFactory::getVioEstimator(vio_config, calib, basalt::constants::g, use_imu, use_double);
  vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  opt_flow->output_queue = &vio->vision_data_queue;
  tbb::concurrent_bounded_queue<basalt::PoseVelBiasState<double>::Ptr> out_state_queue;
  vio->out_state_queue = &out_state_queue;

  std::vector<int64_t> t_ns;
  Eigen::aligned_vector<Sophus::SE3d> T_w_i;
  std::thread t1([&] {
    for (size_t i = 0; i < ds->get_image_timestamps().size(); i++) {
      if (vio->finished || (max_frames > 0 && i >= max_frames)) break;
      basalt::OpticalFlowInput::Ptr d(new basalt::OpticalFlowInput);
      d->t_ns = ds->get_image_timestamps()[i];
      d->img_data = ds->get_image_data(d->t_ns);
      opt_flow->input_queue.push(d);
    }
    opt_flow->input_queue.push(nullptr);
  });
  std::thread t2([&] {
    for (size_t i = 0; i < ds->get_gyro_data().size(); i++) {
      if (vio->finished) break;
      basalt::ImuData<double>::Ptr d(new basalt::ImuData<double>);
      d->t_ns = ds->get_gyro_data()[i].timestamp_ns;
      d->accel = ds->get_accel_data()[i].data;
      d->gyro = ds->get_gyro_data()[i].data;
      vio->imu_data_queue.push(d);
    }
    vio->imu_data_queue.push(nullptr);
  });
  std::thread t4([&] {
    basalt::PoseVelBiasState<double>::Ptr d;
    while (true) {
      out_state_queue.pop(d);
      if (!d.get()) break;
      t_ns.push_back(d->t_ns);
      T_w_i.push_back(d->T_w_i);
    }
  });
  auto t0 = std::chrono::steady_clock::now();
  vio->maybe_join();
  vio->drain_input_queues();
  t1.join();
  t2.join();
  t4.join();
  double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  std::ofstream os(out_path);
  os << "# timestamp tx ty tz qx qy qz qw\n";
  for (size_t i = 0; i < t_ns.size(); i++) {
    const Sophus::SE3d& p = T_w_i[i];
    os << std::scientific << std::setprecision(18) << t_ns[i] * 1e-9 << " " << p.translation().x() << " "
       << p.translation().y() << " " << p.translation().z() << " " << p.unit_quaternion().x() << " "
       << p.unit_quaternion().y() << " " << p.unit_quaternion().z() << " " << p.unit_quaternion().w() << "\n";
  }
  std::cerr << "states " << t_ns.size() << " wall_s " << wall << std::endl;
  return 0;
}
