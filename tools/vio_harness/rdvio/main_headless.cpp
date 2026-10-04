// Headless RD-VIO (Jianxff/rd_vio, Apache-2.0) driver (benchmark glue): EuRoC-layout dir -> TUM trajectory of the body (IMU) pose.
// Usage: rdvio_headless <sensor.yaml> <setting.yaml> <mav0 dir> <out.tum> [max_seconds]
// Images are undistorted here with the radtan parameters of cam0 in sensor.yaml (the library never applies them); no pacing, library built with -DTHREADING=OFF.
#include <unistd.h>
#include <rdvio/handler.h>
#include <rdvio/map/frame.h>
#include <rdvio/feature_tracker.h>
#include <rdvio/frontend.h>
#include <rdvio/extra/yaml_config.h>
#include <rdvio/extra/opencv_image.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
using namespace std;
static double ts(const string& s) { double v = stod(s); return v > 1e12 ? v / 1e9 : v; }
int main(int argc, char** argv) {
  if (argc < 5) { cerr << "usage\n"; return 1; }
  string calib = argv[1], conf = argv[2], d = argv[3], outp = argv[4]; double maxs = argc > 5 ? atof(argv[5]) : 1e18;
  YAML::Node y = YAML::LoadFile(calib);  // opencv-yaml header line "%YAML:1.0" is accepted by yaml-cpp as a directive
  auto K = y["cam0"]["intrinsics"].as<vector<double>>(); auto D = y["cam0"]["distortion"].as<vector<double>>(); auto res = y["cam0"]["resolution"].as<vector<int>>();
  cv::Mat Km = (cv::Mat_<double>(3, 3) << K[0], 0, K[2], 0, K[1], K[3], 0, 0, 1); cv::Mat Dm = (cv::Mat_<double>(1, 4) << D[0], D[1], D[2], D[3]);
  cv::Mat m1, m2; std::string dmodel = y["cam0"]["distortion_model"] ? y["cam0"]["distortion_model"].as<std::string>() : "radtan";
  if (dmodel == "equidistant") cv::fisheye::initUndistortRectifyMap(Km, Dm, cv::Mat::eye(3, 3, CV_64F), Km, cv::Size(res[0], res[1]), CV_32FC1, m1, m2);   // fisheye -> pinhole with the same K (central crop)
  else cv::initUndistortRectifyMap(Km, Dm, cv::Mat(), Km, cv::Size(res[0], res[1]), CV_32FC1, m1, m2);
  auto yc = make_shared<rdvio::extra::YamlConfig>(conf, calib);
  rdvio::Handler h(yc);
  struct I { double t, w[3], a[3]; }; vector<I> imus; vector<pair<double, string>> cams; string line;
  { ifstream f(d + "/imu0/data.csv"); getline(f, line);
    while (getline(f, line)) { if (line.empty()) continue; replace(line.begin(), line.end(), ',', ' '); istringstream ss(line); string t; I m; ss >> t; m.t = ts(t); for (int i = 0; i < 3; i++) ss >> m.w[i]; for (int i = 0; i < 3; i++) ss >> m.a[i]; imus.push_back(m); } }
  { ifstream f(d + "/cam0/data.csv"); getline(f, line);
    while (getline(f, line)) { if (line.empty()) continue; auto p = line.find(','); string t = line.substr(0, p), fn = line.substr(p + 1);
      fn.erase(remove_if(fn.begin(), fn.end(), [](char c) { return c == ' ' || c == '\r' || c == '\n'; }), fn.end()); cams.push_back({ts(t), fn}); } }
  double t0 = min(imus.front().t, cams.front().first);
  ofstream out(outp); out.precision(9); out << fixed;
  size_t ii = 0, frames = 0, poses = 0, losses = 0, inits = 0; int prev = -1; double first_pose = -1, last_t = 0;
  auto w0 = chrono::steady_clock::now();
  for (auto& c : cams) {
    if (c.first - t0 > maxs) break;
    // feed all IMU up to (and including) this frame time, then the frame
    while (ii < imus.size() && imus[ii].t <= c.first) { auto& m = imus[ii]; h.track_gyroscope(m.t, m.w[0], m.w[1], m.w[2]); h.track_accelerometer(m.t, m.a[0], m.a[1], m.a[2]); ii++; }
    cv::Mat raw = cv::imread(d + "/cam0/data/" + c.second, cv::IMREAD_GRAYSCALE);
    if (raw.empty()) { cerr << "bad image " << c.second << endl; continue; }
    cv::Mat un; cv::remap(raw, un, m1, m2, cv::INTER_LINEAR);
    auto im = make_shared<rdvio::extra::OpenCvImage>(); im->image = un.clone(); im->raw = un.clone(); im->t = c.first;
    h.track_camera(im); frames++;
    int st = (int)h.get_system_state();
    if (st != prev) { if (st == rdvio::SYS_TRACKING) inits++; else if (prev == rdvio::SYS_TRACKING) losses++; fprintf(stderr, "t=%.3f state %d -> %d\n", c.first - t0, prev, st); prev = st; }
    if (st == rdvio::SYS_TRACKING) {
      auto [tt, p] = h.get_latest_state();
      out << c.first << ' ' << p.p.x() << ' ' << p.p.y() << ' ' << p.p.z() << ' ' << p.q.x() << ' ' << p.q.y() << ' ' << p.q.z() << ' ' << p.q.w() << '\n';
      poses++; if (first_pose < 0) first_pose = c.first - t0; last_t = c.first - t0;
    }
  }
  double wall = chrono::duration<double>(chrono::steady_clock::now() - w0).count();
  fprintf(stderr, "FRAMES %zu POSES %zu FIRST_POSE_S %.2f LAST_S %.2f INITS %zu LOSSES %zu WALL %.2f\n", frames, poses, first_pose, last_t, inits, losses, wall);
  out.flush(); out.close(); fflush(stdout); fflush(stderr); _exit(0);
}
