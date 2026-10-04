// Headless MSCEqF driver (benchmark glue, Apache-2.0 project): EuRoC-layout dir -> TUM trajectory of the IMU pose.
// Usage: msceqf_headless <config.yaml> <mav0 dir> <out.tum> [max_seconds]
// Own csv reader (the example's dataParser needs a groundtruth file and is O(n^2)); algorithm and options are the library's, untouched.
#include "msceqf/msceqf.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <opencv2/imgcodecs.hpp>
using namespace std;
static double ts(const string& s) { double v = stod(s); return v > 10e12 ? v / 1e9 : v; }
int main(int argc, char** argv) {
  if (argc < 4) { cerr << "usage\n"; return 1; }
  string cfg = argv[1], d = argv[2], outp = argv[3]; double maxs = argc > 4 ? atof(argv[4]) : 1e18;
  vector<msceqf::Imu> imus; vector<pair<double, string>> cams; string line;
  { ifstream f(d + "/imu0/data.csv"); getline(f, line);
    while (getline(f, line)) { if (line.empty()) continue; replace(line.begin(), line.end(), ',', ' '); istringstream ss(line); string t; double v[6]; ss >> t; for (int i = 0; i < 6; i++) ss >> v[i];
      msceqf::Imu m; m.timestamp_ = ts(t); m.ang_ << v[0], v[1], v[2]; m.acc_ << v[3], v[4], v[5]; imus.push_back(m); } }
  { ifstream f(d + "/cam0/data.csv"); getline(f, line);
    while (getline(f, line)) { if (line.empty()) continue; auto p = line.find(','); string t = line.substr(0, p), fn = line.substr(p + 1);
      fn.erase(remove_if(fn.begin(), fn.end(), [](char c) { return c == ' ' || c == '\r' || c == '\n'; }), fn.end()); cams.push_back({ts(t), fn}); } }
  double t0 = min(imus.front().timestamp_, cams.front().first);
  msceqf::MSCEqF sys(cfg);
  ofstream out(outp); out.precision(9); out << fixed;
  size_t ii = 0, frames = 0, poses = 0; double first_pose = -1, last_t = 0;
  auto w0 = chrono::steady_clock::now();
  for (auto& c : cams) {
    if (c.first - t0 > maxs) break;
    while (ii < imus.size() && imus[ii].timestamp_ <= c.first) { sys.processMeasurement(imus[ii]); ii++; }
    msceqf::Camera cam; cam.timestamp_ = c.first; cam.image_ = cv::imread(d + "/cam0/data/" + c.second);
    if (cam.image_.empty()) { cerr << "bad image " << c.second << endl; continue; }
    cam.mask_ = 255 * cv::Mat::ones(cam.image_.rows, cam.image_.cols, CV_8UC1);
    sys.processMeasurement(cam); frames++;
    if (sys.isInit()) {
      auto est = sys.stateEstimate(); auto q = est.P().q(); auto p = est.T().p();
      out << c.first << ' ' << p.x() << ' ' << p.y() << ' ' << p.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
      poses++; if (first_pose < 0) first_pose = c.first - t0; last_t = c.first - t0;
    }
  }
  double wall = chrono::duration<double>(chrono::steady_clock::now() - w0).count();
  fprintf(stderr, "FRAMES %zu POSES %zu FIRST_POSE_S %.2f LAST_S %.2f WALL %.2f\n", frames, poses, first_pose, last_t, wall);
  return 0;
}
