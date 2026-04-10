#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct Config {
  std::string video_path = "test_kitti984.mp4";
  double seconds = 5.0;
  double timeout = 30.0;
  bool use_webcam_first = false;
  bool no_imshow = true;
  bool no_plot = true;
  std::string metrics_out;

  bool use_pnp = true;
  int pnp_min_corr = 8;
  int inlier_min_for_tri = 8;
  int kf_min_inliers = 20;
  double kf_max_rot_deg = 5.0;
  int max_proc_w = 640;
  int max_points = 15000;
  int cull_min_obs = 2;
};

struct Observation {
  int frame_id = -1;
  cv::Point2f uv;
};

struct MapPoint {
  cv::Point3d pt;
  std::vector<Observation> observations;
};

struct Frame {
  int id = -1;
  cv::Matx44d pose = cv::Matx44d::eye();
  std::vector<cv::Point2f> kps;
  cv::Mat des;
  std::unordered_map<std::uint64_t, int> uv_to_point;
};

struct FrameStat {
  int frame_id = 0;
  int inliers = 0;
  bool used_pnp = false;
  bool is_keyframe = false;
  int points_added = 0;
  int points_total = 0;
  cv::Point3d xyz;
};

static cv::Matx33d poseRotation(const cv::Matx44d& T) {
  return cv::Matx33d(
      T(0, 0), T(0, 1), T(0, 2),
      T(1, 0), T(1, 1), T(1, 2),
      T(2, 0), T(2, 1), T(2, 2));
}

static cv::Vec3d poseTranslation(const cv::Matx44d& T) {
  return cv::Vec3d(T(0, 3), T(1, 3), T(2, 3));
}

static cv::Matx44d makePose(const cv::Matx33d& R, const cv::Vec3d& t) {
  cv::Matx44d T = cv::Matx44d::eye();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T(r, c) = R(r, c);
    }
    T(r, 3) = t[r];
  }
  return T;
}

static cv::Matx44d composeRelative(const cv::Matx44d& T_rel, const cv::Matx44d& T_prev) {
  const cv::Matx33d R_rel = poseRotation(T_rel);
  const cv::Vec3d t_rel = poseTranslation(T_rel);
  const cv::Matx33d R_prev = poseRotation(T_prev);
  const cv::Vec3d t_prev = poseTranslation(T_prev);
  return makePose(R_rel * R_prev, R_rel * t_prev + t_rel);
}

static cv::Point3d cameraCenter(const cv::Matx44d& T) {
  const cv::Matx33d R = poseRotation(T);
  const cv::Vec3d t = poseTranslation(T);
  const cv::Vec3d c = -(R.t() * t);
  return cv::Point3d(c[0], c[1], c[2]);
}

static double rotationDegreesBetween(const cv::Matx44d& Ta, const cv::Matx44d& Tb) {
  const cv::Matx33d Ra = poseRotation(Ta);
  const cv::Matx33d Rb = poseRotation(Tb);
  const cv::Matx33d R = Rb * Ra.t();
  cv::Vec3d rvec;
  cv::Rodrigues(cv::Mat(R), rvec);
  return cv::norm(rvec) * 180.0 / CV_PI;
}

static std::uint64_t packUv(const cv::Point2f& uv) {
  const std::uint32_t x = static_cast<std::uint32_t>(static_cast<int>(uv.x));
  const std::uint32_t y = static_cast<std::uint32_t>(static_cast<int>(uv.y));
  return (static_cast<std::uint64_t>(x) << 32u) | static_cast<std::uint64_t>(y);
}

static std::string jsonEscape(const std::string& value) {
  std::ostringstream out;
  for (char ch : value) {
    switch (ch) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << ch; break;
    }
  }
  return out.str();
}

static std::string boolJson(bool value) {
  return value ? "true" : "false";
}

class SimpleSlamOpt {
 public:
  explicit SimpleSlamOpt(Config config)
      : config_(std::move(config)),
        orb_(cv::ORB::create(3000)),
        matcher_(cv::BFMatcher::create(cv::NORM_HAMMING, false)) {}

  void processFrame(const cv::Mat& image, const cv::Matx33d& K) {
    const int fid = static_cast<int>(frames_.size());

    std::vector<cv::Point2f> kps;
    cv::Mat des;
    extractFeatures(image, kps, des);

    cv::Matx44d pose = cv::Matx44d::eye();
    int inliers = 0;
    bool used_pnp = false;

    if (fid > 0) {
      const Frame& prev = frames_.back();
      const auto matches = matchFeatures(prev.des, des);

      std::vector<cv::Point3f> world_pts;
      std::vector<cv::Point2f> image_pts;
      if (config_.use_pnp && !prev.uv_to_point.empty()) {
        world_pts.reserve(matches.size());
        image_pts.reserve(matches.size());
        for (const auto& match : matches) {
          const auto it = prev.uv_to_point.find(packUv(prev.kps[match.queryIdx]));
          if (it == prev.uv_to_point.end()) {
            continue;
          }
          const MapPoint& mp = points_[it->second];
          world_pts.emplace_back(static_cast<float>(mp.pt.x), static_cast<float>(mp.pt.y), static_cast<float>(mp.pt.z));
          image_pts.push_back(kps[match.trainIdx]);
        }
      }

      if (config_.use_pnp && static_cast<int>(world_pts.size()) >= config_.pnp_min_corr) {
        cv::Matx44d pnp_pose = cv::Matx44d::eye();
        int pnp_inliers = 0;
        if (solvePnp(world_pts, image_pts, K, pnp_pose, pnp_inliers)) {
          pose = pnp_pose;
          inliers = pnp_inliers;
          used_pnp = true;
        }
      }

      if (!used_pnp) {
        cv::Matx44d rel = cv::Matx44d::eye();
        std::vector<uchar> mask;
        if (estimatePoseE(prev.kps, kps, matches, K, rel, mask)) {
          inliers = static_cast<int>(std::count(mask.begin(), mask.end(), static_cast<uchar>(1)));
          pose = composeRelative(rel, prev.pose);
        }
      }
    }

    Frame current;
    current.id = fid;
    current.pose = pose;
    current.kps = std::move(kps);
    current.des = des;
    frames_.push_back(std::move(current));

    bool make_kf = false;
    int points_added_this_frame = 0;
    if (fid == 0 || last_keyframe_id_ < 0) {
      make_kf = true;
      last_keyframe_id_ = fid;
    } else {
      const double angle = rotationDegreesBetween(frames_[last_keyframe_id_].pose, pose);
      if (inliers < config_.kf_min_inliers || angle > config_.kf_max_rot_deg) {
        make_kf = true;
      }
    }

    if (make_kf && fid > 0 && inliers >= config_.inlier_min_for_tri) {
      Frame& prev = frames_[fid - 1];
      Frame& curr = frames_[fid];
      const auto tri_matches = matchFeatures(prev.des, curr.des);
      cv::Matx44d rel = cv::Matx44d::eye();
      std::vector<uchar> emask;
      if (estimatePoseE(prev.kps, curr.kps, tri_matches, K, rel, emask)) {
        std::vector<cv::Point2f> pts1;
        std::vector<cv::Point2f> pts2;
        pts1.reserve(tri_matches.size());
        pts2.reserve(tri_matches.size());
        for (std::size_t i = 0; i < tri_matches.size() && i < emask.size(); ++i) {
          if (!emask[i]) {
            continue;
          }
          pts1.push_back(prev.kps[tri_matches[i].queryIdx]);
          pts2.push_back(curr.kps[tri_matches[i].trainIdx]);
        }

        std::vector<cv::Point3d> tri_points;
        std::vector<uchar> tri_mask;
        triangulatePoints(prev.pose, curr.pose, pts1, pts2, K, tri_points, tri_mask);

        const int slots = std::max(0, config_.max_points - static_cast<int>(points_.size()));
        const int add_n = std::min(slots, static_cast<int>(tri_points.size()));
        for (int j = 0; j < add_n; ++j) {
          const int pid = static_cast<int>(points_.size());
          MapPoint mp;
          mp.pt = tri_points[j];
          mp.observations.push_back({prev.id, pts1[maskedIndex(tri_mask, j)]});
          mp.observations.push_back({curr.id, pts2[maskedIndex(tri_mask, j)]});
          points_.push_back(std::move(mp));
          prev.uv_to_point[packUv(pts1[maskedIndex(tri_mask, j)])] = pid;
          curr.uv_to_point[packUv(pts2[maskedIndex(tri_mask, j)])] = pid;
        }
        points_added_this_frame = add_n;
        tri_points_total_ += add_n;
        if (add_n > 0) {
          last_keyframe_id_ = fid;
        }
      }
    }

    if (fid % 10 == 0) {
      cullPoints();
    }

    if (used_pnp) {
      ++pnp_frames_;
    }
    frame_stats_.push_back(FrameStat{
        fid,
        inliers,
        used_pnp,
        make_kf,
        points_added_this_frame,
        static_cast<int>(points_.size()),
        cameraCenter(pose),
    });
  }

  std::string buildMetricsJson(const std::string& video_path, double duration_sec) const {
    double avg_inliers = 0.0;
    if (frame_stats_.size() > 1) {
      double sum = 0.0;
      for (std::size_t i = 1; i < frame_stats_.size(); ++i) {
        sum += frame_stats_[i].inliers;
      }
      avg_inliers = sum / static_cast<double>(frame_stats_.size() - 1);
    }

    int keyframes = 0;
    for (const auto& stat : frame_stats_) {
      if (stat.is_keyframe) {
        ++keyframes;
      }
    }

    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"frames\": " << frames_.size() << ",\n";
    out << "  \"points\": " << points_.size() << ",\n";
    out << "  \"duration_sec\": " << duration_sec << ",\n";
    out << "  \"used_pnp\": " << boolJson(config_.use_pnp) << ",\n";
    out << "  \"video_path\": \"" << jsonEscape(video_path) << "\",\n";
    out << "  \"keyframes\": " << keyframes << ",\n";
    out << "  \"ba_runs\": 0,\n";
    out << "  \"pnp_frames\": " << pnp_frames_ << ",\n";
    out << "  \"tri_points_total\": " << tri_points_total_ << ",\n";
    out << "  \"avg_inliers_after_first\": " << avg_inliers << ",\n";
    out << "  \"timeline\": [\n";
    for (std::size_t i = 0; i < frame_stats_.size(); ++i) {
      const auto& stat = frame_stats_[i];
      out << "    {\n";
      out << "      \"frame_id\": " << stat.frame_id << ",\n";
      out << "      \"inliers\": " << stat.inliers << ",\n";
      out << "      \"used_pnp\": " << boolJson(stat.used_pnp) << ",\n";
      out << "      \"is_keyframe\": " << boolJson(stat.is_keyframe) << ",\n";
      out << "      \"points_added\": " << stat.points_added << ",\n";
      out << "      \"points_total\": " << stat.points_total << ",\n";
      out << "      \"xyz\": [" << stat.xyz.x << ", " << stat.xyz.y << ", " << stat.xyz.z << "]\n";
      out << "    }" << (i + 1 < frame_stats_.size() ? "," : "") << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    return out.str();
  }

  std::size_t frameCount() const { return frames_.size(); }
  std::size_t pointCount() const { return points_.size(); }

 private:
  static int maskedIndex(const std::vector<uchar>& mask, int kept_index) {
    int count = -1;
    for (std::size_t i = 0; i < mask.size(); ++i) {
      if (!mask[i]) {
        continue;
      }
      ++count;
      if (count == kept_index) {
        return static_cast<int>(i);
      }
    }
    return 0;
  }

  void extractFeatures(const cv::Mat& image, std::vector<cv::Point2f>& kps_out, cv::Mat& des_out) const {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(gray, corners, 4000, 0.005, 5.0);
    if (corners.empty()) {
      kps_out.clear();
      des_out.release();
      return;
    }

    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));

    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(corners.size());
    for (const auto& corner : corners) {
      keypoints.emplace_back(corner, 20.0f);
    }

    orb_->compute(gray, keypoints, des_out);
    if (keypoints.empty() || des_out.empty()) {
      kps_out.clear();
      des_out.release();
      return;
    }

    kps_out.clear();
    kps_out.reserve(keypoints.size());
    for (const auto& kp : keypoints) {
      kps_out.push_back(kp.pt);
    }
  }

  std::vector<cv::DMatch> matchFeatures(const cv::Mat& des1, const cv::Mat& des2) const {
    if (des1.empty() || des2.empty()) {
      return {};
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher_->knnMatch(des1, des2, knn_matches, 2);

    std::vector<cv::DMatch> good;
    good.reserve(knn_matches.size());
    for (const auto& pair : knn_matches) {
      if (pair.size() < 2) {
        continue;
      }
      if (pair[0].distance < 0.80f * pair[1].distance) {
        good.push_back(pair[0]);
      }
    }

    std::sort(good.begin(), good.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
      return a.distance < b.distance;
    });
    return good;
  }

  bool estimatePoseE(const std::vector<cv::Point2f>& kps1,
                     const std::vector<cv::Point2f>& kps2,
                     const std::vector<cv::DMatch>& matches,
                     const cv::Matx33d& K,
                     cv::Matx44d& out_pose,
                     std::vector<uchar>& out_mask) const {
    out_pose = cv::Matx44d::eye();
    out_mask.clear();
    if (matches.size() < 8) {
      return false;
    }

    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    pts1.reserve(matches.size());
    pts2.reserve(matches.size());
    for (const auto& match : matches) {
      pts1.push_back(kps1[match.queryIdx]);
      pts2.push_back(kps2[match.trainIdx]);
    }

    cv::Mat mask;
    const cv::Mat E = cv::findEssentialMat(pts1, pts2, cv::Mat(K), cv::RANSAC, 0.999, 1.5, mask);
    if (E.empty()) {
      return false;
    }

    cv::Mat R;
    cv::Mat t;
    cv::recoverPose(E, pts1, pts2, cv::Mat(K), R, t, mask);

    cv::Matx33d Rm;
    cv::Vec3d tv;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        Rm(r, c) = R.at<double>(r, c);
      }
      tv[r] = t.at<double>(r, 0);
    }
    out_pose = makePose(Rm, tv);
    out_mask.assign(mask.begin<uchar>(), mask.end<uchar>());
    return true;
  }

  bool solvePnp(const std::vector<cv::Point3f>& world_pts,
                const std::vector<cv::Point2f>& image_pts,
                const cv::Matx33d& K,
                cv::Matx44d& out_pose,
                int& out_inliers) const {
    out_pose = cv::Matx44d::eye();
    out_inliers = 0;
    if (world_pts.size() < 6 || image_pts.size() != world_pts.size()) {
      return false;
    }

    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat inliers;
    const bool ok = cv::solvePnPRansac(world_pts, image_pts, cv::Mat(K), cv::noArray(),
                                       rvec, tvec, false, 100, 3.0, 0.99, inliers,
                                       cv::SOLVEPNP_AP3P);
    if (!ok || inliers.rows < 6) {
      return false;
    }

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Matx33d Rm;
    cv::Vec3d tv;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        Rm(r, c) = R.at<double>(r, c);
      }
      tv[r] = tvec.at<double>(r, 0);
    }
    out_pose = makePose(Rm, tv);
    out_inliers = inliers.rows;
    return true;
  }

  void triangulatePoints(const cv::Matx44d& pose1,
                         const cv::Matx44d& pose2,
                         const std::vector<cv::Point2f>& pts1,
                         const std::vector<cv::Point2f>& pts2,
                         const cv::Matx33d& K,
                         std::vector<cv::Point3d>& out_points,
                         std::vector<uchar>& out_mask) const {
    out_points.clear();
    out_mask.assign(pts1.size(), 0);
    if (pts1.empty() || pts2.size() != pts1.size()) {
      return;
    }

    const cv::Matx34d Rt1(
        pose1(0, 0), pose1(0, 1), pose1(0, 2), pose1(0, 3),
        pose1(1, 0), pose1(1, 1), pose1(1, 2), pose1(1, 3),
        pose1(2, 0), pose1(2, 1), pose1(2, 2), pose1(2, 3));
    const cv::Matx34d Rt2(
        pose2(0, 0), pose2(0, 1), pose2(0, 2), pose2(0, 3),
        pose2(1, 0), pose2(1, 1), pose2(1, 2), pose2(1, 3),
        pose2(2, 0), pose2(2, 1), pose2(2, 2), pose2(2, 3));
    const cv::Matx34d P1 = K * Rt1;
    const cv::Matx34d P2 = K * Rt2;

    cv::Mat pts1m(2, static_cast<int>(pts1.size()), CV_64F);
    cv::Mat pts2m(2, static_cast<int>(pts2.size()), CV_64F);
    for (int i = 0; i < static_cast<int>(pts1.size()); ++i) {
      pts1m.at<double>(0, i) = pts1[i].x;
      pts1m.at<double>(1, i) = pts1[i].y;
      pts2m.at<double>(0, i) = pts2[i].x;
      pts2m.at<double>(1, i) = pts2[i].y;
    }

    cv::Mat pts4d;
    cv::triangulatePoints(cv::Mat(P1), cv::Mat(P2), pts1m, pts2m, pts4d);

    const cv::Matx33d R1 = poseRotation(pose1);
    const cv::Vec3d t1 = poseTranslation(pose1);
    const cv::Matx33d R2 = poseRotation(pose2);
    const cv::Vec3d t2 = poseTranslation(pose2);

    out_points.reserve(pts1.size());
    for (int i = 0; i < pts4d.cols; ++i) {
      const double w = pts4d.at<double>(3, i);
      if (std::abs(w) <= 1e-8) {
        continue;
      }
      const cv::Vec3d X(
          pts4d.at<double>(0, i) / w,
          pts4d.at<double>(1, i) / w,
          pts4d.at<double>(2, i) / w);
      if (!std::isfinite(X[0]) || !std::isfinite(X[1]) || !std::isfinite(X[2])) {
        continue;
      }
      const double z1 = (R1 * X + t1)[2];
      const double z2 = (R2 * X + t2)[2];
      if (z1 <= 0.0 || z2 <= 0.0) {
        continue;
      }
      out_mask[i] = 1;
      out_points.emplace_back(X[0], X[1], X[2]);
    }
  }

  void cullPoints() {
    if (points_.empty()) {
      return;
    }

    std::vector<int> old_to_new(points_.size(), -1);
    std::vector<MapPoint> kept;
    kept.reserve(points_.size());
    for (int i = 0; i < static_cast<int>(points_.size()); ++i) {
      if (static_cast<int>(points_[i].observations.size()) < config_.cull_min_obs) {
        continue;
      }
      old_to_new[i] = static_cast<int>(kept.size());
      kept.push_back(points_[i]);
    }

    if (kept.size() == points_.size()) {
      return;
    }

    for (auto& frame : frames_) {
      std::unordered_map<std::uint64_t, int> remapped;
      remapped.reserve(frame.uv_to_point.size());
      for (const auto& entry : frame.uv_to_point) {
        const int old_idx = entry.second;
        if (old_idx < 0 || old_idx >= static_cast<int>(old_to_new.size())) {
          continue;
        }
        const int new_idx = old_to_new[old_idx];
        if (new_idx >= 0) {
          remapped[entry.first] = new_idx;
        }
      }
      frame.uv_to_point = std::move(remapped);
    }
    points_ = std::move(kept);
  }

  Config config_;
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::BFMatcher> matcher_;
  std::vector<Frame> frames_;
  std::vector<MapPoint> points_;
  std::vector<FrameStat> frame_stats_;
  int last_keyframe_id_ = -1;
  int pnp_frames_ = 0;
  int tri_points_total_ = 0;
};

static void printUsage() {
  std::cout
      << "usage: ./simple_slam_opt [--video_path PATH] [--seconds N] [--timeout N]\n"
      << "                         [--use_webcam_first] [--metrics_out PATH]\n"
      << "                         [--use_pnp {0,1}] [--pnp_min_corr N]\n"
      << "                         [--inlier_min_for_tri N] [--kf_min_inliers N]\n"
      << "                         [--kf_max_rot_deg N] [--max_proc_w N]\n"
      << "                         [--max_points N] [--cull_min_obs N] [positional_video]\n";
}

static std::string requireValue(int& index, int argc, char** argv, const std::string& arg) {
  if (index + 1 >= argc) {
    throw std::runtime_error("missing value for " + arg);
  }
  return argv[++index];
}

static Config parseArgs(int argc, char** argv) {
  Config cfg;
  std::string positional_video;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      printUsage();
      std::exit(0);
    } else if (arg == "--video_path") {
      cfg.video_path = requireValue(i, argc, argv, arg);
    } else if (arg == "--seconds") {
      cfg.seconds = std::stod(requireValue(i, argc, argv, arg));
    } else if (arg == "--timeout") {
      cfg.timeout = std::stod(requireValue(i, argc, argv, arg));
    } else if (arg == "--use_webcam_first") {
      cfg.use_webcam_first = true;
    } else if (arg == "--no_imshow") {
      cfg.no_imshow = true;
    } else if (arg == "--no_plot") {
      cfg.no_plot = true;
    } else if (arg == "--metrics_out") {
      cfg.metrics_out = requireValue(i, argc, argv, arg);
    } else if (arg == "--use_pnp") {
      cfg.use_pnp = std::stoi(requireValue(i, argc, argv, arg)) != 0;
    } else if (arg == "--pnp_min_corr") {
      cfg.pnp_min_corr = std::stoi(requireValue(i, argc, argv, arg));
    } else if (arg == "--inlier_min_for_tri") {
      cfg.inlier_min_for_tri = std::stoi(requireValue(i, argc, argv, arg));
    } else if (arg == "--kf_min_inliers") {
      cfg.kf_min_inliers = std::stoi(requireValue(i, argc, argv, arg));
    } else if (arg == "--kf_max_rot_deg") {
      cfg.kf_max_rot_deg = std::stod(requireValue(i, argc, argv, arg));
    } else if (arg == "--max_proc_w") {
      cfg.max_proc_w = std::stoi(requireValue(i, argc, argv, arg));
    } else if (arg == "--max_points") {
      cfg.max_points = std::stoi(requireValue(i, argc, argv, arg));
    } else if (arg == "--cull_min_obs") {
      cfg.cull_min_obs = std::stoi(requireValue(i, argc, argv, arg));
    } else if (!arg.empty() && arg[0] != '-' && positional_video.empty()) {
      positional_video = arg;
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (!positional_video.empty()) {
    cfg.video_path = positional_video;
  }
  return cfg;
}

static bool writeTextFile(const std::string& path, const std::string& text) {
  const std::filesystem::path p(path);
  if (p.has_parent_path()) {
    std::filesystem::create_directories(p.parent_path());
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  out << text;
  return static_cast<bool>(out);
}

static std::string autoMetricsPath(double seconds) {
  const int tag = static_cast<int>(seconds);
  return "runs/default/sec-" + std::to_string(tag) + "_auto_cpp.json";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Config cfg = parseArgs(argc, argv);

    cv::VideoCapture cap;
    bool is_webcam = false;
    if (cfg.use_webcam_first) {
      for (int idx = 0; idx < 10; ++idx) {
        cap.open(idx);
        if (cap.isOpened()) {
          std::cout << "Using webcam index " << idx << "\n";
          is_webcam = true;
          break;
        }
      }
    }

    if (!cap.isOpened()) {
      cap.open(cfg.video_path);
    }
    if (!cap.isOpened()) {
      std::cerr << "Failed to open video or webcam.\n";
      return 1;
    }

    cv::Mat first_frame;
    if (!cap.read(first_frame) || first_frame.empty()) {
      std::cerr << "Failed to read first frame.\n";
      return 1;
    }

    const int src_h = first_frame.rows;
    const int src_w = first_frame.cols;
    int proc_w = std::max(1, src_w / 2);
    int proc_h = std::max(1, src_h / 2);
    if (proc_w > cfg.max_proc_w) {
      const double scale = static_cast<double>(cfg.max_proc_w) / static_cast<double>(proc_w);
      proc_w = cfg.max_proc_w;
      proc_h = std::max(1, static_cast<int>(std::round(proc_h * scale)));
    }

    const cv::Matx33d K(
        static_cast<double>(proc_w), 0.0, static_cast<double>(proc_w) / 2.0,
        0.0, static_cast<double>(proc_w), static_cast<double>(proc_h) / 2.0,
        0.0, 0.0, 1.0);

    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 1e-3) {
      fps = 30.0;
    }
    const int max_frames = is_webcam ? std::numeric_limits<int>::max()
                                     : static_cast<int>(fps * cfg.seconds);

    SimpleSlamOpt slam(cfg);
    const auto start = std::chrono::steady_clock::now();
    int count = 0;

    while (cap.isOpened() && count < max_frames) {
      const auto now = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(now - start).count();
      if (elapsed > cfg.timeout) {
        std::cout << "Processing timeout exceeded, breaking loop.\n";
        break;
      }

      cv::Mat frame;
      if (!cap.read(frame) || frame.empty()) {
        break;
      }

      cv::Mat resized;
      cv::resize(frame, resized, cv::Size(proc_w, proc_h));
      slam.processFrame(resized, K);
      ++count;

      if (count % 10 == 0) {
        std::cout << "Frames=" << slam.frameCount() << " Points=" << slam.pointCount() << "\n";
      }
    }

    cap.release();

    const double duration = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const std::string metrics_json = slam.buildMetricsJson(cfg.video_path, duration);

    std::cout << std::fixed << std::setprecision(3)
              << "Processed frames=" << slam.frameCount()
              << ", points=" << slam.pointCount()
              << ", duration=" << duration << "s\n";

    if (cfg.metrics_out.empty()) {
      const std::string auto_path = autoMetricsPath(cfg.seconds);
      if (writeTextFile(auto_path, metrics_json)) {
        std::cout << "Wrote metrics to " << auto_path << "\n";
      } else {
        std::cerr << "Failed to write metrics to " << auto_path << "\n";
      }
    } else {
      if (!writeTextFile(cfg.metrics_out, metrics_json)) {
        std::cerr << "Failed to write metrics to " << cfg.metrics_out << "\n";
        return 1;
      }
      std::cout << "Wrote metrics to " << cfg.metrics_out << "\n";
    }
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << exc.what() << "\n";
    return 1;
  }
}
