#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

struct Candidate {
    cv::Point2f image;
    cv::Point3f world;
    int obs = 0;
};

struct FrameDump {
    int frame_id = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    int candidate_count = 0;
    cv::Vec3d predicted_t = {};
    bool pure_ok = false;
    int pure_inliers = 0;
    cv::Vec3d pure_t = {};
    double pure_rmse = 0.0;
    cv::Vec3d lm_t = {};
    double lm_rmse = 0.0;
    std::vector<Candidate> candidates;
};

static double dist3(const cv::Vec3d &a, const cv::Vec3d &b) {
    const cv::Vec3d d = a - b;
    return std::sqrt(d.dot(d));
}

static double reprojection_rmse(const std::vector<Candidate> &candidates, const cv::Mat &rvec,
                                const cv::Mat &tvec, const cv::Matx33d &K) {
    if (candidates.empty()) {
        return 0.0;
    }
    std::vector<cv::Point3f> world;
    std::vector<cv::Point2f> expected;
    world.reserve(candidates.size());
    expected.reserve(candidates.size());
    for (const Candidate &candidate : candidates) {
        world.push_back(candidate.world);
        expected.push_back(candidate.image);
    }
    std::vector<cv::Point2f> projected;
    cv::projectPoints(world, rvec, tvec, cv::Mat(K), cv::noArray(), projected);

    double sum = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
        const double dx = projected[i].x - expected[i].x;
        const double dy = projected[i].y - expected[i].y;
        sum += dx * dx + dy * dy;
    }
    return std::sqrt(sum / static_cast<double>(projected.size()));
}

static int parse_window(const std::string &text, int &start, int &end) {
    const size_t pos = text.find(':');
    if (pos == std::string::npos) {
        return 0;
    }
    start = std::stoi(text.substr(0, pos));
    end = std::stoi(text.substr(pos + 1));
    return 1;
}

static void usage(const char *argv0) {
    std::cerr << "Usage: " << argv0 << " --dump PATH [--window START:END]\n";
}

int main(int argc, char **argv) {
    std::string dump_path;
    int window_start = -1;
    int window_end = -1;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--dump" && i + 1 < argc) {
            dump_path = argv[++i];
        } else if (arg == "--window" && i + 1 < argc) {
            if (!parse_window(argv[++i], window_start, window_end)) {
                usage(argv[0]);
                return 2;
            }
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (dump_path.empty()) {
        usage(argv[0]);
        return 2;
    }

    std::ifstream in(dump_path);
    if (!in) {
        std::cerr << "Failed to open dump: " << dump_path << "\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "frame cand pure_ok pure_inl pure_rmse pure_jump lm_rmse lm_jump cv_ok cv_inl "
                 "cv_rmse cv_jump\n";

    std::string tag;
    while (in >> tag) {
        if (tag != "FRAME") {
            std::cerr << "Expected FRAME, got " << tag << "\n";
            return 1;
        }
        FrameDump frame;
        in >> frame.frame_id >> frame.fx >> frame.fy >> frame.cx >> frame.cy >>
            frame.candidate_count >> frame.predicted_t[0] >> frame.predicted_t[1] >>
            frame.predicted_t[2] >> frame.pure_ok >> frame.pure_inliers >> frame.pure_t[0] >>
            frame.pure_t[1] >> frame.pure_t[2] >> frame.pure_rmse >> frame.lm_t[0] >>
            frame.lm_t[1] >> frame.lm_t[2] >> frame.lm_rmse;
        if (!in) {
            std::cerr << "Malformed FRAME record\n";
            return 1;
        }

        while (in >> tag && tag != "END") {
            if (tag != "C") {
                std::cerr << "Expected C or END, got " << tag << "\n";
                return 1;
            }
            Candidate candidate;
            in >> candidate.image.x >> candidate.image.y >> candidate.world.x >>
                candidate.world.y >> candidate.world.z >> candidate.obs;
            frame.candidates.push_back(candidate);
        }

        if (window_start >= 0 &&
            (frame.frame_id < window_start || frame.frame_id > window_end)) {
            continue;
        }

        bool cv_ok = false;
        int cv_inliers = 0;
        double cv_rmse = 0.0;
        double cv_jump = 0.0;
        if (frame.candidates.size() >= 6) {
            std::vector<cv::Point3f> world;
            std::vector<cv::Point2f> image;
            world.reserve(frame.candidates.size());
            image.reserve(frame.candidates.size());
            for (const Candidate &candidate : frame.candidates) {
                world.push_back(candidate.world);
                image.push_back(candidate.image);
            }
            const cv::Matx33d K(frame.fx, 0, frame.cx, 0, frame.fy, frame.cy, 0, 0, 1);
            cv::Mat rvec;
            cv::Mat tvec;
            cv::Mat inliers;
            cv_ok = cv::solvePnPRansac(world, image, cv::Mat(K), cv::noArray(), rvec, tvec,
                                       false, 100, 3.0, 0.99, inliers, cv::SOLVEPNP_AP3P);
            if (cv_ok) {
                cv_inliers = inliers.rows;
                cv_rmse = reprojection_rmse(frame.candidates, rvec, tvec, K);
                cv::Vec3d cv_t(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
                cv_jump = dist3(cv_t, frame.predicted_t);
            }
        }

        std::cout << frame.frame_id << ' ' << frame.candidates.size() << ' ' << frame.pure_ok
                  << ' ' << frame.pure_inliers << ' ' << frame.pure_rmse << ' '
                  << dist3(frame.pure_t, frame.predicted_t) << ' ' << frame.lm_rmse << ' '
                  << dist3(frame.lm_t, frame.predicted_t) << ' ' << cv_ok << ' ' << cv_inliers
                  << ' ' << cv_rmse << ' ' << cv_jump << "\n";
    }
    return 0;
}
