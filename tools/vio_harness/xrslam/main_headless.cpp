// Headless XRSLAM driver (benchmark glue; written from the public player main.cpp API usage, Apache-2.0 project).
// Usage: xrslam-headless -sc slam.yaml -dc sensor.yaml --tum out.tum <euroc dir (mav0)>
// Also prints "FRAMES n POSES m LOST l" and wall time on stderr.
#include <argparse.hpp>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <dataset_reader.h>
#include <trajectory_writer.h>
#include <opencv2/opencv.hpp>
#include "XRSLAM.h"

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("xrslam-headless");
    program.add_argument("-sc").nargs(1);
    program.add_argument("-dc").nargs(1);
    program.add_argument("--tum").nargs(1);
    program.add_argument("input");
    program.parse_args(argc, argv);
    std::string data_path = program.get<std::string>("input");
    void *yaml_config = nullptr;
    int ok = XRSLAMCreate(program.get<std::string>("-sc").c_str(), program.get<std::string>("-dc").c_str(), "", "XRSLAM PC", &yaml_config);
    std::cerr << "create SLAM success: " << ok << std::endl;
    TumTrajectoryWriter out(program.get<std::string>("--tum"));
    std::unique_ptr<DatasetReader> reader = DatasetReader::create_reader(data_path, yaml_config);
    if (!reader) { fprintf(stderr, "Cannot open %s\n", data_path.c_str()); return 1; }
    bool hg = false, ha = false;
    DatasetReader::NextDataType nt;
    long frames = 0, poses = 0, lost = 0, state_changes = 0; int prev = -1;
    auto w0 = std::chrono::steady_clock::now();
    while ((nt = reader->next()) != DatasetReader::END) {
        switch (nt) {
        case DatasetReader::AGAIN: continue;
        case DatasetReader::GYROSCOPE: { hg = true; auto [t, g] = reader->read_gyroscope(); XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &g); } break;
        case DatasetReader::ACCELEROMETER: { ha = true; auto [t, a] = reader->read_accelerometer(); XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &a); } break;
        case DatasetReader::CAMERA: {
            auto [t, img] = reader->read_image();
            XRSLAMImage image; image.camera_id = 0; image.timeStamp = t; image.ext = nullptr;
            image.data = img.data; image.channel = img.channels(); image.stride = img.step[0];
            XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
            frames++;
            if (hg && ha) {
                XRSLAMRunOneFrame();
                XRSLAMState st; XRSLAMGetResult(XRSLAM_RESULT_STATE, &st);
                if ((int)st != prev) { state_changes++; fprintf(stderr, "t=%.3f state %d -> %d\n", t, prev, (int)st); prev = (int)st; }
                if (st == XRSLAM_STATE_TRACKING_SUCCESS) {
                    XRSLAMPose pb; XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pb);
                    if (pb.timestamp > 0) { out.write_pose(pb.timestamp, pb); poses++; }
                } else lost++;
            }
        } break;
        default: break;
        }
    }
    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count();
    fprintf(stderr, "FRAMES %ld POSES %ld NOTRACK %ld STATECHANGES %ld WALL %.2f\n", frames, poses, lost, state_changes, wall);
    fflush(stdout); fflush(stderr);
    _exit(0);   // XRSLAMDestroy() segfaults at teardown in this build; the trajectory file is complete (flushed per pose)

}
