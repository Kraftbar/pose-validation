#include "simple_slam_c_shim.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <cstdlib>
#include <cstring>
#include <new>
#include <vector>

struct CVideoCaptureHandle {
    cv::VideoCapture cap;
};

namespace {

static void matToCImage(const cv::Mat &mat, CImage *out) {
    const std::size_t bytes = mat.total() * mat.elemSize();
    out->width = mat.cols;
    out->height = mat.rows;
    out->channels = mat.channels();
    out->stride = static_cast<int>(mat.step);
    out->data = static_cast<unsigned char *>(std::malloc(bytes));
    if (!out->data) {
        out->width = out->height = out->channels = out->stride = 0;
        return;
    }
    std::memcpy(out->data, mat.data, bytes);
}

static cv::Mat cImageToMat(const CImage *image) {
    const int type = image->channels == 1 ? CV_8UC1 : CV_8UC3;
    return cv::Mat(image->height, image->width, type, image->data, image->stride);
}

} // namespace

extern "C" {

CVideoCaptureHandle *cvideo_open(const char *path) {
    CVideoCaptureHandle *handle = new (std::nothrow) CVideoCaptureHandle();
    if (!handle) {
        return nullptr;
    }
    handle->cap.open(path);
    if (!handle->cap.isOpened()) {
        delete handle;
        return nullptr;
    }
    return handle;
}

void cvideo_close(CVideoCaptureHandle *handle) {
    if (!handle) {
        return;
    }
    handle->cap.release();
    delete handle;
}

double cvideo_get_fps(CVideoCaptureHandle *handle) {
    return handle ? handle->cap.get(cv::CAP_PROP_FPS) : 0.0;
}

int cvideo_get_width(CVideoCaptureHandle *handle) {
    return handle ? static_cast<int>(handle->cap.get(cv::CAP_PROP_FRAME_WIDTH)) : 0;
}

int cvideo_get_height(CVideoCaptureHandle *handle) {
    return handle ? static_cast<int>(handle->cap.get(cv::CAP_PROP_FRAME_HEIGHT)) : 0;
}

int cvideo_set_frame_pos(CVideoCaptureHandle *handle, double frame_pos) {
    if (!handle) {
        return 0;
    }
    return handle->cap.set(cv::CAP_PROP_POS_FRAMES, frame_pos) ? 1 : 0;
}

int cvideo_read(CVideoCaptureHandle *handle, CImage *out_bgr) {
    cv::Mat frame;
    if (!handle || !out_bgr) {
        return 0;
    }
    if (!handle->cap.read(frame) || frame.empty()) {
        return 0;
    }
    matToCImage(frame, out_bgr);
    return out_bgr->data ? 1 : 0;
}

void cimage_free(CImage *image) {
    if (!image) {
        return;
    }
    std::free(image->data);
    image->data = nullptr;
    image->width = image->height = image->channels = image->stride = 0;
}

int cimage_resize(const CImage *src, int width, int height, CImage *dst) {
    cv::Mat input;
    cv::Mat resized;
    if (!src || !src->data || !dst) {
        return 0;
    }
    input = cImageToMat(src);
    cv::resize(input, resized, cv::Size(width, height), 0.0, 0.0, cv::INTER_LINEAR);
    matToCImage(resized, dst);
    return dst->data ? 1 : 0;
}

int cimage_to_gray(const CImage *bgr, CImage *gray_out) {
    cv::Mat gray;
    if (!bgr || !bgr->data || !gray_out) {
        return 0;
    }
    cv::cvtColor(cImageToMat(bgr), gray, cv::COLOR_BGR2GRAY);
    matToCImage(gray, gray_out);
    return gray_out->data ? 1 : 0;
}

int cimage_extract_corners(const CImage *bgr, CImage *gray_out, CPoint2f *out_points,
                           int max_points) {
    cv::Mat gray;
    std::vector<cv::Point2f> corners;
    if (!bgr || !bgr->data || !gray_out || !out_points) {
        return 0;
    }
    cv::cvtColor(cImageToMat(bgr), gray, cv::COLOR_BGR2GRAY);
    if (max_points > 0) {
        cv::goodFeaturesToTrack(gray, corners, max_points, 0.005, 5.0);
        if (!corners.empty()) {
            cv::cornerSubPix(
                gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
        }
    }
    for (int i = 0; i < static_cast<int>(corners.size()); ++i) {
        out_points[i].x = corners[i].x;
        out_points[i].y = corners[i].y;
    }
    matToCImage(gray, gray_out);
    return static_cast<int>(corners.size());
}

int cimage_extract_corners_gray(const CImage *gray_in, CPoint2f *out_points, int max_points) {
    std::vector<cv::Point2f> corners;
    if (!gray_in || !gray_in->data || !out_points || max_points <= 0) {
        return 0;
    }
    cv::Mat gray = cImageToMat(gray_in);
    cv::goodFeaturesToTrack(gray, corners, max_points, 0.005, 5.0);
    if (!corners.empty()) {
        cv::cornerSubPix(
            gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.03));
    }
    for (int i = 0; i < static_cast<int>(corners.size()); ++i) {
        out_points[i].x = corners[i].x;
        out_points[i].y = corners[i].y;
    }
    return static_cast<int>(corners.size());
}

int cimage_track_corners_lk(const CImage *prev_gray, const CImage *curr_gray,
                            const CPoint2f *prev_points, int count, CPoint2f *curr_points,
                            unsigned char *status, float *err) {
    std::vector<cv::Point2f> prev;
    std::vector<cv::Point2f> curr;
    std::vector<unsigned char> local_status;
    std::vector<float> local_err;

    if (!prev_gray || !curr_gray || !prev_points || !curr_points || !status || count <= 0) {
        return 0;
    }

    prev.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        prev.emplace_back(prev_points[index].x, prev_points[index].y);
    }

    cv::calcOpticalFlowPyrLK(
        cImageToMat(prev_gray), cImageToMat(curr_gray), prev, curr, local_status, local_err,
        cv::Size(21, 21), 3,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01), 0, 1e-4);

    for (int index = 0; index < count; ++index) {
        curr_points[index].x = curr[index].x;
        curr_points[index].y = curr[index].y;
        status[index] = local_status[index];
        if (err) {
            err[index] = local_err[index];
        }
    }
    return count;
}

int csvd_compute(const double *a, int rows, int cols, double *w, double *u, double *vt) {
    cv::Mat A(rows, cols, CV_64F, const_cast<double *>(a));
    cv::Mat W, U, VT;
    if (!a || !w || !u || !vt || rows <= 0 || cols <= 0) {
        return 0;
    }
    cv::SVD::compute(A.clone(), W, U, VT, cv::SVD::FULL_UV);
    std::memcpy(w, W.ptr<double>(), static_cast<std::size_t>(W.total()) * sizeof(double));
    std::memcpy(u, U.ptr<double>(), static_cast<std::size_t>(U.total()) * sizeof(double));
    std::memcpy(vt, VT.ptr<double>(), static_cast<std::size_t>(VT.total()) * sizeof(double));
    return 1;
}

} // extern "C"
