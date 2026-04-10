#ifndef SIMPLE_SLAM_C_SHIM_H
#define SIMPLE_SLAM_C_SHIM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int width;
  int height;
  int channels;
  int stride;
  unsigned char* data;
} CImage;

typedef struct {
  float x;
  float y;
} CPoint2f;

typedef struct CVideoCaptureHandle CVideoCaptureHandle;

CVideoCaptureHandle* cvideo_open(const char* path);
void cvideo_close(CVideoCaptureHandle* handle);
double cvideo_get_fps(CVideoCaptureHandle* handle);
int cvideo_get_width(CVideoCaptureHandle* handle);
int cvideo_get_height(CVideoCaptureHandle* handle);
int cvideo_set_frame_pos(CVideoCaptureHandle* handle, double frame_pos);
int cvideo_read(CVideoCaptureHandle* handle, CImage* out_bgr);

void cimage_free(CImage* image);
int cimage_resize(const CImage* src, int width, int height, CImage* dst);
int cimage_to_gray(const CImage* bgr, CImage* gray);
int cimage_extract_corners(const CImage* bgr, CImage* gray_out, CPoint2f* out_points, int max_points);
int cimage_extract_corners_gray(const CImage* gray, CPoint2f* out_points, int max_points);
int cimage_track_corners_lk(const CImage* prev_gray,
                            const CImage* curr_gray,
                            const CPoint2f* prev_points,
                            int count,
                            CPoint2f* curr_points,
                            unsigned char* status,
                            float* err);

int csvd_compute(const double* a, int rows, int cols, double* w, double* u, double* vt);

#ifdef __cplusplus
}
#endif

#endif
