/* SPDX-License-Identifier: BSD-3-Clause
 * C99 adaptation of OKVIS2's BRISK 2.0.7. Copyright notices and full terms:
 * ../reference_brisk/LICENSE-BRISK. No SIMD or external libraries.
 */
#ifndef OK_BRISK_H
#define OK_BRISK_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Same seven fields as cv::KeyPoint, in detector order. */
typedef struct {float x,y,size,angle,response;int32_t octave,class_id;} ok_brisk_keypoint;
typedef struct {int32_t score;uint16_t x,y;} ok_brisk_maximum;
typedef void (*ok_brisk_trace_fn)(void*,const char*,const void*,size_t);
typedef struct {ok_brisk_trace_fn emit;void *user;} ok_brisk_trace;
typedef struct {float x,y,sigma;} ok_brisk_point;
typedef struct ok_brisk_context ok_brisk_context;
/* Continuous grayscale 8U images, dimensions >= 20 and <= 65535.
 * The pixel count must fit INT_MAX/255 (signed 32-bit integral domain).
 * Detector specialization used by euroc.yaml: octaves=0, Harris scores.
 * radius>0, threshold>=1, maximum>0. Returns owned keypoints, free with free().
 * No keypoint mask or pre-populated feature mode. 0 success, -1 invalid/OOM. */
int ok_brisk_detect(const uint8_t *image,int width,int height,double radius,
                    int threshold,size_t maximum,ok_brisk_keypoint **out,size_t *count,
                    const ok_brisk_trace *trace);
/* BRISK v2, rotation invariance=true, scale invariance=false, patternScale=1. */
ok_brisk_context *ok_brisk_create(const ok_brisk_trace *trace);
void ok_brisk_destroy(ok_brisk_context *ctx);
/* Maps are caller-owned row-major float XYZ rays and row-major 2x3 image
 * Jacobians (6 floats/pixel), exactly the inputs of setCameraProperties().
 * Both NULL selects ordinary image-gradient orientation. Camera-map creation
 * belongs to the camera module. dir is gravity in the camera frame.
 * Keypoints are compacted in-place. Descriptors are malloc-owned count*48 bytes.
 * Returns 0 success; -1 invalid/OOM. */
int ok_brisk_describe(ok_brisk_context *ctx,const uint8_t *image,int width,int height,
                     const float *rays,const float *jacobians,float focal,const float dir[3],
                     ok_brisk_keypoint *keypoints,size_t *count,uint8_t **descriptors,
                     const ok_brisk_trace *trace);
#ifdef __cplusplus
}
#endif
#endif
