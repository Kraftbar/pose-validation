/* SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
 * OpenCV 4.6 specialization; notices in reference_cv/LICENSE-M7b-OpenCV. */
#ifndef RD_CV_UNDIST_H
#define RD_CV_UNDIST_H
#include <stdint.h>
/* K=(fx,fy,cx,cy), D=4 radtan or equidistant coefficients. Same K on output,
 * identity rectification, contiguous CV_32FC1 maps. Dimensions 1..32766.
 * Return 1 on success, 0 for invalid arguments. Inputs must be finite with
 * nonzero focal lengths; native default AVX2 dispatch is modeled in scalar C. */
int rd_cv_undistort_maps(const double K[4],const double D[4],int equidistant,
                        int w,int h,float *m1,float *m2);
/* Contiguous grayscale, equal source/destination dimensions, BORDER_CONSTANT
 * zero and INTER_LINEAR. src==dst is supported. Nonfinite map entries produce
 * zero like native cvRound's INT_MIN conversion. */
int rd_cv_remap_linear(const uint8_t *src,int w,int h,const float *m1,const float *m2,uint8_t *dst);
/* Same result as rd_cv_remap_linear for fixed maps, faster for repeated use
 * (per-pixel offsets/weights computed once). m1/m2 must outlive the plan. */
typedef struct rd_cv_remap_plan rd_cv_remap_plan;
rd_cv_remap_plan *rd_cv_remap_plan_new(int w,int h,const float *m1,const float *m2);
int rd_cv_remap_plan_apply(const rd_cv_remap_plan *plan,const uint8_t *src,uint8_t *dst);
void rd_cv_remap_plan_free(rd_cv_remap_plan *plan);
#endif
