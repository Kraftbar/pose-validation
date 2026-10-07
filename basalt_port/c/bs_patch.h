/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Basalt pure-C port, module M5 (part 1): the SE2 optical-flow patch.
 * Basalt (c) 2019 Vladyslav Usenko, Nikolaus Demmel, BSD-3-Clause (optical_flow/patch.h, patterns.h, image.h interp / interpGrad);
 * Sophus 1.24.6 SE2::exp (MIT); the Eigen 3.4.0 evaluation-order models (MPL-2.0) are marked "Eigen:" in bs_patch.c.
 * Float as the reference (OpticalFlowPatch<float, Pattern5x<float>>), -O2 -ffp-contract=off -fno-fast-math.
 *
 * Executed path (euroc_config.json: optical_flow_type frame_to_frame, pattern 51, levels 3, 5 iterations):
 *   FrameToFrameOpticalFlow<float, Pattern51>  -> OpticalFlowPatch<float, Pattern51<float>>   (PATTERN_SIZE 52)
 *   patch_optical_flow.h / multiscale_*.h and setData() (non-Jacobian) are NOT on the path.
 * Layouts (Eigen column-major):  pattern 2x52  {x0,y0,x1,y1,...};  AffineCompact2f  m[6] = {l00, l10, l01, l11, tx, ty};
 *   HJ = H_se2_inv_J_se2_T  3x52 (HJ[r + 3*i]).
 */
#ifndef BS_PATCH_H
#define BS_PATCH_H
#include <stdint.h>

#define BS_PAT 52

/* Image<const uint16_t> view: pitch in elements */
typedef struct { const uint16_t *p; int pitch, w, h; } bs_imgv;

typedef struct {
    float pos[2];
    float data[BS_PAT];       /* normalised intensities, -1 for points outside the image */
    float HJ[3 * BS_PAT];     /* H_se2_inv_J_se2_T, 3x52 column-major */
    float mean;
    int valid;
} bs_patch;

/* pattern 50 / 51 / 52 (Pattern50 = 0.75 * Pattern52, Pattern51 = 0.5 * Pattern52); other patterns are not supported (returns 0) */
int bs_pattern_init(float out[2 * BS_PAT], int pattern);

/* Image::InBounds(Vector2f p, border) with border a small int converted to float */
int bs_img_inbounds(const bs_imgv *im, float x, float y, float border);
float bs_img_interp(const bs_imgv *im, float x, float y);                 /* Image::interp<float>, needs InBounds(.,0) */
void bs_img_interp_grad(const bs_imgv *im, float x, float y, float o[3]); /* Image::interpGrad<float>, needs InBounds(.,1) */

/* OpticalFlowPatch::setFromImage(img, pos) (setDataJacSe2 + H^-1 J^T) */
void bs_patch_set(bs_patch *pt, const bs_imgv *im, const float pat[2 * BS_PAT], const float pos[2]);
/* OpticalFlowPatch::residual */
int bs_patch_residual(const bs_patch *pt, const bs_imgv *im, const float tpat[2 * BS_PAT], float res[BS_PAT]);

/* Sophus SE2<float>::exp(inc).matrix() as a 3x3 column-major matrix */
void bs_se2_exp_matrix(const float inc[3], float M[9]);
/* AffineCompact2f *= Matrix3f (Transform::operator*=: affine() * other) */
void bs_affine_mul_assign(float m[6], const float M[9]);

/* FrameToFrameOpticalFlow::trackPointAtLevel (max_iter = optical_flow_max_iterations); tr = AffineCompact2f in/out */
int bs_track_point_at_level(const bs_imgv *img2, const bs_patch *dp, const float pat[2 * BS_PAT], int max_iter, float tr[6]);

/* test hooks for the Eigen sub-models measured in the oracle */
void bs_patch_dbg_ldlt3_inverse(const float H[9], float Hinv[9]);        /* H.ldlt().solveInPlace(Identity) */
float bs_patch_dbg_dot52(const float *a, const float *b);                /* one coefficient of J^T * J (GEBP leftover rows/cols: scalar left fold) */
void bs_patch_dbg_inc(const float HJ[3 * BS_PAT], const float res[BS_PAT], float inc[3]); /* -HJ * res */

#endif
