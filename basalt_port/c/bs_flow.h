/* SPDX-License-Identifier: BSD-3-Clause AND MPL-2.0 */
/* Basalt pure-C port, module M5 (part 2): FrameToFrameOpticalFlow<float, Pattern51> as executed with euroc_config.json (stereo, 2 cameras).
 * Basalt (c) 2019 Vladyslav Usenko, Nikolaus Demmel, BSD-3-Clause.
 *
 * Per frame (processFrame): pyramid of both images (bs_image), then for every camera trackPoints(old -> new): per tracked point
 * trackPoint forward and backward (forward-backward distance^2 < optical_flow_max_recovered_dist2), then addPoints: detectKeypoints on cam0 level 0
 * in cells without a tracked point, ids = last_keypoint_id++ (cam0), stereo trackPoints cam0 -> cam1 of the new points, then filterPoints:
 * unproject both observations and drop cam1 observations with |p0^T E p1| > optical_flow_epipolar_error (or an invalid unprojection).
 * Observation maps are ordered by id (Eigen::aligned_map), so the result is a sorted array per camera.
 */
#ifndef BS_FLOW_H
#define BS_FLOW_H
#include <stddef.h>
#include <stdint.h>

#include "bs_cam.h"
#include "bs_image.h"
#include "bs_patch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { uint64_t id; float m[6]; } bs_flow_kp;     /* AffineCompact2f, column-major {l00, l10, l01, l11, tx, ty} */
typedef struct { bs_flow_kp *kp; int n, cap; } bs_flow_obs;

typedef struct {
    int pattern;                  /* optical_flow_pattern (50, 51, 52) */
    int levels;                   /* optical_flow_levels */
    int max_iterations;           /* optical_flow_max_iterations */
    int grid_size;                /* optical_flow_detection_grid_size */
    int skip_frames;              /* optical_flow_skip_frames (not used by the C flow: every frame is returned) */
    float max_recovered_dist2;    /* optical_flow_max_recovered_dist2 */
    float epipolar_error;         /* optical_flow_epipolar_error */
} bs_flow_config;

typedef struct {
    int ncam;                     /* 1 or 2 (stereo EuRoC: 2) */
    double intr[2][6];            /* fx fy cx cy xi alpha (double sphere) */
    double T_i_c[2][7];           /* raw calibration pose: px py pz qx qy qz qw (the quaternion is stored UNnormalised, as cereal does) */
} bs_flow_calib;

typedef struct bs_flow bs_flow;

/* euroc_config.json defaults of the optical-flow keys */
void bs_flow_config_default(bs_flow_config *c);
/* read the config.optical_flow_* keys of a basalt VioConfig json (strtod); returns 0 on success */
int bs_flow_config_load(const char *path, bs_flow_config *c);
/* read intrinsics / T_imu_cam of a basalt calibration json (double sphere only); returns 0 on success */
int bs_flow_calib_load(const char *path, bs_flow_calib *c);
/* the essential matrix (4x4 float column-major) computeEssential(T_i_c[0]^-1 * T_i_c[1]) cast to float; test hook */
void bs_flow_essential(const bs_flow_calib *c, float E[16]);

/* |p0^T E p1| before abs (float, Eigen evaluation order); test hook */
float bs_flow_epipolar(const float E[16], const float p0[4], const float p1[4]);

bs_flow *bs_flow_new(const bs_flow_config *cfg, const bs_flow_calib *cal);   /* NULL on failure (unsupported pattern, no memory) */
void bs_flow_free(bs_flow *f);
/* processFrame for one stereo frame (images are uint16 `<< 8` as the loader returns them, size w x h, every camera the same size).
 * Returns 0, or an error code (BS_IMG_NOMEM, 10 = images too small for the pyramid). */
int bs_flow_process(bs_flow *f, int64_t t_ns, const uint16_t *const *img, int w, int h);
/* observations of the last frame (valid until the next bs_flow_process) */
const bs_flow_obs *bs_flow_result(const bs_flow *f, int cam);
uint64_t bs_flow_last_keypoint_id(const bs_flow *f);

/* test hooks: trackPoint (both directions are the caller's job) and trackPoints on explicit pyramids */
int bs_flow_track_point(const bs_flow *f, const bs_pyr *old_pyr, const bs_pyr *pyr, const float old_tr[6], float tr[6]);
/* trackPoints(pyr1, pyr2, in, out): out is cleared first; both in and out are id-sorted */
int bs_flow_track_points(const bs_flow *f, const bs_pyr *pyr1, const bs_pyr *pyr2, const bs_flow_obs *in, bs_flow_obs *out);

#ifdef __cplusplus
}
#endif
#endif
