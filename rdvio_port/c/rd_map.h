/* SPDX-License-Identifier: Apache-2.0 */
/*
 * RD-VIO pure-C port, module M6: the map layer (rdvio_map: Frame, Track, Map) and the keypoint bookkeeping of
 * Frame::detect_keypoints / Frame::track_keypoints.
 *
 * Derived from RD-VIO (Jianxff/rd_vio, Apache-2.0, a split of openxrlab/xrslam; see rdvio_port/LICENSES and NOTICE).
 * C99, <stdint.h> <stdlib.h> <string.h> <math.h> only.
 *
 * ---- semantics kept from the C++ ----
 * Frame and Track ids come from per-class counters starting at 1 (Identifiable); a cloned frame keeps the id of its source.
 * Track::keypoint_refs is a std::map ordered by FRAME ID (compare<Frame*> = std::less<Frame> = id <), so lookups go by id while
 * remove_keypoint's "is it the first frame" test compares POINTERS. Map::tracks is a vector with swap-with-last removal
 * (recycle_track), so the track order depends on the removal history; track_id_map is ordered by id.
 * Removing the first keypoint of a track that keeps others re-anchors the landmark: point = first camera * bearing / inv_depth,
 * then inv_depth = 1 / |new first camera^-1 * point| (rd_geom.c).
 *
 * ---- what the map layer does NOT own ----
 * Poses, camera / IMU extrinsics, K, the preintegrated rotation, tags and inverse depths are written by other modules
 * (tracker, initializer, solver). The fields are stored here; a replay harness fills them through `prepare` hooks.
 *
 * ---- the M6 reference log (patch 0009, <RDVIO_PORT_MAP_DIR>/map.bin; framed: u32 tag, u32 bytes, payload) ----
 * map = u32 map serial, obj = u32 frame object serial (assigned at construction), ids u64, indices u64, nil = ~0
 *    1 MAPNEW map            2 MAPDEL map (then FRAMEDEL of its frames, front to back)
 *    3 FRAMENEW obj, id, u32 source obj (clone) or 0         4 FRAMEDEL obj
 *    5 ATTACH map, obj, position, frames after                6 DETACH map, index, obj
 *    7 UNTRACK map, obj (start)    8 ERASEFRAME map, index, obj (start)    9 MARGFRAME map, index, obj (start)
 *   10 FIDX map, id, result     11 CREATETRACK map, track id, map_index (end)
 *   12 ERASETRACK map, track id (start)     13 PRUNE map, u32 n, n x track id (start)
 *   14 RECYCLE map, track id, map_index, id of the track swapped in (0 if it was the last)
 *   15 ADDKP track id, obj, kp, u32 triangulated (input), m_life after
 *   16 RMKP track id, obj, kp, u32 suicide, u32 was_first, refs after, u32 valid after
 *           [was_first && refs > 0: old pose q[4] p[3], old camera q[4] p[3], inv_depth before, new first obj, kp,
 *            new pose q[4] p[3], new camera q[4] p[3], inv_depth after]          (written before a recycle)
 *   17 APPENDKP obj, bearing[3]
 *   18 DETECT obj, K[9] (col-major), old n, new n, new n x pixel[2], (new n - old n) x bearing[3]
 *   19 TRACKKP obj, next obj, n, K[9], next K[9], camera q_cs[4], imu q_cs[4], next delta q[4], next imu q_cs[4], next camera q_cs[4],
 *              u32 predict, f64 rotation_ransac_threshold, rotation_misalignment_threshold, min_keypoint_distance,
 *              u64 np, np x predicted[2], n x tracked[2], n x u32 LK status, n x u32 trash (2 no track), u32 no_translation,
 *              n x u32 final status                            (written before the appends)
 *   20 DIGEST u32 nmaps, per map: map, u64 nframes, nframes x {obj, id, nkp, nkp x track id}, u64 ntracks,
 *              ntracks x {id, map_index, nrefs, nrefs x {obj, kp}, m_life, u32 tags, f64 inv_depth}
 */
#ifndef RD_MAP_H
#define RD_MAP_H
#include <stddef.h>
#include <stdint.h>
#include "../../okvis_port/c/ok_eigen.h"
#include "rd_imu.h"

#define RD_NIL ((size_t)-1)

enum { RD_FT_KEYFRAME = 0, RD_FT_NO_TRANSLATION, RD_FT_FIX_POSE, RD_FT_FIX_MOTION };
enum { RD_TT_VALID = 0, RD_TT_TRIANGULATED, RD_TT_FIX_INVD, RD_TT_TRASH, RD_TT_STATIC, RD_TT_OUTLIER, RD_TT_TEMP };
#define RD_TAG(t) (1u << (t))

typedef struct rd_map rd_map;
typedef struct rd_track rd_track;

/* std::shared_ptr<Image>: the owner embeds this as the first member of its image object; clones share it */
typedef struct rd_image { int refs; double t; void (*destroy)(struct rd_image* im); } rd_image;
rd_image* rd_image_retain(rd_image* im);
void rd_image_release(rd_image* im);

typedef struct rd_frame {
    uint64_t id;
    rd_map* map;
    size_t nkp, cap;
    double* bearing;               /* nkp x 3 */
    rd_track** track;              /* nkp, NULL = no track */
    uint32_t tags;                 /* RD_FT_* bits */
    /* written by other modules (Frame members); read by the map layer */
    double K[9];                   /* column-major */
    ok_quat pose_q; double pose_p[3];
    ok_quat cam_q; double cam_p[3];        /* camera.q_cs, p_cs */
    ok_quat imu_q; double imu_p[3];        /* imu.q_cs, p_cs */
    ok_quat delta_q;                       /* preintegration.delta.q (the M6 replay sets it; the system uses preint.delta.q) */
    void* user;                    /* the replay harness' object serial */
    /* the system state of rdvio::Frame (modules M8-M11) */
    double t;                              /* image->t */
    rd_image* image;                       /* shared with clones */
    double sqrt_inv_cov[4];                /* 2x2, column-major */
    rd_motion motion;
    rd_preint preint, kpreint;             /* preintegration, keyframe_preintegration (delta / jacobian / covariances) */
    rd_imu_sample* data; size_t ndata, cdata;      /* preintegration.data */
    rd_imu_sample* kdata; size_t nkdata, ckdata;   /* keyframe_preintegration.data */
    struct rd_frame** sub; size_t nsub, csub;      /* subframes (owned) */
} rd_frame;
/* the IMU sample lists (std::vector<ImuData> insert / assign) */
void rd_imu_list_insert(rd_imu_sample** a, size_t* n, size_t* cap, size_t at, const rd_imu_sample* src, size_t count);
void rd_frame_sub_push(rd_frame* f, rd_frame* sub);           /* subframes.emplace_back */
rd_frame* rd_frame_sub_pop(rd_frame* f);                      /* std::move(subframes.back()); subframes.pop_back() */
rd_frame* rd_frame_sub_take(rd_frame* f, size_t i);           /* move out and erase element i (subframes.erase) */

typedef struct rd_kref { rd_frame* frame; size_t kp; } rd_kref;

struct rd_track {
    uint64_t id;
    size_t map_index;
    rd_map* map;
    size_t nref, cap;
    rd_kref* ref;                  /* ascending frame id */
    uint32_t tags;                 /* RD_TT_* bits; a new track is TT_STATIC */
    double inv_depth;              /* LandmarkState::inv_depth */
    uint64_t life;                 /* m_life */
};

/* events of the map layer, in program order (a replay harness compares them with the reference log) and the inputs it may
 * have to provide before an operation reads state the map layer does not own */
typedef struct rd_map_event {
    int tag;                       /* the record tag of rd_map.h */
    const rd_map* map; const rd_frame* frame; const rd_track* track;
    uint64_t a, b, c;              /* index / position / id / count, per tag as in the record */
    int flag1, flag2;              /* suicide / was_first, triangulated, ... */
    const rd_frame* new_first; size_t new_kp; double inv_before, inv_after; int reanchor;
} rd_map_event;
typedef struct rd_map_hooks {
    void* ctx;
    void (*event)(void* ctx, const rd_map_event* e);
    /* before a re-anchoring: fill both frames' pose / camera and track->inv_depth (the solver's state) */
    void (*prepare_reanchor)(void* ctx, rd_track* t, rd_frame* old_first, rd_frame* new_first);
    /* before add_keypoint reads TT_TRIANGULATED */
    void (*prepare_add)(void* ctx, rd_track* t);
    /* Map::marginalize_frame: marginalization_factor->marginalize(index) (module M5) */
    void (*marginalize)(void* ctx, rd_map* m, size_t index);
} rd_map_hooks;
void rd_map_set_hooks(const rd_map_hooks* h);      /* process-wide, like the C++ id counters; NULL = none */
void rd_map_reset_ids(void);                       /* restart the Frame / Track id counters at 1 */

/* ---- Frame ---- */
rd_frame* rd_frame_new(void);
rd_frame* rd_frame_clone(const rd_frame* src);     /* Frame::clone: same id and tags, K, sqrt_inv_cov, image (shared), pose, motion,
                                                      camera, imu, preintegration (with its data), bearings; no tracks, no
                                                      keyframe_preintegration, no subframes */
void rd_frame_free(rd_frame* f);                   /* the destructor: reports FRAMEDEL, then destroys the subframes in order */
void rd_frame_append_keypoint(rd_frame* f, const double b[3]);
rd_track* rd_frame_get_track(rd_frame* f, size_t kp, rd_map* allocation_map);   /* allocates in allocation_map (NULL: f->map) */
void rd_apply_k(const double b[3], const double K[9], double px[2]);
void rd_remove_k(const double px[2], const double K[9], double b[3]);

/* Frame::detect_keypoints: `detect` gets the pixel positions of the existing keypoints and returns the full list (existing +
 * new, malloc'd) as the image detector does; the new ones become bearings */
typedef int (*rd_detect_fn)(void* ctx, rd_frame* f, const double* px, size_t n, double** out, size_t* nout);
int rd_frame_detect_keypoints(rd_frame* f, rd_detect_fn detect, void* ctx);

/* Frame::track_keypoints(next): `track` runs the image tracker (forward + backward LK and the image-side checks) on the
 * current pixel positions with the (predicted) next positions as initial flow; it overwrites next and fills status */
typedef struct rd_track_cfg {
    int predict_keypoints;
    double rotation_ransac_threshold, rotation_misalignment_threshold, min_keypoint_distance;
} rd_track_cfg;
typedef int (*rd_lk_fn)(void* ctx, rd_frame* f, rd_frame* next, const double* curr, double* next_px, char* status, size_t n);
int rd_frame_track_keypoints(rd_frame* f, rd_frame* next, const rd_track_cfg* cfg, rd_lk_fn track, void* ctx);

/* ---- Track ---- */
size_t rd_track_keypoint_index(const rd_track* t, const rd_frame* f);   /* by frame id; RD_NIL if absent */
void rd_track_add_keypoint(rd_track* t, rd_frame* f, size_t kp);
void rd_track_remove_keypoint(rd_track* t, rd_frame* f, int suicide_if_empty);

/* ---- Map ---- */
rd_map* rd_map_new(void);
void rd_map_free(rd_map* m);                        /* ~Map: its frames are destroyed front to back, then its tracks */
size_t rd_map_frame_num(const rd_map* m);
rd_frame* rd_map_get_frame(const rd_map* m, size_t index);
void rd_map_attach_frame(rd_map* m, rd_frame* f, size_t position);   /* RD_NIL = append */
rd_frame* rd_map_detach_frame(rd_map* m, size_t index);
void rd_map_untrack_frame(rd_map* m, rd_frame* f);
void rd_map_erase_frame(rd_map* m, size_t index);
void rd_map_marginalize_frame(rd_map* m, size_t index);
size_t rd_map_frame_index_by_id(const rd_map* m, uint64_t id);
size_t rd_map_track_num(const rd_map* m);
rd_track* rd_map_get_track(const rd_map* m, size_t index);
rd_track* rd_map_create_track(rd_map* m);
void rd_map_erase_track(rd_map* m, rd_track* t);
void rd_map_prune_tracks(rd_map* m, int (*condition)(void* ctx, const rd_track* t), void* ctx);
rd_track* rd_map_get_track_by_id(const rd_map* m, uint64_t id);
/* replay harnesses: give a track a logged id (re-sorts the id index) */
void rd_map_set_track_id(rd_map* m, rd_track* t, uint64_t id);

/* std::sort(first, last, comp) of libstdc++ (introsort: median of 3, unguarded partition, depth 2 log2 n, insertion sort below
 * 16); comp(a, b) = a strictly before b. Exposed for the later modules (sorts with ties). */
void rd_std_sort(void* base, size_t n, size_t size, int (*comp)(const void* a, const void* b));

#endif
