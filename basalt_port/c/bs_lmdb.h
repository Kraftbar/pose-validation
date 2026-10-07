/* SPDX-License-Identifier: BSD-3-Clause
 * Basalt port, module M6: LandmarkDatabase<float> (basalt/vi_estimator/landmark_database.{h,cpp}, BSD-3-Clause, (c) 2019 Usenko, Demmel), C99.
 *
 *   kpts          unordered_map<size_t, Keypoint>                      -> bs_htab (BS_HK_U64), iteration order = libstdc++ node order
 *   observations  unordered_map<TimeCamId, map<TimeCamId, set<id>>>    -> bs_htab (BS_HK_TCID); inner map / set are sorted arrays
 *   Keypoint::obs aligned_map<TimeCamId, Vec2>                         -> sorted array (TimeCamId::operator<)
 * The unordered containers' iteration order (bs_hashorder.h) feeds float summation order in LinearizationAbsQR (landmark block / QR row layout,
 * error) and in BundleAdjustmentBase::computeError (host frame order), so it is modelled exactly (PLAN.md 4b).
 * Iterate like the C++ code:  for (bs_hnode* n = db->kpts.before_begin.next; n; n = n->next) { bs_keypoint* k = (bs_keypoint*)n->val; ... }
 * Validated at tolerance 0 (contents and both iteration orders) by basalt_port/reference_tools/bs_linabsqr_test.cc `lmdb` against the real class. */
#ifndef BS_LMDB_H
#define BS_LMDB_H

#include <stddef.h>
#include <stdint.h>
#include "bs_hashorder.h"

typedef struct bs_tcid { int64_t frame_id; uint64_t cam_id; } bs_tcid;   /* basalt::TimeCamId (CamId = size_t) */

typedef struct bs_obs { bs_tcid t; float pos[2]; } bs_obs;               /* Keypoint::obs entry */

typedef struct bs_keypoint {                                              /* Keypoint<float> */
    int64_t id;
    float direction[2];
    float inv_dist;
    bs_tcid host;
    bs_obs* obs;                  /* sorted by tcid */
    int nobs, cap;
    float backup_direction[2], backup_inv_dist;
} bs_keypoint;

typedef struct bs_tgt { bs_tcid t; int64_t* ids; int n, cap; } bs_tgt;   /* map<TimeCamId, set<KeypointId>> entry (ids sorted) */
typedef struct bs_host { bs_tgt* tgt; int n, cap; } bs_host;             /* map<TimeCamId, set<KeypointId>> sorted by target */

typedef struct bs_lmdb {
    bs_htab kpts;           /* val = bs_keypoint* */
    bs_htab observations;   /* val = bs_host* */
} bs_lmdb;

void bs_lmdb_init(bs_lmdb* db);
void bs_lmdb_destroy(bs_lmdb* db);

void bs_lmdb_add_landmark(bs_lmdb* db, int64_t lm_id, const float direction[2], float inv_dist, bs_tcid host);   /* addLandmark: kpts[lm_id] */
/* addObservation(tcid_target, KeypointObservation{kpt_id, pos}); returns 0 if the landmark does not exist (the C++ asserts) */
int bs_lmdb_add_observation(bs_lmdb* db, bs_tcid target, int64_t kpt_id, const float pos[2]);
void bs_lmdb_remove_frame(bs_lmdb* db, int64_t frame);
/* removeKeyframes(kfs_to_marg, poses_to_marg, states_to_marg_all): the sets are int64 arrays (membership only) */
void bs_lmdb_remove_keyframes(bs_lmdb* db, const int64_t* kfs, int nkf, const int64_t* poses, int np, const int64_t* states, int ns);
void bs_lmdb_remove_landmark(bs_lmdb* db, int64_t lm_id);
void bs_lmdb_remove_observations(bs_lmdb* db, int64_t lm_id, const bs_tcid* obs, int n);   /* set<TimeCamId> as array */

bs_keypoint* bs_lmdb_get_landmark(const bs_lmdb* db, int64_t lm_id);   /* NULL if absent */
int bs_lmdb_landmark_exists(const bs_lmdb* db, int64_t lm_id);
size_t bs_lmdb_num_landmarks(const bs_lmdb* db);
int bs_lmdb_num_observations(const bs_lmdb* db);
int bs_lmdb_num_observations_lm(const bs_lmdb* db, int64_t lm_id);
const bs_host* bs_lmdb_host(const bs_lmdb* db, bs_tcid host);          /* observations.at(host), NULL if absent */
void bs_lmdb_backup(bs_lmdb* db);
void bs_lmdb_restore(bs_lmdb* db);

int bs_tcid_cmp(bs_tcid a, bs_tcid b);
extern int bs_lmdb_ub;   /* counts calls that would be undefined behaviour in the C++ (see bs_lmdb.c) */
#endif
