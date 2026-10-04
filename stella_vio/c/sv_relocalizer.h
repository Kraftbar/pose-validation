/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD-2-Clause. AIST 2019 / stella-cv 2022; full notice in sv_relocalizer.c. */
#ifndef SV_RELOCALIZER_H
#define SV_RELOCALIZER_H
#include "sv_track.h"
#include "sv_bow_db.h"
#include "sv_pnp.h"
typedef struct {
 float bow_ratio,projection_ratio,robust_ratio,common_words_ratio;
 unsigned min_bow_matches,min_valid_obs,neighbors,max_ransac_iters,max_local_keyframes;
 int search_neighbor;
 int pnp_lo; /* stella_vio: 0 = sv_pnp_ransac (exact); 1 = P3P LO-RANSAC (sv_poselib.h), up to 1000 iterations */
} sv_reloc_config;
void sv_reloc_config_init(sv_reloc_config *cfg);
/* Candidate traversal order is supplied explicitly. The database wrapper
 * uses ascending IDs, matching deterministic reference patch 0012. */
int sv_reloc_by_candidates(const sv_reloc_config*,const sv_tr_config*,const sv_tr_map*,sv_tr_frame*,
                           const unsigned *ids,unsigned count,int use_robust,sv_pnp_trace_fn,void*);
int sv_relocalize(const sv_reloc_config*,const sv_tr_config*,const sv_tr_map*,const sv_bow_db*,
                  const sv_bow_vector*,sv_tr_frame*,sv_pnp_trace_fn,void*);
/* Automatic-relocalization branch of tracking_module::track. Caller has
 * installed curr_frm and its inherited reference keyframe. On success updates
 * last_reloc_frm_id/timestamp; subsequent local-map tracking stays with caller. */
int sv_reloc_tracking_glue(const sv_reloc_config*,sv_tracker*,const sv_tr_map*,const sv_bow_db*,
                           const sv_bow_vector*,sv_pnp_trace_fn,void*);
#endif
