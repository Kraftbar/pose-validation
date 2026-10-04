/* SPDX-License-Identifier: BSD-2-Clause */
/* BSD 2-Clause License
 * Copyright (c) 2019, National Institute of Advanced Industrial Science
 * and Technology (AIST), All rights reserved.
 * Copyright (c) 2022, stella-cv, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* stella_vslam e445b545: system.cc (feed_monocular_frame, feed_frame, synchronize_background_modules of the
 * reference driver), tracking_module.cc (feed_frame, track, initialize, reset), module/initializer.cc
 * (initialize, create_initializer, try_initialize_for_monocular, create_map_for_monocular, scale_map),
 * module/keyframe_inserter.cc (insert_new_keyframe), mapping_module.cc (run_step order, keyframe queueing),
 * global_optimization_module.cc (run_step, correct_loop bookkeeping, reset), module/loop_detector.cc
 * (BoW database registration, keyframe protection flags), data/map_database.cc (add_keyframe, clear,
 * frame statistics), data/frame_statistics.cc, io/trajectory_io.cc (save_frame_trajectory).
 * See sv_system.h. This file only orchestrates the already validated modules 1-7 and the relocalizer. */
#include "sv_system.h"
#include "sv_bundle_adjuster.h"
#include "sv_eigen_mat4.h"
#include "sv_eigen_quaternion.h"
#include "sv_init.h"
#include "sv_map.h"
#include "sv_match_area.h"
#include "sv_pnp.h"
#include "sv_poselib.h"
#include "sv_rot.h"
#include "sv_triangulate.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define SV_SYS_MAX_KP 20000
#define SV_NONE_ID 0xFFFFFFFFu

/* per-frame observation storage (keypoints + descriptors + derived tables) */
typedef struct sv_sys_fd {
    sv_keypoint* kp;
    uint8_t* desc;
    sv_tr_obs obs;
    unsigned int id;
    double ts;
    int keep; /* a keyframe / the initializer reference: must outlive the frame */
} sv_sys_fd;

/* a BoW database entry: the keyframe object plus its BoW vector */
typedef struct sv_sys_dbk {
    sv_bow_db_keyframe k;
    sv_bow_vector bow;
    int bow_ready;
} sv_sys_dbk;

typedef struct sv_rhist {    /* pose of a frame of the rotation chain */
    unsigned int id;
    double ts, Rcw[9], c[3];
} sv_rhist;

typedef struct sv_sys_fstat {
    unsigned char valid, known, lost, rframe;
    int ref;
    int map;               /* stella_vio: map label of the reference keyframe when the frame was tracked */
    double rel[16];
    double ts;
} sv_sys_fstat;

struct sv_system {
    sv_system_params p;
    sv_image_bounds bounds;
    sv_tr_config cfg;
    sv_camera_perspective cam;
    sv_tr_map map;
    sv_mapping mp;
    sv_loop lp;
    sv_tracker trk;
    sv_reloc_config rcfg;
    sv_bow_db* db;

    /* record pools (pointer-stable objects), indexed by id */
    sv_tr_kf** kf_rec;
    sv_tr_lm** lm_rec;
    sv_sys_fd** kf_fd;
    unsigned int kf_cap, lm_cap;

    /* frames */
    unsigned int next_frame_id;
    sv_sys_fd* fd_prev;
    sv_sys_fd* fd_cur;
    sv_sys_fd** all_fd; /* every keep-marked frame, released at destroy */
    unsigned int n_all_fd, cap_all_fd;
    sv_keypoint* tmp_kp;
    uint8_t* tmp_desc;

    /* module::initializer */
    int init_state; /* 0 NotReady, 1 Initializing */
    sv_sys_fd* init_fd;
    float *prev_x, *prev_y;
    double init_frm_stamp;
    unsigned int init_ok_run; /* stella_vio: consecutive successful initialization attempts */

    /* map_database */
    unsigned int next_keyframe_id;
    int last_ins_kf; /* last_inserted_keyfrm_ */

    /* global optimization queue (keyframes the mapper handed over) */
    unsigned int gq[8];
    unsigned int n_gq;

    /* BoW database (relocalizer + loop detector) */
    sv_sys_dbk** dbk; /* pointer-stable (the database keeps pointers into them) */
    unsigned char* in_db;

    /* keyframe lifetime: erased objects waiting for their destruction */
    unsigned char* erased_pending;
    int* kf_erased_frame;
    int* kf_destroyed_frame;

    /* frame statistics (map_database::frm_stats_) */
    sv_sys_fstat* fs;
    unsigned int fs_cap;

    /* tracker-side persistent trace state */
    int last_path;
    int last_initial_valid;
    double last_initial_pose[16];
    unsigned int last_num_tracked, last_num_reliable;
    sv_tr_kf_decision last_decision;

    /* scratch for the PnP adapter */
    unsigned char* pnp_mask;
    unsigned int* pnp_inl;
    unsigned int pnp_cap;

    /* loop-step outputs of the current frame (for the report) */
    int cur_loop_accepted, cur_loop_cur, cur_loop_cand;
    unsigned int cur_global_steps;
    int cur_reset;

    sv_system_stats stats;
    long cur_frame;

    /* stella_vio: trajectory of dropped maps + re-initialization bookkeeping */
    sv_traj_entry* arch;
    unsigned int n_arch, cap_arch;
    int map_id;
    int next_label;                /* next unused map label (re-initializations and bridged parts both take one) */
    double lost_since;
    /* stella_vio map merge: per keyframe map label, label union (a merged map takes the label of the older one) */
    int* kf_label;
    int* map_root;
    unsigned int map_root_cap;
    unsigned int n_soft;           /* soft resets so far (old maps are kept in the structure) */

    /* stella_vio R-frames: the rotation chain (valid while r_on) */
    double ema_speed;              /* slowly smoothed speed of tracked frames (the scale reference across a gap) */
    int cal_on;                    /* a bridged map part waits for its scale calibration */
    unsigned int cal_k0, cal_lm0;
    double cal_t0, cal_len, cal_vold, cal_c0[3];
    double ema_vel[3];             /* smoothed centre velocity of tracked frames [map units / s] (R-frame extrapolation, bridge scale) */
    int ema_valid;
    int r_on;
    unsigned int r_last_inl;
    double r_last_par;
    double r_Rcw[9], r_c[3];       /* pose of the last chain frame (R_cw row-major, centre) */
    double r_dR[9];                /* its rotation relative to the previous chain frame (constant angular velocity prediction) */
    double r_vel[3];               /* centre velocity at the start of the chain [map units / s] */
    double r_t0, r_last_ts;
    int r_anchor;                  /* reference keyframe of the chain frames */
    unsigned int r_gyro_run;
    sv_rhist* r_hist;
    unsigned int r_nh, r_caph;

    /* stella_vio IMU: last good frame's data (kept alive for Lost dead-reckoning), gravity accumulators per map */
    sv_sys_fd* fd_good;
    double prev_ts;
    int have_prev_ts;
    double (*up_sum)[3];
    unsigned int* up_n;
    unsigned int up_cap;
};

/* ------------------------------------------------------------------ */
/* small math helpers                                                 */
/* ------------------------------------------------------------------ */
static void mat4_identity(double m[16]) {
    int i;
    for (i = 0; i < 16; ++i) {
        m[i] = (i % 5 == 0) ? 1.0 : 0.0;
    }
}

/* plain 4x4 inverse (cofactor expansion); only used for the frame-statistics bookkeeping, whose results
 * (the saved trajectory) are text with 9 significant digits */
static void mat4_inverse(const double m[16], double inv[16]) {
    double t[16];
    double det;
    int i;
    t[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    t[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    t[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    t[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    t[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    t[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    t[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    t[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    t[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    t[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    t[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    t[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    t[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    t[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    t[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    t[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
    det = m[0] * t[0] + m[1] * t[4] + m[2] * t[8] + m[3] * t[12];
    det = 1.0 / det;
    for (i = 0; i < 16; ++i) {
        inv[i] = t[i] * det;
    }
}

/* util::converter::inverse_pose of a rigid pose (column-major) */
static void pose_inverse_rigid(const double p[16], double out[16]) {
    int r, c;
    mat4_identity(out);
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            out[c * 4 + r] = p[r * 4 + c];
        }
    }
    for (r = 0; r < 3; ++r) {
        out[12 + r] = -((out[0 * 4 + r] * p[12] + out[1 * 4 + r] * p[13]) + out[2 * 4 + r] * p[14]);
    }
}

/* ------------------------------------------------------------------ */
/* parameters                                                         */
/* ------------------------------------------------------------------ */
void sv_system_params_default(sv_system_params* p, const sv_bow_vocab* vocab) {
    memset(p, 0, sizeof(*p));
    p->cam.fx = 517.306408;
    p->cam.fy = 516.469215;
    p->cam.cx = 318.643040;
    p->cam.cy = 255.313989;
    p->cam.k1 = 0.262383;
    p->cam.k2 = -0.953104;
    p->cam.p1 = -0.005358;
    p->cam.p2 = 0.002628;
    p->cam.k3 = 1.163314;
    p->cols = 640;
    p->rows = 480;
    p->orb.scale_factor = 1.2f;
    p->orb.num_levels = 8;
    p->orb.ini_fast_thr = 20;
    p->orb.min_fast_thr = 7;
    p->orb.min_area = 800;
    p->vocab = vocab;
    p->init_retry_threshold_time = 5.0;
    p->resume_mapper_after_loop = 0;
    p->enable_loop_closure = 1;
    p->reinit_lost_sec = 2.0;       /* stella_vio default (0 = exact port) */
    p->init_parallax_deg = 1.0f;
    p->init_min_tri = 50;
    p->init_confirm = 2;            /* stella_vio default (1 = exact port) */
    p->init_hamm = 50;
    p->init_ratio = 0.9f;
    p->init_max_level = 3;          /* stella_vio default (0 = exact port) */
    p->init_min_valid = 50;
    p->init_par_frac = 0.0f;
    p->init_refine = 0;
    p->init_lo = 0;
    p->init_lo_thr = 0.0f;
    p->pnp_lo = 0;
    p->init_seeds = 1;              /* opt-in: see RESULTS.md */
    p->imu = NULL;
    p->gyro_mode = 0;
    p->imu_R_BC[0] = p->imu_R_BC[4] = p->imu_R_BC[8] = 1.0;
    p->dr_max_sec = 10.0;
    p->rframe = 0;
    p->rframe_max_sec = 8.0;
    p->rframe_init_after = 1.0;
    p->rframe_hold_sec = 1.0;
    p->rframe_scale = 1;
    p->rframe_gyro_max = 40;
    p->rframe_calib_sec = 4.0;
    p->merge_maps = 0;
}

/* ------------------------------------------------------------------ */
/* pools                                                              */
/* ------------------------------------------------------------------ */
static void ensure_kf_capacity(sv_system* s, unsigned int need) {
    sv_tr_map* m = &s->map;
    unsigned int i, nc;
    if (need <= s->kf_cap) {
        return;
    }
    nc = need + 64;
    s->kf_rec = (sv_tr_kf**)realloc(s->kf_rec, nc * sizeof(sv_tr_kf*));
    s->kf_fd = (sv_sys_fd**)realloc(s->kf_fd, nc * sizeof(sv_sys_fd*));
    m->kfs = (sv_tr_kf**)realloc(m->kfs, nc * sizeof(sv_tr_kf*));
    s->dbk = (sv_sys_dbk**)realloc(s->dbk, nc * sizeof(sv_sys_dbk*));
    s->in_db = (unsigned char*)realloc(s->in_db, nc);
    s->erased_pending = (unsigned char*)realloc(s->erased_pending, nc);
    s->kf_erased_frame = (int*)realloc(s->kf_erased_frame, nc * sizeof(int));
    s->kf_destroyed_frame = (int*)realloc(s->kf_destroyed_frame, nc * sizeof(int));
    s->kf_label = (int*)realloc(s->kf_label, nc * sizeof(int));
    for (i = s->kf_cap; i < nc; ++i) {
        s->kf_label[i] = 0;
        s->kf_rec[i] = NULL;
        s->kf_fd[i] = NULL;
        m->kfs[i] = NULL;
        s->dbk[i] = NULL;
        s->in_db[i] = 0;
        s->erased_pending[i] = 0;
        s->kf_erased_frame[i] = -1;
        s->kf_destroyed_frame[i] = -1;
    }
    s->kf_cap = nc;
    m->kf_cap = nc;
    m->kf_pool = s->kf_rec;
    m->kf_pool_cap = nc;
}

static void ensure_lm_capacity(sv_system* s, unsigned int need) {
    sv_tr_map* m = &s->map;
    unsigned int i, nc;
    if (need <= s->lm_cap) {
        return;
    }
    nc = need + 4096;
    s->lm_rec = (sv_tr_lm**)realloc(s->lm_rec, nc * sizeof(sv_tr_lm*));
    m->lms = (sv_tr_lm**)realloc(m->lms, nc * sizeof(sv_tr_lm*));
    for (i = s->lm_cap; i < nc; ++i) {
        s->lm_rec[i] = NULL;
        m->lms[i] = NULL;
    }
    s->lm_cap = nc;
    m->lm_cap = nc;
}

static sv_tr_kf* kf_record(sv_system* s, unsigned int id) {
    ensure_kf_capacity(s, id + 1);
    if (!s->kf_rec[id]) {
        s->kf_rec[id] = (sv_tr_kf*)calloc(1, sizeof(sv_tr_kf));
    }
    return s->kf_rec[id];
}

static void kf_record_reset(sv_tr_kf* k) {
    free(k->lm);
    free(k->covis);
    free(k->covis_w);
    free(k->children);
    memset(k, 0, sizeof(*k));
}

/* stella_vio: map labels. map_root[l] = the label l was merged into (itself while unmerged) */
static int label_root(const sv_system* s, int l) {
    int guard = 0;
    while (l >= 0 && (unsigned int)l < s->map_root_cap && s->map_root[l] != l && guard++ < 1000) {
        l = s->map_root[l];
    }
    return l;
}

static void label_ensure(sv_system* s, int l) {
    if ((unsigned int)l >= s->map_root_cap) {
        unsigned int nc = (unsigned int)l * 2 + 8, i;
        s->map_root = (int*)realloc(s->map_root, nc * sizeof(int));
        for (i = s->map_root_cap; i < nc; ++i) {
            s->map_root[i] = (int)i;
        }
        s->map_root_cap = nc;
    }
}

static sv_tr_lm* alloc_lm_hook(void* user, unsigned int id) {
    sv_system* s = (sv_system*)user;
    ensure_lm_capacity(s, id + 1);
    if (!s->lm_rec[id]) {
        s->lm_rec[id] = (sv_tr_lm*)calloc(1, sizeof(sv_tr_lm));
    }
    return s->lm_rec[id];
}

/* ------------------------------------------------------------------ */
/* frame data                                                         */
/* ------------------------------------------------------------------ */
static void fd_destroy(sv_sys_fd* fd) {
    if (!fd) {
        return;
    }
    sv_tr_obs_free(&fd->obs);
    free(fd->kp);
    free(fd->desc);
    free(fd);
}

static void fd_keep(sv_system* s, sv_sys_fd* fd) {
    unsigned int i;
    if (fd->keep) {
        return;
    }
    fd->keep = 1;
    for (i = 0; i < s->n_all_fd; ++i) {
        if (s->all_fd[i] == fd) {
            return;
        }
    }
    if (s->n_all_fd == s->cap_all_fd) {
        s->cap_all_fd = s->cap_all_fd ? s->cap_all_fd * 2 : 256;
        s->all_fd = (sv_sys_fd**)realloc(s->all_fd, s->cap_all_fd * sizeof(sv_sys_fd*));
    }
    s->all_fd[s->n_all_fd++] = fd;
}

static int fd_in_all(const sv_system* s, const sv_sys_fd* fd) {
    unsigned int i;
    for (i = 0; i < s->n_all_fd; ++i) {
        if (s->all_fd[i] == fd) {
            return 1;
        }
    }
    return 0;
}

/* frees a non-keep frame that nothing refers to any more */
static void fd_release_if_unused(sv_system* s, sv_sys_fd* fd) {
    if (!fd || fd_in_all(s, fd) || fd == s->fd_cur || fd == s->fd_prev || fd == s->init_fd || fd == s->fd_good) {
        return;
    }
    fd_destroy(fd);
}

/* ------------------------------------------------------------------ */
/* BoW database                                                       */
/* ------------------------------------------------------------------ */
static void db_add(sv_system* s, unsigned int id) {
    const sv_tr_kf* kf = sv_tr_map_kf(&s->map, (int)id);
    sv_bow_feat_vector feat;
    sv_sys_dbk* e;
    if (!kf || s->in_db[id]) {
        return;
    }
    if (!s->dbk[id]) {
        s->dbk[id] = (sv_sys_dbk*)calloc(1, sizeof(sv_sys_dbk));
    }
    e = s->dbk[id];
    if (e->bow_ready) {
        sv_bow_vector_free(&e->bow);
        e->bow_ready = 0;
    }
    memset(&e->bow, 0, sizeof(e->bow));
    memset(&feat, 0, sizeof(feat));
    sv_bow_transform(s->cfg.vocab, kf->obs->desc, kf->obs->num_kp, 4, &e->bow, &feat);
    sv_bow_feat_vector_free(&feat);
    e->bow_ready = 1;
    e->k.id = id;
    e->k.bow = &e->bow;
    sv_bow_db_add(s->db, &e->k);
    s->in_db[id] = 1;
}

static void db_erase(sv_system* s, unsigned int id) {
    if (id < s->kf_cap && s->in_db[id]) {
        sv_bow_db_erase(s->db, &s->dbk[id]->k);
        s->in_db[id] = 0;
    }
}

/* ------------------------------------------------------------------ */
/* frame statistics (data::frame_statistics)                          */
/* ------------------------------------------------------------------ */
static sv_sys_fstat* fs_get(sv_system* s, unsigned int frame_id) {
    if (frame_id >= s->fs_cap) {
        unsigned int nc = s->fs_cap ? s->fs_cap : 1024;
        while (nc <= frame_id) {
            nc *= 2;
        }
        s->fs = (sv_sys_fstat*)realloc(s->fs, nc * sizeof(sv_sys_fstat));
        memset(s->fs + s->fs_cap, 0, (nc - s->fs_cap) * sizeof(sv_sys_fstat));
        s->fs_cap = nc;
    }
    return &s->fs[frame_id];
}

/* frame_statistics::update_frame_statistics(frm, is_lost) */
static void fs_update(sv_system* s, const sv_tr_frame* f, int is_lost) {
    sv_sys_fstat* e = fs_get(s, f->id);
    e->known = 1;
    if (f->pose_valid) {
        const sv_tr_kf* ref = sv_tr_map_kf_any(&s->map, f->ref_kf);
        if (ref) {
            sv_mat4_mul(f->pose_cw, ref->pose_wc, e->rel);
            e->valid = 1;
            e->ref = f->ref_kf;
            e->map = (unsigned int)f->ref_kf < s->kf_cap ? s->kf_label[f->ref_kf] : s->map_id;
            e->ts = f->timestamp;
        }
    }
    e->lost = (unsigned char)(is_lost != 0);
}

/* frame_statistics::replace_reference_keyframe(old, new) */
static void fs_replace_ref(sv_system* s, unsigned int old_id, int new_id) {
    unsigned int i;
    const sv_tr_kf *ok, *nk;
    double new_inv[16];
    if (new_id < 0) {
        return;
    }
    ok = sv_tr_map_kf_any(&s->map, (int)old_id);
    nk = sv_tr_map_kf_any(&s->map, new_id);
    if (!ok || !nk) {
        return;
    }
    mat4_inverse(nk->pose_cw, new_inv);
    for (i = 0; i < s->fs_cap; ++i) {
        if (s->fs[i].valid && s->fs[i].ref == (int)old_id) {
            double t[16], nr[16];
            sv_mat4_mul(s->fs[i].rel, ok->pose_cw, t);
            sv_mat4_mul(t, new_inv, nr);
            memcpy(s->fs[i].rel, nr, sizeof(nr));
            s->fs[i].ref = new_id;
        }
    }
}

/* ------------------------------------------------------------------ */
/* hooks: keyframe protection / erasure                                */
/* ------------------------------------------------------------------ */
static int hook_is_protected(void* user, unsigned int kf_id) {
    sv_system* s = (sv_system*)user;
    const unsigned int* ids;
    /* keyframe::cannot_be_erased_: raised by graph_node::add_loop_edge; the flags raised during a global step
     * (current keyframe, validated candidates) are lowered again by set_to_be_erased() unless a loop edge exists */
    return sv_loop_get_loop_edges(&s->lp, kf_id, &ids) > 0;
}

static void hook_on_erase(void* user, unsigned int kf_id, int parent_id) {
    sv_system* s = (sv_system*)user;
    fs_replace_ref(s, kf_id, parent_id); /* map_db->replace_reference_keyframe(kf, spanning parent) */
    db_erase(s, kf_id);                   /* bow_db->erase_keyframe(kf) */
    s->erased_pending[kf_id] = 1;
    s->kf_erased_frame[kf_id] = (int)s->cur_frame;
    s->stats.erased_keyframes++;
}

/* tracker_->replace_landmarks_in_last_frm(replaced_lms) */
static void hook_replaced(void* user, const int* pairs, unsigned int n) {
    sv_system* s = (sv_system*)user;
    sv_tr_frame* last = &s->trk.last_frm;
    unsigned int idx, k, j;
    if (!s->trk.last_frm_valid) {
        return;
    }
    for (idx = 0; idx < last->obs->num_kp; ++idx) {
        int lm = last->lm[idx];
        int to = -1;
        if (lm < 0) {
            continue;
        }
        for (k = 0; k < n; ++k) {
            if (pairs[2 * k] == lm) {
                to = pairs[2 * k + 1];
                break;
            }
        }
        if (to < 0) {
            continue;
        }
        for (j = 0; j < last->obs->num_kp; ++j) { /* has_landmark(replaced_lm) -> erase_landmark(replaced_lm) */
            if (last->lm[j] == to) {
                last->lm[j] = SV_TR_NONE;
                break;
            }
        }
        last->lm[idx] = to; /* add_landmark(replaced_lm, idx) */
    }
}

/* solve::pnp_solver(valid_bearings, octaves, valid_points, scale_factors, 10, true).find_via_ransac(30, false) */
static int hook_pnp(void* user, const double* b, const double* p, const int* oct, unsigned int n, sv_loop_pnp* out) {
    sv_system* s = (sv_system*)user;
    sv_pnp_result r;
    unsigned int i, ni = 0;
    int rc;
    if (n + 1 > s->pnp_cap) {
        s->pnp_cap = n + 64;
        s->pnp_mask = (unsigned char*)realloc(s->pnp_mask, s->pnp_cap);
        s->pnp_inl = (unsigned int*)realloc(s->pnp_inl, s->pnp_cap * sizeof(unsigned int));
    }
    memset(&r, 0, sizeof(r));
    if (s->p.pnp_lo) {
        rc = sv_pnp_lo_ransac(b, p, oct, n, s->cfg.scale_factors, s->cfg.num_levels, 10, 1000, NULL, &r, s->pnp_mask);
    }
    else {
        rc = sv_pnp_ransac(b, p, oct, n, s->cfg.scale_factors, s->cfg.num_levels, 10, 30, 10, 0, NULL, &r, s->pnp_mask, NULL, NULL);
    }
    if (rc != 0) {
        return -1;
    }
    memset(out, 0, sizeof(*out));
    out->valid = r.valid;
    if (r.valid) {
        int rr, cc;
        for (rr = 0; rr < 3; ++rr) {
            for (cc = 0; cc < 3; ++cc) {
                out->pose_rm[rr * 4 + cc] = r.rotation[rr + 3 * cc];
            }
            out->pose_rm[rr * 4 + 3] = r.translation[rr];
        }
        out->pose_rm[12] = 0.0;
        out->pose_rm[13] = 0.0;
        out->pose_rm[14] = 0.0;
        out->pose_rm[15] = 1.0;
        for (i = 0; i < n; ++i) {
            if (s->pnp_mask[i]) {
                s->pnp_inl[ni++] = i;
            }
        }
        out->n_inliers = ni;
        out->inliers = s->pnp_inl;
    }
    return 0;
}

static int hook_reloc(void* user, sv_tracker* t, const sv_tr_map* map) {
    sv_system* s = (sv_system*)user;
    return sv_reloc_tracking_glue(&s->rcfg, t, map, s->db, NULL, NULL, NULL);
}

/* loop detector's view of the persistent BoW database: the entry of a registered keyframe, else NULL */
static const sv_bow_db_keyframe* hook_db_entry(void* user, unsigned int id) {
    const sv_system* s = (const sv_system*)user;
    return (id < s->kf_cap && s->in_db[id]) ? &s->dbk[id]->k : NULL;
}

/* sv_loop merged the tree of `cur_id` into the tree of `cand_id`: unite the map labels, rescale the tracker's motion state (new map units ->
 * old map units) and let the mapper insert keyframes again */
static void hook_merge(void* user, unsigned int cur_id, unsigned int cand_id, double f) {
    sv_system* s = (sv_system*)user;
    sv_tracker* t = &s->trk;
    const int lc = label_root(s, s->kf_label[cur_id]), lo = label_root(s, s->kf_label[cand_id]);
    int k;
    if (lc != lo) {
        label_ensure(s, lc > lo ? lc : lo);
        s->map_root[lc] = lo;
    }
    for (k = 12; k < 15; ++k) {
        t->twist[k] *= f;
        t->last_cam_pose_from_ref_keyfrm[k] *= f;
    }
    for (k = 0; k < 3; ++k) {
        t->vel_w[k] *= f;
    }
    s->map.kf_floor = 0;
    s->stats.merges++;
}

static void wire_hooks(sv_system* s) {
    s->lp.merge_maps = s->p.merge_maps;
    s->lp.merge_hook = hook_merge;
    s->lp.merge_user = s;
    s->mp.alloc_lm = alloc_lm_hook;
    s->mp.alloc_user = s;
    s->mp.is_protected = hook_is_protected;
    s->mp.on_erase = hook_on_erase;
    s->mp.hook_user = s;
    s->lp.pnp_ransac = hook_pnp;
    s->lp.pnp_ransac_user = s;
    s->lp.replaced_hook = hook_replaced;
    s->lp.replaced_user = s;
    s->lp.ext_db = s->db;
    s->lp.ext_dbk = hook_db_entry;
    s->lp.ext_db_user = s;
    s->trk.reloc_hook = hook_reloc;
    s->trk.reloc_user = s;
}

/* ------------------------------------------------------------------ */
/* create / destroy                                                   */
/* ------------------------------------------------------------------ */
sv_system* sv_system_create(const sv_system_params* p) {
    sv_system* s = (sv_system*)calloc(1, sizeof(sv_system));
    if (!s) {
        return NULL;
    }
    s->p = *p;
    sv_compute_image_bounds(&p->cam, p->cols, p->rows, &s->bounds);
    sv_tr_config_init(&s->cfg, p->cam.fx, p->cam.fy, p->cam.cx, p->cam.cy, &s->bounds, p->vocab);
    s->cam.fx = p->cam.fx;
    s->cam.fy = p->cam.fy;
    s->cam.cx = p->cam.cx;
    s->cam.cy = p->cam.cy;
    s->cam.focal_x_baseline = 0.0;
    s->cam.min_x = s->bounds.min_x;
    s->cam.max_x = s->bounds.max_x;
    s->cam.min_y = s->bounds.min_y;
    s->cam.max_y = s->bounds.max_y;
    s->map.last_inserted_kf = SV_TR_NONE;
    s->last_ins_kf = SV_TR_NONE;
    s->next_label = 1;
    ensure_kf_capacity(s, 64);
    ensure_lm_capacity(s, 4096);
    sv_mapping_init(&s->mp, &s->cfg, &s->map);
    sv_loop_init(&s->lp, &s->cfg, &s->mp);
    sv_tracker_init(&s->trk, &s->cfg);
    s->trk.imu = p->imu;
    s->trk.gyro_mode = p->gyro_mode;
    s->trk.imu_toff = p->imu_toff;
    memcpy(s->trk.R_BC, p->imu_R_BC, sizeof(s->trk.R_BC));
    memcpy(s->trk.bg, p->imu_bg, sizeof(s->trk.bg));
    s->trk.lost_max_sec = p->dr_max_sec;
    s->trk.keep_good = p->rframe;
    sv_reloc_config_init(&s->rcfg);
    s->rcfg.pnp_lo = p->pnp_lo;
    s->db = sv_bow_db_create();
    s->tmp_kp = (sv_keypoint*)malloc(SV_SYS_MAX_KP * sizeof(sv_keypoint));
    s->tmp_desc = (uint8_t*)malloc((size_t)SV_SYS_MAX_KP * 32);
    mat4_identity(s->last_initial_pose);
    s->last_decision.min_interval_elapsed = 1;
    s->last_decision.min_distance_traveled = 1;
    s->last_decision.distance_traveled = -1.0f;
    s->last_path = SV_TR_PATH_NONE;
    wire_hooks(s);
    return s;
}

void sv_system_destroy(sv_system* s) {
    unsigned int i;
    if (!s) {
        return;
    }
    sv_tracker_free(&s->trk);
    sv_loop_free(&s->lp);
    sv_mapping_free(&s->mp);
    sv_bow_db_destroy(s->db);
    for (i = 0; i < s->kf_cap; ++i) {
        if (s->kf_rec[i]) {
            kf_record_reset(s->kf_rec[i]);
            free(s->kf_rec[i]);
        }
        if (s->dbk[i]) {
            if (s->dbk[i]->bow_ready) {
                sv_bow_vector_free(&s->dbk[i]->bow);
            }
            free(s->dbk[i]);
        }
    }
    for (i = 0; i < s->lm_cap; ++i) {
        if (s->lm_rec[i]) {
            free(s->lm_rec[i]->obs_kf);
            free(s->lm_rec[i]->obs_idx);
            free(s->lm_rec[i]);
        }
    }
    for (i = 0; i < s->n_all_fd; ++i) {
        fd_destroy(s->all_fd[i]);
    }
    if (s->fd_cur && !fd_in_all(s, s->fd_cur)) {
        fd_destroy(s->fd_cur);
    }
    if (s->fd_prev && s->fd_prev != s->fd_cur && !fd_in_all(s, s->fd_prev)) {
        fd_destroy(s->fd_prev);
    }
    if (s->fd_good && s->fd_good != s->fd_cur && s->fd_good != s->fd_prev && !fd_in_all(s, s->fd_good)) {
        fd_destroy(s->fd_good);
    }
    free(s->up_sum);
    free(s->up_n);
    free(s->all_fd);
    free(s->kf_rec);
    free(s->lm_rec);
    free(s->kf_fd);
    free(s->map.kfs);
    free(s->map.lms);
    free(s->dbk);
    free(s->in_db);
    free(s->erased_pending);
    free(s->kf_erased_frame);
    free(s->kf_destroyed_frame);
    free(s->fs);
    free(s->prev_x);
    free(s->prev_y);
    free(s->tmp_kp);
    free(s->tmp_desc);
    free(s->pnp_mask);
    free(s->pnp_inl);
    free(s->arch);
    free(s->kf_label);
    free(s->r_hist);
    free(s->map_root);
    free(s);
}

/* ------------------------------------------------------------------ */
/* keyframe lifetime                                                  */
/* ------------------------------------------------------------------ */
/* An erased keyframe object stays alive while a shared_ptr refers to it. The holders outside the mapping module
 * are the loop detector's continuity sets (HANDOVER module 7 "Retention"): destroyed at the first loop-detector
 * step after the erasure whose continuity sets no longer contain it. */
static void update_lifetimes(sv_system* s) {
    unsigned int id, i, j;
    for (id = 0; id < s->kf_cap; ++id) {
        int held = 0;
        if (!s->erased_pending[id]) {
            continue;
        }
        for (i = 0; i < s->lp.n_prev && !held; ++i) {
            const sv_loop_set* set = &s->lp.prev[i];
            if (set->lead == id) {
                held = 1;
                break;
            }
            for (j = 0; j < set->n; ++j) {
                if (set->ids[j] == id) {
                    held = 1;
                    break;
                }
            }
        }
        if (!held) {
            s->erased_pending[id] = 0;
            sv_mapping_set_expired(&s->mp, id, 1);
            s->kf_destroyed_frame[id] = (int)s->cur_frame;
            s->stats.destroyed_keyframes++;
        }
    }
}

/* ------------------------------------------------------------------ */
/* mapping / global optimization                                      */
/* ------------------------------------------------------------------ */
static void set_last_inserted(sv_system* s, unsigned int id) {
    s->last_ins_kf = (int)id;
}

/* the map database's last_inserted_keyfrm_ view for keyframe_inserter (its trans_wc is read live) */
static void refresh_last_inserted(sv_system* s) {
    sv_tr_map* m = &s->map;
    const sv_tr_kf* k = s->last_ins_kf >= 0 ? sv_tr_map_kf_any(m, s->last_ins_kf) : NULL;
    if (!k) {
        m->last_inserted_kf = SV_TR_NONE;
        return;
    }
    m->last_inserted_kf = (int)k->id;
    m->last_inserted_timestamp = k->timestamp;
    m->last_inserted_trans_wc[0] = k->trans_wc[0];
    m->last_inserted_trans_wc[1] = k->trans_wc[1];
    m->last_inserted_trans_wc[2] = k->trans_wc[2];
}

/* mapping_module::run_step for one queued keyframe: mapping_with_new_keyframe, then hand it to the global
 * optimization module unless it is a spanning root */
static int mapping_pass(sv_system* s, unsigned int kf_id) {
    sv_mapping_trace tr;
    sv_tr_kf* kf;
    int rc = sv_mapping_step(&s->mp, kf_id, &tr);
    sv_mapping_trace_free(&tr);
    if (rc != 0) {
        return rc;
    }
    set_last_inserted(s, kf_id); /* map_db_->add_keyframe in store_new_keyframe */
    kf = s->kf_rec[kf_id];
    if (s->p.enable_loop_closure && !kf->is_root && s->n_gq < 8) {
        s->gq[s->n_gq++] = kf_id;
    }
    return 0;
}

/* global_optimization_module::run_step for one keyframe */
static void global_step(sv_system* s, unsigned int cur_id) {
    sv_loop* L = &s->lp;
    unsigned int* db_ids;
    unsigned int n_db = 0, id;
    int detected, validated = 0;
    db_ids = (unsigned int*)malloc((s->kf_cap + 1) * sizeof(unsigned int));
    for (id = 0; id < s->kf_cap; ++id) {
        if (s->in_db[id]) {
            db_ids[n_db++] = id;
        }
    }
    s->cur_global_steps++;
    s->stats.global_steps++;
    detected = sv_loop_detect(L, cur_id, db_ids, n_db);
    free(db_ids);
    db_add(s, cur_id); /* loop_detector::detect_loop_candidates(): register to the BoW database */
    if (detected) {
        validated = sv_loop_validate(L, cur_id);
    }
    if (validated) {
        if (sv_loop_correct(L, cur_id) == 0) {
            s->cur_loop_accepted = 1;
            s->cur_loop_cur = (int)cur_id;
            s->cur_loop_cand = L->selected;
            s->stats.loops_accepted++;
            /* mapper_->async_pause(): pause_is_requested_ stays set (see sv_system_params) */
            if (!s->p.resume_mapper_after_loop && !L->merged_last) { /* stella_vio: a map merge resumes the mapper */
                s->trk.mapper_paused = 1;
            }
        }
    }
    update_lifetimes(s);
}

/* system::synchronize_background_modules() */
static void synchronize(sv_system* s) {
    unsigned int i;
    for (i = 0; i < s->n_gq; ++i) {
        global_step(s, s->gq[i]);
    }
    s->n_gq = 0;
}

/* ------------------------------------------------------------------ */
/* reset                                                              */
/* ------------------------------------------------------------------ */
/* tracking_module::reset(): initializer, keyframe inserter, mapper, global optimizer, BoW database and map
 * database are reset. (Not reachable in the deterministic reference: mapper_->async_reset() would wait forever
 * for a mapping thread that never runs. Ported per source, unvalidated.) */
static void archive_current_map(sv_system* s);
static void sys_reset(sv_system* s) {
    unsigned int i, np = s->lp.n_prev;
    sv_loop_set* keep = NULL;
    unsigned int n_keep = 0;
    /* cont_detected_keyfrm_sets_ survives a reset upstream (loop_detector is never reset) */
    if (np) {
        keep = (sv_loop_set*)calloc(np, sizeof(sv_loop_set));
        for (i = 0; i < np; ++i) {
            keep[i] = s->lp.prev[i];
            keep[i].ids = (unsigned int*)malloc((s->lp.prev[i].n ? s->lp.prev[i].n : 1) * sizeof(unsigned int));
            memcpy(keep[i].ids, s->lp.prev[i].ids, s->lp.prev[i].n * sizeof(unsigned int));
        }
        n_keep = np;
    }
    /* initializer */
    if (s->init_fd) {
        sv_sys_fd* f = s->init_fd;
        s->init_fd = NULL;
        fd_release_if_unused(s, f);
    }
    s->init_state = 0;
    s->init_ok_run = 0;
    s->init_frm_stamp = 0.0;
    if (s->trk.good_valid) { /* stella_vio: the last good frame belongs to the dropped map */
        sv_sys_fd* g = s->fd_good;
        sv_tr_frame_free(&s->trk.good_frm);
        s->trk.good_valid = 0;
        s->fd_good = NULL;
        fd_release_if_unused(s, g);
    }
    s->r_on = 0;
    /* mapper + global optimizer + map database + BoW database */
    sv_mapping_free(&s->mp);
    sv_mapping_init(&s->mp, &s->cfg, &s->map);
    sv_loop_free(&s->lp);
    sv_loop_init(&s->lp, &s->cfg, &s->mp);
    for (i = 0; i < n_keep; ++i) {
        sv_loop_add_prev(&s->lp, keep[i].lead, keep[i].continuity, keep[i].ids, keep[i].n);
        free(keep[i].ids);
    }
    free(keep);
    for (i = 0; i < s->kf_cap; ++i) {
        s->map.kfs[i] = NULL;
        if (s->kf_rec[i]) {
            s->kf_rec[i]->alive = 0;
        }
        s->in_db[i] = 0;
        s->erased_pending[i] = 0;
        s->kf_erased_frame[i] = -1;
        s->kf_destroyed_frame[i] = -1;
        if (s->dbk[i] && s->dbk[i]->bow_ready) {
            sv_bow_vector_free(&s->dbk[i]->bow);
            s->dbk[i]->bow_ready = 0;
        }
    }
    for (i = 0; i < s->lm_cap; ++i) {
        s->map.lms[i] = NULL;
        if (s->lm_rec[i]) {
            s->lm_rec[i]->alive = 0;
        }
    }
    sv_bow_db_clear(s->db);
    memset(s->fs, 0, s->fs_cap * sizeof(sv_sys_fstat)); /* frm_stats_.clear() */
    s->n_gq = 0;
    s->map.num_keyframes = 0;
    s->map.kf_floor = 0;
    s->map.last_inserted_kf = SV_TR_NONE;
    s->last_ins_kf = SV_TR_NONE;
    s->next_keyframe_id = 0;
    s->lp.prev_loop_correct_keyfrm_id = 0;
    s->trk.mapper_paused = 0; /* fresh mapper state */
    s->trk.last_reloc_frm_id = 0;
    s->trk.last_reloc_frm_timestamp = 0.0;
    s->trk.tracking_state = 0;
    s->stats.resets++;
    s->cur_reset = 1;
    wire_hooks(s);
}

/* stella_vio merge_maps: leave the current map in the structure (keyframes, landmarks, BoW database, loop and mapping state, frame
 * statistics) and start a new, unconnected map next to it. Only the tracker / initializer state is reset. */
static void sys_soft_reset(sv_system* s) {
    if (s->init_fd) {
        sv_sys_fd* f = s->init_fd;
        s->init_fd = NULL;
        fd_release_if_unused(s, f);
    }
    s->init_state = 0;
    s->init_ok_run = 0;
    s->init_frm_stamp = 0.0;
    if (s->trk.good_valid) {
        sv_sys_fd* g = s->fd_good;
        sv_tr_frame_free(&s->trk.good_frm);
        s->trk.good_valid = 0;
        s->fd_good = NULL;
        fd_release_if_unused(s, g);
    }
    s->n_gq = 0;
    s->r_on = 0;
    s->trk.mapper_paused = 0;
    s->trk.last_reloc_frm_id = 0;
    s->trk.last_reloc_frm_timestamp = 0.0;
    s->trk.tracking_state = 0;
    s->trk.twist_valid = 0;
    s->trk.vel_valid = 0;
    s->last_ins_kf = SV_TR_NONE;
    s->map.last_inserted_kf = SV_TR_NONE;
    s->map.kf_floor = s->map.num_keyframes; /* the next map is young again */
    s->n_soft++;
    s->stats.resets++;
    s->cur_reset = 1;
}

/* the reset the baseline does at "lost shortly after initialization" / "wrong initialization": with kept old maps (merge_maps) only the
 * tracker side is reset, else everything (exact port) */
static void reset_current(sv_system* s) {
    if (s->p.merge_maps && s->n_soft > 0) {
        sys_soft_reset(s);
    }
    else {
        sys_reset(s);
    }
}


/* ------------------------------------------------------------------ */
/* R-frames: the rotation chain                                       */
/* ------------------------------------------------------------------ */
static void rhist_push(sv_system* s, unsigned int id, double ts, const double Rcw[9], const double c[3]) {
    sv_rhist* h;
    if (s->r_nh == s->r_caph) {
        s->r_caph = s->r_caph ? s->r_caph * 2 : 256;
        s->r_hist = (sv_rhist*)realloc(s->r_hist, s->r_caph * sizeof(sv_rhist));
    }
    h = &s->r_hist[s->r_nh++];
    h->id = id;
    h->ts = ts;
    memcpy(h->Rcw, Rcw, 9 * sizeof(double));
    memcpy(h->c, c, 3 * sizeof(double));
}

static const sv_rhist* rhist_find(const sv_system* s, unsigned int id) {
    unsigned int i;
    for (i = s->r_nh; i > 0; --i) {
        if (s->r_hist[i - 1].id == id) {
            return &s->r_hist[i - 1];
        }
    }
    return NULL;
}

/* column-major 4x4 pose_cw from R_cw (row-major) and the centre */
static void pose_from_chain(const double Rcw[9], const double c[3], double pose[16]) {
    int r, k;
    memset(pose, 0, 16 * sizeof(double));
    for (k = 0; k < 3; ++k) {
        for (r = 0; r < 3; ++r) {
            pose[k * 4 + r] = Rcw[r * 3 + k];
        }
    }
    for (r = 0; r < 3; ++r) {
        pose[12 + r] = -(Rcw[r * 3 + 0] * c[0] + Rcw[r * 3 + 1] * c[1] + Rcw[r * 3 + 2] * c[2]);
    }
    pose[15] = 1.0;
}

/* the chain ends: `failed` = it gave up (else the map took over again / the bridge was built) */
static void rend(sv_system* s, int failed) {
    if (!s->r_on) {
        return;
    }
    s->r_on = 0;
    if (failed) {
        s->stats.rfail++;
    }
    if (s->init_fd) { /* the deferred initialization belongs to the chain */
        sv_sys_fd* f = s->init_fd;
        s->init_fd = NULL;
        f->keep = 0;
        fd_release_if_unused(s, f);
    }
    s->init_state = 0;
    s->init_ok_run = 0;
}

/* tracking failed in state Tracking: start the chain from the last good frame */
static int rstart(sv_system* s) {
    const sv_tracker* t = &s->trk;
    const sv_tr_frame* L = &t->last_frm;
    int r, k;
    if (!t->last_frm_valid || !L->pose_valid || L->ref_kf < 0 || !sv_tr_map_kf_any(&s->map, L->ref_kf) || !s->fd_prev ||
        s->fd_prev->id != L->id) {
        return 0;
    }
    for (r = 0; r < 3; ++r) {
        for (k = 0; k < 3; ++k) {
            s->r_Rcw[r * 3 + k] = L->rot_cw[k * 3 + r];
            s->r_dR[r * 3 + k] = (r == k) ? 1.0 : 0.0;
            if (t->twist_valid) {
                s->r_dR[r * 3 + k] = t->twist[k * 4 + r]; /* rotation of the last motion: constant angular velocity */
            }
        }
        s->r_c[r] = L->trans_wc[r];
        s->r_vel[r] = s->ema_valid ? s->ema_vel[r] : ((t->vel_valid && t->vel_dt > 0.0) ? t->vel_w[r] / t->vel_dt : 0.0);
    }
    s->r_anchor = L->ref_kf;
    s->r_t0 = L->timestamp;
    s->r_last_ts = L->timestamp;
    s->r_gyro_run = 0;
    s->r_nh = 0;
    rhist_push(s, L->id, L->timestamp, s->r_Rcw, s->r_c);
    s->r_on = 1;
    return 1;
}

/* extend the chain by frame `fd` (previous frame = fd_prev): vision rotation (+ gyro). Returns 1 and the pose of fd, 0 = the chain ended. */
static int rstep(sv_system* s, const sv_sys_fd* fd, int* gyro_only) {
    const sv_sys_fd* pv = s->fd_prev;
    double Rpred[9], Rab[9], Rn[9], mid, fade;
    int have_gyro = 0, vis, k;
    sv_rot_result rr;
    *gyro_only = 0;
    if (!pv || pv->id + 1 != fd->id || fd->ts - s->r_t0 > s->p.rframe_max_sec) {
        rend(s, 1);
        return 0;
    }
    memcpy(Rpred, s->r_dR, sizeof(Rpred));
    if (s->p.imu && s->p.rframe_gyro_max > 0) {
        double Rc[9];
        const int64_t a = (int64_t)llround((pv->ts + s->p.imu_toff) * 1e9), b = (int64_t)llround((fd->ts + s->p.imu_toff) * 1e9);
        if (sv_imu_gyro_predict_cam(s->p.imu, a, b, s->p.imu_bg, s->p.imu_R_BC, Rc) >= 0) {
            int r;
            for (r = 0; r < 3; ++r) { /* R_ab = R_C0C1^T */
                for (k = 0; k < 3; ++k) {
                    Rpred[r * 3 + k] = Rc[k * 3 + r];
                }
            }
            have_gyro = 1;
        }
    }
    vis = sv_rot_estimate(&s->cfg, &pv->obs, &fd->obs, Rpred, have_gyro ? 40.0 : 90.0, 20, &rr);
    if (vis && have_gyro) { /* weak vision that contradicts the gyro loses */
        double tr = 0.0, ang;
        for (k = 0; k < 9; ++k) {
            tr += Rpred[k] * rr.R_ab[k];
        }
        ang = acos((tr - 1.0) > 2.0 ? 1.0 : ((tr - 1.0) < -2.0 ? -1.0 : (tr - 1.0) * 0.5));
        if (rr.n_inlier < 40 && ang > 0.17) {
            vis = 0;
        }
    }
    s->r_last_inl = vis ? rr.n_inlier : 0;
    s->r_last_par = vis ? rr.parallax_med * 57.29577951308232 : 0.0;
    if (vis) {
        memcpy(Rab, rr.R_ab, sizeof(Rab));
        s->r_gyro_run = 0;
    }
    else if (have_gyro && s->r_gyro_run < s->p.rframe_gyro_max) {
        memcpy(Rab, Rpred, sizeof(Rab));
        s->r_gyro_run++;
        *gyro_only = 1;
    }
    else {
        rend(s, 1);
        return 0;
    }
    sv_m3_mul(Rab, s->r_Rcw, Rn); /* R_cw(B) = R_ab R_cw(A) */
    memcpy(s->r_Rcw, Rn, sizeof(Rn));
    memcpy(s->r_dR, Rab, sizeof(Rab));
    mid = 0.5 * (s->r_last_ts + fd->ts);
    fade = s->p.rframe_hold_sec > 0.0 ? 1.0 - (mid - s->r_t0) / s->p.rframe_hold_sec : 0.0;
    fade = fade < 0.0 ? 0.0 : fade;
    for (k = 0; k < 3; ++k) {
        s->r_c[k] += s->r_vel[k] * (fd->ts - s->r_last_ts) * fade;
    }
    s->r_last_ts = fd->ts;
    rhist_push(s, fd->id, fd->ts, s->r_Rcw, s->r_c);
    return 1;
}

/* The deferred initialization succeeded in `imap` (K0 = identity, median depth 1): move the new map into the frame of the existing map.
 * K0 sits where the rotation chain placed the reference frame; the scale comes from the priors of rframe_scale. Returns 0 on failure. */
static int bridge_transform(sv_system* s, sv_map_init_map* imap, const sv_sys_fd* ref, const sv_sys_fd* cur, double* sigma_out) {
    const sv_rhist* h = rhist_find(s, ref->id);
    const sv_tr_kf* anchor = sv_tr_map_kf(&s->map, s->r_anchor);
    double sigma, depth_prior = 0.0, speed_prior = 0.0, base_new, vnorm;
    double Rwc[9], p0[16], p1[16], R1[9], R1n[9], t1[3], tn[3];
    unsigned int i;
    int r, k;
    if (!h || !anchor) {
        return 0;
    }
    depth_prior = (double)sv_map_kf_median_depth(&s->map, anchor, 1);
    vnorm = sqrt(s->r_vel[0] * s->r_vel[0] + s->r_vel[1] * s->r_vel[1] + s->r_vel[2] * s->r_vel[2]);
    for (r = 0; r < 3; ++r) {
        for (k = 0; k < 3; ++k) {
            R1[r * 3 + k] = imap->curr_keyfrm.pose_cw[k * 4 + r]; /* column-major -> row-major */
        }
    }
    t1[0] = imap->curr_keyfrm.pose_cw[12];
    t1[1] = imap->curr_keyfrm.pose_cw[13];
    t1[2] = imap->curr_keyfrm.pose_cw[14];
    base_new = sqrt(t1[0] * t1[0] + t1[1] * t1[1] + t1[2] * t1[2]);
    if (vnorm > 1e-6 && base_new > 1e-9) {
        speed_prior = vnorm * (cur->ts - ref->ts) / base_new;
    }
    if (s->p.rframe_scale == 1 && speed_prior > 0.0) {
        sigma = speed_prior;
    }
    else if (s->p.rframe_scale == 2 && speed_prior > 0.0 && depth_prior > 0.0) {
        sigma = sqrt(speed_prior * depth_prior);
    }
    else {
        sigma = depth_prior;
    }
    if (!(sigma > 1e-6) || !(sigma < 1e6)) {
        return 0;
    }
    /* K0 = (Rcw_ref, -Rcw_ref c) ; K1 = (R1 Rcw_ref, sigma t1 - R1 Rcw_ref c) ; p' = Rwc_ref (sigma p) + c */
    for (r = 0; r < 3; ++r) {
        for (k = 0; k < 3; ++k) {
            Rwc[r * 3 + k] = h->Rcw[k * 3 + r];
        }
    }
    pose_from_chain(h->Rcw, h->c, p0);
    sv_m3_mul(R1, h->Rcw, R1n);
    for (r = 0; r < 3; ++r) {
        tn[r] = sigma * t1[r] - (R1n[r * 3 + 0] * h->c[0] + R1n[r * 3 + 1] * h->c[1] + R1n[r * 3 + 2] * h->c[2]);
    }
    memset(p1, 0, sizeof(p1));
    for (k = 0; k < 3; ++k) {
        for (r = 0; r < 3; ++r) {
            p1[k * 4 + r] = R1n[r * 3 + k];
        }
    }
    p1[12] = tn[0];
    p1[13] = tn[1];
    p1[14] = tn[2];
    p1[15] = 1.0;
    for (i = 0; i < imap->num_landmarks; ++i) {
        double q[3], w[3];
        for (k = 0; k < 3; ++k) {
            q[k] = sigma * imap->landmarks[i].pos_w[k];
        }
        sv_m3_mulv(Rwc, q, w);
        for (k = 0; k < 3; ++k) {
            imap->landmarks[i].pos_w[k] = w[k] + h->c[k];
        }
    }
    sv_map_keyframe_set_pose_cw(&imap->init_keyfrm, p0);
    sv_map_keyframe_set_pose_cw(&imap->curr_keyfrm, p1);
    *sigma_out = sigma;
    return 1;
}

/* ------------------------------------------------------------------ */
/* initialization                                                     */
/* ------------------------------------------------------------------ */
/* k0 = id of the first of the two initial keyframes (0 after a reset; later when old maps are kept), the landmark ids and the observation
 * keyframe ids of `map` are already offset by the caller */
static void install_init_map(sv_system* s, const sv_map_init_map* map, const sv_sys_fd* ref, const sv_sys_fd* cur, unsigned int k0) {
    unsigned int i, k;
    sv_tr_kf* kf[2];
    const sv_map_keyframe* mk[2];
    const sv_sys_fd* fds[2];
    fds[0] = ref;
    fds[1] = cur;
    mk[0] = &map->init_keyfrm;
    mk[1] = &map->curr_keyfrm;
    ensure_kf_capacity(s, k0 + 2);
    ensure_lm_capacity(s, s->mp.next_landmark_id + map->num_landmarks + 1);
    for (k = 0; k < 2; ++k) {
        kf[k] = kf_record(s, k0 + k);
        kf_record_reset(kf[k]);
        kf[k]->id = k0 + k;
        s->kf_label[k0 + k] = s->map_id;
        kf[k]->alive = 1;
        kf[k]->timestamp = fds[k]->ts;
        kf[k]->obs = (sv_tr_obs*)&fds[k]->obs;
        sv_tr_kf_set_pose_cw(kf[k], mk[k]->pose_cw);
        kf[k]->lm = (int*)malloc((fds[k]->obs.num_kp ? fds[k]->obs.num_kp : 1) * sizeof(int));
        for (i = 0; i < fds[k]->obs.num_kp; ++i) {
            kf[k]->lm[i] = SV_TR_NONE;
        }
        s->kf_fd[k0 + k] = (sv_sys_fd*)fds[k];
        s->map.kfs[k0 + k] = kf[k];
    }
    kf[0]->parent = SV_TR_NONE;
    kf[0]->is_root = 1;
    kf[0]->n_children = 1;
    kf[0]->children = (unsigned int*)malloc(sizeof(unsigned int));
    kf[0]->children[0] = k0 + 1;
    kf[1]->parent = (int)k0;
    kf[1]->is_root = 0;
    for (i = 0; i < map->num_landmarks; ++i) {
        const sv_map_landmark* ml = &map->landmarks[i];
        sv_tr_lm* lm = alloc_lm_hook(s, ml->id);
        unsigned int o;
        free(lm->obs_kf);
        free(lm->obs_idx);
        memset(lm, 0, sizeof(*lm));
        lm->id = ml->id;
        lm->alive = 1;
        memcpy(lm->pos_w, ml->pos_w, sizeof(lm->pos_w));
        memcpy(lm->desc, ml->descriptor, 32);
        memcpy(lm->mean_normal, ml->mean_normal, sizeof(lm->mean_normal));
        lm->min_valid_dist = ml->min_valid_dist;
        lm->max_valid_dist = ml->max_valid_dist;
        lm->num_observed = ml->num_observed;
        lm->num_observable = ml->num_observable;
        lm->ref_kf = (int)ml->ref_keyfrm_id;
        lm->num_obs = ml->num_observations;
        lm->obs_kf = (unsigned int*)malloc((lm->num_obs ? lm->num_obs : 1) * sizeof(unsigned int));
        lm->obs_idx = (unsigned int*)malloc((lm->num_obs ? lm->num_obs : 1) * sizeof(unsigned int));
        for (o = 0; o < lm->num_obs; ++o) {
            lm->obs_kf[o] = ml->observations[o].keyframe_id;
            lm->obs_idx[o] = ml->observations[o].idx;
            kf[ml->observations[o].keyframe_id - k0]->lm[ml->observations[o].idx] = (int)ml->id;
        }
        s->map.lms[ml->id] = lm;
    }
    s->map.num_keyframes += 2; /* 2 after a reset (the map was empty) */
}

/* global_bundle_adjuster::optimize_for_initialization on the pre-BA init map; the results are written back into
 * `map` exactly like the upstream application step (keyframe poses, landmark positions) */
static int init_global_ba(sv_system* s, sv_map_init_map* map, const sv_sys_fd* ref, const sv_sys_fd* cur) {
    sv_bav_view view;
    sv_bav_kf vk[2];
    sv_bav_lm* vl;
    sv_bav_result res;
    unsigned int* lm_ids;
    unsigned int nl = map->num_landmarks, i, k;
    unsigned int kids[2] = {0, 1};
    const sv_sys_fd* fds[2];
    const sv_map_keyframe* mk[2];
    unsigned int* slots[2];
    sv_bav_kp* kps[2];
    int rc;
    fds[0] = ref;
    fds[1] = cur;
    mk[0] = &map->init_keyfrm;
    mk[1] = &map->curr_keyfrm;
    memset(vk, 0, sizeof(vk));
    for (k = 0; k < 2; ++k) {
        unsigned int nkp = fds[k]->obs.num_kp, n_used = 0;
        int r, c;
        slots[k] = (unsigned int*)malloc((nkp ? nkp : 1) * sizeof(unsigned int));
        kps[k] = (sv_bav_kp*)malloc((nkp ? nkp : 1) * sizeof(sv_bav_kp));
        for (i = 0; i < nkp; ++i) {
            slots[k][i] = SV_NONE_ID;
        }
        for (i = 0; i < nl; ++i) {
            const sv_map_landmark* ml = &map->landmarks[i];
            unsigned int o;
            for (o = 0; o < ml->num_observations; ++o) {
                if (ml->observations[o].keyframe_id == k) {
                    slots[k][ml->observations[o].idx] = ml->id;
                }
            }
        }
        for (i = 0; i < nkp; ++i) {
            if (slots[k][i] != SV_NONE_ID) {
                kps[k][n_used].idx = i;
                kps[k][n_used].x = fds[k]->kp[i].x;
                kps[k][n_used].y = fds[k]->kp[i].y;
                kps[k][n_used].octave = fds[k]->kp[i].octave;
                ++n_used;
            }
        }
        vk[k].id = k;
        vk[k].erased = 0;
        vk[k].spanning_root = (k == 0);
        for (r = 0; r < 4; ++r) {
            for (c = 0; c < 4; ++c) {
                vk[k].pose_cw[r * 4 + c] = mk[k]->pose_cw[c * 4 + r];
            }
        }
        vk[k].has_slots = 1;
        vk[k].n_slots = (int)nkp;
        vk[k].slots = slots[k];
        vk[k].n_kp = (int)n_used;
        vk[k].kps = kps[k];
    }
    vl = (sv_bav_lm*)calloc(nl ? nl : 1, sizeof(sv_bav_lm));
    lm_ids = (unsigned int*)malloc((nl ? nl : 1) * sizeof(unsigned int));
    {
        unsigned int* obs_kf = (unsigned int*)malloc((nl * 2 + 2) * sizeof(unsigned int));
        unsigned int* obs_idx = (unsigned int*)malloc((nl * 2 + 2) * sizeof(unsigned int));
        for (i = 0; i < nl; ++i) {
            const sv_map_landmark* ml = &map->landmarks[i];
            unsigned int o;
            for (o = 0; o < ml->num_observations; ++o) {
                obs_kf[2 * i + o] = ml->observations[o].keyframe_id;
                obs_idx[2 * i + o] = ml->observations[o].idx;
            }
            vl[i].id = ml->id;
            vl[i].erased = 0;
            memcpy(vl[i].pos, ml->pos_w, sizeof(vl[i].pos));
            vl[i].n_obs = (int)ml->num_observations;
            vl[i].obs_kf = obs_kf + 2 * i;
            vl[i].obs_idx = obs_idx + 2 * i;
            lm_ids[i] = ml->id;
        }
        memset(&view, 0, sizeof(view));
        view.kfs = vk;
        view.n_kfs = 2;
        view.lms = vl;
        view.n_lms = (int)nl;
        view.fx = s->cfg.fx;
        view.fy = s->cfg.fy;
        view.cx = s->cfg.cx;
        view.cy = s->cfg.cy;
        view.n_isq = (int)s->cfg.num_levels;
        view.isq = s->cfg.inv_level_sigma_sq;
        view.fixed_keyframe_id_threshold = 0;
        memset(&res, 0, sizeof(res));
        /* module::initializer: num_ba_iterations 100, huber, gain_threshold (float) 1e-5 */
        rc = sv_bav_global_init(&view, kids, 2, lm_ids, (int)nl, 100, 1, (double)1e-5f, NULL, &res);
        if (rc == 0) {
            for (i = 0; i < (unsigned int)res.n_applied; ++i) {
                double col[16];
                int r, c;
                for (r = 0; r < 4; ++r) {
                    for (c = 0; c < 4; ++c) {
                        col[c * 4 + r] = res.applied_pose[i][r * 4 + c];
                    }
                }
                sv_map_keyframe_set_pose_cw(res.applied_kf[i] == 0 ? &map->init_keyfrm : &map->curr_keyfrm, col);
            }
            for (i = 0; i < (unsigned int)res.g.nv; ++i) {
                const sv_ba_vertex* v = &res.g.v[i];
                if (!v->is_landmark) {
                    continue;
                }
                if (res.vtx_owner[i] < nl) {
                    memcpy(map->landmarks[res.vtx_owner[i]].pos_w, v->pos, sizeof(v->pos));
                }
            }
        }
        sv_bav_result_free(&res);
        free(obs_kf);
        free(obs_idx);
    }
    free(vl);
    free(lm_ids);
    for (k = 0; k < 2; ++k) {
        free(slots[k]);
        free(kps[k]);
    }
    return rc;
}

/* module::initializer::initialize() for the monocular case; returns 1 iff a map was created.
 * `curr` is the tracker's curr_frm (its landmarks / pose / reference keyframe are filled on success). */
static int initialize(sv_system* s, sv_sys_fd* fd, sv_frame_result* res) {
    sv_tr_frame* curr = &s->trk.curr_frm;
    const sv_sys_fd* ref;
    unsigned int n_ref, n_cur, i, num_matches;
    int* matched;
    double *b_ref, *b_cur;
    sv_init_attempt_result ar;
    sv_init_params ip;
    double cam_matrix[9];
    const int bridge = s->p.rframe && s->r_on; /* R-frames: install the map into the existing one */

    if (s->init_state == 0) { /* create_initializer(curr_frm) */
        s->init_fd = fd;
        fd_keep(s, fd);
        fd->keep = 1;
        n_ref = fd->obs.num_kp;
        free(s->prev_x);
        free(s->prev_y);
        s->prev_x = (float*)malloc((n_ref ? n_ref : 1) * sizeof(float));
        s->prev_y = (float*)malloc((n_ref ? n_ref : 1) * sizeof(float));
        for (i = 0; i < n_ref; ++i) {
            s->prev_x[i] = fd->kp[i].x;
            s->prev_y[i] = fd->kp[i].y;
        }
        s->init_state = 1;
        return 0;
    }
    ref = s->init_fd;
    n_ref = ref->obs.num_kp;
    n_cur = fd->obs.num_kp;
    matched = (int*)malloc((n_ref ? n_ref : 1) * sizeof(int));
    sv_match_area_hamm_thr = s->p.init_hamm;
    sv_match_area_ratio = s->p.init_ratio;
    sv_match_area_max_level = s->p.init_max_level;
    num_matches = sv_match_in_consistent_area(ref->kp, n_ref, ref->desc, fd->kp, n_cur, fd->desc, &fd->obs.grid,
                                              s->prev_x, s->prev_y, 100, matched);
    if (num_matches < s->p.init_min_valid) { /* min_num_valid_pts_: reset() the initializer; the next frame becomes the reference */
        sv_sys_fd* old = s->init_fd;
        s->init_fd = NULL;
        s->init_state = 0;
        s->init_frm_stamp = 0.0;
        old->keep = 0; /* stays listed in all_fd until destroy (cheap) */
        s->init_ok_run = 0;
        free(matched);
        return 0;
    }
    b_ref = (double*)malloc((n_ref ? n_ref : 1) * 3 * sizeof(double));
    b_cur = (double*)malloc((n_cur ? n_cur : 1) * 3 * sizeof(double));
    for (i = 0; i < n_ref; ++i) {
        sv_camera_convert_point_to_bearing(&s->cam, ref->kp[i].x, ref->kp[i].y, b_ref + 3 * i);
    }
    for (i = 0; i < n_cur; ++i) {
        sv_camera_convert_point_to_bearing(&s->cam, fd->kp[i].x, fd->kp[i].y, b_cur + 3 * i);
    }
    ip.num_ransac_iters = 100;
    ip.min_num_valid_pts = s->p.init_min_valid;
    ip.min_num_triangulated_pts = s->p.init_min_tri;
    ip.parallax_deg_thr = s->p.init_parallax_deg;
    ip.reproj_err_thr = 4.0f;
    ip.par_frac = s->p.init_par_frac;
    ip.num_seeds = s->p.init_seeds;
    ip.refine = s->p.init_refine;
    ip.lo = s->p.init_lo;
    ip.lo_thr_px = s->p.init_lo_thr;
    cam_matrix[0] = s->cfg.fx; cam_matrix[1] = 0; cam_matrix[2] = 0;
    cam_matrix[3] = 0; cam_matrix[4] = s->cfg.fy; cam_matrix[5] = 0;
    cam_matrix[6] = s->cfg.cx; cam_matrix[7] = s->cfg.cy; cam_matrix[8] = 1;
    memset(&ar, 0, sizeof(ar));
    ar.matched_2_in_1 = (int*)malloc((n_ref ? n_ref : 1) * sizeof(int));
    ar.inlier_h = (unsigned char*)malloc(num_matches ? num_matches : 1);
    ar.inlier_f = (unsigned char*)malloc(num_matches ? num_matches : 1);
    ar.triangulated_pts = (double*)malloc((n_ref ? n_ref : 1) * 3 * sizeof(double));
    ar.is_triangulated = (unsigned char*)malloc(n_ref ? n_ref : 1);
    memset(ar.is_triangulated, 0, n_ref ? n_ref : 1);
    sv_init_try_monocular(ref->kp, n_ref, b_ref, fd->kp, n_cur, b_cur, matched, &s->cam, &s->cam, cam_matrix, cam_matrix,
                          &ip, &ar);
    free(b_ref);
    free(b_cur);
    if (ar.verdict == SV_INIT_SUCCESS && ++s->init_ok_run < s->p.init_confirm) { /* stella_vio: wait for more baseline */
        ar.verdict = SV_INIT_FAIL_POSE_NOT_FOUND;
        free(ar.matched_2_in_1); free(ar.inlier_h); free(ar.inlier_f); free(ar.triangulated_pts); free(ar.is_triangulated);
        free(matched);
        return 0;
    }
    if (ar.verdict != SV_INIT_SUCCESS) {
        s->init_ok_run = 0;
        free(ar.matched_2_in_1); free(ar.inlier_h); free(ar.inlier_f); free(ar.triangulated_pts); free(ar.is_triangulated);
        free(matched);
        return 0;
    }

    /* create_map_for_monocular(bow_vocab, curr_frm) */
    {
        sv_map_orb_params op;
        sv_map_init_map imap;
        sv_map_landmark* lms = (sv_map_landmark*)malloc((n_ref ? n_ref : 1) * sizeof(sv_map_landmark));
        double init_kf_pre_wc[16], cur_kf_pre_wc[16], cur_pre_cw[16], init_pre_cw[16];
        int ba_rc;
        sv_sys_fd* refw = s->init_fd;
        const unsigned int k0 = s->next_keyframe_id, lm0 = s->mp.next_landmark_id; /* 0, 0 on a fresh map */
        double bridge_sigma = 1.0;
        sv_map_orb_params_init(&op, s->cfg.scale_factor, (int)s->cfg.num_levels);
        sv_map_build_pre_ba(refw->id, fd->id, ref->kp, ref->desc, n_ref, fd->kp, fd->desc, n_cur, &op, ar.rot_ref_to_cur,
                            ar.trans_ref_to_cur, matched, ar.is_triangulated, ar.triangulated_pts, lms, &imap);
        memcpy(init_pre_cw, imap.init_keyfrm.pose_cw, sizeof(init_pre_cw));
        memcpy(cur_pre_cw, imap.curr_keyfrm.pose_cw, sizeof(cur_pre_cw));
        memcpy(init_kf_pre_wc, imap.init_keyfrm.pose_wc, sizeof(init_kf_pre_wc));
        memcpy(cur_kf_pre_wc, imap.curr_keyfrm.pose_wc, sizeof(cur_kf_pre_wc));
        /* map_db_->update_frame_statistics(init_frm_/curr_frm, false): with the pre-BA poses */
        {
            sv_sys_fstat* e0 = fs_get(s, refw->id);
            sv_sys_fstat* e1 = fs_get(s, fd->id);
            sv_mat4_mul(init_pre_cw, init_kf_pre_wc, e0->rel);
            e0->valid = 1; e0->known = 1; e0->lost = 0; e0->ref = (int)k0; e0->map = s->map_id; e0->ts = refw->ts;
            sv_mat4_mul(cur_pre_cw, cur_kf_pre_wc, e1->rel);
            e1->valid = 1; e1->known = 1; e1->lost = 0; e1->ref = (int)k0 + 1; e1->map = s->map_id; e1->ts = fd->ts;
        }
        ba_rc = init_global_ba(s, &imap, refw, fd);
        (void)ba_rc;
        sv_map_apply_post_ba(&imap, 50, 1.0);
        if (!imap.reset_wrong_init && bridge && !bridge_transform(s, &imap, refw, fd, &bridge_sigma)) {
            imap.reset_wrong_init = 1; /* no bridge: treated like a wrong initialization */
        }
        if (imap.reset_wrong_init) { /* "seems to be wrong initialization, resetting" */
            free(lms);
            free(ar.matched_2_in_1); free(ar.inlier_h); free(ar.inlier_f); free(ar.triangulated_pts); free(ar.is_triangulated);
            free(matched);
            if (bridge) { /* the existing map stays: only this attempt is dropped */
                sv_sys_fd* old = s->init_fd;
                s->init_fd = NULL;
                s->init_state = 0;
                s->init_ok_run = 0;
                old->keep = 0;
            }
            else {
                reset_current(s);
            }
            return 0;
        }
        fd_keep(s, fd);
        fd->keep = 1;
        for (i = 0; i < imap.num_landmarks; ++i) { /* offset the ids when old maps are kept (k0 = lm0 = 0 otherwise) */
            sv_map_landmark* ml = &imap.landmarks[i];
            unsigned int o;
            ml->id += lm0;
            ml->ref_keyfrm_id += k0;
            ml->first_keyfrm_id += k0;
            for (o = 0; o < ml->num_observations; ++o) {
                ml->observations[o].keyframe_id += k0;
            }
        }
        if (bridge) {
            s->map.kf_floor = s->map.num_keyframes; /* count only the keyframes of the new part for the young-map rules */
        }
        install_init_map(s, &imap, refw, fd, k0);
        s->next_keyframe_id = k0 + 2;
        if (bridge) {
            sv_tr_kf* anc = s->map.kfs[s->r_anchor];
            sv_tr_kf* k0r = s->kf_rec[k0];
            for (i = 0; i < imap.num_landmarks; ++i) { /* normals and valid distances of the moved landmarks */
                sv_tr_lm* lm = s->map.lms[imap.landmarks[i].id];
                sv_tr_lm_update_mean_normal_and_obs_scale_variance(&s->cfg, &s->map, lm);
            }
            k0r->is_root = 0;
            k0r->parent = s->r_anchor;
            anc->children = (unsigned int*)realloc(anc->children, (anc->n_children + 1) * sizeof(unsigned int));
            anc->children[anc->n_children++] = k0; /* k0 is above every existing id */
            { /* the bridged part is its own segment (scored separately) of the anchor's map */
                const int L = s->next_label++;
                const int old_label = s->kf_label[s->r_anchor];
                label_ensure(s, L);
                s->map_root[L] = label_root(s, old_label);
                s->kf_label[k0] = s->kf_label[k0 + 1] = L;
                fs_get(s, refw->id)->map = L;
                fs_get(s, fd->id)->map = L;
            }
            s->trk.mapper_paused = 0;
            s->trk.twist_valid = 0;
            s->trk.vel_valid = 0;
            s->stats.rbridges++;
            if (s->p.rframe_calib_sec > 0.0 && s->ema_valid && s->ema_speed > 0.0) { /* wait for the scale calibration of the new part */
                s->cal_on = 1;
                s->cal_k0 = k0;
                s->cal_lm0 = lm0;
                s->cal_t0 = fd->ts;
                s->cal_len = 0.0;
                s->cal_vold = s->ema_speed;
                memcpy(s->cal_c0, rhist_find(s, refw->id)->c, sizeof(s->cal_c0));
            }
            s->r_on = 0; /* the chain has been turned into keyframes */
            (void)bridge_sigma;
            for (i = 0; i < curr->obs->num_kp; ++i) { /* matches left over from the failed map-tracking attempt of this frame */
                curr->lm[i] = SV_TR_NONE;
            }
        }
        /* initial landmarks were created with the current keyframe as their first keyframe */
        for (i = 0; i < imap.num_landmarks; ++i) {
            sv_mapping_set_lm_first_kf(&s->mp, imap.landmarks[i].id, k0 + 1);
        }
        s->mp.next_landmark_id = lm0 + imap.num_landmarks;
        /* current frame: landmarks + pose of the (scaled) current keyframe */
        for (i = 0; i < imap.num_landmarks; ++i) {
            const sv_map_landmark* ml = &imap.landmarks[i];
            unsigned int o;
            for (o = 0; o < ml->num_observations; ++o) {
                if (ml->observations[o].keyframe_id == k0 + 1) {
                    curr->lm[ml->observations[o].idx] = (int)ml->id;
                }
            }
        }
        sv_tr_frame_set_pose_cw(curr, s->kf_rec[k0 + 1]->pose_cw);
        curr->ref_kf = (int)k0 + 1;
        s->init_frm_stamp = bridge ? -1e30 : fd->ts; /* a bridged map that is lost again never triggers the early-loss reset of the whole map */
        free(lms);
    }
    free(ar.matched_2_in_1); free(ar.inlier_h); free(ar.inlier_f); free(ar.triangulated_pts); free(ar.is_triangulated);
    free(matched);
    s->init_fd = NULL;
    s->init_state = 0;

    /* pass all of the keyframes to the mapping module (spanning tree BFS order: keyframe 0, keyframe 1) */
    {
        const unsigned int k0 = s->next_keyframe_id - 2;
        mapping_pass(s, k0);
        mapping_pass(s, k0 + 1);
    }
    res->initialized = 1;
    return 1;
}

/* ------------------------------------------------------------------ */
/* feed_frame                                                         */
/* ------------------------------------------------------------------ */
static unsigned int count_landmarks(const sv_system* s) {
    unsigned int i, n = 0;
    for (i = 0; i < s->lm_cap; ++i) {
        if (s->map.lms[i] && s->map.lms[i]->alive) {
            ++n;
        }
    }
    return n;
}

static void fill_report(sv_system* s, sv_frame_result* res, sv_sys_fd* fd, int state_before) {
    sv_tracker* t = &s->trk;
    int r, c;
    res->tracking_state_before = state_before;
    res->tracking_state_after = t->tracking_state;
    res->track_path = s->last_path;
    res->initial_pose_valid = s->last_initial_valid;
    memcpy(res->initial_pose, s->last_initial_pose, sizeof(res->initial_pose));
    res->pose_valid = t->curr_frm.pose_valid;
    mat4_identity(res->pose_wc);
    if (res->pose_valid) {
        for (c = 0; c < 3; ++c) {
            for (r = 0; r < 3; ++r) {
                res->pose_wc[c * 4 + r] = t->curr_frm.rot_wc[c * 3 + r];
            }
        }
        for (r = 0; r < 3; ++r) {
            res->pose_wc[12 + r] = t->curr_frm.trans_wc[r];
        }
    }
    res->num_tracked = s->last_num_tracked;
    res->num_reliable = s->last_num_reliable;
    res->ref_kf = t->curr_frm.ref_kf;
    res->decision = s->last_decision;
    res->n_global_steps = s->cur_global_steps;
    res->loop_accepted = s->cur_loop_accepted;
    res->loop_cur_kf = s->cur_loop_cur;
    res->loop_cand_kf = s->cur_loop_cand;
    res->n_keyframes = s->map.num_keyframes;
    res->n_landmarks = count_landmarks(s);
    res->n_local_kfs = t->local.n_kfs;
    res->n_local_lms = t->local.n_lms;
    res->local_kfs = t->local.kfs;
    res->local_lms = t->local.lms;
    res->reset_happened = s->cur_reset;
    (void)fd;
}

/* stella_vio: up direction of the current map from the accelerometer. With R_wc the tracked camera rotation and f the mean specific force over
 * the frame interval (body axes; f = +up at rest), sum R_wc R_CB f over all tracked frames: body accelerations average out, the sum points up. */
static void gravity_accumulate(sv_system* s, int tracked, double ts) {
    if (s->p.gravity && s->p.imu && tracked && s->have_prev_ts && ts > s->prev_ts) {
        double f[3];
        const int n = sv_imu_acc_mean(s->p.imu, (int64_t)llround((s->prev_ts + s->p.imu_toff) * 1e9),
                                      (int64_t)llround((ts + s->p.imu_toff) * 1e9), f);
        if (n > 0) {
            const double* R = s->p.imu_R_BC; /* camera -> body, row-major */
            const double* W = s->trk.curr_frm.rot_wc; /* column-major */
            double uc[3], uw[3];
            unsigned int id = (unsigned int)s->map_id, k;
            for (k = 0; k < 3; ++k) {
                uc[k] = R[0 * 3 + k] * f[0] + R[1 * 3 + k] * f[1] + R[2 * 3 + k] * f[2];
            }
            for (k = 0; k < 3; ++k) {
                uw[k] = W[0 * 3 + k] * uc[0] + W[1 * 3 + k] * uc[1] + W[2 * 3 + k] * uc[2];
            }
            if (id >= s->up_cap) {
                const unsigned int nc = id * 2 + 8;
                s->up_sum = (double(*)[3])realloc(s->up_sum, nc * sizeof(*s->up_sum));
                s->up_n = (unsigned int*)realloc(s->up_n, nc * sizeof(unsigned int));
                memset(s->up_sum + s->up_cap, 0, (nc - s->up_cap) * sizeof(*s->up_sum));
                memset(s->up_n + s->up_cap, 0, (nc - s->up_cap) * sizeof(unsigned int));
                s->up_cap = nc;
            }
            for (k = 0; k < 3; ++k) {
                s->up_sum[id][k] += uw[k];
            }
            s->up_n[id]++;
        }
    }
    s->prev_ts = ts;
    s->have_prev_ts = 1;
}

unsigned int sv_system_map_up(const sv_system* s, int map_id, double up[3]) {
    double n;
    if (map_id < 0 || (unsigned int)map_id >= s->up_cap || !s->up_n[map_id]) {
        return 0;
    }
    n = sqrt(s->up_sum[map_id][0] * s->up_sum[map_id][0] + s->up_sum[map_id][1] * s->up_sum[map_id][1] + s->up_sum[map_id][2] * s->up_sum[map_id][2]);
    if (n <= 0.0) {
        return 0;
    }
    up[0] = s->up_sum[map_id][0] / n;
    up[1] = s->up_sum[map_id][1] / n;
    up[2] = s->up_sum[map_id][2] / n;
    return s->up_n[map_id];
}

/* The bridged part (keyframes >= cal_k0, landmarks >= cal_lm0) is a self-consistent similarity copy: scale it by f about the bridge point. */
static void calibrate_scale(sv_system* s, double f) {
    sv_tracker* t = &s->trk;
    unsigned int id, i;
    int k;
    const double* c0 = s->cal_c0;
    for (id = s->cal_k0; id < s->next_keyframe_id && id < s->kf_cap; ++id) {
        sv_tr_kf* kf = s->map.kfs[id];
        double pose[16], c[3], Rcw[9];
        int r;
        if (!kf || !kf->alive) {
            continue;
        }
        memcpy(pose, kf->pose_cw, sizeof(pose));
        for (k = 0; k < 3; ++k) {
            c[k] = c0[k] + f * (kf->trans_wc[k] - c0[k]);
        }
        for (k = 0; k < 3; ++k) {
            for (r = 0; r < 3; ++r) {
                Rcw[r * 3 + k] = pose[k * 4 + r];
            }
        }
        pose[12] = -(Rcw[0] * c[0] + Rcw[1] * c[1] + Rcw[2] * c[2]);
        pose[13] = -(Rcw[3] * c[0] + Rcw[4] * c[1] + Rcw[5] * c[2]);
        pose[14] = -(Rcw[6] * c[0] + Rcw[7] * c[1] + Rcw[8] * c[2]);
        sv_tr_kf_set_pose_cw(kf, pose);
    }
    for (id = s->cal_lm0; id < s->mp.next_landmark_id && id < s->lm_cap; ++id) {
        sv_tr_lm* lm = s->map.lms[id];
        if (!lm || !lm->alive) {
            continue;
        }
        for (k = 0; k < 3; ++k) {
            lm->pos_w[k] = c0[k] + f * (lm->pos_w[k] - c0[k]);
        }
    }
    for (id = s->cal_lm0; id < s->mp.next_landmark_id && id < s->lm_cap; ++id) {
        sv_tr_lm* lm = s->map.lms[id];
        if (lm && lm->alive) {
            sv_tr_lm_update_mean_normal_and_obs_scale_variance(&s->cfg, &s->map, lm);
        }
    }
    for (i = 0; i < s->fs_cap; ++i) { /* frames referenced to the part: relative translation scales */
        if (s->fs[i].valid && s->fs[i].ref >= (int)s->cal_k0) {
            s->fs[i].rel[12] *= f;
            s->fs[i].rel[13] *= f;
            s->fs[i].rel[14] *= f;
        }
    }
    for (k = 12; k < 15; ++k) {
        t->twist[k] *= f;
        t->last_cam_pose_from_ref_keyfrm[k] *= f;
    }
    for (k = 0; k < 3; ++k) {
        t->vel_w[k] *= f;
        s->ema_vel[k] *= f;
    }
    s->ema_speed *= f;
    s->cal_on = 0;
}

int sv_system_feed(sv_system* s, const uint8_t* gray, double timestamp, sv_frame_result* out) {
    sv_frame_result dummy;
    sv_frame_result* res = out ? out : &dummy;
    sv_sys_fd* fd;
    sv_tracker* t = &s->trk;
    int n, succeeded = 0, inserted = -1, state_before;
    int rok = 0, rframe_ok = 0, r_gyro_only = 0;
    unsigned int i;

    memset(res, 0, sizeof(*res));
    s->cur_loop_accepted = 0;
    s->cur_loop_cur = s->cur_loop_cand = -1;
    s->cur_global_steps = 0;
    s->cur_reset = 0;
    s->cur_frame = (long)s->next_frame_id;
    res->frame_id = s->next_frame_id;
    res->timestamp = timestamp;
    res->inserted_kf = -1;
    state_before = t->tracking_state;
    s->stats.frames++;

    { /* a frame that did not become last_frm_ (reset paths) is dropped now */
        sv_sys_fd* stale = s->fd_cur;
        s->fd_cur = NULL;
        if (stale != s->fd_prev) {
            fd_release_if_unused(s, stale);
        }
    }
    /* system::create_monocular_frame: extract + undistort */
    n = sv_orb_extract(gray, s->p.cols, s->p.rows, &s->p.orb, s->tmp_kp, s->tmp_desc, SV_SYS_MAX_KP);
    if (n < 0) {
        n = SV_SYS_MAX_KP;
    }
    fd = (sv_sys_fd*)calloc(1, sizeof(sv_sys_fd));
    fd->id = s->next_frame_id++;
    fd->ts = timestamp;
    fd->kp = (sv_keypoint*)malloc((size_t)(n ? n : 1) * sizeof(sv_keypoint));
    fd->desc = (uint8_t*)malloc((size_t)(n ? n : 1) * 32);
    memcpy(fd->desc, s->tmp_desc, (size_t)n * 32);
    for (i = 0; i < (unsigned int)n; ++i) {
        const sv_keypoint k = s->tmp_kp[i];
        float ux, uy;
        sv_undistort_point(&s->p.cam, k.x, k.y, &ux, &uy);
        memset(&fd->kp[i], 0, sizeof(sv_keypoint)); /* undistort_keypoints copies angle/size/octave only */
        fd->kp[i].x = ux;
        fd->kp[i].y = uy;
        fd->kp[i].angle = k.angle;
        fd->kp[i].size = k.size;
        fd->kp[i].octave = k.octave;
    }
    sv_tr_obs_init(&fd->obs, &s->cfg, fd->kp, fd->desc, (unsigned int)n);
    s->fd_cur = fd;

    refresh_last_inserted(s);
    s->map.fixed_keyframe_id_threshold = 0;

    if (state_before == 0) { /* tracking_module::initialize() */
        sv_tr_frame_free(&t->curr_frm);
        sv_tr_frame_init(&t->curr_frm, fd->id, timestamp, &fd->obs);
        t->succeeded = 0;
        t->decision_evaluated = 0;
        succeeded = initialize(s, fd, res);
        if (s->cur_reset) { /* Wrong: reset() and return nullptr */
            fill_report(s, res, fd, state_before);
            res->pose_valid = 0;
            return 0; /* fd stays fd_cur (the report's views refer to it); released at the next feed */
        }
    }
    else { /* track() + keyframe insertion */
        sv_tr_frame input;
        sv_tr_frame_init(&input, fd->id, timestamp, &fd->obs);
        t->r_pose_valid = 0;
        if (s->p.rframe && state_before == 2 && s->r_on) { /* the rotation-chain pose of this frame guides the map search */
            rok = rstep(s, fd, &r_gyro_only);
            if (rok) {
                pose_from_chain(s->r_Rcw, s->r_c, t->r_pose);
                t->r_pose_valid = 1;
            }
        }
        succeeded = sv_tracker_track(t, &s->map, &input);
        sv_tr_frame_free(&input);
        t->r_pose_valid = 0;
        if (succeeded) {
            rend(s, 0); /* the map took over again */
            if (s->p.rframe && state_before == 1 && t->last_frm_valid && t->last_frm.pose_valid && t->curr_frm.pose_valid &&
                t->curr_frm.timestamp > t->last_frm.timestamp && t->curr_frm.timestamp - t->last_frm.timestamp < 0.5) {
                const double dt = t->curr_frm.timestamp - t->last_frm.timestamp;
                int k;
                double d2 = 0.0;
                for (k = 0; k < 3; ++k) {
                    const double dc = t->curr_frm.trans_wc[k] - t->last_frm.trans_wc[k], v = dc / dt;
                    s->ema_vel[k] = s->ema_valid ? 0.9 * s->ema_vel[k] + 0.1 * v : v;
                    d2 += dc * dc;
                }
                s->ema_speed = s->ema_valid ? 0.97 * s->ema_speed + 0.03 * sqrt(d2) / dt : sqrt(d2) / dt;
                s->ema_valid = 1;
                if (s->cal_on && t->last_frm.ref_kf >= (int)s->cal_k0) { /* path length of the bridged part */
                    s->cal_len += sqrt(d2);
                }
            }
        }
        else if (s->p.rframe) {
            if (state_before == 1 && timestamp - s->init_frm_stamp >= s->p.init_retry_threshold_time && rstart(s)) {
                rok = rstep(s, fd, &r_gyro_only);
            }
            if (rok && s->r_on) { /* this frame is an R-frame: rotation observed, position extrapolated */
                double pose[16];
                for (i = 0; i < t->curr_frm.obs->num_kp; ++i) {
                    t->curr_frm.lm[i] = SV_TR_NONE;
                }
                pose_from_chain(s->r_Rcw, s->r_c, pose);
                sv_tr_frame_set_pose_cw(&t->curr_frm, pose);
                t->curr_frm.ref_kf = s->r_anchor;
                rframe_ok = 1;
                s->stats.rframes++;
                s->stats.rframes_gyro += (unsigned int)r_gyro_only;
                res->rframe_kind = r_gyro_only ? 2 : 1;
                res->rframe_inliers = s->r_last_inl;
                res->rframe_par_deg = s->r_last_par;
                if (state_before == 2 && timestamp - s->r_t0 >= s->p.rframe_init_after && initialize(s, fd, res)) {
                    succeeded = 1; /* bridged: a new map was installed into the existing one */
                    rframe_ok = 0;
                    res->rframe_kind = 3;
                }
            }
        }
        s->last_path = t->path;
        s->last_initial_valid = t->initial_pose_valid;
        if (t->initial_pose_valid) {
            memcpy(s->last_initial_pose, t->initial_pose, sizeof(s->last_initial_pose));
        }
        if (t->optimize_ran) {
            s->last_num_tracked = t->num_tracked_lms;
            s->last_num_reliable = t->num_reliable_lms;
        }
        fs_update(s, &t->curr_frm, !succeeded && !rframe_ok); /* map_db_->update_frame_statistics(curr_frm_, !succeeded) */
        if (rframe_ok) {
            fs_get(s, fd->id)->rframe = 1;
        }
        if (t->decision_evaluated) {
            s->last_decision = t->decision;
        }
        if (succeeded && t->decision_evaluated && t->decision.verdict) {
            sv_tr_kf* kf;
            const unsigned int new_id = s->next_keyframe_id++;
            kf = kf_record(s, new_id);
            kf_record_reset(kf);
            s->kf_label[new_id] = t->curr_frm.ref_kf >= 0 && (unsigned int)t->curr_frm.ref_kf < s->kf_cap ? s->kf_label[t->curr_frm.ref_kf] : s->map_id;
            fd_keep(s, fd);
            s->kf_fd[new_id] = fd;
            if (sv_tr_create_new_keyframe(&s->cfg, &s->map, &t->curr_frm, new_id, timestamp, kf) == 0 &&
                mapping_pass(s, new_id) == 0) {
                inserted = (int)new_id;
                s->stats.keyframes_inserted++;
            }
        }
        res->track_succeeded = succeeded;
    }
    res->inserted_kf = inserted;
    t->succeeded = succeeded;

    /* state transition */
    if (succeeded) {
        t->tracking_state = 1;
    }
    else if (t->tracking_state == 1) {
        t->tracking_state = 2;
        s->stats.lost_frames += rframe_ok ? 0u : 1u;
        s->lost_since = timestamp;
        if (timestamp - s->init_frm_stamp < s->p.init_retry_threshold_time) { /* lost shortly after initialization */
            reset_current(s);
            fill_report(s, res, fd, state_before);
            res->pose_valid = 0;
            return 0;
        }
    }
    else if (t->tracking_state == 2) {
        s->stats.lost_frames += rframe_ok ? 0u : 1u;
        if (s->p.reinit_lost_sec > 0.0 && timestamp - s->lost_since >= s->p.reinit_lost_sec && !s->r_on) {
            /* stella_vio: relocalization has failed for reinit_lost_sec -> start a new map from this frame on */
            if (s->p.merge_maps) { /* keep the map: its frames stay in the frame statistics, a new unconnected map starts */
                s->map_id = s->next_label++;
                label_ensure(s, s->map_id);
                s->stats.reinits++;
                sys_soft_reset(s);
            }
            else {
                archive_current_map(s);
                s->map_id = s->next_label++;
                s->stats.reinits++;
                sys_reset(s);
            }
            fill_report(s, res, fd, state_before);
            res->pose_valid = 0;
            return 0;
        }
    }
    sv_tracker_finish_frame(t, &s->map, inserted);
    if (succeeded && t->good_valid && ((s->p.gyro_mode & 2) || s->p.rframe)) { /* stella_vio: the tracker's good frame refers to this frame's data */
        sv_sys_fd* old = s->fd_good;
        s->fd_good = fd;
        if (old != fd) {
            fd_release_if_unused(s, old);
        }
    }
    gravity_accumulate(s, succeeded && t->curr_frm.pose_valid, timestamp);

    synchronize(s); /* system::synchronize_background_modules() */
    if (s->cal_on) {
        const double dur = timestamp - s->cal_t0;
        if (t->tracking_state != 1 || !succeeded) { /* lost again before the calibration: use what there is if it is long enough */
            if (dur >= 0.5 * s->p.rframe_calib_sec && s->cal_len > 0.0) {
                const double f = s->cal_vold / (s->cal_len / dur);
                if (f > 0.25 && f < 4.0) {
                    res->cal_f = f;
                    res->cal_vold = s->cal_vold;
                    res->cal_vnew = s->cal_len / dur;
                    calibrate_scale(s, f);
                }
            }
            s->cal_on = 0;
        }
        else if (dur >= s->p.rframe_calib_sec && s->cal_len > 0.0) {
            const double f = s->cal_vold / (s->cal_len / dur);
            if (f > 0.25 && f < 4.0) {
                res->cal_f = f;
                res->cal_vold = s->cal_vold;
                res->cal_vnew = s->cal_len / dur;
                calibrate_scale(s, f);
            }
            s->cal_on = 0;
        }
    }

    {
        sv_sys_fd* old = s->fd_prev;
        s->fd_prev = fd;
        fd_release_if_unused(s, old);
    }
    fill_report(s, res, fd, state_before);
    return 0;
}

/* ------------------------------------------------------------------ */
/* views                                                              */
/* ------------------------------------------------------------------ */
const sv_tr_map* sv_system_map(const sv_system* s) { return &s->map; }
sv_mapping* sv_system_mapping(sv_system* s) { return &s->mp; }
const sv_loop* sv_system_loop(const sv_system* s) { return &s->lp; }
const sv_tracker* sv_system_tracker(const sv_system* s) { return &s->trk; }

const int* sv_system_curr_landmarks(const sv_system* s, unsigned int* n) {
    if (n) {
        *n = s->trk.curr_frm.obs ? s->trk.curr_frm.obs->num_kp : 0;
    }
    return s->trk.curr_frm.lm;
}

unsigned int sv_system_num_landmarks(const sv_system* s) { return count_landmarks(s); }
void sv_system_get_stats(const sv_system* s, sv_system_stats* st) { *st = s->stats; }

int sv_system_kf_erased_frame(const sv_system* s, unsigned int kf_id) {
    return kf_id < s->kf_cap ? s->kf_erased_frame[kf_id] : -1;
}

int sv_system_kf_destroyed_frame(const sv_system* s, unsigned int kf_id) {
    return kf_id < s->kf_cap ? s->kf_destroyed_frame[kf_id] : -1;
}

/* ------------------------------------------------------------------ */
/* trajectory                                                         */
/* ------------------------------------------------------------------ */
static int current_trajectory(const sv_system* s, sv_traj_entry** entries, unsigned int* n) {
    unsigned int i, cnt = 0;
    sv_traj_entry* e = (sv_traj_entry*)calloc(s->fs_cap ? s->fs_cap : 1, sizeof(sv_traj_entry));
    for (i = 0; i < s->fs_cap; ++i) {
        const sv_sys_fstat* f = &s->fs[i];
        const sv_tr_kf* ref;
        double cw[16], wc[16], rot[9];
        int r, c;
        sv_quat q;
        if (!f->valid || f->lost) {
            continue;
        }
        ref = sv_tr_map_kf_any(&s->map, f->ref);
        if (!ref) {
            continue;
        }
        sv_mat4_mul(f->rel, ref->pose_cw, cw); /* rel_cam_pose_cr * cam_pose_rw */
        pose_inverse_rigid(cw, wc);
        e[cnt].frame_id = i;
        e[cnt].map_id = label_root(s, f->map);
        e[cnt].rframe = f->rframe;
        e[cnt].seg = f->map;
        e[cnt].timestamp = f->ts;
        memcpy(e[cnt].pose_wc, wc, sizeof(wc));
        for (c = 0; c < 3; ++c) {
            for (r = 0; r < 3; ++r) {
                rot[c * 3 + r] = wc[c * 4 + r];
            }
        }
        sv_quat_from_mat3(rot, &q);
        e[cnt].quat_xyzw[0] = q.x;
        e[cnt].quat_xyzw[1] = q.y;
        e[cnt].quat_xyzw[2] = q.z;
        e[cnt].quat_xyzw[3] = q.w;
        ++cnt;
    }
    *entries = e;
    *n = cnt;
    return 0;
}

/* stella_vio: dropped maps' poses first (ascending time), then the current map */
int sv_system_trajectory(const sv_system* s, sv_traj_entry** entries, unsigned int* n) {
    sv_traj_entry* cur;
    unsigned int ncur, tot;
    sv_traj_entry* all;
    current_trajectory(s, &cur, &ncur);
    tot = s->n_arch + ncur;
    all = (sv_traj_entry*)calloc(tot ? tot : 1, sizeof(sv_traj_entry));
    if (s->n_arch) {
        memcpy(all, s->arch, s->n_arch * sizeof(sv_traj_entry));
    }
    memcpy(all + s->n_arch, cur, ncur * sizeof(sv_traj_entry));
    free(cur);
    *entries = all;
    *n = tot;
    return 0;
}

/* stella_vio: keep the poses of the map that is about to be dropped (frame ids are global, so they never collide) */
static void archive_current_map(sv_system* s) {
    sv_traj_entry* cur;
    unsigned int ncur;
    current_trajectory(s, &cur, &ncur);
    if (s->n_arch + ncur > s->cap_arch) {
        s->cap_arch = (s->n_arch + ncur) * 2 + 1024;
        s->arch = (sv_traj_entry*)realloc(s->arch, s->cap_arch * sizeof(sv_traj_entry));
    }
    memcpy(s->arch + s->n_arch, cur, ncur * sizeof(sv_traj_entry));
    s->n_arch += ncur;
    free(cur);
}
