#ifndef SIMPLE_SLAM_C_PLUS_DIAGNOSTICS_H
#define SIMPLE_SLAM_C_PLUS_DIAGNOSTICS_H

static void ensure_parent_dir(const char *p) {
    char b[512], *s;
    snprintf(b, 512, "%s", p);
    s = strrchr(b, '/');
    if (s) {
        *s = '\0';
        char c[640];
        snprintf(c, 640, "mkdir -p \"%s\"", b);
        (void)!system(c);
    }
}

static void write_metrics_json(FILE *f, const Config *cfg, const FrameStatVec *s, int pts, int tri,
                               double dur) {
    int kf = 0;
    double av = 0;
    for (int i = 0; i < s->size; i++) {
        if (s->data[i].is_keyframe)
            kf++;
        if (i > 0)
            av += s->data[i].inliers;
    }
    if (s->size > 1)
        av /= (s->size - 1);
    fprintf(f,
            "{\n  \"frames\": %d, \"points\": %d, \"duration_sec\": %f, \"video_path\": \"%s\", "
            "\"proc_w\": %d, \"proc_h\": %d, \"keyframes\": %d, \"tri_points_total\": %d, "
            "\"speed_profile\": \"%s\", "
            "\"ransac_seed\": %d, "
            "\"avg_inliers_after_first\": %f, \"kf_min_inliers\": %d, \"kf_period\": %d, "
            "\"kf_min_interval\": %d, \"ba_interval\": %d, "
            "\"ba_start_keyframes\": %d, \"ba_max_points\": %d, "
            "\"local_ba_fix_oldest\": %d, "
            "\"global_ba_interval\": %d, "
            "\"global_ba_start_keyframes\": %d, \"global_ba_max_points\": %d, "
            "\"new_point_obs\": %d, \"pnp_min_obs\": %d, "
            "\"essential_cheirality_max\": %d, "
            "\"pnp_dlt_iters\": %d, "
            "\"pnp_low_e_fallback\": %d, \"pnp_low_e_max_inliers\": %d, "
            "\"pnp_low_e_min_inliers\": %d, \"pnp_low_e_min_gain\": %d, "
            "\"pnp_low_e_min_jump\": %f, "
            "\"pnp_low_e_min_map_points\": %d, "
            "\"pnp_low_e_min_pose_jump\": %f, "
            "\"ffmpeg_gray\": %d, "
            "\"max_points\": %d, \"lk_iters\": %d, \"lk_back_iters\": %d, "
            "\"pose_lm_iters\": %d, \"essential_iters\": %d, "
            "\"oriented_brief\": %d, \"brief_patch_radius\": %d, "
            "\"anchor_e_pose\": %d, \"anchor_max_features\": %d, "
            "\"anchor_max_hamming\": %d, \"anchor_mutual\": %d, "
            "\"anchor_ratio\": %f, "
            "\"normalize_world_scale\": %d, \"tri_map_scale\": %d, "
            "\"output_smooth_alpha\": %f, "
            "\"output_smooth_outlier_alpha\": %f, "
            "\"output_smooth_residual_k\": %f, "
            "\"output_smooth_window\": %d, "
            "\"output_smooth_unstable_alpha\": %f, "
            "\"output_smooth_unstable_residual_k\": %f, "
            "\"output_smooth_unstable_window\": %d, "
            "\"output_smooth_unstable_points_added\": %d, "
            "\"output_smooth_unstable_jump\": %f, "
            "\"output_smooth_unstable_jump_count\": %d, "
            "\"output_smooth_unstable_cap_residual\": %d, "
            "\"output_smooth_low_link_alpha\": %f, "
            "\"output_smooth_low_link_threshold\": %d, "
            "\"output_smooth_low_link_window\": %d, "
            "\"output_smooth_low_link_count\": %d, "
            "\"output_smooth_low_link_include_current\": %d, "
            "\"timeline\": [\n",
            s->size, pts, dur, cfg->video_path, cfg->proc_w, cfg->proc_h, kf, tri,
            cfg->speed_profile ? cfg->speed_profile : "", cfg->ransac_seed, av,
            cfg->kf_min_inliers, cfg->kf_period, cfg->kf_min_interval,
            cfg->ba_interval, cfg->ba_start_keyframes,
            cfg->ba_max_points, cfg->local_ba_fix_oldest,
            cfg->global_ba_interval, cfg->global_ba_start_keyframes,
            cfg->global_ba_max_points,
            cfg->new_point_obs, cfg->pnp_min_obs,
            cfg->essential_cheirality_max,
            cfg->pnp_dlt_iters,
            cfg->pnp_low_e_fallback, cfg->pnp_low_e_max_inliers,
            cfg->pnp_low_e_min_inliers, cfg->pnp_low_e_min_gain,
            cfg->pnp_low_e_min_jump,
            cfg->pnp_low_e_min_map_points,
            cfg->pnp_low_e_min_pose_jump,
            cfg->ffmpeg_gray,
            cfg->max_points, cfg->lk_iters, cfg->lk_back_iters, cfg->pose_lm_iters,
            cfg->essential_iters,
            cfg->oriented_brief,
            cfg->brief_patch_radius,
            cfg->anchor_e_pose,
            cfg->anchor_max_features, cfg->anchor_max_hamming, cfg->anchor_mutual,
            cfg->anchor_ratio,
            cfg->normalize_world_scale, cfg->tri_map_scale, cfg->output_smooth_alpha,
            cfg->output_smooth_outlier_alpha, cfg->output_smooth_residual_k,
            cfg->output_smooth_window,
            cfg->output_smooth_unstable_alpha,
            cfg->output_smooth_unstable_residual_k,
            cfg->output_smooth_unstable_window,
            cfg->output_smooth_unstable_points_added,
            cfg->output_smooth_unstable_jump,
            cfg->output_smooth_unstable_jump_count,
            cfg->output_smooth_unstable_cap_residual,
            cfg->output_smooth_low_link_alpha,
            cfg->output_smooth_low_link_threshold,
            cfg->output_smooth_low_link_window,
            cfg->output_smooth_low_link_count,
            cfg->output_smooth_low_link_include_current);
    for (int i = 0; i < s->size; i++)
        fprintf(f,
                "    {\"frame_id\": %d, \"inliers\": %d, \"is_keyframe\": %s, \"points_added\": "
                "%d, \"points_total\": %d, \"method\": %d, \"tracked_count\": %d, "
                "\"linked_points\": %d, \"linked_before_relink\": %d, \"relinked_points\": %d, "
                "\"pnp_inliers\": %d, \"pred_lm_inliers\": %d, \"e_inliers\": %d, "
                "\"raw_xyz\": [%f,%f,%f], "
                "\"trans_jump\": %f, \"xyz\": [%f,%f,%f]}%s\n",
                s->data[i].frame_id, s->data[i].inliers, s->data[i].is_keyframe ? "true" : "false",
                s->data[i].points_added, s->data[i].points_total, s->data[i].method,
                s->data[i].tracked_count, s->data[i].linked_points, s->data[i].linked_before_relink,
                s->data[i].relinked_points, s->data[i].pnp_inliers, s->data[i].pred_lm_inliers,
                s->data[i].e_inliers, s->data[i].raw_xyz[0],
                s->data[i].raw_xyz[1], s->data[i].raw_xyz[2],
                s->data[i].trans_jump, s->data[i].xyz[0],
                s->data[i].xyz[1], s->data[i].xyz[2],
                (i + 1 < s->size) ? "," : "");
    fprintf(f, "  ]\n}\n");
}

static void write_profile_csv(FILE *f, const ProfileStatVec *p) {
    fprintf(f,
            "frame_id,decode,gray_blur,feature,lk,relink,pred_lm,pnp,essential,pose_lm,"
            "triangulate,refill,loop,ba,ba_local,ba_joint,pnp_quality,map_hygiene,"
            "global_ba,metrics,total\n");
    for (int i = 0; i < p->size; i++) {
        const ProfileStat *s = &p->data[i];
        double total = s->decode + s->gray_blur + s->feature + s->lk + s->relink + s->pred_lm +
                       s->pnp + s->essential + s->pose_lm + s->triangulate + s->refill +
                       s->loop + s->ba + s->pnp_quality + s->map_hygiene + s->global_ba +
                       s->metrics;
        fprintf(f,
                "%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
                "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                s->frame_id, s->decode, s->gray_blur, s->feature, s->lk, s->relink,
                s->pred_lm, s->pnp, s->essential, s->pose_lm, s->triangulate, s->refill,
                s->loop, s->ba, s->ba_local, s->ba_joint, s->pnp_quality, s->map_hygiene,
                s->global_ba, s->metrics, total);
    }
}

static void write_track_summary(FILE *f, int frame_id, const CornerVec *tracked,
                                const MatchVec *matches, const Map *map, int w, int h) {
    if (!f || frame_id <= 0)
        return;
    int n = tracked->size;
    int linked = 0, linked_valid = 0;
    double sum_fb = 0.0, sum_disp = 0.0;
    double min_x = w, min_y = h, max_x = 0.0, max_y = 0.0;
    unsigned char cells[48] = {0};
    int occupied = 0;
    for (int i = 0; i < tracked->size; i++) {
        const Corner *c = &tracked->data[i];
        sum_fb += c->fb_err;
        sum_disp += c->track_disp;
        if (c->x < min_x) min_x = c->x;
        if (c->y < min_y) min_y = c->y;
        if (c->x > max_x) max_x = c->x;
        if (c->y > max_y) max_y = c->y;
        int gx = (int)(c->x * 8.0 / (double)w);
        int gy = (int)(c->y * 6.0 / (double)h);
        if (gx < 0) gx = 0;
        if (gy < 0) gy = 0;
        if (gx > 7) gx = 7;
        if (gy > 5) gy = 5;
        int cell = gy * 8 + gx;
        if (!cells[cell]) {
            cells[cell] = 1;
            occupied++;
        }
        if (c->pt_idx >= 0) {
            linked++;
            if (c->pt_idx < map->size && map->data[c->pt_idx].obs > 0)
                linked_valid++;
        }
    }
    double mean_fb = n > 0 ? sum_fb / (double)n : 0.0;
    double mean_disp = n > 0 ? sum_disp / (double)n : 0.0;
    if (n == 0) {
        min_x = min_y = max_x = max_y = 0.0;
    }
    fprintf(f,
            "%d,%d,%d,%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%d\n",
            frame_id, matches->size, n, linked, linked_valid, mean_fb, mean_disp,
            min_x, min_y, max_x, max_y, occupied);
}

static void write_e_inlier_pairs(FILE *f, int frame_id, const char *source,
                                 const CornerVec *prev_pts, const CornerVec *cur_pts,
                                 const MatchVec *matches, const unsigned char *mask, int w, int h) {
    if (!f || !mask)
        return;
    for (int i = 0; i < matches->size; i++) {
        if (!mask[i])
            continue;
        int qi = matches->data[i].query_idx;
        int ti = matches->data[i].train_idx;
        if (qi < 0 || qi >= prev_pts->size || ti < 0 || ti >= cur_pts->size)
            continue;
        Corner a = prev_pts->data[qi];
        Corner b = cur_pts->data[ti];
        double dx = (double)b.x - (double)a.x;
        double dy = (double)b.y - (double)a.y;
        double disp = sqrt(dx * dx + dy * dy);
        int gx = (int)(b.x * 8.0 / (double)w);
        int gy = (int)(b.y * 6.0 / (double)h);
        if (gx < 0) gx = 0;
        if (gy < 0) gy = 0;
        if (gx > 7) gx = 7;
        if (gy > 5) gy = 5;
        fprintf(f,
                "%d,%s,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f,%.3f,%d,%.6f,%.6f,%d\n",
                frame_id, source, i, qi, ti, a.x, a.y, b.x, b.y, dx, dy, disp,
                matches->data[i].score, b.pt_idx, b.fb_err, b.track_disp, gy * 8 + gx);
    }
}

static void write_map_admission_summary(FILE *f, int frame_id, int candidates, int accepted,
                                        const Pose *p1, const Pose *p2, double fx, double fy,
                                        double cx, double cy, double sum_reproj,
                                        double sum_parallax, double sum_depth,
                                        double min_depth, double max_depth) {
    (void)fx;
    (void)fy;
    (void)cx;
    (void)cy;
    if (!f)
        return;
    double mean_reproj = accepted > 0 ? sum_reproj / (double)accepted : 0.0;
    double mean_parallax = accepted > 0 ? sum_parallax / (double)accepted : 0.0;
    double mean_depth = accepted > 0 ? sum_depth / (double)accepted : 0.0;
    if (accepted <= 0) {
        min_depth = 0.0;
        max_depth = 0.0;
    }
    double c1[3], c2[3];
    camera_center_from_pose(p1, c1);
    camera_center_from_pose(p2, c2);
    double baseline = vec3_dist(c1, c2);
    fprintf(f, "%d,%d,%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            frame_id, candidates, accepted, baseline, mean_reproj, mean_parallax,
            mean_depth, min_depth, max_depth);
}

static void write_map_admission_detail(FILE *f, int frame_id, const char *source,
                                       const char *decision, int method, int inliers,
                                       int match_idx, int query_idx, int train_idx, int cell,
                                       const Pose *p1, const Pose *p2, double reproj,
                                       double parallax, double depth, double z1, double z2,
                                       double fb_err, double track_disp, double score) {
    if (!f)
        return;
    double baseline = NAN;
    if (p1 && p2) {
        double c1[3], c2[3];
        camera_center_from_pose(p1, c1);
        camera_center_from_pose(p2, c2);
        baseline = vec3_dist(c1, c2);
    }
    fprintf(f, "%d,%s,%s,%d,%d,%d,%d,%d,%d,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            frame_id, source ? source : "", decision ? decision : "", method, inliers,
            match_idx, query_idx, train_idx, cell, baseline, reproj, parallax, depth,
            z1, z2, fb_err, track_disp, score);
}

static void write_map_lifecycle_dump(FILE *f, const Map *map,
                                     const MapLifecycleVec *lifecycle,
                                     int final_frame) {
    if (!f)
        return;
    for (int i = 0; i < lifecycle->size; i++) {
        const MapLifecycle *row = &lifecycle->data[i];
        int final_obs = 0, good_obs = 0, bad_obs = 0, alive = 0;
        double x = NAN, y = NAN, z = NAN;
        int span_frames = row->last_frame >= row->birth_frame
                              ? row->last_frame - row->birth_frame
                              : 0;
        int frames_since_seen = final_frame >= row->last_frame
                                    ? final_frame - row->last_frame
                                    : 0;
        if (row->map_idx >= 0 && row->map_idx < map->size) {
            const MapPoint *mp = &map->data[row->map_idx];
            final_obs = mp->obs;
            good_obs = mp->good_obs;
            bad_obs = mp->bad_obs;
            alive = mp->obs > 0;
            x = mp->x;
            y = mp->y;
            z = mp->z;
        }
        fprintf(f,
                "%d,%d,%d,%d,%d,%s,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%d,%d,%.9f,%.9f,%.9f\n",
                row->map_idx, row->birth_frame, row->last_frame, span_frames,
                frames_since_seen, row->source ? row->source : "", row->method,
                row->inliers, row->cell, row->reproj, row->parallax, row->depth,
                row->fb_err, row->track_disp, row->score, final_obs, good_obs,
                bad_obs, alive, x, y, z);
    }
}

typedef struct {
    FILE *pnp_dump;
    FILE *track_dump;
    FILE *map_admission_dump;
    FILE *map_admission_detail_dump;
    FILE *map_lifecycle_dump;
    FILE *e_inlier_dump;
} DiagnosticsFiles;

static int diagnostics_open_files(const Config *cfg, DiagnosticsFiles *diag) {
    memset(diag, 0, sizeof(*diag));
    if (cfg->pnp_dump) {
        ensure_parent_dir(cfg->pnp_dump);
        diag->pnp_dump = fopen(cfg->pnp_dump, "w");
        if (!diag->pnp_dump) {
            fprintf(stderr, "Failed to open PnP dump: %s\n", cfg->pnp_dump);
            return 0;
        }
    }
    if (cfg->track_dump) {
        ensure_parent_dir(cfg->track_dump);
        diag->track_dump = fopen(cfg->track_dump, "w");
        if (!diag->track_dump) {
            fprintf(stderr, "Failed to open track dump: %s\n", cfg->track_dump);
            return 0;
        }
        fprintf(diag->track_dump,
                "frame_id,matches,tracked,linked,linked_valid,mean_fb_err,mean_disp,min_x,min_y,max_x,max_y,grid_8x6\n");
    }
    if (cfg->map_admission_dump) {
        ensure_parent_dir(cfg->map_admission_dump);
        diag->map_admission_dump = fopen(cfg->map_admission_dump, "w");
        if (!diag->map_admission_dump) {
            fprintf(stderr, "Failed to open map admission dump: %s\n", cfg->map_admission_dump);
            return 0;
        }
        fprintf(diag->map_admission_dump,
                "frame_id,candidates,accepted,baseline,mean_reproj,mean_parallax,mean_depth,min_depth,max_depth\n");
    }
    if (cfg->map_admission_detail_dump) {
        ensure_parent_dir(cfg->map_admission_detail_dump);
        diag->map_admission_detail_dump = fopen(cfg->map_admission_detail_dump, "w");
        if (!diag->map_admission_detail_dump) {
            fprintf(stderr, "Failed to open map admission detail dump: %s\n",
                    cfg->map_admission_detail_dump);
            return 0;
        }
        fprintf(diag->map_admission_detail_dump,
                "frame_id,source,decision,method,inliers,match_idx,query_idx,train_idx,cell,baseline,reproj,parallax,depth,z1,z2,fb_err,track_disp,score\n");
    }
    if (cfg->map_lifecycle_dump) {
        ensure_parent_dir(cfg->map_lifecycle_dump);
        diag->map_lifecycle_dump = fopen(cfg->map_lifecycle_dump, "w");
        if (!diag->map_lifecycle_dump) {
            fprintf(stderr, "Failed to open map lifecycle dump: %s\n",
                    cfg->map_lifecycle_dump);
            return 0;
        }
        fprintf(diag->map_lifecycle_dump,
                "map_idx,birth_frame,last_seen_frame,span_frames,frames_since_seen,source,method,inliers,cell,birth_reproj,birth_parallax,birth_depth,birth_fb_err,birth_track_disp,birth_score,final_obs,good_obs,bad_obs,alive,x,y,z\n");
    }
    if (cfg->e_inlier_dump) {
        ensure_parent_dir(cfg->e_inlier_dump);
        diag->e_inlier_dump = fopen(cfg->e_inlier_dump, "w");
        if (!diag->e_inlier_dump) {
            fprintf(stderr, "Failed to open E inlier dump: %s\n", cfg->e_inlier_dump);
            return 0;
        }
        fprintf(diag->e_inlier_dump,
                "frame_id,source,match_idx,query_idx,train_idx,x1,y1,x2,y2,dx,dy,disp,score,linked_pt,fb_err,track_disp,grid_8x6\n");
    }
    return 1;
}

static void diagnostics_write_outputs(const Config *cfg, const FrameStatVec *stats,
                                      const ProfileStatVec *profile, int pts, int tri,
                                      double duration) {
    const char *out = cfg->metrics_out ? cfg->metrics_out : "runs/pure_c_metrics.json";
    ensure_parent_dir(out);
    FILE *f = fopen(out, "wb");
    if (f) {
        write_metrics_json(f, cfg, stats, pts, tri, duration);
        fclose(f);
    }
    if (cfg->profile_out) {
        ensure_parent_dir(cfg->profile_out);
        FILE *pf = fopen(cfg->profile_out, "wb");
        if (pf) {
            write_profile_csv(pf, profile);
            fclose(pf);
        }
    }
}

static void diagnostics_close_files(DiagnosticsFiles *diag) {
    if (diag->pnp_dump)
        fclose(diag->pnp_dump);
    if (diag->track_dump)
        fclose(diag->track_dump);
    if (diag->map_admission_dump)
        fclose(diag->map_admission_dump);
    if (diag->map_admission_detail_dump)
        fclose(diag->map_admission_detail_dump);
    if (diag->map_lifecycle_dump)
        fclose(diag->map_lifecycle_dump);
    if (diag->e_inlier_dump)
        fclose(diag->e_inlier_dump);
}

#endif
