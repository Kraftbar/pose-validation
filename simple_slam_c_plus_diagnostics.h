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
            "\"avg_inliers_after_first\": %f, \"kf_min_inliers\": %d, \"kf_period\": %d, "
            "\"kf_min_interval\": %d, \"healthy_keyframes\": %d, "
            "\"late_kf_cooldown\": %d, \"ba_interval\": %d, "
            "\"ba_start_keyframes\": %d, \"ba_max_points\": %d, "
            "\"global_ba_interval\": %d, "
            "\"global_ba_start_keyframes\": %d, \"global_ba_max_points\": %d, "
            "\"map_hygiene\": %d, "
            "\"kf_warmup_frames\": %d, "
            "\"new_point_obs\": %d, \"pnp_min_obs\": %d, \"pnp_start_frame\": %d, "
            "\"delayed_init_frames\": %d, "
            "\"candidate_tracks\": %d, \"candidate_min_obs\": %d, "
            "\"candidate_min_age\": %d, \"candidate_grid_cols\": %d, "
            "\"candidate_grid_rows\": %d, \"candidate_promote_per_cell\": %d, "
            "\"candidate_max_fb_err\": %f, \"candidate_min_disp\": %f, "
            "\"candidate_max_disp\": %f, "
            "\"first_kf_observations\": %d, "
            "\"unique_kf_observations\": %d, \"essential_cheirality_max\": %d, "
            "\"tri_min_parallax_deg\": %f, \"tri_max_reproj_px\": %f, "
            "\"tri_max_depth\": %f, \"tri_max_depth_ratio\": %f, "
            "\"tri_source_kf_gap\": %d, "
            "\"pnp_pred_reproj_gate\": %f, "
            "\"pnp_quality_gate_px\": %f, \"pnp_quality_min_obs\": %d, "
            "\"pnp_quality_window\": %d, "
            "\"obs_stat_gate_px\": %f, \"obs_stat_min_good\": %d, "
            "\"obs_stat_max_bad_ratio\": %f, "
            "\"pnp_p3p_fallback\": %d, \"pnp_p3p_observe\": %d, "
            "\"pnp_p3p_iterations\": %d, "
            "\"pnp_p3p_max_jump\": %f, \"pnp_p3p_min_inl2\": %d, "
            "\"pnp_p3p_min_gain\": %d, \"pnp_p3p_max_mederr\": %f, "
            "\"pnp_p3p_min_posz\": %f, \"pnp_dlt_iters\": %d, "
            "\"pnp_dlt_pretest\": %d, \"pnp_dlt_pretest_margin\": %d, "
            "\"ffmpeg_gray\": %d, \"fast_corners\": %d, \"distributed_features\": %d, "
            "\"pyramid_features\": %d, "
            "\"max_points\": %d, \"lk_iters\": %d, \"lk_back_iters\": %d, "
            "\"pose_lm_iters\": %d, \"essential_iters\": %d, "
            "\"descriptor_map_admission\": %d, \"descriptor_primary_admission\": %d, "
            "\"descriptor_mutual_admission\": %d, "
            "\"oriented_brief\": %d, "
            "\"descriptor_admission_max_hamming\": %d, "
            "\"descriptor_admission_ratio\": %f, \"descriptor_primary_map_cap\": %d, "
            "\"descriptor_source_kf_gap\": %d, "
            "\"triangulate_with_e_pose\": %d, "
            "\"triangulate_relative_frame\": %d, "
            "\"shape_e_inliers\": %d, \"e_shape_max_matches\": %d, "
            "\"e_shape_grid_cap\": %d, \"e_shape_max_disp\": %f, "
            "\"e_shape_max_fb_err\": %f, \"e_shape_target_disp\": %f, "
            "\"anchor_e_pose\": %d, \"anchor_max_features\": %d, "
            "\"anchor_max_hamming\": %d, \"anchor_ratio\": %f, "
            "\"admission_ranked\": %d, \"admission_finite_only\": %d, "
            "\"admission_batch_ranked\": %d, \"admission_batch_deferred\": %d, "
            "\"admission_target_disp\": %f, "
            "\"admission_fb_weight\": %f, "
            "\"admission_max_new_points\": %d, \"admission_grid_cap\": %d, "
            "\"timeline\": [\n",
            s->size, pts, dur, cfg->video_path, cfg->proc_w, cfg->proc_h, kf, tri,
            cfg->speed_profile ? cfg->speed_profile : "", av,
            cfg->kf_min_inliers, cfg->kf_period, cfg->kf_min_interval, cfg->healthy_keyframes,
            cfg->late_kf_cooldown, cfg->ba_interval, cfg->ba_start_keyframes,
            cfg->ba_max_points, cfg->global_ba_interval, cfg->global_ba_start_keyframes,
            cfg->global_ba_max_points, cfg->map_hygiene,
            cfg->kf_warmup_frames, cfg->new_point_obs, cfg->pnp_min_obs, cfg->pnp_start_frame,
            cfg->delayed_init_frames, cfg->candidate_tracks, cfg->candidate_min_obs, cfg->candidate_min_age,
            cfg->candidate_grid_cols, cfg->candidate_grid_rows, cfg->candidate_promote_per_cell,
            cfg->candidate_max_fb_err, cfg->candidate_min_disp, cfg->candidate_max_disp,
            cfg->first_kf_observations,
            cfg->unique_kf_observations, cfg->essential_cheirality_max,
            cfg->tri_min_parallax_deg, cfg->tri_max_reproj_px,
            cfg->tri_max_depth, cfg->tri_max_depth_ratio, cfg->tri_source_kf_gap,
            cfg->pnp_pred_reproj_gate,
            cfg->pnp_quality_gate_px, cfg->pnp_quality_min_obs, cfg->pnp_quality_window,
            cfg->obs_stat_gate_px, cfg->obs_stat_min_good, cfg->obs_stat_max_bad_ratio,
            cfg->pnp_p3p_fallback, cfg->pnp_p3p_observe, cfg->pnp_p3p_iterations,
            cfg->pnp_p3p_max_jump, cfg->pnp_p3p_min_inl2, cfg->pnp_p3p_min_gain,
            cfg->pnp_p3p_max_mederr, cfg->pnp_p3p_min_posz, cfg->pnp_dlt_iters, cfg->pnp_dlt_pretest,
            cfg->pnp_dlt_pretest_margin, cfg->ffmpeg_gray, cfg->fast_corners,
            cfg->distributed_features, cfg->pyramid_features,
            cfg->max_points, cfg->lk_iters, cfg->lk_back_iters, cfg->pose_lm_iters, cfg->essential_iters,
            cfg->descriptor_map_admission, cfg->descriptor_primary_admission,
            cfg->descriptor_mutual_admission, cfg->oriented_brief,
            cfg->descriptor_admission_max_hamming,
            cfg->descriptor_admission_ratio, cfg->descriptor_primary_map_cap,
            cfg->descriptor_source_kf_gap, cfg->triangulate_with_e_pose,
            cfg->triangulate_relative_frame, cfg->shape_e_inliers,
            cfg->e_shape_max_matches, cfg->e_shape_grid_cap,
            cfg->e_shape_max_disp, cfg->e_shape_max_fb_err,
            cfg->e_shape_target_disp, cfg->anchor_e_pose,
            cfg->anchor_max_features, cfg->anchor_max_hamming, cfg->anchor_ratio,
            cfg->admission_ranked, cfg->admission_finite_only, cfg->admission_batch_ranked,
            cfg->admission_batch_deferred,
            cfg->admission_target_disp, cfg->admission_fb_weight,
            cfg->admission_max_new_points, cfg->admission_grid_cap);
    for (int i = 0; i < s->size; i++)
        fprintf(f,
                "    {\"frame_id\": %d, \"inliers\": %d, \"is_keyframe\": %s, \"points_added\": "
                "%d, \"points_total\": %d, \"method\": %d, \"tracked_count\": %d, "
                "\"linked_points\": %d, \"linked_before_relink\": %d, \"relinked_points\": %d, "
                "\"pnp_inliers\": %d, \"pred_lm_inliers\": %d, \"e_inliers\": %d, "
                "\"pnp_fallback_used\": %d, \"pnp_p3p_attempted\": %d, "
                "\"pnp_p3p_solved\": %d, \"pnp_p3p_accepted\": %d, "
                "\"pnp_p3p_inliers\": %d, \"pnp_p3p_inliers2\": %d, "
                "\"pnp_p3p_inliers3\": %d, \"pnp_p3p_inliers5\": %d, "
                "\"pnp_p3p_mederr\": %f, \"pnp_p3p_posz\": %f, "
                "\"pnp_p3p_jump\": %f, \"pnp_p3p_xyz\": [%f,%f,%f], "
                "\"trans_jump\": %f, \"xyz\": [%f,%f,%f]}%s\n",
                s->data[i].frame_id, s->data[i].inliers, s->data[i].is_keyframe ? "true" : "false",
                s->data[i].points_added, s->data[i].points_total, s->data[i].method,
                s->data[i].tracked_count, s->data[i].linked_points, s->data[i].linked_before_relink,
                s->data[i].relinked_points, s->data[i].pnp_inliers, s->data[i].pred_lm_inliers,
                s->data[i].e_inliers, s->data[i].pnp_fallback_used,
                s->data[i].pnp_p3p_attempted, s->data[i].pnp_p3p_solved,
                s->data[i].pnp_p3p_accepted, s->data[i].pnp_p3p_inliers,
                s->data[i].pnp_p3p_inliers2, s->data[i].pnp_p3p_inliers3,
                s->data[i].pnp_p3p_inliers5, s->data[i].pnp_p3p_mederr,
                s->data[i].pnp_p3p_posz, s->data[i].pnp_p3p_jump,
                s->data[i].pnp_p3p_xyz[0], s->data[i].pnp_p3p_xyz[1],
                s->data[i].pnp_p3p_xyz[2], s->data[i].trans_jump, s->data[i].xyz[0],
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

typedef struct {
    FILE *pnp_dump;
    FILE *track_dump;
    FILE *map_admission_dump;
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
    if (diag->e_inlier_dump)
        fclose(diag->e_inlier_dump);
}

#endif
