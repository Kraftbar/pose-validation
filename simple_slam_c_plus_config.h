#ifndef SIMPLE_SLAM_C_PLUS_CONFIG_H
#define SIMPLE_SLAM_C_PLUS_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HEALTHY_KF_MIN_LINKED 80
#define HEALTHY_KF_START_FRAME 200
#define LATE_KF_COOLDOWN_START_FRAME 200
#define LATE_KF_MIN_INTERVAL 2

typedef struct {
    const char *video_path;
    double seconds, timeout;
    const char *metrics_out;
    const char *pnp_dump;
    const char *profile_out;
    const char *speed_profile;
    const char *track_dump;
    const char *map_admission_dump;
    const char *map_admission_detail_dump;
    const char *map_lifecycle_dump;
    const char *e_inlier_dump;
    int ransac_seed;
    int kf_min_inliers;
    double kf_max_rot_deg;
    int max_points;
    int new_point_obs;
    int pnp_min_obs;
    int pnp_start_frame;
    int delayed_init_frames;
    int candidate_tracks;
    int candidate_min_obs;
    int candidate_min_age;
    int candidate_grid_cols;
    int candidate_grid_rows;
    int candidate_promote_per_cell;
    double candidate_max_fb_err;
    double candidate_min_disp;
    double candidate_max_disp;
    int joint_ba;
    int proc_w, proc_h;
    int kf_period;
    int kf_min_interval;
    int healthy_keyframes;
    int late_kf_cooldown;
    int ba_interval;
    int ba_start_keyframes;
    int ba_max_points;
    int local_ba_fix_oldest;
    int global_ba_interval;
    int global_ba_start_keyframes;
    int global_ba_max_points;
    int map_hygiene;
    int kf_warmup_frames;
    int first_kf_observations;
    int unique_kf_observations;
    int essential_cheirality_max;
    double tri_min_parallax_deg;
    double tri_max_reproj_px;
    double tri_max_depth;
    double tri_max_depth_pnp;
    double tri_max_depth_ratio;
    int tri_source_kf_gap;
    double pnp_pred_reproj_gate;
    double pnp_quality_gate_px;
    int pnp_quality_min_obs;
    int pnp_quality_window;
    double obs_stat_gate_px;
    int obs_stat_min_good;
    double obs_stat_max_bad_ratio;
    int pnp_p3p_fallback;
    int pnp_p3p_observe;
    int pnp_p3p_iterations;
    double pnp_p3p_max_jump;
    int pnp_p3p_min_inl2;
    int pnp_p3p_min_gain;
    double pnp_p3p_max_mederr;
    double pnp_p3p_min_posz;
    int pnp_dlt_iters;
    int pnp_dlt_pretest;
    int pnp_dlt_pretest_margin;
    int pnp_score_rigid;
    int pnp_validate_rigid;
    int pnp_validate_rigid_min_points;
    int pnp_normalize_world;
    int pnp_low_e_fallback;
    int pnp_low_e_max_inliers;
    int pnp_low_e_min_inliers;
    int pnp_low_e_min_gain;
    double pnp_low_e_min_jump;
    int pnp_low_e_min_map_points;
    double pnp_low_e_min_pose_jump;
    int ffmpeg_gray;
    int fast_corners;
    int distributed_features;
    int pyramid_features;
    int subpixel_features;
    int feature_min_dist;
    int lk_iters;
    int lk_back_iters;
    int pose_lm_iters;
    int essential_iters;
    int descriptor_map_admission;
    int descriptor_primary_admission;
    int descriptor_mutual_admission;
    int update_map_descriptors;
    int oriented_brief;
    int brief_patch_radius;
    int descriptor_admission_max_hamming;
    double descriptor_admission_ratio;
    int descriptor_primary_map_cap;
    int descriptor_source_kf_gap;
    int triangulate_with_e_pose;
    int triangulate_relative_frame;
    int shape_e_inliers;
    int e_shape_max_matches;
    int e_shape_grid_cap;
    double e_shape_max_disp;
    double e_shape_max_fb_err;
    double e_shape_target_disp;
    int anchor_e_pose;
    int anchor_max_features;
    int anchor_max_hamming;
    int anchor_mutual;
    double anchor_ratio;
    int admission_ranked;
    int admission_finite_only;
    int admission_batch_ranked;
    int admission_batch_deferred;
    double admission_target_disp;
    double admission_fb_weight;
    int admission_min_inliers;
    int admission_pnp_min_inliers;
    int admission_max_new_points;
    int admission_grid_cap;
    int normalize_world_scale;
    double output_smooth_alpha;
    double output_smooth_outlier_alpha;
    double output_smooth_residual_k;
    int output_smooth_window;
    double output_smooth_unstable_alpha;
    double output_smooth_unstable_residual_k;
    int output_smooth_unstable_window;
    int output_smooth_unstable_points_added;
    double output_smooth_unstable_jump;
    int output_smooth_unstable_jump_count;
    int output_smooth_unstable_cap_residual;
    double output_smooth_low_link_alpha;
    int output_smooth_low_link_threshold;
    int output_smooth_low_link_window;
    int output_smooth_low_link_count;
    int output_smooth_low_link_include_current;
} Config;

static void apply_speed_profile(Config *c, const char *name) {
    c->speed_profile = name;
    if (!strcmp(name, "fast_640")) {
        c->ffmpeg_gray = 1;
        c->pnp_dlt_iters = 250;
        c->lk_iters = 5;
        c->lk_back_iters = 2;
        c->ba_interval = 0;
        c->global_ba_interval = 0;
    } else if (!strcmp(name, "fast_320_noba")) {
        c->proc_w = 320;
        c->proc_h = 240;
        c->ffmpeg_gray = 1;
        c->pnp_dlt_iters = 150;
        c->lk_iters = 5;
        c->lk_back_iters = 2;
        c->ba_interval = 0;
        c->global_ba_interval = 0;
    } else if (!strcmp(name, "fast_320_ba")) {
        c->proc_w = 320;
        c->proc_h = 240;
        c->ffmpeg_gray = 1;
        c->pnp_dlt_iters = 250;
        c->lk_iters = 5;
        c->lk_back_iters = 2;
        c->ba_interval = 1;
        c->global_ba_interval = 0;
    } else {
        fprintf(stderr, "unknown --speed_profile %s\n", name);
    }
}

static int config_uses_descriptor_admission(const Config *c) {
    return c->descriptor_map_admission || c->descriptor_primary_admission;
}

static int config_suppresses_lk_landmarks(const Config *c) {
    return config_uses_descriptor_admission(c) && !c->candidate_tracks;
}

static int config_allows_new_landmarks(const Config *c, int frame_id) {
    return c->delayed_init_frames <= 0 || frame_id >= c->delayed_init_frames;
}

static int should_make_keyframe(const Config *c, int frame_id, int inliers,
                                int linked_points, double rot_deg) {
    if (frame_id <= c->kf_warmup_frames)
        return 1;
    if (rot_deg > c->kf_max_rot_deg)
        return 1;
    if (frame_id % c->kf_period == 0)
        return 1;
    if (inliers < c->kf_min_inliers) {
        if (!c->healthy_keyframes || frame_id < HEALTHY_KF_START_FRAME)
            return 1;
        return linked_points >= HEALTHY_KF_MIN_LINKED;
    }
    return 0;
}

static int keyframe_interval_for_frame(const Config *c, int frame_id) {
    int interval = c->kf_min_interval;
    if (c->late_kf_cooldown && frame_id >= LATE_KF_COOLDOWN_START_FRAME &&
        interval < LATE_KF_MIN_INTERVAL)
        interval = LATE_KF_MIN_INTERVAL;
    return interval;
}

static int config_allows_ba(const Config *c, int keyframes, int map_points) {
    if (c->ba_interval <= 0 || keyframes <= 0)
        return 0;
    if (keyframes % c->ba_interval != 0)
        return 0;
    if (c->ba_start_keyframes > 0 && keyframes < c->ba_start_keyframes)
        return 0;
    if (c->ba_max_points > 0 && map_points > c->ba_max_points)
        return 0;
    return 1;
}

static int config_allows_global_ba(const Config *c, int keyframes, int map_points) {
    if (c->global_ba_interval <= 0 || keyframes <= 0)
        return 0;
    if (keyframes % c->global_ba_interval != 0)
        return 0;
    if (c->global_ba_start_keyframes > 0 && keyframes < c->global_ba_start_keyframes)
        return 0;
    if (c->global_ba_max_points > 0 && map_points > c->global_ba_max_points)
        return 0;
    return 1;
}

static Config parse_args(int argc, char **argv) {
    Config c = {
        .video_path = "test_kitti984.mp4",
        .seconds = 5.0,
        .timeout = 30.0,
        .ransac_seed = 0,
        .kf_min_inliers = 40,
        .kf_max_rot_deg = 45.0,
        .max_points = 1000,
        .new_point_obs = 1,
        .pnp_min_obs = 2,
        .candidate_min_obs = 3,
        .candidate_min_age = 5,
        .candidate_grid_cols = 4,
        .candidate_grid_rows = 3,
        .proc_w = 640,
        .proc_h = 480,
        .kf_period = 20,
        .kf_min_interval = 2,
        .ba_interval = 1,
        .local_ba_fix_oldest = 0,
        .global_ba_interval = 10,
        .essential_cheirality_max = 32,
        .pnp_quality_min_obs = 2,
        .pnp_quality_window = 5,
        .obs_stat_max_bad_ratio = 1.0,
        .pnp_p3p_iterations = 500,
        .pnp_p3p_max_jump = 500000.0,
        .pnp_p3p_min_inl2 = 16,
        .pnp_p3p_min_gain = 10,
        .pnp_p3p_max_mederr = 15.0,
        .pnp_p3p_min_posz = 0.8,
        .pnp_dlt_iters = 500,
        .pnp_dlt_pretest_margin = 2,
        .pnp_score_rigid = 0,
        .pnp_validate_rigid = 0,
        .pnp_validate_rigid_min_points = 0,
        .pnp_normalize_world = 0,
        .pnp_low_e_max_inliers = 16,
        .pnp_low_e_min_inliers = 24,
        .pnp_low_e_min_gain = 8,
        .pnp_low_e_fallback = 1,
        .pnp_low_e_min_jump = 500000.0,
        .pnp_low_e_min_map_points = 4000,
        .pnp_low_e_min_pose_jump = 250000.0,
        .lk_iters = 10,
        .lk_back_iters = 5,
        .pose_lm_iters = 10,
        .essential_iters = 500,
        .descriptor_admission_max_hamming = 80,
        .descriptor_admission_ratio = 0.80,
        .update_map_descriptors = 0,
        .oriented_brief = 1,
        .brief_patch_radius = 0,
        .pyramid_features = 1,
        .descriptor_primary_map_cap = 15000,
        .e_shape_max_matches = 64,
        .e_shape_grid_cap = 2,
        .e_shape_max_disp = 14.0,
        .e_shape_max_fb_err = 0.20,
        .e_shape_target_disp = 8.0,
        .anchor_e_pose = 1,
        .anchor_max_features = 600,
        .anchor_max_hamming = 80,
        .anchor_mutual = 0,
        .anchor_ratio = 0.80,
        .admission_target_disp = 12.0,
        .admission_fb_weight = 10.0,
        .normalize_world_scale = 0,
        .output_smooth_alpha = 0.040,
        .output_smooth_outlier_alpha = 0.003,
        .output_smooth_residual_k = 3.50,
        .output_smooth_window = 48,
        .output_smooth_unstable_alpha = 0.15,
        .output_smooth_unstable_residual_k = 3.00,
        .output_smooth_unstable_window = 48,
        .output_smooth_unstable_points_added = 1200,
        .output_smooth_unstable_jump = 250000.0,
        .output_smooth_unstable_jump_count = 5,
        .output_smooth_unstable_cap_residual = 0,
        .output_smooth_low_link_alpha = 0.007,
        .output_smooth_low_link_threshold = 70,
        .output_smooth_low_link_window = 20,
        .output_smooth_low_link_count = 4,
        .output_smooth_low_link_include_current = 1,
    };
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--video_path") && i + 1 < argc)
            c.video_path = argv[++i];
        else if (!strcmp(argv[i], "--seconds") && i + 1 < argc)
            c.seconds = atof(argv[++i]);
        else if (!strcmp(argv[i], "--timeout") && i + 1 < argc)
            c.timeout = atof(argv[++i]);
        else if (!strcmp(argv[i], "--metrics_out") && i + 1 < argc)
            c.metrics_out = argv[++i];
        else if (!strcmp(argv[i], "--pnp_dump") && i + 1 < argc)
            c.pnp_dump = argv[++i];
        else if (!strcmp(argv[i], "--profile_out") && i + 1 < argc)
            c.profile_out = argv[++i];
        else if (!strcmp(argv[i], "--track_dump") && i + 1 < argc)
            c.track_dump = argv[++i];
        else if (!strcmp(argv[i], "--map_admission_dump") && i + 1 < argc)
            c.map_admission_dump = argv[++i];
        else if (!strcmp(argv[i], "--map_admission_detail_dump") && i + 1 < argc)
            c.map_admission_detail_dump = argv[++i];
        else if (!strcmp(argv[i], "--map_lifecycle_dump") && i + 1 < argc)
            c.map_lifecycle_dump = argv[++i];
        else if (!strcmp(argv[i], "--e_inlier_dump") && i + 1 < argc)
            c.e_inlier_dump = argv[++i];
        else if (!strcmp(argv[i], "--speed_profile") && i + 1 < argc)
            apply_speed_profile(&c, argv[++i]);
        else if (!strcmp(argv[i], "--ransac_seed") && i + 1 < argc)
            c.ransac_seed = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--proc_w") && i + 1 < argc)
            c.proc_w = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--proc_h") && i + 1 < argc)
            c.proc_h = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_min_inliers") && i + 1 < argc)
            c.kf_min_inliers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_max_rot_deg") && i + 1 < argc)
            c.kf_max_rot_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--max_points") && i + 1 < argc)
            c.max_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--new_point_obs") && i + 1 < argc)
            c.new_point_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_min_obs") && i + 1 < argc)
            c.pnp_min_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_start_frame") && i + 1 < argc)
            c.pnp_start_frame = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--delayed_init_frames") && i + 1 < argc)
            c.delayed_init_frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_tracks"))
            c.candidate_tracks = 1;
        else if (!strcmp(argv[i], "--descriptor_map_admission"))
            c.descriptor_map_admission = 1;
        else if (!strcmp(argv[i], "--descriptor_primary_admission")) {
            c.descriptor_primary_admission = 1;
            c.descriptor_map_admission = 1;
        }
        else if (!strcmp(argv[i], "--descriptor_mutual_admission"))
            c.descriptor_mutual_admission = 1;
        else if (!strcmp(argv[i], "--update_map_descriptors"))
            c.update_map_descriptors = 1;
        else if (!strcmp(argv[i], "--oriented_brief"))
            c.oriented_brief = 1;
        else if (!strcmp(argv[i], "--brief_patch_radius") && i + 1 < argc)
            c.brief_patch_radius = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_admission_max_hamming") && i + 1 < argc)
            c.descriptor_admission_max_hamming = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_admission_ratio") && i + 1 < argc)
            c.descriptor_admission_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_primary_map_cap") && i + 1 < argc)
            c.descriptor_primary_map_cap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--descriptor_source_kf_gap") && i + 1 < argc)
            c.descriptor_source_kf_gap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--triangulate_with_e_pose"))
            c.triangulate_with_e_pose = 1;
        else if (!strcmp(argv[i], "--triangulate_relative_frame"))
            c.triangulate_relative_frame = 1;
        else if (!strcmp(argv[i], "--shape_e_inliers"))
            c.shape_e_inliers = 1;
        else if (!strcmp(argv[i], "--e_shape_max_matches") && i + 1 < argc)
            c.e_shape_max_matches = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_grid_cap") && i + 1 < argc)
            c.e_shape_grid_cap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_max_disp") && i + 1 < argc)
            c.e_shape_max_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_max_fb_err") && i + 1 < argc)
            c.e_shape_max_fb_err = atof(argv[++i]);
        else if (!strcmp(argv[i], "--e_shape_target_disp") && i + 1 < argc)
            c.e_shape_target_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--anchor_e_pose"))
            c.anchor_e_pose = 1;
        else if (!strcmp(argv[i], "--anchor_max_features") && i + 1 < argc)
            c.anchor_max_features = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--anchor_max_hamming") && i + 1 < argc)
            c.anchor_max_hamming = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--anchor_mutual"))
            c.anchor_mutual = 1;
        else if (!strcmp(argv[i], "--anchor_ratio") && i + 1 < argc)
            c.anchor_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--admission_ranked"))
            c.admission_ranked = 1;
        else if (!strcmp(argv[i], "--admission_finite_only"))
            c.admission_finite_only = 1;
        else if (!strcmp(argv[i], "--admission_batch_ranked"))
            c.admission_batch_ranked = 1;
        else if (!strcmp(argv[i], "--admission_batch_deferred")) {
            c.admission_batch_deferred = 1;
            c.admission_batch_ranked = 1;
        }
        else if (!strcmp(argv[i], "--admission_target_disp") && i + 1 < argc)
            c.admission_target_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--admission_fb_weight") && i + 1 < argc)
            c.admission_fb_weight = atof(argv[++i]);
        else if (!strcmp(argv[i], "--admission_min_inliers") && i + 1 < argc)
            c.admission_min_inliers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--admission_pnp_min_inliers") && i + 1 < argc)
            c.admission_pnp_min_inliers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--admission_max_new_points") && i + 1 < argc)
            c.admission_max_new_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--admission_grid_cap") && i + 1 < argc)
            c.admission_grid_cap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--normalize_world_scale"))
            c.normalize_world_scale = 1;
        else if (!strcmp(argv[i], "--output_smooth_alpha") && i + 1 < argc)
            c.output_smooth_alpha = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_outlier_alpha") && i + 1 < argc)
            c.output_smooth_outlier_alpha = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_residual_k") && i + 1 < argc)
            c.output_smooth_residual_k = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_window") && i + 1 < argc)
            c.output_smooth_window = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_alpha") && i + 1 < argc)
            c.output_smooth_unstable_alpha = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_residual_k") && i + 1 < argc)
            c.output_smooth_unstable_residual_k = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_window") && i + 1 < argc)
            c.output_smooth_unstable_window = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_points_added") && i + 1 < argc)
            c.output_smooth_unstable_points_added = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_jump") && i + 1 < argc)
            c.output_smooth_unstable_jump = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_jump_count") && i + 1 < argc)
            c.output_smooth_unstable_jump_count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_unstable_cap_residual") && i + 1 < argc)
            c.output_smooth_unstable_cap_residual = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_low_link_alpha") && i + 1 < argc)
            c.output_smooth_low_link_alpha = atof(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_low_link_threshold") && i + 1 < argc)
            c.output_smooth_low_link_threshold = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_low_link_window") && i + 1 < argc)
            c.output_smooth_low_link_window = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_low_link_count") && i + 1 < argc)
            c.output_smooth_low_link_count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--output_smooth_low_link_include_current") && i + 1 < argc)
            c.output_smooth_low_link_include_current = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_min_obs") && i + 1 < argc)
            c.candidate_min_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_min_age") && i + 1 < argc)
            c.candidate_min_age = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_grid_cols") && i + 1 < argc)
            c.candidate_grid_cols = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_grid_rows") && i + 1 < argc)
            c.candidate_grid_rows = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_promote_per_cell") && i + 1 < argc)
            c.candidate_promote_per_cell = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_max_fb_err") && i + 1 < argc)
            c.candidate_max_fb_err = atof(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_min_disp") && i + 1 < argc)
            c.candidate_min_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--candidate_max_disp") && i + 1 < argc)
            c.candidate_max_disp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--kf_period") && i + 1 < argc)
            c.kf_period = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_min_interval") && i + 1 < argc)
            c.kf_min_interval = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--healthy_keyframes"))
            c.healthy_keyframes = 1;
        else if (!strcmp(argv[i], "--late_kf_cooldown"))
            c.late_kf_cooldown = 1;
        else if (!strcmp(argv[i], "--ba_interval") && i + 1 < argc)
            c.ba_interval = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ba_start_keyframes") && i + 1 < argc)
            c.ba_start_keyframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ba_max_points") && i + 1 < argc)
            c.ba_max_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--local_ba_fix_oldest"))
            c.local_ba_fix_oldest = 1;
        else if (!strcmp(argv[i], "--global_ba_interval") && i + 1 < argc)
            c.global_ba_interval = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--global_ba_start_keyframes") && i + 1 < argc)
            c.global_ba_start_keyframes = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--global_ba_max_points") && i + 1 < argc)
            c.global_ba_max_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--map_hygiene"))
            c.map_hygiene = 1;
        else if (!strcmp(argv[i], "--kf_warmup_frames") && i + 1 < argc)
            c.kf_warmup_frames = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--first_kf_observations"))
            c.first_kf_observations = 1;
        else if (!strcmp(argv[i], "--unique_kf_observations"))
            c.unique_kf_observations = 1;
        else if (!strcmp(argv[i], "--essential_cheirality_max") && i + 1 < argc)
            c.essential_cheirality_max = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tri_min_parallax_deg") && i + 1 < argc)
            c.tri_min_parallax_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_reproj_px") && i + 1 < argc)
            c.tri_max_reproj_px = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_depth") && i + 1 < argc)
            c.tri_max_depth = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_depth_pnp") && i + 1 < argc)
            c.tri_max_depth_pnp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_max_depth_ratio") && i + 1 < argc)
            c.tri_max_depth_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tri_source_kf_gap") && i + 1 < argc)
            c.tri_source_kf_gap = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_pred_reproj_gate") && i + 1 < argc)
            c.pnp_pred_reproj_gate = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_quality_gate_px") && i + 1 < argc)
            c.pnp_quality_gate_px = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_quality_min_obs") && i + 1 < argc)
            c.pnp_quality_min_obs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_quality_window") && i + 1 < argc)
            c.pnp_quality_window = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--obs_stat_gate_px") && i + 1 < argc)
            c.obs_stat_gate_px = atof(argv[++i]);
        else if (!strcmp(argv[i], "--obs_stat_min_good") && i + 1 < argc)
            c.obs_stat_min_good = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--obs_stat_max_bad_ratio") && i + 1 < argc)
            c.obs_stat_max_bad_ratio = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_fallback"))
            c.pnp_p3p_fallback = 1;
        else if (!strcmp(argv[i], "--pnp_p3p_observe"))
            c.pnp_p3p_observe = 1;
        else if (!strcmp(argv[i], "--pnp_solver") && i + 1 < argc) {
            const char *name = argv[++i];
            if (!strcmp(name, "dlt"))
                c.pnp_p3p_fallback = 0;
            else if (!strcmp(name, "p3p-numeric"))
                c.pnp_p3p_fallback = 1;
            else
                fprintf(stderr, "unknown --pnp_solver %s (expected dlt or p3p-numeric)\n", name);
        }
        else if (!strcmp(argv[i], "--pnp_p3p_iterations") && i + 1 < argc)
            c.pnp_p3p_iterations = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_max_jump") && i + 1 < argc)
            c.pnp_p3p_max_jump = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_min_inl2") && i + 1 < argc)
            c.pnp_p3p_min_inl2 = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_min_gain") && i + 1 < argc)
            c.pnp_p3p_min_gain = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_max_mederr") && i + 1 < argc)
            c.pnp_p3p_max_mederr = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_p3p_min_posz") && i + 1 < argc)
            c.pnp_p3p_min_posz = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_iters") && i + 1 < argc)
            c.pnp_dlt_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_pretest") && i + 1 < argc)
            c.pnp_dlt_pretest = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_pretest_margin") && i + 1 < argc)
            c.pnp_dlt_pretest_margin = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_score_rigid"))
            c.pnp_score_rigid = 1;
        else if (!strcmp(argv[i], "--pnp_validate_rigid"))
            c.pnp_validate_rigid = 1;
        else if (!strcmp(argv[i], "--pnp_validate_rigid_min_points") && i + 1 < argc)
            c.pnp_validate_rigid_min_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_normalize_world"))
            c.pnp_normalize_world = 1;
        else if (!strcmp(argv[i], "--pnp_low_e_fallback"))
            c.pnp_low_e_fallback = 1;
        else if (!strcmp(argv[i], "--no_pnp_low_e_fallback"))
            c.pnp_low_e_fallback = 0;
        else if (!strcmp(argv[i], "--pnp_low_e_max_inliers") && i + 1 < argc)
            c.pnp_low_e_max_inliers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_low_e_min_inliers") && i + 1 < argc)
            c.pnp_low_e_min_inliers = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_low_e_min_gain") && i + 1 < argc)
            c.pnp_low_e_min_gain = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_low_e_min_jump") && i + 1 < argc)
            c.pnp_low_e_min_jump = atof(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_low_e_min_map_points") && i + 1 < argc)
            c.pnp_low_e_min_map_points = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_low_e_min_pose_jump") && i + 1 < argc)
            c.pnp_low_e_min_pose_jump = atof(argv[++i]);
        else if (!strcmp(argv[i], "--ffmpeg_gray"))
            c.ffmpeg_gray = 1;
        else if (!strcmp(argv[i], "--fast_corners"))
            c.fast_corners = 1;
        else if (!strcmp(argv[i], "--distributed_features"))
            c.distributed_features = 1;
        else if (!strcmp(argv[i], "--pyramid_features"))
            c.pyramid_features = 1;
        else if (!strcmp(argv[i], "--subpixel_features"))
            c.subpixel_features = 1;
        else if (!strcmp(argv[i], "--feature_min_dist") && i + 1 < argc)
            c.feature_min_dist = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--lk_iters") && i + 1 < argc)
            c.lk_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--lk_back_iters") && i + 1 < argc)
            c.lk_back_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pose_lm_iters") && i + 1 < argc)
            c.pose_lm_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--essential_iters") && i + 1 < argc)
            c.essential_iters = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--joint_ba"))
            c.joint_ba = 1;
        else if (argv[i][0] != '-')
            c.video_path = argv[i];
    }
    if (c.proc_w < 64)
        c.proc_w = 64;
    if (c.proc_h < 64)
        c.proc_h = 64;
    if (c.kf_min_inliers < 1)
        c.kf_min_inliers = 1;
    if (c.kf_period < 1)
        c.kf_period = 1;
    if (c.kf_min_interval < 0)
        c.kf_min_interval = 0;
    if (c.max_points < 50)
        c.max_points = 50;
    if (c.new_point_obs < 0)
        c.new_point_obs = 0;
    if (c.pnp_min_obs < 0)
        c.pnp_min_obs = 0;
    if (c.pnp_start_frame < 0)
        c.pnp_start_frame = 0;
    if (c.delayed_init_frames < 0)
        c.delayed_init_frames = 0;
    if (c.candidate_min_obs < 2)
        c.candidate_min_obs = 2;
    if (c.candidate_min_age < 1)
        c.candidate_min_age = 1;
    if (c.candidate_grid_cols < 1)
        c.candidate_grid_cols = 1;
    if (c.candidate_grid_rows < 1)
        c.candidate_grid_rows = 1;
    if (c.candidate_promote_per_cell < 0)
        c.candidate_promote_per_cell = 0;
    if (c.candidate_max_fb_err < 0.0)
        c.candidate_max_fb_err = 0.0;
    if (c.candidate_min_disp < 0.0)
        c.candidate_min_disp = 0.0;
    if (c.candidate_max_disp < 0.0)
        c.candidate_max_disp = 0.0;
    if (c.descriptor_admission_max_hamming < 0)
        c.descriptor_admission_max_hamming = 0;
    if (c.descriptor_admission_ratio < 0.0)
        c.descriptor_admission_ratio = 0.0;
    if (c.descriptor_admission_ratio > 1.0)
        c.descriptor_admission_ratio = 1.0;
    if (c.brief_patch_radius < 0)
        c.brief_patch_radius = 0;
    if (c.brief_patch_radius > 2)
        c.brief_patch_radius = 2;
    if (c.descriptor_primary_map_cap < 0)
        c.descriptor_primary_map_cap = 0;
    if (c.descriptor_source_kf_gap < 0)
        c.descriptor_source_kf_gap = 0;
    if (c.kf_warmup_frames < 0)
        c.kf_warmup_frames = 0;
    if (c.essential_cheirality_max < 0)
        c.essential_cheirality_max = 32;
    if (c.tri_min_parallax_deg < 0.0)
        c.tri_min_parallax_deg = 0.0;
    if (c.tri_max_reproj_px < 0.0)
        c.tri_max_reproj_px = 0.0;
    if (c.tri_max_depth < 0.0)
        c.tri_max_depth = 0.0;
    if (c.tri_max_depth_pnp < 0.0)
        c.tri_max_depth_pnp = 0.0;
    if (c.tri_max_depth_ratio < 0.0)
        c.tri_max_depth_ratio = 0.0;
    if (c.tri_source_kf_gap < 0)
        c.tri_source_kf_gap = 0;
    if (c.candidate_tracks) {
        if (c.tri_min_parallax_deg == 0.0)
            c.tri_min_parallax_deg = 0.5;
        if (c.tri_max_reproj_px == 0.0)
            c.tri_max_reproj_px = 8.0;
    }
    if (c.pnp_pred_reproj_gate < 0.0)
        c.pnp_pred_reproj_gate = 0.0;
    if (c.pnp_quality_gate_px < 0.0)
        c.pnp_quality_gate_px = 0.0;
    if (c.pnp_quality_min_obs < 1)
        c.pnp_quality_min_obs = 1;
    if (c.pnp_quality_window < 1)
        c.pnp_quality_window = 1;
    if (c.obs_stat_gate_px < 0.0)
        c.obs_stat_gate_px = 0.0;
    if (c.obs_stat_min_good < 0)
        c.obs_stat_min_good = 0;
    if (c.obs_stat_max_bad_ratio < 0.0)
        c.obs_stat_max_bad_ratio = 0.0;
    if (c.obs_stat_max_bad_ratio > 1.0)
        c.obs_stat_max_bad_ratio = 1.0;
    if (c.pnp_p3p_iterations < 1)
        c.pnp_p3p_iterations = 1;
    if (c.pnp_p3p_max_jump < 0.0)
        c.pnp_p3p_max_jump = 0.0;
    if (c.pnp_p3p_min_inl2 < 0)
        c.pnp_p3p_min_inl2 = 0;
    if (c.pnp_p3p_max_mederr < 0.0)
        c.pnp_p3p_max_mederr = 0.0;
    if (c.pnp_p3p_min_posz < 0.0)
        c.pnp_p3p_min_posz = 0.0;
    if (c.pnp_dlt_iters < 1)
        c.pnp_dlt_iters = 1;
    if (c.pnp_dlt_pretest < 0)
        c.pnp_dlt_pretest = 0;
    if (c.pnp_dlt_pretest_margin < 0)
        c.pnp_dlt_pretest_margin = 0;
    if (c.pnp_validate_rigid_min_points < 0)
        c.pnp_validate_rigid_min_points = 0;
    if (c.pnp_low_e_max_inliers < 0)
        c.pnp_low_e_max_inliers = 0;
    if (c.pnp_low_e_min_inliers < 0)
        c.pnp_low_e_min_inliers = 0;
    if (c.pnp_low_e_min_gain < 0)
        c.pnp_low_e_min_gain = 0;
    if (c.pnp_low_e_min_jump < 0.0)
        c.pnp_low_e_min_jump = 0.0;
    if (c.pnp_low_e_min_map_points < 0)
        c.pnp_low_e_min_map_points = 0;
    if (c.pnp_low_e_min_pose_jump < 0.0)
        c.pnp_low_e_min_pose_jump = 0.0;
    if (c.lk_iters < 1)
        c.lk_iters = 1;
    if (c.lk_back_iters < 0)
        c.lk_back_iters = 0;
    if (c.pose_lm_iters < 1)
        c.pose_lm_iters = 1;
    if (c.essential_iters < 1)
        c.essential_iters = 1;
    if (c.e_shape_max_matches < 0)
        c.e_shape_max_matches = 0;
    if (c.e_shape_grid_cap < 0)
        c.e_shape_grid_cap = 0;
    if (c.e_shape_max_disp < 0.0)
        c.e_shape_max_disp = 0.0;
    if (c.e_shape_max_fb_err < 0.0)
        c.e_shape_max_fb_err = 0.0;
    if (c.e_shape_target_disp < 0.0)
        c.e_shape_target_disp = 0.0;
    if (c.anchor_max_features < 0)
        c.anchor_max_features = 0;
    if (c.anchor_max_hamming < 0)
        c.anchor_max_hamming = 0;
    if (c.anchor_ratio < 0.0)
        c.anchor_ratio = 0.0;
    if (c.anchor_ratio > 1.0)
        c.anchor_ratio = 1.0;
    if (c.feature_min_dist < 0)
        c.feature_min_dist = 0;
    if (c.admission_target_disp < 0.0)
        c.admission_target_disp = 0.0;
    if (c.admission_fb_weight < 0.0)
        c.admission_fb_weight = 0.0;
    if (c.admission_min_inliers < 0)
        c.admission_min_inliers = 0;
    if (c.admission_pnp_min_inliers < 0)
        c.admission_pnp_min_inliers = 0;
    if (c.admission_max_new_points < 0)
        c.admission_max_new_points = 0;
    if (c.admission_grid_cap < 0)
        c.admission_grid_cap = 0;
    if (c.output_smooth_alpha < 0.0)
        c.output_smooth_alpha = 0.0;
    if (c.output_smooth_alpha > 1.0)
        c.output_smooth_alpha = 1.0;
    if (c.output_smooth_outlier_alpha < 0.0)
        c.output_smooth_outlier_alpha = 0.0;
    if (c.output_smooth_outlier_alpha > 1.0)
        c.output_smooth_outlier_alpha = 1.0;
    if (c.output_smooth_residual_k < 0.0)
        c.output_smooth_residual_k = 0.0;
    if (c.output_smooth_window < 1)
        c.output_smooth_window = 1;
    if (c.output_smooth_window > 64)
        c.output_smooth_window = 64;
    if (c.output_smooth_unstable_alpha < 0.0)
        c.output_smooth_unstable_alpha = 0.0;
    if (c.output_smooth_unstable_alpha > 1.0)
        c.output_smooth_unstable_alpha = 1.0;
    if (c.output_smooth_unstable_residual_k < 0.0)
        c.output_smooth_unstable_residual_k = 0.0;
    if (c.output_smooth_unstable_window < 0)
        c.output_smooth_unstable_window = 0;
    if (c.output_smooth_unstable_window > 64)
        c.output_smooth_unstable_window = 64;
    if (c.output_smooth_unstable_points_added < 0)
        c.output_smooth_unstable_points_added = 0;
    if (c.output_smooth_unstable_jump < 0.0)
        c.output_smooth_unstable_jump = 0.0;
    if (c.output_smooth_unstable_jump_count < 0)
        c.output_smooth_unstable_jump_count = 0;
    c.output_smooth_unstable_cap_residual =
        c.output_smooth_unstable_cap_residual ? 1 : 0;
    if (c.output_smooth_low_link_alpha < 0.0)
        c.output_smooth_low_link_alpha = 0.0;
    if (c.output_smooth_low_link_alpha > 1.0)
        c.output_smooth_low_link_alpha = 1.0;
    if (c.output_smooth_low_link_threshold < 0)
        c.output_smooth_low_link_threshold = 0;
    if (c.output_smooth_low_link_window < 0)
        c.output_smooth_low_link_window = 0;
    if (c.output_smooth_low_link_window > 64)
        c.output_smooth_low_link_window = 64;
    if (c.output_smooth_low_link_count < 0)
        c.output_smooth_low_link_count = 0;
    c.output_smooth_low_link_include_current =
        c.output_smooth_low_link_include_current ? 1 : 0;
    return c;
}

#endif
