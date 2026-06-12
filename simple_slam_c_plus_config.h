#ifndef SIMPLE_SLAM_C_PLUS_CONFIG_H
#define SIMPLE_SLAM_C_PLUS_CONFIG_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    int joint_ba;
    int proc_w, proc_h;
    int kf_period;
    int kf_min_interval;
    int ba_interval;
    int ba_start_keyframes;
    int ba_max_points;
    int local_ba_fix_oldest;
    int global_ba_interval;
    int global_ba_start_keyframes;
    int global_ba_max_points;
    int essential_cheirality_max;
    int pnp_dlt_iters;
    int pnp_low_e_fallback;
    int pnp_low_e_max_inliers;
    int pnp_low_e_min_inliers;
    int pnp_low_e_min_gain;
    double pnp_low_e_min_jump;
    int pnp_low_e_min_map_points;
    double pnp_low_e_min_pose_jump;
    int ffmpeg_gray;
    int lk_iters;
    int lk_back_iters;
    int pose_lm_iters;
    int essential_iters;
    int oriented_brief;
    int brief_patch_radius;
    int anchor_e_pose;
    int anchor_max_features;
    int anchor_max_hamming;
    int anchor_mutual;
    double anchor_ratio;
    int normalize_world_scale;
    int tri_map_scale;
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

static int should_make_keyframe(const Config *c, int frame_id, int inliers, double rot_deg) {
    if (rot_deg > c->kf_max_rot_deg)
        return 1;
    if (frame_id % c->kf_period == 0)
        return 1;
    if (inliers < c->kf_min_inliers)
        return 1;
    return 0;
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
        .proc_w = 640,
        .proc_h = 480,
        .kf_period = 20,
        .kf_min_interval = 2,
        .ba_interval = 1,
        .local_ba_fix_oldest = 0,
        .global_ba_interval = 10,
        .essential_cheirality_max = 32,
        .pnp_dlt_iters = 500,
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
        .oriented_brief = 1,
        .brief_patch_radius = 0,
        .anchor_e_pose = 1,
        .anchor_max_features = 600,
        .anchor_max_hamming = 80,
        .anchor_mutual = 0,
        .anchor_ratio = 0.80,
        .normalize_world_scale = 0,
        .tri_map_scale = 0,
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
        else if (!strcmp(argv[i], "--oriented_brief"))
            c.oriented_brief = 1;
        else if (!strcmp(argv[i], "--brief_patch_radius") && i + 1 < argc)
            c.brief_patch_radius = atoi(argv[++i]);
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
        else if (!strcmp(argv[i], "--normalize_world_scale"))
            c.normalize_world_scale = 1;
        else if (!strcmp(argv[i], "--tri_map_scale"))
            c.tri_map_scale = 1;
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
        else if (!strcmp(argv[i], "--kf_period") && i + 1 < argc)
            c.kf_period = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--kf_min_interval") && i + 1 < argc)
            c.kf_min_interval = atoi(argv[++i]);
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
        else if (!strcmp(argv[i], "--essential_cheirality_max") && i + 1 < argc)
            c.essential_cheirality_max = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pnp_dlt_iters") && i + 1 < argc)
            c.pnp_dlt_iters = atoi(argv[++i]);
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
    if (c.brief_patch_radius < 0)
        c.brief_patch_radius = 0;
    if (c.brief_patch_radius > 2)
        c.brief_patch_radius = 2;
    if (c.essential_cheirality_max < 0)
        c.essential_cheirality_max = 32;
    if (c.pnp_dlt_iters < 1)
        c.pnp_dlt_iters = 1;
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
    if (c.anchor_max_features < 0)
        c.anchor_max_features = 0;
    if (c.anchor_max_hamming < 0)
        c.anchor_max_hamming = 0;
    if (c.anchor_ratio < 0.0)
        c.anchor_ratio = 0.0;
    if (c.anchor_ratio > 1.0)
        c.anchor_ratio = 1.0;
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
