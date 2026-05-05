#ifndef SIMPLE_SLAM_C_PLUS_PNP_H
#define SIMPLE_SLAM_C_PLUS_PNP_H

typedef struct {
    int inliers2;
    int inliers3;
    int inliers5;
    double median_error;
    double positive_depth_ratio;
} PnPScore;
typedef struct {
    int attempted;
    int solved;
    int accepted;
    int inliers;
    int inliers2;
    int inliers3;
    int inliers5;
    double median_error;
    double positive_depth_ratio;
    double predicted_jump;
    double center[3];
} PnPProbe;

static int count_pnp_linear_inliers(const Map *map, const CornerVec *corners, const int *ids,
                                    int n, double fx, double fy, double cx, double cy,
                                    const double P[12], int max_count) {
    double R[9] = {P[0], P[1], P[2], P[4], P[5], P[6], P[8], P[9], P[10]},
           t[3] = {P[3], P[7], P[11]};
    double norm = sqrt(R[6] * R[6] + R[7] * R[7] + R[8] * R[8]);
    if (!isfinite(norm) || norm < 1e-12)
        return 0;
    double sc = 1.0 / norm;
    if (P[11] * sc < 0)
        sc = -sc;
    for (int j = 0; j < 9; j++)
        R[j] *= sc;
    for (int j = 0; j < 3; j++)
        t[j] *= sc;
    int limit = max_count > 0 && max_count < n ? max_count : n;
    int inl = 0;
    for (int j = 0; j < limit; j++) {
        MapPoint p = map->data[corners->data[ids[j]].pt_idx];
        double cx_p = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
               cy_p = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
               cz_p = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (cz_p < 0.1)
            continue;
        double u = fx * cx_p / cz_p + cx, v = fy * cy_p / cz_p + cy;
        double dx = u - corners->data[ids[j]].x, dy = v - corners->data[ids[j]].y;
        if (dx * dx + dy * dy < 4.0)
            inl++;
    }
    return inl;
}

static int estimate_pose_PnP(const Map *map, const CornerVec *corners, double fx, double fy,
                             double cx, double cy, int dlt_iters, int dlt_pretest,
                             int dlt_pretest_margin, int min_obs, const unsigned char *point_ok,
                             Pose *out_pose, int *out_inl) {
    if (out_inl)
        *out_inl = 0;
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1)
            n++;
    if (n < 12)
        return 0;
    int *ids = malloc(n * sizeof(int));
    int k = 0;
    for (int i = 0; i < corners->size; i++) {
        int pi = corners->data[i].pt_idx;
        if (pi != -1 && map->data[pi].obs >= min_obs && (!point_ok || point_ok[pi]))
            ids[k++] = i;
    }
    n = k;
    if (n < 12) {
        free(ids);
        return 0;
    }
    double best_P[12];
    int best_inl = 0;
    int best_pre = 0;
    int iters = dlt_iters > 0 ? dlt_iters : 500;
    srand(0);
    for (int it = 0; it < iters; it++) {
        double AtA[144] = {0}, W[12], V[144], P[12];
        for (int i = 0; i < 6; i++) {
            int idx = ids[rand() % n];
            MapPoint p = map->data[corners->data[idx].pt_idx];
            double u = (corners->data[idx].x - cx) / fx, v = (corners->data[idx].y - cy) / fy;
            double r1[12] = {p.x, p.y, p.z, 1,   0,   0,   0,   0,  -u*p.x, -u*p.y, -u*p.z, -u};
            double r2[12] = {  0,   0,   0, 0, p.x, p.y, p.z, 1,   -v*p.x, -v*p.y, -v*p.z, -v};
            for (int r = 0; r < 12; r++)
                for (int c = 0; c < 12; c++)
                    AtA[r * 12 + c] += r1[r] * r1[c] + r2[r] * r2[c];
        }
        jacobi_12x12(AtA, W, V);
        int bi = 0;
        double mw = W[0];
        for (int j = 1; j < 12; j++)
            if (W[j] < mw) {
                mw = W[j];
                bi = j;
            }
        for (int j = 0; j < 12; j++)
            P[j] = V[j * 12 + bi];
        int pre_inl = n;
        if (dlt_pretest > 0) {
            pre_inl = count_pnp_linear_inliers(map, corners, ids, n, fx, fy, cx, cy, P,
                                               dlt_pretest);
            if (best_pre > 0 && pre_inl + dlt_pretest_margin < best_pre)
                continue;
        }
        int inl = count_pnp_linear_inliers(map, corners, ids, n, fx, fy, cx, cy, P, 0);
        if (inl > best_inl) {
            best_inl = inl;
            best_pre = pre_inl;
            memcpy(best_P, P, 12 * sizeof(double));
        }
        if (inl > n * 0.8)
            break;
    }
    if (out_inl)
        *out_inl = best_inl;
    if (best_inl >= 12) {
        double R[9] = {best_P[0], best_P[1], best_P[2], best_P[4], best_P[5],
                       best_P[6], best_P[8], best_P[9], best_P[10]},
               t[3] = {best_P[3], best_P[7], best_P[11]};
        double sc = 1.0 / sqrt(R[6] * R[6] + R[7] * R[7] + R[8] * R[8]);
        if (best_P[11] * sc < 0)
            sc = -sc;
        for (int j = 0; j < 9; j++)
            R[j] *= sc;
        for (int j = 0; j < 3; j++)
            t[j] *= sc;
        double W[3], U[9], V[9], Ro[9];
        svd_3x3(R, W, U, V);
        double VT[9];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                VT[i * 3 + j] = V[j * 3 + i];
        mat3_mul(U, VT, Ro);
        if (mat3_det(Ro) < 0)
            for (int j = 0; j < 9; j++)
                Ro[j] = -Ro[j];
        pose_from_rt(Ro, t, out_pose);
        free(ids);
        return 1;
    }
    free(ids);
    return 0;
}

static int pose_from_3d3d(double Pw[3][3], double Pc[3][3], Pose *pose) {
    double cw[3] = {0}, cc[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            cw[k] += Pw[i][k];
            cc[k] += Pc[i][k];
        }
    }
    for (int k = 0; k < 3; k++) {
        cw[k] /= 3.0;
        cc[k] /= 3.0;
    }
    double H[9] = {0};
    for (int i = 0; i < 3; i++) {
        double pw[3] = {Pw[i][0] - cw[0], Pw[i][1] - cw[1], Pw[i][2] - cw[2]};
        double pc[3] = {Pc[i][0] - cc[0], Pc[i][1] - cc[1], Pc[i][2] - cc[2]};
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                H[r * 3 + c] += pc[r] * pw[c];
    }
    double W[3], U[9], V[9], VT[9], R[9];
    svd_3x3(H, W, U, V);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            VT[i * 3 + j] = V[j * 3 + i];
    mat3_mul(U, VT, R);
    if (mat3_det(R) < 0) {
        for (int r = 0; r < 3; r++)
            U[r * 3 + 2] = -U[r * 3 + 2];
        mat3_mul(U, VT, R);
    }
    double t[3] = {
        cc[0] - (R[0] * cw[0] + R[1] * cw[1] + R[2] * cw[2]),
        cc[1] - (R[3] * cw[0] + R[4] * cw[1] + R[5] * cw[2]),
        cc[2] - (R[6] * cw[0] + R[7] * cw[1] + R[8] * cw[2]),
    };
    if (!isfinite(t[0]) || !isfinite(t[1]) || !isfinite(t[2]))
        return 0;
    pose_from_rt(R, t, pose);
    return 1;
}

static int numeric_p3p_depths(double f[3][3], double Pw[3][3], const double seed[3],
                              double out_lam[3]) {
    const double d01 = vec3_dist(Pw[0], Pw[1]);
    const double d02 = vec3_dist(Pw[0], Pw[2]);
    const double d12 = vec3_dist(Pw[1], Pw[2]);
    const double c01 = vec3_dot(f[0], f[1]);
    const double c02 = vec3_dot(f[0], f[2]);
    const double c12 = vec3_dot(f[1], f[2]);
    double l[3] = {seed[0], seed[1], seed[2]};
    if (l[0] <= 0 || l[1] <= 0 || l[2] <= 0)
        return 0;
    for (int iter = 0; iter < 30; iter++) {
        double r[3] = {
            l[0] * l[0] + l[1] * l[1] - 2.0 * l[0] * l[1] * c01 - d01 * d01,
            l[0] * l[0] + l[2] * l[2] - 2.0 * l[0] * l[2] * c02 - d02 * d02,
            l[1] * l[1] + l[2] * l[2] - 2.0 * l[1] * l[2] * c12 - d12 * d12,
        };
        double J[9] = {
            2 * l[0] - 2 * l[1] * c01, 2 * l[1] - 2 * l[0] * c01, 0,
            2 * l[0] - 2 * l[2] * c02, 0, 2 * l[2] - 2 * l[0] * c02,
            0, 2 * l[1] - 2 * l[2] * c12, 2 * l[2] - 2 * l[1] * c12,
        };
        double A[9] = {0}, b[3] = {0};
        for (int row = 0; row < 3; row++) {
            for (int a = 0; a < 3; a++) {
                b[a] -= J[row * 3 + a] * r[row];
                for (int c = 0; c < 3; c++)
                    A[a * 3 + c] += J[row * 3 + a] * J[row * 3 + c];
            }
        }
        for (int k = 0; k < 3; k++)
            A[k * 3 + k] += 1e-6;
        double dl[3];
        if (!solve_3x3(A, b, dl))
            return 0;
        double step2 = dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2];
        for (int k = 0; k < 3; k++) {
            l[k] += dl[k];
            if (l[k] <= 1e-6 || !isfinite(l[k]))
                return 0;
        }
        if (step2 < 1e-12)
            break;
    }
    double err = fabs(l[0] * l[0] + l[1] * l[1] - 2.0 * l[0] * l[1] * c01 - d01 * d01) +
                 fabs(l[0] * l[0] + l[2] * l[2] - 2.0 * l[0] * l[2] * c02 - d02 * d02) +
                 fabs(l[1] * l[1] + l[2] * l[2] - 2.0 * l[1] * l[2] * c12 - d12 * d12);
    double scale = d01 * d01 + d02 * d02 + d12 * d12 + 1e-9;
    if (err / scale > 1e-4)
        return 0;
    out_lam[0] = l[0];
    out_lam[1] = l[1];
    out_lam[2] = l[2];
    return 1;
}

static int p3p_numeric_pose_from_sample(const Map *map, const CornerVec *corners,
                                        const int sample_ids[4], double fx, double fy,
                                        double cx, double cy, Pose *best_pose) {
    double f[3][3], Pw[3][3];
    for (int i = 0; i < 3; i++) {
        int idx = sample_ids[i];
        MapPoint p = map->data[corners->data[idx].pt_idx];
        Pw[i][0] = p.x;
        Pw[i][1] = p.y;
        Pw[i][2] = p.z;
        f[i][0] = (corners->data[idx].x - cx) / fx;
        f[i][1] = (corners->data[idx].y - cy) / fy;
        f[i][2] = 1.0;
        vec3_normalize(f[i]);
    }
    double side = (vec3_dist(Pw[0], Pw[1]) + vec3_dist(Pw[0], Pw[2]) +
                   vec3_dist(Pw[1], Pw[2])) / 3.0;
    if (!isfinite(side) || side < 1e-9)
        return 0;
    const double scales[] = {0.35, 0.7, 1.0, 1.4, 2.5, 5.0};
    const double ratios[][3] = {
        {1, 1, 1}, {0.7, 1, 1.3}, {1.3, 1, 0.7}, {1, 0.7, 1.3}, {1, 1.3, 0.7},
    };
    int found = 0;
    double best_err = 1e300;
    for (int si = 0; si < (int)(sizeof(scales) / sizeof(scales[0])); si++) {
        for (int ri = 0; ri < (int)(sizeof(ratios) / sizeof(ratios[0])); ri++) {
            double seed[3] = {
                side * scales[si] * ratios[ri][0],
                side * scales[si] * ratios[ri][1],
                side * scales[si] * ratios[ri][2],
            };
            double lam[3];
            if (!numeric_p3p_depths(f, Pw, seed, lam))
                continue;
            double Pc[3][3];
            for (int i = 0; i < 3; i++) {
                Pc[i][0] = lam[i] * f[i][0];
                Pc[i][1] = lam[i] * f[i][1];
                Pc[i][2] = lam[i] * f[i][2];
            }
            Pose pose;
            if (!pose_from_3d3d(Pw, Pc, &pose))
                continue;
            int idx4 = sample_ids[3];
            MapPoint p4 = map->data[corners->data[idx4].pt_idx];
            double x = pose.m[0] * p4.x + pose.m[1] * p4.y + pose.m[2] * p4.z + pose.m[3];
            double y = pose.m[4] * p4.x + pose.m[5] * p4.y + pose.m[6] * p4.z + pose.m[7];
            double z = pose.m[8] * p4.x + pose.m[9] * p4.y + pose.m[10] * p4.z + pose.m[11];
            if (z < 0.1)
                continue;
            double u = fx * x / z + cx, v = fy * y / z + cy;
            double du = u - corners->data[idx4].x, dv = v - corners->data[idx4].y;
            double err = du * du + dv * dv;
            if (err < best_err) {
                best_err = err;
                *best_pose = pose;
                found = 1;
            }
        }
    }
    return found;
}

static void sample_unique4(const int *ids, int n, int out[4]) {
    for (int i = 0; i < 4; i++) {
        int pick, duplicate;
        do {
            duplicate = 0;
            pick = ids[rand() % n];
            for (int j = 0; j < i; j++) {
                if (out[j] == pick) {
                    duplicate = 1;
                    break;
                }
            }
        } while (duplicate);
        out[i] = pick;
    }
}

static int cmp_double(const void *a, const void *b) {
    double da = *(const double *)a, db = *(const double *)b;
    return (da > db) - (da < db);
}

static PnPScore score_pnp_pose(const Map *map, const CornerVec *corners, double fx, double fy,
                               double cx, double cy, const Pose *pose) {
    PnPScore score = {0};
    if (corners->size <= 0)
        return score;
    double *errors = malloc((size_t)corners->size * sizeof(double));
    int nerr = 0, pos = 0, total = 0;
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
            continue;
        total++;
        MapPoint p = map->data[corners->data[i].pt_idx];
        double x = pose->m[0] * p.x + pose->m[1] * p.y + pose->m[2] * p.z + pose->m[3];
        double y = pose->m[4] * p.x + pose->m[5] * p.y + pose->m[6] * p.z + pose->m[7];
        double z = pose->m[8] * p.x + pose->m[9] * p.y + pose->m[10] * p.z + pose->m[11];
        if (z > 0.1)
            pos++;
        if (z < 0.1)
            continue;
        double u = fx * x / z + cx, v = fy * y / z + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        double err = sqrt(dx * dx + dy * dy);
        errors[nerr++] = err;
        if (err < 2.0)
            score.inliers2++;
        if (err < 3.0)
            score.inliers3++;
        if (err < 5.0)
            score.inliers5++;
    }
    if (nerr > 0) {
        qsort(errors, (size_t)nerr, sizeof(double), cmp_double);
        score.median_error = errors[nerr / 2];
    }
    score.positive_depth_ratio = total > 0 ? (double)pos / (double)total : 0.0;
    free(errors);
    return score;
}

static int count_pose_inliers(const Map *map, const CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose);

static int estimate_pose_PnP_p3p_numeric(const Map *map, const CornerVec *corners, double fx,
                                         double fy, double cx, double cy, const Pose *predicted,
                                         int baseline_inliers, int iterations, double max_jump,
                                         int min_inl2, int min_gain, double max_mederr,
                                         double min_posz,
                                         Pose *out_pose, int *out_inl, PnPProbe *probe) {
    if (out_inl)
        *out_inl = 0;
    if (probe)
        memset(probe, 0, sizeof(*probe));
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1 && map->data[corners->data[i].pt_idx].obs > 0)
            n++;
    if (n < 12)
        return 0;
    if (probe)
        probe->attempted = 1;
    int *ids = malloc((size_t)n * sizeof(int));
    int k = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1 && map->data[corners->data[i].pt_idx].obs > 0)
            ids[k++] = i;

    Pose best_pose;
    int best_inl = 0;
    int iters = iterations > 0 ? iterations : 500;
    srand(0);
    for (int it = 0; it < iters; it++) {
        int sample[4];
        sample_unique4(ids, n, sample);
        Pose pose;
        if (!p3p_numeric_pose_from_sample(map, corners, sample, fx, fy, cx, cy, &pose))
            continue;
        int inl = count_pose_inliers(map, corners, fx, fy, cx, cy, &pose);
        if (inl > best_inl) {
            best_inl = inl;
            best_pose = pose;
        }
        if (inl > n * 0.8)
            break;
    }
    free(ids);
    if (out_inl)
        *out_inl = best_inl;
    if (probe)
        probe->inliers = best_inl;
    if (best_inl < 16)
        return 0;

    PnPScore score = score_pnp_pose(map, corners, fx, fy, cx, cy, &best_pose);
    if (probe) {
        probe->solved = 1;
        probe->inliers2 = score.inliers2;
        probe->inliers3 = score.inliers3;
        probe->inliers5 = score.inliers5;
        probe->median_error = score.median_error;
        probe->positive_depth_ratio = score.positive_depth_ratio;
        camera_center_from_pose(&best_pose, probe->center);
    }
    double predicted_jump = 0.0;
    if (predicted) {
        double a[3], b[3];
        camera_center_from_pose(predicted, a);
        camera_center_from_pose(&best_pose, b);
        predicted_jump = vec3_dist(a, b);
        if (probe)
            probe->predicted_jump = predicted_jump;
    }
    if (score.inliers2 < min_inl2 || score.inliers2 - baseline_inliers < min_gain ||
        score.median_error > max_mederr || score.positive_depth_ratio < min_posz)
        return 0;
    if (predicted && max_jump > 0.0 && predicted_jump > max_jump)
        return 0;
    *out_pose = best_pose;
    if (probe)
        probe->accepted = 1;
    return 1;
}

static int count_pose_inliers(const Map *map, const CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    int inl = 0;
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
            continue;
        MapPoint p = map->data[corners->data[i].pt_idx];
        double px = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0];
        double py = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1];
        double pz = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (pz < 0.1)
            continue;
        double u = fx * px / pz + cx, v = fy * py / pz + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        if (dx * dx + dy * dy < 4.0)
            inl++;
    }
    return inl;
}

static int gate_links_by_pose(const Map *map, CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose, double gate_px) {
    if (gate_px <= 0.0)
        return 0;
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double th2 = gate_px * gate_px;
    int removed = 0;
    for (int i = 0; i < corners->size; i++) {
        int pi = corners->data[i].pt_idx;
        if (pi < 0 || pi >= map->size)
            continue;
        MapPoint p = map->data[pi];
        double x = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
               y = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
               z = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (z < 0.1) {
            corners->data[i].pt_idx = -1;
            removed++;
            continue;
        }
        double u = fx * x / z + cx, v = fy * y / z + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        if (dx * dx + dy * dy > th2) {
            corners->data[i].pt_idx = -1;
            removed++;
        }
    }
    return removed;
}

static double reprojection_error_point(const MapPoint *p, const Corner *corner, double fx,
                                       double fy, double cx, double cy, const Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double x = R[0] * p->x + R[1] * p->y + R[2] * p->z + t[0],
           y = R[3] * p->x + R[4] * p->y + R[5] * p->z + t[1],
           z = R[6] * p->x + R[7] * p->y + R[8] * p->z + t[2];
    if (z < 0.1)
        return INFINITY;
    double u = fx * x / z + cx, v = fy * y / z + cy;
    double dx = u - corner->x, dy = v - corner->y;
    return sqrt(dx * dx + dy * dy);
}

static void update_observation_stats(Map *map, const CornerVec *corners, double fx, double fy,
                                     double cx, double cy, const Pose *pose, double gate_px) {
    if (gate_px <= 0.0)
        return;
    for (int i = 0; i < corners->size; i++) {
        int pi = corners->data[i].pt_idx;
        if (pi < 0 || pi >= map->size)
            continue;
        double err = reprojection_error_point(&map->data[pi], &corners->data[i], fx, fy, cx, cy, pose);
        if (err <= gate_px) {
            if (map->data[pi].good_obs < 65535)
                map->data[pi].good_obs++;
        } else {
            if (map->data[pi].bad_obs < 65535)
                map->data[pi].bad_obs++;
        }
    }
}

static unsigned char *build_observation_stat_mask(const Map *map, int min_good,
                                                  double max_bad_ratio) {
    if (min_good <= 0 && max_bad_ratio >= 1.0)
        return NULL;
    unsigned char *ok = malloc((size_t)map->size);
    if (!ok)
        return NULL;
    for (int i = 0; i < map->size; i++) {
        int total = (int)map->data[i].good_obs + (int)map->data[i].bad_obs;
        double bad_ratio = total > 0 ? (double)map->data[i].bad_obs / (double)total : 1.0;
        ok[i] = map->data[i].good_obs >= min_good && bad_ratio <= max_bad_ratio;
    }
    return ok;
}

static double pose_reprojection_rmse(const Map *map, const CornerVec *corners, double fx,
                                     double fy, double cx, double cy, const Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
            continue;
        MapPoint p = map->data[corners->data[i].pt_idx];
        double px = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0];
        double py = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1];
        double pz = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
        if (pz < 0.1)
            continue;
        double u = fx * px / pz + cx, v = fy * py / pz + cy;
        double dx = u - corners->data[i].x, dy = v - corners->data[i].y;
        sum += dx * dx + dy * dy;
        n++;
    }
    return n ? sqrt(sum / n) : 0.0;
}

static void dump_pnp_frame(FILE *f, int frame_id, const Map *map, const CornerVec *corners,
                           double fx, double fy, double cx, double cy, const Pose *predicted,
                           const Pose *pnp_pose, int pnp_inliers, const Pose *lm_pose) {
    if (!f)
        return;
    int n = 0;
    for (int i = 0; i < corners->size; i++)
        if (corners->data[i].pt_idx != -1 && map->data[corners->data[i].pt_idx].obs > 0)
            n++;

    double pred_t[3], pnp_t[3] = {0}, lm_t[3] = {0};
    pose_get_translation(predicted, pred_t);
    if (pnp_pose)
        pose_get_translation(pnp_pose, pnp_t);
    if (lm_pose)
        pose_get_translation(lm_pose, lm_t);
    double pnp_rmse = pnp_pose ? pose_reprojection_rmse(map, corners, fx, fy, cx, cy, pnp_pose) : 0.0;
    double lm_rmse = lm_pose ? pose_reprojection_rmse(map, corners, fx, fy, cx, cy, lm_pose) : 0.0;

    fprintf(f,
            "FRAME %d %.17g %.17g %.17g %.17g %d %.17g %.17g %.17g %d %d %.17g %.17g %.17g %.17g "
            "%.17g %.17g %.17g %.17g\n",
            frame_id, fx, fy, cx, cy, n, pred_t[0], pred_t[1], pred_t[2], pnp_pose ? 1 : 0,
            pnp_inliers, pnp_t[0], pnp_t[1], pnp_t[2], pnp_rmse, lm_t[0], lm_t[1], lm_t[2],
            lm_rmse);
    for (int i = 0; i < corners->size; i++) {
        if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs <= 0)
            continue;
        MapPoint p = map->data[corners->data[i].pt_idx];
        fprintf(f, "C %.9g %.9g %.17g %.17g %.17g %d\n", corners->data[i].x,
                corners->data[i].y, p.x, p.y, p.z, p.obs);
    }
    fprintf(f, "END\n");
}

static void refine_pose_lm(const Map *map, const CornerVec *corners, double fx, double fy,
                           double cx, double cy, int max_iters, Pose *pose) {
    double R[9], t[3];
    pose_get_rotation(pose, R);
    pose_get_translation(pose, t);
    double lambda = 1e-3;
    int iters = max_iters > 0 ? max_iters : 10;
    for (int iter = 0; iter < iters; iter++) {
        double H[36] = {0}, b[6] = {0};
        for (int i = 0; i < corners->size; i++) {
            if (corners->data[i].pt_idx == -1 || map->data[corners->data[i].pt_idx].obs == 0)
                continue;
            MapPoint p = map->data[corners->data[i].pt_idx];

            // Project point P into camera frame:  cp = R·P + t
            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            if (cp[2] < 0.1)
                continue;

            // Pinhole projection and reprojection residual:
            double inv_z  = 1.0 / cp[2];
            double inv_z2 = inv_z * inv_z;
            double u  = fx*cp[0]*inv_z + cx;
            double v  = fy*cp[1]*inv_z + cy;
            double du = corners->data[i].x - u;
            double dv = corners->data[i].y - v;

            // Huber-ish weight on reprojection error.
            double err2   = du*du + dv*dv;
            double weight = (err2 > 4.0) ? 2.0 / sqrt(err2) : 1.0;
            double J[2][6] = {
                {fx*inv_z, 0,        -fx*cp[0]*inv_z2,  -fx*cp[0]*cp[1]*inv_z2,             fx*(1 + cp[0]*cp[0]*inv_z2),  -fx*cp[1]*inv_z},
                {0,        fy*inv_z, -fy*cp[1]*inv_z2,  -fy*(1 + cp[1]*cp[1]*inv_z2),       fy*cp[0]*cp[1]*inv_z2,         fy*cp[0]*inv_z},
            };
            for (int r = 0; r < 6; r++) {
                b[r] += weight * (J[0][r] * du + J[1][r] * dv);
                for (int c = 0; c < 6; c++)
                    H[r * 6 + c] += weight * (J[0][r] * J[0][c] + J[1][r] * J[1][c]);
            }
        }

        for (int i = 0; i < 6; i++)
            H[i * 6 + i] += lambda * H[i * 6 + i] + 1e-6;
        double dx[6];
        if (!solve_6x6(H, b, dx))
            break;
        t[0] += dx[0];
        t[1] += dx[1];
        t[2] += dx[2];
        double dr[9] = {1, -dx[5], dx[4],
                        dx[5], 1, -dx[3],
                        -dx[4], dx[3], 1};
        double Rn[9];
        mat3_mul(dr, R, Rn);
        double W[3], U[9], V[9], VT[9];
        svd_3x3(Rn, W, U, V);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                VT[i * 3 + j] = V[j * 3 + i];
        mat3_mul(U, VT, R);
        if (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2] < 1e-8)
            break;
    }
    pose_from_rt(R, t, pose);
}

#endif
