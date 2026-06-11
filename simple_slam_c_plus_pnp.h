#ifndef SIMPLE_SLAM_C_PLUS_PNP_H
#define SIMPLE_SLAM_C_PLUS_PNP_H

static int count_pnp_linear_inliers(const Map *map, const CornerVec *corners, const int *ids,
                                    int n, double fx, double fy, double cx, double cy,
                                    const double P[12]) {
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
    int limit = n;
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

static int count_pose_inliers(const Map *map, const CornerVec *corners, double fx, double fy,
                              double cx, double cy, const Pose *pose);

static int dlt_projection_to_pose(const double P[12], Pose *out_pose) {
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
    return 1;
}

static int estimate_pose_PnP(const Map *map, const CornerVec *corners, double fx, double fy,
                             double cx, double cy, int dlt_iters, int min_obs,
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
        if (pi != -1 && map->data[pi].obs >= min_obs)
            ids[k++] = i;
    }
    n = k;
    if (n < 12) {
        free(ids);
        return 0;
    }
    double best_P[12];
    int best_inl = 0;
    int iters = dlt_iters > 0 ? dlt_iters : 500;
    srand(g_ransac_seed);
    for (int it = 0; it < iters; it++) {
        double AtA[144] = {0}, W[12], V[144], P[12];
        int sample_ids[6];
        for (int i = 0; i < 6; i++)
            sample_ids[i] = ids[rand() % n];
        for (int i = 0; i < 6; i++) {
            int idx = sample_ids[i];
            MapPoint p = map->data[corners->data[idx].pt_idx];
            double u = (corners->data[idx].x - cx) / fx, v = (corners->data[idx].y - cy) / fy;
            double r1[12] = {p.x, p.y, p.z, 1,  0,   0,   0,   0, -u*p.x, -u*p.y, -u*p.z, -u};
            double r2[12] = {0,   0,   0,   0,  p.x, p.y, p.z, 1, -v*p.x, -v*p.y, -v*p.z, -v};
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
        int inl = count_pnp_linear_inliers(map, corners, ids, n, fx, fy, cx, cy, P);
        if (inl > best_inl) {
            best_inl = inl;
            memcpy(best_P, P, 12 * sizeof(double));
        }
        if (inl > n * 0.8)
            break;
    }
    if (out_inl)
        *out_inl = best_inl;
    int ok = 0;
    if (best_inl >= 12)
        ok = dlt_projection_to_pose(best_P, out_pose);
    free(ids);
    return ok;
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
