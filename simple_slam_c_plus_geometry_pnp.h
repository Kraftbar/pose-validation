#ifndef SIMPLE_SLAM_C_PLUS_GEOMETRY_PNP_H
#define SIMPLE_SLAM_C_PLUS_GEOMETRY_PNP_H

static int geometry_dlt_to_pose(const double P[12], Pose *out_pose) {
    double R[9] = {P[0], P[1], P[2], P[4], P[5], P[6], P[8], P[9], P[10]},
           t[3] = {P[3], P[7], P[11]};
    // DLT has an arbitrary global sign; translation.z is not a sign test.
    if (mat3_det(R) < 0) {
        for (int i = 0; i < 9; i++) R[i] = -R[i];
        for (int i = 0; i < 3; i++) t[i] = -t[i];
    }
    double W[3], U[9], V[9], Ro[9], VT[9];
    geometry_svd3(R, W, U, V);
    double scale = (W[0] + W[1] + W[2])/3;
    if (!isfinite(scale) || scale < 1e-12 || W[2] < 1e-8*W[0]) return 0;
    mat3_transpose(V, VT);
    mat3_mul(U, VT, Ro);
    if (mat3_det(Ro) <= 0) return 0;
    for (int i = 0; i < 3; i++) t[i] /= scale;
    pose_from_rt(Ro, t, out_pose);
    return 1;
}

static int geometry_estimate_pnp(const Map *map, const CornerVec *corners, double fx, double fy,
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
    Pose best_pose;
    int best_inl = 0;
    int iters = dlt_iters > 0 ? dlt_iters : 500;
    srand(g_ransac_seed);
    for (int it = 0; it < iters; it++) {
        double AtA[144] = {0}, W[12], V[144], P[12];
        int sample_ids[6];
        double center[3] = {0};
        for (int i = 0; i < 6; i++) {
            int duplicate;
            do {
                sample_ids[i] = ids[rand() % n];
                duplicate = 0;
                for (int j = 0; j < i; j++)
                    if (sample_ids[j] == sample_ids[i]) duplicate = 1;
            } while (duplicate);
            MapPoint p = map->data[corners->data[sample_ids[i]].pt_idx];
            center[0] += p.x/6; center[1] += p.y/6; center[2] += p.z/6;
        }
        double world_scale = 0;
        for (int i = 0; i < 6; i++) {
            MapPoint p = map->data[corners->data[sample_ids[i]].pt_idx];
            double x = p.x-center[0], y = p.y-center[1], z = p.z-center[2];
            world_scale += (x*x+y*y+z*z)/6;
        }
        world_scale = sqrt(world_scale);
        if (!isfinite(world_scale) || world_scale < 1e-12) continue;
        for (int i = 0; i < 6; i++) {
            int idx = sample_ids[i];
            MapPoint p = map->data[corners->data[idx].pt_idx];
            p.x = (p.x-center[0])/world_scale;
            p.y = (p.y-center[1])/world_scale;
            p.z = (p.z-center[2])/world_scale;
            double u = (corners->data[idx].x - cx) / fx, v = (corners->data[idx].y - cy) / fy;
            double r1[12] = {p.x, p.y, p.z, 1,  0,   0,   0,   0, -u*p.x, -u*p.y, -u*p.z, -u};
            double r2[12] = {0,   0,   0,   0,  p.x, p.y, p.z, 1, -v*p.x, -v*p.y, -v*p.z, -v};
            for (int r = 0; r < 12; r++)
                for (int c = 0; c < 12; c++)
                    AtA[r * 12 + c] += r1[r] * r1[c] + r2[r] * r2[c];
        }
        geometry_eigen(AtA, 12, W, V);
        int bi = 0;
        double mw = W[0];
        for (int j = 1; j < 12; j++)
            if (W[j] < mw) {
                mw = W[j];
                bi = j;
            }
        for (int j = 0; j < 12; j++)
            P[j] = V[j * 12 + bi];
        Pose candidate;
        if (!geometry_dlt_to_pose(P, &candidate)) continue;
        for (int r = 0; r < 3; r++)
            candidate.m[4*r+3] = world_scale*candidate.m[4*r+3]
                - candidate.m[4*r]*center[0] - candidate.m[4*r+1]*center[1]
                - candidate.m[4*r+2]*center[2];
        int inl = count_pose_inliers(map, corners, fx, fy, cx, cy, &candidate);
        if (inl > best_inl) {
            best_inl = inl;
            best_pose = candidate;
        }
        if (inl > n * 0.8)
            break;
    }
    if (out_inl)
        *out_inl = best_inl;
    int ok = 0;
    if (best_inl >= 12) {
        *out_pose = best_pose;
        ok = 1;
    }
    free(ids);
    return ok;
}

#endif
