#ifndef SIMPLE_SLAM_C_PLUS_BACKEND_BA_H
#define SIMPLE_SLAM_C_PLUS_BACKEND_BA_H

static void refine_map_point_against_kfs(Map *map, int i, KFDB *db, int k_start, int k_end,
                                         double fx, double fy, double cx, double cy) {
    if (map->data[i].obs < 2)
        return;
    double H[9] = {0}, b[3] = {0};
    for (int k = k_start; k < k_end; k++) {
        KFEntry *kf = &db->data[k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            if (kf->corners.data[j].pt_idx != i)
                continue;
            MapPoint p = map->data[i];

            // Project P into camera k:  cp = R·P + t
            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            if (cp[2] < 0.1)
                continue;

            double inv_z  = 1.0 / cp[2];
            double inv_z2 = inv_z * inv_z;
            double u  = fx*cp[0]*inv_z + cx;
            double v  = fy*cp[1]*inv_z + cy;
            double du = kf->corners.data[j].x - u;
            double dv = kf->corners.data[j].y - v;

            // J[2][3] = ∂(u,v)/∂P  (3-DoF Jacobian wrt world point):
            double J[2][3] = {
                {fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2,  fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2,  fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2},
                {fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2,  fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2,  fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2},
            };
            for (int r = 0; r < 3; r++) {
                b[r] += J[0][r] * du + J[1][r] * dv;
                for (int c = 0; c < 3; c++)
                    H[r * 3 + c] += J[0][r] * J[0][c] + J[1][r] * J[1][c];
            }
        }
    }
    for (int r = 0; r < 3; r++)
        H[r * 3 + r] += 1e-4;
    double det = H[0] * (H[4] * H[8] - H[5] * H[7]) - H[1] * (H[3] * H[8] - H[5] * H[6]) +
                 H[2] * (H[3] * H[7] - H[4] * H[6]);
    if (fabs(det) > 1e-9) {
        double dx = (b[0]*(H[4]*H[8] - H[5]*H[7]) - H[1]*(b[1]*H[8] - H[5]*b[2]) + H[2]*(b[1]*H[7] - H[4]*b[2])) / det;
        double dy = (H[0]*(b[1]*H[8] - H[5]*b[2]) - b[0]*(H[3]*H[8] - H[5]*H[6]) + H[2]*(H[3]*b[2] - b[1]*H[6])) / det;
        double dz = (H[0]*(H[4]*b[2] - b[1]*H[7]) - H[1]*(H[3]*b[2] - b[1]*H[6]) + b[0]*(H[3]*H[7] - H[4]*H[6])) / det;
        map->data[i].x += dx;
        map->data[i].y += dy;
        map->data[i].z += dz;
    }
}

static unsigned char *build_pnp_quality_mask(const KFDB *db, const Map *map, double fx, double fy,
                                             double cx, double cy, double gate_px, int min_obs,
                                             int window) {
    if (gate_px <= 0.0 || map->size <= 0 || db->size <= 0)
        return NULL;
    if (min_obs < 1)
        min_obs = 1;
    if (window < 1 || window > db->size)
        window = db->size;
    unsigned char *ok = malloc((size_t)map->size);
    unsigned short *total = calloc((size_t)map->size, sizeof(unsigned short));
    unsigned short *bad = calloc((size_t)map->size, sizeof(unsigned short));
    if (!ok || !total || !bad) {
        free(ok);
        free(total);
        free(bad);
        return NULL;
    }
    memset(ok, 1, (size_t)map->size);
    double th2 = gate_px * gate_px;
    int start = db->size - window;
    if (start < 0)
        start = 0;
    for (int k = start; k < db->size; k++) {
        const KFEntry *kf = &db->data[k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || pi >= map->size)
                continue;
            MapPoint p = map->data[pi];
            double x = R[0] * p.x + R[1] * p.y + R[2] * p.z + t[0],
                   y = R[3] * p.x + R[4] * p.y + R[5] * p.z + t[1],
                   z = R[6] * p.x + R[7] * p.y + R[8] * p.z + t[2];
            total[pi]++;
            if (z < 0.1) {
                bad[pi]++;
                continue;
            }
            double u = fx * x / z + cx, v = fy * y / z + cy;
            double du = kf->corners.data[j].x - u, dv = kf->corners.data[j].y - v;
            if (du * du + dv * dv > th2)
                bad[pi]++;
        }
    }
    for (int pi = 0; pi < map->size; pi++)
        if (total[pi] >= min_obs && bad[pi] == total[pi])
            ok[pi] = 0;
    free(total);
    free(bad);
    return ok;
}

static void global_ba(KFDB *db, Map *map, double fx, double fy, double cx, double cy,
                      int iters, int pose_lm_iters) {
    if (db->size < 3)
        return;
    for (int iter = 0; iter < iters; iter++) {
#pragma omp parallel for
        for (int k = 1; k < db->size; k++)
            refine_pose_lm(map, &db->data[k].corners, fx, fy, cx, cy, pose_lm_iters,
                           &db->data[k].pose);
#pragma omp parallel for
        for (int i = 0; i < map->size; i++)
            refine_map_point_against_kfs(map, i, db, 0, db->size, fx, fy, cx, cy);
    }
}

static void local_ba(KFDB *db, Map *map, double fx, double fy, double cx, double cy,
                     int pose_lm_iters) {
    if (db->size < 3)
        return;
    for (int iter = 0; iter < 5; iter++) {

        for (int k = db->size - 3; k < db->size; k++) {
            refine_pose_lm(map, &db->data[k].corners, fx, fy, cx, cy, pose_lm_iters,
                           &db->data[k].pose);
        }
        for (int i = 0; i < map->size; i++) {
            if (map->data[i].obs < 2)
                continue;
            double H[9] = {0}, b[3] = {0};
            for (int k = db->size - 3; k < db->size; k++) {
                KFEntry *kf = &db->data[k];
                double R[9], t[3];
                pose_get_rotation(&kf->pose, R);
                pose_get_translation(&kf->pose, t);
                for (int j = 0; j < kf->corners.size; j++) {
                    if (kf->corners.data[j].pt_idx != i)
                        continue;
                    MapPoint p = map->data[i];

                    // Project P into camera k:  cp = R·P + t
                    double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                                    R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                                    R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
                    if (cp[2] < 0.1)
                        continue;

                    double inv_z  = 1.0 / cp[2];
                    double inv_z2 = inv_z * inv_z;
                    double u  = fx*cp[0]*inv_z + cx;
                    double v  = fy*cp[1]*inv_z + cy;
                    double du = kf->corners.data[j].x - u;
                    double dv = kf->corners.data[j].y - v;

                    // J[2][3] = ∂(u,v)/∂P
                    double J[2][3] = {
                        {fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2,  fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2,  fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2},
                        {fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2,  fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2,  fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2},
                    };
                    for (int r = 0; r < 3; r++) {
                        b[r] += J[0][r] * du + J[1][r] * dv;
                        for (int c = 0; c < 3; c++)
                            H[r * 3 + c] += J[0][r] * J[0][c] + J[1][r] * J[1][c];
                    }
                }
            }
            for (int r = 0; r < 3; r++)
                H[r * 3 + r] += 1e-4;
            double det = H[0] * (H[4] * H[8] - H[5] * H[7]) - H[1] * (H[3] * H[8] - H[5] * H[6]) +
                         H[2] * (H[3] * H[7] - H[4] * H[6]);
            if (fabs(det) > 1e-9) {
                double dx = (b[0]*(H[4]*H[8] - H[5]*H[7]) - H[1]*(b[1]*H[8] - H[5]*b[2]) + H[2]*(b[1]*H[7] - H[4]*b[2])) / det;
                double dy = (H[0]*(b[1]*H[8] - H[5]*b[2]) - b[0]*(H[3]*H[8] - H[5]*H[6]) + H[2]*(H[3]*b[2] - b[1]*H[6])) / det;
                double dz = (H[0]*(H[4]*b[2] - b[1]*H[7]) - H[1]*(H[3]*b[2] - b[1]*H[6]) + b[0]*(H[3]*H[7] - H[4]*H[6])) / det;
                map->data[i].x += dx;
                map->data[i].y += dy;
                map->data[i].z += dz;
            }
        }
    }
}

// Robust χ² (Huber-rho) over all observations in the local BA window — used to
// score tentative LM steps in joint_local_ba.
static double window_chi2(const KFDB *db, const Map *map, int kf_start, int nkf_window,
                          double fx, double fy, double cx, double cy,
                          const int *point_local, double huber) {
    (void)point_local;
    double chi2 = 0.0;
    for (int k = 0; k < nkf_window; k++) {
        const KFEntry *kf = &db->data[kf_start + k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || map->data[pi].obs == 0)
                continue;
            MapPoint p = map->data[pi];

            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            if (cp[2] < 0.1)
                continue;

            double u  = fx*cp[0]/cp[2] + cx;
            double v  = fy*cp[1]/cp[2] + cy;
            double du = kf->corners.data[j].x - u;
            double dv = kf->corners.data[j].y - v;
            double e2 = du*du + dv*dv;

            chi2 += (e2 > huber) ? (2.0 * sqrt(huber * e2) - huber) : e2;
        }
    }
    return chi2;
}

// Joint local Bundle Adjustment (Schur-complement form, ORB-SLAM-style).
//
//   Window: last NK keyframes; the oldest is held FIXED as a gauge anchor.
//   Free states:  ξ_k ∈ se(3) for each free KF (6 dof each)
//                 P_i ∈ ℝ³    for each map point seen by ≥1 window KF (3 dof each)
//
//   Cost:  Σ_{k,i}  ρ( || z_{k,i} − π(R_k P_i + t_k) ||² )
//          ρ = Huber with χ²(2 dof, 95%) = 5.991.
//
//   Normal equations (LM-damped, then block-eliminated):
//
//     ⎡ U   W ⎤ ⎡ dξ ⎤   ⎡ b_ξ ⎤
//     ⎣ Wᵀ  V ⎦ ⎣ dP ⎦ = ⎣ b_P ⎦
//
//          U_k = Σ_i wᵢ J_ξᵀ J_ξ          (block-diag in k, 6×6 each)
//          V_i = Σ_k wᵢ J_Pᵀ J_P          (block-diag in i, 3×3 each)
//        W_{k,i} = wᵢ J_ξᵀ J_P            (cross block, 6×3)
//
//   Schur-eliminate the points (V is block-diag, cheap to invert):
//
//        S       = U − W V⁻¹ Wᵀ
//        g       = b_ξ − W V⁻¹ b_P
//        S · dξ  = g
//        dP_i    = V_i⁻¹ (b_P_i − Σ_k W_{k,i}ᵀ dξ_k)
//
//   LM control: damp diag(U), diag(V) by (1+λ); accept step if χ² decreased,
//   else revert and retry with 10·λ. Inner LM tries up to 5 λ values per
//   linearization; outer loop relinearizes up to 10 times.
static void joint_local_ba(KFDB *db, Map *map, double fx, double fy, double cx, double cy) {
    const int    NK    = 5;
    const double huber = 5.991;

    if (db->size < 3)
        return;
    int kf_start   = db->size - NK;
    if (kf_start < 0) kf_start = 0;
    int nkf_window = db->size - kf_start;
    int nfree      = nkf_window - 1;
    if (nfree < 1)
        return;

    // Local-point set: points with ≥2 observations within the window itself.
    // (Points seen only once in-window are weakly constrained when co-optimized,
    //  so we keep them as fixed-position pose constraints via the li<0 path.)
    int *point_local  = malloc(map->size * sizeof(int));
    int *win_obs_cnt  = calloc(map->size, sizeof(int));
    for (int i = 0; i < map->size; i++)
        point_local[i] = -1;
    for (int k = 0; k < nkf_window; k++) {
        KFEntry *kf = &db->data[kf_start + k];
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || map->data[pi].obs < 2)
                continue;
            win_obs_cnt[pi]++;
        }
    }
    int Np = 0;
    for (int i = 0; i < map->size; i++)
        if (win_obs_cnt[i] >= 2)
            point_local[i] = Np++;
    free(win_obs_cnt);
    if (Np < 5) {
        free(point_local);
        return;
    }
    int *local_to_global = malloc(Np * sizeof(int));
    for (int i = 0; i < map->size; i++)
        if (point_local[i] >= 0)
            local_to_global[point_local[i]] = i;

    // Block storage.
    int     N_pose = nfree * 6;
    double *U      = malloc((size_t)nfree * 36 * sizeof(double));
    double *V      = malloc((size_t)Np    * 9  * sizeof(double));
    double *W      = malloc((size_t)nfree * Np * 18 * sizeof(double));
    double *bxi    = malloc((size_t)nfree * 6  * sizeof(double));
    double *bp     = malloc((size_t)Np    * 3  * sizeof(double));
    double *U_damp = malloc((size_t)nfree * 36 * sizeof(double));
    double *V_damp = malloc((size_t)Np    * 9  * sizeof(double));
    double *Vinv   = malloc((size_t)Np    * 9  * sizeof(double));
    unsigned char *V_ok = malloc(Np);
    double *S      = malloc((size_t)N_pose * N_pose * sizeof(double));
    double *g      = malloc((size_t)N_pose * sizeof(double));
    double *dxi    = malloc((size_t)N_pose * sizeof(double));
    double *Y_buf  = malloc((size_t)nfree * 18 * sizeof(double));

    // Snapshots for rollback.
    Pose   *kf_save = malloc(nfree * sizeof(Pose));
    double *pt_save = malloc((size_t)Np * 3 * sizeof(double));

    double lambda    = 1e-3;
    double prev_chi2 = window_chi2(db, map, kf_start, nkf_window, fx, fy, cx, cy, point_local, huber);

    for (int outer = 0; outer < 10; outer++) {
        // Snapshot pre-step state.
        for (int k = 0; k < nfree; k++)
            kf_save[k] = db->data[kf_start + 1 + k].pose;
        for (int i = 0; i < Np; i++) {
            MapPoint p = map->data[local_to_global[i]];
            pt_save[i*3 + 0] = p.x;
            pt_save[i*3 + 1] = p.y;
            pt_save[i*3 + 2] = p.z;
        }

        // Build U, V, W, bxi, bp at current state.
        memset(U,   0, (size_t)nfree * 36 * sizeof(double));
        memset(V,   0, (size_t)Np    * 9  * sizeof(double));
        memset(W,   0, (size_t)nfree * Np * 18 * sizeof(double));
        memset(bxi, 0, (size_t)nfree * 6  * sizeof(double));
        memset(bp,  0, (size_t)Np    * 3  * sizeof(double));

        for (int k = 0; k < nkf_window; k++) {
            int      free_idx = (k == 0) ? -1 : k - 1;
            KFEntry *kf       = &db->data[kf_start + k];
            double R[9], t[3];
            pose_get_rotation(&kf->pose, R);
            pose_get_translation(&kf->pose, t);

            for (int j = 0; j < kf->corners.size; j++) {
                int pi = kf->corners.data[j].pt_idx;
                if (pi < 0 || map->data[pi].obs == 0)
                    continue;
                int li = point_local[pi];      // -1 ⇒ point is fixed (obs<2 or out of window)
                MapPoint p = map->data[pi];

                // cp = R · P + t
                double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                                R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                                R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
                if (cp[2] < 0.1)
                    continue;

                double inv_z  = 1.0 / cp[2];
                double inv_z2 = inv_z * inv_z;
                double u  = fx*cp[0]*inv_z + cx;
                double v  = fy*cp[1]*inv_z + cy;
                double du = kf->corners.data[j].x - u;
                double dv = kf->corners.data[j].y - v;

                double e2 = du*du + dv*dv;
                double w_robust = (e2 > huber) ? sqrt(huber / e2) : 1.0;

                // J_ξ (2×6): residual w.r.t. left-multiplied SE(3) perturbation (ρ, φ).
                double J_xi[2][6] = {
                    {fx*inv_z, 0,        -fx*cp[0]*inv_z2,  -fx*cp[0]*cp[1]*inv_z2,        fx*(1 + cp[0]*cp[0]*inv_z2),  -fx*cp[1]*inv_z},
                    {0,        fy*inv_z, -fy*cp[1]*inv_z2,  -fy*(1 + cp[1]*cp[1]*inv_z2),  fy*cp[0]*cp[1]*inv_z2,         fy*cp[0]*inv_z},
                };
                // J_P (2×3): residual w.r.t. world point P  (∂π/∂P = (∂π/∂cp) · R).
                double J_p[2][3] = {
                    {fx*R[0]*inv_z - fx*cp[0]*R[6]*inv_z2,  fx*R[1]*inv_z - fx*cp[0]*R[7]*inv_z2,  fx*R[2]*inv_z - fx*cp[0]*R[8]*inv_z2},
                    {fy*R[3]*inv_z - fy*cp[1]*R[6]*inv_z2,  fy*R[4]*inv_z - fy*cp[1]*R[7]*inv_z2,  fy*R[5]*inv_z - fy*cp[1]*R[8]*inv_z2},
                };

                if (li >= 0) {
                    // V_i  += w · J_Pᵀ J_P,    b_P_i += w · J_Pᵀ r   (free point)
                    double *Vi  = V  + li * 9;
                    double *bpi = bp + li * 3;
                    for (int r = 0; r < 3; r++) {
                        bpi[r] += w_robust * (J_p[0][r]*du + J_p[1][r]*dv);
                        for (int c = 0; c < 3; c++)
                            Vi[r*3 + c] += w_robust * (J_p[0][r]*J_p[0][c] + J_p[1][r]*J_p[1][c]);
                    }
                }

                if (free_idx >= 0) {
                    // U_k    += w · J_ξᵀ J_ξ,    b_ξ_k  += w · J_ξᵀ r
                    double *Uk   = U   + free_idx * 36;
                    double *bxik = bxi + free_idx * 6;
                    for (int r = 0; r < 6; r++) {
                        bxik[r] += w_robust * (J_xi[0][r]*du + J_xi[1][r]*dv);
                        for (int c = 0; c < 6; c++)
                            Uk[r*6 + c] += w_robust * (J_xi[0][r]*J_xi[0][c] + J_xi[1][r]*J_xi[1][c]);
                    }
                    if (li >= 0) {
                        // W_{k,i} += w · J_ξᵀ J_P     (only when point is free)
                        double *Wki = W + ((size_t)free_idx * Np + li) * 18;
                        for (int r = 0; r < 6; r++)
                            for (int c = 0; c < 3; c++)
                                Wki[r*3 + c] += w_robust * (J_xi[0][r]*J_p[0][c] + J_xi[1][r]*J_p[1][c]);
                    }
                }
            }
        }

        int accepted = 0;
        for (int inner = 0; inner < 5 && !accepted; inner++) {
            // LM diagonal damping (Marquardt scaling):
            //   U_damp = U + λ · diag(U)        V_damp = V + λ · diag(V)
            memcpy(U_damp, U, (size_t)nfree * 36 * sizeof(double));
            memcpy(V_damp, V, (size_t)Np    * 9  * sizeof(double));
            for (int k = 0; k < nfree; k++) {
                double *Uk = U_damp + k * 36;
                for (int r = 0; r < 6; r++)
                    Uk[r*6 + r] *= (1.0 + lambda);
            }
            for (int i = 0; i < Np; i++) {
                double *Vi = V_damp + i * 9;
                for (int r = 0; r < 3; r++)
                    Vi[r*3 + r] *= (1.0 + lambda);
            }
            for (int i = 0; i < Np; i++)
                V_ok[i] = mat3_inverse(V_damp + i * 9, Vinv + i * 9);

            // S ← block-diag(U_damp);      g ← b_ξ
            memset(S, 0, (size_t)N_pose * N_pose * sizeof(double));
            for (int k = 0; k < nfree; k++) {
                double *Uk = U_damp + k * 36;
                for (int r = 0; r < 6; r++)
                    for (int c = 0; c < 6; c++)
                        S[(k*6 + r) * N_pose + (k*6 + c)] = Uk[r*6 + c];
            }
            memcpy(g, bxi, (size_t)N_pose * sizeof(double));

            // S −= Σ_i W_{·,i} V_i⁻¹ W_{·,i}ᵀ;    g −= Σ_i W_{·,i} V_i⁻¹ b_P_i
            for (int i = 0; i < Np; i++) {
                if (!V_ok[i])
                    continue;
                double *Vi_inv = Vinv + i * 9;

                // Y_k = W_{k,i} · V_i⁻¹  (6×3) for each free k.
                for (int k = 0; k < nfree; k++) {
                    double *Wki = W + ((size_t)k * Np + i) * 18;
                    double *Yk  = Y_buf + k * 18;
                    for (int r = 0; r < 6; r++)
                        for (int c = 0; c < 3; c++) {
                            double s = 0;
                            for (int kk = 0; kk < 3; kk++)
                                s += Wki[r*3 + kk] * Vi_inv[kk*3 + c];
                            Yk[r*3 + c] = s;
                        }
                }
                // S[k1][k2] −= Y_{k1} · W_{k2,i}ᵀ
                for (int k1 = 0; k1 < nfree; k1++) {
                    double *Yk1 = Y_buf + k1 * 18;
                    for (int k2 = 0; k2 < nfree; k2++) {
                        double *Wk2i = W + ((size_t)k2 * Np + i) * 18;
                        for (int r = 0; r < 6; r++)
                            for (int c = 0; c < 6; c++) {
                                double s = 0;
                                for (int kk = 0; kk < 3; kk++)
                                    s += Yk1[r*3 + kk] * Wk2i[c*3 + kk];
                                S[(k1*6 + r) * N_pose + (k2*6 + c)] -= s;
                            }
                    }
                }
                // g[k] −= Y_k · b_P_i
                double *bpi = bp + i * 3;
                for (int k = 0; k < nfree; k++) {
                    double *Yk = Y_buf + k * 18;
                    for (int r = 0; r < 6; r++) {
                        double s = 0;
                        for (int c = 0; c < 3; c++)
                            s += Yk[r*3 + c] * bpi[c];
                        g[k*6 + r] -= s;
                    }
                }
            }

            if (!solve_dense_n(S, g, dxi, N_pose)) {
                lambda *= 10.0;
                if (lambda > 1e8) goto done;
                continue;
            }

            // Apply pose updates: t += ρ;   R ← (I + [φ]_×) R, then re-orthogonalize.
            for (int k = 0; k < nfree; k++) {
                Pose  *pose = &db->data[kf_start + 1 + k].pose;
                double R[9], t[3];
                pose_get_rotation(pose, R);
                pose_get_translation(pose, t);

                double rho[3] = {dxi[k*6 + 0], dxi[k*6 + 1], dxi[k*6 + 2]};
                double phi[3] = {dxi[k*6 + 3], dxi[k*6 + 4], dxi[k*6 + 5]};
                t[0] += rho[0];
                t[1] += rho[1];
                t[2] += rho[2];

                double dR[9] = { 1,       -phi[2],  phi[1],
                                 phi[2],   1,      -phi[0],
                                -phi[1],   phi[0],  1     };
                double Rn[9], R_proj[9];
                mat3_mul(dR, R, Rn);
                project_to_SO3(Rn, R_proj);
                pose_from_rt(R_proj, t, pose);
            }
            // Back-substitute point updates: dP_i = V_i⁻¹ (b_P_i − Σ_k W_{k,i}ᵀ dξ_k)
            for (int i = 0; i < Np; i++) {
                if (!V_ok[i])
                    continue;
                double rhs[3] = {bp[i*3 + 0], bp[i*3 + 1], bp[i*3 + 2]};
                for (int k = 0; k < nfree; k++) {
                    double *Wki  = W   + ((size_t)k * Np + i) * 18;
                    double *dxik = dxi + k * 6;
                    for (int c = 0; c < 3; c++) {
                        double s = 0;
                        for (int r = 0; r < 6; r++)
                            s += Wki[r*3 + c] * dxik[r];
                        rhs[c] -= s;
                    }
                }
                double *Vi_inv = Vinv + i * 9;
                int gi = local_to_global[i];
                map->data[gi].x += Vi_inv[0]*rhs[0] + Vi_inv[1]*rhs[1] + Vi_inv[2]*rhs[2];
                map->data[gi].y += Vi_inv[3]*rhs[0] + Vi_inv[4]*rhs[1] + Vi_inv[5]*rhs[2];
                map->data[gi].z += Vi_inv[6]*rhs[0] + Vi_inv[7]*rhs[1] + Vi_inv[8]*rhs[2];
            }

            double new_chi2 = window_chi2(db, map, kf_start, nkf_window, fx, fy, cx, cy,
                                          point_local, huber);
            if (new_chi2 < prev_chi2) {
                prev_chi2 = new_chi2;
                lambda *= 0.5;
                if (lambda < 1e-8) lambda = 1e-8;
                accepted = 1;
            } else {
                // Revert and try again with stronger damping.
                for (int k = 0; k < nfree; k++)
                    db->data[kf_start + 1 + k].pose = kf_save[k];
                for (int i = 0; i < Np; i++) {
                    int gi = local_to_global[i];
                    map->data[gi].x = pt_save[i*3 + 0];
                    map->data[gi].y = pt_save[i*3 + 1];
                    map->data[gi].z = pt_save[i*3 + 2];
                }
                lambda *= 10.0;
                if (lambda > 1e8) goto done;
            }
        }
        if (!accepted)
            break;
    }
done:
    free(U);     free(V);     free(W);     free(bxi);   free(bp);
    free(U_damp); free(V_damp); free(Vinv); free(V_ok);
    free(S);     free(g);     free(dxi);   free(Y_buf);
    free(point_local); free(local_to_global);
    free(kf_save); free(pt_save);
}

#endif
