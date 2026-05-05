#ifndef SIMPLE_SLAM_C_PLUS_BACKEND_MAP_H
#define SIMPLE_SLAM_C_PLUS_BACKEND_MAP_H

// Cull map points that reproject above `err_thresh_px` in EVERY observing
// keyframe of the local window (with >=2 obs in window). Setting obs=0 marks
// the point invalid; existing pt_idx links are gated on map->data[i].obs > 0.
static void cull_map_points_window(KFDB *db, Map *map, int kf_start, int kf_end,
                                   double fx, double fy, double cx, double cy,
                                   double err_thresh_px) {
    double th2   = err_thresh_px * err_thresh_px;
    int   *total = calloc(map->size, sizeof(int));
    int   *bad   = calloc(map->size, sizeof(int));

    for (int k = kf_start; k < kf_end; k++) {
        KFEntry *kf = &db->data[k];
        double R[9], t[3];
        pose_get_rotation(&kf->pose, R);
        pose_get_translation(&kf->pose, t);
        for (int j = 0; j < kf->corners.size; j++) {
            int pi = kf->corners.data[j].pt_idx;
            if (pi < 0 || map->data[pi].obs < 2)
                continue;
            MapPoint p = map->data[pi];

            double cp[3] = {R[0]*p.x + R[1]*p.y + R[2]*p.z + t[0],
                            R[3]*p.x + R[4]*p.y + R[5]*p.z + t[1],
                            R[6]*p.x + R[7]*p.y + R[8]*p.z + t[2]};
            total[pi]++;
            if (cp[2] < 0.1) {
                bad[pi]++;
                continue;
            }
            double u  = fx*cp[0]/cp[2] + cx;
            double v  = fy*cp[1]/cp[2] + cy;
            double du = kf->corners.data[j].x - u;
            double dv = kf->corners.data[j].y - v;
            if (du*du + dv*dv > th2)
                bad[pi]++;
        }
    }
    for (int pi = 0; pi < map->size; pi++) {
        if (total[pi] >= 2 && bad[pi] == total[pi])
            map->data[pi].obs = 0;
    }
    free(total);
    free(bad);
}

#endif
