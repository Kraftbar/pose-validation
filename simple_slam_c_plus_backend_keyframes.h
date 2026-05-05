#ifndef SIMPLE_SLAM_C_PLUS_BACKEND_KEYFRAMES_H
#define SIMPLE_SLAM_C_PLUS_BACKEND_KEYFRAMES_H

typedef struct {
    unsigned char thumb[256];
    int frame_id;
    Pose pose;
    CornerVec corners;
    ImageGray gray;
} KFEntry;
typedef struct {
    KFEntry *data;
    int size, cap;
} KFDB;
static void kfdb_push(KFDB *db, KFEntry e) {
    if (db->size == db->cap) {
        db->cap = db->cap ? db->cap * 2 : 64;
        db->data = xrealloc(db->data, db->cap * sizeof(KFEntry));
    }
    db->data[db->size++] = e;
}
static void kfentry_free(KFEntry *e) {
    free(e->corners.data);
    image_gray_free(&e->gray);
    memset(e, 0, sizeof(*e));
}
static void kfdb_free(KFDB *db) {
    for (int i = 0; i < db->size; i++)
        kfentry_free(&db->data[i]);
    free(db->data);
    memset(db, 0, sizeof(*db));
}
static void gen_thumbnail(const unsigned char *g, int w, int h, unsigned char *out) {
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 16; x++)
            out[y * 16 + x] = g[(y * h / 16) * w + (x * w / 16)];
}
static int find_loop_candidate(KFDB *db, const unsigned char *thumb, int current_id) {
    int bi = -1;
    uint32_t bs = 0xFFFFFFFF;
    for (int i = 0; i < db->size - 50; i++) {
        uint32_t s = 0;
        for (int j = 0; j < 256; j++)
            s += abs(thumb[j] - db->data[i].thumb[j]);
        if (s < bs) {
            bs = s;
            bi = i;
        }
    }
    return (bs < 3500) ? bi : -1;
}

static int verify_loop(KFDB *db, int lidx, const unsigned char *cgray, int w, int h, double fx,
                       double fy, double cx, double cy, Pose *out_pose) {
    KFEntry *kf = &db->data[lidx];
    MatchVec matches = {0};
    CornerVec cpts = {0};
    for (int i = 0; i < kf->corners.size; i += 2) {
        Corner p = kf->corners.data[i];
        float best_ncc = -1;
        int best_x = -1, best_y = -1;
        for (int y = (int)p.y - 20; y < (int)p.y + 20; y += 2)
            for (int x = (int)p.x - 20; x < (int)p.x + 20; x += 2) {
                if (x < 4 || x >= w - 4 || y < 4 || y >= h - 4)
                    continue;
                float ncc = 0, s1 = 0, s2 = 0;
                for (int dy = -3; dy <= 3; dy++)
                    for (int dx = -3; dx <= 3; dx++) {
                        float v1 = kf->gray.data[((int)p.y + dy) * w + (int)p.x + dx],
                              v2 = cgray[(y + dy) * w + x + dx];
                        ncc += v1 * v2;
                        s1 += v1 * v1;
                        s2 += v2 * v2;
                    }
                ncc /= sqrt(s1 * s2 + 1e-6);
                if (ncc > best_ncc) {
                    best_ncc = ncc;
                    best_x = x;
                    best_y = y;
                }
            }
        if (best_ncc > 0.95) {
            match_vec_push(&matches, (Match){i, cpts.size, 0});
            corner_vec_push(&cpts, (Corner){(float)best_x, (float)best_y, -1, -1, 0.0f, 0.0f});
        }
    }
    Pose rel;
    unsigned char *mask;
    int inl;
    int ok = estimate_pose_E(&kf->corners, &cpts, &matches, fx, fy, cx, cy, &rel, 500, 32,
                             &mask, &inl);
    if (ok && inl > 20) {
        pose_compose_relative(&rel, &kf->pose, out_pose);
        free(mask);
        free(matches.data);
        free(cpts.data);
        return 1;
    }
    free(matches.data);
    free(cpts.data);
    return 0;
}

#endif
