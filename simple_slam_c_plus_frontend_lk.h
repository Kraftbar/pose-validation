#ifndef SIMPLE_SLAM_C_PLUS_FRONTEND_LK_H
#define SIMPLE_SLAM_C_PLUS_FRONTEND_LK_H

typedef struct {
    unsigned char *p_g1, *c_g1, *p_g2, *c_g2, *p_g3, *c_g3;
    size_t pyr_cap;
    Corner *res;
    int *ok;
    int point_cap;
} LKScratch;

static void lk_scratch_reserve(LKScratch *s, int w, int h, int npoints) {
    size_t pyr_need = (size_t)(w / 2) * (h / 2);
    if (pyr_need > s->pyr_cap) {
        s->p_g1 = (unsigned char *)xrealloc(s->p_g1, pyr_need);
        s->c_g1 = (unsigned char *)xrealloc(s->c_g1, pyr_need);
        s->p_g2 = (unsigned char *)xrealloc(s->p_g2, pyr_need);
        s->c_g2 = (unsigned char *)xrealloc(s->c_g2, pyr_need);
        s->p_g3 = (unsigned char *)xrealloc(s->p_g3, pyr_need);
        s->c_g3 = (unsigned char *)xrealloc(s->c_g3, pyr_need);
        s->pyr_cap = pyr_need;
    }
    if (npoints > s->point_cap) {
        s->res = (Corner *)xrealloc(s->res, (size_t)npoints * sizeof(Corner));
        s->ok = (int *)xrealloc(s->ok, (size_t)npoints * sizeof(int));
        s->point_cap = npoints;
    }
}

static void lk_scratch_free(LKScratch *s) {
    free(s->p_g1);
    free(s->c_g1);
    free(s->p_g2);
    free(s->c_g2);
    free(s->p_g3);
    free(s->c_g3);
    free(s->res);
    free(s->ok);
}

static void track_corners_pure_lk(const unsigned char *p_g, const unsigned char *c_g, int w, int h,
                                  const CornerVec *p_pts, CornerVec *c_pts, MatchVec *m,
                                  LKScratch *scratch, int lk_iters, int lk_back_iters) {
    lk_scratch_reserve(scratch, w, h, p_pts->size);
    unsigned char *p_g1 = scratch->p_g1, *c_g1 = scratch->c_g1,
                  *p_g2 = scratch->p_g2, *c_g2 = scratch->c_g2,
                  *p_g3 = scratch->p_g3, *c_g3 = scratch->c_g3;
    downsample2x(p_g, w, h, p_g1);
    downsample2x(c_g, w, h, c_g1);
    downsample2x(p_g1, w / 2, h / 2, p_g2);
    downsample2x(c_g1, w / 2, h / 2, c_g2);
    downsample2x(p_g2, w / 4, h / 4, p_g3);
    downsample2x(c_g2, w / 4, h / 4, c_g3);
    Corner *res = scratch->res;
    int *ok = scratch->ok;
    memset(ok, 0, (size_t)p_pts->size * sizeof(int));
#pragma omp parallel for
    for (int i = 0; i < p_pts->size; i++) {
        Corner p = p_pts->data[i];
        float dx = 0, dy = 0;
        for (int level = 3; level >= 0; level--) {
            const unsigned char *pg, *cg;
            int lw, lh;
            float sc;
            if (level == 3) {
                pg = p_g3;
                cg = c_g3;
                lw = w / 8;
                lh = h / 8;
                sc = 0.125f;
            } else if (level == 2) {
                pg = p_g2;
                cg = c_g2;
                lw = w / 4;
                lh = h / 4;
                sc = 0.25f;
            } else if (level == 1) {
                pg = p_g1;
                cg = c_g1;
                lw = w / 2;
                lh = h / 2;
                sc = 0.5f;
            } else {
                pg = p_g;
                cg = c_g;
                lw = w;
                lh = h;
                sc = 1.0f;
            }
            float lx = p.x * sc, ly = p.y * sc;
            float l_dx = dx * sc, l_dy = dy * sc;
            for (int it = 0; it < lk_iters; it++) {
                float G[4] = {0}, b[2] = {0};
                for (int y = -3; y <= 3; y++)
                    for (int x = -3; x <= 3; x++) {
                        float cur_x = lx + x, cur_y = ly + y, nxt_x = lx + l_dx + x,
                              nxt_y = ly + l_dy + y;
                        if (cur_x < 1 || cur_x >= lw - 1 || cur_y < 1 || cur_y >= lh - 1 ||
                            nxt_x < 1 || nxt_x >= lw - 1 || nxt_y < 1 || nxt_y >= lh - 1)
                            continue;
                        float Ix = (get_pixel_bilinear(pg, lw, lh, cur_x + 1, cur_y) -
                                    get_pixel_bilinear(pg, lw, lh, cur_x - 1, cur_y)) *
                                   0.5f;
                        float Iy = (get_pixel_bilinear(pg, lw, lh, cur_x, cur_y + 1) -
                                    get_pixel_bilinear(pg, lw, lh, cur_x, cur_y - 1)) *
                                   0.5f;
                        float It = get_pixel_bilinear(cg, lw, lh, nxt_x, nxt_y) -
                                   get_pixel_bilinear(pg, lw, lh, cur_x, cur_y);
                        G[0] += Ix * Ix;
                        G[1] += Ix * Iy;
                        G[3] += Iy * Iy;
                        b[0] -= Ix * It;
                        b[1] -= Iy * It;
                    }
                G[2] = G[1];
                float det = G[0] * G[3] - G[1] * G[2];
                if (fabs(det) < 1e-6)
                    break;
                float vx = (G[3] * b[0] - G[1] * b[1]) / det,
                      vy = (G[0] * b[1] - G[2] * b[0]) / det;
                l_dx += vx;
                l_dy += vy;
                if (vx * vx + vy * vy < 1e-6)
                    break;
            }
            dx = l_dx / sc;
            dy = l_dy / sc;
        }
        float back_dx = -dx, back_dy = -dy;
        for (int it = 0; it < lk_back_iters; it++) {
            float G[4] = {0}, b[2] = {0};
            for (int y = -3; y <= 3; y++)
                for (int x = -3; x <= 3; x++) {
                    float cur_x = p.x + dx + x, cur_y = p.y + dy + y,
                          nxt_x = p.x + dx + back_dx + x, nxt_y = p.y + dy + back_dy + y;
                    if (cur_x < 1 || cur_x >= w - 1 || cur_y < 1 || cur_y >= h - 1 || nxt_x < 1 ||
                        nxt_x >= w - 1 || nxt_y < 1 || nxt_y >= h - 1)
                        continue;
                    float Ix = (get_pixel_bilinear(c_g, w, h, cur_x + 1, cur_y) -
                                get_pixel_bilinear(c_g, w, h, cur_x - 1, cur_y)) *
                               0.5f;
                    float Iy = (get_pixel_bilinear(c_g, w, h, cur_x, cur_y + 1) -
                                get_pixel_bilinear(c_g, w, h, cur_x, cur_y - 1)) *
                               0.5f;
                    float It = get_pixel_bilinear(p_g, w, h, nxt_x, nxt_y) -
                               get_pixel_bilinear(c_g, w, h, cur_x, cur_y);
                    G[0] += Ix * Ix;
                    G[1] += Ix * Iy;
                    G[3] += Iy * Iy;
                    b[0] -= Ix * It;
                    b[1] -= Iy * It;
                }
            G[2] = G[1];
            float det = G[0] * G[3] - G[1] * G[2];
            if (fabs(det) < 1e-6)
                break;
            float vx = (G[3] * b[0] - G[1] * b[1]) / det, vy = (G[0] * b[1] - G[2] * b[0]) / det;
            back_dx += vx;
            back_dy += vy;
        }
        float fb_err = (dx + back_dx) * (dx + back_dx) + (dy + back_dy) * (dy + back_dy);
        float tx = p.x + dx, ty = p.y + dy;
        if (fb_err < 0.5f && tx >= 2 && tx < w - 2 && ty >= 2 && ty < h - 2) {
            res[i] = (Corner){tx, ty, p.pt_idx, p.cand_idx, fb_err, sqrtf(dx * dx + dy * dy)};
            ok[i] = 1;
        }
    }
    for (int i = 0; i < p_pts->size; i++)
        if (ok[i]) {
            match_vec_push(m, (Match){i, c_pts->size, 0});
            corner_vec_push(c_pts, res[i]);
        }
}

#endif
