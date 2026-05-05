#ifndef SIMPLE_SLAM_C_PLUS_IMAGE_H
#define SIMPLE_SLAM_C_PLUS_IMAGE_H

typedef struct {
    unsigned char *data;
    int w, h;
} ImageGray;

typedef struct {
    FILE *pipe;
    int w, h;
    int bytes_per_frame;
} FFmpegCap;

static ImageGray image_gray_clone(const unsigned char *src, int w, int h) {
    ImageGray img = {0};
    img.w = w;
    img.h = h;
    if (w <= 0 || h <= 0)
        return img;
    img.data = (unsigned char *)malloc((size_t)w * (size_t)h);
    if (!img.data) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    memcpy(img.data, src, (size_t)w * (size_t)h);
    return img;
}

static ImageGray image_gray_alloc_zero(int w, int h) {
    ImageGray img = {0};
    img.w = w;
    img.h = h;
    if (w <= 0 || h <= 0)
        return img;
    img.data = (unsigned char *)calloc((size_t)w * (size_t)h, 1);
    if (!img.data) {
        fprintf(stderr, "out of memory\n");
        exit(1);
    }
    return img;
}

static void image_gray_free(ImageGray *img) {
    free(img->data);
    memset(img, 0, sizeof(*img));
}

static FFmpegCap *ffmpeg_open(const char *p, int w, int h, int gray) {
    FFmpegCap *c = malloc(sizeof(FFmpegCap));
    c->w = w;
    c->h = h;
    c->bytes_per_frame = w * h * (gray ? 1 : 3);
    char cmd[1024];
    snprintf(
        cmd, 1024,
        "ffmpeg -hide_banner -loglevel error -i \"%s\" -f rawvideo -pix_fmt %s -s %dx%d -",
        p, gray ? "gray" : "bgr24", w, h);
    c->pipe = popen(cmd, "r");
    if (!c->pipe) {
        free(c);
        return NULL;
    }
    return c;
}

static int ffmpeg_read(FFmpegCap *c, unsigned char *b) {
    return fread(b, 1, (size_t)c->bytes_per_frame, c->pipe) == (size_t)c->bytes_per_frame;
}

static void ffmpeg_close(FFmpegCap *c) {
    if (c) {
        pclose(c->pipe);
        free(c);
    }
}

static void bgr_to_gray(const unsigned char *b, int w, int h, unsigned char *g) {
    for (int i = 0; i < w * h; i++)
        g[i] = (unsigned char)(0.299f * b[i * 3 + 2] + 0.587f * b[i * 3 + 1] + 0.114f * b[i * 3]);
}

static void downsample2x(const unsigned char *src, int sw, int sh, unsigned char *dst) {
    int dw = sw / 2, dh = sh / 2;
    for (int y = 0; y < dh; y++)
        for (int x = 0; x < dw; x++)
            dst[y * dw + x] = src[(y * 2) * sw + (x * 2)];
}

static float get_pixel_bilinear(const unsigned char *g, int w, int h, float x, float y) {
    int ix = (int)x, iy = (int)y;
    float dx = x - ix, dy = y - iy;
    if (ix < 0 || ix >= w - 1 || iy < 0 || iy >= h - 1)
        return 0;
    return (1 - dx) * (1 - dy) * g[iy * w + ix] + dx * (1 - dy) * g[iy * w + ix + 1] +
           (1 - dx) * dy * g[(iy + 1) * w + ix] + dx * dy * g[(iy + 1) * w + ix + 1];
}

static void blur_3x3(const unsigned char *src, int w, int h, unsigned char *dst) {
#pragma omp parallel for collapse(2)
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++) {
            int s = src[(y - 1) * w + x - 1] + 2 * src[(y - 1) * w + x] + src[(y - 1) * w + x + 1] +
                    2 * src[y * w + x - 1] + 4 * src[y * w + x] + 2 * src[y * w + x + 1] +
                    src[(y + 1) * w + x - 1] + 2 * src[(y + 1) * w + x] + src[(y + 1) * w + x + 1];
            dst[y * w + x] = (unsigned char)(s >> 4);
        }
}

#endif
