#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <syslog.h>
#include <time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/videodev2.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm.h>

/* ══════════════════════════════════════════════════════════
 * CONFIGURATION
 * ══════════════════════════════════════════════════════════ */
#define ACCEL_DEV        "/sys/bus/iio/devices/iio:device1"
#define GYRO_DEV         "/sys/bus/iio/devices/iio:device0"
#define ACCEL_SCALE      0.000598205
#define GYRO_SCALE       0.000152716

#define SUBDEV           "/dev/v4l-subdev0"
#define DRM_NODE         "/dev/dri/card0"

#define IMU_POLL_HZ      50
#define IMU_POLL_NS      (1000000000L / IMU_POLL_HZ)
#define DRM_REFRESH_HZ   10
#define DRM_REFRESH_US   (1000000 / DRM_REFRESH_HZ)
#define V4L2_POLL_HZ     20
#define V4L2_POLL_US     (1000000 / V4L2_POLL_HZ)

#define TILT_DEG         45.0
#define GYRO_FAST_RAD    1.0
#define FREEFALL_G       1.0

#define EXPOSURE_MIN     4
#define EXPOSURE_MAX     3522
#define EXPOSURE_NOM     3000
#define GAIN_MIN         0
#define GAIN_MAX         232
#define GAIN_NOM         150

/* V4L2 control IDs */
#define CID_EXPOSURE      0x00980911
#define CID_ANALOGUE_GAIN 0x009e0903
#define CID_TEST_PATTERN  0x009f0903

/* Colors XRGB8888 */
#define COL_BG    0xFF0A0A0A
#define COL_WHITE 0xFFFFFFFF
#define COL_GREEN 0xFF00FF66
#define COL_RED   0xFFFF3333
#define COL_YELL  0xFFFFCC00
#define COL_GRAY  0xFF888888
#define COL_BLUE  0xFF4488FF

/* ── Camera / debayer dimensions ─────────────────────────
 * Sensor native: 3280×2464 RGGB10 (2 bytes/pixel)
 * Debayer output: half resolution per axis = 1640×1232 XRGB
 * ────────────────────────────────────────────────────────*/
#define CAM_W    3280
#define CAM_H    2464
#define CAM_BUFS 4
#define DEBAY_W  (CAM_W / 2)   /* 1640 */
#define DEBAY_H  (CAM_H / 2)   /* 1232 */

/* ══════════════════════════════════════════════════════════
 * SHARED STATE
 * ══════════════════════════════════════════════════════════ */
typedef struct {
    double ax, ay, az;
    double gx, gy, gz;
    double pitch_deg;
    double roll_deg;
    double tilt_mag;
    double gyro_mag;
    double accel_mag;
    int    tilt_alert;
    int    freefall;
} imu_state_t;

static imu_state_t     g_imu;
static pthread_mutex_t g_mutex       = PTHREAD_MUTEX_INITIALIZER;
static volatile int    g_running     = 1;

/* camera register state: v4l2_thread writes, drm_thread reads */
static volatile int    g_cam_exp     = EXPOSURE_NOM;
static volatile int    g_cam_gain    = GAIN_NOM;
static volatile int    g_cam_pat     = 0;

/* debayered frame: cam_thread writes, drm_thread reads
 * size = DEBAY_W * DEBAY_H * 4 bytes = ~8 MB              */
static uint32_t       *g_frame       = NULL;
static pthread_mutex_t g_frame_mutex = PTHREAD_MUTEX_INITIALIZER;

typedef struct {
    void   *start;
    size_t  length;
} cam_buf_t;

static void sig_handler(int s) { (void)s; g_running = 0; }

/* ══════════════════════════════════════════════════════════
 * GENERIC HELPERS
 * ══════════════════════════════════════════════════════════ */
static int read_raw(const char *dev, const char *ch, long *out)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", dev, ch);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int r = fscanf(f, "%ld", out);
    fclose(f);
    return (r == 1) ? 0 : -1;
}

static int clamp(int v, int lo, int hi)
{ return v < lo ? lo : v > hi ? hi : v; }

static void ts_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

/* ══════════════════════════════════════════════════════════
 * DRM
 * ══════════════════════════════════════════════════════════ */
typedef struct {
    int              fd;
    uint32_t         conn_id, crtc_id, fb_id, buf_id;
    uint32_t         width, height, pitch;
    uint64_t         size;
    uint32_t        *map;
    drmModeModeInfo  mode;
    drmModeCrtcPtr   saved_crtc;
} drm_dev_t;

/* ── 8×8 bitmap font (ASCII 32–126) ── */
static const uint8_t font8x8[95][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00},
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00},
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00},
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00},
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00},
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00},
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00},
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00},
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00},
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06},
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00},
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00},
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00},
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00},
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00},
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00},
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00},
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00},
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00},
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00},
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00},
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00},
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00},
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06},
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00},
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00},
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00},
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00},
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00},
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00},
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00},
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00},
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00},
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00},
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00},
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00},
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00},
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00},
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00},
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00},
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00},
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00},
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00},
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00},
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00},
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00},
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00},
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00},
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00},
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00},
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00},
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00},
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00},
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00},
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00},
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00},
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00},
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00},
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF},
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00},
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00},
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00},
    {0x38,0x30,0x30,0x3e,0x33,0x33,0x6E,0x00},
    {0x00,0x00,0x1E,0x33,0x3f,0x03,0x1E,0x00},
    {0x1C,0x36,0x06,0x0f,0x06,0x06,0x0F,0x00},
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F},
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00},
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00},
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E},
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00},
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00},
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00},
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00},
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00},
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F},
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78},
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00},
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00},
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00},
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00},
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00},
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00},
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00},
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F},
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00},
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00},
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00},
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00},
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00},
};

static inline void put_pixel(drm_dev_t *d, int x, int y, uint32_t c)
{
    if (x<0||y<0||(uint32_t)x>=d->width||(uint32_t)y>=d->height) return;
    d->map[y*(d->pitch/4)+x] = c;
}

static void fill_rect(drm_dev_t *d, int x, int y, int w, int h, uint32_t c)
{
    for (int r=y; r<y+h; r++)
        for (int col=x; col<x+w; col++)
            put_pixel(d, col, r, c);
}

static void draw_char(drm_dev_t *d, int x, int y, char ch, uint32_t fg, int sc)
{
    if (ch<32||ch>126) ch='?';
    const uint8_t *b = font8x8[(uint8_t)(ch-32)];
    for (int row=0; row<8; row++)
        for (int col=0; col<8; col++)
            if (b[row] & (1<<col))
                for (int sy=0; sy<sc; sy++)
                    for (int sx=0; sx<sc; sx++)
                        put_pixel(d, x+col*sc+sx, y+row*sc+sy, fg);
}

static void draw_string(drm_dev_t *d, int x, int y,
                        const char *s, uint32_t fg, int sc)
{ for (; *s; s++, x+=8*sc) draw_char(d, x, y, *s, fg, sc); }

static void draw_bar(drm_dev_t *d, int x, int y, int w, int h,
                     double val, uint32_t col)
{
    fill_rect(d, x, y, w, h, 0xFF222222);
    int mid  = x + w/2;
    int fill = (int)(val * (w/2));
    if (fill > 0)      fill_rect(d, mid,       y, fill,  h, col);
    else if (fill < 0) fill_rect(d, mid+fill,  y, -fill, h, col);
    put_pixel(d, mid, y,     0xFFFFFFFF);
    put_pixel(d, mid, y+h-1, 0xFFFFFFFF);
}

/* ── DRM init ── */
static int drm_open(drm_dev_t *d, const char *node)
{
    d->fd = open(node, O_RDWR|O_CLOEXEC);
    if (d->fd < 0) { perror("open drm"); return -1; }

    drmModeResPtr res = drmModeGetResources(d->fd);
    if (!res) { fprintf(stderr,"drmModeGetResources failed\n"); return -1; }

    drmModeConnectorPtr conn = NULL;
    for (int i=0; i<res->count_connectors && !conn; i++) {
        drmModeConnectorPtr c = drmModeGetConnector(d->fd, res->connectors[i]);
        if (c && c->connection==DRM_MODE_CONNECTED && c->count_modes>0) {
            conn = c; d->conn_id = c->connector_id;
        } else drmModeFreeConnector(c);
    }
    if (!conn) { fprintf(stderr,"No connected connector\n"); return -1; }

    d->mode   = conn->modes[0];
    d->width  = d->mode.hdisplay;
    d->height = d->mode.vdisplay;

    drmModeEncoderPtr enc = drmModeGetEncoder(d->fd, conn->encoder_id);
    if (!enc) {
        for (int i=0; i<conn->count_encoders; i++) {
            enc = drmModeGetEncoder(d->fd, conn->encoders[i]);
            if (enc && enc->crtc_id) break;
            drmModeFreeEncoder(enc); enc = NULL;
        }
    }
    if (!enc) { fprintf(stderr,"No encoder\n"); return -1; }
    d->crtc_id    = enc->crtc_id;
    d->saved_crtc = drmModeGetCrtc(d->fd, d->crtc_id);
    drmModeFreeEncoder(enc);
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    struct drm_mode_create_dumb cr = {
        .width=d->width, .height=d->height, .bpp=32 };
    if (drmIoctl(d->fd, DRM_IOCTL_MODE_CREATE_DUMB, &cr)) {
        perror("CREATE_DUMB"); return -1; }
    d->buf_id = cr.handle; d->pitch = cr.pitch; d->size = cr.size;

    if (drmModeAddFB(d->fd, d->width, d->height, 24, 32,
                     d->pitch, d->buf_id, &d->fb_id)) {
        perror("AddFB"); return -1; }

    struct drm_mode_map_dumb mr = { .handle = d->buf_id };
    if (drmIoctl(d->fd, DRM_IOCTL_MODE_MAP_DUMB, &mr)) {
        perror("MAP_DUMB"); return -1; }
    d->map = mmap(NULL, d->size, PROT_READ|PROT_WRITE,
                  MAP_SHARED, d->fd, mr.offset);
    if (d->map == MAP_FAILED) { perror("mmap"); return -1; }

    if (drmModeSetCrtc(d->fd, d->crtc_id, d->fb_id, 0, 0,
                       &d->conn_id, 1, &d->mode)) {
        perror("SetCrtc"); return -1; }

    fprintf(stderr, "DRM: %ux%u\n", d->width, d->height);
    return 0;
}

static void drm_cleanup(drm_dev_t *d)
{
    if (d->saved_crtc) {
        drmModeSetCrtc(d->fd, d->saved_crtc->crtc_id,
            d->saved_crtc->buffer_id, d->saved_crtc->x, d->saved_crtc->y,
            &d->conn_id, 1, &d->saved_crtc->mode);
        drmModeFreeCrtc(d->saved_crtc);
    }
    if (d->map) munmap(d->map, d->size);
    struct drm_mode_destroy_dumb dr = { .handle = d->buf_id };
    drmIoctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dr);
    close(d->fd);
}

/* ── Blit debayered camera frame as background ──────────
 * Source: g_frame[DEBAY_W × DEBAY_H] XRGB8888
 * Dest:   DRM framebuffer [d->width × d->height]
 * Uses nearest-neighbour scaling.
 * ─────────────────────────────────────────────────────── */
static void blit_camera(drm_dev_t *d)
{
    pthread_mutex_lock(&g_frame_mutex);
    if (!g_frame) { pthread_mutex_unlock(&g_frame_mutex); return; }

    uint32_t dw     = d->width;
    uint32_t dh     = d->height;
    uint32_t pitch4 = d->pitch / 4;

    for (uint32_t dy = 0; dy < dh; dy++) {
        uint32_t sy = dy * DEBAY_H / dh;
        for (uint32_t dx = 0; dx < dw; dx++) {
            uint32_t sx = dx * DEBAY_W / dw;
            d->map[dy * pitch4 + dx] = g_frame[sy * DEBAY_W + sx];
        }
    }
    pthread_mutex_unlock(&g_frame_mutex);
}

/* ── HUD render ── */
static void drm_render(drm_dev_t *d, imu_state_t *s,
                       int exposure, int gain, int pattern)
{
    int W = d->width, H = d->height;
    uint32_t sc = s->tilt_alert ? COL_RED : COL_GREEN;
    char buf[80];

    blit_camera(d);   /* live camera feed as background */

    /* title bar */
    fill_rect(d, 0, 0, W, 52, 0xFF1A1A2E);
    draw_string(d, 20, 12, "REACTIVE IMU HUD", COL_BLUE, 3);

    /* status badge */
    const char *st = s->freefall   ? "FREEFALL"   :
                     s->tilt_alert ? "TILT ALERT" : "  OK  ";
    uint32_t bc    = s->freefall   ? 0xFF4A2000   :
                     s->tilt_alert ? 0xFF4A0000   : 0xFF003A00;
    fill_rect(d, W-280, 8, 260, 38, bc);
    draw_string(d, W-268, 16, st, sc, 3);

    fill_rect(d, 0, 52, W, 3, COL_BLUE);

    /* pitch / roll values */
    int lx = 40, ly = 80;
    draw_string(d, lx, ly, "Pitch:", COL_GRAY, 2);
    snprintf(buf, sizeof(buf), "%+7.2f deg", s->pitch_deg);
    draw_string(d, lx+120, ly, buf,
        fabs(s->pitch_deg)>TILT_DEG ? COL_RED : COL_WHITE, 2);

    ly += 40;
    draw_string(d, lx, ly, "Roll: ", COL_GRAY, 2);
    snprintf(buf, sizeof(buf), "%+7.2f deg", s->roll_deg);
    draw_string(d, lx+120, ly, buf,
        fabs(s->roll_deg)>TILT_DEG ? COL_RED : COL_WHITE, 2);

    /* bars */
    ly += 50;
    draw_string(d, lx, ly, "P", COL_GRAY, 2);
    draw_bar(d, lx+30, ly, W-80, 20, s->pitch_deg/90.0,
        fabs(s->pitch_deg)>TILT_DEG ? COL_RED : COL_BLUE);

    ly += 35;
    draw_string(d, lx, ly, "R", COL_GRAY, 2);
    draw_bar(d, lx+30, ly, W-80, 20, s->roll_deg/180.0,
        fabs(s->roll_deg)>TILT_DEG ? COL_RED : COL_GREEN);

    /* accel */
    ly += 55;
    fill_rect(d, 0, ly-5, W, 2, 0xFF333333);
    ly += 10;
    draw_string(d, lx, ly, "ACCEL (m/s2)", COL_YELL, 2);
    ly += 30;
    snprintf(buf, sizeof(buf), "X:%+6.3f  Y:%+6.3f  Z:%+6.3f",
             s->ax, s->ay, s->az);
    draw_string(d, lx, ly, buf, COL_WHITE, 2);

    /* gyro */
    ly += 45;
    draw_string(d, lx, ly, "GYRO  (rad/s)", COL_YELL, 2);
    ly += 30;
    snprintf(buf, sizeof(buf), "X:%+7.4f Y:%+7.4f Z:%+7.4f",
             s->gx, s->gy, s->gz);
    draw_string(d, lx, ly, buf, COL_WHITE, 2);

    /* camera state */
    ly += 55;
    fill_rect(d, 0, ly-5, W, 2, 0xFF333333);
    ly += 10;
    draw_string(d, lx, ly, "CAMERA", COL_YELL, 2);
    ly += 30;
    snprintf(buf, sizeof(buf), "Exp:%-4d  Gain:%-3d  Pat:%d",
             exposure, gain, pattern);
    draw_string(d, lx, ly, buf, COL_WHITE, 2);

    /* bottom border */
    fill_rect(d, 0, H-4, W, 4, COL_BLUE);
}

/* ══════════════════════════════════════════════════════════
 * V4L2 HELPERS
 * ══════════════════════════════════════════════════════════ */
static int v4l2_set(int fd, uint32_t id, int32_t val)
{
    struct v4l2_ext_control  ctrl  = { .id=id, .value=val };
    struct v4l2_ext_controls ctrls = {
        .which=V4L2_CTRL_WHICH_CUR_VAL, .count=1, .controls=&ctrl };
    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls);
}

static int tilt_to_exposure(double t)
{
    if (t >= TILT_DEG) return EXPOSURE_MIN;
    return clamp((int)(EXPOSURE_NOM * (1.0 - 0.8*(t/TILT_DEG))),
                 EXPOSURE_MIN, EXPOSURE_MAX);
}

static int gyro_to_gain(double g)
{
    if (g >= GYRO_FAST_RAD) return GAIN_MAX;
    return clamp((int)(GAIN_MAX * (g/GYRO_FAST_RAD)), GAIN_MIN, GAIN_MAX);
}

/* ══════════════════════════════════════════════════════════
 * THREAD 1 — IMU POLL @ 50 Hz
 * ══════════════════════════════════════════════════════════ */
static void *imu_thread(void *arg)
{
    (void)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_running) {
        long raw;
        double ax=0,ay=0,az=0,gx=0,gy=0,gz=0;

        if (read_raw(ACCEL_DEV,"in_accel_x_raw",&raw)==0) ax=raw*ACCEL_SCALE;
        if (read_raw(ACCEL_DEV,"in_accel_y_raw",&raw)==0) ay=raw*ACCEL_SCALE;
        if (read_raw(ACCEL_DEV,"in_accel_z_raw",&raw)==0) az=raw*ACCEL_SCALE;
        if (read_raw(GYRO_DEV, "in_anglvel_x_raw",&raw)==0) gx=raw*GYRO_SCALE;
        if (read_raw(GYRO_DEV, "in_anglvel_y_raw",&raw)==0) gy=raw*GYRO_SCALE;
        if (read_raw(GYRO_DEV, "in_anglvel_z_raw",&raw)==0) gz=raw*GYRO_SCALE;

        double pitch = atan2(-ax, sqrt(ay*ay+az*az)) * (180.0/M_PI);
        double roll  = atan2( ay, az)                * (180.0/M_PI);
        double tilt  = fmax(fabs(pitch), fabs(roll));
        double gmag  = sqrt(gx*gx + gy*gy + gz*gz);
        double amag  = sqrt(ax*ax + ay*ay + az*az);

        pthread_mutex_lock(&g_mutex);
        g_imu.ax=ax; g_imu.ay=ay; g_imu.az=az;
        g_imu.gx=gx; g_imu.gy=gy; g_imu.gz=gz;
        g_imu.pitch_deg  = pitch;
        g_imu.roll_deg   = roll;
        g_imu.tilt_mag   = tilt;
        g_imu.gyro_mag   = gmag;
        g_imu.accel_mag  = amag;
        g_imu.tilt_alert = (tilt  > TILT_DEG);
        g_imu.freefall   = (amag  < FREEFALL_G);
        pthread_mutex_unlock(&g_mutex);

        ts_add_ns(&next, IMU_POLL_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}

/* ══════════════════════════════════════════════════════════
 * THREAD 2 — DRM HUD @ 10 Hz
 * ══════════════════════════════════════════════════════════ */
static void *drm_thread(void *arg)
{
    (void)arg;
    drm_dev_t dev = {0};

    if (drm_open(&dev, DRM_NODE) < 0) {
        fprintf(stderr, "DRM init failed — stop X first\n");
        g_running = 0;
        return NULL;
    }

    while (g_running) {
        imu_state_t s;
        pthread_mutex_lock(&g_mutex);
        s = g_imu;
        pthread_mutex_unlock(&g_mutex);

        drm_render(&dev, &s, g_cam_exp, g_cam_gain, g_cam_pat);
        usleep(DRM_REFRESH_US);
    }

    drm_cleanup(&dev);
    return NULL;
}

/* ══════════════════════════════════════════════════════════
 * THREAD 3 — V4L2 CAMERA CONTROL @ 20 Hz
 * ══════════════════════════════════════════════════════════ */
static void *v4l2_thread(void *arg)
{
    (void)arg;
    int fd = open(SUBDEV, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", SUBDEV, strerror(errno));
        return NULL;
    }

    int last_exp=EXPOSURE_NOM, last_gain=GAIN_NOM, last_pat=0;

    while (g_running) {
        imu_state_t s;
        pthread_mutex_lock(&g_mutex);
        s = g_imu;
        pthread_mutex_unlock(&g_mutex);

        int want_pat = s.freefall ? 1 : 0;
        if (want_pat != last_pat) {
            v4l2_set(fd, CID_TEST_PATTERN, want_pat);
            fprintf(stderr, "[V4L2] test_pattern=%d (accel=%.2f)\n",
                    want_pat, s.accel_mag);
            last_pat = want_pat; g_cam_pat = want_pat;
        }

        int want_exp = tilt_to_exposure(s.tilt_mag);
        if (abs(want_exp - last_exp) > 10) {
            v4l2_set(fd, CID_EXPOSURE, want_exp);
            fprintf(stderr, "[V4L2] exposure=%d (tilt=%.1f)\n",
                    want_exp, s.tilt_mag);
            last_exp = want_exp; g_cam_exp = want_exp;
        }

        int want_gain = gyro_to_gain(s.gyro_mag);
        if (abs(want_gain - last_gain) > 5) {
            v4l2_set(fd, CID_ANALOGUE_GAIN, want_gain);
            fprintf(stderr, "[V4L2] gain=%d (gyro=%.3f)\n",
                    want_gain, s.gyro_mag);
            last_gain = want_gain; g_cam_gain = want_gain;
        }

        usleep(V4L2_POLL_US);
    }

    v4l2_set(fd, CID_TEST_PATTERN,  0);
    v4l2_set(fd, CID_EXPOSURE,      EXPOSURE_NOM);
    v4l2_set(fd, CID_ANALOGUE_GAIN, GAIN_NOM);
    close(fd);
    return NULL;
}

/* ══════════════════════════════════════════════════════════
 * THREAD 4 — CAMERA CAPTURE (RGGB10 → XRGB debayer)
 *
 * Sensor boots at 3280×2464 SRGGB10 — no media-ctl needed.
 * Each 2×2 Bayer block → 1 XRGB pixel → DEBAY_W×DEBAY_H output.
 * ══════════════════════════════════════════════════════════ */
/* White balance gains for IMX219 daylight
 * G is reference=1.0, R and B need boosting */
#define WB_R  1.1f
#define WB_B  1.5f

static void rggb10_to_xrgb(const uint8_t *src, uint32_t *dst,
                             int width, int height)
{
    const uint16_t *s = (const uint16_t *)src;
    int ow = width  / 2;
    int oh = height / 2;
    #define CAM_GAIN 6
    #define CLAMP255(v) ((v) > 255 ? 255 : (uint8_t)(v))

    for (int y = 0; y < oh; y++) {
        for (int x = 0; x < ow; x++) {
            int sy = y * 2, sx = x * 2;

            uint16_t R  = s[ sy    * width + sx    ] >> 2;
            uint16_t Gr = s[ sy    * width + sx + 1] >> 2;
            uint16_t Gb = s[(sy+1) * width + sx    ] >> 2;
            uint16_t B  = s[(sy+1) * width + sx + 1] >> 2;
            uint8_t  G  = (uint8_t)((Gr + Gb) / 2);

            /* apply white balance then brightness gain */
            uint8_t r = CLAMP255((uint32_t)(R * WB_R * CAM_GAIN));
            uint8_t g = CLAMP255((uint32_t)(G         * CAM_GAIN));
            uint8_t b = CLAMP255((uint32_t)(B * WB_B * CAM_GAIN));

            dst[y * ow + x] = 0xFF000000 |
                               ((uint32_t)r << 16) |
                               ((uint32_t)g <<  8) |
                               ((uint32_t)b);
        }
    }
}
static void *cam_thread(void *arg)
{
    (void)arg;

    int fd = open("/dev/video0", O_RDWR);
    if (fd < 0) { perror("[CAM] open /dev/video0"); return NULL; }
    fprintf(stderr, "[CAM] opened /dev/video0\n");

    /* set RAW10 format — sensor default, no media-ctl required */
    struct v4l2_format fmt = {0};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_W;
    fmt.fmt.pix.height      = CAM_H;
    fmt.fmt.pix.pixelformat = v4l2_fourcc('R','G','1','0');
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("[CAM] VIDIOC_S_FMT"); close(fd); return NULL;
    }
    fprintf(stderr, "[CAM] fmt RG10 %dx%d stride=%d\n",
            fmt.fmt.pix.width, fmt.fmt.pix.height,
            fmt.fmt.pix.bytesperline);

    struct v4l2_requestbuffers req = {0};
    req.count  = CAM_BUFS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("[CAM] VIDIOC_REQBUFS"); close(fd); return NULL;
    }
    fprintf(stderr, "[CAM] got %d buffers\n", req.count);

    cam_buf_t bufs[CAM_BUFS];
    for (int i = 0; i < (int)req.count; i++) {
        struct v4l2_buffer buf = {0};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        ioctl(fd, VIDIOC_QUERYBUF, &buf);
        bufs[i].length = buf.length;
        bufs[i].start  = mmap(NULL, buf.length,
                               PROT_READ|PROT_WRITE,
                               MAP_SHARED, fd, buf.m.offset);
        if (bufs[i].start == MAP_FAILED) {
            perror("[CAM] mmap"); close(fd); return NULL;
        }
        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("[CAM] VIDIOC_STREAMON"); close(fd); return NULL;
    }
    fprintf(stderr, "[CAM] STREAMON success — debayer to %dx%d\n",
            DEBAY_W, DEBAY_H);

    /* FIX: tmp buffer sized for DEBAYER output, not raw input */
    uint32_t *tmp = malloc(DEBAY_W * DEBAY_H * sizeof(uint32_t));
    if (!tmp) {
        fprintf(stderr, "[CAM] malloc tmp failed\n");
        close(fd); return NULL;
    }

    while (g_running) {
        fd_set fds;
        FD_ZERO(&fds); FD_SET(fd, &fds);
        struct timeval tv = { .tv_sec=1, .tv_usec=0 };
        if (select(fd+1, &fds, NULL, NULL, &tv) <= 0)
            continue;

        struct v4l2_buffer buf = {0};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("[CAM] VIDIOC_DQBUF"); break;
        }

        /* debayer: CAM_W×CAM_H RAW10 → DEBAY_W×DEBAY_H XRGB */
        rggb10_to_xrgb(bufs[buf.index].start, tmp, CAM_W, CAM_H);

        pthread_mutex_lock(&g_frame_mutex);
        /* FIX: copy exactly DEBAY_W*DEBAY_H pixels — matches g_frame alloc */
        memcpy(g_frame, tmp, DEBAY_W * DEBAY_H * sizeof(uint32_t));
        pthread_mutex_unlock(&g_frame_mutex);

        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    free(tmp);
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (int i = 0; i < CAM_BUFS; i++)
        munmap(bufs[i].start, bufs[i].length);
    close(fd);
    fprintf(stderr, "[CAM] stopped\n");
    return NULL;
}

/* ══════════════════════════════════════════════════════════
 * MAIN
 * ══════════════════════════════════════════════════════════ */
int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    /* FIX: allocate exactly DEBAY_W*DEBAY_H — not CAM_W*CAM_H */
    g_frame = malloc(DEBAY_W * DEBAY_H * sizeof(uint32_t));
    if (!g_frame) { perror("malloc g_frame"); return 1; }
    /* FIX: zero only the allocated size */
    memset(g_frame, 0, DEBAY_W * DEBAY_H * sizeof(uint32_t));

    openlog("reactive_imu", LOG_PID|LOG_CONS, LOG_USER);
    syslog(LOG_INFO, "Reactive IMU starting — cam=%dx%d debay=%dx%d",
           CAM_W, CAM_H, DEBAY_W, DEBAY_H);
    fprintf(stderr, "Reactive IMU — 4-thread integration\n");
    fprintf(stderr, "Stop X first: /etc/init.d/xserver-nodm stop\n\n");

    pthread_t t_imu, t_drm, t_v4l2, t_cam;
    pthread_create(&t_imu,  NULL, imu_thread,  NULL);
    pthread_create(&t_cam,  NULL, cam_thread,  NULL);
    pthread_create(&t_drm,  NULL, drm_thread,  NULL);
    pthread_create(&t_v4l2, NULL, v4l2_thread, NULL);

    pthread_join(t_imu,  NULL);
    pthread_join(t_cam,  NULL);
    pthread_join(t_drm,  NULL);
    pthread_join(t_v4l2, NULL);

    free(g_frame);
    syslog(LOG_INFO, "Reactive IMU stopped");
    closelog();
    return 0;
}
