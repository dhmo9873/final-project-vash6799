#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm.h>

/* ── IIO paths ────────────────────────────────────────────── */
#define ACCEL_DEV    "/sys/bus/iio/devices/iio:device1"
#define GYRO_DEV     "/sys/bus/iio/devices/iio:device0"
#define ACCEL_SCALE  0.000598205
#define GYRO_SCALE   0.000152716
#define TILT_DEG     45.0

/* ── Colors (XRGB8888) ───────────────────────────────────── */
#define COL_BG       0xFF0A0A0A
#define COL_WHITE    0xFFFFFFFF
#define COL_GREEN    0xFF00FF66
#define COL_RED      0xFFFF3333
#define COL_YELLOW   0xFFFFCC00
#define COL_GRAY     0xFF888888
#define COL_BLUE     0xFF4488FF

/* ── DRM state ───────────────────────────────────────────── */
typedef struct {
    int              fd;
    uint32_t         conn_id;
    uint32_t         crtc_id;
    uint32_t         fb_id;
    uint32_t         buf_id;
    uint32_t         width;
    uint32_t         height;
    uint32_t         pitch;
    uint64_t         size;
    uint32_t        *map;
    drmModeModeInfo  mode;
    drmModeCrtcPtr   saved_crtc;
} drm_dev_t;

static volatile int g_running = 1;
static void sig_handler(int s) { (void)s; g_running = 0; }

/* ── Minimal 8×8 bitmap font (printable ASCII 32–126) ───── */
/* Each char = 8 bytes, one byte per row, MSB = left pixel   */
static const uint8_t font8x8[95][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, /* '!' */
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, /* '"' */
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, /* '#' */
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, /* '$' */
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, /* '%' */
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, /* '&' */
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, /* ''' */
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, /* '(' */
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, /* ')' */
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, /* '*' */
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, /* '+' */
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, /* ',' */
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, /* '-' */
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, /* '.' */
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, /* '/' */
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, /* '0' */
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, /* '1' */
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, /* '2' */
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, /* '3' */
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, /* '4' */
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, /* '5' */
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, /* '6' */
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, /* '7' */
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, /* '8' */
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, /* '9' */
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, /* ':' */
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, /* ';' */
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, /* '<' */
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, /* '=' */
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, /* '>' */
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, /* '?' */
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, /* '@' */
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, /* 'A' */
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, /* 'B' */
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, /* 'C' */
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, /* 'D' */
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, /* 'E' */
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, /* 'F' */
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, /* 'G' */
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, /* 'H' */
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'I' */
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, /* 'J' */
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, /* 'K' */
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, /* 'L' */
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, /* 'M' */
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, /* 'N' */
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, /* 'O' */
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, /* 'P' */
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, /* 'Q' */
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, /* 'R' */
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, /* 'S' */
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'T' */
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, /* 'U' */
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 'V' */
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, /* 'W' */
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, /* 'X' */
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, /* 'Y' */
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, /* 'Z' */
    {0x1E,0x06,0x06,0x06,0x06,0x06,0x1E,0x00}, /* '[' */
    {0x03,0x06,0x0C,0x18,0x30,0x60,0x40,0x00}, /* '\' */
    {0x1E,0x18,0x18,0x18,0x18,0x18,0x1E,0x00}, /* ']' */
    {0x08,0x1C,0x36,0x63,0x00,0x00,0x00,0x00}, /* '^' */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, /* '_' */
    {0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00}, /* '`' */
    {0x00,0x00,0x1E,0x30,0x3E,0x33,0x6E,0x00}, /* 'a' */
    {0x07,0x06,0x06,0x3E,0x66,0x66,0x3B,0x00}, /* 'b' */
    {0x00,0x00,0x1E,0x33,0x03,0x33,0x1E,0x00}, /* 'c' */
    {0x38,0x30,0x30,0x3e,0x33,0x33,0x6E,0x00}, /* 'd' */
    {0x00,0x00,0x1E,0x33,0x3f,0x03,0x1E,0x00}, /* 'e' */
    {0x1C,0x36,0x06,0x0f,0x06,0x06,0x0F,0x00}, /* 'f' */
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x1F}, /* 'g' */
    {0x07,0x06,0x36,0x6E,0x66,0x66,0x67,0x00}, /* 'h' */
    {0x0C,0x00,0x0E,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'i' */
    {0x30,0x00,0x30,0x30,0x30,0x33,0x33,0x1E}, /* 'j' */
    {0x07,0x06,0x66,0x36,0x1E,0x36,0x67,0x00}, /* 'k' */
    {0x0E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, /* 'l' */
    {0x00,0x00,0x33,0x7F,0x7F,0x6B,0x63,0x00}, /* 'm' */
    {0x00,0x00,0x1F,0x33,0x33,0x33,0x33,0x00}, /* 'n' */
    {0x00,0x00,0x1E,0x33,0x33,0x33,0x1E,0x00}, /* 'o' */
    {0x00,0x00,0x3B,0x66,0x66,0x3E,0x06,0x0F}, /* 'p' */
    {0x00,0x00,0x6E,0x33,0x33,0x3E,0x30,0x78}, /* 'q' */
    {0x00,0x00,0x3B,0x6E,0x66,0x06,0x0F,0x00}, /* 'r' */
    {0x00,0x00,0x3E,0x03,0x1E,0x30,0x1F,0x00}, /* 's' */
    {0x08,0x0C,0x3E,0x0C,0x0C,0x2C,0x18,0x00}, /* 't' */
    {0x00,0x00,0x33,0x33,0x33,0x33,0x6E,0x00}, /* 'u' */
    {0x00,0x00,0x33,0x33,0x33,0x1E,0x0C,0x00}, /* 'v' */
    {0x00,0x00,0x63,0x6B,0x7F,0x7F,0x36,0x00}, /* 'w' */
    {0x00,0x00,0x63,0x36,0x1C,0x36,0x63,0x00}, /* 'x' */
    {0x00,0x00,0x33,0x33,0x33,0x3E,0x30,0x1F}, /* 'y' */
    {0x00,0x00,0x3F,0x19,0x0C,0x26,0x3F,0x00}, /* 'z' */
    {0x38,0x0C,0x0C,0x07,0x0C,0x0C,0x38,0x00}, /* '{' */
    {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00}, /* '|' */
    {0x07,0x0C,0x0C,0x38,0x0C,0x0C,0x07,0x00}, /* '}' */
    {0x6E,0x3B,0x00,0x00,0x00,0x00,0x00,0x00}, /* '~' */
};

/* ── Draw pixel ──────────────────────────────────────────── */
static inline void put_pixel(drm_dev_t *d, int x, int y, uint32_t col)
{
    if (x < 0 || y < 0 || (uint32_t)x >= d->width || (uint32_t)y >= d->height)
        return;
    d->map[y * (d->pitch / 4) + x] = col;
}

/* ── Draw char (scale = pixel multiplier) ────────────────── */
static void draw_char(drm_dev_t *d, int x, int y, char c,
                      uint32_t fg, int scale)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t *bmp = font8x8[(uint8_t)(c - 32)];
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            //if (bmp[row] & (0x80 >> col)) {
            if (bmp[row] & (1 << col)) {
                for (int sy = 0; sy < scale; sy++)
                    for (int sx = 0; sx < scale; sx++)
                        put_pixel(d,
                            x + col*scale + sx,
                            y + row*scale + sy, fg);
            }
        }
    }
}

/* ── Draw string ─────────────────────────────────────────── */
static void draw_string(drm_dev_t *d, int x, int y,
                        const char *s, uint32_t fg, int scale)
{
    for (; *s; s++, x += 8 * scale)
        draw_char(d, x, y, *s, fg, scale);
}

/* ── Fill rect ───────────────────────────────────────────── */
static void fill_rect(drm_dev_t *d, int x, int y, int w, int h, uint32_t col)
{
    for (int row = y; row < y + h; row++)
        for (int col2 = x; col2 < x + w; col2++)
            put_pixel(d, col2, row, col);
}

/* ── Horizontal bar (value -1.0..1.0 mapped to bar width) ── */
static void draw_bar(drm_dev_t *d, int x, int y, int w, int h,
                     double val, uint32_t col)
{
    fill_rect(d, x, y, w, h, 0xFF222222);          /* background */
    int mid = x + w / 2;
    int fill = (int)(val * (w / 2));
    if (fill > 0)
        fill_rect(d, mid, y, fill, h, col);
    else if (fill < 0)
        fill_rect(d, mid + fill, y, -fill, h, col);
    put_pixel(d, mid, y,     0xFFFFFFFF);           /* centre mark */
    put_pixel(d, mid, y+h-1, 0xFFFFFFFF);
}

/* ── IIO read helper ─────────────────────────────────────── */
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

/* ── DRM setup ───────────────────────────────────────────── */
static int drm_open(drm_dev_t *d, const char *node)
{
    d->fd = open(node, O_RDWR | O_CLOEXEC);
    if (d->fd < 0) { perror("open drm"); return -1; }

    drmModeResPtr res = drmModeGetResources(d->fd);
    if (!res) { fprintf(stderr, "drmModeGetResources failed\n"); return -1; }

    /* Find first connected connector */
    drmModeConnectorPtr conn = NULL;
    for (int i = 0; i < res->count_connectors && !conn; i++) {
        drmModeConnectorPtr c =
            drmModeGetConnector(d->fd, res->connectors[i]);
        if (c && c->connection == DRM_MODE_CONNECTED && c->count_modes > 0) {
            conn = c;
            d->conn_id = c->connector_id;
        } else {
            drmModeFreeConnector(c);
        }
    }
    if (!conn) { fprintf(stderr, "No connected connector\n"); return -1; }

    /* Use first (preferred) mode */
    d->mode  = conn->modes[0];
    d->width = d->mode.hdisplay;
    d->height= d->mode.vdisplay;

    /* Find CRTC via encoder */
    drmModeEncoderPtr enc =
        drmModeGetEncoder(d->fd, conn->encoder_id);
    if (!enc) {
        /* pick first valid crtc from connector */
        for (int i = 0; i < conn->count_encoders; i++) {
            enc = drmModeGetEncoder(d->fd, conn->encoders[i]);
            if (enc && enc->crtc_id) break;
            drmModeFreeEncoder(enc); enc = NULL;
        }
    }
    if (!enc) { fprintf(stderr, "No encoder\n"); return -1; }
    d->crtc_id   = enc->crtc_id;
    d->saved_crtc= drmModeGetCrtc(d->fd, d->crtc_id);
    drmModeFreeEncoder(enc);
    drmModeFreeConnector(conn);
    drmModeFreeResources(res);

    /* Create dumb buffer */
    struct drm_mode_create_dumb creq = {
        .width  = d->width,
        .height = d->height,
        .bpp    = 32,
    };
    if (drmIoctl(d->fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq)) {
        perror("CREATE_DUMB"); return -1;
    }
    d->buf_id = creq.handle;
    d->pitch  = creq.pitch;
    d->size   = creq.size;

    /* Add framebuffer */
    if (drmModeAddFB(d->fd, d->width, d->height, 24, 32,
                     d->pitch, d->buf_id, &d->fb_id)) {
        perror("AddFB"); return -1;
    }

    /* Map dumb buffer */
    struct drm_mode_map_dumb mreq = { .handle = d->buf_id };
    if (drmIoctl(d->fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq)) {
        perror("MAP_DUMB"); return -1;
    }
    d->map = mmap(NULL, d->size, PROT_READ | PROT_WRITE,
                  MAP_SHARED, d->fd, mreq.offset);
    if (d->map == MAP_FAILED) { perror("mmap"); return -1; }

    /* Activate CRTC */
    if (drmModeSetCrtc(d->fd, d->crtc_id, d->fb_id,
                       0, 0, &d->conn_id, 1, &d->mode)) {
        perror("SetCrtc"); return -1;
    }

    printf("DRM: %ux%u pitch=%u fb=%u\n",
           d->width, d->height, d->pitch, d->fb_id);
    return 0;
}

static void drm_close(drm_dev_t *d)
{
    if (d->saved_crtc) {
        drmModeSetCrtc(d->fd, d->saved_crtc->crtc_id,
                       d->saved_crtc->buffer_id,
                       d->saved_crtc->x, d->saved_crtc->y,
                       &d->conn_id, 1, &d->saved_crtc->mode);
        drmModeFreeCrtc(d->saved_crtc);
    }
    if (d->map)
        munmap(d->map, d->size);
    struct drm_mode_destroy_dumb dreq = { .handle = d->buf_id };
    drmIoctl(d->fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    close(d->fd);
}

/* ── Render one frame ────────────────────────────────────── */
static void render(drm_dev_t *d,
                   double pitch, double roll,
                   double ax, double ay, double az,
                   double gx, double gy, double gz,
                   int alert)
{
    int W = d->width, H = d->height;
    uint32_t status_col = alert ? COL_RED : COL_GREEN;

    /* Clear */
    memset(d->map, 0x0A, d->size);

    /* ── Title bar ── */
    fill_rect(d, 0, 0, W, 52, 0xFF1A1A2E);
    draw_string(d, 20, 12, "IMU TELEMETRY HUD",
                COL_BLUE, 3);

    /* ── Status badge ── */
    const char *status_str = alert ? "TILT ALERT" : "  OK  ";
    fill_rect(d, W - 260, 8, 240, 38,
              alert ? 0xFF4A0000 : 0xFF003A00);
    draw_string(d, W - 248, 16, status_str, status_col, 3);

    /* ── Divider ── */
    fill_rect(d, 0, 52, W, 3, COL_BLUE);

    /* ── Pitch / Roll labels + values ── */
    char buf[64];
    int lx = 40, ly = 80;

    draw_string(d, lx, ly, "Pitch:", COL_GRAY, 2);
    snprintf(buf, sizeof(buf), "%+7.2f deg", pitch);
    draw_string(d, lx + 120, ly, buf,
                fabs(pitch) > TILT_DEG ? COL_RED : COL_WHITE, 2);

    ly += 40;
    draw_string(d, lx, ly, "Roll: ", COL_GRAY, 2);
    snprintf(buf, sizeof(buf), "%+7.2f deg", roll);
    draw_string(d, lx + 120, ly, buf,
                fabs(roll) > TILT_DEG ? COL_RED : COL_WHITE, 2);

    /* ── Pitch bar ── */
    ly += 50;
    draw_string(d, lx, ly, "P", COL_GRAY, 2);
    draw_bar(d, lx + 30, ly, W - 80, 20,
             pitch / 90.0,
             fabs(pitch) > TILT_DEG ? COL_RED : COL_BLUE);

    /* ── Roll bar ── */
    ly += 35;
    draw_string(d, lx, ly, "R", COL_GRAY, 2);
    draw_bar(d, lx + 30, ly, W - 80, 20,
             roll / 180.0,
             fabs(roll) > TILT_DEG ? COL_RED : COL_GREEN);

    /* ── Accel ── */
    ly += 55;
    fill_rect(d, 0, ly - 5, W, 2, 0xFF333333);
    ly += 10;
    draw_string(d, lx, ly, "ACCEL (m/s2)", COL_YELLOW, 2);
    ly += 30;
    snprintf(buf, sizeof(buf),
             "X: %+6.3f   Y: %+6.3f   Z: %+6.3f", ax, ay, az);
    draw_string(d, lx, ly, buf, COL_WHITE, 2);

    /* ── Gyro ── */
    ly += 50;
    draw_string(d, lx, ly, "GYRO  (rad/s)", COL_YELLOW, 2);
    ly += 30;
    snprintf(buf, sizeof(buf),
             "X: %+7.4f  Y: %+7.4f  Z: %+7.4f", gx, gy, gz);
    draw_string(d, lx, ly, buf, COL_WHITE, 2);

    /* ── Bottom border ── */
    fill_rect(d, 0, H - 4, W, 4, COL_BLUE);
}

/* ── main ────────────────────────────────────────────────── */
int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    drm_dev_t dev = {0};
    if (drm_open(&dev, "/dev/dri/card0") < 0) {
        fprintf(stderr, "DRM init failed. "
                "Is X11 stopped? (systemctl stop xserver-nodm)\n");
        return 1;
    }

    printf("Rendering at %ux%u. Press Ctrl+C to exit.\n",
           dev.width, dev.height);

    while (g_running) {
        long raw;
        double ax = 0, ay = 0, az = 0;
        double gx = 0, gy = 0, gz = 0;

        if (read_raw(ACCEL_DEV, "in_accel_x_raw", &raw) == 0) ax = raw * ACCEL_SCALE;
        if (read_raw(ACCEL_DEV, "in_accel_y_raw", &raw) == 0) ay = raw * ACCEL_SCALE;
        if (read_raw(ACCEL_DEV, "in_accel_z_raw", &raw) == 0) az = raw * ACCEL_SCALE;
        if (read_raw(GYRO_DEV,  "in_anglvel_x_raw", &raw) == 0) gx = raw * GYRO_SCALE;
        if (read_raw(GYRO_DEV,  "in_anglvel_y_raw", &raw) == 0) gy = raw * GYRO_SCALE;
        if (read_raw(GYRO_DEV,  "in_anglvel_z_raw", &raw) == 0) gz = raw * GYRO_SCALE;

        double pitch = atan2(-ax, sqrt(ay*ay + az*az)) * (180.0 / M_PI);
        double roll  = atan2( ay, az)                  * (180.0 / M_PI);
        int    alert = (fabs(pitch) > TILT_DEG || fabs(roll) > TILT_DEG);

        render(&dev, pitch, roll, ax, ay, az, gx, gy, gz, alert);

        usleep(100000); /* 10 Hz refresh */
    }

    drm_close(&dev);
    printf("DRM HUD stopped.\n");
    return 0;
}
