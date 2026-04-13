/* drm_telemetry.c
 *
 * Issues 6 & 11: DRM Telemetry Overlay — Userspace Implementation
 *
 * Allocates a DRM dumb buffer, renders HUD pixel data (text + indicators)
 * into it using a built-in 8×8 bitmap font, and issues the custom
 * DRM_IOCTL_VC4_TELEMETRY_OVERLAY to display it on the secondary plane.
 *
 * BUILT-IN 8×8 FONT:
 *   We embed a minimal ASCII bitmap font for digits 0-9, letters A-Z,
 *   and punctuation needed for the HUD labels. No external font library.
 *   Each character is 8 rows × 8 columns = 8 bytes.
 */

#include "drm_telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm/drm.h>

/* Include our UAPI header — this defines DRM_IOCTL_VC4_TELEMETRY_OVERLAY */
#include "telemetry_types.h"

/* =========================================================================
 * Minimal 8×8 bitmap font
 *
 * Each character is 8 bytes, one byte per row.
 * Bit 7 of each byte = leftmost pixel.
 * Only printable ASCII 0x20 (space) through 0x5A (Z) is included.
 * =========================================================================*/
static const uint8_t font8x8[('Z' - ' ' + 1)][8] = {
    /* 0x20 ' ' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    /* 0x21 '!' */ {0x18,0x18,0x18,0x18,0x00,0x00,0x18,0x00},
    /* 0x22 '"' */ {0x66,0x66,0x24,0x00,0x00,0x00,0x00,0x00},
    /* 0x23 '#' */ {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00},
    /* 0x24 '$' */ {0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00},
    /* 0x25 '%' */ {0x62,0x66,0x0C,0x18,0x30,0x66,0x46,0x00},
    /* 0x26 '&' */ {0x3C,0x66,0x3C,0x38,0x67,0x66,0x3F,0x00},
    /* 0x27 '\''*/ {0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
    /* 0x28 '(' */ {0x0E,0x1C,0x18,0x18,0x18,0x1C,0x0E,0x00},
    /* 0x29 ')' */ {0x70,0x38,0x18,0x18,0x18,0x38,0x70,0x00},
    /* 0x2A '*' */ {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00},
    /* 0x2B '+' */ {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00},
    /* 0x2C ',' */ {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30},
    /* 0x2D '-' */ {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00},
    /* 0x2E '.' */ {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00},
    /* 0x2F '/' */ {0x02,0x06,0x0C,0x18,0x30,0x60,0x40,0x00},
    /* 0x30 '0' */ {0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00},
    /* 0x31 '1' */ {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00},
    /* 0x32 '2' */ {0x3C,0x66,0x06,0x0C,0x18,0x30,0x7E,0x00},
    /* 0x33 '3' */ {0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00},
    /* 0x34 '4' */ {0x06,0x0E,0x1E,0x66,0x7F,0x06,0x06,0x00},
    /* 0x35 '5' */ {0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00},
    /* 0x36 '6' */ {0x3C,0x60,0x60,0x7C,0x66,0x66,0x3C,0x00},
    /* 0x37 '7' */ {0x7E,0x06,0x0C,0x18,0x30,0x30,0x30,0x00},
    /* 0x38 '8' */ {0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00},
    /* 0x39 '9' */ {0x3C,0x66,0x66,0x3E,0x06,0x06,0x3C,0x00},
    /* 0x3A ':' */ {0x00,0x18,0x18,0x00,0x18,0x18,0x00,0x00},
    /* 0x3B ';' */ {0x00,0x18,0x18,0x00,0x18,0x18,0x30,0x00},
    /* 0x3C '<' */ {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00},
    /* 0x3D '=' */ {0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00},
    /* 0x3E '>' */ {0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00},
    /* 0x3F '?' */ {0x3C,0x66,0x06,0x0C,0x18,0x00,0x18,0x00},
    /* 0x40 '@' */ {0x3E,0x63,0x6F,0x69,0x6F,0x60,0x3E,0x00},
    /* 0x41 'A' */ {0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x00},
    /* 0x42 'B' */ {0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00},
    /* 0x43 'C' */ {0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00},
    /* 0x44 'D' */ {0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00},
    /* 0x45 'E' */ {0x7E,0x60,0x60,0x78,0x60,0x60,0x7E,0x00},
    /* 0x46 'F' */ {0x7E,0x60,0x60,0x78,0x60,0x60,0x60,0x00},
    /* 0x47 'G' */ {0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00},
    /* 0x48 'H' */ {0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00},
    /* 0x49 'I' */ {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00},
    /* 0x4A 'J' */ {0x1E,0x0C,0x0C,0x0C,0x6C,0x6C,0x38,0x00},
    /* 0x4B 'K' */ {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00},
    /* 0x4C 'L' */ {0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00},
    /* 0x4D 'M' */ {0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00},
    /* 0x4E 'N' */ {0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00},
    /* 0x4F 'O' */ {0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00},
    /* 0x50 'P' */ {0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00},
    /* 0x51 'Q' */ {0x3C,0x66,0x66,0x66,0x66,0x3C,0x0E,0x00},
    /* 0x52 'R' */ {0x7C,0x66,0x66,0x7C,0x78,0x6C,0x66,0x00},
    /* 0x53 'S' */ {0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00},
    /* 0x54 'T' */ {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
    /* 0x55 'U' */ {0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00},
    /* 0x56 'V' */ {0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00},
    /* 0x57 'W' */ {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00},
    /* 0x58 'X' */ {0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00},
    /* 0x59 'Y' */ {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00},
    /* 0x5A 'Z' */ {0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00},
};

/* =========================================================================
 * HUD rendering helpers
 * =========================================================================*/

/* Write one ARGB8888 pixel into the mapped buffer */
static inline void put_pixel(drm_ctx_t *ctx, int x, int y, uint32_t color)
{
    if (x < 0 || x >= HUD_WIDTH || y < 0 || y >= HUD_HEIGHT)
        return;
    uint32_t *px = (uint32_t *)((uint8_t *)ctx->map + y * ctx->pitch + x * 4);
    *px = color;
}

/* Draw one 8×8 character at pixel position (px, py) with given color */
static void draw_char(drm_ctx_t *ctx, int px, int py,
                      char c, uint32_t color)
{
    if (c < ' ' || c > 'Z')
        c = '?';
    const uint8_t *glyph = font8x8[(uint8_t)(c - ' ')];
    for (int row = 0; row < 8; row++) {
        for (int col = 0; col < 8; col++) {
            if (glyph[row] & (0x80 >> col))
                put_pixel(ctx, px + col, py + row, color);
        }
    }
}

/* Draw a string at (px, py), spacing = 9 pixels per character */
static void draw_string(drm_ctx_t *ctx, int px, int py,
                        const char *str, uint32_t color)
{
    for (; *str; str++, px += 9) {
        /* Convert lowercase to uppercase for our font */
        char c = (*str >= 'a' && *str <= 'z') ? (*str - 32) : *str;
        draw_char(ctx, px, py, c, color);
    }
}

/* Fill a rectangle with a solid color */
static void fill_rect(drm_ctx_t *ctx, int x, int y, int w, int h, uint32_t color)
{
    for (int row = y; row < y + h; row++)
        for (int col = x; col < x + w; col++)
            put_pixel(ctx, col, row, color);
}

/* =========================================================================
 * drm_telemetry_init
 * =========================================================================*/
int drm_telemetry_init(drm_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->drm_fd = -1;

    /* 1. Open DRM device */
    ctx->drm_fd = open(DRM_DEVICE_PATH, O_RDWR | O_CLOEXEC);
    if (ctx->drm_fd < 0) {
        fprintf(stderr, "[DRM] Cannot open %s: %s\n",
                DRM_DEVICE_PATH, strerror(errno));
        return -1;
    }

    /* 2. Request dumb buffer capability */
    uint64_t has_dumb = 0;
    if (drmGetCap(ctx->drm_fd, DRM_CAP_DUMB_BUFFER, &has_dumb) || !has_dumb) {
        fprintf(stderr, "[DRM] Dumb buffers not supported\n");
        close(ctx->drm_fd);
        return -1;
    }

    /* 3. Create the dumb buffer (CPU-accessible ARGB8888) */
    struct drm_mode_create_dumb create_req = {
        .width  = HUD_WIDTH,
        .height = HUD_HEIGHT,
        .bpp    = HUD_BPP,
    };
    if (ioctl(ctx->drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &create_req)) {
        fprintf(stderr, "[DRM] CREATE_DUMB failed: %s\n", strerror(errno));
        close(ctx->drm_fd);
        return -1;
    }
    ctx->gem_handle = create_req.handle;
    ctx->pitch      = create_req.pitch;
    ctx->size       = create_req.size;

    /* 4. Create DRM framebuffer from the dumb buffer */
    struct drm_mode_fb_cmd fb_cmd = {
        .width   = HUD_WIDTH,
        .height  = HUD_HEIGHT,
        .pitch   = ctx->pitch,
        .bpp     = HUD_BPP,
        .depth   = 32,
        .handle  = ctx->gem_handle,
    };
    if (ioctl(ctx->drm_fd, DRM_IOCTL_MODE_ADDFB, &fb_cmd)) {
        fprintf(stderr, "[DRM] ADDFB failed: %s\n", strerror(errno));
        goto err_destroy_dumb;
    }
    ctx->fb_id = fb_cmd.fb_id;

    /* 5. Map the dumb buffer into userspace for CPU writes */
    struct drm_mode_map_dumb map_req = { .handle = ctx->gem_handle };
    if (ioctl(ctx->drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &map_req)) {
        fprintf(stderr, "[DRM] MAP_DUMB failed: %s\n", strerror(errno));
        goto err_rm_fb;
    }
    ctx->map = mmap(NULL, ctx->size, PROT_READ | PROT_WRITE,
                    MAP_SHARED, ctx->drm_fd, map_req.offset);
    if (ctx->map == MAP_FAILED) {
        fprintf(stderr, "[DRM] mmap failed: %s\n", strerror(errno));
        goto err_rm_fb;
    }

    fprintf(stderr,
            "[DRM] Init OK: fd=%d gem=%u fb=%u pitch=%u size=%u "
            "map=%p\n",
            ctx->drm_fd, ctx->gem_handle, ctx->fb_id,
            ctx->pitch, ctx->size, ctx->map);
    return 0;

err_rm_fb:
    ioctl(ctx->drm_fd, DRM_IOCTL_MODE_RMFB, &ctx->fb_id);
err_destroy_dumb: {
    struct drm_mode_destroy_dumb dd = { .handle = ctx->gem_handle };
    ioctl(ctx->drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    }
    close(ctx->drm_fd);
    return -1;
}

/* =========================================================================
 * drm_telemetry_update  — render + flip (Issues 6 & 11)
 * =========================================================================*/
int drm_telemetry_update(drm_ctx_t *ctx, const imu_data_t *imu)
{
    char line[64];

    /* --- Step 1: Render HUD into the dumb buffer --- */

    /* Clear to semi-transparent dark background */
    fill_rect(ctx, 0, 0, HUD_WIDTH, HUD_HEIGHT, COLOR_DARK_BG);

    /* Title bar */
    fill_rect(ctx, 0, 0, HUD_WIDTH, 12, 0xFF202020u);
    draw_string(ctx, 4, 2, "REACTIVE CAMERA TELEMETRY HUD", COLOR_WHITE);

    /* Separator line */
    fill_rect(ctx, 0, 12, HUD_WIDTH, 1, COLOR_WHITE);

    /* Pitch */
    snprintf(line, sizeof(line), "PITCH: %+5d DEG",
             (int)(imu->pitch_mdeg / 1000));
    draw_string(ctx, 4, 16, line, COLOR_GREEN);

    /* Roll */
    snprintf(line, sizeof(line), "ROLL:  %+5d DEG",
             (int)(imu->roll_mdeg / 1000));
    draw_string(ctx, 4, 28, line, COLOR_GREEN);

    /* Acceleration */
    snprintf(line, sizeof(line), "AX:%+5d AY:%+5d AZ:%+5d MG",
             imu->accel_x_mg, imu->accel_y_mg, imu->accel_z_mg);
    draw_string(ctx, 4, 40, line, 0xFFAAFFAAu);

    /* Sample rate indicator */
    snprintf(line, sizeof(line), "SAMPLES: %lu",
             (unsigned long)imu->sample_count);
    draw_string(ctx, 4, 52, line, 0xFF888888u);

    /* Tilt alert banner (Issue 11 visual feedback) */
    if (imu->tilt_alert) {
        fill_rect(ctx, 0, 64, HUD_WIDTH, 16, COLOR_RED);
        draw_string(ctx, 4, 66, "!!! TILT ALERT > 45 DEG - V4L2 TRIGGER FIRED !!!", COLOR_WHITE);
    } else {
        fill_rect(ctx, 0, 64, HUD_WIDTH, 16, 0xFF003300u);
        draw_string(ctx, 4, 66, "TILT OK  < 45 DEG", COLOR_GREEN);
    }

    /* --- Step 2: Build the IOCTL argument struct --- */
    struct drm_vc4_telemetry_overlay ov = {
        .accel_x_mg = imu->accel_x_mg,
        .accel_y_mg = imu->accel_y_mg,
        .accel_z_mg = imu->accel_z_mg,
        .pitch_mdeg = imu->pitch_mdeg,
        .roll_mdeg  = imu->roll_mdeg,
        .fb_handle  = ctx->fb_id,    /* NOTE: pass fb_id, not gem_handle */
        .x_pos      = HUD_X_POS,
        .y_pos      = HUD_Y_POS,
        .width      = HUD_WIDTH,
        .height     = HUD_HEIGHT,
        .flags      = 0,
    };
    snprintf(ov.text, VC4_TELEMETRY_TEXT_LEN,
             "P:%+d R:%+d TILT:%d",
             (int)(imu->pitch_mdeg / 1000),
             (int)(imu->roll_mdeg  / 1000),
             imu->tilt_alert);

    /* --- Step 3: Issue the kernel IOCTL --- */
    if (ioctl(ctx->drm_fd, DRM_IOCTL_VC4_TELEMETRY_OVERLAY, &ov) < 0) {
        fprintf(stderr, "[DRM] TELEMETRY_OVERLAY ioctl failed: %s\n",
                strerror(errno));
        return -1;
    }

    return 0;
}

/* =========================================================================
 * drm_telemetry_cleanup
 * =========================================================================*/
void drm_telemetry_cleanup(drm_ctx_t *ctx)
{
    if (ctx->map && ctx->map != MAP_FAILED)
        munmap(ctx->map, ctx->size);

    if (ctx->drm_fd >= 0) {
        if (ctx->fb_id) {
            ioctl(ctx->drm_fd, DRM_IOCTL_MODE_RMFB, &ctx->fb_id);
        }
        if (ctx->gem_handle) {
            struct drm_mode_destroy_dumb dd = { .handle = ctx->gem_handle };
            ioctl(ctx->drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
        }
        close(ctx->drm_fd);
    }
    memset(ctx, 0, sizeof(*ctx));
}
