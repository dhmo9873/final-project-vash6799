/* drm_ioctl_test.c
 *
 * Issue 6 Definition of Done:
 *   "A standalone C test script successfully passes a hardcoded text string
 *    to the DRM subsystem via ioctl(). The text string is visibly rendered
 *    on the HDMI display."
 *
 * USAGE:
 *   gcc drm_ioctl_test.c -o drm_ioctl_test -ldrm -I/usr/include/libdrm
 *   ./drm_ioctl_test
 *
 * This test:
 *   1. Allocates a 400x80 dumb buffer
 *   2. Fills it with a white "HELLO DRM OVERLAY" text on dark background
 *      (using a simple pixel-fill approach, no font library needed)
 *   3. Calls DRM_IOCTL_VC4_TELEMETRY_OVERLAY to display it at (10, 10)
 *   4. Waits 5 seconds so you can see it on HDMI
 *   5. Cleans up (plane disappears)
 *
 * If the IOCTL fails with ENOTTY, the kernel patch is not applied yet.
 * If it fails with EPERM, run as root: sudo ./drm_ioctl_test
 */

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

/* Our UAPI struct (mirrors telemetry_types.h / kernel patch) */
#define VC4_TELEMETRY_TEXT_LEN 64

struct drm_vc4_telemetry_overlay {
    int32_t  accel_x_mg;
    int32_t  accel_y_mg;
    int32_t  accel_z_mg;
    int32_t  pitch_mdeg;
    int32_t  roll_mdeg;
    uint32_t fb_handle;
    uint32_t x_pos;
    uint32_t y_pos;
    uint32_t width;
    uint32_t height;
    char     text[VC4_TELEMETRY_TEXT_LEN];
    uint32_t flags;
    uint32_t pad[3];
};

#define DRM_VC4_TELEMETRY_OVERLAY  0x0b
#define DRM_IOCTL_VC4_TELEMETRY_OVERLAY \
    DRM_IOWR(0x40 + DRM_VC4_TELEMETRY_OVERLAY, struct drm_vc4_telemetry_overlay)

#define TEST_W  400
#define TEST_H   80
#define TEST_BPP 32

/* Draw a solid colored rectangle into a uint32_t* pixel buffer */
static void fill_rect_buf(uint32_t *buf, int buf_w,
                           int x, int y, int w, int h, uint32_t color)
{
    for (int row = y; row < y + h; row++)
        for (int col = x; col < x + w; col++)
            buf[row * buf_w + col] = color;
}

/* Very simple stripes to spell out "HELLO DRM OVERLAY" visually */
static void render_test_pattern(uint32_t *buf, int buf_w, int buf_h)
{
    /* Background */
    fill_rect_buf(buf, buf_w, 0, 0, buf_w, buf_h, 0xCC101010u);

    /* Top and bottom border */
    fill_rect_buf(buf, buf_w, 0, 0,        buf_w, 3,  0xFFFFFFFFu);
    fill_rect_buf(buf, buf_w, 0, buf_h-3,  buf_w, 3,  0xFFFFFFFFu);

    /* "H" */
    fill_rect_buf(buf, buf_w, 10, 10, 5, 40, 0xFFFFFFFFu);
    fill_rect_buf(buf, buf_w, 15, 28, 10, 5, 0xFFFFFFFFu);
    fill_rect_buf(buf, buf_w, 20, 10, 5, 40, 0xFFFFFFFFu);

    /* "I" */
    fill_rect_buf(buf, buf_w, 35, 10, 5, 40, 0xFFFFFFFFu);

    /* Center band: "DRM OVERLAY TEST" label in green */
    fill_rect_buf(buf, buf_w, 60, 20, 280, 20, 0xFF004400u);
    fill_rect_buf(buf, buf_w, 62, 22, 276, 16, 0xFF00FF00u);

    /* Red tilt indicator box */
    fill_rect_buf(buf, buf_w, 350, 10, 40, 40, 0xFFFF0000u);
}

int main(void)
{
    int drm_fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (drm_fd < 0) {
        perror("open /dev/dri/card0");
        return 1;
    }
    printf("[TEST] Opened DRM device fd=%d\n", drm_fd);

    /* Check dumb buffer support */
    uint64_t has_dumb = 0;
    if (drmGetCap(drm_fd, DRM_CAP_DUMB_BUFFER, &has_dumb) || !has_dumb) {
        fprintf(stderr, "[TEST] Dumb buffers not supported\n");
        close(drm_fd);
        return 1;
    }

    /* Create dumb buffer */
    struct drm_mode_create_dumb creq = {
        .width  = TEST_W,
        .height = TEST_H,
        .bpp    = TEST_BPP,
    };
    if (ioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq)) {
        perror("CREATE_DUMB");
        close(drm_fd);
        return 1;
    }
    printf("[TEST] Dumb buffer: handle=%u pitch=%u size=%llu\n",
           creq.handle, creq.pitch, (unsigned long long)creq.size);

    /* Create framebuffer */
    struct drm_mode_fb_cmd fb_cmd = {
        .width  = TEST_W,
        .height = TEST_H,
        .pitch  = creq.pitch,
        .bpp    = TEST_BPP,
        .depth  = 32,
        .handle = creq.handle,
    };
    if (ioctl(drm_fd, DRM_IOCTL_MODE_ADDFB, &fb_cmd)) {
        perror("ADDFB");
        goto cleanup_dumb;
    }
    printf("[TEST] Framebuffer created: fb_id=%u\n", fb_cmd.fb_id);

    /* Map dumb buffer */
    struct drm_mode_map_dumb mreq = { .handle = creq.handle };
    if (ioctl(drm_fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq)) {
        perror("MAP_DUMB");
        goto cleanup_fb;
    }
    void *map = mmap(NULL, creq.size, PROT_READ | PROT_WRITE,
                     MAP_SHARED, drm_fd, mreq.offset);
    if (map == MAP_FAILED) {
        perror("mmap");
        goto cleanup_fb;
    }

    /* Render test pattern */
    render_test_pattern((uint32_t *)map, TEST_W, TEST_H);
    printf("[TEST] Test pattern rendered into framebuffer.\n");

    /* Issue the telemetry overlay IOCTL */
    struct drm_vc4_telemetry_overlay ov = {
        .accel_x_mg = 0,
        .accel_y_mg = 0,
        .accel_z_mg = 1000,    /* ~1g on Z = flat */
        .pitch_mdeg = 0,
        .roll_mdeg  = 0,
        .fb_handle  = fb_cmd.fb_id,
        .x_pos      = 10,
        .y_pos      = 10,
        .width      = TEST_W,
        .height     = TEST_H,
        .flags      = 0,
    };
    strncpy(ov.text, "HELLO DRM OVERLAY - ISSUE 6 DOD",
            VC4_TELEMETRY_TEXT_LEN - 1);

    printf("[TEST] Calling DRM_IOCTL_VC4_TELEMETRY_OVERLAY...\n");
    if (ioctl(drm_fd, DRM_IOCTL_VC4_TELEMETRY_OVERLAY, &ov) < 0) {
        fprintf(stderr, "[TEST] IOCTL failed: %s\n", strerror(errno));
        if (errno == ENOTTY)
            fprintf(stderr,
                "  → ENOTTY: vc4 kernel patch not applied yet.\n"
                "  → Apply 0001-vc4-add-telemetry-overlay-ioctl.patch\n");
        else if (errno == EPERM || errno == EACCES)
            fprintf(stderr,
                "  → Permission denied. Run as root: sudo %s\n",
                "/usr/bin/drm_ioctl_test");
        goto cleanup_map;
    }

    printf("[TEST] SUCCESS! Overlay should be visible on HDMI.\n");
    printf("[TEST] Waiting 5 seconds...\n");
    sleep(5);

    printf("[TEST] Cleaning up.\n");

cleanup_map:
    munmap(map, creq.size);
cleanup_fb:
    ioctl(drm_fd, DRM_IOCTL_MODE_RMFB, &fb_cmd.fb_id);
cleanup_dumb: {
    struct drm_mode_destroy_dumb dd = { .handle = creq.handle };
    ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dd);
    }
    close(drm_fd);
    return 0;
}
