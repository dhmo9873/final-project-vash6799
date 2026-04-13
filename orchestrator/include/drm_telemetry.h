/* drm_telemetry.h
 *
 * Issues 5 & 6: DRM Overlay Plane — Userspace Interface
 *
 * Provides the user-space side of the DRM telemetry overlay:
 *   - Opens the DRM device and finds the vc4 CRTC + connector
 *   - Allocates a DRM dumb buffer (ARGB8888) for the HUD framebuffer
 *   - Maps the buffer into userspace for CPU rendering (text/indicators)
 *   - Creates a DRM framebuffer object from the dumb buffer
 *   - Issues DRM_IOCTL_VC4_TELEMETRY_OVERLAY to display the HUD
 *   - Renders a simple 8×8 bitmap font for HUD text (no external libs)
 *
 * Why a dumb buffer instead of GBM/EGL?
 *   For this project we want zero external dependencies beyond libdrm.
 *   A DRM dumb buffer is CPU-accessible shared memory between userspace
 *   and the display controller — perfect for a simple text HUD.
 */

#ifndef DRM_TELEMETRY_H
#define DRM_TELEMETRY_H

#include <stdint.h>
#include "telemetry_types.h"
#include "iio_poll.h"

/* -------------------------------------------------------------------------
 * HUD framebuffer dimensions (pixels)
 * -------------------------------------------------------------------------
 * The HUD is 400×120 pixels, positioned at top-left of the display (0,0).
 * ARGB8888: 4 bytes per pixel → 400*120*4 = 192 000 bytes.
 * Adjust HUD_X_POS / HUD_Y_POS to reposition on screen.              */
#define HUD_WIDTH      400
#define HUD_HEIGHT     120
#define HUD_BPP        32         /* bits per pixel (ARGB8888)          */
#define HUD_X_POS      0
#define HUD_Y_POS      0

/* DRM device node */
#define DRM_DEVICE_PATH   "/dev/dri/card0"

/* ARGB8888 colour constants for the HUD */
#define COLOR_TRANSPARENT    0x00000000u
#define COLOR_WHITE          0xFFFFFFFFu
#define COLOR_GREEN          0xFF00FF00u
#define COLOR_RED            0xFFFF0000u
#define COLOR_YELLOW         0xFFFFFF00u
#define COLOR_DARK_BG        0xCC101010u  /* Semi-transparent dark background */

/* -------------------------------------------------------------------------
 * DRM context struct — holds all handles for one session
 * -------------------------------------------------------------------------*/
typedef struct {
    int          drm_fd;        /* File descriptor to /dev/dri/card0     */
    uint32_t     gem_handle;    /* GEM object handle for the dumb buffer  */
    uint32_t     fb_id;         /* DRM framebuffer ID (passed to IOCTL)   */
    uint32_t     pitch;         /* Row stride in bytes                    */
    uint32_t     size;          /* Total buffer size in bytes             */
    void        *map;           /* mmap pointer to the pixel data         */
} drm_ctx_t;

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * drm_telemetry_init - Open DRM device and allocate HUD framebuffer.
 *
 * Must be called once before drm_telemetry_update().
 * @ctx : output; filled with DRM handles and mmap pointer
 * Returns 0 on success, -1 on error (error message printed to stderr).
 */
int drm_telemetry_init(drm_ctx_t *ctx);

/**
 * drm_telemetry_update - Render telemetry text into the HUD and flip plane.
 *
 * Renders the current IMU values and tilt alert into the dumb buffer,
 * then calls DRM_IOCTL_VC4_TELEMETRY_OVERLAY to make the HUD visible.
 *
 * @ctx  : initialized DRM context from drm_telemetry_init()
 * @imu  : current IMU snapshot (caller holds lock or passes a copy)
 * Returns 0 on success, -1 on IOCTL error.
 */
int drm_telemetry_update(drm_ctx_t *ctx, const imu_data_t *imu);

/**
 * drm_telemetry_cleanup - Unmap and release all DRM resources.
 * @ctx : context to clean up
 */
void drm_telemetry_cleanup(drm_ctx_t *ctx);

#endif /* DRM_TELEMETRY_H */
