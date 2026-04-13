/* v4l2_ctrl.h
 *
 * Issue 10 & 11: V4L2 Subdevice Custom Control — Userspace Interface
 *
 * Sets IMX219_CID_REACTIVE_CTRL (V4L2_CID_USER_CUSTOM_BASE + 0) on
 * the /dev/v4l-subdev0 device from userspace, mirroring what the
 * v4l2-ctl --set-ctrl command does at the command line.
 *
 * Used by the orchestrator's reactive logic:
 *   - When tilt_alert == 0: set reactive_ctrl = NORMAL (0)
 *   - When tilt_alert == 1: set reactive_ctrl = HFLIP  (1) or TEST_PATTERN (2)
 */

#ifndef V4L2_CTRL_H
#define V4L2_CTRL_H

#include <stdint.h>

/* V4L2 device nodes for IMX219 on RPi4:
 *   /dev/video0      — the video capture device (mmap streaming)
 *   /dev/v4l-subdev0 — the IMX219 subdevice (controls, format)
 * Our custom CID is on the subdevice.                              */
#define V4L2_VIDEO_NODE   "/dev/video0"
#define V4L2_SUBDEV_NODE  "/dev/v4l-subdev0"

/* Matches kernel IMX219_CID_REACTIVE_CTRL definition */
#define IMX219_CID_REACTIVE_CTRL   (0x00980000 + 0x1000)

/* Control values */
#define REACTIVE_CTRL_NORMAL        0
#define REACTIVE_CTRL_HFLIP         1
#define REACTIVE_CTRL_TEST_PATTERN  2

/* V4L2 context */
typedef struct {
    int  subdev_fd;    /* fd to /dev/v4l-subdev0 */
    int  video_fd;     /* fd to /dev/video0       */
    int  current_ctrl; /* last value sent         */
} v4l2_ctx_t;

/**
 * v4l2_ctrl_init - Open V4L2 subdevice and video nodes.
 * Returns 0 on success, -1 on error.
 */
int v4l2_ctrl_init(v4l2_ctx_t *ctx);

/**
 * v4l2_ctrl_set_reactive - Write IMX219_CID_REACTIVE_CTRL to the subdevice.
 *
 * @ctx : initialized context
 * @val : REACTIVE_CTRL_NORMAL / HFLIP / TEST_PATTERN
 * Returns 0 on success, -1 on ioctl error.
 */
int v4l2_ctrl_set_reactive(v4l2_ctx_t *ctx, int val);

/**
 * v4l2_ctrl_cleanup - Close file descriptors.
 */
void v4l2_ctrl_cleanup(v4l2_ctx_t *ctx);

#endif /* V4L2_CTRL_H */
