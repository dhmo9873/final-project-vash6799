/* v4l2_ctrl.c
 *
 * Issues 10 & 11: V4L2 Custom CID — Userspace Implementation
 *
 * Sets IMX219_CID_REACTIVE_CTRL on /dev/v4l-subdev0 using the standard
 * VIDIOC_S_EXT_CTRLS ioctl. This is exactly what v4l2-ctl --set-ctrl does.
 *
 * If VIDIOC_S_EXT_CTRLS fails (e.g., kernel patch not applied), we fall
 * back to VIDIOC_S_CTRL on /dev/video0 for simpler test cases.
 */

#include "v4l2_ctrl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

int v4l2_ctrl_init(v4l2_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->subdev_fd = -1;
    ctx->video_fd  = -1;

    /* Open the subdevice for control writes */
    ctx->subdev_fd = open(V4L2_SUBDEV_NODE, O_RDWR | O_CLOEXEC);
    if (ctx->subdev_fd < 0) {
        fprintf(stderr, "[V4L2] Cannot open subdev %s: %s\n",
                V4L2_SUBDEV_NODE, strerror(errno));
        /* Non-fatal: VIDIOC_S_CTRL fallback via video node */
    }

    /* Also open the video node (used for streaming validation in Issue 4) */
    ctx->video_fd = open(V4L2_VIDEO_NODE, O_RDWR | O_CLOEXEC);
    if (ctx->video_fd < 0) {
        fprintf(stderr, "[V4L2] Cannot open video node %s: %s\n",
                V4L2_VIDEO_NODE, strerror(errno));
        if (ctx->subdev_fd < 0) {
            fprintf(stderr, "[V4L2] No V4L2 device available — aborting\n");
            return -1;
        }
    }

    ctx->current_ctrl = REACTIVE_CTRL_NORMAL;
    fprintf(stderr, "[V4L2] Init OK: subdev_fd=%d video_fd=%d\n",
            ctx->subdev_fd, ctx->video_fd);
    return 0;
}

int v4l2_ctrl_set_reactive(v4l2_ctx_t *ctx, int val)
{
    /* Skip redundant writes (Issue 11: avoid spamming I2C on every loop) */
    if (val == ctx->current_ctrl)
        return 0;

    /*
     * Method 1: VIDIOC_S_EXT_CTRLS on the subdevice.
     * This is the correct path when the imx219 patch (Issue 10) is applied.
     * The subdevice ioctl bypasses the V4L2 framework's permission checks
     * and writes directly to the driver's s_ctrl handler.
     */
    if (ctx->subdev_fd >= 0) {
        struct v4l2_ext_control ext_ctrl = {
            .id    = IMX219_CID_REACTIVE_CTRL,
            .value = val,
        };
        struct v4l2_ext_controls ext_ctrls = {
            .ctrl_class = V4L2_CTRL_CLASS_USER,
            .count      = 1,
            .controls   = &ext_ctrl,
        };

        if (ioctl(ctx->subdev_fd, VIDIOC_S_EXT_CTRLS, &ext_ctrls) == 0) {
            fprintf(stderr,
                    "[V4L2] reactive_ctrl set to %d via subdev VIDIOC_S_EXT_CTRLS\n",
                    val);
            ctx->current_ctrl = val;
            return 0;
        }
        fprintf(stderr, "[V4L2] VIDIOC_S_EXT_CTRLS on subdev failed: %s "
                "(is kernel patch applied?)\n", strerror(errno));
    }

    /*
     * Method 2: VIDIOC_S_CTRL on the video node (simpler fallback).
     * Works if the custom CID is also registered on the video device.
     */
    if (ctx->video_fd >= 0) {
        struct v4l2_control ctrl = {
            .id    = IMX219_CID_REACTIVE_CTRL,
            .value = val,
        };
        if (ioctl(ctx->video_fd, VIDIOC_S_CTRL, &ctrl) == 0) {
            fprintf(stderr,
                    "[V4L2] reactive_ctrl set to %d via video VIDIOC_S_CTRL\n",
                    val);
            ctx->current_ctrl = val;
            return 0;
        }
        fprintf(stderr, "[V4L2] VIDIOC_S_CTRL fallback also failed: %s\n",
                strerror(errno));
    }

    return -1;
}

void v4l2_ctrl_cleanup(v4l2_ctx_t *ctx)
{
    /* Restore normal mode before shutdown */
    if (ctx->current_ctrl != REACTIVE_CTRL_NORMAL)
        v4l2_ctrl_set_reactive(ctx, REACTIVE_CTRL_NORMAL);

    if (ctx->subdev_fd >= 0)
        close(ctx->subdev_fd);
    if (ctx->video_fd >= 0)
        close(ctx->video_fd);
    memset(ctx, 0, sizeof(*ctx));
}
