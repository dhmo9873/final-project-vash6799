/* main.c
 *
 * Issues 7, 11: Multi-Threaded C Orchestrator — Full Reactive Integration
 *
 * This is the top-level application. It:
 *   1. Spawns the IIO polling thread (iio_poll_thread_fn) at 50 Hz
 *   2. Initializes the DRM telemetry overlay (drm_telemetry_init)
 *   3. Opens the V4L2 subdevice (v4l2_ctrl_init)
 *   4. Runs the main event loop at ~30 Hz:
 *        a. Copies the shared IMU data under mutex (zero-copy-safe)
 *        b. Updates the DRM HUD framebuffer and issues the IOCTL
 *        c. If tilt_alert transitions 0→1: fires V4L2 HFLIP control
 *        d. If tilt_alert transitions 1→0: restores V4L2 normal mode
 *   5. On SIGINT/SIGTERM: sets running=0, joins the IIO thread, cleans up
 *
 * ISSUE 7 DoD VERIFICATION:
 *   Compile with: gcc -pthread -o orchestrator main.c iio_poll.c
 *                     drm_telemetry.c v4l2_ctrl.c -ldrm -lm
 *   Without any hardware, the IIO thread will print "Cannot open .../iio"
 *   errors but will keep running. The mutex usage is visible in the log.
 *
 * ISSUE 11 DoD VERIFICATION:
 *   With all hardware connected and kernel patches applied:
 *   - Tilt the IMU > 45°
 *   - Observe: HUD shows red "TILT ALERT" banner on HDMI display
 *   - Observe: Camera feed shows color bar test pattern (V4L2 trigger)
 *   - Restore < 45°: camera returns to normal, HUD turns green
 *
 * THREAD ARCHITECTURE:
 *
 *   ┌─────────────────────────┐     mutex-protected     ┌───────────────────┐
 *   │   iio_poll_thread       │ ──── imu_data_t ──────> │   Main Loop       │
 *   │   50 Hz, sysfs reads    │                         │   30 Hz updates   │
 *   │   pitch/roll compute    │                         │   DRM IOCTL       │
 *   └─────────────────────────┘                         │   V4L2 CID        │
 *                                                        └───────────────────┘
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <unistd.h>
#include <time.h>

#include "iio_poll.h"
#include "drm_telemetry.h"
#include "v4l2_ctrl.h"

/* -------------------------------------------------------------------------
 * Global state
 * -------------------------------------------------------------------------*/
static volatile sig_atomic_t  g_running = 1;   /* Set to 0 on SIGINT/SIGTERM */

static imu_data_t             g_imu;            /* Shared IMU data struct     */
static pthread_mutex_t        g_imu_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Main loop rate: 30 Hz = 33 333 µs per iteration */
#define MAIN_LOOP_INTERVAL_US  33333

/* -------------------------------------------------------------------------
 * Signal handler
 * -------------------------------------------------------------------------*/
static void sig_handler(int signo)
{
    (void)signo;
    g_running = 0;
    fprintf(stderr, "\n[MAIN] Signal received — shutting down...\n");
}

/* -------------------------------------------------------------------------
 * main
 * -------------------------------------------------------------------------*/
int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    fprintf(stderr,
            "=== Reactive Camera Orchestrator ===\n"
            "  Issues 7 (skeleton) + 11 (integration)\n"
            "  Press Ctrl+C to stop.\n\n");

    /* ------------------------------------------------------------------
     * Step 1: Signal handling
     * ------------------------------------------------------------------*/
    struct sigaction sa = { .sa_handler = sig_handler };
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    /* ------------------------------------------------------------------
     * Step 2: Initialize shared data
     * ------------------------------------------------------------------*/
    memset(&g_imu, 0, sizeof(g_imu));

    /* ------------------------------------------------------------------
     * Step 3: Spawn IIO polling thread (Issue 7 & 8)
     * ------------------------------------------------------------------*/
    iio_thread_args_t iio_args = {
        .shared_data = &g_imu,
        .mutex       = &g_imu_mutex,
        .running     = 1,
    };

    pthread_t iio_tid;
    int ret = pthread_create(&iio_tid, NULL, iio_poll_thread_fn, &iio_args);
    if (ret != 0) {
        fprintf(stderr, "[MAIN] pthread_create failed: %s\n", strerror(ret));
        return EXIT_FAILURE;
    }
    fprintf(stderr, "[MAIN] IIO polling thread spawned (tid=%lu)\n",
            (unsigned long)iio_tid);

    /* ------------------------------------------------------------------
     * Step 4: Initialize DRM overlay (Issue 6)
     * ------------------------------------------------------------------*/
    drm_ctx_t drm_ctx;
    int drm_ok = (drm_telemetry_init(&drm_ctx) == 0);
    if (!drm_ok)
        fprintf(stderr, "[MAIN] DRM init failed — HUD overlay disabled\n");

    /* ------------------------------------------------------------------
     * Step 5: Initialize V4L2 custom control (Issue 10)
     * ------------------------------------------------------------------*/
    v4l2_ctx_t v4l2_ctx;
    int v4l2_ok = (v4l2_ctrl_init(&v4l2_ctx) == 0);
    if (!v4l2_ok)
        fprintf(stderr, "[MAIN] V4L2 init failed — reactive CID disabled\n");

    /* ------------------------------------------------------------------
     * Step 6: Main reactive event loop (Issue 11)
     *
     * Runs at MAIN_LOOP_INTERVAL_US (≈30 Hz).
     * Each iteration:
     *   a. Copy the shared IMU snapshot under mutex
     *   b. Update DRM HUD
     *   c. React to tilt transitions
     * ------------------------------------------------------------------*/
    imu_data_t imu_snapshot;
    int prev_tilt_alert = 0;

    struct timespec ts_sleep = {
        .tv_sec  = 0,
        .tv_nsec = MAIN_LOOP_INTERVAL_US * 1000L,
    };

    fprintf(stderr, "[MAIN] Entering reactive event loop at ~30 Hz...\n");

    while (g_running) {

        /* a. Atomic copy of shared IMU data — minimal critical section */
        pthread_mutex_lock(&g_imu_mutex);
        memcpy(&imu_snapshot, &g_imu, sizeof(imu_snapshot));
        pthread_mutex_unlock(&g_imu_mutex);

        /* b. Update DRM HUD overlay with current telemetry */
        if (drm_ok) {
            if (drm_telemetry_update(&drm_ctx, &imu_snapshot) < 0) {
                fprintf(stderr, "[MAIN] DRM update error\n");
                /* Non-fatal — keep looping */
            }
        }

        /* c. React to tilt_alert state TRANSITIONS only (edge-triggered)
         *    This avoids hammering the I2C bus with repeat writes.       */
        if (imu_snapshot.tilt_alert && !prev_tilt_alert) {
            /* Tilt just crossed 45° → fire reactive responses */
            fprintf(stderr,
                "[MAIN] *** TILT ALERT: pitch=%d mdeg (>%d mdeg) ***\n",
                imu_snapshot.pitch_mdeg, TILT_THRESHOLD_MDEG);

            if (v4l2_ok) {
                /* Issue 11 DoD: immediately alter camera feed hardware */
                v4l2_ctrl_set_reactive(&v4l2_ctx,
                                       REACTIVE_CTRL_TEST_PATTERN);
                fprintf(stderr,
                    "[MAIN] V4L2: reactive_ctrl → TEST_PATTERN (2)\n");
            }

        } else if (!imu_snapshot.tilt_alert && prev_tilt_alert) {
            /* Tilt recovered below 45° → restore normal operation */
            fprintf(stderr, "[MAIN] Tilt recovered — restoring normal mode\n");

            if (v4l2_ok) {
                v4l2_ctrl_set_reactive(&v4l2_ctx, REACTIVE_CTRL_NORMAL);
                fprintf(stderr,
                    "[MAIN] V4L2: reactive_ctrl → NORMAL (0)\n");
            }
        }

        prev_tilt_alert = imu_snapshot.tilt_alert;

        /* d. Print a periodic status line to terminal (visible on SSH) */
        static uint32_t loop_count = 0;
        if (++loop_count % 90 == 0) {  /* Every ~3 seconds at 30 Hz */
            fprintf(stderr,
                "[MAIN] loop=%u  pitch=%+d roll=%+d  tilt=%d  "
                "samples=%lu\n",
                loop_count,
                (int)(imu_snapshot.pitch_mdeg / 1000),
                (int)(imu_snapshot.roll_mdeg  / 1000),
                imu_snapshot.tilt_alert,
                (unsigned long)imu_snapshot.sample_count);
        }

        nanosleep(&ts_sleep, NULL);
    }

    /* ------------------------------------------------------------------
     * Step 7: Shutdown — signal IIO thread and join
     * ------------------------------------------------------------------*/
    fprintf(stderr, "[MAIN] Stopping IIO thread...\n");
    iio_args.running = 0;
    pthread_join(iio_tid, NULL);
    fprintf(stderr, "[MAIN] IIO thread joined.\n");

    /* Restore normal V4L2 state and close FDs */
    if (v4l2_ok)
        v4l2_ctrl_cleanup(&v4l2_ctx);

    /* Release DRM framebuffer and unmap */
    if (drm_ok)
        drm_telemetry_cleanup(&drm_ctx);

    pthread_mutex_destroy(&g_imu_mutex);

    fprintf(stderr,
            "[MAIN] Shutdown complete. Total IIO samples: %lu\n",
            (unsigned long)imu_snapshot.sample_count);
    return EXIT_SUCCESS;
}
