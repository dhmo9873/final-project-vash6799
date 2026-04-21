#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

/* ── IIO paths ────────────────────────────────────────────── */
#define ACCEL_DEV    "/sys/bus/iio/devices/iio:device1"
#define GYRO_DEV     "/sys/bus/iio/devices/iio:device0"
#define ACCEL_SCALE  0.000598205
#define GYRO_SCALE   0.000152716

/* ── IMX219 subdevice ────────────────────────────────────── */
#define SUBDEV       "/dev/v4l-subdev0"

/* ── IMX219 control IDs (from v4l2 -l output) ───────────── */
#define CID_EXPOSURE      0x00980911
#define CID_ANALOGUE_GAIN 0x009e0903
#define CID_DIGITAL_GAIN  0x009f0905
#define CID_TEST_PATTERN  0x009f0903

/* ── Exposure range ──────────────────────────────────────── */
#define EXPOSURE_MIN   4
#define EXPOSURE_MAX   3522
#define EXPOSURE_NOM   1600   /* nominal / flat */

/* ── Gain range ──────────────────────────────────────────── */
#define GAIN_MIN       0
#define GAIN_MAX       232
#define GAIN_NOM       0

/* ── Thresholds ──────────────────────────────────────────── */
#define TILT_DEG           45.0   /* degrees — triggers exposure reduction */
#define GYRO_FAST_RAD      1.0    /* rad/s magnitude — triggers gain boost  */
#define FREEFALL_G         1.0    /* m/s² total accel — triggers test pattern */

/* ── Poll rate ───────────────────────────────────────────── */
#define POLL_HZ   20
#define POLL_US   (1000000 / POLL_HZ)

static volatile int g_running = 1;
static void sig_handler(int s) { (void)s; g_running = 0; }

/* ── IIO read ────────────────────────────────────────────── */
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

/* ── V4L2 set control ────────────────────────────────────── */
static int set_ctrl(int fd, uint32_t id, int32_t val)
{
    struct v4l2_ext_control ctrl = {
        .id    = id,
        .value = val,
    };
    struct v4l2_ext_controls ctrls = {
        .which      = V4L2_CTRL_WHICH_CUR_VAL,
        .count      = 1,
        .controls   = &ctrl,
    };
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) < 0) {
        fprintf(stderr, "set_ctrl 0x%08x=%d: %s\n",
                id, val, strerror(errno));
        return -1;
    }
    return 0;
}

/* ── Clamp helper ────────────────────────────────────────── */
static int clamp(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* ── Map tilt angle to exposure ──────────────────────────── */
/*  flat (0°) → EXPOSURE_NOM                                  */
/*  at TILT_DEG or beyond → EXPOSURE_MIN                      */
static int tilt_to_exposure(double tilt_mag)
{
    if (tilt_mag >= TILT_DEG)
        return EXPOSURE_MIN;
    double frac = tilt_mag / TILT_DEG;     /* 0..1 */
    int exp = (int)(EXPOSURE_NOM * (1.0 - 0.8 * frac));
    return clamp(exp, EXPOSURE_MIN, EXPOSURE_MAX);
}

/* ── Map gyro magnitude to analogue gain ─────────────────── */
/*  slow (0 rad/s) → GAIN_NOM                                 */
/*  fast (>=GYRO_FAST_RAD) → GAIN_MAX                         */
static int gyro_to_gain(double gyro_mag)
{
    if (gyro_mag >= GYRO_FAST_RAD)
        return GAIN_MAX;
    double frac = gyro_mag / GYRO_FAST_RAD;
    int gain = (int)(GAIN_MAX * frac);
    return clamp(gain, GAIN_MIN, GAIN_MAX);
}

int main(void)
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    int fd = open(SUBDEV, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", SUBDEV, strerror(errno));
        return 1;
    }
    printf("V4L2 IMU controller started on %s at %d Hz\n",
           SUBDEV, POLL_HZ);
    printf("Controls: exposure=%d..%d  gain=%d..%d\n",
           EXPOSURE_MIN, EXPOSURE_MAX, GAIN_MIN, GAIN_MAX);
    printf("Tilt >%.0f deg → exposure reduction\n", TILT_DEG);
    printf("Gyro >%.1f rad/s → gain boost\n", GYRO_FAST_RAD);
    printf("Free-fall (<%.1f m/s² total) → test pattern\n\n",
           FREEFALL_G);

    int last_pattern = 0;
    int last_exposure = EXPOSURE_NOM;
    int last_gain     = GAIN_NOM;

    while (g_running) {
        long raw;
        double ax = 0, ay = 0, az = 0;
        double gx = 0, gy = 0, gz = 0;

        /* Read IMU */
        if (read_raw(ACCEL_DEV, "in_accel_x_raw", &raw) == 0) ax = raw * ACCEL_SCALE;
        if (read_raw(ACCEL_DEV, "in_accel_y_raw", &raw) == 0) ay = raw * ACCEL_SCALE;
        if (read_raw(ACCEL_DEV, "in_accel_z_raw", &raw) == 0) az = raw * ACCEL_SCALE;
        if (read_raw(GYRO_DEV,  "in_anglvel_x_raw", &raw) == 0) gx = raw * GYRO_SCALE;
        if (read_raw(GYRO_DEV,  "in_anglvel_y_raw", &raw) == 0) gy = raw * GYRO_SCALE;
        if (read_raw(GYRO_DEV,  "in_anglvel_z_raw", &raw) == 0) gz = raw * GYRO_SCALE;

        /* Derived values */
        double accel_mag = sqrt(ax*ax + ay*ay + az*az);
        double pitch     = atan2(-ax, sqrt(ay*ay + az*az)) * (180.0 / M_PI);
        double roll      = atan2( ay, az)                  * (180.0 / M_PI);
        double tilt_mag  = fmax(fabs(pitch), fabs(roll));
        double gyro_mag  = sqrt(gx*gx + gy*gy + gz*gz);

        /* ── Feature 3: Free-fall → test pattern ── */
        int want_pattern = (accel_mag < FREEFALL_G) ? 1 : 0;
        if (want_pattern != last_pattern) {
            set_ctrl(fd, CID_TEST_PATTERN, want_pattern);
            printf("[TEST_PATTERN] %s (accel_mag=%.2f m/s²)\n",
                   want_pattern ? "ON (colour bars)" : "OFF",
                   accel_mag);
            last_pattern = want_pattern;
        }

        /* ── Feature 1: Tilt → exposure reduction ── */
        int want_exposure = tilt_to_exposure(tilt_mag);
        if (abs(want_exposure - last_exposure) > 10) {
            set_ctrl(fd, CID_EXPOSURE, want_exposure);
            printf("[EXPOSURE] %d  (tilt=%.1f deg)\n",
                   want_exposure, tilt_mag);
            last_exposure = want_exposure;
        }

        /* ── Feature 2: Gyro → analogue gain boost ── */
        int want_gain = gyro_to_gain(gyro_mag);
        if (abs(want_gain - last_gain) > 5) {
            set_ctrl(fd, CID_ANALOGUE_GAIN, want_gain);
            printf("[GAIN] %d  (gyro_mag=%.3f rad/s)\n",
                   want_gain, gyro_mag);
            last_gain = want_gain;
        }

        printf("pitch=%+6.1f roll=%+6.1f tilt=%5.1f "
               "gyro=%6.3f accel=%.2f | "
               "exp=%-4d gain=%-3d pat=%d\n",
               pitch, roll, tilt_mag,
               gyro_mag, accel_mag,
               last_exposure, last_gain, last_pattern);

        usleep(POLL_US);
    }

    /* Restore defaults on exit */
    printf("\nRestoring camera defaults...\n");
    set_ctrl(fd, CID_TEST_PATTERN,  0);
    set_ctrl(fd, CID_EXPOSURE,      EXPOSURE_NOM);
    set_ctrl(fd, CID_ANALOGUE_GAIN, GAIN_NOM);
    close(fd);
    printf("Done.\n");
    return 0;
}
