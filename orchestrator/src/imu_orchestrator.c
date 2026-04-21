#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <syslog.h>
#include <signal.h>
#include <errno.h>

/* ── IIO sysfs paths ─────────────────────────────────────── */
#define ACCEL_DEV   "/sys/bus/iio/devices/iio:device1"
#define GYRO_DEV    "/sys/bus/iio/devices/iio:device0"

#define ACCEL_SCALE  0.000598205   /* m/s² per count */
#define GYRO_SCALE   0.000152716   /* rad/s per count */

#define POLL_HZ          50
#define POLL_PERIOD_NS   (1000000000L / POLL_HZ)
#define TILT_ALERT_DEG   45.0

/* ── Shared IMU state ────────────────────────────────────── */
typedef struct {
    double ax, ay, az;       /* m/s² */
    double gx, gy, gz;       /* rad/s */
    double pitch_deg;
    double roll_deg;
    int    tilt_alert;
} imu_state_t;

static imu_state_t  g_imu;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_running = 1;

/* ── Helpers ─────────────────────────────────────────────── */
static int read_iio_raw(const char *dev, const char *channel, long *out)
{
    char path[128];
    snprintf(path, sizeof(path), "%s/%s", dev, channel);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int r = fscanf(f, "%ld", out);
    fclose(f);
    return (r == 1) ? 0 : -1;
}

static void timespec_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += 1;
        ts->tv_nsec -= 1000000000L;
    }
}

static void handle_signal(int sig)
{
    (void)sig;
    g_running = 0;
}

/* ── IIO poll thread (50 Hz) ────────────────────────────── */
static void *iio_poll_thread(void *arg)
{
    (void)arg;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (g_running) {
        long raw;
        double ax = 0, ay = 0, az = 0;
        double gx = 0, gy = 0, gz = 0;

        /* Read accelerometer */
        if (read_iio_raw(ACCEL_DEV, "in_accel_x_raw", &raw) == 0)
            ax = raw * ACCEL_SCALE;
        if (read_iio_raw(ACCEL_DEV, "in_accel_y_raw", &raw) == 0)
            ay = raw * ACCEL_SCALE;
        if (read_iio_raw(ACCEL_DEV, "in_accel_z_raw", &raw) == 0)
            az = raw * ACCEL_SCALE;

        /* Read gyroscope */
        if (read_iio_raw(GYRO_DEV, "in_anglvel_x_raw", &raw) == 0)
            gx = raw * GYRO_SCALE;
        if (read_iio_raw(GYRO_DEV, "in_anglvel_y_raw", &raw) == 0)
            gy = raw * GYRO_SCALE;
        if (read_iio_raw(GYRO_DEV, "in_anglvel_z_raw", &raw) == 0)
            gz = raw * GYRO_SCALE;

        /* Compute pitch/roll from accel */
        double pitch = atan2(-ax, sqrt(ay*ay + az*az)) * (180.0 / M_PI);
        double roll  = atan2( ay, az)                  * (180.0 / M_PI);
        int    alert = (fabs(pitch) > TILT_ALERT_DEG ||
                        fabs(roll)  > TILT_ALERT_DEG);

        pthread_mutex_lock(&g_mutex);
        g_imu.ax = ax; g_imu.ay = ay; g_imu.az = az;
        g_imu.gx = gx; g_imu.gy = gy; g_imu.gz = gz;
        g_imu.pitch_deg  = pitch;
        g_imu.roll_deg   = roll;
        g_imu.tilt_alert = alert;
        pthread_mutex_unlock(&g_mutex);

        /* Absolute-time sleep for accurate 50 Hz */
        timespec_add_ns(&next, POLL_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    return NULL;
}

/* ── Report thread (1 Hz) ───────────────────────────────── */
static void *report_thread(void *arg)
{
    (void)arg;
    while (g_running) {
        imu_state_t s;
        pthread_mutex_lock(&g_mutex);
        s = g_imu;
        pthread_mutex_unlock(&g_mutex);

        /* stdout */
        printf("[IMU] pitch=%.2f° roll=%.2f° | "
               "a=(%.3f,%.3f,%.3f) g=(%.4f,%.4f,%.4f) | "
               "TILT=%s\n",
               s.pitch_deg, s.roll_deg,
               s.ax, s.ay, s.az,
               s.gx, s.gy, s.gz,
               s.tilt_alert ? "ALERT" : "ok");
        fflush(stdout);

        /* syslog — only on alert state change */
        static int last_alert = 0;
        if (s.tilt_alert && !last_alert) {
            syslog(LOG_WARNING,
                   "TILT ALERT: pitch=%.2f roll=%.2f",
                   s.pitch_deg, s.roll_deg);
        } else if (!s.tilt_alert && last_alert) {
            syslog(LOG_INFO, "Tilt cleared: pitch=%.2f roll=%.2f",
                   s.pitch_deg, s.roll_deg);
        }
        last_alert = s.tilt_alert;

        sleep(1);
    }
    return NULL;
}

/* ── main ────────────────────────────────────────────────── */
int main(void)
{
    signal(SIGINT,  handle_signal);
    signal(SIGTERM, handle_signal);

    openlog("imu_orchestrator", LOG_PID | LOG_CONS, LOG_USER);
    syslog(LOG_INFO, "Starting IMU orchestrator at %d Hz", POLL_HZ);

    pthread_t t_poll, t_report;
    pthread_create(&t_poll,   NULL, iio_poll_thread, NULL);
    pthread_create(&t_report, NULL, report_thread,   NULL);

    pthread_join(t_poll,   NULL);
    pthread_join(t_report, NULL);

    syslog(LOG_INFO, "IMU orchestrator stopped");
    closelog();
    return 0;
}
