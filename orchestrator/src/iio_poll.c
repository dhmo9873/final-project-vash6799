/* iio_poll.c
 *
 * Issue 8: IIO Polling & Struct Packing
 *
 * Reads ICM-20948 accelerometer and gyroscope raw values from the Linux
 * IIO sysfs interface at 50 Hz and packs them into the shared imu_data_t
 * struct. Computes pitch and roll from accelerometer using integer math.
 *
 * MATH (integer-only, no libm):
 *   The standard pitch/roll from accelerometer:
 *     pitch = atan2(ax, sqrt(ay² + az²))
 *     roll  = atan2(ay, sqrt(ax² + az²))
 *
 *   We use a lookup table approximation to avoid floating point.
 *   atan2_int(y, x) returns milli-degrees using a 91-entry table.
 *   Accuracy: ±1 degree, sufficient for a 45° threshold trigger.
 *
 * ISSUE 11 INTEGRATION:
 *   Sets imu_data->tilt_alert = 1 when |pitch_mdeg| > TILT_THRESHOLD_MDEG.
 *   The main orchestrator loop reads tilt_alert under mutex to decide
 *   whether to fire the DRM IOCTL and V4L2 CID.
 */

#include "iio_poll.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>       /* Only for sqrtl() in the atan2 approx helper */
#include <stdint.h>
#include <pthread.h>
#include <time.h>

/* -------------------------------------------------------------------------
 * Integer square root (Newton-Raphson, no libm needed)
 * Returns floor(sqrt(n)) for n >= 0.
 * -------------------------------------------------------------------------*/
static int32_t isqrt32(int32_t n)
{
    if (n <= 0)
        return 0;
    int32_t x = n;
    int32_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

/* -------------------------------------------------------------------------
 * Integer atan2 approximation.
 *
 * Returns angle in milli-degrees in the range [-180000, 180000].
 * Uses the minimax polynomial approximation:
 *   atan(z) ≈ z * (180000/PI) / (1 + 0.28125 * z²)  for |z| <= 1
 * with quadrant correction for |z| > 1.
 *
 * Accuracy: <1.5 degree error, acceptable for 45° tilt detection.
 *
 * @y, @x : both in the same integer units (e.g., milli-g)
 * Returns angle in milli-degrees.
 * -------------------------------------------------------------------------*/
static int32_t atan2_mdeg(int32_t y, int32_t x)
{
    /* 180000 / PI ≈ 57296 milli-degrees per radian */
    const int32_t MDEG_PER_RAD = 57296;

    if (x == 0 && y == 0)
        return 0;

    int32_t abs_y = (y < 0) ? -y : y;
    int32_t abs_x = (x < 0) ? -x : x;
    int32_t angle;

    if (abs_x >= abs_y) {
        /* |z| = abs_y/abs_x <= 1 */
        /* atan(z) ≈ z / (1 + 0.28125*z²)  scaled to avoid division */
        /* angle_mrad = (abs_y * 1000 / abs_x) * MDEG_PER_RAD / 1000 */
        int64_t z_scaled = ((int64_t)abs_y * 1000) / abs_x;  /* z*1000 */
        int64_t z2 = z_scaled * z_scaled / 1000000;           /* z²     */
        int64_t denom = 1000 + 281 * z2 / 1000;              /* 1000*(1+0.281z²) */
        angle = (int32_t)(z_scaled * MDEG_PER_RAD / denom);
    } else {
        /* |z| > 1: use atan(y/x) = 90° - atan(x/y) */
        int64_t z_scaled = ((int64_t)abs_x * 1000) / abs_y;
        int64_t z2 = z_scaled * z_scaled / 1000000;
        int64_t denom = 1000 + 281 * z2 / 1000;
        angle = 90000 - (int32_t)(z_scaled * MDEG_PER_RAD / denom);
    }

    /* Quadrant correction */
    if (x < 0 && y >= 0)
        angle = 180000 - angle;
    else if (x < 0 && y < 0)
        angle = -(180000 - angle);
    else if (x >= 0 && y < 0)
        angle = -angle;

    return angle;
}

/* -------------------------------------------------------------------------
 * iio_read_raw - Read one integer from an IIO sysfs file.
 * -------------------------------------------------------------------------*/
int iio_read_raw(const char *path, int32_t *value)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "[IIO] Cannot open %s: %s\n", path, strerror(errno));
        return -1;
    }
    int ret = fscanf(f, "%" SCNd32, value);
    fclose(f);
    if (ret != 1) {
        fprintf(stderr, "[IIO] Parse error reading %s\n", path);
        return -1;
    }
    return 0;
}

/* -------------------------------------------------------------------------
 * iio_poll_thread_fn - Main polling loop (Issue 7 + 8 combined)
 * -------------------------------------------------------------------------*/
void *iio_poll_thread_fn(void *arg)
{
    iio_thread_args_t *targs = (iio_thread_args_t *)arg;
    imu_data_t        *data  = targs->shared_data;
    pthread_mutex_t   *mtx   = targs->mutex;

    /* Build sysfs paths for all 6 axes */
    char path_ax[128], path_ay[128], path_az[128];
    char path_gx[128], path_gy[128], path_gz[128];

    snprintf(path_ax, sizeof(path_ax), "%s/in_accel_x_raw",       IIO_DEVICE_PATH);
    snprintf(path_ay, sizeof(path_ay), "%s/in_accel_y_raw",       IIO_DEVICE_PATH);
    snprintf(path_az, sizeof(path_az), "%s/in_accel_z_raw",       IIO_DEVICE_PATH);
    snprintf(path_gx, sizeof(path_gx), "%s/in_anglvel_x_raw",     IIO_DEVICE_PATH);
    snprintf(path_gy, sizeof(path_gy), "%s/in_anglvel_y_raw",     IIO_DEVICE_PATH);
    snprintf(path_gz, sizeof(path_gz), "%s/in_anglvel_z_raw",     IIO_DEVICE_PATH);

    struct timespec ts_sleep = {
        .tv_sec  = 0,
        .tv_nsec = IIO_POLL_INTERVAL_US * 1000L,  /* convert µs → ns */
    };

    fprintf(stderr, "[IIO] Polling thread started. Reading from %s\n",
            IIO_DEVICE_PATH);

    int32_t  ax_raw, ay_raw, az_raw;
    int32_t  gx_raw, gy_raw, gz_raw;
    uint64_t count = 0;

    while (targs->running) {
        /* --- Read all 6 raw axes from sysfs --- */
        if (iio_read_raw(path_ax, &ax_raw) != 0 ||
            iio_read_raw(path_ay, &ay_raw) != 0 ||
            iio_read_raw(path_az, &az_raw) != 0) {
            /* I2C timeout or device not ready — skip this sample */
            nanosleep(&ts_sleep, NULL);
            continue;
        }

        /* Gyroscope reads are best-effort; don't abort if they fail */
        if (iio_read_raw(path_gx, &gx_raw) != 0) gx_raw = 0;
        if (iio_read_raw(path_gy, &gy_raw) != 0) gy_raw = 0;
        if (iio_read_raw(path_gz, &gz_raw) != 0) gz_raw = 0;

        /* --- Convert raw counts to milli-g ---
         * IIO raw count × scale = physical value in m/s²
         * For ICM-20948 ±2g range: scale ≈ 0.000598 m/s²/count
         * milli-g = raw × 61 / 1000  (approximation, see iio_poll.h)  */
        int32_t ax_mg = (ax_raw * (int32_t)IIO_ACCEL_SCALE_NUM) / IIO_ACCEL_SCALE_DEN;
        int32_t ay_mg = (ay_raw * (int32_t)IIO_ACCEL_SCALE_NUM) / IIO_ACCEL_SCALE_DEN;
        int32_t az_mg = (az_raw * (int32_t)IIO_ACCEL_SCALE_NUM) / IIO_ACCEL_SCALE_DEN;

        /* --- Compute pitch and roll (milli-degrees) ---
         * pitch = atan2(ax, sqrt(ay² + az²))
         * roll  = atan2(ay, sqrt(ax² + az²))              */
        int32_t ay2_az2 = isqrt32(ay_mg * ay_mg + az_mg * az_mg);
        int32_t ax2_az2 = isqrt32(ax_mg * ax_mg + az_mg * az_mg);

        int32_t pitch = atan2_mdeg(ax_mg, ay2_az2);
        int32_t roll  = atan2_mdeg(ay_mg, ax2_az2);

        /* --- Tilt alert: |pitch| > 45000 mdeg = 45 degrees --- */
        int tilt = (pitch > TILT_THRESHOLD_MDEG ||
                    pitch < -TILT_THRESHOLD_MDEG) ? 1 : 0;

        count++;

        /* --- Update shared struct under mutex --- */
        pthread_mutex_lock(mtx);
        data->accel_x_mg  = ax_mg;
        data->accel_y_mg  = ay_mg;
        data->accel_z_mg  = az_mg;
        data->gyro_x_mdps = gx_raw;   /* raw mdps ≈ raw for ±250dps range */
        data->gyro_y_mdps = gy_raw;
        data->gyro_z_mdps = gz_raw;
        data->pitch_mdeg  = pitch;
        data->roll_mdeg   = roll;
        data->sample_count= count;
        data->tilt_alert  = tilt;
        pthread_mutex_unlock(mtx);

        /* Debug print every 50 samples (~1 sec at 50 Hz) */
        if (count % 50 == 0) {
            fprintf(stderr,
                "[IIO] sample=%lu  ax=%dmg ay=%dmg az=%dmg  "
                "pitch=%ddeg roll=%ddeg  tilt=%d\n",
                (unsigned long)count,
                ax_mg, ay_mg, az_mg,
                pitch / 1000, roll / 1000,
                tilt);
        }

        nanosleep(&ts_sleep, NULL);
    }

    fprintf(stderr, "[IIO] Polling thread exiting. Total samples: %lu\n",
            (unsigned long)count);
    return NULL;
}
