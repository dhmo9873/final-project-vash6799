/* iio_poll.h
 *
 * Issue 8: IIO Polling & Struct Packing
 *
 * Interface for the IIO sensor polling thread. Reads raw accelerometer
 * and gyroscope values from the ICM-20948's IIO sysfs interface, computes
 * pitch and roll in milli-degrees, and packs everything into a
 * telemetry_data struct that is shared (under mutex) with the DRM/V4L2 thread.
 */

#ifndef IIO_POLL_H
#define IIO_POLL_H

#include <stdint.h>
#include <pthread.h>

/* -------------------------------------------------------------------------
 * Shared telemetry data struct.
 *
 * ALL fields are in integer units to avoid floating-point in the hot path.
 * Pitch/roll are in milli-degrees (mdeg). Acceleration in milli-g (mg).
 *
 * Thread safety: always acquire imu_mutex before reading or writing.
 * -------------------------------------------------------------------------*/
typedef struct {
    int32_t accel_x_mg;     /* Acceleration X in milli-g              */
    int32_t accel_y_mg;     /* Acceleration Y in milli-g              */
    int32_t accel_z_mg;     /* Acceleration Z in milli-g              */
    int32_t gyro_x_mdps;    /* Gyroscope X in milli-degrees/sec       */
    int32_t gyro_y_mdps;    /* Gyroscope Y in milli-degrees/sec       */
    int32_t gyro_z_mdps;    /* Gyroscope Z in milli-degrees/sec       */
    int32_t pitch_mdeg;     /* Pitch angle in milli-degrees           */
    int32_t roll_mdeg;      /* Roll angle in milli-degrees            */
    uint64_t sample_count;  /* Total samples read since start         */
    int      tilt_alert;    /* 1 if |pitch| > TILT_THRESHOLD_MDEG     */
} imu_data_t;

/* Tilt threshold: 45 degrees = 45000 milli-degrees (Issue 11) */
#define TILT_THRESHOLD_MDEG   45000

/* IIO sysfs base path for the ICM-20948 */
#define IIO_DEVICE_PATH       "/sys/bus/iio/devices/iio:device0"

/* Target polling rate: 50 Hz = 20 ms sleep between reads.
 * This exceeds the Issue 8 DoD minimum of 30 Hz.             */
#define IIO_POLL_INTERVAL_US  20000   /* 20 ms = 50 Hz */

/* Scale factors for ICM-20948 in default ±2g / ±250dps range.
 * The IIO driver exposes raw integer counts; multiply by scale to get SI.
 * scale_accel = 0.000598550 m/s² per count → ×1000/9.80665 ≈ 0.061 mg/count
 * We use integer arithmetic: raw * 61 / 1000 ≈ milli-g.
 * These are approximate; read /sys/bus/iio/devices/iio:device0/in_accel_scale
 * for the exact value and adjust if needed.                   */
#define IIO_ACCEL_SCALE_NUM   61     /* numerator   */
#define IIO_ACCEL_SCALE_DEN   1000   /* denominator */

/* -------------------------------------------------------------------------
 * Thread control structure
 * -------------------------------------------------------------------------*/
typedef struct {
    imu_data_t   *shared_data;   /* Pointer to shared imu_data_t         */
    pthread_mutex_t *mutex;      /* Mutex protecting shared_data         */
    volatile int  running;       /* Set to 0 to request thread stop      */
} iio_thread_args_t;

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

/**
 * iio_poll_thread_fn - pthreads entry point for the IIO polling thread.
 *
 * Continuously reads IIO sysfs at IIO_POLL_INTERVAL_US, computes pitch/roll,
 * packs into shared imu_data_t under mutex lock, and checks tilt alert.
 *
 * Usage:
 *   pthread_t tid;
 *   iio_thread_args_t args = { &shared_imu, &imu_mutex, 1 };
 *   pthread_create(&tid, NULL, iio_poll_thread_fn, &args);
 *
 * @arg : pointer to iio_thread_args_t cast to void*
 * Returns NULL always (thread exit value unused).
 */
void *iio_poll_thread_fn(void *arg);

/**
 * iio_read_raw - Read a single raw integer from an IIO sysfs attribute.
 *
 * @path  : full path to the sysfs file (e.g., ".../in_accel_x_raw")
 * @value : output pointer; set to the parsed integer on success
 * Returns 0 on success, -1 on file/parse error.
 */
int iio_read_raw(const char *path, int32_t *value);

#endif /* IIO_POLL_H */
