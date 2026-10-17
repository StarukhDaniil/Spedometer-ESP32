#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float z;
} imu_data_t;

/* Register encodings, not numerical Hz values. Gyro supports up to 1660 Hz. */
typedef enum {
    IMU_ODR_OFF = 0, IMU_ODR_12_5_HZ, IMU_ODR_26_HZ,
    IMU_ODR_52_HZ, IMU_ODR_104_HZ, IMU_ODR_208_HZ,
    IMU_ODR_416_HZ, IMU_ODR_833_HZ, IMU_ODR_1660_HZ,
    IMU_ODR_3330_HZ, IMU_ODR_6660_HZ,
} imu_odr_t;

typedef enum {
    IMU_XL_FS_2G = 0, IMU_XL_FS_16G, IMU_XL_FS_4G, IMU_XL_FS_8G,
} imu_xl_fs_t;

typedef enum {
    IMU_G_FS_245DPS = 0, IMU_G_FS_125DPS = 2,
    IMU_G_FS_500DPS = 4, IMU_G_FS_1000DPS = 8, IMU_G_FS_2000DPS = 12,
} imu_g_fs_t;

/* Accelerometer analog anti-aliasing bandwidth in high-performance mode.
 * AUTO follows ODR. These are not the cutoff values of the digital LPF stages. */
typedef enum {
    IMU_XL_BW_AUTO = 0,
    IMU_XL_BW_400_HZ,
    IMU_XL_BW_200_HZ,
    IMU_XL_BW_100_HZ,
    IMU_XL_BW_50_HZ,
} imu_xl_bandwidth_t;

typedef enum {
    IMU_FIFO_BYPASS = 0,
    IMU_FIFO_STOP_WHEN_FULL = 1,
    IMU_FIFO_CONTINUOUS = 6,
} imu_fifo_mode_t;

typedef enum {
    IMU_FIFO_DISABLED = 0, IMU_FIFO_NO_DECIMATION,
    IMU_FIFO_DECIMATE_2, IMU_FIFO_DECIMATE_3, IMU_FIFO_DECIMATE_4,
    IMU_FIFO_DECIMATE_8, IMU_FIFO_DECIMATE_16, IMU_FIFO_DECIMATE_32,
} imu_fifo_decimation_t;

typedef struct {
    imu_fifo_mode_t mode;
    imu_odr_t odr;
    imu_fifo_decimation_t xl_decimation;
    imu_fifo_decimation_t g_decimation;
    imu_xl_bandwidth_t xl_bandwidth; /* Also applied to live accel output. */
    uint16_t threshold_words; /* 0..4095, 16-bit axis words, NOT bytes/XYZ samples. */
    bool stop_on_threshold;
} imu_fifo_config_t;

typedef struct {
    imu_data_t g;  /* deg/s */
    imu_data_t xl; /* g */
} imu_fifo_sample_t;

typedef void (*imu_fifo_callback_t)(const imu_fifo_sample_t *samples,
                                    size_t count, void *context);

typedef struct {
    int interrupt_gpio; /* ESP32 input connected to the sensor INT1/INT2. */
    bool use_int2;
    imu_odr_t odr; /* Both sensors: 12.5..1660 Hz before FIFO decimation. */
    imu_xl_fs_t xl_fs;
    imu_g_fs_t g_fs;
    imu_xl_bandwidth_t xl_bandwidth;
    /* Paired streaming requires IMU_FIFO_NO_DECIMATION for both sensors:
     * hardware needs an undecimated dataset, and pairs require matching rates.
     * Reduce odr to lower the rate of paired samples. */
    imu_fifo_decimation_t xl_decimation;
    imu_fifo_decimation_t g_decimation;
    uint16_t watermark_samples; /* 1..682 paired XYZ samples (six words each). */
} imu_stream_config_t;

/* Requires init. Configures both sensors, continuous FIFO, GPIO ISR and worker.
 * Wire the selected sensor INT pin to interrupt_gpio; timeout wakeups only
 * drain when that pin is high. SPI FIFO status reads require an interrupt.
 * Each drain reads the FIFO occupancy snapshot in one SPI payload transaction.
 * Partial pairs are retained until the next drain; callbacks receive complete pairs.
 * Callback runs in the worker task; data is valid only during the call. Keep it
 * short and do not call any imu API from it. Copy/queue data for lengthy work.
 * Other driver operations are rejected while streaming. Stop before changing
 * settings. Overruns are logged and flush the FIFO; samples have no timestamps.
 * GPIO ISR service must use non-IRAM flags (this ISR uses FreeRTOS event groups).
 * On partial configuration failure, call imu_init() before retrying.
 */
esp_err_t imu_start_fifo(const imu_stream_config_t *config,
                         imu_fifo_callback_t callback, void *context);
/* Waits for an in-progress callback, removes GPIO handler, disables routing and
 * flushes FIFO. Sensors remain powered on. No callbacks occur after return.
 * Worker/event group persist for reuse; the shared GPIO ISR service is retained.
 */
esp_err_t imu_stop_fifo(void);

/* Singleton using SPI3, MISO=16, MOSI=17, SCLK=18, CS=15.
 * Serialize initial init calls; subsequent calls are internally serialized.
 * Do not call from an ISR or the FIFO callback.
 * Init verifies WHO_AM_I, resets, enables BDU and address increment.
 * Sensors remain powered down until configured. Repeated successful init is a no-op.
 */
esp_err_t imu_init(void);

/* Reload factory calibration using BOOT, then restore sensor/FIFO settings.
 * Requires successful init. Blocks at least 20 ms; completion timeout 100 ms.
 * Flushes FIFO; allow sensor settling before consuming new samples.
 * On failure after reconfiguration starts, call imu_init() and reapply settings.
 * This is calibration reboot, not a reset to register defaults.
 */
esp_err_t imu_reboot(void);

/* Configure ODR, full scale and (for accel) analog bandwidth together.
 * Put FIFO in bypass before changing sensor settings to avoid mixing scales/rates in buffered samples.
 * Allow sensor settling time after enabling/changing settings before reading.
 * Gyro low-pass bandwidth follows its ODR; it has no independent selector.
 * Accel configuration can partially apply on a bus error; retry the full call.
 */
esp_err_t imu_configure_xl(imu_odr_t odr, imu_xl_fs_t fs, imu_xl_bandwidth_t bandwidth);
esp_err_t imu_configure_g(imu_odr_t odr, imu_g_fs_t fs);

/* Latest output registers, scaled using the cached configured FS: g and deg/s.
 * These do not wait for data-ready and may return the previous sample.
 */
esp_err_t imu_read_xl_data(imu_data_t *out);
esp_err_t imu_read_g_data(imu_data_t *out);

/* Reconfiguration flushes FIFO via bypass. Configures 16-bit accel/gyro only.
 * Configure enabled sensors first; use a FIFO ODR >= their ODR, and use
 * decimation to reduce stored data (1, 2, 3, 4, 8, 16 or 32; 0 disables a source).
 * This config also applies xl_bandwidth, preserving the accel ODR/full scale.
 * Decimation drops samples; it does not apply additional anti-alias filtering.
 * Active FIFO requires a nonzero ODR and at least one source set to
 * IMU_FIFO_NO_DECIMATION (AN4650 section 8.1.3). Other sources may be decimated. On a bus error, settings may be partially
 * applied: retry the complete configuration before consuming FIFO data.
 */
esp_err_t imu_configure_fifo(const imu_fifo_config_t *config);
/* Updates the 12-bit watermark without flushing FIFO. Zero is permitted by
 * hardware but is not useful as a nonempty-data notification. A partial bus
 * failure may leave a partial watermark; retry before relying on it.
 */
esp_err_t imu_set_fifo_threshold(uint16_t words);
/* Route watermark to sensor INT1/INT2. Does not install an ESP32 GPIO ISR.
 * A bus failure can partially update routing; retry on error.
 */
esp_err_t imu_set_fifo_threshold_interrupt(bool int1, bool int2);

#ifdef __cplusplus
}
#endif
