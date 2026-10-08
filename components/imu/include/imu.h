#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x, y, z;
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
    uint16_t threshold_words; /* 0..4095, 16-bit axis words, NOT bytes/XYZ samples. */
    bool stop_on_threshold;
} imu_fifo_config_t;

/* Singleton using SPI3, MISO=16, MOSI=17, SCLK=18, CS=19.
 * Serialize ALL calls (including init) from tasks; do not call from an ISR.
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

/* Configure ODR and full scale together. Put FIFO in bypass before changing
 * sensor settings to avoid mixing scales/rates in buffered samples.
 * Allow sensor settling time after enabling/changing settings before reading.
 */
esp_err_t imu_configure_xl(imu_odr_t odr, imu_xl_fs_t fs);
esp_err_t imu_configure_g(imu_odr_t odr, imu_g_fs_t fs);

/* Latest output registers, scaled using the current hardware FS: g and deg/s.
 * These do not wait for data-ready and may return the previous sample.
 */
esp_err_t imu_read_xl_data(imu_data_t *out);
esp_err_t imu_read_g_data(imu_data_t *out);

/* Reconfiguration flushes FIFO via bypass. Configures 16-bit accel/gyro only.
 * Configure enabled sensors first; use a FIFO ODR >= their ODR, and use
 * decimation to reduce stored data. Active FIFO requires a nonzero ODR and
 * at least one enabled source. On a bus error, settings may be partially
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

/* Legacy reads: fs uses the old shifted encoding (XL: 0/8/16/24 for
 * 2/16/4/8 g; G: 0/8/24 for 245/500/2000 dps). Prefer the reads above,
 * which cannot accidentally use a scale different from the sensor setting.
 */
esp_err_t _imu_c_read_xl_data(imu_data_t *out, uint8_t fs);
esp_err_t _imu_c_read_g_data(imu_data_t *out, uint8_t fs);

#ifdef __cplusplus
}
#endif
