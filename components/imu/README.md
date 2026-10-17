# LSM6DS3 IMU

SPI3 wiring: MISO GPIO16, MOSI GPIO17, SCLK GPIO18, CS GPIO15, mode 3,
10 MHz. This component owns the bus. Serialize calls from tasks; no ISR calls.

Initialization leaves both sensors powered down. For 104 Hz, ±4 g,
±500 degrees/s and a continuous FIFO:

```c
ESP_ERROR_CHECK(imu_init());
ESP_ERROR_CHECK(imu_configure_xl(IMU_ODR_104_HZ, IMU_XL_FS_4G, IMU_XL_BW_50_HZ));
ESP_ERROR_CHECK(imu_configure_g(IMU_ODR_104_HZ, IMU_G_FS_500DPS));
imu_fifo_config_t fifo = {
    .mode = IMU_FIFO_CONTINUOUS,
    .odr = IMU_ODR_104_HZ,
    .xl_decimation = IMU_FIFO_NO_DECIMATION,
    .g_decimation = IMU_FIFO_NO_DECIMATION,
    .xl_bandwidth = IMU_XL_BW_50_HZ,
    .threshold_words = 60,
    .stop_on_threshold = false,
};
ESP_ERROR_CHECK(imu_configure_fifo(&fifo));
// Optional: route watermark to sensor INT1; wire it to an ESP32 input separately.
ESP_ERROR_CHECK(imu_set_fifo_threshold_interrupt(true, false));
// Change the threshold without flushing buffered data:
ESP_ERROR_CHECK(imu_set_fifo_threshold(120));
```

Call `imu_reboot()` after initialization to reload factory calibration using
the LSM6DS3 BOOT sequence. It waits at least 20 ms before accessing registers,
checks completion with a 100 ms timeout, and restores sensor/FIFO settings.
FIFO contents are discarded. Allow sensor settling afterward. If reboot fails
after changing hardware state, call `imu_init()` and reapply configuration.

Thresholds are 0–4095 **16-bit words**. One XYZ sample is three words.
At equal ODR with no decimation, an accel + gyro pair is six words, so
60 words holds ten pairs. Configuring FIFO flushes previous contents.
Bypass, stop-when-full, and continuous modes are supported. Trigger-based
mode transitions, FIFO draining/pattern decoding and ESP32 GPIO interrupt
handling are not implemented by these configuration APIs.

`imu_read_xl_data()` returns g; `imu_read_g_data()` returns degrees/s.
They read output registers, not FIFO, and use the current hardware full scale.
Wait for sensor settling/data-ready as appropriate before consuming readings.
Put FIFO in bypass before changing ODR/FS and configure it again afterward.
## Bandwidth and decimation

`imu_configure_xl(odr, fs, bandwidth)` selects automatic analog bandwidth
(`IMU_XL_BW_AUTO`) or 400, 200, 100, or 50 Hz. `imu_configure_fifo()` also
applies its `xl_bandwidth` field to the sensor, including normal output reads;
use the same value in both configurations. These settings control the analog
anti-alias filter, not the accelerometer's digital filter stages.

`imu_configure_g(odr, fs)` keeps the gyro low-pass cutoff tied to ODR; the
LSM6DS3 has no separate gyro low-pass bandwidth selector. See
[ST AN4650, sections 2.6–2.7](https://cdn.sparkfun.com/assets/learn_tutorials/4/1/6/AN4650_DM00157511.pdf).

`imu_fifo_config_t` accepts independent `xl_decimation` and `g_decimation`:
disabled, no decimation, or factors 2, 3, 4, 8, 16 and 32. At least one enabled
source must use `IMU_FIFO_NO_DECIMATION` (AN4650 section 8.1.3). Both sources
set to `/2`, for example, is invalid. Decimation drops
samples and does not provide additional anti-alias filtering.

For callback streaming, both sources must use `IMU_FIFO_NO_DECIMATION`:
paired samples need matching rates, and the hardware requires an undecimated
source. Use a lower sensor ODR to reduce the paired sample rate. Stop streaming
before changing settings. The temporary `main.c` example uses 52 Hz sensors,
no FIFO decimation, and 50 Hz accel analog bandwidth.

Connect the selected IMU INT1/INT2 pin to the configured ESP32 GPIO. The worker
reads FIFO status after an interrupt; a timeout only triggers a read when the
interrupt pin is already high. This follows the SPI status-read requirement in
AN4650 section 8.4. Configuration registers are checked once at stream startup
using two burst reads, and mismatches fail startup with a diagnostic.
