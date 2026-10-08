# LSM6DS3 IMU

SPI3 wiring: MISO GPIO16, MOSI GPIO17, SCLK GPIO18, CS GPIO19, mode 3,
10 MHz. This component owns the bus. Serialize calls from tasks; no ISR calls.

Initialization leaves both sensors powered down. For 104 Hz, ±4 g,
±500 degrees/s and a continuous FIFO:

```c
ESP_ERROR_CHECK(imu_init());
ESP_ERROR_CHECK(imu_configure_xl(IMU_ODR_104_HZ, IMU_XL_FS_4G));
ESP_ERROR_CHECK(imu_configure_g(IMU_ODR_104_HZ, IMU_G_FS_500DPS));
imu_fifo_config_t fifo = {
    .mode = IMU_FIFO_CONTINUOUS,
    .odr = IMU_ODR_104_HZ,
    .xl_decimation = IMU_FIFO_NO_DECIMATION,
    .g_decimation = IMU_FIFO_NO_DECIMATION,
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
The sequence follows [ST's AN4650 guidance](https://community.st.com/mems-sensors-48/lsm6ds3-what-exactly-does-sw-reset-do-78552).

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
The old `_imu_c_read_*` functions remain as compatibility wrappers;
their explicit scale argument uses the old encoding documented in imu.h.

Register fields and sensitivities were checked against
[ST's LSM6DS3 driver](https://github.com/STMicroelectronics/lsm6ds3-pid).
Fixes include SPI bit lengths/command framing, transport initialization,
reset at CTRL3_C, bounded reset polling, BDU/auto-increment, correct FS
fields and ±16 g conversion, argument validation, and bus error propagation.

Run mocked-register regression tests from the project root:

```sh
cc -std=c17 -Wall -Wextra -Werror -Icomponents/imu/tests/stubs \
  -Icomponents/imu/include -Icomponents/imu components/imu/imu.c \
  components/imu/tests/test_imu.c -lm -o /tmp/imu-register-tests
/tmp/imu-register-tests
```

These tests do not validate electrical wiring, sensor timing or physical SPI.
