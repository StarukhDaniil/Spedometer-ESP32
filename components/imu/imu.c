#include "imu.h"
#include "imu_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WHO_AM_I       0x0f
#define CTRL1_XL       0x10
#define CTRL2_G        0x11
#define CTRL3_C        0x12
#define CTRL4_C        0x13
#define CTRL6_C        0x15
#define OUT_G          0x22
#define OUT_XL         0x28
#define FIFO_CTRL1     0x06
#define FIFO_CTRL2     0x07
#define FIFO_CTRL3     0x08
#define FIFO_CTRL4     0x09
#define FIFO_CTRL5     0x0a
#define INT1_CTRL      0x0d
#define INT2_CTRL      0x0e

static bool s_initialized;

static esp_err_t update_reg(uint8_t addr, uint8_t mask, uint8_t value)
{
    esp_err_t rc;

    uint8_t old;

    rc = _imu_t_read_reg(addr, &old);

    if (rc != ESP_OK) {
        return rc;
    }

    return _imu_t_write_reg(addr, (old & (uint8_t)~mask) | (value & mask));
}

esp_err_t imu_init(void)
{
    esp_err_t rc;

    if (s_initialized) {
        return ESP_OK;
    }

    rc = _imu_t_init();

    if (rc != ESP_OK) {
        return rc;
    }

    uint8_t value;

    rc = _imu_t_read_reg(WHO_AM_I, &value);

    if (rc != ESP_OK) {
        return rc;
    }

    if (value != 0x69) {
        return ESP_ERR_NOT_FOUND;
    }

    rc = _imu_t_write_reg(CTRL3_C, 0x01);

    if (rc != ESP_OK) {
        return rc;
    }

    /* Bound the reset wait, including when the tick period exceeds 1 ms. */
    TickType_t start = xTaskGetTickCount();

    do {
        vTaskDelay(1);

        rc = _imu_t_read_reg(CTRL3_C, &value);

        if (rc != ESP_OK) {
            return rc;
        }

        if ((value & 0x01) == 0) {
            /* BDU | IF_INC, little-endian */
            rc = _imu_t_write_reg(CTRL3_C, 0x44);

            if (rc != ESP_OK) {
                return rc;
            }

            s_initialized = true;

            return ESP_OK;
        }
    } while ((TickType_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(100));

    return ESP_ERR_TIMEOUT;
}

esp_err_t imu_configure_xl(imu_odr_t odr, imu_xl_fs_t fs)
{
    if ((unsigned)odr > IMU_ODR_6660_HZ || (unsigned)fs > IMU_XL_FS_8G) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return update_reg(CTRL1_XL, 0xfc, ((uint8_t)odr << 4) | ((uint8_t)fs << 2));
}

esp_err_t imu_configure_g(imu_odr_t odr, imu_g_fs_t fs)
{
    if ((unsigned)odr > IMU_ODR_1660_HZ ||
        (fs != IMU_G_FS_125DPS && fs != IMU_G_FS_245DPS &&
         fs != IMU_G_FS_500DPS && fs != IMU_G_FS_1000DPS && fs != IMU_G_FS_2000DPS)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return update_reg(CTRL2_G, 0xfe, ((uint8_t)odr << 4) | (uint8_t)fs);
}

esp_err_t imu_reboot(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t addresses[] = {CTRL1_XL, CTRL2_G, CTRL3_C, CTRL6_C, FIFO_CTRL5};
    uint8_t saved[sizeof(addresses)];

    for (size_t i = 0; i < sizeof(addresses); ++i) {
        esp_err_t rc = _imu_t_read_reg(addresses[i], &saved[i]);

        if (rc != ESP_OK) {
            return rc;
        }
    }

    if (saved[2] & 0x81) {
        return ESP_ERR_INVALID_STATE;
    }

    /* A failed reboot leaves settings uncertain; require imu_init() recovery. */
    s_initialized = false;

    esp_err_t rc = _imu_t_write_reg(FIFO_CTRL5, saved[4] & 0xf8);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = _imu_t_write_reg(CTRL2_G, saved[1] & 0x0f);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = _imu_t_write_reg(CTRL6_C, saved[3] & 0xef);

    if (rc != ESP_OK) {
        return rc;
    }

    /* Run XL at 104 Hz during boot, including when it was powered down. */
    rc = _imu_t_write_reg(CTRL1_XL, (saved[0] & 0x0f) | 0x40);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = _imu_t_write_reg(CTRL3_C, saved[2] | 0x80);

    if (rc != ESP_OK) {
        return rc;
    }

    TickType_t start = xTaskGetTickCount();

    /* No register access during boot. Extra tick covers tick-boundary rounding. */
    vTaskDelay(pdMS_TO_TICKS(20) + 1);

    while (true) {
        uint8_t ctrl;

        rc = _imu_t_read_reg(CTRL3_C, &ctrl);

        if (rc != ESP_OK) {
            return rc;
        }

        if ((ctrl & 0x80) == 0) {
            break;
        }

        if ((TickType_t)(xTaskGetTickCount() - start) >= pdMS_TO_TICKS(100)) {
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(1);
    }

    /* Restore sensor settings first; resume FIFO only after restoration succeeds. */
    const size_t restore_order[] = {0, 3, 1, 4};

    for (size_t i = 0; i < sizeof(restore_order) / sizeof(restore_order[0]); ++i) {
        size_t index = restore_order[i];

        rc = _imu_t_write_reg(addresses[index], saved[index]);

        if (rc != ESP_OK) {
            return rc;
        }
    }

    s_initialized = true;

    return ESP_OK;
}

static float axis(const uint8_t *bytes, float sensitivity)
{
    uint16_t bits = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);

    int32_t signed_value = bits >= 0x8000 ? (int32_t)bits - 65536 : bits;

    return signed_value * sensitivity;
}

static esp_err_t read_scaled(uint8_t addr, float sensitivity, imu_data_t *out)
{
    esp_err_t rc;

    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[6];

    rc = _imu_t_read(addr, data, sizeof(data));

    if (rc != ESP_OK) {
        return rc;
    }

    *out = (imu_data_t){axis(data, sensitivity), axis(data + 2, sensitivity),
                        axis(data + 4, sensitivity)};

    return ESP_OK;
}

esp_err_t imu_read_xl_data(imu_data_t *out)
{
    esp_err_t rc;

    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ctrl;

    rc = _imu_t_read_reg(CTRL1_XL, &ctrl);

    if (rc != ESP_OK) {
        return rc;
    }

    static const float scale[] = {0.000061f, 0.000488f, 0.000122f, 0.000244f};

    return read_scaled(OUT_XL, scale[(ctrl >> 2) & 3], out);
}

esp_err_t imu_read_g_data(imu_data_t *out)
{
    esp_err_t rc;

    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ctrl;

    rc = _imu_t_read_reg(CTRL2_G, &ctrl);

    if (rc != ESP_OK) {
        return rc;
    }

    static const float scale[] = {0.00875f, 0.0175f, 0.035f, 0.070f};

    return read_scaled(OUT_G, (ctrl & 2) ? 0.004375f : scale[(ctrl >> 2) & 3], out);
}

esp_err_t imu_set_fifo_threshold(uint16_t words)
{
    esp_err_t rc;

    if (words > 4095) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    rc = _imu_t_write_reg(FIFO_CTRL1, (uint8_t)words);

    if (rc != ESP_OK) {
        return rc;
    }

    return update_reg(FIFO_CTRL2, 0x0f, (uint8_t)(words >> 8));
}

esp_err_t imu_configure_fifo(const imu_fifo_config_t *config)
{
    esp_err_t rc;

    if (config == NULL || config->threshold_words > 4095 ||
        (unsigned)config->odr > IMU_ODR_6660_HZ ||
        (unsigned)config->xl_decimation > IMU_FIFO_DECIMATE_32 ||
        (unsigned)config->g_decimation > IMU_FIFO_DECIMATE_32 ||
        (config->mode != IMU_FIFO_BYPASS && config->mode != IMU_FIFO_STOP_WHEN_FULL &&
         config->mode != IMU_FIFO_CONTINUOUS)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->mode != IMU_FIFO_BYPASS &&
        (config->odr == IMU_ODR_OFF ||
         (config->xl_decimation == IMU_FIFO_DISABLED && config->g_decimation == IMU_FIFO_DISABLED))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config->mode != IMU_FIFO_BYPASS) {
        uint8_t xl, g;

        rc = _imu_t_read_reg(CTRL1_XL, &xl);

        if (rc != ESP_OK) {
            return rc;
        }

        rc = _imu_t_read_reg(CTRL2_G, &g);

        if (rc != ESP_OK) {
            return rc;
        }

        if ((config->xl_decimation != IMU_FIFO_DISABLED &&
             ((xl >> 4) == 0 || (xl >> 4) > config->odr)) ||
            (config->g_decimation != IMU_FIFO_DISABLED &&
             ((g >> 4) == 0 || (g >> 4) > config->odr))) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    /* Bypass first: flush old pattern. */
    rc = update_reg(FIFO_CTRL5, 0x7f, 0);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = imu_set_fifo_threshold(config->threshold_words);

    if (rc != ESP_OK) {
        return rc;
    }

    /* No timestamp/pedometer batching. */
    rc = update_reg(FIFO_CTRL2, 0xc0, 0);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = update_reg(FIFO_CTRL3, 0x3f,
        (uint8_t)config->xl_decimation | ((uint8_t)config->g_decimation << 3));

    if (rc != ESP_OK) {
        return rc;
    }

    /* 16-bit, no third/fourth datasets. */
    rc = update_reg(FIFO_CTRL4, 0x7f, 0);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = update_reg(CTRL4_C, 0x11, config->stop_on_threshold ? 1 : 0);

    if (rc != ESP_OK) {
        return rc;
    }

    return update_reg(FIFO_CTRL5, 0x7f, ((uint8_t)config->odr << 3) | (uint8_t)config->mode);
}

esp_err_t imu_set_fifo_threshold_interrupt(bool int1, bool int2)
{
    esp_err_t rc;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    rc = update_reg(INT1_CTRL, 0x08, int1 ? 0x08 : 0);

    if (rc != ESP_OK) {
        return rc;
    }

    return update_reg(INT2_CTRL, 0x08, int2 ? 0x08 : 0);
}

esp_err_t _imu_c_read_xl_data(imu_data_t *out, uint8_t fs)
{
    if (fs != 0 && fs != 8 && fs != 16 && fs != 24) {
        return ESP_ERR_INVALID_ARG;
    }

    static const float scale[] = {0.000061f, 0.000488f, 0.000122f, 0.000244f};

    return read_scaled(OUT_XL, scale[fs >> 3], out);
}

esp_err_t _imu_c_read_g_data(imu_data_t *out, uint8_t fs)
{
    if (fs != 0 && fs != 8 && fs != 24) {
        return ESP_ERR_INVALID_ARG;
    }

    static const float scale[] = {0.00875f, 0.0175f, 0.035f, 0.070f};

    return read_scaled(OUT_G, scale[fs >> 3], out);
}
