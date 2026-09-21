#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"
#include "imu_transport.h"
#include "esp_log.h"

#define __IMU_CTRL_REG6_XL_FS_OFF           (3U)

#define WHO_AM_I_ADDR       0x0F
#define WHO_AM_I_VAL        0x69

#define OUT_G_START_ADDR    0x22
#define OUT_XL_START_ADDR   0x28

#define G_DATA_SIZE         6
#define XL_DATA_SIZE        6
#define XL_G_DATA_SIZE      12

#define CTRL_REG8_ADDR      0x22
#define CTRL_REG8_BOOT      0x80
#define CTRL_REG8_SW_RESET  0x01

#define IMU_CTRL_REG6_XL_FS_2G              (0x00 << __IMU_CTRL_REG6_XL_FS_OFF)
#define IMU_CTRL_REG6_XL_FS_16G             (0x01 << __IMU_CTRL_REG6_XL_FS_OFF)
#define IMU_CTRL_REG6_XL_FS_4G              (0x02 << __IMU_CTRL_REG6_XL_FS_OFF)
#define IMU_CTRL_REG6_XL_FS_8G              (0x03 << __IMU_CTRL_REG6_XL_FS_OFF)

#define __IMU_CTRL_REG1_G_FS_OFF            (3U)
#define IMU_CTRL_REG1_G_FS_245DPS           (0x00 << __IMU_CTRL_REG1_G_FS_OFF)
#define IMU_CTRL_REG1_G_FS_500DPS           (0x01 << __IMU_CTRL_REG1_G_FS_OFF)
#define IMU_CTRL_REG1_G_FS_2000DPS          (0x03 << __IMU_CTRL_REG1_G_FS_OFF)

#define XL_FS_2G_COEF       0.000061f
#define XL_FS_4G_COEF       0.000122f
#define XL_FS_8G_COEF       0.000244f
#define XL_FS_16G_COEF      0.000732f

#define G_FS_245DPS_COEF    0.00875f
#define G_FS_500DPS_COEF    0.01750f
#define G_FS_2000DPS_COEF   0.070f

#define IMU_DATA_X_IDX      0
#define IMU_DATA_Y_IDX      1
#define IMU_DATA_Z_IDX      2

#define TXRX_BUF_SIZE       1024

typedef struct {
    uint8_t tx[TXRX_BUF_SIZE];
    uint8_t rx[TXRX_BUF_SIZE];
} imu_runtime_t;

static imu_runtime_t s_runtime = {0};

static const char *TAG = "IMU";

static imu_data_t parse_xl_data(const uint8_t *data, uint8_t fs) {
    int16_t raw[3];
    for (size_t i = 0; i < 3; ++i) {
        raw[i] = (int16_t)(data[i * 2]) | (((int16_t)(data[(i * 2) + 1])) << 8);
    }

    imu_data_t parsed;

    switch (fs) {
    case IMU_CTRL_REG6_XL_FS_2G:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * XL_FS_2G_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * XL_FS_2G_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * XL_FS_2G_COEF;
        break;
    case IMU_CTRL_REG6_XL_FS_4G:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * XL_FS_4G_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * XL_FS_4G_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * XL_FS_4G_COEF;
        break;
    case IMU_CTRL_REG6_XL_FS_8G:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * XL_FS_8G_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * XL_FS_8G_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * XL_FS_8G_COEF;

        break;
    case IMU_CTRL_REG6_XL_FS_16G:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * XL_FS_16G_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * XL_FS_16G_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * XL_FS_16G_COEF;

        break;
    }

    return parsed;
}

static imu_data_t parse_g_data(const uint8_t *data, uint8_t fs) {
    int16_t raw[3];
    for (size_t i = 0; i < 3; ++i) {
        raw[i] = (int16_t)(data[i * 2]) | (((int16_t)(data[(i * 2) + 1])) << 8);
    }

    imu_data_t parsed;

    switch (fs) {
    case IMU_CTRL_REG1_G_FS_245DPS:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * G_FS_245DPS_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * G_FS_245DPS_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * G_FS_245DPS_COEF;

        break;
    case IMU_CTRL_REG1_G_FS_500DPS:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * G_FS_500DPS_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * G_FS_500DPS_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * G_FS_500DPS_COEF;

        break;
    case IMU_CTRL_REG1_G_FS_2000DPS:
        parsed.x = (float)raw[IMU_DATA_X_IDX] * G_FS_2000DPS_COEF;
        parsed.y = (float)raw[IMU_DATA_Y_IDX] * G_FS_2000DPS_COEF;
        parsed.z = (float)raw[IMU_DATA_Z_IDX] * G_FS_2000DPS_COEF;

        break;
    }

    return parsed;
}

esp_err_t imu_init() {
    esp_err_t rc = _imu_t_write_reg(CTRL_REG8_ADDR, CTRL_REG8_SW_RESET);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with LSM6DS3");
        return rc;
    }

    rc = _imu_t_write_reg(CTRL_REG8_ADDR, CTRL_REG8_BOOT);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with LSM6DS3");
        return rc;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    rc = _imu_t_read_reg(WHO_AM_I_ADDR, s_runtime.rx);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DS3: %s", esp_err_to_name(rc));
        return rc;
    }

    if (s_runtime.rx[0] != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I value: %02X", s_runtime.rx[0]);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t _imu_c_read_xl_data(imu_data_t *out, uint8_t fs) {
    if (out == NULL || 
        fs != IMU_CTRL_REG6_XL_FS_2G ||
        fs != IMU_CTRL_REG6_XL_FS_4G ||
        fs != IMU_CTRL_REG6_XL_FS_8G ||
        fs != IMU_CTRL_REG6_XL_FS_16G
    ) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t rc;

    rc = _imu_t_read(OUT_XL_START_ADDR, s_runtime.rx, XL_DATA_SIZE);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DS3: %s", esp_err_to_name(rc));
        return ESP_FAIL;
    }

    *out = parse_xl_data(s_runtime.rx, fs);

    return ESP_OK;
}

esp_err_t _imu_c_read_g_data(imu_data_t *out, uint8_t fs) {
    if (out == NULL ||
        (fs != IMU_CTRL_REG1_G_FS_245DPS &&
        fs != IMU_CTRL_REG1_G_FS_500DPS &&
        fs != IMU_CTRL_REG1_G_FS_2000DPS))
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t rc;

    rc = _imu_t_read(OUT_G_START_ADDR, s_runtime.rx, G_DATA_SIZE);
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read LSM6DS3: %s", esp_err_to_name(rc));
        return ESP_FAIL;
    }

    *out = parse_g_data(s_runtime.rx, fs);

    return ESP_OK;
}