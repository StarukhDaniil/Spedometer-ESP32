#pragma once

#include "esp_err.h"

typedef struct {
    float x;
    float y;
    float z;
} imu_data_t;

esp_err_t imu_init();
esp_err_t _imu_c_read_xl_data(imu_data_t *out, uint8_t fs);
esp_err_t _imu_c_read_g_data(imu_data_t *out, uint8_t fs);