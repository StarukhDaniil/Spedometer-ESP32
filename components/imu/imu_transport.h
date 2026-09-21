#pragma once

#include <stdint.h>
#include "esp_err.h"

esp_err_t _imu_t_init();
esp_err_t _imu_t_write_reg(uint8_t addr, uint8_t val);
esp_err_t _imu_t_read_reg(uint8_t addr, uint8_t *out);
esp_err_t _imu_t_read(uint8_t start_addr, uint8_t *out, size_t size);