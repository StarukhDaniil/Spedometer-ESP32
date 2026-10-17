#include "imu_transport.h"
#include "driver/spi_master.h"

#define IMU_SPI_HOST    SPI3_HOST

static spi_device_handle_t s_device;

esp_err_t _imu_t_init(void)
{
    if (s_device != NULL) {
        return ESP_OK;
    }

    const spi_bus_config_t bus = {
        .miso_io_num = IMU_MISO_PIN, .mosi_io_num = IMU_MOSI_PIN, .sclk_io_num = IMU_SCLK_PIN,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = IMU_FIFO_CAPACITY_BYTES,
    };

    esp_err_t rc = spi_bus_initialize(IMU_SPI_HOST, &bus, SPI_DMA_CH_AUTO);

    if (rc != ESP_OK) {
        return rc;
    }

    const spi_device_interface_config_t device = {
        .command_bits = 8, .mode = 3,
        .clock_speed_hz = 10 * 1000 * 1000,
        .spics_io_num = IMU_CS_PIN, .queue_size = 1,
    };

    rc = spi_bus_add_device(IMU_SPI_HOST, &device, &s_device);

    if (rc != ESP_OK) {
        spi_bus_free(IMU_SPI_HOST);
    }

    return rc;
}

esp_err_t _imu_t_write_reg(uint8_t addr, uint8_t val)
{
    if (addr > 0x7f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t transaction = {
        .cmd = addr, .length = 8,
        .flags = SPI_TRANS_USE_TXDATA, .tx_data = {val},
    };

    return spi_device_transmit(s_device, &transaction);
}

esp_err_t _imu_t_read_reg(uint8_t addr, uint8_t *out)
{
    return _imu_t_read(addr, out, 1);
}

esp_err_t _imu_t_read(uint8_t addr, uint8_t *out, size_t size)
{
    if (out == NULL || size == 0 || size > IMU_FIFO_CAPACITY_BYTES || addr > 0x7f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_device == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_transaction_t transaction = {
        .cmd = 0x80 | addr, .length = size * 8, .rx_buffer = out,
    };

    return spi_device_transmit(s_device, &transaction);
}
