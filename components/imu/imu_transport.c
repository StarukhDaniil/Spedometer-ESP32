#include "imu_transport.h"

#include "driver/spi_master.h"
#include <string.h>

#define TXRX_BUF_SIZE       1024

#define SPI_HOST            SPI3_HOST

#define MISO_PIN            16
#define MOSI_PIN            17
#define SCLK_PIN            18
#define CS_PIN              19

#define WRITE_BIT           0x00
#define READ_BIT            BIT7

typedef struct {
    uint8_t *tx_buf;
    uint8_t *rx_buf;
    spi_device_handle_t spi_dev;
    EventGroupHandle_t eg;
} imu_transport_runtime_t;

static imu_transport_runtime_t s_runtime = {0};

esp_err_t _imu_t_init() {
    spi_bus_config_t bus_cfg = {
        .miso_io_num = MISO_PIN,
        .mosi_io_num = MOSI_PIN,
        .sclk_io_num = SCLK_PIN,
        .data_io_default_level = 1,
    };

    esp_err_t rc = spi_bus_initialize(SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (rc) return rc;

    s_runtime.tx_buf = spi_bus_dma_memory_alloc(SPI_HOST, TXRX_BUF_SIZE, 0);
    if (s_runtime.tx_buf == NULL) return ESP_ERR_NO_MEM;

    s_runtime.rx_buf = spi_bus_dma_memory_alloc(SPI_HOST, TXRX_BUF_SIZE, 0);
    if (s_runtime.rx_buf == NULL) return ESP_ERR_NO_MEM;

    spi_device_interface_config_t dev_cfg = {
        .mode = 3,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .clock_speed_hz = 10 * 1000 * 1000,
        .dummy_bits = 0,
        .spics_io_num = CS_PIN
    };

    rc = spi_bus_add_device(SPI_HOST, &dev_cfg, &s_runtime.spi_dev);
    if (rc) return rc;

    s_runtime.eg = xEventGroupCreate();
    if (s_runtime.eg == NULL) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t _imu_t_write_reg(uint8_t addr, uint8_t val) {
    s_runtime.tx_buf[0] = WRITE_BIT | addr;
    s_runtime.tx_buf[1] = val;

    spi_transaction_t transaction = {
        .length = 2,
        .rx_buffer = NULL,
        .tx_buffer = s_runtime.tx_buf
    };

    esp_err_t rc = spi_device_transmit(s_runtime.spi_dev, &transaction);
    if (rc) return rc;

    return ESP_OK;
}

esp_err_t _imu_t_read_reg(uint8_t addr, uint8_t *out) {
    s_runtime.tx_buf[0] = READ_BIT | addr;

    spi_transaction_t transaction = {
        .length = 2,
        .rx_buffer = s_runtime.rx_buf,
        .tx_buffer = s_runtime.tx_buf
    };

    esp_err_t rc = spi_device_transmit(s_runtime.spi_dev, &transaction);
    if (rc) return rc;

    *out = s_runtime.rx_buf[1];

    return ESP_OK;
}

esp_err_t _imu_t_read(uint8_t start_addr, uint8_t *out, size_t size) {
    s_runtime.tx_buf[0] = READ_BIT | start_addr;

    spi_transaction_t transaction = {
        .length = size + 1,
        .rx_buffer = s_runtime.rx_buf,
        .tx_buffer = s_runtime.tx_buf
    };

    esp_err_t rc = spi_device_transmit(s_runtime.spi_dev, &transaction);
    if (rc) return rc;

    memcpy(out, s_runtime.rx_buf + 1, size);

    return ESP_OK;
}