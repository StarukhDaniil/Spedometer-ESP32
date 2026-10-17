#include "imu.h"
#include "imu_transport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"
#include <string.h>

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

#define FIFO_STATUS1   0x3a
#define FIFO_DATA_OUT  0x3e
#define FIFO_READY     BIT0

static bool s_initialized;
/* Configuration registers only; status and samples are not cached. */
static uint8_t s_registers[0x16];

static SemaphoreHandle_t s_lock;
static EventGroupHandle_t s_events;
static TaskHandle_t s_task;

static bool s_streaming;
static bool s_fifo_restart_pending;
static uint8_t s_stream_fifo_ctrl5;
static int s_interrupt_gpio;
static imu_fifo_callback_t s_callback;
static void *s_callback_context;

/* Keep full-FIFO buffers off the worker's 4 KiB stack. */
static DMA_ATTR uint8_t s_fifo_raw[IMU_FIFO_CAPACITY_BYTES];
static imu_fifo_sample_t s_fifo_samples[(IMU_FIFO_CAPACITY_BYTES / 2 + 5) / 6];

/* Retain incomplete gyro/accel pairs between reads. */
static uint8_t s_fifo_partial[12];
static unsigned s_fifo_partial_words;

static esp_err_t write_reg(uint8_t addr, uint8_t value)
{
    esp_err_t rc = _imu_t_write_reg(addr, value);

    if (rc == ESP_OK) {
        s_registers[addr] = value;
    }

    return rc;
}

static esp_err_t update_reg(uint8_t addr, uint8_t mask, uint8_t value)
{
    uint8_t next = (s_registers[addr] & (uint8_t)~mask) | (value & mask);

    return next == s_registers[addr] ? ESP_OK : write_reg(addr, next);
}

/* _locked helpers require s_lock. */
static esp_err_t imu_init_locked(void)
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

    rc = write_reg(CTRL3_C, 0x01);

    if (rc != ESP_OK) {
        return rc;
    }

    TickType_t start = xTaskGetTickCount();

    do {
        vTaskDelay(1);

        rc = _imu_t_read_reg(CTRL3_C, &value);

        if (rc != ESP_OK) {
            return rc;
        }

        if ((value & 0x01) == 0) {

            memset(s_registers, 0, sizeof(s_registers));

            /* BDU | IF_INC: consistent sample bytes and burst addressing. */
            rc = write_reg(CTRL3_C, 0x44);

            if (rc != ESP_OK) {
                return rc;
            }

            s_initialized = true;

            return ESP_OK;
        }

    } while ((TickType_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(100));

    return ESP_ERR_TIMEOUT;
}

static esp_err_t configure_xl_bandwidth(imu_xl_bandwidth_t bandwidth)
{
    uint8_t bw = bandwidth == IMU_XL_BW_AUTO ? 0 : (uint8_t)bandwidth - 1;
    esp_err_t rc = update_reg(CTRL1_XL, 0x03, bw);

    if (rc != ESP_OK) {
        return rc;
    }

    /* BW_XL takes effect only with XL_BW_SCAL_ODR set. */
    return update_reg(CTRL4_C, 0x80, bandwidth == IMU_XL_BW_AUTO ? 0 : 0x80);
}

static esp_err_t imu_configure_xl_locked(imu_odr_t odr, imu_xl_fs_t fs,
                                         imu_xl_bandwidth_t bandwidth)
{
    if ((unsigned)odr > IMU_ODR_6660_HZ || (unsigned)fs > IMU_XL_FS_8G ||
        (unsigned)bandwidth > IMU_XL_BW_50_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bw = bandwidth == IMU_XL_BW_AUTO ? 0 : (uint8_t)bandwidth - 1;
    esp_err_t rc = update_reg(CTRL1_XL, 0xff,
                             ((uint8_t)odr << 4) | ((uint8_t)fs << 2) | bw);

    if (rc != ESP_OK) {
        return rc;
    }

    return configure_xl_bandwidth(bandwidth);
}

static esp_err_t imu_configure_g_locked(imu_odr_t odr, imu_g_fs_t fs)
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

static esp_err_t imu_reboot_locked(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint8_t addresses[] = {CTRL1_XL, CTRL2_G, CTRL3_C, CTRL6_C, FIFO_CTRL5};
    uint8_t saved[sizeof(addresses)];

    for (size_t i = 0; i < sizeof(addresses); ++i) {
        saved[i] = s_registers[addresses[i]];
    }

    if (saved[2] & 0x81) {
        return ESP_ERR_INVALID_STATE;
    }

    s_initialized = false;

    esp_err_t rc = write_reg(FIFO_CTRL5, saved[4] & 0xf8);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = write_reg(CTRL2_G, saved[1] & 0x0f);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = write_reg(CTRL6_C, saved[3] & 0xef);

    if (rc != ESP_OK) {
        return rc;
    }

    /* BOOT requires the accelerometer running at 104 Hz. */
    rc = write_reg(CTRL1_XL, (saved[0] & 0x0f) | 0x40);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = write_reg(CTRL3_C, saved[2] | 0x80);

    if (rc != ESP_OK) {
        return rc;
    }

    TickType_t start = xTaskGetTickCount();

    vTaskDelay(pdMS_TO_TICKS(20));

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

    /* Restore sensors before restarting FIFO. */
    const size_t restore_order[] = {0, 3, 1, 4};

    for (size_t i = 0; i < sizeof(restore_order) / sizeof(restore_order[0]); ++i) {
        size_t index = restore_order[i];

        rc = write_reg(addresses[index], saved[index]);

        if (rc != ESP_OK) {
            return rc;
        }
    }

    s_registers[CTRL3_C] = saved[2];
    s_initialized = true;

    return ESP_OK;
}

static float axis(const uint8_t *bytes, float sensitivity)
{
    int16_t value = (int16_t)(
        (uint16_t)bytes[0] |
        ((uint16_t)bytes[1] << 8)
    );

    return value * sensitivity;
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

    *out = (imu_data_t) {
        .x = axis(data, sensitivity),
        .y = axis(data + 2, sensitivity),
        .z = axis(data + 4, sensitivity)
    };

    return ESP_OK;
}

static esp_err_t imu_read_xl_data_locked(imu_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ctrl = s_registers[CTRL1_XL];

    static const float scale[] = {0.000061f, 0.000488f, 0.000122f, 0.000244f};

    return read_scaled(OUT_XL, scale[(ctrl >> 2) & 3], out);
}

static esp_err_t imu_read_g_data_locked(imu_data_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t ctrl = s_registers[CTRL2_G];

    static const float scale[] = {0.00875f, 0.0175f, 0.035f, 0.070f};

    return read_scaled(OUT_G, (ctrl & 2) ? 0.004375f : scale[(ctrl >> 2) & 3], out);
}

static esp_err_t imu_set_fifo_threshold_locked(uint16_t words)
{
    esp_err_t rc;

    if (words > 4095) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    rc = write_reg(FIFO_CTRL1, (uint8_t)words);

    if (rc != ESP_OK) {
        return rc;
    }

    return update_reg(FIFO_CTRL2, 0x0f, (uint8_t)(words >> 8));
}

static esp_err_t imu_configure_fifo_locked(const imu_fifo_config_t *config)
{
    esp_err_t rc;

    if (config == NULL || config->threshold_words > 4095 ||
        (unsigned)config->odr > IMU_ODR_6660_HZ ||
        (unsigned)config->xl_decimation > IMU_FIFO_DECIMATE_32 ||
        (unsigned)config->g_decimation > IMU_FIFO_DECIMATE_32 ||
        (unsigned)config->xl_bandwidth > IMU_XL_BW_50_HZ ||
        (config->mode != IMU_FIFO_BYPASS && config->mode != IMU_FIFO_STOP_WHEN_FULL &&
         config->mode != IMU_FIFO_CONTINUOUS)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* AN4650 requires at least one undecimated FIFO dataset. */
    if (config->mode != IMU_FIFO_BYPASS &&
        (config->odr == IMU_ODR_OFF ||
         (config->xl_decimation != IMU_FIFO_NO_DECIMATION &&
          config->g_decimation != IMU_FIFO_NO_DECIMATION))) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config->mode != IMU_FIFO_BYPASS) {
        uint8_t xl = s_registers[CTRL1_XL], g = s_registers[CTRL2_G];

        if ((config->xl_decimation != IMU_FIFO_DISABLED &&
             ((xl >> 4) == 0 || (xl >> 4) > config->odr)) ||
            (config->g_decimation != IMU_FIFO_DISABLED &&
             ((g >> 4) == 0 || (g >> 4) > config->odr))) {
            return ESP_ERR_INVALID_STATE;
        }
    }

    rc = update_reg(FIFO_CTRL5, 0x7f, 0);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = configure_xl_bandwidth(config->xl_bandwidth);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = imu_set_fifo_threshold_locked(config->threshold_words);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = update_reg(FIFO_CTRL2, 0xc0, 0);

    if (rc != ESP_OK) {
        return rc;
    }

    rc = update_reg(FIFO_CTRL3, 0x3f,
        (uint8_t)config->xl_decimation | ((uint8_t)config->g_decimation << 3));

    if (rc != ESP_OK) {
        return rc;
    }

    /* 16-bit data; third and fourth datasets disabled. */
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

static esp_err_t imu_set_fifo_threshold_interrupt_locked(bool int1, bool int2)
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

/* Callbacks hold s_lock; reentering a public API from the worker would deadlock. */
esp_err_t imu_init(void)
{
    /* Initial mutex creation must be serialized by the caller. */
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();

        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_init_locked();
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_configure_xl(imu_odr_t odr, imu_xl_fs_t fs, imu_xl_bandwidth_t bandwidth)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_configure_xl_locked(odr, fs, bandwidth);
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_configure_g(imu_odr_t odr, imu_g_fs_t fs)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_configure_g_locked(odr, fs);
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_reboot(void)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_reboot_locked();
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_read_xl_data(imu_data_t *out)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_read_xl_data_locked(out);
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_read_g_data(imu_data_t *out)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_read_g_data_locked(out);
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_set_fifo_threshold(uint16_t words)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_set_fifo_threshold_locked(words);
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_configure_fifo(const imu_fifo_config_t *config)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_configure_fifo_locked(config);
    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_set_fifo_threshold_interrupt(bool int1, bool int2)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = s_streaming ? ESP_ERR_INVALID_STATE : imu_set_fifo_threshold_interrupt_locked(int1, int2);
    xSemaphoreGive(s_lock);

    return rc;
}

static void fifo_isr(void *arg)
{
    BaseType_t wake = pdFALSE;

    xEventGroupSetBitsFromISR(s_events, FIFO_READY, &wake);

    if (wake) {
        portYIELD_FROM_ISR();
    }
}

static esp_err_t fifo_restart(void)
{
    s_fifo_partial_words = 0;

    s_fifo_restart_pending = true;
    esp_err_t rc = write_reg(FIFO_CTRL5, s_stream_fifo_ctrl5 & 0xf8);

    if (rc == ESP_OK) {
        rc = write_reg(FIFO_CTRL5, s_stream_fifo_ctrl5);
    }

    if (rc == ESP_OK) {
        s_fifo_restart_pending = false;
    }

    return rc;
}

static esp_err_t read_fifo(void)
{
    if (s_fifo_restart_pending) {
        return fifo_restart();
    }

    uint8_t status[4];
    esp_err_t rc = _imu_t_read(FIFO_STATUS1, status, sizeof(status));

    if (rc != ESP_OK) {
        return rc;
    }

    const size_t fifo_words = status[0] | ((status[1] & 0x0f) << 8);
    const size_t fifo_bytes = fifo_words * 2;
    /* Next word in the Gx, Gy, Gz, Ax, Ay, Az pattern. */
    const unsigned pattern = status[2] | ((status[3] & 3) << 8);

    if (status[1] & 0x40) {
        ESP_LOGW("imu", "FIFO overrun: status=%02x %02x %02x %02x, words=%u, pattern=%u",
                 status[0], status[1], status[2], status[3], (unsigned)fifo_words, pattern);

        return fifo_restart();
    }

    if (fifo_words == 0) {
        return ESP_OK;
    }

    if (pattern >= 6) {
        s_fifo_partial_words = 0;
        ESP_LOGE("imu", "Invalid FIFO pattern: status=%02x %02x %02x %02x",
                 status[0], status[1], status[2], status[3]);

        return ESP_ERR_INVALID_RESPONSE;
    }

    /* FIFO addressing wraps 0x3f -> 0x3e during this burst. */
    rc = _imu_t_read(FIFO_DATA_OUT, s_fifo_raw, fifo_bytes);

    if (rc != ESP_OK) {
        s_fifo_partial_words = 0;

        return rc;
    }

    uint8_t status2;
    rc = _imu_t_read_reg(FIFO_STATUS1 + 1, &status2);

    if (rc != ESP_OK) {
        s_fifo_partial_words = 0;

        return rc;
    }

    if (status2 & 0x40) {
        ESP_LOGW("imu", "FIFO overrun during read; batch discarded");

        return fifo_restart();
    }

    static const float xl_scale[] = {0.000061f, 0.000488f, 0.000122f, 0.000244f};
    static const float g_scale[] = {0.00875f, 0.0175f, 0.035f, 0.070f};

    float xl = xl_scale[(s_registers[CTRL1_XL] >> 2) & 0x03];
    float g = (s_registers[CTRL2_G] & 0x02) ? 0.004375f :
              g_scale[(s_registers[CTRL2_G] >> 2) & 0x03];

    size_t count = 0;

    for (size_t i = 0; i < fifo_words; ++i) {
        unsigned position = (pattern + i) % 6;

        if (position == 0 || position != s_fifo_partial_words) {
            s_fifo_partial_words = 0;
        }

        if (position != s_fifo_partial_words) {
            /* Resynchronize at gyro X after lost alignment. */
            continue;
        }

        memcpy(s_fifo_partial + position * 2, s_fifo_raw + i * 2, 2);

        if (++s_fifo_partial_words == 6) {
            s_fifo_samples[count].g = (imu_data_t) {
                .x = axis(s_fifo_partial, g),
                .y = axis(s_fifo_partial + 2, g),
                .z = axis(s_fifo_partial + 4, g)
            };
            s_fifo_samples[count].xl = (imu_data_t) {
                .x = axis(s_fifo_partial + 6, xl),
                .y = axis(s_fifo_partial + 8, xl),
                .z = axis(s_fifo_partial + 10, xl)
            };

            ++count;
            s_fifo_partial_words = 0;
        }
    }

    /* Holding s_lock makes stop wait for the callback to finish. */
    if (count != 0) {
        s_callback(s_fifo_samples, count, s_callback_context);
    }

    return ESP_OK;
}

static void fifo_task(void *arg)
{
    for (;;) {
        EventBits_t events = xEventGroupWaitBits(s_events, FIFO_READY, pdTRUE, pdFALSE,
                                                pdMS_TO_TICKS(100) + 1);
        xSemaphoreTake(s_lock, portMAX_DELAY);

        /* SPI FIFO status reads require an interrupt; timeout recovery needs the pin high. */
        if (s_streaming && ((events & FIFO_READY) || gpio_get_level(s_interrupt_gpio))) {
            esp_err_t rc = read_fifo();

            if (rc != ESP_OK) {
                ESP_LOGW("imu", "FIFO read failed: %s", esp_err_to_name(rc));
            }
        }

        xSemaphoreGive(s_lock);
    }
}

static esp_err_t verify_stream_registers(void)
{
    static const struct {
        uint8_t first;
        uint8_t count;
    } ranges[] = {{FIFO_CTRL1, 9}, {CTRL1_XL, 4}};

    for (size_t i = 0; i < sizeof(ranges) / sizeof(ranges[0]); ++i) {
        uint8_t actual[9];
        esp_err_t rc = _imu_t_read(ranges[i].first, actual, ranges[i].count);

        if (rc != ESP_OK) {
            return rc;
        }

        for (unsigned j = 0; j < ranges[i].count; ++j) {
            uint8_t addr = ranges[i].first + j;

            /* Skip unmanaged ORIENT_CFG_G and reserved 0x0c. */
            if (addr == 0x0b || addr == 0x0c) {
                continue;
            }

            if (actual[j] != s_registers[addr]) {
                ESP_LOGE("imu", "Register 0x%02x: wrote 0x%02x, read 0x%02x",
                         addr, s_registers[addr], actual[j]);

                return ESP_ERR_INVALID_RESPONSE;
            }
        }
    }

    return ESP_OK;
}

esp_err_t imu_start_fifo(const imu_stream_config_t *config,
                         imu_fifo_callback_t callback, void *context)
{
    if (config == NULL || callback == NULL ||
        !GPIO_IS_VALID_GPIO(config->interrupt_gpio) ||
        config->interrupt_gpio == IMU_MISO_PIN || config->interrupt_gpio == IMU_MOSI_PIN ||
        config->interrupt_gpio == IMU_SCLK_PIN || config->interrupt_gpio == IMU_CS_PIN ||
        config->odr < IMU_ODR_12_5_HZ || config->odr > IMU_ODR_1660_HZ ||
        (unsigned)config->xl_fs > IMU_XL_FS_8G ||
        (unsigned)config->xl_bandwidth > IMU_XL_BW_50_HZ ||
        config->xl_decimation != IMU_FIFO_NO_DECIMATION ||
        config->g_decimation != IMU_FIFO_NO_DECIMATION ||
        (config->g_fs != IMU_G_FS_125DPS && config->g_fs != IMU_G_FS_245DPS &&
         config->g_fs != IMU_G_FS_500DPS && config->g_fs != IMU_G_FS_1000DPS &&
         config->g_fs != IMU_G_FS_2000DPS) ||
        config->watermark_samples == 0 || config->watermark_samples > 682)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_lock == NULL || (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task)) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t rc = ESP_ERR_INVALID_STATE;
    bool handler_added = false;

    if (!s_initialized || s_streaming) {
        goto done;
    }

    if (s_events == NULL) {
        s_events = xEventGroupCreate();

        if (s_events == NULL) {
            rc = ESP_ERR_NO_MEM;
            goto done;
        }
    }

    if (s_task == NULL && xTaskCreate(fifo_task, "imu_fifo", 4096, NULL, 5, &s_task) != pdPASS) {
        rc = ESP_ERR_NO_MEM;
        goto done;
    }

    rc = gpio_install_isr_service(0);

    if (rc != ESP_OK && rc != ESP_ERR_INVALID_STATE) {
        goto done;
    }

    gpio_config_t gpio = {
        .pin_bit_mask = 1ULL << config->interrupt_gpio,
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    rc = gpio_config(&gpio);

    if (rc != ESP_OK) {
        goto done;
    }

    xEventGroupClearBits(s_events, FIFO_READY);
    rc = gpio_isr_handler_add(config->interrupt_gpio, fifo_isr, NULL);

    if (rc != ESP_OK) {
        goto done;
    }

    handler_added = true;

    rc = update_reg(FIFO_CTRL5, 0x7f, 0);

    if (rc != ESP_OK) {
        goto fail;
    }

    rc = imu_configure_xl_locked(config->odr, config->xl_fs, config->xl_bandwidth);

    if (rc != ESP_OK) {
        goto fail;
    }

    rc = imu_configure_g_locked(config->odr, config->g_fs);

    if (rc != ESP_OK) {
        goto fail;
    }

    /* Paired rates plus the hardware constraint require both sources undecimated. */
    imu_fifo_config_t fifo = {
        .mode = IMU_FIFO_CONTINUOUS, .odr = config->odr,
        .xl_decimation = config->xl_decimation,
        .g_decimation = config->g_decimation,
        .xl_bandwidth = config->xl_bandwidth,
        .threshold_words = config->watermark_samples * 6,
    };
    rc = imu_configure_fifo_locked(&fifo);

    if (rc != ESP_OK) {
        goto fail;
    }

    rc = imu_set_fifo_threshold_interrupt_locked(!config->use_int2, config->use_int2);

    if (rc != ESP_OK) {
        goto fail;
    }

    rc = verify_stream_registers();

    if (rc != ESP_OK) {
        goto fail;
    }

    s_interrupt_gpio = config->interrupt_gpio;
    s_callback = callback;
    s_callback_context = context;
    s_stream_fifo_ctrl5 = s_registers[FIFO_CTRL5];
    s_fifo_restart_pending = false;
    s_fifo_partial_words = 0;
    s_streaming = true;

    if (gpio_get_level(s_interrupt_gpio)) {
        xEventGroupSetBits(s_events, FIFO_READY);
    }
    goto done;

fail:

    s_initialized = false;

done:
    if (rc != ESP_OK && handler_added) {
        gpio_intr_disable(config->interrupt_gpio);
        gpio_isr_handler_remove(config->interrupt_gpio);
    }

    xSemaphoreGive(s_lock);

    return rc;
}

esp_err_t imu_stop_fifo(void)
{
    if (s_lock == NULL || (s_task != NULL && xTaskGetCurrentTaskHandle() == s_task)) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    if (!s_streaming) {
        xSemaphoreGive(s_lock);

        return ESP_OK;
    }

    gpio_intr_disable(s_interrupt_gpio);
    gpio_isr_handler_remove(s_interrupt_gpio);
    s_streaming = false;
    s_callback = NULL;
    s_callback_context = NULL;

    esp_err_t rc = imu_set_fifo_threshold_interrupt_locked(false, false);
    esp_err_t fifo_rc = update_reg(FIFO_CTRL5, 0x7f, 0);

    if (rc == ESP_OK) {
        rc = fifo_rc;
    }

    if (rc != ESP_OK) {
        s_initialized = false;
    }

    xSemaphoreGive(s_lock);

    return rc;
}
