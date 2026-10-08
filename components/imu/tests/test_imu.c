#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "imu.h"
#include "imu_transport.h"
#include "freertos/task.h"

static uint8_t regs[128];
static unsigned writes, transport_inits;
static TickType_t ticks;
static bool stuck_reset;
static bool stuck_boot;
static TickType_t boot_started;
static unsigned boots;
static int fail_addr = -1;
static int fail_write_addr = -1;

TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t n) { ticks += n; }
esp_err_t _imu_t_init(void) { ++transport_inits; return ESP_OK; }
esp_err_t _imu_t_write_reg(uint8_t addr, uint8_t value)
{
    if (addr == fail_write_addr) return ESP_ERR_TIMEOUT;
    ++writes;
    regs[addr] = value;

    if (addr == 0x12 && (value & 0x80)) {
        assert((value & 1) == 0);
        assert((regs[0x11] & 0xf0) == 0);
        assert((regs[0x15] & 0x10) == 0);
        assert((regs[0x10] & 0xf0) != 0);
        assert((regs[0x0a] & 7) == 0);
        boot_started = ticks;
        ++boots;
    }

    if (addr == 0x12 && (value & 1) && !stuck_reset) regs[addr] = 0;
    return ESP_OK;
}
esp_err_t _imu_t_read(uint8_t addr, uint8_t *out, size_t size)
{
    if (regs[0x12] & 0x80) {
        assert(ticks - boot_started >= 20);

        if (!stuck_boot) {
            regs[0x12] &= 0x7f;
        }
    }

    if (addr == fail_addr) return ESP_ERR_TIMEOUT;
    assert(addr + size <= sizeof(regs));
    memcpy(out, regs + addr, size);
    return ESP_OK;
}
esp_err_t _imu_t_read_reg(uint8_t addr, uint8_t *out)
{
    return _imu_t_read(addr, out, 1);
}

int main(void)
{
    imu_data_t data = {1, 2, 3};
    assert(imu_reboot() == ESP_ERR_INVALID_STATE);
    assert(imu_configure_xl(IMU_ODR_104_HZ, IMU_XL_FS_4G) == ESP_ERR_INVALID_STATE);
    assert(imu_init() == ESP_ERR_NOT_FOUND);
    regs[0x0f] = 0x69;
    stuck_reset = true;
    assert(imu_init() == ESP_ERR_TIMEOUT);
    assert(ticks >= 100);
    stuck_reset = false;
    assert(imu_init() == ESP_OK);
    assert(regs[0x12] == 0x44);
    unsigned before = transport_inits;
    assert(imu_init() == ESP_OK && transport_inits == before);

    regs[0x10] = 3; /* Preserve bandwidth bits. */
    const imu_xl_fs_t xl_fs[] = {IMU_XL_FS_2G, IMU_XL_FS_4G, IMU_XL_FS_8G, IMU_XL_FS_16G};
    const uint8_t xl_bits[] = {0, 8, 12, 4};
    const float xl_scale[] = {0.000061f, 0.000122f, 0.000244f, 0.000488f};
    const uint8_t samples[] = {0xff, 0x7f, 0x00, 0x80, 0xff, 0xff};
    memcpy(regs + 0x28, samples, 6);
    memcpy(regs + 0x22, samples, 6);
    for (unsigned i = 0; i < 4; ++i) {
        assert(imu_configure_xl(IMU_ODR_104_HZ, xl_fs[i]) == ESP_OK);
        assert(regs[0x10] == (0x43 | xl_bits[i]));
        assert(imu_read_xl_data(&data) == ESP_OK);
        assert(fabsf(data.x - 32767 * xl_scale[i]) < 0.00001f);
        assert(fabsf(data.y + 32768 * xl_scale[i]) < 0.00001f);
        assert(fabsf(data.z + xl_scale[i]) < 0.000001f);
    }
    const imu_g_fs_t g_fs[] = {IMU_G_FS_125DPS, IMU_G_FS_245DPS, IMU_G_FS_500DPS,
                               IMU_G_FS_1000DPS, IMU_G_FS_2000DPS};
    const uint8_t g_bits[] = {2, 0, 4, 8, 12};
    const float g_scale[] = {0.004375f, 0.00875f, 0.0175f, 0.035f, 0.070f};
    for (unsigned i = 0; i < 5; ++i) {
        assert(imu_configure_g(IMU_ODR_104_HZ, g_fs[i]) == ESP_OK);
        assert(regs[0x11] == (0x40 | g_bits[i]));
        assert(imu_read_g_data(&data) == ESP_OK);
        assert(fabsf(data.y + 32768 * g_scale[i]) < 0.0001f);
    }
    before = writes;
    assert(imu_configure_g(IMU_ODR_3330_HZ, IMU_G_FS_245DPS) == ESP_ERR_INVALID_ARG);
    assert(imu_configure_xl((imu_odr_t)-1, IMU_XL_FS_2G) == ESP_ERR_INVALID_ARG);
    assert(imu_configure_g(IMU_ODR_104_HZ, (imu_g_fs_t)3) == ESP_ERR_INVALID_ARG);
    assert(imu_read_xl_data(NULL) == ESP_ERR_INVALID_ARG);
    assert(imu_set_fifo_threshold(4096) == ESP_ERR_INVALID_ARG);
    assert(writes == before);

    regs[0x07] = 0xc0;
    assert(imu_set_fifo_threshold(4095) == ESP_OK);
    assert(regs[0x06] == 0xff && regs[0x07] == 0xcf);
    assert(imu_set_fifo_threshold(0) == ESP_OK);
    assert(regs[0x06] == 0 && regs[0x07] == 0xc0);
    assert(imu_set_fifo_threshold(256) == ESP_OK);
    assert(regs[0x06] == 0 && regs[0x07] == 0xc1);
    imu_fifo_config_t fifo = {
        .mode = IMU_FIFO_CONTINUOUS, .odr = IMU_ODR_104_HZ,
        .xl_decimation = IMU_FIFO_NO_DECIMATION,
        .g_decimation = IMU_FIFO_DECIMATE_2,
        .threshold_words = 300, .stop_on_threshold = true,
    };
    regs[0x13] = 0x34;
    regs[0x09] = 0x7f;
    assert(imu_configure_fifo(&fifo) == ESP_OK);
    assert(regs[0x06] == 0x2c && regs[0x07] == 1);
    assert(regs[0x08] == 0x11 && regs[0x09] == 0 && regs[0x0a] == 0x26);
    assert(regs[0x13] == 0x25); /* Stop bit set, temperature cleared, others kept. */
    regs[0x0d] = 0x81;
    regs[0x0e] = 0xff;
    assert(imu_set_fifo_threshold_interrupt(true, false) == ESP_OK);
    assert(regs[0x0d] == 0x89 && regs[0x0e] == 0xf7);

    before = writes;
    fifo.odr = IMU_ODR_OFF;
    assert(imu_configure_fifo(&fifo) == ESP_ERR_INVALID_ARG);
    fifo.odr = IMU_ODR_52_HZ;
    assert(imu_configure_fifo(&fifo) == ESP_ERR_INVALID_STATE);
    assert(writes == before);
    fifo.odr = IMU_ODR_104_HZ;
    fail_write_addr = 0x08;
    assert(imu_configure_fifo(&fifo) == ESP_ERR_TIMEOUT);
    assert(regs[0x0a] == 0); /* Failure does not restart FIFO with partial settings. */
    fail_write_addr = -1;
    assert(imu_configure_fifo(&fifo) == ESP_OK);

    data = (imu_data_t){1, 2, 3};
    fail_addr = 0x28;
    assert(imu_read_xl_data(&data) == ESP_ERR_TIMEOUT);
    assert(data.x == 1 && data.y == 2 && data.z == 3);
    fail_addr = -1;
    assert(_imu_c_read_xl_data(&data, 8) == ESP_OK);
    assert(fabsf(data.y + 15.990784f) < 0.00001f);
    assert(_imu_c_read_g_data(&data, 16) == ESP_ERR_INVALID_ARG);
    regs[0x15] = 0xb0;
    uint8_t snapshot[sizeof(regs)];
    memcpy(snapshot, regs, sizeof(regs));

    assert(imu_reboot() == ESP_OK);
    assert(boots == 1);
    assert(ticks - boot_started >= 20);
    assert(memcmp(snapshot, regs, sizeof(regs)) == 0);

    /* Also reboot when both sensors are powered down. */
    assert(imu_configure_xl(IMU_ODR_OFF, IMU_XL_FS_2G) == ESP_OK);
    assert(imu_configure_g(IMU_ODR_OFF, IMU_G_FS_245DPS) == ESP_OK);
    assert(imu_reboot() == ESP_OK);
    assert((regs[0x10] & 0xf0) == 0 && (regs[0x11] & 0xf0) == 0);

    fail_write_addr = 0x12;
    assert(imu_reboot() == ESP_ERR_TIMEOUT);
    assert(imu_read_xl_data(&data) == ESP_ERR_INVALID_STATE);
    fail_write_addr = -1;
    assert(imu_init() == ESP_OK);

    stuck_boot = true;
    assert(imu_reboot() == ESP_ERR_TIMEOUT);
    assert(ticks - boot_started >= 100);
    assert(imu_read_xl_data(&data) == ESP_ERR_INVALID_STATE);
    stuck_boot = false;
    assert(imu_init() == ESP_OK);
    assert(imu_reboot() == ESP_OK);

    puts("IMU register and reboot tests passed");
}
