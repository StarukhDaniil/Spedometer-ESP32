#include "ble.h"
#include "ble_gatt.h"
#include "ble_characteristics.h"

#include "services/gatt/ble_svc_gatt.h"

/* UUIDs: 7b6e0001/0002/0003-6b1a-4b90-8c2d-5f7a9e103246. */
#define CUSTOM_UUID(id) BLE_UUID128_INIT( \
    0x46, 0x32, 0x10, 0x9e, 0x7a, 0x5f, 0x2d, 0x8c, \
    0x90, 0x4b, 0x1a, 0x6b, (id), 0x00, 0x6e, 0x7b)

static const ble_uuid128_t service_uuid = CUSTOM_UUID(1);
const ble_uuid128_t BLE_TELEMETRY_UUID = CUSTOM_UUID(2);
const ble_uuid128_t BLE_CONFIG_UUID = CUSTOM_UUID(3);

static uint8_t telemetry_value[BLE_TELEMETRY_SIZE];
static uint8_t config_value = 0x00;

static struct ble_characteristic characteristics[] = {
    {
        .uuid = &BLE_TELEMETRY_UUID.u,
        .value = telemetry_value,
        .size = sizeof(telemetry_value),
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                 BLE_GATT_CHR_F_INDICATE | BLE_GATT_CHR_F_READ_ENC |
                 BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC,
    },
    {
        .uuid = &BLE_CONFIG_UUID.u,
        .value = &config_value,
        .size = sizeof(config_value),
        .write_event_bits = BLE_EVENT_CONFIG_WRITTEN,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                 BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC,
    },
};

#define CHARACTERISTIC_COUNT (sizeof(characteristics) / sizeof(characteristics[0]))

static struct ble_gatt_chr_def gatt_characteristics[CHARACTERISTIC_COUNT + 1];
static const struct ble_gatt_svc_def services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &service_uuid.u,
        .characteristics = gatt_characteristics,
    },
    {0},
};

int ble_gatt_init(void)
{
    for (size_t i = 0; i < CHARACTERISTIC_COUNT; ++i) {
        struct ble_characteristic *chr = &characteristics[i];

        gatt_characteristics[i] = (struct ble_gatt_chr_def) {
            .uuid = chr->uuid,
            .access_cb = ble_characteristic_access,
            .arg = chr,
            .flags = chr->flags,
            .min_key_size = 16,
            .val_handle = &chr->handle,
        };
    }

    ble_svc_gatt_init();
    
    int rc = ble_gatts_count_cfg(services);
    return rc == 0 ? ble_gatts_add_svcs(services) : rc;
}

const ble_uuid128_t *ble_gatt_service_uuid(void)
{
    return &service_uuid;
}

struct ble_characteristic *ble_gatt_find_characteristic(const ble_uuid_t *uuid)
{
    for (size_t i = 0; i < CHARACTERISTIC_COUNT; ++i) {
        if (ble_uuid_cmp(uuid, characteristics[i].uuid) == 0) {
            return &characteristics[i];
        }
    }
    return NULL;
}
