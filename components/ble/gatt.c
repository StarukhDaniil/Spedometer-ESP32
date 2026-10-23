#include "gatt.h"

#include <stdint.h>

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "GATT";

static const ble_uuid128_t service_uuid = BLE_UUID128_INIT(
    0x01, 0x63, 0x47, 0x31, 0x82, 0x52, 0x49, 0x9A,
    0x93, 0x47, 0x21, 0x2D, 0x08, 0x51, 0xD2, 0xA7
);

static const ble_uuid128_t speed_uuid = BLE_UUID128_INIT(
    0x02, 0x63, 0x47, 0x31, 0x82, 0x52, 0x49, 0x9A,
    0x93, 0x47, 0x21, 0x2D, 0x08, 0x51, 0xD2, 0xA7
);

static uint16_t speed_handle;

static int characteristic_access(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg
) {
    int err;

    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        break;
    
    default:
        break;
    }

    err = BLE_ATT_ERR_UNLIKELY;
    return err;
}

static const struct ble_gatt_chr_def characteristics[] = {
    {
        .uuid = &speed_uuid.u,
        .access_cb = characteristic_access,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &speed_handle
    },
    {
        0
    }
};

static const struct ble_gatt_svc_def services[] = {
    {
        .uuid = &service_uuid.u,
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .characteristics = characteristics
    },
    {
        0
    }
};

int gatt_init(void) {
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(services);
    if (rc) {
        ESP_LOGE(TAG, "Failed to count BLE services %i", rc);
        return rc;
    }

    rc = ble_gatts_add_svcs(services);
    if (rc) {
        ESP_LOGE(TAG, "Failed to add BLE services %i", rc);
        return rc;
    }

    return rc;
}