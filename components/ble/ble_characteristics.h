#pragma once

#include <stddef.h>
#include <stdint.h>
#include "host/ble_gatt.h"

/* Profile metadata and storage, shared with the generic value handler. */
struct ble_characteristic {
    const ble_uuid_t *uuid;
    uint8_t *value;
    size_t size;
    ble_gatt_chr_flags flags;
    uint16_t handle;
};

int ble_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);
