#pragma once

#include <stddef.h>
#include <stdint.h>
#include "host/ble_gatt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

struct ble_characteristic {
    const ble_uuid_t *uuid;
    uint8_t *value;
    size_t size;
    ble_gatt_chr_flags flags;
    uint16_t handle;
    EventBits_t write_event_bits;
};

/* Initialize once before starting the BLE host or application listeners. */
void ble_characteristics_init(void);

int ble_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);
