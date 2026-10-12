#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "host/ble_uuid.h"

#define BLE_TELEMETRY_SIZE 12

extern const ble_uuid128_t BLE_TELEMETRY_UUID;
extern const ble_uuid128_t BLE_CONFIG_UUID;

/* Call once at startup; advertises with the NimBLE GAP name from menuconfig. */
void ble_init(void);

/* Copy a local characteristic value after ble_init(), from task context.
 * UUIDs are compared by value. Size must match the profile (telemetry: BLE_TELEMETRY_SIZE,
 * config: 1). Local updates are allowed even for client-read-only values.
 * Notify/indicate characteristics publish to encrypted subscribers; bonding
 * is optional. Updates may coalesce.
 * ESP_OK means stored, not confirmed delivered. Returns ESP_ERR_INVALID_ARG
 * for NULL input, ESP_ERR_NOT_FOUND for an unknown UUID, or
 * ESP_ERR_INVALID_SIZE for a size mismatch. */
esp_err_t ble_set_value(const ble_uuid_t *uuid, const void *value, size_t size);

/* Copy a snapshot of a local characteristic into value, from task context.
 * Size must exactly match the profile, as with ble_set_value(). No BLE
 * connection is required. Returns ESP_ERR_INVALID_ARG for NULL input,
 * ESP_ERR_NOT_FOUND for an unknown UUID, or ESP_ERR_INVALID_SIZE for a
 * size mismatch. The output buffer is unchanged on error. */
esp_err_t ble_get_value(const ble_uuid_t *uuid, void *value, size_t size);
