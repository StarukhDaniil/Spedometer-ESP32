#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "host/ble_uuid.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define BLE_TELEMETRY_SIZE 12

#define BLE_EVENT_CONFIG_WRITTEN ((EventBits_t)1U << 0)

extern const ble_uuid128_t BLE_TELEMETRY_UUID;
extern const ble_uuid128_t BLE_CONFIG_UUID;

/* Call once at startup; advertises with the NimBLE GAP name from menuconfig. */
void ble_init(void);

/* Available after ble_init(); owned by BLE, so do not delete or set its bits.
 * BLE_EVENT_CONFIG_WRITTEN is set after each accepted client config write,
 * including same-value writes. Local ble_set_value() calls do not set it.
 * Wait with xEventGroupWaitBits(..., pdTRUE, pdFALSE, ...) to clear on exit.
 * All tasks already waiting on the bit are released. Events can coalesce;
 * use ble_get_value() to read the latest value, not a per-write payload. */
EventGroupHandle_t ble_get_event_group(void);

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
