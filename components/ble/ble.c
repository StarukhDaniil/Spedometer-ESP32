#include "ble.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "freertos/FreeRTOS.h"

#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

#include "gap.h"
#include "gatt.h"

static const char *TAG = "BLE";

static void on_sync(void) {
    adv_init();

    advertise();
}

static void nimble_host_task(void *arg) {
    nimble_port_run();

    vTaskDelete(NULL);
}

esp_err_t ble_init(void) {
    ble_hs_cfg.sync_cb = on_sync;

    esp_err_t err = gap_init();
    if (err) {
        return err;
    }

    err = gatt_init();
    if (err) {
        return err;
    }

    BaseType_t ret = xTaskCreate(
        nimble_host_task,
        "NimBLE Host",
        4096,
        NULL,
        5,
        NULL
    );
    if (ret != pdPASS) {
        return ESP_FAIL;
    }

    return ESP_OK;
}