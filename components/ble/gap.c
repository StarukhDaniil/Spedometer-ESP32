#include "gap.h"

#include <string.h>

#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/ble_hs_id.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#define DEVICE_NAME         "Speedometer ESP32"

static const char *TAG = "GAP";

static uint8_t own_addr_type;

void advertise(void);

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGE(TAG, "Connection failed: %i", event->connect.status);
            advertise();
        }
        else {
            ESP_LOGI(TAG, "Connected: handle=%u", event->connect.conn_handle);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected: handle=%u reason=%i",
            event->disconnect.conn.conn_handle,
            event->disconnect.reason
        );
        advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "Subscription: handle=%u attr=%u notify=%u indicate=%u",
            event->subscribe.conn_handle,
            event->subscribe.attr_handle,
            event->subscribe.cur_notify,
            event->subscribe.cur_indicate
        );
        return 0;
    
    default:
        return 0;
    }
}

void advertise(void) {
    const char *name = ble_svc_gap_device_name();

    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (const uint8_t *)name,
        .name_len = strlen(name),
        .name_is_complete = 1
    };

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc) {
        ESP_LOGE(TAG, "Advertising data failed: %i", rc);
        return;
    }

    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN
    };

    rc = ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &params,
        gap_event,
        NULL
    );
    if (rc) {
        ESP_LOGE(TAG, "Advertising start failed: %i", rc);
    }
}

void adv_init(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &own_addr_type);
    }

    if (rc) {
        ESP_LOGE(TAG, "Address setup failed: %i", rc);
        return;
    }
}

int gap_init(void) {
    ble_svc_gap_init();

    return ble_svc_gap_device_name_set(DEVICE_NAME);
}
