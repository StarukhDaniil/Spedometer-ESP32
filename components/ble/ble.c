#include "ble.h"
#include "ble_gatt.h"
#include "ble_security.h"
#include "ble_characteristics.h"

#include <string.h>
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs_flash.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "BLE";
static uint8_t own_addr_type;
static uint16_t connection_handle = BLE_HS_CONN_HANDLE_NONE;

static int gap_event(struct ble_gap_event *event, void *arg);

static void advertise(void)
{
    if (connection_handle != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active()) {
        return;
    }

    struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .uuids128 = ble_gatt_service_uuid(),
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Setting advertisement failed: %d", rc);
        return;
    }

    const char *name = ble_svc_gap_device_name();
    struct ble_hs_adv_fields response = {
        .name = (const uint8_t *)name,
        .name_len = strlen(name),
        .name_is_complete = 1,
    };

    rc = ble_gap_adv_rsp_set_fields(&response);
    if (rc != 0) {
        ESP_LOGE(TAG, "Setting scan response failed: %d", rc);
        return;
    }

    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &params, gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Starting advertising failed: %d", rc);
    }
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            advertise();
            break;
        }

        connection_handle = event->connect.conn_handle;
        
        int rc = ble_security_on_connect(connection_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Starting security failed: %d", rc);
            ble_gap_terminate(connection_handle, BLE_ERR_REM_USER_CONN_TERM);
        }

        break;
    
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected: reason=%d", event->disconnect.reason);
        connection_handle = BLE_HS_CONN_HANDLE_NONE;
        
        ble_security_on_disconnect();
        advertise();
        
        break;
    
    case BLE_GAP_EVENT_ENC_CHANGE: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
            ESP_LOGI(TAG, "Security changed: status=%d encrypted=%u authenticated=%u "
                     "bonded=%u key_size=%u", event->enc_change.status,
                     (unsigned)desc.sec_state.encrypted,
                     (unsigned)desc.sec_state.authenticated,
                     (unsigned)desc.sec_state.bonded,
                     (unsigned)desc.sec_state.key_size);
        }
        if (event->enc_change.status != 0 || !ble_security_is_encrypted()) {
            ESP_LOGW(TAG, "Encryption failed or insufficient: %d",
                     event->enc_change.status);

            ble_security_on_disconnect();
            ble_gap_terminate(event->enc_change.conn_handle,
                              BLE_ERR_REM_USER_CONN_TERM);
        }

        break;
    }
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        return ble_security_on_repeat_pairing(event->repeat_pairing.conn_handle);
    
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        
        break;
    
    default:
        break;
    }
    return 0;
}

static void on_reset(int reason)
{
    connection_handle = BLE_HS_CONN_HANDLE_NONE;
    ble_security_on_disconnect();
    ESP_LOGW(TAG, "Host reset: %d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) {
        rc = ble_hs_id_infer_auto(0, &own_addr_type);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE address initialization failed: %d", rc);
        return;
    }
    advertise();
}

static void host_task(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_init(void)
{
    ble_characteristics_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_security_init();
    ble_svc_gap_init();
    ESP_ERROR_CHECK(ble_gatt_init());
    nimble_port_freertos_init(host_task);
}
