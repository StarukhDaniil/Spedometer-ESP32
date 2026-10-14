#include "ble_security.h"

#include <stdatomic.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"

#if !CONFIG_BT_NIMBLE_NVS_PERSIST
#error "Enable CONFIG_BT_NIMBLE_NVS_PERSIST to retain BLE bonds"
#endif

_Static_assert(CONFIG_BT_NIMBLE_MAX_CONNECTIONS == 1,
               "This BLE component requires exactly one connection");

void ble_store_config_init(void);

static atomic_uint_least16_t connection = BLE_HS_CONN_HANDLE_NONE;

void ble_security_init(void)
{
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_sc_only = 0;
    ble_hs_cfg.sm_sec_lvl = 2;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_store_config_init();
}

int ble_security_on_connect(uint16_t conn_handle)
{
    atomic_store(&connection, conn_handle);
    ESP_LOGI("BLE", "Connected: handle=%u; requesting security now", conn_handle);
    int rc = ble_gap_security_initiate(conn_handle);
    if (rc == 0) {
        ESP_LOGI("BLE", "Security request sent: handle=%u", conn_handle);
    } else if (rc == BLE_HS_EALREADY) {
        ESP_LOGI("BLE", "Security procedure already running: handle=%u", conn_handle);
    }
    return rc == BLE_HS_EALREADY ? 0 : rc;
}

void ble_security_on_disconnect(void)
{
    atomic_store(&connection, BLE_HS_CONN_HANDLE_NONE);
}

int ble_security_on_repeat_pairing(uint16_t conn_handle)
{
    struct ble_gap_conn_desc desc;

    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc != 0) {
        ESP_LOGE("BLE", "Cannot find peer for repeat pairing: %d", rc);
        return BLE_GAP_REPEAT_PAIRING_IGNORE;
    }

    rc = ble_store_util_delete_peer(&desc.peer_id_addr);
    if (rc != 0) {
        ESP_LOGE("BLE", "Cannot delete old bond for repeat pairing: %d", rc);
        return BLE_GAP_REPEAT_PAIRING_IGNORE;
    }

    ESP_LOGI("BLE", "Old bond removed; retrying pairing");
    return BLE_GAP_REPEAT_PAIRING_RETRY;
}

bool ble_security_is_encrypted(void)
{
    uint16_t handle = atomic_load(&connection);

    struct ble_gap_conn_desc desc;
    
    return handle != BLE_HS_CONN_HANDLE_NONE &&
           ble_gap_conn_find(handle, &desc) == 0 &&
           desc.sec_state.encrypted && desc.sec_state.key_size == 16;
}

