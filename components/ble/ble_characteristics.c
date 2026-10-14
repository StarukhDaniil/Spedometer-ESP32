#include "ble.h"
#include "ble_characteristics.h"
#include "ble_gatt.h"
#include "ble_security.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "host/ble_hs.h"

static portMUX_TYPE value_lock = portMUX_INITIALIZER_UNLOCKED;
static StaticEventGroup_t event_group_storage;
static EventGroupHandle_t event_group;

void ble_characteristics_init(void)
{
    event_group = xEventGroupCreateStatic(&event_group_storage);
    configASSERT(event_group != NULL);
}

EventGroupHandle_t ble_get_event_group(void)
{
    return event_group;
}

int ble_characteristic_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    struct ble_characteristic *chr = arg;

    uint8_t snapshot[BLE_ATT_ATTR_MAX_LEN];
    if (chr == NULL || chr->size > sizeof(snapshot)) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (!ble_security_is_encrypted()) {
        return BLE_ATT_ERR_INSUFFICIENT_ENC;
    }

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (!(chr->flags & BLE_GATT_CHR_F_READ)) {
            return BLE_ATT_ERR_READ_NOT_PERMITTED;
        }

        portENTER_CRITICAL(&value_lock);
        memcpy(snapshot, chr->value, chr->size);
        portEXIT_CRITICAL(&value_lock);

        return os_mbuf_append(ctxt->om, snapshot, chr->size) == 0
                   ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (!(chr->flags & BLE_GATT_CHR_F_WRITE)) {
            return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
        }
        
        if (OS_MBUF_PKTLEN(ctxt->om) != chr->size) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        
        if (os_mbuf_copydata(ctxt->om, 0, chr->size, snapshot) != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        
        portENTER_CRITICAL(&value_lock);
        memcpy(chr->value, snapshot, chr->size);
        portEXIT_CRITICAL(&value_lock);

        /* Signal only after committing the value and releasing its lock:
         * awakened application tasks may immediately call ble_get_value(). */
        if (chr->write_event_bits != 0) {
            xEventGroupSetBits(event_group, chr->write_event_bits);
        }
        
        return 0;
    
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}

esp_err_t ble_set_value(const ble_uuid_t *uuid, const void *value, size_t size)
{
    if (uuid == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct ble_characteristic *chr = ble_gatt_find_characteristic(uuid);
    if (chr == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    if (size != chr->size) {
        return ESP_ERR_INVALID_SIZE;
    }

    portENTER_CRITICAL(&value_lock);
    memcpy(chr->value, value, size);
    portEXIT_CRITICAL(&value_lock);
    
    if ((chr->flags & (BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE)) &&
        ble_hs_synced() && chr->handle != 0 && ble_security_is_encrypted()) {
        ble_gatts_chr_updated(chr->handle);
    }
    
    return ESP_OK;
}

esp_err_t ble_get_value(const ble_uuid_t *uuid, void *value, size_t size)
{
    if (uuid == NULL || value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct ble_characteristic *chr = ble_gatt_find_characteristic(uuid);
    if (chr == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    
    if (size != chr->size) {
        return ESP_ERR_INVALID_SIZE;
    }

    portENTER_CRITICAL(&value_lock);
    memcpy(value, chr->value, size);
    portEXIT_CRITICAL(&value_lock);
    
    return ESP_OK;
}
