#include "host/ble_gatt.h"

static uint8_t value = 42;

static int characteristicAccess(
    uint16_t connHandle,
    uint16_t attrHandle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        os_mbuf_append(ctxt->om, &value, sizeof(value));
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (OS_MBUF_PKTLEN(ctxt->om) == 1) {
            value = ctxt->om->om_data[0];
            printf("Received: %d\n", value);
        }

        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}