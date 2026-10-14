#pragma once

#include "host/ble_uuid.h"

struct ble_characteristic;

int ble_gatt_init(void);
const ble_uuid128_t *ble_gatt_service_uuid(void);
struct ble_characteristic *ble_gatt_find_characteristic(const ble_uuid_t *uuid);
