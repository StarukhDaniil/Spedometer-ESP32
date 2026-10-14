#pragma once

#include <stdbool.h>
#include <stdint.h>

void ble_security_init(void);
int ble_security_on_connect(uint16_t conn_handle);
int ble_security_on_repeat_pairing(uint16_t conn_handle);
void ble_security_on_disconnect(void);
bool ble_security_is_encrypted(void);
