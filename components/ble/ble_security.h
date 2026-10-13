#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Private security API for the single active connection. */
void ble_security_init(void);
int ble_security_on_connect(uint16_t conn_handle);
/* Replace this peer's old bond when it requests pairing again. */
int ble_security_on_repeat_pairing(uint16_t conn_handle);
void ble_security_on_disconnect(void);
/* Application data requires current encryption with a 16-byte key, not a bond. */
bool ble_security_is_encrypted(void);
