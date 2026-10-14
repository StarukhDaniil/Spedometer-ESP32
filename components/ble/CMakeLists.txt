idf_component_register(SRCS "ble_security.c" "ble_characteristics.c" "ble_gatt.c" "ble.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash freertos)
