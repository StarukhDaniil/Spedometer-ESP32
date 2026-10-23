idf_component_register(SRCS
                    "ble.c"
                    "gap.c"
                    "gatt.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES
                         bt
                         nvs_flash
                         freertos
)
