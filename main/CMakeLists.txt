idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES 
                        bt
                        ble
                        nvs_flash
)
