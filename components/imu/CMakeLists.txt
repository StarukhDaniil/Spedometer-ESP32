idf_component_register(SRCS "imu_transport.c" "imu.c"
                    INCLUDE_DIRS "include"
                    REQUIRES
                        esp_driver_spi
                        esp_driver_gpio
                        freertos
)
