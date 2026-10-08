#include <stdio.h>
#include "esp_err.h"
#include "imu.h"


void app_main(void)
{
    ESP_ERROR_CHECK(imu_init());
}
