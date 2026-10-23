#include <stdlib.h>

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "ble.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    esp_err_t err;
    
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize flas: %s", esp_err_to_name(err));
        return;
    }

    err = nimble_port_init();
    if (err) {
        ESP_LOGE(TAG, "Failed to initialize nimble stack: %s", esp_err_to_name(err));
        return;
    }

    ESP_ERROR_CHECK(ble_init());
}
