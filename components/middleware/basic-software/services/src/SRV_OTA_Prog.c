#include "SRV_OTA_Interface.h"
#include "SRV_OTA_Cfg.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_log.h"
#include "HAL_LED_Interface.h"

static const char *TAG = "SRV_OTA";

ErrorState_TypeDef SRV_OTA_PerformUpdate(void)
{
    ESP_LOGI(TAG, "Starting OTA Update from %s", SRV_OTA_FIRMWARE_URL);
    HAL_LED_SetColor(0, 255, 0); // Solid Green indicates OTA in progress

    esp_http_client_config_t config = {
        .url = SRV_OTA_FIRMWARE_URL,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (esp_http_client_open(client, 0) != ESP_OK) {
        esp_http_client_cleanup(client);
        return FUNCTION_COMMUNICATION_ERROR;
    }
    esp_http_client_fetch_headers(client);

    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle) != ESP_OK) {
        esp_http_client_cleanup(client);
        return FUNCTION_ERROR;
    }

    char buffer[1024];
    int data_read;
    while ((data_read = esp_http_client_read(client, buffer, sizeof(buffer))) > 0) {
        esp_ota_write(update_handle, (const void *)buffer, data_read);
    }

    esp_ota_end(update_handle);
    esp_ota_set_boot_partition(update_partition);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "OTA Success. Rebooting...");
    esp_restart();
    return FUNCTION_OK;
}