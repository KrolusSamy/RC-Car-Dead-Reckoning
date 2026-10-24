#include "APP_Orchestrator_Interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "--- Tele-Operated 2D Mapping Robot ---");
    ESP_LOGI(TAG, "--- Firmware version is reported by APP_ORCHESTRATOR below ---");

    ErrorState_TypeDef result = APP_Orchestrator_Init();
    if (result != FUNCTION_OK) {
        ESP_LOGE(TAG, "Fatal Error: Orchestrator initialization returned ErrorState=%d", (int)result);
        vTaskDelete(NULL); // Suspend execution on critical failure
    }

    result = APP_Orchestrator_StartTasks();
    if (result != FUNCTION_OK) {
        ESP_LOGE(TAG, "Fatal Error: Task startup returned ErrorState=%d", (int)result);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "System handed over to FreeRTOS Scheduler.");
    
    /* Clean up the setup task to reclaim memory */
    vTaskDelete(NULL);
}
