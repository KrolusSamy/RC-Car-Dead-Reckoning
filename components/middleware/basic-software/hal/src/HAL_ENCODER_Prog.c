/*-----------------------------------------------------------------------------------------------------*-
 * File Name          : HAL_ENCODER_Prog.c
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : Implementation of Encoder tracking via GPIO hardware interrupts.                                          
--------------------------------------------------------------------------------------------------------
-See HAL_ENCODER_Interface.h and HAL_ENCODER_Cfg.h for details
-*-----------------------------------------------------------------------------------------------------*/

#include "HAL_ENCODER_Interface.h"
#include "HAL_ENCODER_Cfg.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

static const char *TAG = "HAL_ENCODER";

// Volatile keyword ensures variables changed inside ISRs are correctly read by main tasks
static volatile uint32_t left_encoder_ticks = 0;
static volatile uint32_t right_encoder_ticks = 0;

// FreeRTOS ISR flag
static bool isr_service_installed = false;

/* Interrupt Service Routines (ISRs) - Kept extremely minimal for deterministic execution */
static void IRAM_ATTR encoder_left_isr_handler(void* arg) {
    left_encoder_ticks++;
}

static void IRAM_ATTR encoder_right_isr_handler(void* arg) {
    right_encoder_ticks++;
}

ErrorState_TypeDef HAL_ENCODER_Init(void)
{
    ESP_LOGI(TAG, "Initializing Optical Encoders...");

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Trigger on rising edge (slot detected)
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<HAL_ENCODER_LEFT_PIN) | (1ULL<<HAL_ENCODER_RIGHT_PIN),
        .pull_up_en = 1, // Enable internal pull-ups for stable signal
        .pull_down_en = 0
    };
    gpio_config(&io_conf);

    if (!isr_service_installed) {
        gpio_install_isr_service(0);
        isr_service_installed = true;
    }

    gpio_isr_handler_add(HAL_ENCODER_LEFT_PIN, encoder_left_isr_handler, (void*) HAL_ENCODER_LEFT_PIN);
    gpio_isr_handler_add(HAL_ENCODER_RIGHT_PIN, encoder_right_isr_handler, (void*) HAL_ENCODER_RIGHT_PIN);

    return FUNCTION_OK;
}

ErrorState_TypeDef HAL_ENCODER_GetTicks(uint32_t* left_ticks, uint32_t* right_ticks)
{
    if (left_ticks == NULL || right_ticks == NULL) return FUNCTION_ERROR;
    
    *left_ticks = left_encoder_ticks;
    *right_ticks = right_encoder_ticks;
    
    return FUNCTION_OK;
}

ErrorState_TypeDef HAL_ENCODER_ResetTicks(void)
{
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    return FUNCTION_OK;
}