/*-----------------------------------------------------------------------------------------------------*-
 * File Name          : HAL_LED_Prog.c
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : Implementation of the WS2812 LED driver using the RMT peripheral.                                          
--------------------------------------------------------------------------------------------------------
-See HAL_LED_Interface.h and HAL_LED_Cfg.h for details
-*-----------------------------------------------------------------------------------------------------*/

// --------------------------------- Includes -----------------------------------------------------
#include "HAL_LED_Interface.h"
#include "HAL_LED_Cfg.h"
#include "led_strip.h"
#include "esp_log.h"

// --------------------------------- Private variables definition ---------------------------------
static led_strip_handle_t led_strip;
static const char *TAG = "HAL_LED";

// --------------------------------  Public/Exported functions definitions ------------------------

ErrorState_TypeDef HAL_LED_Init(void)
{
    ESP_LOGI(TAG, "Initializing WS2812 LED on GPIO %d", HAL_LED_WS2812_PIN);

    /* LED strip initialization with the RMT peripheral */
    led_strip_config_t strip_config = {
        .strip_gpio_num = HAL_LED_WS2812_PIN,
        .max_leds = HAL_LED_WS2812_NUM, 
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed");
        return FUNCTION_ERROR;
    }

    led_strip_clear(led_strip);
    return FUNCTION_OK;
}

ErrorState_TypeDef HAL_LED_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    if (led_strip == NULL) {
        return FUNCTION_COMMUNICATION_ERROR;
    }

    esp_err_t err = led_strip_set_pixel(led_strip, 0, red, green, blue);
    err |= led_strip_refresh(led_strip);

    if (err != ESP_OK) {
        return FUNCTION_ERROR;
    }

    return FUNCTION_OK;
}