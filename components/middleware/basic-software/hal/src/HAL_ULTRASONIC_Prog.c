#include "HAL_ULTRASONIC_Interface.h"
#include "HAL_ULTRASONIC_Cfg.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "HAL_ULTRASONIC";

ErrorState_TypeDef HAL_Ultrasonic_Init(void)
{
    // Configure Trigger pin as output
    gpio_config_t io_conf_trig = {
        .pin_bit_mask = (1ULL << ULTRASONIC_TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_trig);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    // Configure Echo pin as input
    gpio_config_t io_conf_echo = {
        .pin_bit_mask = (1ULL << ULTRASONIC_ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_echo);

    ESP_LOGI(TAG, "Ultrasonic Sensor Initialized on Trig:%d, Echo:%d", ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    return FUNCTION_OK;
}

ErrorState_TypeDef HAL_Ultrasonic_GetDistance(float *distance_cm)
{
    if (distance_cm == NULL) return FUNCTION_ERROR;

    // 1. Send 10us high pulse to Trigger
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    // 2. Wait for Echo pin to go HIGH (with 30ms timeout)
    int64_t start_timeout = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - start_timeout) > 30000) {
            return FUNCTION_ERROR; // Timeout: No obstacle detected or sensor missing
        }
    }

    // 3. Measure duration while Echo pin stays HIGH
    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - pulse_start) > 30000) {
            return FUNCTION_ERROR; // Timeout: Echo too long
        }
    }
    int64_t pulse_end = esp_timer_get_time();

    int64_t pulse_duration = pulse_end - pulse_start;

    // 4. Calculate distance in cm 
    // Speed of sound = 0.0343 cm/us. Divide by 2 because the wave travels out and back.
    *distance_cm = (float)pulse_duration * 0.0343f / 2.0f;

    return FUNCTION_OK;
}