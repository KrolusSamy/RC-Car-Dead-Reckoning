/*-----------------------------------------------------------------------------------------------------*-
 * File Name          : HAL_MOTOR_Prog.c
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : Implementation of the L298N Motor Driver using ESP-IDF LEDC.                                          
--------------------------------------------------------------------------------------------------------
-See HAL_MOTOR_Interface.h and HAL_MOTOR_Cfg.h for details
-*-----------------------------------------------------------------------------------------------------*/

// --------------------------------- Includes -----------------------------------------------------
#include "HAL_MOTOR_Interface.h"
#include "HAL_MOTOR_Cfg.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// --------------------------------- Private variables definition ---------------------------------
static const char *TAG = "HAL_MOTOR";

// --------------------------------  Public/Exported functions definitions ------------------------
ErrorState_TypeDef HAL_MOTOR_Init(void)
{
    ESP_LOGI(TAG, "Initializing L298N Motor Driver...");

    /* Configure Directional GPIOs */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<HAL_MOTOR_IN1_PIN) | (1ULL<<HAL_MOTOR_IN2_PIN) | 
                        (1ULL<<HAL_MOTOR_IN3_PIN) | (1ULL<<HAL_MOTOR_IN4_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    
    // Ensure motors are initially stopped
    gpio_set_level(HAL_MOTOR_IN1_PIN, 0);
    gpio_set_level(HAL_MOTOR_IN2_PIN, 0);
    gpio_set_level(HAL_MOTOR_IN3_PIN, 0);
    gpio_set_level(HAL_MOTOR_IN4_PIN, 0);

    /* Configure LEDC Timer for PWM */
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = HAL_MOTOR_PWM_RES,
        .freq_hz          = HAL_MOTOR_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    /* Configure LEDC Channels for ENA and ENB */
    ledc_channel_config_t ledc_channel_ena = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = HAL_MOTOR_ENA_PIN,
        .duty           = 0, // 0% duty
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_ena);

    ledc_channel_config_t ledc_channel_enb = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = HAL_MOTOR_ENB_PIN,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel_enb);

    return FUNCTION_OK;
}

ErrorState_TypeDef HAL_MOTOR_SetSpeedAndDir(MotorSelect_TypeDef motor, MotorDirection_TypeDef dir, uint8_t speed_pwm)
{
    uint8_t pin_a, pin_b;
    ledc_channel_t pwm_channel;

    if (motor == MOTOR_LEFT) {
        /* SWAPPED IN1 and IN2 to fix the reversed motor direction in software */
        pin_a = HAL_MOTOR_IN2_PIN; 
        pin_b = HAL_MOTOR_IN1_PIN;
        pwm_channel = LEDC_CHANNEL_0;
    } else {
        pin_a = HAL_MOTOR_IN3_PIN;
        pin_b = HAL_MOTOR_IN4_PIN;
        pwm_channel = LEDC_CHANNEL_1;
    }

    /* Set Direction */
    if (dir == MOTOR_DIR_FORWARD) {
        gpio_set_level(pin_a, 1);
        gpio_set_level(pin_b, 0);
    } else if (dir == MOTOR_DIR_REVERSE) {
        gpio_set_level(pin_a, 0);
        gpio_set_level(pin_b, 1);
    } else {
        gpio_set_level(pin_a, 0);
        gpio_set_level(pin_b, 0);
        speed_pwm = 0;
    }

    /* Apply PWM Duty Cycle */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel, speed_pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel);

    return FUNCTION_OK;
}

ErrorState_TypeDef HAL_MOTOR_StopAll(void)
{
    HAL_MOTOR_SetSpeedAndDir(MOTOR_LEFT, MOTOR_DIR_STOP, 0);
    HAL_MOTOR_SetSpeedAndDir(MOTOR_RIGHT, MOTOR_DIR_STOP, 0);
    return FUNCTION_OK;
}