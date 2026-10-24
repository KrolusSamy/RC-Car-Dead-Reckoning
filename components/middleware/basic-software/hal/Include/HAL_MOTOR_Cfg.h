/*------------------------------------------------------------------------------------------------------*-
* File Name          : HAL_MOTOR_Cfg.h
* Author             : Tele-Operated Robot Team
* Date First Issued  : 10-2026
* Description        : This file contains all the configurations for the L298N Motor Driver.
 -------------------------------------------------------------------------------------------------------
-See HAL_MOTOR_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

#ifndef HAL_MOTOR_CFG_H_
#define HAL_MOTOR_CFG_H_

// ------------------------------------ Public/Exported constant - Macros ------------------------------
/* Motor A (Left) Pins */
#define HAL_MOTOR_IN1_PIN       2
#define HAL_MOTOR_IN2_PIN       3
#define HAL_MOTOR_ENA_PIN       6

/* Motor B (Right) Pins */
#define HAL_MOTOR_IN3_PIN       4
#define HAL_MOTOR_IN4_PIN       5
#define HAL_MOTOR_ENB_PIN       7

/* LEDC PWM Configuration */
#define HAL_MOTOR_PWM_FREQ_HZ   5000
#define HAL_MOTOR_PWM_RES       LEDC_TIMER_8_BIT // 0-255 duty cycle

#endif /* HAL_MOTOR_CFG_H_ */