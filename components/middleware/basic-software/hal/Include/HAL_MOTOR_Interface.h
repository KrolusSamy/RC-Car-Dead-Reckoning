/*------------------------------------------------------------------------------------------------------*-
 * File Name          : HAL_MOTOR_Interface.h
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : This file contains all the functions prototypes for the Motor Driver.
 -------------------------------------------------------------------------------------------------------
-See HAL_MOTOR_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

#ifndef HAL_MOTOR_INTERFACE_H_
#define HAL_MOTOR_INTERFACE_H_

// ------------------------------------ Includes -------------------------------------------------------
#include "Error_States.h"
#include <stdint.h>

// ------------------------------------ Public/Exported Data Types -------------------------------------
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_STOP
} MotorDirection_TypeDef;

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} MotorSelect_TypeDef;

// ------------------------------------ Public/Exported functions Prototypes ---------------------------
ErrorState_TypeDef HAL_MOTOR_Init(void);
ErrorState_TypeDef HAL_MOTOR_SetSpeedAndDir(MotorSelect_TypeDef motor, MotorDirection_TypeDef dir, uint8_t speed_pwm);
ErrorState_TypeDef HAL_MOTOR_StopAll(void);

#endif /* HAL_MOTOR_INTERFACE_H_ */