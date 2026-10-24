#ifndef HAL_ULTRASONIC_INTERFACE_H
#define HAL_ULTRASONIC_INTERFACE_H

#include "Error_States.h"
#include <stdint.h>

ErrorState_TypeDef HAL_Ultrasonic_Init(void);
ErrorState_TypeDef HAL_Ultrasonic_GetDistance(float *distance_cm);

#endif