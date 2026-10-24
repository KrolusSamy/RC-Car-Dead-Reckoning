/*------------------------------------------------------------------------------------------------------*-
 * File Name          : HAL_ENCODER_Interface.h
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : Function prototypes for the Encoder driver.
 -------------------------------------------------------------------------------------------------------
-See HAL_ENCODER_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

#ifndef HAL_ENCODER_INTERFACE_H_
#define HAL_ENCODER_INTERFACE_H_

#include "Error_States.h"
#include <stdint.h>

ErrorState_TypeDef HAL_ENCODER_Init(void);
ErrorState_TypeDef HAL_ENCODER_GetTicks(uint32_t* left_ticks, uint32_t* right_ticks);
ErrorState_TypeDef HAL_ENCODER_ResetTicks(void);

#endif /* HAL_ENCODER_INTERFACE_H_ */