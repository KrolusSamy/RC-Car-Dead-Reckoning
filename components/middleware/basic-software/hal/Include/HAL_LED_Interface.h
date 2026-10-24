/*------------------------------------------------------------------------------------------------------*-
 * File Name          : HAL_LED_Interface.h
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : This file contains all the functions prototypes for the HAL_LED firmware library.
 -------------------------------------------------------------------------------------------------------
-See HAL_LED_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

#ifndef HAL_LED_INTERFACE_H_
#define HAL_LED_INTERFACE_H_

// ------------------------------------ Includes -------------------------------------------------------
#include "Error_States.h"
#include <stdint.h>

// ------------------------------------ Public/Exported functions Prototypes ---------------------------

/*******************************************************************************
 * Function Name  : HAL_LED_Init
 * Description    : Initializes the RMT peripheral for the WS2812 addressable LED.
 * Input          : None
 * Output         : None
 * Return         : ErrorState_TypeDef (FUNCTION_OK if successful)
 *******************************************************************************/
ErrorState_TypeDef HAL_LED_Init(void);

/*******************************************************************************
 * Function Name  : HAL_LED_SetColor
 * Description    : Sets the RGB color of the onboard LED.
 * Input          : red (0-255), green (0-255), blue (0-255)
 * Output         : None
 * Return         : ErrorState_TypeDef 
 *******************************************************************************/
ErrorState_TypeDef HAL_LED_SetColor(uint8_t red, uint8_t green, uint8_t blue);

#endif /* HAL_LED_INTERFACE_H_ */