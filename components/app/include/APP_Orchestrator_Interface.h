/*------------------------------------------------------------------------------------------------------*-
 * File Name          : APP_Orchestrator_Interface.h
 * Author             : Tele-Operated Robot Team
 * Date First Issued  : 10-2026
 * Description        : Application layer initialization and task orchestration.
 -------------------------------------------------------------------------------------------------------
-See APP_Orchestrator_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

#ifndef APP_ORCHESTRATOR_INTERFACE_H_
#define APP_ORCHESTRATOR_INTERFACE_H_

// ------------------------------------ Includes -------------------------------------------------------
#include "Error_States.h"

// ------------------------------------ Public/Exported functions Prototypes ---------------------------

/*******************************************************************************
 * Function Name  : APP_Orchestrator_Init
 * Description    : Initializes all underlying HAL and Service layer modules.
 * Input          : None
 * Output         : None
 * Return         : ErrorState_TypeDef 
 *******************************************************************************/
ErrorState_TypeDef APP_Orchestrator_Init(void);

/*******************************************************************************
 * Function Name  : APP_Orchestrator_StartTasks
 * Description    : Creates and schedules all FreeRTOS tasks.
 * Input          : None
 * Output         : None
 * Return         : ErrorState_TypeDef 
 *******************************************************************************/
ErrorState_TypeDef APP_Orchestrator_StartTasks(void);

#endif /* APP_ORCHESTRATOR_INTERFACE_H_ */