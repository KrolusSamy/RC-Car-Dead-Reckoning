/**
 ******************************************************************************
 * @file    Error_States.h
 * @brief   This file contains all the Error states that are shared with all functions.
 ******************************************************************************
 */

/*-----------------------------------------------------------------------------------------------------*/

// ------------------------------------- Define Guardian to prevent recursive inclusion ----------------
#ifndef ERROR_STATES_H_
#define ERROR_STATES_H_


// ------------------------------------ Public/Exported Data Types -------------------------------------

/** @addtogroup Error_States_Exported_Types
  * @{
  */

/**
  * @brief ErrorState_TypeDef 
  * The definition of the ErrorState_TypeDef that is shared with all functions.
  */

typedef enum{
	FUNCTION_ERROR = 0,                         /*!< The function does not operate correctly */
	FUNCTION_COMMUNICATION_ERROR,               /*!< The function has a communication error */
  FUNCTION_TIMEOUT_ERROR,                     /*!< The function has a time out error */
  FUNCTION_BUSY,                              /*!< The function is busy */
	FUNCTION_OK                                 /*!< The function operates correctly */
}ErrorState_TypeDef;

/**
  * @}
  */


/*------------------------------------------------------------------------------------------------*/

#endif /* ERROR_STATES_H_ */

/*---------------------------------------------------------------------------------------------------*-
------------------------------------- END OF FILE -----------------------------------------------------
----------------------------------------------------------------------------------------------------*/
