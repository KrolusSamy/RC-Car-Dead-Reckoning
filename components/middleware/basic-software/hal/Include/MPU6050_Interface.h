/*------------------------------------------------------------------------------------------------------*-
 * File Name          : MPU6050_Interface.h
 * Author             : 
 * Date First Issued  : Oct 2026
 * Description        : This file contains all the functions prototypes for the MPU6050 HAL driver.
 -------------------------------------------------------------------------------------------------------
-See MPU6050_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/



// ------------------------------------- Define Guardian to prevent recursive inclusion ----------------
#ifndef MPU6050_INTERFACE_H_
#define MPU6050_INTERFACE_H_

// ------------------------------------ Includes -------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "Error_States.h"
#include "MPU6050_Cfg.h"   /* Exposes MPU6050_GYRO_LSB_PER_DPS, MPU6050_ACCEL_LSB_PER_G, sample rate */


// ------------------------------------ Public/Exported Data Types -------------------------------------

/**
  * @brief One raw sample. Values are exactly what the sensor registers hold (no bias, no scaling).
  *        Timestamp_us is taken from esp_timer just before the I2C burst starts.
  */
typedef struct
{
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Temp;            /*!< Raw temperature, convert with MPU6050_TempRawToCelsius() */
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    int64_t Timestamp_us;    /*!< Microseconds since boot                                  */
}MPU6050_RawData_TypeDef;


// ------------------------------------ Public/Exported constant - Macros ------------------------------


// ------------------------------------ Public/Exported Variables declaration --------------------------


// ------------------------------------ Public/Exported functions Prototypes -------------------------------------

/*------------------------------------------------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : MPU6050_Init
 * Description    : Creates the I2C bus, resets the sensor, selects the PLL clock, and applies the
 *                  DLPF, sample rate and full-scale ranges from MPU6050_Cfg.h. Configuration
 *                  registers are read back and verified.
 *                  Safe to call again after MPU6050_DeInit() (use that pair to recover from a hung bus).
 * Input          : None
 * Output         : None
 * Input/Output   : None
 * Return         : FUNCTION_OK, FUNCTION_COMMUNICATION_ERROR (no device / wrong WHO_AM_I / bad
 *                  read-back), FUNCTION_TIMEOUT_ERROR, FUNCTION_ERROR (bus creation failed)
 *******************************************************************************/
ErrorState_TypeDef MPU6050_Init(void);

/*******************************************************************************
 * Function Name  : MPU6050_DeInit
 * Description    : Puts the sensor to sleep and releases the I2C device and bus.
 * Input          : None
 * Output         : None
 * Input/Output   : None
 * Return         : FUNCTION_OK, FUNCTION_ERROR
 *******************************************************************************/
ErrorState_TypeDef MPU6050_DeInit(void);

/*******************************************************************************
 * Function Name  : MPU6050_ReadRaw
 * Description    : Reads accel, temperature and gyro in ONE 14-byte burst, so all values belong to
 *                  the same sample. If the transfer fails after the configured retries, the output
 *                  structure is left UNTOUCHED and an error is returned. The caller must never
 *                  integrate a sample for which an error was returned (this is what prevents stale
 *                  data from being integrated into heading drift).
 * Input          : None
 * Output         : Copy_pRawData : pointer to the structure that receives the sample
 * Input/Output   : None
 * Return         : FUNCTION_OK, FUNCTION_COMMUNICATION_ERROR, FUNCTION_TIMEOUT_ERROR,
 *                  FUNCTION_BUSY, FUNCTION_ERROR (not initialised / NULL pointer)
 *******************************************************************************/
ErrorState_TypeDef MPU6050_ReadRaw(MPU6050_RawData_TypeDef *Copy_pRawData);

/*******************************************************************************
 * Function Name  : MPU6050_IsDataReady
 * Description    : Reads INT_STATUS (this clears the flag). Use it to skip a read when no new
 *                  sample has been produced, or to detect missed samples.
 * Input          : None
 * Output         : Copy_pReady : true when a new sample is available
 * Input/Output   : None
 * Return         : FUNCTION_OK, FUNCTION_COMMUNICATION_ERROR, FUNCTION_TIMEOUT_ERROR, FUNCTION_ERROR
 *******************************************************************************/
ErrorState_TypeDef MPU6050_IsDataReady(bool *Copy_pReady);

/*******************************************************************************
 * Function Name  : MPU6050_TempRawToCelsius
 * Description    : Converts the raw temperature to degrees Celsius (datasheet: raw/340 + 36.53).
 *                  Useful for logging gyro bias against temperature.
 * Input          : Copy_s16TempRaw : raw temperature from MPU6050_RawData_TypeDef
 * Output         : None
 * Input/Output   : None
 * Return         : Temperature in degrees Celsius
 *******************************************************************************/
float MPU6050_TempRawToCelsius(int16_t Copy_s16TempRaw);

/*------------------------------------------------------------------------------------------------*/



#endif /* MPU6050_INTERFACE_H_ */

/*-------------------------------------------------------------------------------------------------------------------*-
----------------------------------------------------- END OF FILE -----------------------------------------------------
-*-------------------------------------------------------------------------------------------------------------------*/
