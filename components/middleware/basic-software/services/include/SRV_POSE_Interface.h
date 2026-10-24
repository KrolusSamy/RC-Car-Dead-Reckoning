/*------------------------------------------------------------------------------------------------------*-
 * File Name          : SRV_POSE_Interface.h
 * Author             : 
 * Date First Issued  : Oct 2026
 * Description        : Dead-reckoning estimator: gyro heading with bias tracking (ZUPT), Madgwick 6-DoF
 *                      tilt, encoder odometry, velocity and terrain/impact events.
 *
 *                      LAYERING: this service uses HAL *types and constants* only (MPU6050_Interface.h)
 *                      and NEVER calls a HAL function or another service. The APP layer reads the
 *                      hardware and feeds the numbers in. No FreeRTOS inside.
 *
 *                      THREADING: not thread-safe. Call every function from ONE task (or wrap all calls
 *                      in an APP mutex).
 -------------------------------------------------------------------------------------------------------
-See SRV_POSE_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

// ------------------------------------- Define Guardian to prevent recursive inclusion ----------------
#ifndef SRV_POSE_INTERFACE_H_
#define SRV_POSE_INTERFACE_H_

// ------------------------------------ Includes -------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "Error_States.h"
#include "Robot_Types.h"
#include "MPU6050_Interface.h"      /* HAL types/constants only (rename to HAL_MPU6050_Interface.h if you renamed the driver) */
#include "SRV_POSE_Cfg.h"

// ------------------------------------ Public/Exported Data Types -------------------------------------

typedef enum
{
    SRV_POSE_STATE_WARMUP = 0,      /*!< Waiting for the sensor to warm up (car must be still)       */
    SRV_POSE_STATE_CALIBRATING,     /*!< Averaging gyro for the boot bias (car must be still)        */
    SRV_POSE_STATE_READY,           /*!< Estimating                                                  */
    SRV_POSE_STATE_CAL_FAILED       /*!< Car moved during calibration: call SRV_POSE_Recalibrate()   */
}SRV_POSE_State_TypeDef;

/* Event / status flags (levels). Rising edges are also available through SRV_POSE_PopNewEvents() */
#define SRV_POSE_FLAG_STATIONARY        (1u << 0)
#define SRV_POSE_FLAG_RAMP_UP           (1u << 1)
#define SRV_POSE_FLAG_RAMP_DOWN         (1u << 2)
#define SRV_POSE_FLAG_TILT_LEFT_UP      (1u << 3)
#define SRV_POSE_FLAG_TILT_RIGHT_UP     (1u << 4)
#define SRV_POSE_FLAG_STALL             (1u << 5)   /*!< Commanded but not moving                       */
#define SRV_POSE_FLAG_WALL_HIT          (1u << 6)   /*!< Stall + flat pitch + impact spike              */
#define SRV_POSE_FLAG_TOO_STEEP         (1u << 7)   /*!< Stall while tilted nose up/down                */
#define SRV_POSE_FLAG_STUCK             (1u << 8)   /*!< Long stall, no impact, flat (wheel caught)     */

typedef struct
{
    float   X_m, Y_m, Theta_deg;
}SRV_POSE_Track_TypeDef;

/** Baseline estimators for the "what does the IMU buy us" demo (same run, three different headings). */
typedef struct
{
    SRV_POSE_Track_TypeDef EncodersOnly;    /*!< Heading from the wheel difference only (no IMU)           */
    SRV_POSE_Track_TypeDef GyroBootBias;    /*!< Heading from the gyro with the boot bias, no tracking     */
    SRV_POSE_Track_TypeDef ImuOnly;         /*!< IMU orientation + raw calibrated accel, double integrated */
}SRV_POSE_Ablation_TypeDef;

typedef struct
{
    float    GyroBias_dps[3];
    float    GyroZRaw_dps;
    float    Temp_C;
    int32_t  SignedTicksL, SignedTicksR;    /*!< Cumulative, direction applied                             */
    uint32_t MissedSamples;                 /*!< IMU gaps (dt too long or non-positive)                    */
}SRV_POSE_Debug_TypeDef;


// ------------------------------------ Public/Exported functions Prototypes -------------------------------------

/*******************************************************************************
 * Function Name  : SRV_POSE_Init
 * Description    : Resets the whole estimator and starts the warm-up phase.
 * Return         : FUNCTION_OK
 *******************************************************************************/
ErrorState_TypeDef SRV_POSE_Init(void);

/*******************************************************************************
 * Function Name  : SRV_POSE_Recalibrate
 * Description    : Restarts the boot gyro calibration (skips the warm-up). Car must be still.
 * Return         : FUNCTION_OK
 *******************************************************************************/
ErrorState_TypeDef SRV_POSE_Recalibrate(void);

/*******************************************************************************
 * Function Name  : SRV_POSE_AbortCalibration
 * Description    : Marks the active boot/recalibration window as failed. The APP
 *                  uses this when encoder motion is detected during calibration.
 *******************************************************************************/
ErrorState_TypeDef SRV_POSE_AbortCalibration(void);

/*******************************************************************************
 * Function Name  : SRV_POSE_UpdateImu
 * Description    : Feed ONE valid IMU sample (call at the IMU rate, 200 Hz). Handles warm-up and boot
 *                  calibration, tilt filter, heading integration, stationary detector and bias tracking.
 *                  Call it ONLY when MPU6050_ReadRaw() returned FUNCTION_OK; never feed a stale sample.
 * Input          : Copy_pRaw : sample from the HAL (the timestamp is used for dt)
 * Return         : FUNCTION_OK   (sample processed in READY state)
 *                  FUNCTION_BUSY (warm-up / calibrating: not ready yet)
 *                  FUNCTION_ERROR (calibration failed, or NULL)
 *******************************************************************************/
ErrorState_TypeDef SRV_POSE_UpdateImu(const MPU6050_RawData_TypeDef *Copy_pRaw);

/*******************************************************************************
 * Function Name  : SRV_POSE_UpdateOdom
 * Description    : Feed the encoders and the motor commands (call at the control rate, e.g. 50 Hz).
 *                  Updates the pose, wheel speeds, velocity and event flags.
 * Input          : Copy_u32TicksL/R : CUMULATIVE unsigned tick counts from the HAL (never reset them)
 *                  Copy_s16PwmL/R   : the signed PWM that was commanded (sign = direction, 0 = coast/stop).
 *                                     This is how the unsigned LM393 ticks get a direction.
 *                  Copy_s64Time_us  : timestamp (esp_timer_get_time())
 * Return         : FUNCTION_OK, FUNCTION_BUSY (not READY yet)
 *******************************************************************************/
ErrorState_TypeDef SRV_POSE_UpdateOdom(uint32_t Copy_u32TicksL, uint32_t Copy_u32TicksR,
                                       int16_t Copy_s16PwmL, int16_t Copy_s16PwmR, int64_t Copy_s64Time_us);

/* Outputs (all copy a snapshot; cheap) */
void                    SRV_POSE_GetPose(Pose_TypeDef *Copy_pPose);
void                    SRV_POSE_GetWheelSpeeds(float *Copy_pLeft_mps, float *Copy_pRight_mps);
uint32_t                SRV_POSE_GetFlags(void);
uint32_t                SRV_POSE_PopNewEvents(void);        /*!< Flags raised since the last call (rising edges) */
SRV_POSE_State_TypeDef  SRV_POSE_GetState(void);
void                    SRV_POSE_GetDebug(SRV_POSE_Debug_TypeDef *Copy_pDebug);
void                    SRV_POSE_GetAblation(SRV_POSE_Ablation_TypeDef *Copy_pAblation);

/*******************************************************************************
 * Function Name  : SRV_POSE_ResetPose
 * Description    : Sets x, y, z and heading to zero (and the ablation tracks). Keeps the gyro bias.
 *******************************************************************************/
void SRV_POSE_ResetPose(void);

#endif /* SRV_POSE_INTERFACE_H_ */

/*-------------------------------------------------------------------------------------------------------------------*-
----------------------------------------------------- END OF FILE -----------------------------------------------------
-*-------------------------------------------------------------------------------------------------------------------*/
