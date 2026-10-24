/*------------------------------------------------------------------------------------------------------*-
* File Name          : MPU6050_Cfg.h
* Author             : 
* Date First Issued  : Oct 2026
* Description        : This file contains all the configurations for the MPU6050 HAL driver.
 -------------------------------------------------------------------------------------------------------
-See MPU6050_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/


// ------------------------------------- Define Guardian to prevent recursive inclusion ----------------
#ifndef MPU6050_CFG_H_
#define MPU6050_CFG_H_


// ------------------------------------ Includes -------------------------------------------------------


// ------------------------------------ Public/Exported Data Types -------------------------------------


// ------------------------------------ Public/Exported constant - Macros ------------------------------

/* ---------------------------- I2C bus ---------------------------- */
#define MPU6050_I2C_PORT                 0              /* I2C_NUM_0                                      */
#define MPU6050_I2C_SDA_PIN              21             /* SDA GPIO                                       */
#define MPU6050_I2C_SCL_PIN              22             /* SCL GPIO                                       */
#define MPU6050_I2C_FREQ_HZ              400000         /* Fast mode                                      */
#define MPU6050_I2C_TIMEOUT_MS           10             /* Per transaction                                */
#define MPU6050_I2C_RETRIES              3              /* Extra attempts before reporting an error       */
#define MPU6050_INTERNAL_PULLUP_ENABLE   1              /* 1: also enable ESP32 pull-ups (use external 4.7k for 400 kHz) */

/* ---------------------------- Device address --------------------- */
#define MPU6050_ADDR                     0x68           /* AD0 = LOW -> 0x68, AD0 = HIGH -> 0x69          */

/* ---------------------------- Identity check --------------------- */
#define MPU6050_WHO_AM_I_CHECK_ENABLE    1
#define MPU6050_WHO_AM_I_EXPECTED        0x68           /* Genuine MPU6050. Some clones report 0x70/0x72  */

/* ---------------------------- Gyroscope full-scale range --------- */
#define MPU6050_GYRO_RANGE_250DPS        0
#define MPU6050_GYRO_RANGE_500DPS        1
#define MPU6050_GYRO_RANGE_1000DPS       2
#define MPU6050_GYRO_RANGE_2000DPS       3
#define MPU6050_GYRO_RANGE               MPU6050_GYRO_RANGE_500DPS

/* ---------------------------- Accelerometer full-scale range ----- */
#define MPU6050_ACCEL_RANGE_2G           0
#define MPU6050_ACCEL_RANGE_4G           1
#define MPU6050_ACCEL_RANGE_8G           2
#define MPU6050_ACCEL_RANGE_16G          3
#define MPU6050_ACCEL_RANGE              MPU6050_ACCEL_RANGE_4G

/* ---------------------------- Digital low-pass filter ------------ */
/* 0: 260 Hz | 1: 184 Hz | 2: 94 Hz | 3: 44 Hz | 4: 21 Hz | 5: 10 Hz | 6: 5 Hz
   (accel bandwidth; gyro is slightly lower, 42 Hz for setting 3).
   Setting 0 makes the internal gyro rate 8 kHz, every other setting 1 kHz.                              */
#define MPU6050_DLPF_CFG                 3

/* ---------------------------- Output data rate ------------------- */
#define MPU6050_SAMPLE_RATE_HZ           200            /* SMPLRT_DIV is derived from this                */

/* ---------------------------- Timing ----------------------------- */
#define MPU6050_RESET_DELAY_MS           100            /* After DEVICE_RESET                             */
#define MPU6050_WAKE_DELAY_MS            50             /* After leaving sleep / selecting PLL clock      */


/* ---------------------------- Derived values (do not edit) ------- */
#if (MPU6050_GYRO_RANGE == MPU6050_GYRO_RANGE_250DPS)
    #define MPU6050_GYRO_LSB_PER_DPS     131.0f
#elif (MPU6050_GYRO_RANGE == MPU6050_GYRO_RANGE_500DPS)
    #define MPU6050_GYRO_LSB_PER_DPS     65.5f
#elif (MPU6050_GYRO_RANGE == MPU6050_GYRO_RANGE_1000DPS)
    #define MPU6050_GYRO_LSB_PER_DPS     32.8f
#elif (MPU6050_GYRO_RANGE == MPU6050_GYRO_RANGE_2000DPS)
    #define MPU6050_GYRO_LSB_PER_DPS     16.4f
#else
    #error "MPU6050_GYRO_RANGE is invalid"
#endif

#if (MPU6050_ACCEL_RANGE == MPU6050_ACCEL_RANGE_2G)
    #define MPU6050_ACCEL_LSB_PER_G      16384.0f
#elif (MPU6050_ACCEL_RANGE == MPU6050_ACCEL_RANGE_4G)
    #define MPU6050_ACCEL_LSB_PER_G      8192.0f
#elif (MPU6050_ACCEL_RANGE == MPU6050_ACCEL_RANGE_8G)
    #define MPU6050_ACCEL_LSB_PER_G      4096.0f
#elif (MPU6050_ACCEL_RANGE == MPU6050_ACCEL_RANGE_16G)
    #define MPU6050_ACCEL_LSB_PER_G      2048.0f
#else
    #error "MPU6050_ACCEL_RANGE is invalid"
#endif

#if (MPU6050_DLPF_CFG < 0) || (MPU6050_DLPF_CFG > 6)
    #error "MPU6050_DLPF_CFG must be 0..6"
#endif

#if (MPU6050_DLPF_CFG == 0)
    #define MPU6050_GYRO_OUTPUT_RATE_HZ  8000
#else
    #define MPU6050_GYRO_OUTPUT_RATE_HZ  1000
#endif

#if (MPU6050_SAMPLE_RATE_HZ < 4) || (MPU6050_SAMPLE_RATE_HZ > MPU6050_GYRO_OUTPUT_RATE_HZ)
    #error "MPU6050_SAMPLE_RATE_HZ out of range for the selected DLPF"
#endif

#define MPU6050_SMPLRT_DIV_VALUE         ((MPU6050_GYRO_OUTPUT_RATE_HZ / MPU6050_SAMPLE_RATE_HZ) - 1)
#define MPU6050_SAMPLE_PERIOD_US         (1000000UL / MPU6050_SAMPLE_RATE_HZ)


// ------------------------------------ Public/Exported Variables declaration --------------------------


// ------------------------------------ Public/Exported functions Prototypes -------------------------------------



#endif /* MPU6050_CFG_H_ */


/*-------------------------------------------------------------------------------------------------------------------*-
----------------------------------------------------- END OF FILE -----------------------------------------------------
-*-------------------------------------------------------------------------------------------------------------------*/
