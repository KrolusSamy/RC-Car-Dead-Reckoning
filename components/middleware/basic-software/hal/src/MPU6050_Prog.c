/*-----------------------------------------------------------------------------------------------------*-
 * File Name          : MPU6050_Prog.c
 * Author             : 
 * Date First Issued  : Oct 2026
 * Description        : HAL driver for the MPU6050 using the ESP-IDF 5.1 legacy I2C master driver.
 *                      Knows registers and bytes only. It returns RAW counts: bias removal,
 *                      scaling and filtering belong to the service layer.
--------------------------------------------------------------------------------------------------------
-See MPU6050_Interface.h and MPU6050_Cfg.h for details
-*-----------------------------------------------------------------------------------------------------*/

// --------------------------------- Includes -----------------------------------------------------
#include <stddef.h>
#include "MPU6050_Interface.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// --------------------------------- Private macros -----------------------------------------------
#define MPU6050_TAG                     "MPU6050"

/* Register map */
#define MPU6050_REG_SMPLRT_DIV          0x19
#define MPU6050_REG_CONFIG              0x1A
#define MPU6050_REG_GYRO_CONFIG         0x1B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_INT_ENABLE          0x38
#define MPU6050_REG_INT_STATUS          0x3A
#define MPU6050_REG_ACCEL_XOUT_H        0x3B
#define MPU6050_REG_PWR_MGMT_1          0x6B
#define MPU6050_REG_WHO_AM_I            0x75

/* Bit values */
#define MPU6050_PWR1_DEVICE_RESET       0x80
#define MPU6050_PWR1_SLEEP              0x40
#define MPU6050_PWR1_CLKSEL_PLL_XGYRO   0x01
#define MPU6050_INT_DATA_RDY            0x01

#define MPU6050_BURST_LEN               14u

/* Early-return helper: runs a statement returning ErrorState_TypeDef, leaves on failure */
#define MPU6050_CHECK(Expr)                                   \
    do {                                                      \
        ErrorState_TypeDef Local_CheckState = (Expr);         \
        if (Local_CheckState != FUNCTION_OK)                  \
        {                                                     \
            MPU6050_ReleaseBus();                             \
            return Local_CheckState;                          \
        }                                                     \
    } while (0)


// --------------------------------- Private variables definition ---------------------------------
static bool Local_DriverInstalled = false;
static bool                    Local_Initialized = false;


// --------------------------------- Private functions prototypes ---------------------------------
static ErrorState_TypeDef MPU6050_MapError(esp_err_t Copy_Err);
static ErrorState_TypeDef MPU6050_WriteReg(uint8_t Copy_u8Reg, uint8_t Copy_u8Value);
static ErrorState_TypeDef MPU6050_ReadRegs(uint8_t Copy_u8Reg, uint8_t *Copy_pu8Buf, size_t Copy_Len);
static ErrorState_TypeDef MPU6050_VerifyReg(uint8_t Copy_u8Reg, uint8_t Copy_u8Expected);
static void               MPU6050_ReleaseBus(void);


// --------------------------------- Public/Exported variables definition -------------------------


// --------------------------------  Public/Exported functions definitions ------------------------

/*------------------------------------------------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : MPU6050_Init
 * Description    : See MPU6050_Interface.h
 *******************************************************************************/
ErrorState_TypeDef MPU6050_Init(void)
{
    if (Local_Initialized)
    {
        return FUNCTION_OK;
    }

    /* ESP-IDF 5.1 uses the legacy port-based I2C master driver. */
    i2c_config_t Local_I2cCfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MPU6050_I2C_SDA_PIN,
        .scl_io_num = MPU6050_I2C_SCL_PIN,
        .sda_pullup_en = MPU6050_INTERNAL_PULLUP_ENABLE ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .scl_pullup_en = MPU6050_INTERNAL_PULLUP_ENABLE ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .master.clk_speed = MPU6050_I2C_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t Local_Err = i2c_param_config((i2c_port_t)MPU6050_I2C_PORT, &Local_I2cCfg);
    if (Local_Err != ESP_OK) {
        ESP_LOGE(MPU6050_TAG, "I2C parameter configuration failed: %s", esp_err_to_name(Local_Err));
        return MPU6050_MapError(Local_Err);
    }
    Local_Err = i2c_driver_install((i2c_port_t)MPU6050_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (Local_Err != ESP_OK) {
        ESP_LOGE(MPU6050_TAG, "I2C driver install failed: %s", esp_err_to_name(Local_Err));
        return MPU6050_MapError(Local_Err);
    }
    Local_DriverInstalled = true;

    /* The WHO_AM_I register read below verifies the address and sensor connection. */

    /* 4. Full reset, then wake with the gyro PLL as clock source (more stable than the internal RC) */
    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET));
    vTaskDelay(pdMS_TO_TICKS(MPU6050_RESET_DELAY_MS));

    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_XGYRO));
    vTaskDelay(pdMS_TO_TICKS(MPU6050_WAKE_DELAY_MS));

    /* 5. Identity */
#if MPU6050_WHO_AM_I_CHECK_ENABLE
    uint8_t Local_u8Id = 0;
    MPU6050_CHECK(MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &Local_u8Id, 1));
    if (Local_u8Id != MPU6050_WHO_AM_I_EXPECTED)
    {
        MPU6050_ReleaseBus();
        ESP_LOGE(MPU6050_TAG, "WHO_AM_I = 0x%02X, expected 0x%02X", Local_u8Id, MPU6050_WHO_AM_I_EXPECTED);
        return FUNCTION_COMMUNICATION_ERROR;
    }
#endif

    /* 6. Configuration */
    const uint8_t Local_u8Config      = (uint8_t)MPU6050_DLPF_CFG;
    const uint8_t Local_u8SmplrtDiv   = (uint8_t)MPU6050_SMPLRT_DIV_VALUE;
    const uint8_t Local_u8GyroConfig  = (uint8_t)(MPU6050_GYRO_RANGE  << 3);
    const uint8_t Local_u8AccelConfig = (uint8_t)(MPU6050_ACCEL_RANGE << 3);

    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_CONFIG,       Local_u8Config));
    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV,   Local_u8SmplrtDiv));
    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG,  Local_u8GyroConfig));
    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, Local_u8AccelConfig));

    /* Enables the data-ready flag so MPU6050_IsDataReady() works (INT pin does not need to be wired) */
    MPU6050_CHECK(MPU6050_WriteReg(MPU6050_REG_INT_ENABLE,   MPU6050_INT_DATA_RDY));

    /* 7. Read-back: a range/scale mismatch is the classic cause of "turns 90 deg but reads 45 deg" */
    MPU6050_CHECK(MPU6050_VerifyReg(MPU6050_REG_CONFIG,       Local_u8Config));
    MPU6050_CHECK(MPU6050_VerifyReg(MPU6050_REG_SMPLRT_DIV,   Local_u8SmplrtDiv));
    MPU6050_CHECK(MPU6050_VerifyReg(MPU6050_REG_GYRO_CONFIG,  Local_u8GyroConfig));
    MPU6050_CHECK(MPU6050_VerifyReg(MPU6050_REG_ACCEL_CONFIG, Local_u8AccelConfig));

    Local_Initialized = true;
    ESP_LOGI(MPU6050_TAG, "Ready: gyro %.1f LSB/dps, accel %.0f LSB/g, %d Hz, DLPF %d",
             (double)MPU6050_GYRO_LSB_PER_DPS, (double)MPU6050_ACCEL_LSB_PER_G,
             MPU6050_SAMPLE_RATE_HZ, MPU6050_DLPF_CFG);
    return FUNCTION_OK;
}

/*******************************************************************************
 * Function Name  : MPU6050_DeInit
 * Description    : See MPU6050_Interface.h
 *******************************************************************************/
ErrorState_TypeDef MPU6050_DeInit(void)
{
    if (Local_DriverInstalled)
    {
        /* Best effort: sleep the sensor, ignore the result because we are leaving anyway */
        (void)MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_SLEEP);
    }

    MPU6050_ReleaseBus();
    Local_Initialized = false;
    return FUNCTION_OK;
}

/*******************************************************************************
 * Function Name  : MPU6050_ReadRaw
 * Description    : See MPU6050_Interface.h
 *******************************************************************************/
ErrorState_TypeDef MPU6050_ReadRaw(MPU6050_RawData_TypeDef *Copy_pRawData)
{
    if ((!Local_Initialized) || (Copy_pRawData == NULL))
    {
        return FUNCTION_ERROR;
    }

    uint8_t Local_u8Buf[MPU6050_BURST_LEN];

    /* The sample instant is taken just before the I2C burst (about 0.4 ms at 400 kHz) */
    const int64_t Local_s64Time = esp_timer_get_time();

    ErrorState_TypeDef Local_State = MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, Local_u8Buf, MPU6050_BURST_LEN);
    if (Local_State != FUNCTION_OK)
    {
        return Local_State;   /* output untouched on purpose */
    }

    /* Sensor sends high byte first */
    Copy_pRawData->Accel_X = (int16_t)(((uint16_t)Local_u8Buf[0]  << 8) | Local_u8Buf[1]);
    Copy_pRawData->Accel_Y = (int16_t)(((uint16_t)Local_u8Buf[2]  << 8) | Local_u8Buf[3]);
    Copy_pRawData->Accel_Z = (int16_t)(((uint16_t)Local_u8Buf[4]  << 8) | Local_u8Buf[5]);
    Copy_pRawData->Temp    = (int16_t)(((uint16_t)Local_u8Buf[6]  << 8) | Local_u8Buf[7]);
    Copy_pRawData->Gyro_X  = (int16_t)(((uint16_t)Local_u8Buf[8]  << 8) | Local_u8Buf[9]);
    Copy_pRawData->Gyro_Y  = (int16_t)(((uint16_t)Local_u8Buf[10] << 8) | Local_u8Buf[11]);
    Copy_pRawData->Gyro_Z  = (int16_t)(((uint16_t)Local_u8Buf[12] << 8) | Local_u8Buf[13]);
    Copy_pRawData->Timestamp_us = Local_s64Time;

    return FUNCTION_OK;
}

/*******************************************************************************
 * Function Name  : MPU6050_IsDataReady
 * Description    : See MPU6050_Interface.h
 *******************************************************************************/
ErrorState_TypeDef MPU6050_IsDataReady(bool *Copy_pReady)
{
    if ((!Local_Initialized) || (Copy_pReady == NULL))
    {
        return FUNCTION_ERROR;
    }

    uint8_t Local_u8Status = 0;
    ErrorState_TypeDef Local_State = MPU6050_ReadRegs(MPU6050_REG_INT_STATUS, &Local_u8Status, 1);
    if (Local_State == FUNCTION_OK)
    {
        *Copy_pReady = ((Local_u8Status & MPU6050_INT_DATA_RDY) != 0u);
    }
    return Local_State;
}

/*******************************************************************************
 * Function Name  : MPU6050_TempRawToCelsius
 * Description    : See MPU6050_Interface.h
 *******************************************************************************/
float MPU6050_TempRawToCelsius(int16_t Copy_s16TempRaw)
{
    return ((float)Copy_s16TempRaw / 340.0f) + 36.53f;
}

/*------------------------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------------------------*/
// --------------------------------- Private functions definitions ---------------------------------

/*------------------------------------------------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : MPU6050_MapError
 * Description    : Converts an esp_err_t into the project-wide ErrorState_TypeDef.
 * Input          : Copy_Err : ESP-IDF error code
 * Return         : Matching ErrorState_TypeDef
 *******************************************************************************/
static ErrorState_TypeDef MPU6050_MapError(esp_err_t Copy_Err)
{
    switch (Copy_Err)
    {
        case ESP_OK:                 return FUNCTION_OK;
        case ESP_ERR_TIMEOUT:        return FUNCTION_TIMEOUT_ERROR;
        case ESP_ERR_INVALID_STATE:  return FUNCTION_BUSY;
        case ESP_ERR_INVALID_ARG:    return FUNCTION_ERROR;
        default:                     return FUNCTION_COMMUNICATION_ERROR;   /* NACK, bus error, ... */
    }
}

/*******************************************************************************
 * Function Name  : MPU6050_WriteReg
 * Description    : Writes one register, retrying on failure.
 * Input          : Copy_u8Reg   : register address
 *                  Copy_u8Value : value to write
 * Return         : ErrorState_TypeDef
 *******************************************************************************/
static ErrorState_TypeDef MPU6050_WriteReg(uint8_t Copy_u8Reg, uint8_t Copy_u8Value)
{
    const uint8_t Local_u8Frame[2] = { Copy_u8Reg, Copy_u8Value };
    esp_err_t Local_Err = ESP_FAIL;

    for (uint8_t Local_u8Try = 0; Local_u8Try <= MPU6050_I2C_RETRIES; Local_u8Try++)
    {
        Local_Err = i2c_master_write_to_device((i2c_port_t)MPU6050_I2C_PORT, MPU6050_ADDR,
                                              Local_u8Frame, sizeof(Local_u8Frame),
                                              pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));
        if (Local_Err == ESP_OK)
        {
            break;
        }
    }

    if (Local_Err != ESP_OK)
    {
        ESP_LOGW(MPU6050_TAG, "Write 0x%02X failed: %s", Copy_u8Reg, esp_err_to_name(Local_Err));
    }
    return MPU6050_MapError(Local_Err);
}

/*******************************************************************************
 * Function Name  : MPU6050_ReadRegs
 * Description    : Reads Copy_Len consecutive registers (write address, repeated start, read),
 *                  retrying on failure.
 * Input          : Copy_u8Reg : first register address
 *                  Copy_Len   : number of bytes
 * Output         : Copy_pu8Buf : receives the bytes
 * Return         : ErrorState_TypeDef
 *******************************************************************************/
static ErrorState_TypeDef MPU6050_ReadRegs(uint8_t Copy_u8Reg, uint8_t *Copy_pu8Buf, size_t Copy_Len)
{
    esp_err_t Local_Err = ESP_FAIL;

    for (uint8_t Local_u8Try = 0; Local_u8Try <= MPU6050_I2C_RETRIES; Local_u8Try++)
    {
        Local_Err = i2c_master_write_read_device((i2c_port_t)MPU6050_I2C_PORT, MPU6050_ADDR,
                                                 &Copy_u8Reg, 1, Copy_pu8Buf, Copy_Len,
                                                 pdMS_TO_TICKS(MPU6050_I2C_TIMEOUT_MS));
        if (Local_Err == ESP_OK)
        {
            break;
        }
    }

    if (Local_Err != ESP_OK)
    {
        ESP_LOGW(MPU6050_TAG, "Read 0x%02X failed: %s", Copy_u8Reg, esp_err_to_name(Local_Err));
    }
    return MPU6050_MapError(Local_Err);
}

/*******************************************************************************
 * Function Name  : MPU6050_VerifyReg
 * Description    : Reads a register back and compares it with the expected value.
 * Input          : Copy_u8Reg      : register address
 *                  Copy_u8Expected : expected content
 * Return         : FUNCTION_OK, FUNCTION_COMMUNICATION_ERROR on mismatch, or the read error
 *******************************************************************************/
static ErrorState_TypeDef MPU6050_VerifyReg(uint8_t Copy_u8Reg, uint8_t Copy_u8Expected)
{
    uint8_t Local_u8Value = 0;
    ErrorState_TypeDef Local_State = MPU6050_ReadRegs(Copy_u8Reg, &Local_u8Value, 1);

    if ((Local_State == FUNCTION_OK) && (Local_u8Value != Copy_u8Expected))
    {
        ESP_LOGE(MPU6050_TAG, "Reg 0x%02X read-back 0x%02X, expected 0x%02X",
                 Copy_u8Reg, Local_u8Value, Copy_u8Expected);
        Local_State = FUNCTION_COMMUNICATION_ERROR;
    }
    return Local_State;
}

/*******************************************************************************
 * Function Name  : MPU6050_ReleaseBus
 * Description    : Removes the I2C device and deletes the bus (whatever exists).
 *******************************************************************************/
static void MPU6050_ReleaseBus(void)
{
    if (Local_DriverInstalled)
    {
        (void)i2c_driver_delete((i2c_port_t)MPU6050_I2C_PORT);
        Local_DriverInstalled = false;
    }
}

/*------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------*-
----------------------------------------------------- END OF FILE -----------------------------------------------------
-*-------------------------------------------------------------------------------------------------------------------*/
