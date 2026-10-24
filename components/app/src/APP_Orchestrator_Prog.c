/*-----------------------------------------------------------------------------------------------------*-
 * File Name          : APP_Orchestrator_Prog.c
 * Description        : Application task wiring, boot calibration gate, and tele-operation commands.
--------------------------------------------------------------------------------------------------------*/

#include "APP_Orchestrator_Interface.h"
#include "APP_Tasks_Cfg.h"
#include "HAL_LED_Interface.h"
#include "HAL_MOTOR_Interface.h"
#include "HAL_ENCODER_Interface.h"
#include "HAL_ULTRASONIC_Interface.h"
#include "MPU6050_Interface.h"
#include "SRV_POSE_Interface.h"
#include "SRV_WIFI_Interface.h"
#include "SRV_WIFI_Cfg.h"
#include "SRV_UDP_Interface.h"
#include "SRV_OTA_Interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#define APP_BUILD_VERSION "waypoint-estimator-comparison-1"
static const char *TAG = "APP_ORCHESTRATOR";

#define APP_EVENT_POSE_READY       (1u << 0)
#define APP_EVENT_POSE_FAILED      (1u << 1)
#define APP_EVENT_RECALIBRATE      (1u << 2)
#define APP_EVENT_POSE_WARMUP      (1u << 3)
#define APP_EVENT_POSE_CALIBRATING (1u << 4)

/* Calibration move based on the user's repeated 80-tick ≈ 52 cm trials. */
#define APP_CAL_STRAIGHT_TICKS           80u
#define APP_CAL_PIVOT_RIGHT_TICKS        15u
#define APP_CAL_PIVOT_LEFT_TICKS         20u
#define APP_TAP_STRAIGHT_TICKS           10u
#define APP_TAP_PIVOT_TICKS               3u

/* Starting gains for short, slow trials. Edit these only between runs.
 * Encoder correction balances wheel travel; heading correction damps yaw. */
#define APP_TUNE_BASE_PWM               132
#define APP_TUNE_MIN_PWM                125
#define APP_TUNE_MAX_PWM                155
#define APP_TUNE_ENCODER_KP            20.0f  /* PWM per metre of normalized wheel-distance error */
#define APP_TUNE_ENCODER_CORR_MAX       18.0f
#define APP_TUNE_HEADING_KP              0.50f  /* PWM per degree of heading error */
#define APP_TUNE_HEADING_CORR_MAX       12.0f
#define APP_TUNE_MAX_MOVE_MS             8000u
#define APP_OBSTACLE_STOP_CM               10.0f

static float App_ClampF(float value, float low, float high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

static float App_WrapDegrees(float degrees)
{
    while (degrees > 180.0f) degrees -= 360.0f;
    while (degrees < -180.0f) degrees += 360.0f;
    return degrees;
}

static EventGroupHandle_t Local_PoseEvents = NULL;
static volatile bool Local_SystemFault = false;
static volatile bool Local_UdpReady = false;
static bool Local_ImuReady = false;
static bool Local_UltrasonicReady = false;
static volatile int16_t Local_CommandPwmL = 0;
static volatile int16_t Local_CommandPwmR = 0;
/* Published by Task_IMU; motion commands consume this snapshot without calling
 * the single-owner pose service from a second task. */
static volatile float Local_HeadingDeg = 0.0f;
static volatile bool Local_ObstacleBlocked = false;

static ErrorState_TypeDef App_LatchFault(const char *context, ErrorState_TypeDef error)
{
    Local_SystemFault = true;
    (void)HAL_MOTOR_StopAll();
    ESP_LOGE(TAG, "FAULT: %s (ErrorState_TypeDef=%d)", context, (int)error);
    (void)SRV_UDP_Send("ERROR:SYSTEM_FAULT");
    return error;
}

typedef enum {
    MOVE_FORWARD,
    MOVE_REVERSE,
    MOVE_PIVOT_RIGHT,
    MOVE_PIVOT_LEFT
} MoveDir_t;

/* Slow straight-run controller. The prior PID values were removed: they were
 * tuned under different battery/motor conditions and are not a safe baseline. */
static void Execute_Move(MoveDir_t dir, uint32_t target_ticks) {
    uint32_t ticks_l = 0, ticks_r = 0;
    uint32_t start_ticks_l = 0, start_ticks_r = 0;
    uint32_t moved_l = 0, moved_r = 0;
    int32_t left_pwm = APP_TUNE_BASE_PWM, right_pwm = APP_TUNE_BASE_PWM;
    int64_t start_us = esp_timer_get_time();
    float start_heading = 0.0f;
    float heading_error = 0.0f;
    float encoder_correction = 0.0f;
    float heading_correction = 0.0f;
    bool completed = false;
    bool obstacle_abort = false;
    bool calibration_abort = false;

    if (dir == MOVE_FORWARD) {
        if (!Local_UltrasonicReady) {
            (void)SRV_UDP_Send("NACK:ULTRASONIC_UNAVAILABLE");
            return;
        }
        if (Local_ObstacleBlocked) {
            (void)SRV_UDP_Send("NACK:OBSTACLE_BLOCKED");
            return;
        }
    }

    uint8_t dir_l = MOTOR_DIR_FORWARD;
    uint8_t dir_r = MOTOR_DIR_FORWARD;

    if (dir == MOVE_REVERSE) {
        dir_l = MOTOR_DIR_REVERSE;
        dir_r = MOTOR_DIR_REVERSE;
    } else if (dir == MOVE_PIVOT_RIGHT) {
        dir_l = MOTOR_DIR_FORWARD;
        dir_r = MOTOR_DIR_REVERSE;
    } else if (dir == MOVE_PIVOT_LEFT) {
        dir_l = MOTOR_DIR_REVERSE;
        dir_r = MOTOR_DIR_FORWARD;
    }

    ESP_LOGI(TAG, "TUNE start dir=%d target_ticks=%lu base_pwm=%d encKp=%.3f headingKp=%.3f",
             dir, (unsigned long)target_ticks, APP_TUNE_BASE_PWM,
             (double)APP_TUNE_ENCODER_KP, (double)APP_TUNE_HEADING_KP);
    /* Keep hardware counts cumulative; use a local baseline for this move. */
    (void)HAL_ENCODER_GetTicks(&start_ticks_l, &start_ticks_r);
    start_heading = Local_HeadingDeg;
    HAL_MOTOR_SetSpeedAndDir(MOTOR_LEFT, (MotorDirection_TypeDef)dir_l, APP_TUNE_BASE_PWM);
    Local_CommandPwmL = (dir_l == MOTOR_DIR_REVERSE) ? -(int16_t)APP_TUNE_BASE_PWM : (int16_t)APP_TUNE_BASE_PWM;
    Local_CommandPwmR = 0;
    (void)HAL_MOTOR_SetSpeedAndDir(MOTOR_RIGHT, (MotorDirection_TypeDef)dir_r, APP_TUNE_BASE_PWM);
    Local_CommandPwmR = (dir_r == MOTOR_DIR_REVERSE) ? -(int16_t)APP_TUNE_BASE_PWM : (int16_t)APP_TUNE_BASE_PWM;

    while (1) {
        if ((xEventGroupGetBits(Local_PoseEvents) & APP_EVENT_POSE_READY) == 0u) {
            calibration_abort = true;
            break;
        }
        if ((dir == MOVE_FORWARD) && Local_ObstacleBlocked) {
            obstacle_abort = true;
            break;
        }
        HAL_ENCODER_GetTicks(&ticks_l, &ticks_r);
        moved_l = ticks_l - start_ticks_l;
        moved_r = ticks_r - start_ticks_r;
        if (moved_l >= target_ticks && moved_r >= target_ticks) {
            completed = true;
            break;
        }
        if ((esp_timer_get_time() - start_us) >= ((int64_t)APP_TUNE_MAX_MOVE_MS * 1000LL)) {
            break;
        }

        if ((dir == MOVE_FORWARD || dir == MOVE_REVERSE) && (dir_l != MOTOR_DIR_STOP)) {
            const float distance_error_m = ((float)moved_l / ROBOT_TICKS_PER_M_L) -
                                           ((float)moved_r / ROBOT_TICKS_PER_M_R);
            heading_error = App_WrapDegrees(Local_HeadingDeg - start_heading);
            encoder_correction = App_ClampF(APP_TUNE_ENCODER_KP * distance_error_m,
                                             -APP_TUNE_ENCODER_CORR_MAX,
                                             APP_TUNE_ENCODER_CORR_MAX);
            /* Reverse travel turns opposite for the same wheel-speed split. */
            heading_correction = APP_TUNE_HEADING_KP * heading_error;
            if (dir == MOVE_REVERSE) heading_correction = -heading_correction;
            heading_correction = App_ClampF(heading_correction,
                                             -APP_TUNE_HEADING_CORR_MAX,
                                             APP_TUNE_HEADING_CORR_MAX);
            left_pwm = (int32_t)(APP_TUNE_BASE_PWM - encoder_correction + heading_correction);
            right_pwm = (int32_t)(APP_TUNE_BASE_PWM + encoder_correction - heading_correction);
            left_pwm = (int32_t)App_ClampF((float)left_pwm, APP_TUNE_MIN_PWM, APP_TUNE_MAX_PWM);
            right_pwm = (int32_t)App_ClampF((float)right_pwm, APP_TUNE_MIN_PWM, APP_TUNE_MAX_PWM);
            (void)HAL_MOTOR_SetSpeedAndDir(MOTOR_LEFT, (MotorDirection_TypeDef)dir_l, (uint8_t)left_pwm);
            (void)HAL_MOTOR_SetSpeedAndDir(MOTOR_RIGHT, (MotorDirection_TypeDef)dir_r, (uint8_t)right_pwm);
            Local_CommandPwmL = (dir_l == MOTOR_DIR_REVERSE) ? -(int16_t)left_pwm : (int16_t)left_pwm;
            Local_CommandPwmR = (dir_r == MOTOR_DIR_REVERSE) ? -(int16_t)right_pwm : (int16_t)right_pwm;
        } else {
            /* Keep manual pivot behavior unchanged while straight tuning is evaluated. */
            left_pwm = APP_TUNE_BASE_PWM;
            right_pwm = APP_TUNE_BASE_PWM;
            (void)HAL_MOTOR_SetSpeedAndDir(MOTOR_LEFT, (MotorDirection_TypeDef)dir_l, (uint8_t)left_pwm);
            (void)HAL_MOTOR_SetSpeedAndDir(MOTOR_RIGHT, (MotorDirection_TypeDef)dir_r, (uint8_t)right_pwm);
            Local_CommandPwmL = (dir_l == MOTOR_DIR_REVERSE) ? -(int16_t)left_pwm : (int16_t)left_pwm;
            Local_CommandPwmR = (dir_r == MOTOR_DIR_REVERSE) ? -(int16_t)right_pwm : (int16_t)right_pwm;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    (void)HAL_ENCODER_GetTicks(&ticks_l, &ticks_r);
    moved_l = ticks_l - start_ticks_l;
    moved_r = ticks_r - start_ticks_r;
    heading_error = App_WrapDegrees(Local_HeadingDeg - start_heading);
    HAL_MOTOR_StopAll();
    Local_CommandPwmL = 0;
    Local_CommandPwmR = 0;
    const char *move_result = obstacle_abort ? "OBSTACLE" :
                              (calibration_abort ? "CALIBRATING" : (completed ? "DONE" : "TIMEOUT"));
    ESP_LOGI(TAG, "TUNE %s dir=%d target=%lu movedL=%lu movedR=%lu heading_delta=%.2f enc_corr=%.1f gyro_corr=%.1f",
             move_result, dir, (unsigned long)target_ticks,
             (unsigned long)moved_l, (unsigned long)moved_r, (double)heading_error,
             (double)encoder_correction, (double)heading_correction);
    {
        char report[128];
        snprintf(report, sizeof(report), "TUNE_%s,%d,%lu,%lu,%lu,%.2f",
                 move_result, dir, (unsigned long)target_ticks,
                 (unsigned long)moved_l, (unsigned long)moved_r, (double)heading_error);
        (void)SRV_UDP_Send(report);
    }
}

static const char *App_PoseStateText(SRV_POSE_State_TypeDef state)
{
    switch (state) {
        case SRV_POSE_STATE_WARMUP:       return "WARMUP";
        case SRV_POSE_STATE_CALIBRATING:  return "CALIBRATING";
        case SRV_POSE_STATE_READY:        return "READY";
        case SRV_POSE_STATE_CAL_FAILED:   return "CAL_FAILED";
        default:                          return "UNKNOWN";
    }
}

/* The IMU task is the only task that calls SRV_POSE. */
static void Task_SystemLed(void *pvParameters)
{
    bool blink_on = false;
    (void)pvParameters;
    for (;;) {
        uint8_t r = 0, g = 0, b = 0;
        EventBits_t pose_bits = (Local_PoseEvents != NULL) ? xEventGroupGetBits(Local_PoseEvents) : 0;
        if (Local_SystemFault || (pose_bits & APP_EVENT_POSE_FAILED) != 0u) {
            r = 255;
        } else if (!SRV_WIFI_IsConnected()) {
            blink_on = !blink_on;
            r = blink_on ? 255 : 0;
        } else if (!Local_UdpReady || (pose_bits & APP_EVENT_POSE_WARMUP) != 0u) {
            /* Link + command/telemetry sockets are ready; pose warm-up is still underway. */
            b = 255;
        } else if ((pose_bits & APP_EVENT_POSE_CALIBRATING) != 0u) {
            blink_on = !blink_on;
            g = blink_on ? 255 : 0;
        } else if ((pose_bits & APP_EVENT_POSE_READY) != 0u) {
            g = 255;
        } else {
            r = 255;
        }
        if (HAL_LED_SetColor(r, g, b) != FUNCTION_OK) {
            Local_SystemFault = true;
            ESP_LOGE(TAG, "FAULT: LED state update failed (ErrorState_TypeDef=%d)", (int)FUNCTION_ERROR);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void Task_IMU(void *pvParameters)
{
    MPU6050_RawData_TypeDef raw;
    TickType_t lastWakeTime = xTaskGetTickCount();
    SRV_POSE_State_TypeDef lastPublishedState = (SRV_POSE_State_TypeDef)-1;
    bool havePublishedState = false;
    bool imuErrorReported = false;
    bool haveEncoderSnapshot = false;
    uint32_t previousTicksL = 0;
    uint32_t previousTicksR = 0;
    TickType_t lastStatusTick = 0;
    int64_t lastOdom_us = 0;
    int64_t lastTelemetry_us = 0;

    (void)pvParameters;
    ESP_LOGI(TAG, "IMU task started at %u Hz; motors remain inhibited until pose is READY",
             (unsigned)(1000u / TASK_IMU_PERIOD_MS));

    for (;;) {
        EventBits_t events = xEventGroupGetBits(Local_PoseEvents);
        if ((events & APP_EVENT_RECALIBRATE) != 0u) {
            HAL_MOTOR_StopAll();
            Local_CommandPwmL = 0;
            Local_CommandPwmR = 0;
            xEventGroupClearBits(Local_PoseEvents,
                                 APP_EVENT_POSE_READY | APP_EVENT_POSE_FAILED | APP_EVENT_RECALIBRATE);
            (void)SRV_POSE_Recalibrate();
            xEventGroupClearBits(Local_PoseEvents, APP_EVENT_POSE_READY | APP_EVENT_POSE_FAILED | APP_EVENT_POSE_WARMUP);
            xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_CALIBRATING);
            havePublishedState = false;
            imuErrorReported = false;
        }

        ErrorState_TypeDef readState = MPU6050_ReadRaw(&raw);
        if (readState == FUNCTION_OK) {
            imuErrorReported = false;
            uint32_t ticksL = 0, ticksR = 0;
            (void)HAL_ENCODER_GetTicks(&ticksL, &ticksR);
            SRV_POSE_State_TypeDef beforeUpdate = SRV_POSE_GetState();
            if ((beforeUpdate == SRV_POSE_STATE_CALIBRATING) && haveEncoderSnapshot &&
                ((ticksL != previousTicksL) || (ticksR != previousTicksR))) {
                (void)SRV_POSE_AbortCalibration();
                ESP_LOGE(TAG, "Calibration rejected: encoder movement detected (ErrorState_TypeDef=%d)", (int)FUNCTION_ERROR);
            } else if (beforeUpdate != SRV_POSE_STATE_CAL_FAILED) {
                (void)SRV_POSE_UpdateImu(&raw);
            }
            if (SRV_POSE_GetState() == SRV_POSE_STATE_READY) {
                Pose_TypeDef headingSnapshot;
                SRV_POSE_GetPose(&headingSnapshot);
                Local_HeadingDeg = headingSnapshot.Theta_deg;
            }
            previousTicksL = ticksL;
            previousTicksR = ticksR;
            haveEncoderSnapshot = true;

            /* Keep all SRV_POSE calls in this one task; update odometry at 50 Hz. */
            const int64_t now_us = esp_timer_get_time();
            if ((SRV_POSE_GetState() == SRV_POSE_STATE_READY) &&
                ((lastOdom_us == 0) || ((now_us - lastOdom_us) >= 20000))) {
                ErrorState_TypeDef odomResult = SRV_POSE_UpdateOdom(
                    ticksL, ticksR, Local_CommandPwmL, Local_CommandPwmR, now_us);
                if (odomResult != FUNCTION_OK && odomResult != FUNCTION_BUSY) {
                    ESP_LOGE(TAG, "Odometry update failed (ErrorState_TypeDef=%d)", (int)odomResult);
                }
                lastOdom_us = now_us;
            }

            /* Versioned, timestamped calibration telemetry for laptop trial capture. */
            if ((SRV_POSE_GetState() == SRV_POSE_STATE_READY) &&
                ((lastTelemetry_us == 0) || ((now_us - lastTelemetry_us) >= 100000))) {
                Pose_TypeDef pose;
                SRV_POSE_Debug_TypeDef debug;
                SRV_POSE_Ablation_TypeDef ablation;
                char telemetry[180];
                char motionTelemetry[128];
                char comparisonTelemetry[240];
                SRV_POSE_GetPose(&pose);
                SRV_POSE_GetDebug(&debug);
                SRV_POSE_GetAblation(&ablation);
                snprintf(telemetry, sizeof(telemetry),
                         "TELEM1,%lld,%.4f,%.4f,%.2f,%ld,%ld,%.3f,%.3f",
                         (long long)(now_us / 1000), (double)pose.X_m, (double)pose.Y_m,
                         (double)pose.Theta_deg, (long)debug.SignedTicksL, (long)debug.SignedTicksR,
                         (double)debug.GyroZRaw_dps, (double)debug.GyroBias_dps[2]);
                (void)SRV_UDP_Send(telemetry);
                snprintf(motionTelemetry, sizeof(motionTelemetry),
                         "TELEM2,%lld,%.3f,%.2f,%.2f,%.2f,%lu,%.2f",
                         (long long)(now_us / 1000), (double)pose.V_mps,
                         (double)pose.Omega_dps, (double)pose.Pitch_deg, (double)pose.Roll_deg,
                         (unsigned long)SRV_POSE_GetFlags(),
                         (double)ablation.EncodersOnly.Theta_deg);
                (void)SRV_UDP_Send(motionTelemetry);
                snprintf(comparisonTelemetry, sizeof(comparisonTelemetry),
                         "TELEM3,%lld,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f",
                         (long long)(now_us / 1000),
                         (double)pose.X_m, (double)pose.Y_m, (double)pose.Theta_deg,
                         (double)ablation.EncodersOnly.X_m, (double)ablation.EncodersOnly.Y_m,
                         (double)ablation.EncodersOnly.Theta_deg,
                         (double)ablation.GyroBootBias.X_m, (double)ablation.GyroBootBias.Y_m,
                         (double)ablation.GyroBootBias.Theta_deg,
                         (double)ablation.ImuOnly.X_m, (double)ablation.ImuOnly.Y_m,
                         (double)ablation.ImuOnly.Theta_deg);
                (void)SRV_UDP_Send(comparisonTelemetry);
                lastTelemetry_us = now_us;
            }

            SRV_POSE_State_TypeDef state = SRV_POSE_GetState();
            TickType_t nowTick = xTaskGetTickCount();
            bool stateChanged = (!havePublishedState || (state != lastPublishedState));
            bool heartbeatDue = havePublishedState &&
                                ((nowTick - lastStatusTick) >= pdMS_TO_TICKS(1000));
            if (stateChanged || heartbeatDue) {
                char status[40];
                snprintf(status, sizeof(status), "STATUS:%s", App_PoseStateText(state));
                (void)SRV_UDP_Send(status);
                if (stateChanged) {
                    if (state == SRV_POSE_STATE_CAL_FAILED) {
                        ESP_LOGE(TAG, "Calibration failed; motors remain inhibited (ErrorState_TypeDef=%d)", (int)FUNCTION_ERROR);
                    } else {
                        ESP_LOGI(TAG, "Pose startup state: %s", App_PoseStateText(state));
                    }
                }

                xEventGroupClearBits(Local_PoseEvents, APP_EVENT_POSE_READY | APP_EVENT_POSE_FAILED |
                                     APP_EVENT_POSE_WARMUP | APP_EVENT_POSE_CALIBRATING);
                if (state == SRV_POSE_STATE_READY) {
                    xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_READY);
                } else if (state == SRV_POSE_STATE_CAL_FAILED) {
                    xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_FAILED);
                } else if (state == SRV_POSE_STATE_WARMUP) {
                    xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_WARMUP);
                } else if (state == SRV_POSE_STATE_CALIBRATING) {
                    xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_CALIBRATING);
                }
                lastPublishedState = state;
                havePublishedState = true;
                lastStatusTick = nowTick;
            }
        } else {
            xEventGroupClearBits(Local_PoseEvents, APP_EVENT_POSE_READY);
            xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_FAILED);
            (void)HAL_MOTOR_StopAll();
            if (!imuErrorReported) {
                (void)SRV_UDP_Send("STATUS:IMU_ERROR");
                ESP_LOGE(TAG, "FAULT: MPU6050 read failed (ErrorState_TypeDef=%d); motion inhibited", (int)readState);
                imuErrorReported = true;
            }
            Local_SystemFault = true;
        }

        TickType_t imuPeriodTicks = pdMS_TO_TICKS(TASK_IMU_PERIOD_MS);
        if (imuPeriodTicks == 0) imuPeriodTicks = 1; /* Prevent xTaskDelayUntil assert with a stale low tick rate. */
        vTaskDelayUntil(&lastWakeTime, imuPeriodTicks);
    }
}

static bool App_IsMotionCommand(const char *command)
{
    return (strcmp(command, "CMD_FWD") == 0) ||
           (strcmp(command, "CMD_REV") == 0) ||
           (strcmp(command, "CMD_RIGHT") == 0) ||
           (strcmp(command, "CMD_LEFT") == 0) ||
           (strcmp(command, "CAL_FWD_50CM") == 0) ||
           (strcmp(command, "CAL_REV_50CM") == 0) ||
           (strcmp(command, "CAL_PIVOT_RIGHT") == 0) ||
           (strcmp(command, "CAL_PIVOT_LEFT") == 0);
}

/* UDP receive only accepts commands. The readiness gate is authoritative on the ESP32. */
static void Task_UDP_Monitor(void *pvParameters)
{
    char rx_buffer[64];
    (void)pvParameters;

    (void)SRV_UDP_Send(Local_ImuReady ? "STATUS:WARMUP" : "STATUS:IMU_ERROR");
    (void)SRV_UDP_Send("VERSION:" APP_BUILD_VERSION);
    ESP_LOGI(TAG, "ESP32 online. UDP command port is configured in SRV_UDP_Cfg.h");

    for (;;) {
        memset(rx_buffer, 0, sizeof(rx_buffer));
        if (SRV_UDP_Receive(rx_buffer, sizeof(rx_buffer)) == FUNCTION_OK) {
            ESP_LOGI(TAG, "UDP command received: '%s'", rx_buffer);

            if (strcmp(rx_buffer, "OTA_UPDATE") == 0) {
                (void)HAL_MOTOR_StopAll();
                (void)SRV_UDP_Send("ACK:OTA_UPDATE");
                ErrorState_TypeDef ota_state = SRV_OTA_PerformUpdate();
                if (ota_state != FUNCTION_OK) {
                    (void)SRV_UDP_Send("ERROR:OTA_FAILED");
                    (void)App_LatchFault("OTA update failed", ota_state);
                }
            } else if ((strcmp(rx_buffer, "CAL") == 0) ||
                       (strcmp(rx_buffer, "CMD_CAL") == 0)) {
                (void)HAL_MOTOR_StopAll();
                Local_CommandPwmL = 0;
                Local_CommandPwmR = 0;
                if (!Local_ImuReady) {
                    (void)SRV_UDP_Send("NACK:IMU_UNAVAILABLE");
                    ESP_LOGW(TAG, "Calibration request rejected: MPU6050 is unavailable");
                    continue;
                }
                xEventGroupClearBits(Local_PoseEvents,
                                     APP_EVENT_POSE_READY | APP_EVENT_POSE_FAILED);
                xEventGroupSetBits(Local_PoseEvents, APP_EVENT_RECALIBRATE);
                (void)SRV_UDP_Send("ACK:CAL");
            } else if (App_IsMotionCommand(rx_buffer)) {
                EventBits_t events = xEventGroupGetBits(Local_PoseEvents);
                if ((events & APP_EVENT_POSE_READY) == 0u) {
                    (void)HAL_MOTOR_StopAll();
                    if ((events & APP_EVENT_POSE_FAILED) != 0u) {
                        (void)SRV_UDP_Send("NACK:CAL_FAILED");
                    } else {
                        (void)SRV_UDP_Send("NACK:CALIBRATING");
                    }
                } else {
                    char ack[80];
                    snprintf(ack, sizeof(ack), "ACK:%s", rx_buffer);
                    (void)SRV_UDP_Send(ack);

                    if (strcmp(rx_buffer, "CAL_FWD_50CM") == 0) {
                        Execute_Move(MOVE_FORWARD, APP_CAL_STRAIGHT_TICKS);
                    } else if (strcmp(rx_buffer, "CAL_REV_50CM") == 0) {
                        Execute_Move(MOVE_REVERSE, APP_CAL_STRAIGHT_TICKS);
                    } else if (strcmp(rx_buffer, "CAL_PIVOT_RIGHT") == 0) {
                        Execute_Move(MOVE_PIVOT_RIGHT, APP_CAL_PIVOT_RIGHT_TICKS);
                    } else if (strcmp(rx_buffer, "CAL_PIVOT_LEFT") == 0) {
                        Execute_Move(MOVE_PIVOT_LEFT, APP_CAL_PIVOT_LEFT_TICKS);
                    } else if (strcmp(rx_buffer, "CMD_FWD") == 0) {
                        Execute_Move(MOVE_FORWARD, APP_TAP_STRAIGHT_TICKS);
                    } else if (strcmp(rx_buffer, "CMD_REV") == 0) {
                        Execute_Move(MOVE_REVERSE, APP_TAP_STRAIGHT_TICKS);
                    } else if (strcmp(rx_buffer, "CMD_RIGHT") == 0) {
                        Execute_Move(MOVE_PIVOT_RIGHT, APP_TAP_PIVOT_TICKS);
                    } else if (strcmp(rx_buffer, "CMD_LEFT") == 0) {
                        Execute_Move(MOVE_PIVOT_LEFT, APP_TAP_PIVOT_TICKS);
                    }
                }
            } else {
                (void)SRV_UDP_Send("NACK:UNKNOWN_COMMAND");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void Task_Ultrasonic_Monitor(void *pvParameters)
{
    float distance = 0;
    bool last_blocked = false;
    int64_t last_map_report_us = 0;
    int64_t last_no_echo_report_us = 0;
    (void)pvParameters;

    while (1) {
        if (HAL_Ultrasonic_GetDistance(&distance) == FUNCTION_OK) {
            /* Trigger only at 5 cm or closer. Readings beyond this threshold
             * clear the live marker and allow forward motion. */
            Local_ObstacleBlocked = (distance <= APP_OBSTACLE_STOP_CM);
            if (Local_ObstacleBlocked && !last_blocked) {
                if ((Local_CommandPwmL > 0) && (Local_CommandPwmR > 0)) {
                    (void)HAL_MOTOR_StopAll();
                }
                (void)SRV_UDP_Send("OBSTACLE_BLOCKED");
                last_blocked = true;
            } else if (!Local_ObstacleBlocked && last_blocked) {
                (void)SRV_UDP_Send("OBSTACLE_CLEAR");
                last_blocked = false;
            }
            const int64_t now_us = esp_timer_get_time();
            if (Local_ObstacleBlocked &&
                ((last_map_report_us == 0) || ((now_us - last_map_report_us) >= 500000))) {
                char obstacle[48];
                snprintf(obstacle, sizeof(obstacle), "OBSTACLE_RANGE,%.1f", (double)distance);
                (void)SRV_UDP_Send(obstacle);
                last_map_report_us = now_us;
            }
        } else {
            const int64_t now_us = esp_timer_get_time();
            if ((last_no_echo_report_us == 0) || ((now_us - last_no_echo_report_us) >= 1000000)) {
                (void)SRV_UDP_Send("ULTRASONIC_NO_ECHO");
                last_no_echo_report_us = now_us;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

ErrorState_TypeDef APP_Orchestrator_Init(void)
{
    ErrorState_TypeDef result;
    ESP_LOGI(TAG, "========== BOOT DIAGNOSTICS BEGIN ==========");
    ESP_LOGI(TAG, "Step 1/8: initialize status LED");
    result = HAL_LED_Init();
    if (result != FUNCTION_OK) { ESP_LOGE(TAG, "BOOT FAIL: HAL_LED_Init ErrorState=%d", (int)result); return result; }
    (void)HAL_LED_SetColor(255, 0, 0);
    ESP_LOGI(TAG, "Step 1/8 PASS: LED initialized; solid red means boot");

    /* Match the previously working startup order: bring networking up first. */
    ESP_LOGI(TAG, "Step 2/8: start Wi-Fi station; configured SSID='%s'", SRV_WIFI_SSID);
    result = SRV_WIFI_Init();
    if (result != FUNCTION_OK) return App_LatchFault("SRV_WIFI_Init", result);
    ESP_LOGI(TAG, "Step 2/8 PASS: Wi-Fi driver started; waiting for GOT_IP event");

    ESP_LOGI(TAG, "Step 3/8: initialize UDP telemetry/command socket");
    result = SRV_UDP_Init();
    if (result != FUNCTION_OK) return App_LatchFault("SRV_UDP_Init", result);
    Local_UdpReady = true;
    ESP_LOGI(TAG, "Step 3/8 PASS: UDP socket bound to port 12345");

    ESP_LOGI(TAG, "Step 4/8: initialize motor HAL");
    result = HAL_MOTOR_Init();
    if (result != FUNCTION_OK) return App_LatchFault("HAL_MOTOR_Init", result);
    result = HAL_MOTOR_StopAll();
    if (result != FUNCTION_OK) return App_LatchFault("HAL_MOTOR_StopAll", result);
    ESP_LOGI(TAG, "Step 4/8 PASS: motors initialized and stopped");

    ESP_LOGI(TAG, "Step 5/8: initialize encoder HAL");
    result = HAL_ENCODER_Init();
    if (result != FUNCTION_OK) return App_LatchFault("HAL_ENCODER_Init", result);
    ESP_LOGI(TAG, "Step 5/8 PASS: encoders initialized");

    result = HAL_Ultrasonic_Init();
    if (result != FUNCTION_OK) {
        ESP_LOGW(TAG, "Ultrasonic unavailable; obstacle stop/map disabled (ErrorState=%d)", (int)result);
    } else {
        Local_UltrasonicReady = true;
        ESP_LOGI(TAG, "Ultrasonic obstacle sensor initialized");
    }

    ESP_LOGI(TAG, "Step 6/8: MPU6050 I2C port=%d SDA=GPIO%d SCL=GPIO%d address=0x%02X",
             MPU6050_I2C_PORT, MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN, MPU6050_ADDR);
    result = MPU6050_Init();
    if (result == FUNCTION_OK) {
        Local_ImuReady = true;
        ESP_LOGI(TAG, "Step 6/8 PASS: MPU6050 initialized and identity/configuration checked");
    } else {
        /* Keep Wi-Fi/UDP/OTA available for diagnosis and recovery. Never enable motors. */
        Local_ImuReady = false;
        Local_SystemFault = true;
        ESP_LOGE(TAG, "DEGRADED STARTUP: MPU6050_Init ErrorState=%d; telemetry/OTA stay active, motion is locked", (int)result);
        (void)SRV_UDP_Send("STATUS:IMU_ERROR");
    }

    ESP_LOGI(TAG, "Step 7/8: initialize pose estimator and calibration state");
    Local_PoseEvents = xEventGroupCreate();
    if (Local_PoseEvents == NULL) return App_LatchFault("xEventGroupCreate", FUNCTION_ERROR);
    if (Local_ImuReady) {
        result = SRV_POSE_Init();
        if (result != FUNCTION_OK) {
            Local_ImuReady = false;
            Local_SystemFault = true;
            ESP_LOGE(TAG, "DEGRADED STARTUP: SRV_POSE_Init ErrorState=%d; telemetry/OTA stay active, motion is locked", (int)result);
            xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_FAILED);
        } else {
            xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_WARMUP);
            ESP_LOGI(TAG, "Step 7/8 PASS: pose service initialized");
        }
    } else {
        xEventGroupSetBits(Local_PoseEvents, APP_EVENT_POSE_FAILED);
        ESP_LOGW(TAG, "Pose service skipped because MPU6050 initialization failed");
    }
    ESP_LOGI(TAG, "Step 8/8 PASS: all application initialization stages complete");
    ESP_LOGI(TAG, "========== BOOT DIAGNOSTICS COMPLETE ==========");
    ESP_LOGI(TAG, "FIRMWARE_VERSION=%s", APP_BUILD_VERSION);
    return FUNCTION_OK;
}

ErrorState_TypeDef APP_Orchestrator_StartTasks(void)
{
    if (Local_PoseEvents == NULL) return App_LatchFault("Pose event group unavailable", FUNCTION_ERROR);

    ESP_LOGI(TAG, "Creating System_LED task");
    if (xTaskCreate(Task_SystemLed, "System_LED", TASK_STACK_LED_STATE, NULL,
                    TASK_PRIO_LED_STATE, NULL) != pdPASS) return App_LatchFault("System LED task creation failed", FUNCTION_ERROR);
    if (Local_ImuReady) {
        ESP_LOGI(TAG, "Creating IMU_Pose task at %u Hz", (unsigned)(1000u / TASK_IMU_PERIOD_MS));
        if (xTaskCreate(Task_IMU, "IMU_Pose", TASK_STACK_SENSOR_PID, NULL,
                        TASK_PRIO_SENSOR_PID, NULL) != pdPASS) {
            return App_LatchFault("IMU task creation failed", FUNCTION_ERROR);
        }
    } else {
        ESP_LOGW(TAG, "IMU task not started; boot continues in safe network-only mode");
    }
    ESP_LOGI(TAG, "Creating UDP_Monitor task");
    if (xTaskCreate(Task_UDP_Monitor, "UDP_Monitor", TASK_STACK_UDP_TELEMETRY, NULL,
                    TASK_PRIO_UDP_TELEMETRY, NULL) != pdPASS) {
        (void)HAL_MOTOR_StopAll();
        return App_LatchFault("UDP task creation failed", FUNCTION_ERROR);
    }

    ESP_LOGI(TAG, "All application tasks started successfully");
    if (Local_UltrasonicReady &&
        xTaskCreate(Task_Ultrasonic_Monitor, "Ultrasonic", TASK_STACK_UDP_TELEMETRY, NULL,
                    TASK_PRIO_UDP_TELEMETRY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Ultrasonic task creation failed; forward obstacle protection unavailable");
        Local_UltrasonicReady = false;
    }
    return FUNCTION_OK;
}
