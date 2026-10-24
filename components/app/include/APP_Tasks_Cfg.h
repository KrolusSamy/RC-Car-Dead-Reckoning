/*------------------------------------------------------------------------------------------------------*-
* File Name          : APP_Tasks_Cfg.h
* Author             : Tele-Operated Robot Team
* Date First Issued  : 10-2026
* Description        : Centralized FreeRTOS task priorities, stack sizes, and core affinities.
 -------------------------------------------------------------------------------------------------------
-See APP_Orchestrator_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

#ifndef APP_TASKS_CFG_H_
#define APP_TASKS_CFG_H_

// ------------------------------------ Public/Exported constant - Macros ------------------------------

/* Task Priorities (Higher Number = Higher Priority) */
#define TASK_PRIO_ENCODER_PROC    20  // For deferred interrupt processing if needed
#define TASK_PRIO_SENSOR_PID      15  // 200 Hz MPU6050 sampling and pose calibration
#define TASK_PRIO_UDP_TELEMETRY   10  // Wi-Fi UDP packet transmission
#define TASK_PRIO_WIFI_MGR        5   // Wi-Fi event handling
#define TASK_PRIO_LED_STATE       2   // Visual status indication
#define TASK_PRIO_OTA_UPDATE      1   // Background firmware downloads

/* Task Stack Sizes (in Bytes for ESP-IDF FreeRTOS) */
#define TASK_STACK_SENSOR_PID     4096
#define TASK_IMU_PERIOD_MS        5   // 200 Hz sample period
#define TASK_STACK_UDP_TELEMETRY  4096
#define TASK_STACK_OTA_UPDATE     8192 // OTA requires larger stack for HTTP/Flash operations
#define TASK_STACK_LED_STATE      2048

#endif /* APP_TASKS_CFG_H_ */
