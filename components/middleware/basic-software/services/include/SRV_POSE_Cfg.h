/*------------------------------------------------------------------------------------------------------*-
* File Name          : SRV_POSE_Cfg.h
* Author             : 
* Date First Issued  : Oct 2026
* Description        : All tunables of the SRV_POSE service (dead-reckoning estimator).
 -------------------------------------------------------------------------------------------------------
-See SRV_POSE_Prog.c for details
-*-----------------------------------------------------------------------------------------------------*/

// ------------------------------------- Define Guardian to prevent recursive inclusion ----------------
#ifndef SRV_POSE_CFG_H_
#define SRV_POSE_CFG_H_

// ------------------------------------ Includes -------------------------------------------------------
#include "Robot_Params.h"

// ------------------------------------ Public/Exported constant - Macros ------------------------------

/* ---------------------------- Boot calibration ---------------------------- */
#define SRV_POSE_WARMUP_MS               5000      /* Use 120000..300000 for real runs (MPU6050 bias moves while it warms up) */
#define SRV_POSE_CAL_SAMPLES             1000      /* 5 s at 200 Hz                                                            */
#define SRV_POSE_CAL_MAX_STD_DPS         0.5f      /* Reject calibration if the car was touched                                */
#define SRV_POSE_CAL_MAX_MEAN_DPS        20.0f     /* MPU6050 zero-rate offset limit                                           */
#define SRV_POSE_CAL_ACCEL_MIN_G         0.7f      /* Sanity check of |accel| during calibration                               */
#define SRV_POSE_CAL_ACCEL_MAX_G         1.3f
#define SRV_POSE_CAL_ACCEL_MAX_STD_G     0.03f     /* Reject vibration/touching during the still calibration window           */

/* ---------------------------- Mounting / sign conventions ----------------- */
/* Physical mounting: +Y forward, +X right, +Z up. SRV_POSE remaps X/Y for
   the tilt filter. Verify on the bench: CCW turn -> heading increases;
   nose up -> Pitch increases; left side up -> Roll increases.                                  */
#define SRV_POSE_YAW_SIGN                1.0f
#define SRV_POSE_PITCH_SIGN              (-1.0f)
#define SRV_POSE_ROLL_SIGN               1.0f

/* Accelerometer calibration (g): a = (raw/LSB - OFFSET) * SCALE. Produce with the accel_calib test */
#define SRV_POSE_ACCEL_OFFSET_X_G  0.090149f
#define SRV_POSE_ACCEL_SCALE_X     1.00633f
#define SRV_POSE_ACCEL_OFFSET_Y_G -0.017029f
#define SRV_POSE_ACCEL_SCALE_Y     0.99872f
#define SRV_POSE_ACCEL_OFFSET_Z_G  0.035706f
#define SRV_POSE_ACCEL_SCALE_Z     0.98967f

/* ---------------------------- Tilt filter (Madgwick 6-DoF) ---------------- */
#define SRV_POSE_MADGWICK_BETA           0.05f
#define SRV_POSE_ACCEL_GATE_MIN_G        0.8f      /* Outside this band the accel is not trusted (bump / acceleration) */
#define SRV_POSE_ACCEL_GATE_MAX_G        1.2f
#define SRV_POSE_PITCH_LPF_ALPHA         0.13f     /* About 5 Hz at 200 Hz, used for the event decisions                */
#define SRV_POSE_DT_MAX_US               (5u * MPU6050_SAMPLE_PERIOD_US)   /* Longer gap = missed samples            */

/* ---------------------------- Stationary detector (ZUPT) ------------------ */
#define SRV_POSE_STAT_WINDOW_SAMPLES     (MPU6050_SAMPLE_RATE_HZ / 2)      /* 0.5 s                                  */
#define SRV_POSE_STAT_SETTLE_US          300000    /* Wait after the last motion/command before trusting the gyro      */
#define SRV_POSE_STAT_GYRO_STD_DPS       0.3f      /* Max gyro noise std in the window (all axes)                      */
#define SRV_POSE_STAT_MEAN_DEV_DPS       1.0f      /* Max |window mean - current bias|: rejects slow real rotation     */
#define SRV_POSE_STAT_ACCEL_STD_G        0.03f
#define SRV_POSE_STAT_BIAS_ALPHA         0.3f      /* Fraction of the error removed per stationary window              */
#define SRV_POSE_STAT_INSTANT_ROT_DPS    2.0f      /* Any faster yaw clears "stationary" immediately                   */

/* ---------------------------- Odometry ------------------------------------ */
#define SRV_POSE_REVERSAL_HOLD_US        120000    /* Keep the old direction while a spinning wheel reverses           */
#define SRV_POSE_IDLE_SWITCH_US          150000    /* A wheel with no ticks for this long takes the new direction now  */
#define SRV_POSE_MAX_TICKS_STEP          500       /* A bigger jump = counter was reset elsewhere: rebase              */
#define SRV_POSE_SPEED_WINDOW_SAMPLES    5         /* Speed over this many odom intervals (5 x 20 ms = 100 ms)         */
#define SRV_POSE_ABLATION_ENABLE         1         /* Also run encoder-only and raw-gyro tracks for the demo           */

/* ---------------------------- Straight-drive bias update (optional) ------- */
/* Compares the gyro against the encoder turn rate on long straight segments.
   It needs well calibrated per-wheel ticks/m (a 1% mismatch fakes about 1 dps), so keep it OFF until then. */
#define SRV_POSE_STRAIGHT_BIAS_ENABLE    0
#define SRV_POSE_STRAIGHT_MIN_SEG_S      6.0f
#define SRV_POSE_STRAIGHT_MIN_SPEED_MPS  0.08f
#define SRV_POSE_STRAIGHT_MAX_OMEGA_DPS  10.0f
#define SRV_POSE_STRAIGHT_GAIN           0.3f
#define SRV_POSE_STRAIGHT_MAX_ERR_DPS    1.0f

/* ---------------------------- Event detection ----------------------------- */
#define SRV_POSE_RAMP_ENTER_DEG          8.0f
#define SRV_POSE_RAMP_EXIT_DEG           5.0f
#define SRV_POSE_TILT_ENTER_DEG          10.0f
#define SRV_POSE_TILT_EXIT_DEG           6.0f
#define SRV_POSE_HOLD_US                 500000    /* Condition must persist this long to raise a ramp/tilt flag       */
#define SRV_POSE_STEEP_DEG               25.0f
#define SRV_POSE_WALL_PITCH_MAX_DEG      5.0f
#define SRV_POSE_SPIKE_G                 0.40f     /* | |a| - 1g | above this = impact                                  */
#define SRV_POSE_SPIKE_BEFORE_STALL_US   700000    /* Impact may precede the last wheel tick by this much (wheels coast after a hit) */
#define SRV_POSE_STALL_PWM_MIN           120       /* Command magnitude that counts as "trying to move"                */
#define SRV_POSE_STALL_US                400000    /* No ticks for this long while commanded = stall                   */
#define SRV_POSE_STUCK_US                1500000   /* Stall with no impact and flat pitch for this long = stuck        */

#endif /* SRV_POSE_CFG_H_ */

/*-------------------------------------------------------------------------------------------------------------------*-
----------------------------------------------------- END OF FILE -----------------------------------------------------
-*-------------------------------------------------------------------------------------------------------------------*/
