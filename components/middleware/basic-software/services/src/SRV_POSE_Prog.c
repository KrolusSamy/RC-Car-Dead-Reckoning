/*-----------------------------------------------------------------------------------------------------*-
 * File Name          : SRV_POSE_Prog.c
 * Author             : 
 * Date First Issued  : Oct 2026
 * Description        : Dead-reckoning estimator. See SRV_POSE_Interface.h.
 *
 *   Heading   : theta += (gz - bias) * dt          bias = boot average, then re-measured at every stop
 *   Tilt      : Madgwick 6-DoF (pitch/roll only, its yaw is never used)
 *   Position  : x += d*cos(pitch)*cos(theta_mid)   d = mean of the two signed wheel distances
 *   Speed     : encoder distance over a 100 ms window, forced to 0 when the IMU says "stationary"
 *   Events    : ramp / tilt from filtered pitch+roll; stall / wall / steep / stuck from encoders+accel
--------------------------------------------------------------------------------------------------------
-See SRV_POSE_Interface.h and SRV_POSE_Cfg.h for details
-*-----------------------------------------------------------------------------------------------------*/

// --------------------------------- Includes -----------------------------------------------------
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "SRV_POSE_Interface.h"


// --------------------------------- Private macros -----------------------------------------------
#define POSE_RAD2DEG        57.29577951f
#define POSE_DEG2RAD        0.01745329252f
#define POSE_RING_SIZE      (SRV_POSE_SPEED_WINDOW_SAMPLES + 1)
#define POSE_WARMUP_US      ((int64_t)SRV_POSE_WARMUP_MS * 1000)

#define POSE_LEFT           0
#define POSE_RIGHT          1


// --------------------------------- Private types ------------------------------------------------
typedef struct { uint32_t n; float mean; float m2; } Welford_t;     /* Running mean/variance, no cancellation */
typedef struct { int64_t since_us; bool active; }    Hold_t;

typedef struct
{
    SRV_POSE_State_TypeDef State;

    /* IMU timing */
    bool     HaveLastImu;
    int64_t  LastImuTime_us;
    bool     WarmupStarted;
    int64_t  WarmupStart_us;
    uint32_t MissedSamples;

    /* Boot calibration */
    Welford_t CalG[3];
    Welford_t CalA[3];
    Welford_t CalT;

    /* Gyro / tilt */
    float    GyroBias[3];
    float    BootBias[3];
    float    Q[4];
    float    PitchDeg, RollDeg, PitchLpfDeg;
    float    GzRaw_dps, Omega_dps, Temp_C;

    /* Headings (unwrapped, degrees) */
    float    Theta, ThetaBoot, ThetaEnc;
    float    PendingGyroDeg;                 /* Gyro heading change since the last odom call */

    /* Stationary detector */
    Welford_t WinG[3];
    Welford_t WinA;
    bool     Stationary;
    int64_t  LastMotion_us;
    int64_t  LastSpike_us;

    /* Odometry */
    bool     OdomInit;
    uint32_t PrevTicks[2];
    int64_t  PrevOdomTime_us;
    int8_t   CmdDir[2], ActiveDir[2];
    int64_t  CmdDirSince_us[2], LastTick_us[2], CmdActiveSince_us[2];
    int32_t  SignedTicks[2];
    float    Dist[2];
    int64_t  RingTime[POSE_RING_SIZE];
    float    RingDist[2][POSE_RING_SIZE];
    uint8_t  RingHead, RingCount;
    float    X, Y, Z;
    float    PrevThetaFull, PrevThetaBoot, PrevThetaEnc;
    SRV_POSE_Track_TypeDef TrackEnc, TrackBoot;
    SRV_POSE_Track_TypeDef TrackImuOnly;
    float    ImuOnlyVx, ImuOnlyVy, ImuOnlyPrevAx, ImuOnlyPrevAy;
    bool     ImuOnlyHavePrevAccel;
    float    V, VL, VR;

    /* Straight-drive bias segment */
    float    SegTime, SegResid;

    /* Events */
    Hold_t   RampUp, RampDown, TiltL, TiltR;
    uint32_t Flags, NewEvents;
} SRV_POSE_Ctx_t;

static SRV_POSE_Ctx_t Local_Ctx;


// --------------------------------- Private functions prototypes ---------------------------------
static void  Welford_Reset(Welford_t *Copy_pW);
static void  Welford_Add(Welford_t *Copy_pW, float Copy_fX);
static float Welford_Var(const Welford_t *Copy_pW);
static float Pose_Wrap180(float Copy_fAngle);
static void  Pose_Madgwick6(float Copy_fGx, float Copy_fGy, float Copy_fGz,
                            float Copy_fAx, float Copy_fAy, float Copy_fAz, float Copy_fDt, float Copy_fBeta);
static void  Pose_QuatFromAccel(float Copy_fAx, float Copy_fAy, float Copy_fAz);
static void  Pose_UpdateAngles(void);
static void  Pose_IntegrateImuOnly(float Copy_fAx, float Copy_fAy, float Copy_fAz, float Copy_fDt);
static void  Pose_CalibrationStart(void);
static void  Pose_CalibrationFinish(void);
static void  Pose_StationaryStep(const float Copy_fG[3], float Copy_fAmag, float Copy_fGz, int64_t Copy_s64Now);
static bool  Pose_Hold(Hold_t *Copy_pH, bool Copy_bEnter, bool Copy_bExit, int64_t Copy_s64Now, int64_t Copy_s64HoldUs);


// --------------------------------  Public/Exported functions definitions ------------------------

/*------------------------------------------------------------------------------------------------*/

ErrorState_TypeDef SRV_POSE_Init(void)
{
    memset(&Local_Ctx, 0, sizeof(Local_Ctx));
    Local_Ctx.State     = SRV_POSE_STATE_WARMUP;
    Local_Ctx.Q[0]      = 1.0f;
    Local_Ctx.CmdDir[POSE_LEFT]    = Local_Ctx.CmdDir[POSE_RIGHT]    = 1;
    Local_Ctx.ActiveDir[POSE_LEFT] = Local_Ctx.ActiveDir[POSE_RIGHT] = 1;
    return FUNCTION_OK;
}

ErrorState_TypeDef SRV_POSE_Recalibrate(void)
{
    Pose_CalibrationStart();
    return FUNCTION_OK;
}

ErrorState_TypeDef SRV_POSE_AbortCalibration(void)
{
    if (Local_Ctx.State != SRV_POSE_STATE_CALIBRATING)
    {
        return FUNCTION_ERROR;
    }
    Local_Ctx.State = SRV_POSE_STATE_CAL_FAILED;
    return FUNCTION_OK;
}

ErrorState_TypeDef SRV_POSE_UpdateImu(const MPU6050_RawData_TypeDef *Copy_pRaw)
{
    if (Copy_pRaw == NULL)
    {
        return FUNCTION_ERROR;
    }

    const int64_t Local_Now = Copy_pRaw->Timestamp_us;

    /* ---- Raw -> physical units ---- */
    float Local_G[3] =
    {
        (float)Copy_pRaw->Gyro_X / MPU6050_GYRO_LSB_PER_DPS,
        (float)Copy_pRaw->Gyro_Y / MPU6050_GYRO_LSB_PER_DPS,
        (float)Copy_pRaw->Gyro_Z / MPU6050_GYRO_LSB_PER_DPS
    };
    float Local_A[3] =
    {
        ((float)Copy_pRaw->Accel_X / MPU6050_ACCEL_LSB_PER_G - SRV_POSE_ACCEL_OFFSET_X_G) * SRV_POSE_ACCEL_SCALE_X,
        ((float)Copy_pRaw->Accel_Y / MPU6050_ACCEL_LSB_PER_G - SRV_POSE_ACCEL_OFFSET_Y_G) * SRV_POSE_ACCEL_SCALE_Y,
        ((float)Copy_pRaw->Accel_Z / MPU6050_ACCEL_LSB_PER_G - SRV_POSE_ACCEL_OFFSET_Z_G) * SRV_POSE_ACCEL_SCALE_Z
    };
    Local_Ctx.Temp_C    = ((float)Copy_pRaw->Temp / 340.0f) + 36.53f;   /* Datasheet formula; no HAL call from a service */
    Local_Ctx.GzRaw_dps = Local_G[2];

    /* ---- dt from the sample timestamps ---- */
    float Local_Dt = (float)MPU6050_SAMPLE_PERIOD_US * 1e-6f;
    if (Local_Ctx.HaveLastImu)
    {
        const int64_t Local_Delta = Local_Now - Local_Ctx.LastImuTime_us;
        if ((Local_Delta <= 0) || (Local_Delta > (int64_t)SRV_POSE_DT_MAX_US))
        {
            Local_Ctx.MissedSamples++;          /* Keep the nominal dt for this step */
        }
        else
        {
            Local_Dt = (float)Local_Delta * 1e-6f;
        }
    }
    Local_Ctx.HaveLastImu    = true;
    Local_Ctx.LastImuTime_us = Local_Now;

    /* ---- State machine ---- */
    switch (Local_Ctx.State)
    {
        case SRV_POSE_STATE_WARMUP:
            if (!Local_Ctx.WarmupStarted)
            {
                Local_Ctx.WarmupStarted  = true;
                Local_Ctx.WarmupStart_us = Local_Now;
            }
            if ((Local_Now - Local_Ctx.WarmupStart_us) >= POSE_WARMUP_US)
            {
                Pose_CalibrationStart();
            }
            return FUNCTION_BUSY;

        case SRV_POSE_STATE_CALIBRATING:
            for (uint8_t i = 0; i < 3; i++)
            {
                Welford_Add(&Local_Ctx.CalG[i], Local_G[i]);
                Welford_Add(&Local_Ctx.CalA[i], Local_A[i]);
            }
            Welford_Add(&Local_Ctx.CalT, Local_Ctx.Temp_C);
            if (Local_Ctx.CalG[0].n >= SRV_POSE_CAL_SAMPLES)
            {
                Pose_CalibrationFinish();
            }
            return (Local_Ctx.State == SRV_POSE_STATE_CAL_FAILED) ? FUNCTION_ERROR : FUNCTION_BUSY;

        case SRV_POSE_STATE_CAL_FAILED:
            return FUNCTION_ERROR;

        case SRV_POSE_STATE_READY:
        default:
            break;
    }

    /* ---- READY: bias-corrected rates ---- */
    const float Local_Gc[3] = { Local_G[0] - Local_Ctx.GyroBias[0],
                                Local_G[1] - Local_Ctx.GyroBias[1],
                                Local_G[2] - Local_Ctx.GyroBias[2] };
    const float Local_Gz = SRV_POSE_YAW_SIGN * Local_Gc[2];

    /* ---- Tilt (accel is ignored while it is not about 1 g: bumps, hard acceleration) ---- */
    const float Local_Amag = sqrtf(Local_A[0] * Local_A[0] + Local_A[1] * Local_A[1] + Local_A[2] * Local_A[2]);
    const float Local_Beta = ((Local_Amag >= SRV_POSE_ACCEL_GATE_MIN_G) && (Local_Amag <= SRV_POSE_ACCEL_GATE_MAX_G))
                             ? SRV_POSE_MADGWICK_BETA : 0.0f;
    /* Vehicle mounting: +Y is forward and +X is right. Madgwick Euler math
     * below expects +X forward and +Y left, so remap the horizontal axes.
     * Z yaw is unchanged and remains the heading integration axis. */
    Pose_Madgwick6(Local_Gc[1] * POSE_DEG2RAD, -Local_Gc[0] * POSE_DEG2RAD, Local_Gc[2] * POSE_DEG2RAD,
                   Local_A[1], -Local_A[0], Local_A[2], Local_Dt, Local_Beta);
    Pose_UpdateAngles();
    /* Diagnostic only: estimate planar translation using the IMU alone.
     * This intentionally has no encoder input or zero-velocity correction. */
    Pose_IntegrateImuOnly(Local_A[1], -Local_A[0], Local_A[2], Local_Dt);
    Local_Ctx.PitchLpfDeg += SRV_POSE_PITCH_LPF_ALPHA * (Local_Ctx.PitchDeg - Local_Ctx.PitchLpfDeg);

    /* ---- Impact spike (used with the stall flag to call a wall) ---- */
    if (fabsf(Local_Amag - 1.0f) > SRV_POSE_SPIKE_G)
    {
        Local_Ctx.LastSpike_us = Local_Now;
    }

    /* ---- Stationary detector / bias tracking ---- */
    Pose_StationaryStep(Local_G, Local_Amag, Local_Gz, Local_Now);

    /* ---- Heading. While stationary the residual rate is bias/noise: do not integrate it ---- */
    if (!Local_Ctx.Stationary)
    {
        const float Local_dTheta = Local_Gz * Local_Dt;
        Local_Ctx.Theta          += Local_dTheta;
        Local_Ctx.PendingGyroDeg += Local_dTheta;
    }
    /* Baseline for the demo: boot bias only, no tracking, no freeze */
    Local_Ctx.ThetaBoot += SRV_POSE_YAW_SIGN * (Local_G[2] - Local_Ctx.BootBias[2]) * Local_Dt;

    Local_Ctx.Omega_dps = Local_Ctx.Stationary ? 0.0f : Local_Gz;
    return FUNCTION_OK;
}

ErrorState_TypeDef SRV_POSE_UpdateOdom(uint32_t Copy_u32TicksL, uint32_t Copy_u32TicksR,
                                       int16_t Copy_s16PwmL, int16_t Copy_s16PwmR, int64_t Copy_s64Time_us)
{
    const uint32_t Local_TicksNow[2] = { Copy_u32TicksL, Copy_u32TicksR };
    const int16_t  Local_Pwm[2]      = { Copy_s16PwmL,  Copy_s16PwmR };

    /* Baseline: first call, or while the estimator is not ready (motors must not run then) */
    if ((Local_Ctx.State != SRV_POSE_STATE_READY) || (!Local_Ctx.OdomInit))
    {
        Local_Ctx.PrevTicks[POSE_LEFT]  = Local_TicksNow[POSE_LEFT];
        Local_Ctx.PrevTicks[POSE_RIGHT] = Local_TicksNow[POSE_RIGHT];
        Local_Ctx.PrevOdomTime_us       = Copy_s64Time_us;
        Local_Ctx.OdomInit              = (Local_Ctx.State == SRV_POSE_STATE_READY);
        return (Local_Ctx.State == SRV_POSE_STATE_READY) ? FUNCTION_OK : FUNCTION_BUSY;
    }

    const float Local_DtO = (float)(Copy_s64Time_us - Local_Ctx.PrevOdomTime_us) * 1e-6f;
    if (Local_DtO <= 0.0f)
    {
        return FUNCTION_OK;
    }
    Local_Ctx.PrevOdomTime_us = Copy_s64Time_us;

    float   Local_dDist[2];
    bool    Local_Stalled[2] = { false, false };
    int64_t Local_StallStart = INT64_MAX;
    bool    Local_AnyMotion  = false;

    for (uint8_t w = 0; w < 2; w++)
    {
        /* Counter jump (someone reset it): rebase instead of integrating garbage */
        int32_t Local_Step = (int32_t)(Local_TicksNow[w] - Local_Ctx.PrevTicks[w]);
        Local_Ctx.PrevTicks[w] = Local_TicksNow[w];
        if ((Local_Step < 0) || (Local_Step > SRV_POSE_MAX_TICKS_STEP))
        {
            Local_Step = 0;
        }
        if (Local_Step > 0)
        {
            Local_Ctx.LastTick_us[w] = Copy_s64Time_us;
            Local_AnyMotion = true;
        }
        if (Local_Pwm[w] != 0)
        {
            Local_AnyMotion = true;
        }

        /* Direction of the (unsigned) ticks = direction of the last non-zero command, with a short hold
           while a spinning wheel is being reversed (it still turns the old way for a moment) */
        const int8_t Local_Sgn = (Local_Pwm[w] > 0) ? 1 : ((Local_Pwm[w] < 0) ? -1 : 0);
        if ((Local_Sgn != 0) && (Local_Sgn != Local_Ctx.CmdDir[w]))
        {
            Local_Ctx.CmdDir[w]       = Local_Sgn;
            Local_Ctx.CmdDirSince_us[w] = Copy_s64Time_us;
        }
        if (Local_Ctx.ActiveDir[w] != Local_Ctx.CmdDir[w])
        {
            if (((Copy_s64Time_us - Local_Ctx.CmdDirSince_us[w]) >= SRV_POSE_REVERSAL_HOLD_US) ||
                ((Copy_s64Time_us - Local_Ctx.LastTick_us[w])    >= SRV_POSE_IDLE_SWITCH_US))
            {
                Local_Ctx.ActiveDir[w] = Local_Ctx.CmdDir[w];
            }
        }

        const int32_t Local_Signed = Local_Step * Local_Ctx.ActiveDir[w];
        Local_Ctx.SignedTicks[w] += Local_Signed;
        Local_dDist[w] = (float)Local_Signed / ((w == POSE_LEFT) ? ROBOT_TICKS_PER_M_L : ROBOT_TICKS_PER_M_R);
        Local_Ctx.Dist[w] += Local_dDist[w];

        /* Stall bookkeeping: commanded hard, but no tick for SRV_POSE_STALL_US */
        const bool Local_Trying = (abs((int)Local_Pwm[w]) >= SRV_POSE_STALL_PWM_MIN);
        if (Local_Trying)
        {
            if (Local_Ctx.CmdActiveSince_us[w] == 0)
            {
                Local_Ctx.CmdActiveSince_us[w] = Copy_s64Time_us;
            }
            const int64_t Local_Ref = (Local_Ctx.LastTick_us[w] > Local_Ctx.CmdActiveSince_us[w])
                                      ? Local_Ctx.LastTick_us[w] : Local_Ctx.CmdActiveSince_us[w];
            if ((Copy_s64Time_us - Local_Ref) >= SRV_POSE_STALL_US)
            {
                Local_Stalled[w] = true;
                if (Local_Ref < Local_StallStart)
                {
                    Local_StallStart = Local_Ref;
                }
            }
        }
        else
        {
            Local_Ctx.CmdActiveSince_us[w] = 0;
        }
    }

    if (Local_AnyMotion)
    {
        Local_Ctx.LastMotion_us = Copy_s64Time_us;      /* Tells the stationary detector to wait */
    }

    /* ---- Kinematics ---- */
    const float Local_d       = 0.5f * (Local_dDist[POSE_LEFT] + Local_dDist[POSE_RIGHT]);
    const float Local_dThEnc  = ((Local_dDist[POSE_RIGHT] - Local_dDist[POSE_LEFT]) / ROBOT_TRACK_WIDTH_M) * POSE_RAD2DEG;
    Local_Ctx.ThetaEnc += Local_dThEnc;

    const float Local_PitchRad = Local_Ctx.PitchLpfDeg * POSE_DEG2RAD;
    const float Local_dH       = Local_d * cosf(Local_PitchRad);      /* Horizontal part of the travelled distance */
    const float Local_MidFull  = 0.5f * (Local_Ctx.PrevThetaFull + Local_Ctx.Theta)     * POSE_DEG2RAD;
    Local_Ctx.X += Local_dH * cosf(Local_MidFull);
    Local_Ctx.Y += Local_dH * sinf(Local_MidFull);
    Local_Ctx.Z += Local_d  * sinf(Local_PitchRad);

#if SRV_POSE_ABLATION_ENABLE
    const float Local_MidEnc  = 0.5f * (Local_Ctx.PrevThetaEnc  + Local_Ctx.ThetaEnc)  * POSE_DEG2RAD;
    const float Local_MidBoot = 0.5f * (Local_Ctx.PrevThetaBoot + Local_Ctx.ThetaBoot) * POSE_DEG2RAD;
    Local_Ctx.TrackEnc.X_m  += Local_dH * cosf(Local_MidEnc);
    Local_Ctx.TrackEnc.Y_m  += Local_dH * sinf(Local_MidEnc);
    Local_Ctx.TrackBoot.X_m += Local_dH * cosf(Local_MidBoot);
    Local_Ctx.TrackBoot.Y_m += Local_dH * sinf(Local_MidBoot);
#endif
    const float Local_GyroDeg = Local_Ctx.PendingGyroDeg;
    Local_Ctx.PrevThetaFull = Local_Ctx.Theta;
    Local_Ctx.PrevThetaEnc  = Local_Ctx.ThetaEnc;
    Local_Ctx.PrevThetaBoot = Local_Ctx.ThetaBoot;
    Local_Ctx.PendingGyroDeg = 0.0f;

    /* ---- Wheel speeds and velocity: distance over a sliding window ---- */
    Local_Ctx.RingTime[Local_Ctx.RingHead]            = Copy_s64Time_us;
    Local_Ctx.RingDist[POSE_LEFT][Local_Ctx.RingHead]  = Local_Ctx.Dist[POSE_LEFT];
    Local_Ctx.RingDist[POSE_RIGHT][Local_Ctx.RingHead] = Local_Ctx.Dist[POSE_RIGHT];
    const uint8_t Local_Newest = Local_Ctx.RingHead;
    Local_Ctx.RingHead = (uint8_t)((Local_Ctx.RingHead + 1u) % POSE_RING_SIZE);
    if (Local_Ctx.RingCount < POSE_RING_SIZE)
    {
        Local_Ctx.RingCount++;
    }
    const uint8_t Local_Oldest = (uint8_t)((Local_Ctx.RingHead + POSE_RING_SIZE - Local_Ctx.RingCount) % POSE_RING_SIZE);
    const float   Local_Span   = (float)(Local_Ctx.RingTime[Local_Newest] - Local_Ctx.RingTime[Local_Oldest]) * 1e-6f;
    if (Local_Span > 0.02f)
    {
        Local_Ctx.VL = (Local_Ctx.RingDist[POSE_LEFT][Local_Newest]  - Local_Ctx.RingDist[POSE_LEFT][Local_Oldest])  / Local_Span;
        Local_Ctx.VR = (Local_Ctx.RingDist[POSE_RIGHT][Local_Newest] - Local_Ctx.RingDist[POSE_RIGHT][Local_Oldest]) / Local_Span;
    }
    if (Local_Ctx.Stationary)           /* ZUPT on velocity: the IMU says we are not moving */
    {
        Local_Ctx.VL = 0.0f;
        Local_Ctx.VR = 0.0f;
    }
    Local_Ctx.V = 0.5f * (Local_Ctx.VL + Local_Ctx.VR);

#if SRV_POSE_STRAIGHT_BIAS_ENABLE
    /* ---- Gyro bias from "the encoders say we are driving straight" (long segments only) ---- */
    const bool Local_SameDir  = ((Local_Pwm[POSE_LEFT] > 0) && (Local_Pwm[POSE_RIGHT] > 0)) ||
                                ((Local_Pwm[POSE_LEFT] < 0) && (Local_Pwm[POSE_RIGHT] < 0));
    const bool Local_Straight = Local_SameDir &&
                                (fabsf(Local_Ctx.VL) > SRV_POSE_STRAIGHT_MIN_SPEED_MPS) &&
                                (fabsf(Local_Ctx.VR) > SRV_POSE_STRAIGHT_MIN_SPEED_MPS) &&
                                (fabsf(Local_Ctx.Omega_dps) < SRV_POSE_STRAIGHT_MAX_OMEGA_DPS) &&
                                !Local_Stalled[POSE_LEFT] && !Local_Stalled[POSE_RIGHT];
    if (Local_Straight)
    {
        Local_Ctx.SegTime  += Local_DtO;
        Local_Ctx.SegResid += Local_GyroDeg - Local_dThEnc;
        if (Local_Ctx.SegTime >= SRV_POSE_STRAIGHT_MIN_SEG_S)
        {
            const float Local_Err = Local_Ctx.SegResid / Local_Ctx.SegTime;     /* Yaw-rate units */
            if (fabsf(Local_Err) < SRV_POSE_STRAIGHT_MAX_ERR_DPS)
            {
                Local_Ctx.GyroBias[2] += SRV_POSE_STRAIGHT_GAIN * Local_Err * SRV_POSE_YAW_SIGN;
            }
            Local_Ctx.SegTime = 0.0f;
            Local_Ctx.SegResid = 0.0f;
        }
    }
    else
    {
        Local_Ctx.SegTime = 0.0f;
        Local_Ctx.SegResid = 0.0f;
    }
#else
    (void)Local_GyroDeg;
#endif

    /* ---- Events ---- */
    const bool Local_Stall = Local_Stalled[POSE_LEFT] || Local_Stalled[POSE_RIGHT];
    const float Local_Pitch = Local_Ctx.PitchLpfDeg;
    const float Local_Roll  = Local_Ctx.RollDeg;
    uint32_t Local_Flags = 0;

    if (Local_Ctx.Stationary) { Local_Flags |= SRV_POSE_FLAG_STATIONARY; }
    if (Pose_Hold(&Local_Ctx.RampUp,   Local_Pitch >  SRV_POSE_RAMP_ENTER_DEG, Local_Pitch <  SRV_POSE_RAMP_EXIT_DEG,
                  Copy_s64Time_us, SRV_POSE_HOLD_US)) { Local_Flags |= SRV_POSE_FLAG_RAMP_UP; }
    if (Pose_Hold(&Local_Ctx.RampDown, Local_Pitch < -SRV_POSE_RAMP_ENTER_DEG, Local_Pitch > -SRV_POSE_RAMP_EXIT_DEG,
                  Copy_s64Time_us, SRV_POSE_HOLD_US)) { Local_Flags |= SRV_POSE_FLAG_RAMP_DOWN; }
    if (Pose_Hold(&Local_Ctx.TiltL,    Local_Roll >  SRV_POSE_TILT_ENTER_DEG, Local_Roll <  SRV_POSE_TILT_EXIT_DEG,
                  Copy_s64Time_us, SRV_POSE_HOLD_US)) { Local_Flags |= SRV_POSE_FLAG_TILT_LEFT_UP; }
    if (Pose_Hold(&Local_Ctx.TiltR,    Local_Roll < -SRV_POSE_TILT_ENTER_DEG, Local_Roll > -SRV_POSE_TILT_EXIT_DEG,
                  Copy_s64Time_us, SRV_POSE_HOLD_US)) { Local_Flags |= SRV_POSE_FLAG_TILT_RIGHT_UP; }

    if (Local_Stall)
    {
        const bool Local_Flat  = (fabsf(Local_Pitch) < SRV_POSE_WALL_PITCH_MAX_DEG);
        const bool Local_Spike = (Local_Ctx.LastSpike_us != 0) &&
                                 (Local_Ctx.LastSpike_us >= (Local_StallStart - SRV_POSE_SPIKE_BEFORE_STALL_US));
        Local_Flags |= SRV_POSE_FLAG_STALL;
        if (Local_Flat && Local_Spike)
        {
            Local_Flags |= SRV_POSE_FLAG_WALL_HIT;
        }
        else if ((Local_Pitch > SRV_POSE_RAMP_ENTER_DEG) || (fabsf(Local_Pitch) > SRV_POSE_STEEP_DEG))
        {
            Local_Flags |= SRV_POSE_FLAG_TOO_STEEP;
        }
        else if (Local_Flat && ((Copy_s64Time_us - Local_StallStart) >= SRV_POSE_STUCK_US))
        {
            Local_Flags |= SRV_POSE_FLAG_STUCK;
        }
    }

    Local_Ctx.NewEvents |= (Local_Flags & ~Local_Ctx.Flags);
    Local_Ctx.Flags      = Local_Flags;
    return FUNCTION_OK;
}

void SRV_POSE_GetPose(Pose_TypeDef *Copy_pPose)
{
    if (Copy_pPose == NULL) { return; }
    Copy_pPose->X_m       = Local_Ctx.X;
    Copy_pPose->Y_m       = Local_Ctx.Y;
    Copy_pPose->Z_m       = Local_Ctx.Z;
    Copy_pPose->Theta_deg = Pose_Wrap180(Local_Ctx.Theta);
    Copy_pPose->V_mps     = Local_Ctx.V;
    Copy_pPose->Omega_dps = Local_Ctx.Omega_dps;
    Copy_pPose->Pitch_deg = Local_Ctx.PitchDeg;
    Copy_pPose->Roll_deg  = Local_Ctx.RollDeg;
}

void SRV_POSE_GetWheelSpeeds(float *Copy_pLeft_mps, float *Copy_pRight_mps)
{
    if (Copy_pLeft_mps  != NULL) { *Copy_pLeft_mps  = Local_Ctx.VL; }
    if (Copy_pRight_mps != NULL) { *Copy_pRight_mps = Local_Ctx.VR; }
}

uint32_t SRV_POSE_GetFlags(void)
{
    return Local_Ctx.Flags;
}

uint32_t SRV_POSE_PopNewEvents(void)
{
    const uint32_t Local_Ev = Local_Ctx.NewEvents;
    Local_Ctx.NewEvents = 0;
    return Local_Ev;
}

SRV_POSE_State_TypeDef SRV_POSE_GetState(void)
{
    return Local_Ctx.State;
}

void SRV_POSE_GetDebug(SRV_POSE_Debug_TypeDef *Copy_pDebug)
{
    if (Copy_pDebug == NULL) { return; }
    for (uint8_t i = 0; i < 3; i++)
    {
        Copy_pDebug->GyroBias_dps[i] = Local_Ctx.GyroBias[i];
    }
    Copy_pDebug->GyroZRaw_dps   = Local_Ctx.GzRaw_dps;
    Copy_pDebug->Temp_C         = Local_Ctx.Temp_C;
    Copy_pDebug->SignedTicksL   = Local_Ctx.SignedTicks[POSE_LEFT];
    Copy_pDebug->SignedTicksR   = Local_Ctx.SignedTicks[POSE_RIGHT];
    Copy_pDebug->MissedSamples  = Local_Ctx.MissedSamples;
}

void SRV_POSE_GetAblation(SRV_POSE_Ablation_TypeDef *Copy_pAblation)
{
    if (Copy_pAblation == NULL) { return; }
    Copy_pAblation->EncodersOnly         = Local_Ctx.TrackEnc;
    Copy_pAblation->EncodersOnly.Theta_deg = Pose_Wrap180(Local_Ctx.ThetaEnc);
    Copy_pAblation->GyroBootBias         = Local_Ctx.TrackBoot;
    Copy_pAblation->GyroBootBias.Theta_deg = Pose_Wrap180(Local_Ctx.ThetaBoot);
    Copy_pAblation->ImuOnly              = Local_Ctx.TrackImuOnly;
    Copy_pAblation->ImuOnly.Theta_deg    = Pose_Wrap180(
        atan2f(2.0f * (Local_Ctx.Q[0] * Local_Ctx.Q[3] + Local_Ctx.Q[1] * Local_Ctx.Q[2]),
               1.0f - 2.0f * (Local_Ctx.Q[2] * Local_Ctx.Q[2] + Local_Ctx.Q[3] * Local_Ctx.Q[3])) * POSE_RAD2DEG);
}

void SRV_POSE_ResetPose(void)
{
    Local_Ctx.X = Local_Ctx.Y = Local_Ctx.Z = 0.0f;
    Local_Ctx.Theta = Local_Ctx.ThetaBoot = Local_Ctx.ThetaEnc = 0.0f;
    Local_Ctx.PrevThetaFull = Local_Ctx.PrevThetaBoot = Local_Ctx.PrevThetaEnc = 0.0f;
    Local_Ctx.PendingGyroDeg = 0.0f;
    memset(&Local_Ctx.TrackEnc,  0, sizeof(Local_Ctx.TrackEnc));
    memset(&Local_Ctx.TrackBoot, 0, sizeof(Local_Ctx.TrackBoot));
    memset(&Local_Ctx.TrackImuOnly, 0, sizeof(Local_Ctx.TrackImuOnly));
    Local_Ctx.ImuOnlyVx = Local_Ctx.ImuOnlyVy = 0.0f;
    Local_Ctx.ImuOnlyPrevAx = Local_Ctx.ImuOnlyPrevAy = 0.0f;
    Local_Ctx.ImuOnlyHavePrevAccel = false;
}

/*------------------------------------------------------------------------------------------------*/



/*------------------------------------------------------------------------------------------------*/
// --------------------------------- Private functions definitions ---------------------------------

/*------------------------------------------------------------------------------------------------*/

static void Welford_Reset(Welford_t *Copy_pW)
{
    Copy_pW->n = 0; Copy_pW->mean = 0.0f; Copy_pW->m2 = 0.0f;
}

static void Welford_Add(Welford_t *Copy_pW, float Copy_fX)
{
    Copy_pW->n++;
    const float Local_Delta = Copy_fX - Copy_pW->mean;
    Copy_pW->mean += Local_Delta / (float)Copy_pW->n;
    Copy_pW->m2   += Local_Delta * (Copy_fX - Copy_pW->mean);
}

static float Welford_Var(const Welford_t *Copy_pW)
{
    return (Copy_pW->n > 1u) ? (Copy_pW->m2 / (float)(Copy_pW->n - 1u)) : 0.0f;
}

static float Pose_Wrap180(float Copy_fAngle)
{
    float Local_A = fmodf(Copy_fAngle, 360.0f);
    if (Local_A >  180.0f) { Local_A -= 360.0f; }
    if (Local_A <= -180.0f) { Local_A += 360.0f; }
    return Local_A;
}

/* Rising-edge-with-hold flag: enter condition must hold for Copy_s64HoldUs, leaves on the exit condition */
static bool Pose_Hold(Hold_t *Copy_pH, bool Copy_bEnter, bool Copy_bExit, int64_t Copy_s64Now, int64_t Copy_s64HoldUs)
{
    if (Copy_pH->active)
    {
        if (Copy_bExit) { Copy_pH->active = false; Copy_pH->since_us = 0; }
    }
    else if (Copy_bEnter)
    {
        if (Copy_pH->since_us == 0) { Copy_pH->since_us = Copy_s64Now; }
        if ((Copy_s64Now - Copy_pH->since_us) >= Copy_s64HoldUs) { Copy_pH->active = true; }
    }
    else
    {
        Copy_pH->since_us = 0;
    }
    return Copy_pH->active;
}

/* Standard Madgwick IMU (6-DoF) update. Gyro in rad/s, accel in g (normalised inside). beta = 0 -> gyro only. */
static void Pose_Madgwick6(float Copy_fGx, float Copy_fGy, float Copy_fGz,
                           float Copy_fAx, float Copy_fAy, float Copy_fAz, float Copy_fDt, float Copy_fBeta)
{
    float q0 = Local_Ctx.Q[0], q1 = Local_Ctx.Q[1], q2 = Local_Ctx.Q[2], q3 = Local_Ctx.Q[3];

    float qDot1 = 0.5f * (-q1 * Copy_fGx - q2 * Copy_fGy - q3 * Copy_fGz);
    float qDot2 = 0.5f * ( q0 * Copy_fGx + q2 * Copy_fGz - q3 * Copy_fGy);
    float qDot3 = 0.5f * ( q0 * Copy_fGy - q1 * Copy_fGz + q3 * Copy_fGx);
    float qDot4 = 0.5f * ( q0 * Copy_fGz + q1 * Copy_fGy - q2 * Copy_fGx);

    const float Local_Norm = sqrtf(Copy_fAx * Copy_fAx + Copy_fAy * Copy_fAy + Copy_fAz * Copy_fAz);
    if ((Copy_fBeta > 0.0f) && (Local_Norm > 1e-6f))
    {
        const float ax = Copy_fAx / Local_Norm, ay = Copy_fAy / Local_Norm, az = Copy_fAz / Local_Norm;

        const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        const float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        const float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        const float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        const float Local_SNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (Local_SNorm > 1e-9f)
        {
            s0 /= Local_SNorm; s1 /= Local_SNorm; s2 /= Local_SNorm; s3 /= Local_SNorm;
            qDot1 -= Copy_fBeta * s0;
            qDot2 -= Copy_fBeta * s1;
            qDot3 -= Copy_fBeta * s2;
            qDot4 -= Copy_fBeta * s3;
        }
    }

    q0 += qDot1 * Copy_fDt; q1 += qDot2 * Copy_fDt; q2 += qDot3 * Copy_fDt; q3 += qDot4 * Copy_fDt;
    const float Local_QNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Local_Ctx.Q[0] = q0 / Local_QNorm; Local_Ctx.Q[1] = q1 / Local_QNorm;
    Local_Ctx.Q[2] = q2 / Local_QNorm; Local_Ctx.Q[3] = q3 / Local_QNorm;
}

/* Initial orientation from gravity (yaw = 0), same convention as the Euler extraction below */
static void Pose_QuatFromAccel(float Copy_fAx, float Copy_fAy, float Copy_fAz)
{
    const float Local_Roll  = atan2f(Copy_fAy, Copy_fAz);
    const float Local_Pitch = atan2f(-Copy_fAx, sqrtf(Copy_fAy * Copy_fAy + Copy_fAz * Copy_fAz));
    const float cr = cosf(Local_Roll * 0.5f),  sr = sinf(Local_Roll * 0.5f);
    const float cp = cosf(Local_Pitch * 0.5f), sp = sinf(Local_Pitch * 0.5f);
    Local_Ctx.Q[0] =  cr * cp;
    Local_Ctx.Q[1] =  sr * cp;
    Local_Ctx.Q[2] =  cr * sp;
    Local_Ctx.Q[3] = -sr * sp;
}

static void Pose_UpdateAngles(void)
{
    const float q0 = Local_Ctx.Q[0], q1 = Local_Ctx.Q[1], q2 = Local_Ctx.Q[2], q3 = Local_Ctx.Q[3];
    const float Local_Roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float Local_SinP = 2.0f * (q0 * q2 - q3 * q1);
    if (Local_SinP >  1.0f) { Local_SinP =  1.0f; }
    if (Local_SinP < -1.0f) { Local_SinP = -1.0f; }
    Local_Ctx.RollDeg  = SRV_POSE_ROLL_SIGN  * Local_Roll * POSE_RAD2DEG;
    Local_Ctx.PitchDeg = SRV_POSE_PITCH_SIGN * asinf(Local_SinP) * POSE_RAD2DEG;
}

/* Rotate calibrated specific force into the world frame, remove +1 g at rest,
 * then integrate acceleration twice. This is deliberately an uncorrected
 * IMU-only comparison trace; it must never feed the driving pose estimator. */
static void Pose_IntegrateImuOnly(float Copy_fAx, float Copy_fAy, float Copy_fAz, float Copy_fDt)
{
    const float q0 = Local_Ctx.Q[0], q1 = Local_Ctx.Q[1];
    const float q2 = Local_Ctx.Q[2], q3 = Local_Ctx.Q[3];
    const float r11 = 1.0f - 2.0f * (q2*q2 + q3*q3);
    const float r12 = 2.0f * (q1*q2 - q0*q3);
    const float r13 = 2.0f * (q1*q3 + q0*q2);
    const float r21 = 2.0f * (q1*q2 + q0*q3);
    const float r22 = 1.0f - 2.0f * (q1*q1 + q3*q3);
    const float r23 = 2.0f * (q2*q3 - q0*q1);
    const float r31 = 2.0f * (q1*q3 - q0*q2);
    const float r32 = 2.0f * (q2*q3 + q0*q1);
    const float r33 = 1.0f - 2.0f * (q1*q1 + q2*q2);
    const float ax_world = r11*Copy_fAx + r12*Copy_fAy + r13*Copy_fAz;
    const float ay_world = r21*Copy_fAx + r22*Copy_fAy + r23*Copy_fAz;
    const float az_world = r31*Copy_fAx + r32*Copy_fAy + r33*Copy_fAz - 1.0f;
    const float linear_ax = ax_world * 9.80665f;
    const float linear_ay = ay_world * 9.80665f;
    (void)az_world; /* Vertical motion is outside this 2D comparison display. */

    if (!Local_Ctx.ImuOnlyHavePrevAccel)
    {
        Local_Ctx.ImuOnlyPrevAx = linear_ax;
        Local_Ctx.ImuOnlyPrevAy = linear_ay;
        Local_Ctx.ImuOnlyHavePrevAccel = true;
        return;
    }

    const float avg_ax = 0.5f * (Local_Ctx.ImuOnlyPrevAx + linear_ax);
    const float avg_ay = 0.5f * (Local_Ctx.ImuOnlyPrevAy + linear_ay);
    Local_Ctx.TrackImuOnly.X_m += Local_Ctx.ImuOnlyVx * Copy_fDt + 0.5f * avg_ax * Copy_fDt * Copy_fDt;
    Local_Ctx.TrackImuOnly.Y_m += Local_Ctx.ImuOnlyVy * Copy_fDt + 0.5f * avg_ay * Copy_fDt * Copy_fDt;
    Local_Ctx.ImuOnlyVx += avg_ax * Copy_fDt;
    Local_Ctx.ImuOnlyVy += avg_ay * Copy_fDt;
    Local_Ctx.ImuOnlyPrevAx = linear_ax;
    Local_Ctx.ImuOnlyPrevAy = linear_ay;
}

static void Pose_CalibrationStart(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        Welford_Reset(&Local_Ctx.CalG[i]);
        Welford_Reset(&Local_Ctx.CalA[i]);
    }
    Welford_Reset(&Local_Ctx.CalT);
    Local_Ctx.State = SRV_POSE_STATE_CALIBRATING;
}

static void Pose_CalibrationFinish(void)
{
    bool Local_Ok = true;
    for (uint8_t i = 0; i < 3; i++)
    {
        if ((sqrtf(Welford_Var(&Local_Ctx.CalG[i])) > SRV_POSE_CAL_MAX_STD_DPS) ||
            (fabsf(Local_Ctx.CalG[i].mean)          > SRV_POSE_CAL_MAX_MEAN_DPS) ||
            (sqrtf(Welford_Var(&Local_Ctx.CalA[i])) > SRV_POSE_CAL_ACCEL_MAX_STD_G))
        {
            Local_Ok = false;
        }
    }
    const float Local_Ax = Local_Ctx.CalA[0].mean, Local_Ay = Local_Ctx.CalA[1].mean, Local_Az = Local_Ctx.CalA[2].mean;
    const float Local_Amag = sqrtf(Local_Ax * Local_Ax + Local_Ay * Local_Ay + Local_Az * Local_Az);
    if ((Local_Amag < SRV_POSE_CAL_ACCEL_MIN_G) || (Local_Amag > SRV_POSE_CAL_ACCEL_MAX_G))
    {
        Local_Ok = false;
    }

    if (!Local_Ok)
    {
        Local_Ctx.State = SRV_POSE_STATE_CAL_FAILED;
        return;
    }

    for (uint8_t i = 0; i < 3; i++)
    {
        Local_Ctx.GyroBias[i] = Local_Ctx.CalG[i].mean;
        Local_Ctx.BootBias[i] = Local_Ctx.CalG[i].mean;
    }
    /* Apply the same chassis-axis remap used by the live tilt filter. */
    Pose_QuatFromAccel(Local_Ay, -Local_Ax, Local_Az);
    Pose_UpdateAngles();
    Local_Ctx.PitchLpfDeg = Local_Ctx.PitchDeg;
    Local_Ctx.Temp_C      = Local_Ctx.CalT.mean;

    Local_Ctx.Stationary = true;
    for (uint8_t i = 0; i < 3; i++) { Welford_Reset(&Local_Ctx.WinG[i]); }
    Welford_Reset(&Local_Ctx.WinA);
    SRV_POSE_ResetPose();
    Local_Ctx.OdomInit = false;                 /* Re-baseline the encoders */
    Local_Ctx.State    = SRV_POSE_STATE_READY;
}

/* Window statistics -> stationary decision -> bias update */
static void Pose_StationaryStep(const float Copy_fG[3], float Copy_fAmag, float Copy_fGz, int64_t Copy_s64Now)
{
    const bool Local_RecentMotion = (Local_Ctx.LastMotion_us != 0) &&
                                    ((Copy_s64Now - Local_Ctx.LastMotion_us) < SRV_POSE_STAT_SETTLE_US);

    if (Local_RecentMotion || (fabsf(Copy_fGz) > SRV_POSE_STAT_INSTANT_ROT_DPS))
    {
        Local_Ctx.Stationary = false;
        for (uint8_t i = 0; i < 3; i++) { Welford_Reset(&Local_Ctx.WinG[i]); }
        Welford_Reset(&Local_Ctx.WinA);
        return;
    }

    for (uint8_t i = 0; i < 3; i++) { Welford_Add(&Local_Ctx.WinG[i], Copy_fG[i]); }
    Welford_Add(&Local_Ctx.WinA, Copy_fAmag);

    if (Local_Ctx.WinG[0].n < SRV_POSE_STAT_WINDOW_SAMPLES)
    {
        return;
    }

    bool Local_Quiet = (sqrtf(Welford_Var(&Local_Ctx.WinA)) < SRV_POSE_STAT_ACCEL_STD_G);
    for (uint8_t i = 0; i < 3; i++)
    {
        if ((sqrtf(Welford_Var(&Local_Ctx.WinG[i])) >= SRV_POSE_STAT_GYRO_STD_DPS) ||
            (fabsf(Local_Ctx.WinG[i].mean - Local_Ctx.GyroBias[i]) >= SRV_POSE_STAT_MEAN_DEV_DPS))
        {
            Local_Quiet = false;
        }
    }

    Local_Ctx.Stationary = Local_Quiet;
    if (Local_Quiet)
    {
        for (uint8_t i = 0; i < 3; i++)         /* The ZUPT: whatever the gyro reads now IS the bias */
        {
            Local_Ctx.GyroBias[i] += SRV_POSE_STAT_BIAS_ALPHA * (Local_Ctx.WinG[i].mean - Local_Ctx.GyroBias[i]);
        }
    }
    for (uint8_t i = 0; i < 3; i++) { Welford_Reset(&Local_Ctx.WinG[i]); }
    Welford_Reset(&Local_Ctx.WinA);
}

/*------------------------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------------------------------------------*-
----------------------------------------------------- END OF FILE -----------------------------------------------------
-*-------------------------------------------------------------------------------------------------------------------*/
