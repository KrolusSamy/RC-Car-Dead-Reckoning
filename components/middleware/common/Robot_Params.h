/*------------------------------------------------------------------------------------------------------*-
 * File Name          : Robot_Params.h
 * Description        : Physical parameters of the car that more than one service needs.
 *                      Lives in "common" (below every layer) so SRV_POSE and SRV_NAV never have to
 *                      include each other. Every value here is a STARTING GUESS: calibrate it.
 -------------------------------------------------------------------------------------------------------*/
#ifndef ROBOT_PARAMS_H_
#define ROBOT_PARAMS_H_

#define ROBOT_TRACK_WIDTH_M     0.13f   /* Still a provisional estimate; validate using repeatable measured pivots. */
#define ROBOT_TICKS_PER_M_L     166.0f  /* Estimated from repeated 80-tick runs: left=519 ticks / 3.13 m          */
#define ROBOT_TICKS_PER_M_R     181.0f  /* Estimated from repeated 80-tick runs: right=566 ticks / 3.13 m         */
#define ROBOT_PWM_MIN           125     /* Lowest PWM that moves the motor (deadband)                                       */
#define ROBOT_PWM_MAX           255
#define ROBOT_V_MAX_MPS         0.50f   /* Wheel speed at ROBOT_PWM_MAX (measure it)                                        */

#endif /* ROBOT_PARAMS_H_ */
