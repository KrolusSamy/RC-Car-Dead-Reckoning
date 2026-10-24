/*------------------------------------------------------------------------------------------------------*-
 * File Name          : Robot_Types.h
 * Description        : Data types shared between services and the application layer.
 *                      Frame convention: X forward at start, Y to the left, Z up, heading CCW positive.
 -------------------------------------------------------------------------------------------------------*/
#ifndef ROBOT_TYPES_H_
#define ROBOT_TYPES_H_

typedef struct
{
    float X_m;          /*!< Position relative to the start, metres        */
    float Y_m;
    float Z_m;          /*!< Height change from slopes                     */
    float Theta_deg;    /*!< Heading, 0 = start heading, CCW+, (-180,180]  */
    float V_mps;        /*!< Forward speed                                 */
    float Omega_dps;    /*!< Yaw rate (bias corrected, 0 when stationary)  */
    float Pitch_deg;    /*!< Nose UP positive                              */
    float Roll_deg;     /*!< Left side UP positive                         */
}Pose_TypeDef;

#endif /* ROBOT_TYPES_H_ */
