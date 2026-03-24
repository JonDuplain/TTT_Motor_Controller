#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include "stm32f7xx_hal.h"

/* Number of joints / VESC motors */
#define NUM_JOINTS   5

/* Joystick absolute value below which the axis is treated as zero */
#define JOY_DEADBAND 0.05f

/*
 * Per-joint tuning parameters.
 *
 * kp          Position proportional gain [A/rev of output shaft].
 *             Higher → stiffer hold.  Too high → oscillation.
 *
 * kv          Velocity derivative gain [A / (rev/s of output shaft)].
 *             Higher → more damping.  Too high → sluggish.
 *
 * kg          Gravity feedforward amplitude [A].
 *             Applied as:  I_ff = kg * cos(joint_angle_rad)
 *             Tune with the joystick zeroed: increase kg until the arm
 *             holds a horizontal pose without drifting.
 *             Set to 0.0 for joints that rotate around the gravity vector
 *             (e.g. base yaw) — gravity does no work on those joints.
 *
 * v_max       Maximum output-shaft speed [rev/s] at full joystick deflection.
 *
 * i_max       Peak current limit [A].  Must not exceed VESC / motor ratings.
 *
 * gear_ratio  Output shaft revolutions per motor revolution.
 *             e.g. 50:1 reduction → gear_ratio = 1/50 = 0.02
 *             Used to convert ERPM → output shaft velocity.
 *
 * pole_pairs  Motor electrical pole pairs.  ERPM = RPM × pole_pairs.
 *             Common outrunner values: 7, 14.
 *
 * pos_min     Soft lower travel limit [output revolutions from home].
 * pos_max     Soft upper travel limit [output revolutions from home].
 *             The controller clamps the setpoint to these values.
 *             Set symmetrically, e.g. -0.5 / +0.5 for ±180°.
 *
 * has_encoder 1 = hardware encoder fitted; ERPM is accurate at all speeds.
 *             Enables: (a) velocity feedback without the sensorless noise-floor
 *             gate, and (b) tachometer-based absolute position from STATUS_5
 *             instead of dead-reckoning.  Requires STATUS_5 broadcast enabled
 *             in VESC Tool.  Set to 0 for sensorless joints.
 */
typedef struct {
    float   kp;
    float   kv;
    float   kg;
    float   v_max;
    float   i_max;
    float   gear_ratio;
    int     pole_pairs;
    float   pos_min;
    float   pos_max;
    uint8_t has_encoder;
} JointConfig_t;

/*
 * Per-joint run-time state (read-only from outside arm_controller.c).
 */
typedef struct {
    float   pos_est;    /* Estimated output-shaft position [rev from home]  */
    float   vel_est;    /* Estimated output-shaft velocity [rev/s]          */
    float   pos_sp;     /* Current position setpoint [rev]                  */
    float   i_cmd;      /* Current command last sent to VESC [A]            */
    int32_t tach_home;  /* Tachometer value at last Home() call (encoder joints) */
} JointState_t;

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/* Call once after VESC_Init(). */
void ArmController_Init(void);

/*
 * Call at a fixed rate (100 Hz recommended) from the main loop.
 *
 * joy[NUM_JOINTS]  Joystick axes, each clamped to [-1.0, +1.0].
 *                  Positive → positive velocity (away from home).
 * dt               Elapsed time since last call [seconds].
 */
void ArmController_Update(const float joy[NUM_JOINTS], float dt);

/* Immediately brake all motors and freeze setpoints at current position. */
void ArmController_EStop(void);

/* Zero all position estimates and setpoints (call at a known home pose). */
void ArmController_Home(void);

/* Live tuning helpers — take effect on the next Update() call. */
void ArmController_SetGravityGain(int joint_0based, float kg);
void ArmController_SetPDGains(int joint_0based, float kp, float kv);

/* Print a human-readable status table over the given UART. */
void ArmController_PrintStatus(UART_HandleTypeDef *huart);

/* Read-only access to the run-time state of one joint (0-based index). */
const JointState_t *ArmController_GetState(int joint_0based);

#endif /* ARM_CONTROLLER_H */
