#include "arm_controller.h"
#include "vesc_can.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* -----------------------------------------------------------------------
 * Convenience macro
 * ----------------------------------------------------------------------- */
#define CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* -----------------------------------------------------------------------
 * Per-joint configuration table
 *
 * ALL VALUES ARE PLACEHOLDERS — adjust to your arm before running.
 *
 * Start with all gains at zero and add them one at a time:
 *   1. Set kg first: hold a joint horizontal, increase kg until it stops
 *      drooping.  Good starting range: 5–20 A depending on link weight.
 *   2. Set kp: increase until joints snap back after a nudge without
 *      oscillating.  Good starting range: 3–10 A/rev.
 *   3. Set kv: increase until transient overshoot is damped.
 *      Good starting range: 1–4 A/(rev/s).
 *   4. Tune v_max: this is output-shaft rev/s at full stick.
 *      0.3 rev/s ≈ 108 °/s — a reasonable starting speed for a heavy arm.
 *   5. Set pos_min / pos_max to safe travel limits for each joint
 *      (output-shaft revolutions from the home position).
 *
 * gear_ratio = 1 / gearbox_reduction
 *   e.g.  80:1 → 0.0125,   50:1 → 0.02,   30:1 → 0.033
 *
 * pole_pairs: check your motor spec sheet.  Many outrunners use 7.
 * ----------------------------------------------------------------------- */
static JointConfig_t cfg[NUM_JOINTS] = {
    /* J1 – Base yaw (rotation around gravity vector → kg = 0) */
    { .kp=5.0f,  .kv=2.0f,  .kg= 0.0f, .v_max=0.5f, .i_max=20.0f,
      .gear_ratio=0.02f,   .pole_pairs=7, .pos_min=-2.0f,  .pos_max= 2.0f  },

    /* J2 – Shoulder (carries full arm weight — start with high kg) */
    { .kp=8.0f,  .kv=3.0f,  .kg=12.0f, .v_max=0.3f, .i_max=25.0f,
      .gear_ratio=0.0125f, .pole_pairs=7, .pos_min=-0.5f,  .pos_max= 0.5f  },

    /* J3 – Elbow */
    { .kp=6.0f,  .kv=2.5f,  .kg= 6.0f, .v_max=0.4f, .i_max=20.0f,
      .gear_ratio=0.02f,   .pole_pairs=7, .pos_min=-0.75f, .pos_max= 0.75f },

    /* J4 – Wrist pitch */
    { .kp=4.0f,  .kv=1.5f,  .kg= 2.0f, .v_max=0.5f, .i_max=15.0f,
      .gear_ratio=0.033f,  .pole_pairs=7, .pos_min=-1.0f,  .pos_max= 1.0f  },

    /* J5 – Wrist roll (rotation around long axis → kg = 0) */
    { .kp=3.0f,  .kv=1.0f,  .kg= 0.0f, .v_max=0.6f, .i_max=15.0f,
      .gear_ratio=0.033f,  .pole_pairs=7, .pos_min=-2.0f,  .pos_max= 2.0f  },
};

/* -----------------------------------------------------------------------
 * Run-time state
 * ----------------------------------------------------------------------- */
static JointState_t state[NUM_JOINTS];

/*
 * Track whether each joystick axis was in the deadband on the previous
 * cycle.  Used to snap the position setpoint to the current estimate the
 * instant the user picks up a stick, preventing a sudden lurch.
 */
static uint8_t was_in_deadband[NUM_JOINTS];

/* -----------------------------------------------------------------------
 * Init
 * ----------------------------------------------------------------------- */
void ArmController_Init(void)
{
    memset(state,           0, sizeof(state));
    memset(was_in_deadband, 1, sizeof(was_in_deadband));
}

/* -----------------------------------------------------------------------
 * Main control update — call at 100 Hz
 *
 * Control structure per joint:
 *
 *   1. State estimation
 *      Velocity is derived from the VESC ERPM telemetry received via the
 *      CAN RX interrupt (STATUS_1 frames that VESCs broadcast automatically).
 *      Position is obtained by integrating that velocity — open-loop dead-
 *      reckoning.  This is sufficient for velocity-mode joystick control.
 *      For higher accuracy, configure the VESC to stream STATUS_4 frames
 *      (which carry the encoder PID position) and use that instead.
 *
 *   2. Joystick → setpoint
 *      While the stick is deflected, the position setpoint advances at
 *      v_des = stick * v_max [rev/s].  The moment the stick returns to the
 *      deadband the setpoint freezes and the controller holds that pose.
 *      On joystick re-engage, the setpoint is snapped to the current
 *      position estimate so there is no lurch from accumulated drift.
 *
 *   3. PD + gravity feedforward
 *      I = kp*(pos_sp - pos_est) + kv*(v_des - vel_est) + kg*cos(angle)
 *      The cosine term approximates the gravity torque on a single rigid
 *      link.  For a multi-link arm the shoulder joint should use a larger
 *      kg because it supports more distal mass — tune empirically.
 * ----------------------------------------------------------------------- */
void ArmController_Update(const float joy[NUM_JOINTS], float dt)
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        VescStatus_t *s = VESC_GetStatus((VescMotor_t)i);

        /* ---- 1. State estimation ---------------------------------------- */

        /* ERPM = electrical RPM = mechanical RPM × pole_pairs
         * Convert to output-shaft velocity [rev/s]:
         *   out_vel = (ERPM / pole_pairs / 60) × gear_ratio             */
        float erpm = s ? (float)s->erpm : 0.0f;
        float out_vel = (erpm / (float)cfg[i].pole_pairs / 60.0f)
                        * cfg[i].gear_ratio;

        /* Low-pass filter to reduce quantisation noise from the CAN frames.
         * α = 0.25 at 100 Hz gives ~4 Hz bandwidth — smooth but responsive. */
        state[i].vel_est = 0.75f * state[i].vel_est + 0.25f * out_vel;

        /* Dead-reckoning position integration */
        state[i].pos_est += state[i].vel_est * dt;

        /* ---- 2. Joystick processing ------------------------------------- */

        float j = CLAMP(joy[i], -1.0f, 1.0f);
        if (fabsf(j) < JOY_DEADBAND) j = 0.0f;

        float v_des = j * cfg[i].v_max;   /* desired output velocity [rev/s] */

        if (fabsf(j) > JOY_DEADBAND)
        {
            /* On joystick engage: snap setpoint to current position so
             * there is no lurch caused by accumulated position drift.     */
            if (was_in_deadband[i])
            {
                state[i].pos_sp    = state[i].pos_est;
                was_in_deadband[i] = 0;
            }

            /* Advance setpoint at the commanded velocity */
            state[i].pos_sp += v_des * dt;
            state[i].pos_sp  = CLAMP(state[i].pos_sp,
                                     cfg[i].pos_min, cfg[i].pos_max);
        }
        else
        {
            /* Stick released — hold the frozen setpoint */
            was_in_deadband[i] = 1;
            /* v_des is already 0 here, so velocity error term still damps
             * any residual motion.                                         */
        }

        /* ---- 3. PD + gravity feedforward -------------------------------- */

        float e_pos = state[i].pos_sp  - state[i].pos_est;
        float e_vel = v_des            - state[i].vel_est;

        /* Gravity feedforward.
         * pos_est is in output revolutions; 1 rev = 2π rad.
         * I_ff = kg * cos(θ)  where θ=0 is horizontal (max gravity torque),
         * θ=π/2 is vertical (zero gravity torque).
         * This is exact for a single-link pendulum; for a serial arm the
         * shoulder joint carries extra distal mass — compensate with a
         * larger kg for that joint.                                         */
        float angle_rad = state[i].pos_est * 2.0f * (float)M_PI;
        float i_grav    = cfg[i].kg * cosf(angle_rad);

        float i_cmd = cfg[i].kp * e_pos
                    + cfg[i].kv * e_vel
                    + i_grav;
        i_cmd = CLAMP(i_cmd, -cfg[i].i_max, cfg[i].i_max);

        state[i].i_cmd = i_cmd;
        VESC_SetCurrent((VescMotor_t)i, i_cmd);
    }
}

/* -----------------------------------------------------------------------
 * Emergency stop
 * ----------------------------------------------------------------------- */
void ArmController_EStop(void)
{
    /* Snap all setpoints to current estimates so that when the operator
     * re-enables the arm it holds where it stopped rather than driving
     * back to wherever the setpoints were.                                */
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        state[i].pos_sp    = state[i].pos_est;
        was_in_deadband[i] = 1;
    }
    VESC_BrakeAll();
}

/* -----------------------------------------------------------------------
 * Home — zero all position estimates at the current physical pose.
 * Call once the arm has been manually moved to a known reference position.
 * ----------------------------------------------------------------------- */
void ArmController_Home(void)
{
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        state[i].pos_est   = 0.0f;
        state[i].vel_est   = 0.0f;
        state[i].pos_sp    = 0.0f;
        was_in_deadband[i] = 1;
    }
}

/* -----------------------------------------------------------------------
 * Live tuning
 * ----------------------------------------------------------------------- */
void ArmController_SetGravityGain(int j, float kg)
{
    if (j >= 0 && j < NUM_JOINTS) cfg[j].kg = kg;
}

void ArmController_SetPDGains(int j, float kp, float kv)
{
    if (j >= 0 && j < NUM_JOINTS) { cfg[j].kp = kp; cfg[j].kv = kv; }
}

/* -----------------------------------------------------------------------
 * Status print
 * ----------------------------------------------------------------------- */
void ArmController_PrintStatus(UART_HandleTypeDef *huart)
{
    char line[128];
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        VescStatus_t *s = VESC_GetStatus((VescMotor_t)i);
        snprintf(line, sizeof(line),
                 "J%d  pos:%+7.3frev  sp:%+7.3frev  vel:%+6.3f r/s"
                 "  I:%+6.2fA  erpm:%+7ld\r\n",
                 i + 1,
                 state[i].pos_est, state[i].pos_sp, state[i].vel_est,
                 state[i].i_cmd,
                 s ? (long)s->erpm : 0L);
        HAL_UART_Transmit(huart, (uint8_t *)line, strlen(line), 100);
    }
}

/* -----------------------------------------------------------------------
 * State accessor
 * ----------------------------------------------------------------------- */
const JointState_t *ArmController_GetState(int j)
{
    if (j < 0 || j >= NUM_JOINTS) return NULL;
    return &state[j];
}
