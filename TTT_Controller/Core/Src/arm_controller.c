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

/*
 * Joystick slew rate [full-scale units per second].
 * Limits how fast the commanded axis value can change each control cycle,
 * preventing the kv × (v_des − vel_est) term from spiking when the operator
 * suddenly deflects the stick.
 *
 * 1.5 / sec  → ~0.67 s to reach full deflection from rest.
 * Lower this value for heavier / more dangerous arms.
 */
#define JOY_SLEW_RATE  1.5f

/*
 * Maximum dt accepted by the integrators [seconds].
 * Guards against a stale control tick (e.g. delayed by a long UART print)
 * producing an oversized setpoint step.
 */
#define DT_MAX  0.025f   /* 2.5 × the nominal 10 ms period */

/*
 * Sensorless ERPM noise floor.
 *
 * Without a hardware encoder, ERPM comes from the VESC's sensorless FOC
 * flux estimator.  Near zero speed this estimator is unreliable — it
 * produces noisy, jittery values that, if integrated, accumulate fake
 * position error and drive the kp term into rapid oscillation.
 *
 * When |ERPM| is below this threshold the motor is treated as stationary:
 * velocity is forced to zero and the position integrator is frozen.
 * The threshold is in electrical RPM.  50 ERPM ≈ 7 mechanical RPM for a
 * 7-pole-pair motor — well within the unreliable sensorless region.
 *
 * Once a real encoder is fitted, this can be set to 0.
 */
#define ERPM_NOISE_FLOOR  50.0f

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
    /*
     * CONSERVATIVE STARTING VALUES — sensorless, no encoder fitted.
     *
     * kp is near-zero intentionally.  Without an encoder, the position
     * estimate drifts at low speed and a significant kp will chase that
     * drift, causing oscillation.  Increase kp only after an encoder is
     * fitted and position feedback is verified to be stable.
     *
     * kg = 0 until gravity compensation is needed.  Add it only once the
     * arm is mechanically loaded and kp/kv are tuned.
     *
     * i_max = 5 A for bench safety.  Raise joint-by-joint after testing.
     *
     * v_max = 0.05 output-shaft rev/s ≈ 18 °/s.  Very slow on purpose.
     * Increase only after verifying smooth, stable motion at this speed.
     *
     * gear_ratio / pole_pairs MUST be set correctly before running —
     * they determine velocity scaling and position estimation accuracy.
     */

    /* J1 – Base yaw */
    { .kp=0.5f,  .kv=0.5f,  .kg=0.0f, .v_max=0.05f, .i_max=5.0f,
      .gear_ratio=0.02f,   .pole_pairs=7, .pos_min=-2.0f,  .pos_max= 2.0f  },

    /* J2 – Shoulder  (direct-drive 6374, no load, bench test)
     * kv × v_max = 2.0 × 0.5 = 1.0 A initial kick — enough to spin freely.
     * Update gear_ratio when the gearbox is fitted.                        */
    { .kp=1.0f,  .kv=2.0f,  .kg=0.0f, .v_max=0.5f, .i_max=10.0f,
      .gear_ratio= 1.0f, .pole_pairs=7, .pos_min=-0.5f,  .pos_max= 0.5f  },

    /* J3 – Elbow */
    { .kp=0.5f,  .kv=0.5f,  .kg=0.0f, .v_max=0.05f, .i_max=5.0f,
      .gear_ratio=0.02f,   .pole_pairs=7, .pos_min=-0.75f, .pos_max= 0.75f },

    /* J4 – Wrist pitch  (5010, 24 poles = 12 pole pairs) */
    { .kp=0.5f,  .kv=0.5f,  .kg=0.0f, .v_max=0.05f, .i_max=5.0f,
      .gear_ratio=0.033f,  .pole_pairs=12, .pos_min=-1.0f,  .pos_max= 1.0f  },

    /* J5 – Wrist roll  (5010, 24 poles = 12 pole pairs) */
    { .kp=0.5f,  .kv=0.5f,  .kg=0.0f, .v_max=0.05f, .i_max=5.0f,
      .gear_ratio=0.033f,  .pole_pairs=12, .pos_min=-2.0f,  .pos_max= 2.0f  },
};

/* -----------------------------------------------------------------------
 * Run-time state
 * ----------------------------------------------------------------------- */
static JointState_t state[NUM_JOINTS];

/*
 * Slew-rate-limited version of the raw joystick axes.
 * The raw axis value is never used directly for control — it is always
 * approached gradually at JOY_SLEW_RATE to prevent current spikes.
 */
static float joy_slewed[NUM_JOINTS];

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
    memset(joy_slewed,      0, sizeof(joy_slewed));
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
    /* Guard against stale ticks (e.g. delayed by a long UART print).
     * A dt spike would cause a large setpoint step and a current surge.    */
    if (dt > DT_MAX) dt = DT_MAX;

    float slew_step = JOY_SLEW_RATE * dt;

    for (int i = 0; i < NUM_JOINTS; i++)
    {
        VescStatus_t *s = VESC_GetStatus((VescMotor_t)i);

        /* ---- 1. State estimation ---------------------------------------- */

        /* ERPM = electrical RPM = mechanical RPM × pole_pairs
         * Convert to output-shaft velocity [rev/s]:
         *   out_vel = (ERPM / pole_pairs / 60) × gear_ratio
         *
         * Sensorless noise floor: below ERPM_NOISE_FLOOR the FOC estimator
         * is unreliable.  Treat the motor as stationary so that integrated
         * position drift does not drive the kp term into oscillation.      */
        float erpm = s ? (float)s->erpm : 0.0f;
        if (fabsf(erpm) < ERPM_NOISE_FLOOR) erpm = 0.0f;

        float out_vel = (erpm / (float)cfg[i].pole_pairs / 60.0f)
                        * cfg[i].gear_ratio;

        /* Low-pass filter to reduce quantisation noise from the CAN frames.
         * α = 0.25 at 100 Hz gives ~4 Hz bandwidth — smooth but responsive. */
        state[i].vel_est = 0.75f * state[i].vel_est + 0.25f * out_vel;

        /* Dead-reckoning position integration.
         * Only advance when the motor is confirmed to be moving (above the
         * noise floor).  This prevents slow ERPM noise from accumulating
         * into a fake position offset that kp would then try to correct.   */
        if (fabsf(erpm) >= ERPM_NOISE_FLOOR)
            state[i].pos_est += state[i].vel_est * dt;

        /* ---- 2. Joystick slew rate limiting ----------------------------- */

        /* Approach the raw joystick value gradually.  This prevents the
         * derivative term kv × (v_des − vel_est) from spiking when the
         * operator suddenly deflects the stick, which would produce a large
         * instantaneous current command and a lurch.                        */
        float raw = CLAMP(joy[i], -1.0f, 1.0f);
        if      (raw > joy_slewed[i] + slew_step) joy_slewed[i] += slew_step;
        else if (raw < joy_slewed[i] - slew_step) joy_slewed[i] -= slew_step;
        else                                       joy_slewed[i]  = raw;

        float j = joy_slewed[i];
        if (fabsf(j) < JOY_DEADBAND) j = 0.0f;

        float v_des = j * cfg[i].v_max;   /* desired output velocity [rev/s] */

        /* ---- 3. Position setpoint update -------------------------------- */

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
            /* Stick in deadband — hold the frozen setpoint */
            was_in_deadband[i] = 1;
            /* v_des is 0, so the velocity error term damps any residual
             * motion and holds the arm against gravity.                    */
        }

        /* ---- 4. PD + gravity feedforward -------------------------------- */

        float e_pos = state[i].pos_sp  - state[i].pos_est;
        float e_vel = v_des            - state[i].vel_est;

        /* Gravity feedforward.
         * pos_est is in output revolutions; 1 rev = 2π rad.
         * I_ff = kg × cos(θ)  where θ=0 is horizontal (max gravity torque)
         * and θ=π/2 is vertical (zero gravity torque).
         * This is exact for a single-link pendulum.  For a serial arm the
         * shoulder kg should be higher because it supports distal mass too.
         * Tune kg empirically: arm should hold horizontal with zero stick.  */
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
     * back to wherever the setpoints were.
     * Also zero the slewed joystick values so the next engage starts
     * from a clean ramp rather than whatever the stick was at.           */
    for (int i = 0; i < NUM_JOINTS; i++)
    {
        state[i].pos_sp    = state[i].pos_est;
        joy_slewed[i]      = 0.0f;
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
        joy_slewed[i]      = 0.0f;
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
