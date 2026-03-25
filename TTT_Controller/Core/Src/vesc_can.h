#ifndef VESC_CAN_H
#define VESC_CAN_H

#include "stm32f7xx_hal.h"

/* -----------------------------------------------------------------------
 * VESC Controller IDs  (CAN IDs 1-5)
 * Must match App Settings > VESC ID in VESC Tool for each controller
 * ----------------------------------------------------------------------- */
#define VESC_ID_MOTOR1   1
#define VESC_ID_MOTOR2   2
#define VESC_ID_MOTOR3   3
#define VESC_ID_MOTOR4   4
#define VESC_ID_MOTOR5   5

#define VESC_NUM_MOTORS  5

/* -----------------------------------------------------------------------
 * Motor index enum
 * ----------------------------------------------------------------------- */
typedef enum {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
    MOTOR_3 = 2,
    MOTOR_4 = 3,
    MOTOR_5 = 4,
} VescMotor_t;

/* -----------------------------------------------------------------------
 * VESC CANSimple command IDs
 * ----------------------------------------------------------------------- */
#define CAN_PACKET_SET_DUTY                 0
#define CAN_PACKET_SET_CURRENT              1
#define CAN_PACKET_SET_CURRENT_BRAKE        2
#define CAN_PACKET_SET_RPM                  3
#define CAN_PACKET_STATUS_1                 9   /* ERPM, current, duty - broadcast by VESC */
#define CAN_PACKET_SET_CURRENT_HANDBRAKE    2  /* Hold position at standstill without rotor snap */
#define CAN_PACKET_STATUS_5                 27  /* Tachometer (cumulative motor steps), abs tachometer */

/* Handbrake current applied when motor is idle.
 * SET_CURRENT_HANDBRAKE (cmd 12) resists movement from the current rotor
 * position without snapping to a new flux alignment angle on startup.
 * Increase if motors drift under load, decrease if they run warm at standstill.
 * Units: Amps */
#define VESC_BRAKE_CURRENT       15.0f

/* -----------------------------------------------------------------------
 * Status data - one per motor, updated on CAN receive
 * ----------------------------------------------------------------------- */
typedef struct {
    int32_t erpm;           /* Electrical RPM - divide by pole_pairs for mech RPM */
    float   current;        /* Motor current in Amps */
    float   duty;           /* Duty cycle -1.0 to +1.0 */
    int32_t tachometer;     /* Cumulative motor steps from STATUS_5 (encoder only).
                             * motor_revs = tachometer / (pole_pairs * 6)          */
    uint8_t has_tach;       /* 1 once the first STATUS_5 frame has been received   */
    uint8_t updated;        /* Set to 1 when new data arrives, cleared after read  */
} VescStatus_t;

/* -----------------------------------------------------------------------
 * Public API - Transmit
 * ----------------------------------------------------------------------- */
void VESC_Init(CAN_HandleTypeDef *hcan);
void VESC_SetDuty(VescMotor_t motor, float duty);
void VESC_SetCurrent(VescMotor_t motor, float amps);
void VESC_SetERPM(VescMotor_t motor, int32_t erpm);
void VESC_SetDutyAll(float duty);
void VESC_Brake(VescMotor_t motor);
void VESC_BrakeAll(void);
void VESC_KeepAlive(VescMotor_t motor);
void VESC_KeepAliveAll(void);

/* -----------------------------------------------------------------------
 * Public API - Receive
 * ----------------------------------------------------------------------- */
void          VESC_ProcessRx(void);
VescStatus_t* VESC_GetStatus(VescMotor_t motor);
void          VESC_PrintDiag(UART_HandleTypeDef *huart);

#endif /* VESC_CAN_H */
