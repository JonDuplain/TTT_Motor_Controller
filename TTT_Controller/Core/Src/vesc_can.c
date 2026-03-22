#include "vesc_can.h"

/* -----------------------------------------------------------------------
 * Motor ID lookup table (index = VescMotor_t value)
 * ----------------------------------------------------------------------- */
static const uint8_t vesc_ids[5] = {
    VESC_ID_MOTOR1,
    VESC_ID_MOTOR2,
    VESC_ID_MOTOR3,
    VESC_ID_MOTOR4,
    VESC_ID_MOTOR5,
};

/* -----------------------------------------------------------------------
 * Private state
 * ----------------------------------------------------------------------- */
static CAN_HandleTypeDef *_hcan  = NULL;
static VescStatus_t       _status[5] = {0};

/* -----------------------------------------------------------------------
 * Internal: send a 4-byte extended-ID CAN frame
 * ----------------------------------------------------------------------- */
static void VESC_Send(uint8_t vesc_id, uint8_t cmd_id, int32_t value)
{
    if (_hcan == NULL) return;

    /* Wait for a free TX mailbox, timeout 10ms */
    uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
    {
        if (HAL_GetTick() - start > 10) return;
    }

    CAN_TxHeaderTypeDef header;
    header.ExtId              = ((uint32_t)cmd_id << 8) | (uint32_t)vesc_id;
    header.IDE                = CAN_ID_EXT;
    header.RTR                = CAN_RTR_DATA;
    header.DLC                = 4;
    header.TransmitGlobalTime = DISABLE;

    /* Big-endian encode */
    uint8_t data[4];
    data[0] = (value >> 24) & 0xFF;
    data[1] = (value >> 16) & 0xFF;
    data[2] = (value >>  8) & 0xFF;
    data[3] = (value >>  0) & 0xFF;

    uint32_t mailbox;
    HAL_CAN_AddTxMessage(_hcan, &header, data, &mailbox);
}

/* -----------------------------------------------------------------------
 * Init
 * ----------------------------------------------------------------------- */
void VESC_Init(CAN_HandleTypeDef *hcan)
{
    _hcan = hcan;

    /* Pass-all filter for extended frames on FIFO0 */
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(_hcan, &filter);

    HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(_hcan);
}

/* -----------------------------------------------------------------------
 * Transmit API
 * ----------------------------------------------------------------------- */
void VESC_SetDuty(VescMotor_t motor, float duty)
{
    if (motor >= VESC_NUM_MOTORS) return;
    if (duty >  1.0f) duty =  1.0f;
    if (duty < -1.0f) duty = -1.0f;
    VESC_Send(vesc_ids[motor], CAN_PACKET_SET_DUTY, (int32_t)(duty * 100000.0f));
}

void VESC_SetCurrent(VescMotor_t motor, float amps)
{
    if (motor >= VESC_NUM_MOTORS) return;
    VESC_Send(vesc_ids[motor], CAN_PACKET_SET_CURRENT, (int32_t)(amps * 1000.0f));
}

void VESC_SetERPM(VescMotor_t motor, int32_t erpm)
{
    if (motor >= VESC_NUM_MOTORS) return;
    VESC_Send(vesc_ids[motor], CAN_PACKET_SET_RPM, erpm);
}

void VESC_SetDutyAll(float duty)
{
    for (int i = 0; i < VESC_NUM_MOTORS; i++)
        VESC_SetDuty((VescMotor_t)i, duty);
}

void VESC_KeepAlive(VescMotor_t motor)
{
    if (motor >= VESC_NUM_MOTORS) return;
    VESC_Send(vesc_ids[motor], CAN_PACKET_SET_CURRENT, 0);
}

void VESC_KeepAliveAll(void)
{
    for (int i = 0; i < VESC_NUM_MOTORS; i++)
        VESC_KeepAlive((VescMotor_t)i);
}

void VESC_Brake(VescMotor_t motor)
{
    if (motor >= VESC_NUM_MOTORS) return;
    /* SET_CURRENT_HANDBRAKE (cmd 12): resists movement from current rotor
     * position without snapping to a new flux alignment angle on startup.
     * This is the correct command for holding position at standstill. */
    VESC_Send(vesc_ids[motor], CAN_PACKET_SET_CURRENT_HANDBRAKE,
              (int32_t)(VESC_BRAKE_CURRENT * 1000.0f));
}

void VESC_BrakeAll(void)
{
    for (int i = 0; i < VESC_NUM_MOTORS; i++)
        VESC_Brake((VescMotor_t)i);
}

/* -----------------------------------------------------------------------
 * Receive - parse incoming CAN Status 1 frames from all VESCs
 * Call from HAL_CAN_RxFifo0MsgPendingCallback
 * ----------------------------------------------------------------------- */
void VESC_ProcessRx(void)
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(_hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
            break;

        if (header.IDE != CAN_ID_EXT) continue;

        uint8_t cmd_id  = (header.ExtId >> 8) & 0xFF;
        uint8_t vesc_id = (header.ExtId >> 0) & 0xFF;

        /* Find which motor slot this ID belongs to */
        int motor_idx = -1;
        for (int i = 0; i < VESC_NUM_MOTORS; i++)
        {
            if (vesc_ids[i] == vesc_id)
            {
                motor_idx = i;
                break;
            }
        }
        if (motor_idx < 0) continue;

        /* Status 1: ERPM (int32), Current (int16 /10), Duty (int16 /1000) */
        if (cmd_id == CAN_PACKET_STATUS_1 && header.DLC >= 8)
        {
            int32_t erpm = (int32_t)(
                ((uint32_t)data[0] << 24) |
                ((uint32_t)data[1] << 16) |
                ((uint32_t)data[2] <<  8) |
                ((uint32_t)data[3]));

            int16_t current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
            int16_t duty_raw    = (int16_t)(((uint16_t)data[6] << 8) | data[7]);

            _status[motor_idx].erpm    = erpm;
            _status[motor_idx].current = (float)current_raw / 10.0f;
            _status[motor_idx].duty    = (float)duty_raw    / 1000.0f;
            _status[motor_idx].updated = 1;
        }
    }
}

VescStatus_t* VESC_GetStatus(VescMotor_t motor)
{
    if (motor >= 5) return NULL;
    return &_status[motor];
}
