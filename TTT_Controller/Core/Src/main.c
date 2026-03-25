/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TTT 5-joint robotic arm controller
  *
  * Serial protocol (host PC → STM32, 115200 baud, newline-terminated):
  *
  *   J j1 j2 j3 j4 j5   Joystick axes, each -1.0 … +1.0   (send at ~50 Hz)
  *   G n kg              Set gravity feedforward gain on joint n (1-5)
  *   P n kp kv           Set PD gains on joint n
  *   H                   Home — zero all position estimates at current pose
  *   E                   Emergency stop
  *   R                   Print joint status
  *   ?                   Help
  *
  * The host PC should poll the Xbox controller and stream J commands at 50 Hz.
  * If no J command arrives within 250 ms the STM32 will brake all motors.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vesc_can.h"
#include "arm_controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_BUF_LEN      80    /* max incoming serial command length         */
#define JOY_WATCHDOG_MS  250   /* brake if no J command received in this ms  */
#define CTRL_PERIOD_MS   10    /* control loop period → 100 Hz               */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint8_t  uart_rx_byte;
static char     cmd_buf[CMD_BUF_LEN];
static uint8_t  cmd_idx   = 0;
static uint8_t  cmd_ready = 0;

/* Joystick axes, written by Process_Command() and read by the control loop */
static float    joy_axes[NUM_JOINTS] = {0};

static uint32_t last_joy_tick  = 0;   /* timestamp of last J command         */
static uint32_t last_ctrl_tick = 0;   /* timestamp of last control loop tick  */
static uint8_t  watchdog_active = 0;  /* 1 once brake has been sent; cleared when J resumes */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Process_Command(const char *cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* CAN RX interrupt → drain FIFO and update VESC status structs */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    VESC_ProcessRx();
}

/* UART RX interrupt — accumulate bytes into cmd_buf, set cmd_ready on newline */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    char c = (char)uart_rx_byte;
    if (c == '\n' || c == '\r')
    {
        if (cmd_idx > 0)
        {
            cmd_buf[cmd_idx] = '\0';
            cmd_ready = 1;
            cmd_idx   = 0;
        }
    }
    else if (cmd_idx < CMD_BUF_LEN - 1)
    {
        cmd_buf[cmd_idx++] = c;
    }
    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);
}

/*
 * Process one null-terminated command string.
 *
 * J commands arrive at ~50 Hz and get no response (keep the channel clear).
 * All other commands are interactive and get a short ACK or data reply.
 */
static void Process_Command(const char *cmd)
{
    char resp[160];
    resp[0] = '\0';

    if (cmd[0] == 'J' || cmd[0] == 'j')
    {
        /* J j1 j2 j3 j4 j5 — joystick update from host PC
         * Values are floats in the range -1.0 … +1.0.
         * Example: "J 0.50 -0.25 0.00 0.75 -0.10"                         */
        char *p = (char *)cmd + 1;
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            float v    = strtof(p, &p);
            joy_axes[i] = v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
        }
        last_joy_tick = HAL_GetTick();
        return;  /* no response — this path runs at 50 Hz */
    }
    else if (cmd[0] == 'V' || cmd[0] == 'v')
    {
        /* V n v_max — set max output-shaft speed for joint n (1-based) [rev/s] */
        char *p    = (char *)cmd + 1;
        int   n    = (int)strtol(p, &p, 10) - 1;
        float vmax = strtof(p, &p);
        ArmController_SetVMax(n, vmax);
        snprintf(resp, sizeof(resp), "OK V J%d v_max=%.3f\r\n", n + 1, vmax);
    }
    else if (cmd[0] == 'G' || cmd[0] == 'g')
    {
        /* G n kg — set gravity feedforward gain for joint n (1-based) */
        char *p  = (char *)cmd + 1;
        int   n  = (int)strtol(p, &p, 10) - 1;
        float kg = strtof(p, &p);
        ArmController_SetGravityGain(n, kg);
        snprintf(resp, sizeof(resp), "OK G J%d kg=%.3f\r\n", n + 1, kg);
    }
    else if (cmd[0] == 'P' || cmd[0] == 'p')
    {
        /* P n kp kv — set PD gains for joint n (1-based) */
        char *p  = (char *)cmd + 1;
        int   n  = (int)strtol(p, &p, 10) - 1;
        float kp = strtof(p, &p);
        float kv = strtof(p, &p);
        ArmController_SetPDGains(n, kp, kv);
        snprintf(resp, sizeof(resp), "OK P J%d kp=%.2f kv=%.2f\r\n",
                 n + 1, kp, kv);
    }
    else if (cmd[0] == 'H' || cmd[0] == 'h')
    {
        /* H — home: zero all position estimates at the current physical pose */
        ArmController_Home();
        snprintf(resp, sizeof(resp), "OK HOME\r\n");
    }
    else if (cmd[0] == 'E' || cmd[0] == 'e')
    {
        /* E — emergency stop */
        ArmController_EStop();
        last_joy_tick = 0;   /* keep watchdog triggered until J resumes */
        snprintf(resp, sizeof(resp), "OK ESTOP\r\n");
    }
    else if (cmd[0] == 'R' || cmd[0] == 'r')
    {
        ArmController_PrintStatus(&huart3);
        return;
    }
    else if (cmd[0] == 'C' || cmd[0] == 'c')
    {
        /* C — CAN bus diagnostic: error counters + receive frame count */
        VESC_PrintDiag(&huart3);
        return;
    }
    else if (cmd[0] == '?')
    {
        const char *help =
            "J j1..j5        Joystick axes -1.0..+1.0  (50 Hz stream)\r\n"
            "G n kg          Gravity gain joint n (1-5)\r\n"
            "P n kp kv       PD gains joint n\r\n"
            "H               Home (zero position estimates)\r\n"
            "E               Emergency stop\r\n"
            "R               Print joint status\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t *)help, strlen(help), 300);
        return;
    }
    else
    {
        snprintf(resp, sizeof(resp), "ERR unknown: %.40s\r\n", cmd);
    }

    if (resp[0] != '\0')
        HAL_UART_Transmit(&huart3, (uint8_t *)resp, strlen(resp), 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  VESC_Init(&hcan1);
  ArmController_Init();

  /* Safe start: brake all motors before enabling the control loop */
  VESC_BrakeAll();
  HAL_Delay(100);

  const char *welcome =
      "TTT Arm Controller ready\r\n"
      "Stream: J j1 j2 j3 j4 j5  at 50 Hz\r\n"
      "Type ? for help, R for status\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)welcome, strlen(welcome), 300);

  HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1);

  /* Do NOT satisfy the watchdog here.  The arm stays braked until the PC
   * sends its first J command.  This prevents the control loop from
   * outputting gravity-comp current before the operator is ready.         */
  last_joy_tick  = 0;
  last_ctrl_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t now = HAL_GetTick();

    /* --- CAN bus-off recovery -------------------------------------------
     * If the CAN peripheral goes bus-off (e.g. a node disconnects while
     * we are transmitting), restart it so it recovers automatically.     */
    if (hcan1.Instance->ESR & CAN_ESR_BOFF)
    {
        HAL_CAN_Stop(&hcan1);
        HAL_CAN_Start(&hcan1);
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }

    /* --- Drain CAN RX FIFO (belt-and-suspenders alongside the ISR).
     * If frames arrive between ISR triggers, or if the interrupt is not
     * firing due to a hardware issue, this ensures they are processed
     * at least once per main-loop iteration (~10 ms worst case).        */
    VESC_ProcessRx();

    /* --- Process any buffered UART command (runs as fast as the loop) --- */
    if (cmd_ready)
    {
        cmd_ready = 0;
        Process_Command(cmd_buf);
    }

    /* --- 100 Hz control loop -------------------------------------------- */
    if (now - last_ctrl_tick >= CTRL_PERIOD_MS)
    {
        float dt       = (float)(now - last_ctrl_tick) * 0.001f;
        last_ctrl_tick = now;

        if (now - last_joy_tick > JOY_WATCHDOG_MS)
        {
            /* Host PC comms lost.  Send brake once when watchdog first fires,
             * then hold — do NOT call VESC_BrakeAll() every tick or the CAN
             * bus will be flooded with 500 frames/sec and go bus-off.       */
            if (!watchdog_active)
            {
                watchdog_active = 1;
                ArmController_EStop();   /* calls VESC_BrakeAll() once */
                for (int i = 0; i < NUM_JOINTS; i++) joy_axes[i] = 0.0f;
            }
        }
        else
        {
            watchdog_active = 0;
            ArmController_Update(joy_axes, dt);
        }
    }

  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
