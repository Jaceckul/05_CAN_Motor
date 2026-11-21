/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t angle;                //编码器值，0~8191
  int16_t  speed;                //转速 rpm，正转正值、反转负值
  int16_t  real_current;         //电流的原始报文值，-16384~16384 对应 -20~20A
  float    real_current_handle;  //一阶低通滤波
  uint8_t  temp;                 //温度，单位 ℃
} Motor_Measure_t;

// 一个电调的收发信息
typedef struct
{
  CAN_TxHeaderTypeDef txconf;
  uint8_t  tx_buff[8];
  uint32_t tx_mailbox;
  uint32_t rx_id;
  uint8_t  rx_buff[8];
  CAN_RxHeaderTypeDef rx_header;
} Motor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ---- CAN 报文相关 ---- */
#define MOTOR_ID        1U
#define MOTOR_TX_ID     0x200U
#define MOTOR_RX_ID     0x201U
#define MOTOR_DATA_LEN  8U

/* ---- 正弦电流指令相关 ---- */
#define SINE_AMPLITUDE  1000.0f
#define SINE_STEP       0.02f
#define PI_F            3.14159265f

/* ---- 电流滤波相关 ---- */
#define FILTER_ALPHA    0.1f // 一阶低通系数
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//全局变量才能监视
Motor_t         motor1;
Motor_Measure_t motor1_measure;
int16_t         current_cmd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void motor_init(void);
static void motor_send(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 初始化：配过滤器 -> 启动 CAN1 -> 打开 FIFO1 接收通知 -> 填好发送报文头
static void motor_init(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank           = 0;
  filter.SlaveStartFilterBank = 14;
  filter.FilterMode           = CAN_FILTERMODE_IDLIST;
  filter.FilterScale          = CAN_FILTERSCALE_16BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  filter.FilterActivation     = CAN_FILTER_ENABLE;//激活过滤器

  filter.FilterIdHigh         = (uint16_t)(MOTOR_RX_ID << 5);
  filter.FilterIdLow          = (uint16_t)(MOTOR_RX_ID << 5);
  filter.FilterMaskIdHigh     = (uint16_t)(MOTOR_RX_ID << 5);
  filter.FilterMaskIdLow      = (uint16_t)(MOTOR_RX_ID << 5);

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)//FIFO1新报文回调
  {
    Error_Handler();
  }

  motor1.rx_id = MOTOR_RX_ID;

  motor1.txconf.StdId = MOTOR_TX_ID;
  motor1.txconf.IDE   = CAN_ID_STD;
  motor1.txconf.RTR   = CAN_RTR_DATA;
  motor1.txconf.DLC   = MOTOR_DATA_LEN;
}

/* 发送一帧控制报文：current_cmd 拆成高低字节放进前两字节，其余填 0 */
static void motor_send(void)
{
  uint16_t raw = (uint16_t)current_cmd;
  uint8_t  i;

  motor1.tx_buff[0] = (uint8_t)(raw >> 8);
  motor1.tx_buff[1] = (uint8_t)(raw & 0xFFU);

  for (i = 2; i < MOTOR_DATA_LEN; i++) 
  {
    motor1.tx_buff[i] = 0; //只发1、2字节
  }

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)//空闲邮箱检测
  {
    HAL_CAN_AddTxMessage(&hcan1, &motor1.txconf,
                         motor1.tx_buff,
                         &motor1.tx_mailbox); 
  }
}

//FIFO1报文回调
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  uint8_t *p;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1,
                             &motor1.rx_header,
                             motor1.rx_buff) != HAL_OK)
    {
      return;
    }

    if (motor1.rx_header.StdId != motor1.rx_id)
    {
      continue;
    }

    if (motor1.rx_header.DLC != MOTOR_DATA_LEN)
    {
      continue;
    }

    p = motor1.rx_buff;

    motor1_measure.angle        = (uint16_t)((p[0] << 8) | p[1]);
    motor1_measure.speed        = (int16_t)((p[2] << 8) | p[3]);
    motor1_measure.real_current = (int16_t)((p[4] << 8) | p[5]);
    motor1_measure.temp         = p[6];

    motor1_measure.real_current_handle =
        (1.0f-FILTER_ALPHA) * motor1_measure.real_current_handle
      + FILTER_ALPHA * (float)motor1_measure.real_current;
  }
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
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  motor_init();

  float phase = 0.0f;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    current_cmd = (int16_t)(SINE_AMPLITUDE * sinf(phase));

    phase += SINE_STEP * PI_F;
    if (phase > 2.0f * PI_F)
    {
      phase -= 2.0f * PI_F;
    }

    motor_send();

    HAL_Delay(10); 
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
