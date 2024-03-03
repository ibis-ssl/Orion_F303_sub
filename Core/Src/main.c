/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define BALL_DETECTOR_THRESH (2000)

#define USER_SW_SERVO_PULSE_WITDH (500)
#define USER_SW_ESC_PULSE_WITDH (300)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char * ptr, int len)
{
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len);  // 2ms
  return len;
}

uint8_t uart_rx_buf[10] = {0};
uint8_t uart3_rx_buf[10] = {0};
volatile bool uart_rx_flag = false, uart3_rx_flag = false;
volatile uint32_t uart_rx_cnt = 0, uart3_rx_cnt = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance == USART1) {
    uart_rx_flag = true;
    uart_rx_cnt++;
    HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);
  } else if (huart->Instance == USART3) {
    uart3_rx_flag = true;
    uart3_rx_cnt++;
    HAL_UART_Receive_IT(&huart3, uart3_rx_buf, 1);
  }
}

float serv_angle = 0, dribbler_speed = 0;
int servo_timeout_cnt = 0, dribbler_timeout_cnt = 0;
uint32_t can_rx_cnt = 0;
can_msg_buf_t can_rx_buf;
CAN_RxHeaderTypeDef can_rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buf.data) != HAL_OK) {
    /* Reception Error */
    Error_Handler();
  }

  can_rx_cnt++;
  switch (can_rx_header.StdId) {
    case 0x104:
      dribbler_timeout_cnt = 0;
      dribbler_speed = can_rx_buf.speed;
      break;

    case 0x105:
      servo_timeout_cnt = 0;
      serv_angle = can_rx_buf.speed;
      break;

    case 0x300:
      break;
    default:
      break;
  }
}

int32_t ball_detect[2] = {0, 0};

static CAN_TxHeaderTypeDef can_header;
static uint32_t can_mailbox;
static uint8_t can_data[8];

static uint8_t ball_detect_cycle_cnt = 0;

void ball_sensor(void)
{
  static int32_t ball_detect_process = 0;
  static int32_t adc_raw[3];

  bool ball_detected[2];

  switch (ball_detect_process) {
    case 0:
      HAL_GPIO_WritePin(PHOTO_0_GPIO_Port, PHOTO_0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PHOTO_1_GPIO_Port, PHOTO_1_Pin, GPIO_PIN_RESET);
      ball_detect_process++;
      break;
    case 1:
      adc_raw[0] = HAL_ADC_GetValue(&hadc2);
      ball_detect_process++;
      break;
    case 2:
      HAL_GPIO_WritePin(PHOTO_0_GPIO_Port, PHOTO_0_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PHOTO_1_GPIO_Port, PHOTO_1_Pin, GPIO_PIN_RESET);
      ball_detect_process++;
      break;
    case 3:
      adc_raw[1] = HAL_ADC_GetValue(&hadc2);
      ball_detect_process++;
      break;
    case 4:
      HAL_GPIO_WritePin(PHOTO_0_GPIO_Port, PHOTO_0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PHOTO_1_GPIO_Port, PHOTO_1_Pin, GPIO_PIN_SET);
      ball_detect_process++;
      break;
    case 5:
      adc_raw[2] = HAL_ADC_GetValue(&hadc2);
      ball_detect_process++;
      ball_detect[0] = adc_raw[0] - adc_raw[1];
      ball_detect[1] = adc_raw[0] - adc_raw[2];

      if (HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin) == GPIO_PIN_RESET) {
        ball_detect[0] = 0;
        ball_detect[1] = 0;
      }

      //
      if (ball_detect[1] < BALL_DETECTOR_THRESH) {
        ball_detected[0] = true;
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
      } else {
        ball_detected[0] = false;
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
      }
      if (ball_detect[0] < BALL_DETECTOR_THRESH) {
        ball_detected[1] = true;
        HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
      } else {
        ball_detected[1] = false;
        HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
      }

      if (ball_detected[0]) {
        can_data[0] = 1;
        can_data[1] = 0;
      } else if (ball_detected[1]) {
        can_data[0] = 1;
        can_data[1] = 5;
      } else {
        can_data[0] = 0;
        can_data[1] = 0;
      }
      ball_detect_cycle_cnt++;

      can_header.StdId = 0x240;
      can_header.RTR = CAN_RTR_DATA;
      can_header.DLC = 4;
      can_header.IDE = CAN_ID_STD;
      can_header.TransmitGlobalTime = DISABLE;
      HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &can_mailbox);
      break;
    default:
      ball_detect_process = 0;
      break;
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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  printf("start sub board 0426!!\n");
  HAL_UART_Receive_IT(&huart3, uart3_rx_buf, 1);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

  CAN_Filter_Init();
  HAL_CAN_Start(&hcan);

  HAL_TIM_PWM_Init(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  htim3.Instance->CCR3 = 0;
  htim3.Instance->CCR4 = 0;
  servo_timeout_cnt = 0;
  dribbler_timeout_cnt = 0;

  uint32_t print_interval = 0;

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    /* Calibration Error */
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    /* Calibration Error */
    Error_Handler();
  }

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_Delay(1);
    ball_sensor();

    print_interval++;
    if (print_interval >= 100) {
      print_interval = 0;
      printf(
        "ball %3d mbx %ld free %ld can rx %3ld uart rx %4ld %4ld dribbler %6.3f servo %6.3f timeout %4d %4d ball %+5ld %+5ld %d%d\n", ball_detect_cycle_cnt, can_mailbox,
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan), can_rx_cnt, uart_rx_cnt, uart3_rx_cnt, dribbler_speed, serv_angle, dribbler_timeout_cnt, servo_timeout_cnt, ball_detect[0], ball_detect[1],uart3_rx_flag,uart_rx_flag);
      ball_detect_cycle_cnt = 0;
      // TEL (LED0,PA3)
      if (uart3_rx_cnt > 0) {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
      }

      // RX (can rx,LED2,PA5)
      if (can_rx_cnt > 0) {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
      }

      if (dribbler_speed != 0) {
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
      } else {
        HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
      }

      can_rx_cnt = 0;
      uart_rx_cnt = 0;
      uart3_rx_cnt = 0;

      if (HAL_GPIO_ReadPin(SW_0_GPIO_Port, SW_0_Pin) == GPIO_PIN_SET) {
        htim3.Instance->CCR3 = 1500 + 600 * dribbler_speed;  // esc
      } else {
        htim3.Instance->CCR3 = 1500 + USER_SW_ESC_PULSE_WITDH;  // esc
      }

      if (HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin) == GPIO_PIN_SET) {
        htim3.Instance->CCR4 = 1500 - 600 * serv_angle;  // servo
      } else {
        htim3.Instance->CCR4 = 1500 - USER_SW_SERVO_PULSE_WITDH;  // servo
      }

      dribbler_timeout_cnt++;
      servo_timeout_cnt++;
      if (dribbler_timeout_cnt > 50) {
        dribbler_speed = 0;
      }
      if (servo_timeout_cnt > 50) {
        serv_angle = 0;
      }
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  while (1) {
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
