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
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#include "lcd.h"

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

#define SERVO_CENTOR_OFFSET (100)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
struct
{
  uint8_t header;
  int16_t head_1;
  int16_t head_2;
  int16_t head_3;
  int16_t speed;
  uint8_t sum;
} tlm_msg;
volatile int16_t tmp = 0;
volatile uint32_t tlm_rx_idx = 0;

void parseTelemetryCmt(uint8_t rx_data)
{
  switch (tlm_rx_idx) {
    case 0:
      if (rx_data == 0x2c || rx_data == 0x2d) {
        tlm_rx_idx++;
      }
      break;
    case 1:
      tmp = rx_data << 8;
      tlm_rx_idx++;
      break;
    case 2:
      tmp |= rx_data;
      tlm_msg.head_1 = tmp;
      tlm_rx_idx++;
      break;

    case 3:
      tmp = rx_data << 8;
      tlm_rx_idx++;
      break;
    case 4:
      tmp |= rx_data;
      tlm_msg.head_2 = tmp;
      tlm_rx_idx++;
      break;

    case 5:
      tmp = rx_data << 8;
      tlm_rx_idx++;
      break;
    case 6:
      tmp |= rx_data;
      tlm_msg.head_3 = tmp;
      tlm_rx_idx++;
      break;

    case 7:
      tmp = rx_data << 8;
      tlm_rx_idx++;
      break;
    case 8:
      tmp |= rx_data;
      tlm_msg.speed = tmp;
      tlm_rx_idx++;
      break;

    case 9:
      tlm_msg.sum = rx_data;
      tlm_rx_idx = 0;
      break;

    default:
      tlm_rx_idx = 0;
      break;
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char * ptr, int len)
{
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len);  // 2ms
  return len;
}

#define ESC_TLM_BUF_SIZE (100)
uint8_t uart_rx_buf[10] = {0};
uint8_t uart3_rx_buf[10] = {0}, esc_tlm_buf[ESC_TLM_BUF_SIZE];
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
    esc_tlm_buf[uart3_rx_cnt] = uart3_rx_buf[0];
    uart3_rx_cnt++;
    parseTelemetryCmt(uart3_rx_buf[0]);
    HAL_UART_Receive_IT(&huart3, uart3_rx_buf, 1);
  }
}

volatile float serv_angle = 0, dribbler_speed = 0;
volatile int servo_timeout_cnt = 0, dribbler_timeout_cnt = 0;
volatile float battery_voltage = 0;
volatile uint32_t can_rx_cnt = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  can_msg_buf_t can_rx_buf;
  CAN_RxHeaderTypeDef can_rx_header;
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

void ball_sensor(void)
{
  static can_msg_buf_t ball_msg, voltage_msg, speed_msg;

  static int32_t ball_detect_process = 0;
  static int32_t adc_raw[3];

  bool ball_detected[2];

  switch (ball_detect_process) {
    case 0:
      HAL_GPIO_WritePin(PHOTO_0_GPIO_Port, PHOTO_0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PHOTO_1_GPIO_Port, PHOTO_1_Pin, GPIO_PIN_SET);
      adc_raw[0] = HAL_ADC_GetValue(&hadc2);
      ball_detect_process++;

      speed_msg.speed = tlm_msg.speed;
      can_send(0x204, speed_msg);
      break;

    case 1:
      HAL_GPIO_WritePin(PHOTO_0_GPIO_Port, PHOTO_0_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(PHOTO_1_GPIO_Port, PHOTO_1_Pin, GPIO_PIN_RESET);

      adc_raw[1] = HAL_ADC_GetValue(&hadc2);

      ball_detect_process++;
      battery_voltage = battery_voltage * 0.9 + (HAL_ADC_GetValue(&hadc1) * 36.3 / 4096) * 0.1;
      voltage_msg.voltage = battery_voltage;
      can_send(0x214, voltage_msg);
      break;

    case 2:

      HAL_GPIO_WritePin(PHOTO_0_GPIO_Port, PHOTO_0_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(PHOTO_1_GPIO_Port, PHOTO_1_Pin, GPIO_PIN_RESET);

      adc_raw[2] = HAL_ADC_GetValue(&hadc2);

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
        ball_msg.data[0] = 1;
        ball_msg.data[1] = 0;
      } else if (ball_detected[1]) {
        ball_msg.data[0] = 1;
        ball_msg.data[1] = 5;
      } else {
        ball_msg.data[0] = 0;
        ball_msg.data[1] = 0;
      }

      can_send(0x240, ball_msg);

      ball_detect_process = 0;
      break;

    default:
      ball_detect_process = 0;
      break;
  }
}

// 2kHz cycle
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  static uint32_t print_interval = 0;
  ball_sensor();

  print_interval++;
  if (print_interval >= 200) {
    print_interval = 0;
    printf(
      "\e[0mbatt %4.1f spd %4d Mbx %ld can rx %3ld uart rx %4ld %4ld dribbler %6.3f servo %6.3f timeout %4d %4d ball %+5ld %+5ld %d%d \n", battery_voltage, tlm_msg.speed,
      HAL_CAN_GetTxMailboxesFreeLevel(&hcan), can_rx_cnt, uart_rx_cnt, uart3_rx_cnt, dribbler_speed, serv_angle, dribbler_timeout_cnt, servo_timeout_cnt, ball_detect[0], ball_detect[1], uart3_rx_flag,
      uart_rx_flag);

    if (HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin) == GPIO_PIN_RESET) {
      lcdPrint(&hi2c1, battery_voltage, can_rx_cnt, ball_detect[0], ball_detect[1]);
    }

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
      htim3.Instance->CCR4 = 1500 - 600 * serv_angle + SERVO_CENTOR_OFFSET;  // servo
    } else {
      htim3.Instance->CCR4 = 1500 - USER_SW_SERVO_PULSE_WITDH + SERVO_CENTOR_OFFSET;  // servo
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
  MX_TIM17_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("start sub board !! %s %s\n", __DATE__, __TIME__);
  HAL_UART_Receive_IT(&huart3, uart3_rx_buf, 1);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

  lcdInit(&hi2c1);

  CAN_Filter_Init();
  HAL_CAN_Start(&hcan);

  HAL_TIM_Base_Start_IT(&htim17);

  HAL_TIM_PWM_Init(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  htim3.Instance->CCR3 = 0;
  htim3.Instance->CCR4 = 0;
  servo_timeout_cnt = 0;
  dribbler_timeout_cnt = 0;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
