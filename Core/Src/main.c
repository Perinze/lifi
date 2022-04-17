/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*  
#define CARRIER_FREQ 2000.0
#define CARRIER_PERIOD (1.0 / CARRIER_FREQ)
#define CARRIER_HALF_PERIOD (CARRIER_PERIOD / 2.0)
#define CARRIER_FREQ_AMP 666.6
#define INPUT_AMP 1.0
#define CVF (CARRIER_FREQ_AMP / INPUT_AMP)
#define LIGHT_TH 2850
*/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t queue[QUESIZ];
uint8_t qfront, qback;

uint8_t recvbuf[RECVSIZ];
uint8_t recvtmp[RECVSIZ+4];
uint8_t recvidx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_us(uint32_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void recv2uart() {
  // HAL_UART_Transmit_IT(&huart2, "recv2uart\r\n", sizeof("recv2uart\r\n"));
  // HAL_Delay(1000);
  char buf[128] = {0};
  char *p = buf;
  for (uint8_t i = 0; i < recvidx; i++) {
    p += sprintf(p, " %02x", recvbuf[i]);
  }
  p += sprintf(p, "\r\n");
  HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, sizeof(buf));
  recvidx = 0;
  /*
  char str[128] = {0};
  unsigned char *pin = recvbuf;
  const char *hex = "0123456789ABCDEF";
  char *pout = str;
  int i = 0;
  for(; i < sizeof(recvbuf)-1; ++i){
    *pout++ = hex[(*pin>>4)&0xF];
    *pout++ = hex[(*pin++)&0xF];
    *pout++ = ' ';
  }
  *pout++ = hex[(*pin>>4)&0xF];
  *pout++ = hex[(*pin)&0xF];
  *pout++ = '\n';
  *pout = 0;

  HAL_UART_Transmit(&huart2, str, sizeof(str), 1);
  */
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
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LD0_GPIO_Port->ODR |= (LD0_Pin);
  // LD2_GPIO_Port->ODR |= (LD2_Pin);

  /*
  recvidx = 0;
  recvbuf[recvidx++] = 0x12;
  recvbuf[recvidx++] = 0x34;
  recvbuf[recvidx++] = 0x56;
  recvbuf[recvidx++] = 0x78;
  recv2uart();
  */

  HAL_Delay(2000);

  // push data to queue
  // frame start
  queue[qback++] = FRMBEGIN1;
  queue[qback++] = FRMBEGIN2;

  // data
  queue[qback++] = 0x11;
  queue[qback++] = 0x45;
  queue[qback++] = 0x14;
  queue[qback++] = 0x19;
  queue[qback++] = 0x19;
  queue[qback++] = 0x81;
  queue[qback++] = 0x00;

  // frame end
  queue[qback++] = FRMEND1;
  queue[qback++] = FRMEND2;

  // redundancy
  queue[qback++] = 0x00;
  queue[qback++] = 0x00;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_ADC_Start_IT(&hadc3);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
