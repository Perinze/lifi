/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "usart.h"
#include "tim.h"
#include "main.h"
/* USER CODE END 0 */

ADC_HandleTypeDef hadc3;

/* ADC3 init function */
void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC3 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC3 interrupt Init */
    HAL_NVIC_SetPriority(ADC3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC3_IRQn);
  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_IN1
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);

    /* ADC3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC3_IRQn);
  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint32_t ad_res = HAL_ADC_GetValue(&hadc3);
    /*
    #ifdef LIGHT_DEBUG
      char buf[8] = {0};
      sprintf(buf, "%u\r\n", ad_res);
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, sizeof(buf));
    #elif
    */
    static int start;
    static int end;
    /*
      0 idle
      1 wait for posedge
      2 wait for negedge
    */
    static int state = 0;
    static uint8_t data = 0;
    static uint8_t last_data = 0;
    static uint8_t bitcnt = 0;

    if (!state && ad_res > LIGHTTH) {
      state = 2;
      start = __HAL_TIM_GetCounter(&htim2);
    }
    if (state == 1 && ad_res > LIGHTTH) {
      state = 2;
      start = __HAL_TIM_GetCounter(&htim2);
    }
    if (state == 2 && ad_res < LIGHTTH) {
      state = 1;
      end = __HAL_TIM_GetCounter(&htim2);
      int pulse = end - start;
      int bit = pulse > PULSEDIV ? 0 : 1;

      /*
      char buf[8] = {0};
      sprintf(buf, "%d ", bit);
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, sizeof(buf));
      */
      
      data = ((data << 1) | bit);
      if (bitcnt) bitcnt--;
    }

    static int byte_state = 1;
    /*
      0 idle
      1 wait for frame begin 1
      2 wait for frame begin 2
      3 wait for frame end 1, receive data
      4 wait for frame end 2, receive data, assume just received frame end 1 as data
      5 frame end all matched, end packet receive, back to state 1
    */
    if (!bitcnt && byte_state > 1) {
      // char buf[64] = {0};
      // sprintf(buf, "data::0x%02x\tlast_data::0x%02x\tbyte_state::%d\r\n", (int)data, (int)last_data, byte_state);
      // HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, sizeof(buf));
    }

    if (byte_state == 1 && data == FRMBEGIN1) {
      // HAL_UART_Transmit(&huart2, "state 1\r\n", sizeof("\r\nstate 1\r\n"), 0xff);
      bitcnt = 8;
      byte_state = 2;
      last_data = data;
    } else if (!bitcnt && byte_state == 2 && data == FRMBEGIN2) {
      // HAL_UART_Transmit(&huart2, "state 2\r\n", sizeof("\r\nstate 2\r\n"), 0xff);
      bitcnt = 8;
      byte_state = 3;
      recvidx = 0;
      last_data = data;
    } else if (!bitcnt && byte_state == 3) {
      // HAL_UART_Transmit(&huart2, "state 3\r\n", sizeof("\r\nstate 3\r\n"), 0xff);
      bitcnt = 8;
      /*
      char str[16];
      sprintf(str, "%1x", (int)(data & 0xf));
      sprintf(str, "%1x", (int)((data >> 4) & 0xf));
      */
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)&data, sizeof(data));
      /* recvbuf[recvidx++] = data; */
      if (data == FRMEND2 && last_data == FRMEND1) {
        // HAL_UART_Transmit(&huart2, "state 4\r\n", sizeof("\r\nstate 4\r\n"), 0xff);
        LD2_GPIO_Port->ODR |= (LD2_Pin);
        recvidx -= 2;
        /* recv2uart(); */
        bitcnt = 0;
        byte_state = 1;
        LD2_GPIO_Port->ODR &= (~LD2_Pin);
        return;
      }
      last_data = data;
    }/* else if (byte_state == 4) {
      HAL_UART_Transmit_IT(&huart2, "state 4\r\n", sizeof("state 4\r\n"));
      recvidx -= 2;
      recv2uart();
      bitcnt = 0;
      byte_state = 1;
      */
      /*
      bitcnt = 8;
      recvbuf[recvidx++] = data;
      if (data == FRMEND2) {
        recvidx--;
        byte_state = 1;
        recv2uart();
      }
    }
      */
   
    /*
    if (data) {
      char buf[8] = {0};
      sprintf(buf, "%d\r\n", data);
      data &= 0x11111;
      HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, sizeof(buf));
      // data = 0;
    }
    */
    
    // HAL_ADC_Start_IT(&hadc3);
}
/* USER CODE END 1 */
