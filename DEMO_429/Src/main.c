/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
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
uint8_t debug_buf[UART_MAX_LENGTH] = {0};
uint8_t wd_enable = 1;
uint8_t pinout_enable;
uint8_t reset_flag = 0;
RespondStruct resp_msg;
UartStu uart_msg;
RunState run_state;

#ifdef USE_SRAM_FOR_FW_IMG
uint8_t fw_buffer[FW_MAX_LENGTH] __attribute__((at(0x20020000)));
#endif

extern osSemaphoreId_t watchdogSemaphore;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);

  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();
 
  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 
 
  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
void Mon_Init(void)
{
  run_state.mode = RUN_MODE_APPLICATION;
  uart_msg.stage = UART_WAIT_START;
  wd_enable = 1;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET){
    SET_RESETFLAG(BOR_RESET_BIT);
    EPT("BOR Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET){
    SET_RESETFLAG(PIN_RESET_BIT);
    EPT("PIN Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET){
    SET_RESETFLAG(POR_RESET_BIT);
    EPT("POR/PDR Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET){
    SET_RESETFLAG(SFT_RESET_BIT);
    EPT("Software Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET){
    SET_RESETFLAG(IWDG_RESET_BIT);
    EPT("Independent Watchdog Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET){
    SET_RESETFLAG(WWDG_RESET_BIT);
    EPT("Window Watchdog Reset is set\n");
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET){
    SET_RESETFLAG(LPWR_RESET_BIT);
    EPT("Low-Power Reset is set\n");
  }
/*
  if (IS_RESETFLAG_SET(BOR_RESET_BIT)) {
  }
*/
  __HAL_RCC_CLEAR_RESET_FLAGS();
  
  pinout_enable = 0;
  HAL_GPIO_WritePin(PINOUT_GPIO_Port, PINOUT_Pin, GPIO_PIN_SET);

  // SPI_CS
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM8) {
    if (pinout_enable) {
      if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        // Rising Edge
        HAL_GPIO_WritePin(PINOUT_GPIO_Port, PINOUT_Pin, GPIO_PIN_SET);
      } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        // Failing Edge
        HAL_GPIO_WritePin(PINOUT_GPIO_Port, PINOUT_Pin, GPIO_PIN_RESET);
      }
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM3) {
    osSemaphoreRelease(watchdogSemaphore);
    //HAL_IWDG_Refresh(&hiwdg);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
