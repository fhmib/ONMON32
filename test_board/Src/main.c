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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t buf[256];
uint8_t debug_buf[256];
uint8_t fw_buf[40 * 1024];
uint32_t fw_length;
uint16_t fw_crc;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t ch;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (HAL_UART_Receive(&huart1, &ch, 1, 0xFFFFFFFF) == HAL_OK) {
      if (ch == 'A') {
        if (HAL_UART_Receive(&huart1, &ch, 1, 0xFFFFFFFF) == HAL_OK) {
          while (1) {
            send_cmd(ch, NULL);
            HAL_Delay(3000);
          }
        }
      } else {
        send_cmd(ch, NULL);
      }
    }
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    TOGGLE_LED1();
  }
}

uint32_t send_cmd(uint8_t ch, char *arg)
{
  uint32_t length, rcv_len;
  uint8_t rcv_crc;

  uint32_t every_size = 128, send_size = 0;
  uint16_t seq = 1;

  CLEAR_BIT(huart2.Instance->SR, USART_SR_RXNE);
  __HAL_UART_FLUSH_DRREGISTER(&huart2);

  switch (ch) {
    case '0':
      EPT("Close the watchdog\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 5;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_FOR_TEST;
      buf[CMD_SEQ_DATA] = 0;
      buf[CMD_SEQ_DATA + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 4);
      length = 6;
      break;
    case '1':
      EPT("Get SN\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 4;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_QUERY_SN;
      buf[CMD_SEQ_COMMAND_LOW + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 3);
      length = 5;
      break;
    case '2':
      EPT("Change mode to Application\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 5;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_UPGRADE_MODE;
      buf[CMD_SEQ_DATA] = 1;
      buf[CMD_SEQ_DATA + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 4);
      length = 6;
      break;
    case '3':
      EPT("Change mode to Upgrade\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 5;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_UPGRADE_MODE;
      buf[CMD_SEQ_DATA] = 2;
      buf[CMD_SEQ_DATA + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 4);
      length = 6;
      break;
    case '4':
      EPT("Download image...\n");
      HAL_UART_Receive(&huart1, fw_buf, 256, 0xFFFFFFFF);
      fw_length = (fw_buf[FW_FILE_FW_LENGTH] << 24) | (fw_buf[FW_FILE_FW_LENGTH + 1] << 16) |\
               (fw_buf[FW_FILE_FW_LENGTH + 2] << 8) | (fw_buf[FW_FILE_FW_LENGTH + 3] << 0);
      fw_crc = (fw_buf[FW_FILE_CRC] << 8) | fw_buf[FW_FILE_CRC + 1];
      HAL_UART_Receive(&huart1, fw_buf + 256, fw_length, 0xFFFFFFFF);
      for (int i = 0; i < 256; ++i) {
        EPT("%d = %#X\n", i, fw_buf[i]);
      }
      EPT("Download success, Length = %u, crc = %#X\n", fw_length, fw_crc);
      if (Cal_CRC16(&fw_buf[256], fw_length) == fw_crc) {
        EPT("CRC16 success\n");
      } else {
        EPT("CRC16 failed\n");
      }

      EPT("Send image...\n");
      while (send_size < fw_length + 256) {
        buf[CMD_SEQ_START] = 0x55;
        buf[CMD_SEQ_LENGTH] = 1 + 2 + 2 + 128 + 1;
        buf[CMD_SEQ_COMMAND_HIGH] = 0;
        buf[CMD_SEQ_COMMAND_LOW] = CMD_UPGRADE_DATA;
        buf[CMD_SEQ_DATA] = seq >> 8;
        buf[CMD_SEQ_DATA + 1] = seq++;
        if (every_size + send_size <= fw_length + 256) {
          memcpy(&buf[CMD_SEQ_DATA + 2], &fw_buf[send_size], every_size);
          send_size += every_size;
        } else {
          memset(&buf[CMD_SEQ_DATA + 2], 0, every_size);
          memcpy(&buf[CMD_SEQ_DATA + 2], &fw_buf[send_size], fw_length + 256 - send_size);
          send_size = fw_length + 256;
        }
        buf[CMD_SEQ_DATA + 2 + every_size] = Cal_Check(&buf[CMD_SEQ_LENGTH], 1 + 2 + 2 + 128);
        length = 1 + 1 + 2 + 2 + 128 + 1;
        if (HAL_UART_Transmit(&huart2, buf, length, 0xFF) != HAL_OK) {
          EPT("Transmit failed\n");
          return ~0;
        }
        if (HAL_UART_Receive(&huart2, buf, 2, 0xFFF) != HAL_OK) {
          EPT("Receive failed 1\n");
          return ~0;
        }
        rcv_len = buf[1] + 1;
        if (HAL_UART_Receive(&huart2, buf + 2, rcv_len - 2, 0xFFF) != HAL_OK) {
          EPT("Receive failed 2\n");
          return ~0;
        }
        EPT("Package %u send success\n", seq - 1);
        rcv_crc = Cal_Check(&buf[CMD_SEQ_LENGTH], rcv_len - 3);
        if (rcv_crc != buf[rcv_len - 1]) {
          EPT("Receive check failed\n");
          return ~0;
        }
        if (buf[CMD_SEQ_COMMAND_LOW + 1] != 0x0) {
          EPT("Receive failure packet\n");
          break;
        }
        if (seq == 3)
          HAL_Delay(100);
      }
      if (send_size >= fw_length + 256) {
        EPT("Send fw success\n");
        return 0;
      } else {
        EPT("Send fw failed\n");
        return ~0;
      }
    case '5':
      EPT("Run application\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 4;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_UPGRADE_RUN;
      buf[CMD_SEQ_COMMAND_LOW + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 3);
      length = 5;
      break;
    case '6':
      EPT("Get temperature\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 4;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_QUERY_TEMP;
      buf[CMD_SEQ_COMMAND_LOW + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 3);
      length = 5;
      break;
    case '7':
      EPT("Get version\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 4;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_QUERY_VERSION;
      buf[CMD_SEQ_COMMAND_LOW + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 3);
      length = 5;
      break;
    case '8':
      EPT("Reset\n");
      buf[CMD_SEQ_START] = 0x55;
      buf[CMD_SEQ_LENGTH] = 4;
      buf[CMD_SEQ_COMMAND_HIGH] = 0;
      buf[CMD_SEQ_COMMAND_LOW] = CMD_SOFTRESET;
      buf[CMD_SEQ_COMMAND_LOW + 1] = Cal_Check(&buf[CMD_SEQ_LENGTH], 3);
      length = 5;
      break;
    default:
      EPT("Unknown command\n");
      return 0;
  }

  if (HAL_UART_Transmit(&huart2, buf, length, 0xFF) != HAL_OK) {
    EPT("Transmit failed\n");
    return ~0;
  }
  if (HAL_UART_Receive(&huart2, buf, 2, 0xFFF) != HAL_OK) {
    EPT("Receive failed 1\n");
    return ~0;
  }
  rcv_len = buf[1] + 1;
  if (HAL_UART_Receive(&huart2, buf + 2, rcv_len - 2, 0xFFF) != HAL_OK) {
    EPT("Receive failed 2\n");
    EPT("%#X %#X\n", buf[0], buf[1]);
    return ~0;
  }
  rcv_crc = Cal_Check(&buf[CMD_SEQ_LENGTH], rcv_len - 3);
  if (rcv_crc != buf[rcv_len - 1]) {
    EPT("Receive check failed\n");
    return ~0;
  }
  for (int i = 0; i < rcv_len; ++i) {
    EPT("%#X\n", buf[i]);
  }
  return 0;
}

uint8_t Cal_Check(uint8_t *pdata, uint32_t len)
{
  uint32_t i;
  uint8_t chk = 0;

  for (i = 0; i < len; ++i) {
    chk ^= pdata[i];
  }
  
  return (chk + 1);
}

/**
  * @brief  Update CRC16 for input byte
  * @param  crc_in input value 
  * @param  input byte
  * @retval None
  */
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
  uint32_t crc = crc_in;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;
    if(in & 0x100)
      ++crc;
    if(crc & 0x10000)
      crc ^= 0x1021;
  }
  
  while(!(in & 0x10000));

  return crc & 0xffffu;
}

/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t* dataEnd = p_data+size;

  while(p_data < dataEnd)
    crc = UpdateCRC16(crc, *p_data++);
 
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);

  return crc&0xffffu;
}

/* USER CODE END 4 */

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
