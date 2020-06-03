/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
  CMD_SEQ_START          = 0x00,
  CMD_SEQ_LENGTH         = 0x01,
  CMD_SEQ_COMMAND_HIGH   = 0x02,
  CMD_SEQ_COMMAND_LOW    = 0x03,
  CMD_SEQ_DATA           = 0x04,
} CmdSeq;

typedef enum {
  CMD_QUERY_SN           = 0x02,
  CMD_QUERY_VERSION      = 0x03,
  CMD_QUERY_MDATE        = 0x06,
  CMD_QUERY_PN           = 0x07,
  CMD_QUERY_TEMP         = 0x17,
  CMD_UPGRADE_MODE       = 0x80,
  CMD_UPGRADE_DATA       = 0x81,
  CMD_UPGRADE_RUN        = 0x83,
  CMD_SOFTRESET          = 0x84,

  // for test
  CMD_FOR_TEST           = 0xFF,
} CmdId;

typedef enum {
  FW_FILE_MODULE_NAME    = 0x00,
  FW_FILE_FW_VERSION     = 0x20,
  FW_FILE_HW_VERSION     = 0x30,
  FW_FILE_SN             = 0x40,
  FW_FILE_PN             = 0x60,
  FW_FILE_Date           = 0x80,
  FW_FILE_FW_LENGTH      = 0xC0,
  FW_FILE_CRC            = 0xC4,
  FW_FILE_END            = 0xFF,
  FW_FILE_HEADER_LENGTH,
} FwFileHeader;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define TOGGLE_LED1() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define TOGGLE_LED2() HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)
#define EPT(format, ...)	do{\
                      sprintf((char*)debug_buf, "%s,%d: " format, __func__, __LINE__, ##__VA_ARGS__);\
                      HAL_UART_Transmit(&huart1, debug_buf, strlen((char*)debug_buf), 0xFF);\
                    } while(0)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t send_cmd(uint8_t ch, char *arg);
uint8_t Cal_Check(uint8_t *pdata, uint32_t len);
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PINOUT2_Pin GPIO_PIN_13
#define PINOUT2_GPIO_Port GPIOC
#define PINOUT_Pin GPIO_PIN_14
#define PINOUT_GPIO_Port GPIOC
#define PINOUT3_Pin GPIO_PIN_5
#define PINOUT3_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
