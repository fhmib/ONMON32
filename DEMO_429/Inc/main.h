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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "command.h"

#undef USE_FLASH_FOR_FW_IMG
#undef USE_SRAM_FOR_FW_IMG
#define RUN_WITH_SRAM

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  uint8_t stage;
  uint16_t length;
  uint8_t uart_buf[UART_MAX_LENGTH];
} UartStu;

typedef enum {
  UART_WAIT_START,
  UART_WAIT_LENGTH,
  UART_RECV_DATA,
} UartStage;

typedef enum {
  UART_DMA_RECV_START,
  UART_DMA_RECV_SUCCESS,
} UartDmaState;


typedef struct {
  uint8_t mode;
} RunState;

typedef enum {
  RUN_MODE_APPLICATION,
  RUN_MODE_UPGRADE,
} RunMode;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t debug_buf[];
extern uint8_t wd_enable;
extern uint8_t pinout_enable;
extern uint8_t reset_flag;
extern RespondStruct resp_msg;
extern UartStu uart_msg;
extern RunState run_state;
#if defined(USE_SRAM_FOR_FW_IMG)
extern uint8_t fw_buffer[];
#endif
#if defined(RUN_WITH_SRAM)
extern uint8_t flash_in_use;
extern uint32_t upgrade_addr;
extern uint32_t upgrade_sector;
extern UpgradeFlashState upgrade_status;
#endif
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// Task priority
#define WATCHDOG_PRIORITY         osPriorityISR
#define UART_PROCESS_PRIORITY     osPriorityRealtime6
#define UART_DMA_WAIT_PRIORITY    osPriorityRealtime5
#define LED_CONTROL_PRIORITY      osPriorityLow
#define CMD_PROCESS_PRIORITY      osPriorityHigh

#define FW_MAX_LENGTH             0x10000
#define LED_PERIOD                250    //ms

// Uart control macro
#define UART_START_BYTE           0x55
#define UART_QUEUE_LENGTH         8
#define UART_WAIT_CMD_SEMA        2000   //ms
#define UART_DMA_TIMEOUT          5000   //ms
#define CMD_WAIT_TEMP             50   //ms

// Reset Flags
#define BOR_RESET_BIT             (1UL << 0)
#define PIN_RESET_BIT             (1UL << 1)
#define POR_RESET_BIT             (1UL << 2)
#define SFT_RESET_BIT             (1UL << 3)
#define IWDG_RESET_BIT            (1UL << 4)
#define WWDG_RESET_BIT            (1UL << 5)
#define LPWR_RESET_BIT            (1UL << 6)
#define IS_RESETFLAG_SET(bit)     (reset_flag & bit)
#define SET_RESETFLAG(bit)        (reset_flag |= bit)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Mon_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOF
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_9
#define LED4_GPIO_Port GPIOF
#define SPI1_CS0_Pin GPIO_PIN_4
#define SPI1_CS0_GPIO_Port GPIOA
#define PINOUT_Pin GPIO_PIN_11
#define PINOUT_GPIO_Port GPIOH
#define EEPROM_WP_Pin GPIO_PIN_5
#define EEPROM_WP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
