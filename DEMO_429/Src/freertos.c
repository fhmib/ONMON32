/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "common.h"
#include "command.h"
#include "usart.h"
#include "iwdg.h"
#include "tim.h"
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
/* USER CODE BEGIN Variables */
osSemaphoreId_t uartProcessSemaphore;                 // semaphore id
osSemaphoreId_t watchdogSemaphore;                    // semaphore id
osSemaphoreId_t cmdProcessSemaphore;                  // semaphore id

osMessageQueueId_t mid_DmaWait;                // message queue id

osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) LED_CONTROL_PRIORITY,
  .stack_size = 128
};

osThreadId_t cmdProcessTaskHandle;
const osThreadAttr_t cmdProcessTask_attributes = {
  .name = "cmdProcessTask",
  .priority = (osPriority_t) CMD_PROCESS_PRIORITY,
  .stack_size = 512
};

osThreadId_t uartProcessTaskHandle;
const osThreadAttr_t uartProcessTask_attributes = {
  .name = "uartProcessTask",
  .priority = (osPriority_t) UART_PROCESS_PRIORITY,
  .stack_size = 512
};

osThreadId_t uartDmaWaitTaskHandle;
const osThreadAttr_t uartDmaWaitTask_attributes = {
  .name = "uartDmaWaitTask",
  .priority = (osPriority_t) UART_DMA_WAIT_PRIORITY,
  .stack_size = 128
};

osThreadId_t watchdogTaskHandle;
const osThreadAttr_t watchdogTask_attributes = {
  .name = "watchdogTask",
  .priority = (osPriority_t) WATCHDOG_PRIORITY,
  .stack_size = 512
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void ledTask(void *argument);
void cmdProcessTask(void *argument);
void uartProcessTask(void *argument);
void uartDmaWaitTask(void *argument);
void watchdogTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  Mon_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  watchdogSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (watchdogSemaphore == NULL) {
    POWER_UP_LED2();
  }

  uartProcessSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (uartProcessSemaphore == NULL) {
    POWER_UP_LED2();
  }

  cmdProcessSemaphore = osSemaphoreNew(1U, 0U, NULL);
  if (cmdProcessSemaphore == NULL) {
    POWER_UP_LED2();
  }
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  mid_DmaWait = osMessageQueueNew(UART_QUEUE_LENGTH, sizeof(uint8_t), NULL);
  if (mid_DmaWait == NULL) {
    // Message Queue object not created, handle failure
    POWER_UP_LED2();
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  ledTaskHandle = osThreadNew(ledTask, NULL, &ledTask_attributes);
  uartProcessTaskHandle = osThreadNew(uartProcessTask, NULL, &uartProcessTask_attributes);
  uartDmaWaitTaskHandle = osThreadNew(uartDmaWaitTask, NULL, &uartDmaWaitTask_attributes);
  cmdProcessTaskHandle = osThreadNew(cmdProcessTask, NULL, &cmdProcessTask_attributes);
  watchdogTaskHandle = osThreadNew(watchdogTask, NULL, &watchdogTask_attributes);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void ledTask(void *argument)
{
  uint32_t ticks = pdMS_TO_TICKS(LED_PERIOD);
  for (;;)
  {
    osDelay(ticks);
    TOGGLE_LED1();
  }
}

void cmdProcessTask(void *argument)
{
  uint32_t ret;

  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (uart_msg.stage != UART_RECV_DATA) {
      EPT("Stage incorrect\n");
      continue;
    }

    ret = Cmd_Process();
    switch (ret) {
    case RESPOND_SUCCESS:
    case RESPOND_FAILURE:
    case RESPOND_INVALID_PARAM:
    case RESPOND_UNSUPPORT:
      break;
    default:
      // Impossible
      EPT("Unknow return value\n");
      break;
    }

    osSemaphoreRelease(cmdProcessSemaphore);
    Uart_Respond(resp_msg.status, resp_msg.cmd, resp_msg.buf, resp_msg.length);
  }
}

/* WARNNING: If you want to receive new messages while processing the message you just received, 
      You must pay attention to HAL_LOCK () in functions HAL_UART_Receive_IT() and HAL_UART_Transmit()
      between different tasks.*/
void uartProcessTask(void *argument)
{
  uint32_t msg_length;
  uint8_t chk;
  uint32_t wait_cmd_ticks = pdMS_TO_TICKS(UART_WAIT_CMD_SEMA);
  uint8_t uart_dma;
  osStatus_t status;

  CLEAR_BIT(huart1.Instance->SR, USART_SR_RXNE);
  HAL_UART_Receive_IT(&huart1, uart_msg.uart_buf, 1);

  for(;;)
  {
    osSemaphoreAcquire(uartProcessSemaphore, osWaitForever);

    if (uart_msg.stage == UART_WAIT_START) {
      if (uart_msg.uart_buf[CMD_SEQ_START] != UART_START_BYTE) {
        // Invalid data
        EPT("Invalid character: %#X\n", uart_msg.uart_buf[CMD_SEQ_START]);
        HAL_UART_Receive_IT(&huart1, uart_msg.uart_buf, 1);
      } else {
        // Received start
        uart_msg.stage = UART_WAIT_LENGTH;
        HAL_UART_Receive_IT(&huart1, &uart_msg.uart_buf[CMD_SEQ_LENGTH], 1);
      }
    } else if (uart_msg.stage == UART_WAIT_LENGTH) {
      // Received length
      msg_length = uart_msg.uart_buf[CMD_SEQ_LENGTH];
      if (msg_length > UART_MAX_LENGTH - 2 || msg_length < 3) {
        // Length invalid
        uart_msg.stage = UART_WAIT_START;
        HAL_UART_Receive_IT(&huart1, uart_msg.uart_buf, 1);
        EPT("Invalid length %#X\n", msg_length);
      }else {
        if (osMessageQueueGetCount(mid_DmaWait) >= UART_QUEUE_LENGTH - 1) {
          // No enough space in the queue, do not start DMA
          uart_msg.stage = UART_WAIT_START;
          HAL_UART_Receive_IT(&huart1, uart_msg.uart_buf, 1);
          EPT("No enough space in the queue\n");
        } else {
          // TODO: Send RECV_START message to the queue
          uart_dma = UART_DMA_RECV_START;
          if (osMessageQueuePut(mid_DmaWait, &uart_dma, 0U, 0U) == osOK) {
            // Total length = msg_length + start byte
            uart_msg.length = msg_length + 1;
            uart_msg.stage = UART_RECV_DATA;
            HAL_UART_Receive_DMA(&huart1, &uart_msg.uart_buf[CMD_SEQ_LENGTH + 1], uart_msg.length - 2);
          } else {
            EPT("Unknown error\n");
          }
        }

      }
    } else if (uart_msg.stage == UART_RECV_DATA){
      // Received message body

      // Send RECV_SUCCUESS message to the queue
      uart_dma = UART_DMA_RECV_SUCCESS;
      if (osMessageQueuePut(mid_DmaWait, &uart_dma, 0U, 0U) != osOK) {
        EPT("Unknown error\n");
      }

      // TODO: Check
      chk = Cal_Check(&uart_msg.uart_buf[CMD_SEQ_LENGTH], uart_msg.length - 2);
      if (chk ^ uart_msg.uart_buf[uart_msg.length - 1]) {
        // Command check failed
        EPT("Command check failed\n");
        Uart_Respond(RESPOND_CHECKSUM_ERR, 0, NULL, 0);
      } else {
        // Process Command, Respond
        xTaskNotifyGive(cmdProcessTaskHandle);
        status = osSemaphoreAcquire(cmdProcessSemaphore, wait_cmd_ticks);
        if (status == osErrorTimeout) {
          // TODO: Process command timeout
          EPT("Process command timeout\n");
        } else if (status != osOK) {
          EPT("Error, status=%#X\n", status);
        }
      }

      //HAL_UART_Transmit(&huart3, &uart_msg.uart_buf[CMD_SEQ_COMMAND], uart_msg.length - 2, 0xFFFF);
      uart_msg.stage = UART_WAIT_START;
      HAL_UART_Receive_IT(&huart1, uart_msg.uart_buf, 1);
    }
  }
}

void uartDmaWaitTask(void *argument)
{
  osStatus_t uart_status;
  uint32_t wait_ticks = pdMS_TO_TICKS(UART_DMA_TIMEOUT);
  uint8_t uart_dma;

  for(;;)
  {
    if (osMessageQueueGet(mid_DmaWait, &uart_dma, 0U, osWaitForever) != osOK) {
      EPT("Unknown error\n");
    }

    // Get the last one
    while ((osMessageQueueGetCount(mid_DmaWait)) != 0) {
      uart_status = osMessageQueueGet(mid_DmaWait, &uart_dma, 0U, 0U);
      if (uart_status != osOK) {
        EPT("Error, status=%#X\n", uart_status);
      }
    }

    if (uart_dma == UART_DMA_RECV_SUCCESS) {
      continue;
    }

    while (1) {
      uart_status = osMessageQueueGet(mid_DmaWait, &uart_dma, 0U, wait_ticks);
      if (uart_status == osErrorTimeout) {
        // DMA timeout
        // TODO: Stop DMA
        HAL_UART_DMAStop(&huart1);
        EPT("DMA timeout\n");
        // TODO: Respond?

        uart_msg.stage = UART_WAIT_START;
        HAL_UART_Receive_IT(&huart1, uart_msg.uart_buf, 1);
        break;
      } else if (uart_status != osOK) {
        EPT("Error, status=%#X\n", uart_status);
      }

      // Get the last one
      while ((osMessageQueueGetCount(mid_DmaWait)) != 0) {
        uart_status = osMessageQueueGet(mid_DmaWait, &uart_dma, 0U, 0U);
        if (uart_status != osOK) {
          EPT("Error, status=%#X\n", uart_status);
        }
      }

      if (uart_dma == UART_DMA_RECV_START) {
        continue;
      } else {
        break;
      }
    }
  }
}

void watchdogTask(void *argument)
{
  HAL_IWDG_Refresh(&hiwdg);
  HAL_TIM_Base_Start_IT(&htim3);
  for(;;)
  {
    osSemaphoreAcquire(watchdogSemaphore, osWaitForever);
    if (wd_enable)
      HAL_IWDG_Refresh(&hiwdg);
  }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
