#ifndef __common_H
#define __common_H

#include "main.h"

#define POWER_UP_LED1() HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define POWER_UP_LED2() HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)
#define POWER_UP_LED3() HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define POWER_UP_LED4() HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define TOGGLE_LED1() HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin)
#define TOGGLE_LED2() HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin)
#define TOGGLE_LED3() HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)
#define TOGGLE_LED4() HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin)

#define EPT(format, ...)	do{\
                      sprintf((char*)debug_buf, "%s,%d: " format, __func__, __LINE__, ##__VA_ARGS__);\
                      HAL_UART_Transmit(&huart3, debug_buf, strlen((char*)debug_buf), 0xFF);\
                    } while(0)

uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);

#endif
