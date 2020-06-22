/**
  ******************************************************************************
  * @file    IAP_Main/Inc/flash_if.h 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   This file provides all the headers of the flash_if functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_IF_H
#define __FLASH_IF_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/


/* Error code */
enum 
{
  FLASHIF_OK = 0,
  FLASHIF_ERASEKO,
  FLASHIF_WRITINGCTRL_ERROR,
  FLASHIF_WRITING_ERROR,
  FLASHIF_PROTECTION_ERRROR
};

/* protection type */  
enum{
  FLASHIF_PROTECTION_NONE         = 0,
  FLASHIF_PROTECTION_PCROPENABLED = 0x1,
  FLASHIF_PROTECTION_WRPENABLED   = 0x2,
  FLASHIF_PROTECTION_RDPENABLED   = 0x4,
};

/* protection update */
enum {
	FLASHIF_WRP_ENABLE,
	FLASHIF_WRP_DISABLE
};

/* Define the address from where user application will be loaded.
   Note: this area is reserved for the IAP code                  */
#if defined(USE_SRAM_FOR_IMAGE) || defined(USE_FLASH_FOR_IMAGE)
#define BOOTLOADER_ADDRESS        (uint32_t)0x08000000
#define CONFIG_ADDRESS            (uint32_t)0x08008000
#define TRANSITION_ADDRESS        (uint32_t)0x08010000
#define FACTORY_ADDRESS           (uint32_t)0x08020000
#define APPLICATION_ADDRESS       (uint32_t)0x08040000
#define DOWNLOAD_ADDRESS          (uint32_t)0x08060000
#define SRAM_TARGET_ADDRESS       (uint32_t)0x20020000
#define CONFIG_SECTOR             FLASH_SECTOR_2
#define FACTORY_SECTOR            FLASH_SECTOR_5
#define APPLICATION_SECTOR        FLASH_SECTOR_6
#define DOWNLOAD_SECTOR           FLASH_SECTOR_7
#elif defined(RUN_WITH_SRAM)
#define BOOTLOADER_ADDRESS        (uint32_t)0x08000000
#define CONFIG_ADDRESS            (uint32_t)0x08008000
#define TRANSITION_ADDRESS        (uint32_t)0x08010000
#define FACTORY_ADDRESS           (uint32_t)0x08020000
#define APPLICATION_1_ADDRESS     (uint32_t)0x08040000
//#define APPLICATION_1_ADDRESS     (uint32_t)0x080A0000
#define APPLICATION_2_ADDRESS     (uint32_t)0x08060000
#define RESERVE_ADDRESS           (uint32_t)0x08080000
#define SRAM_TARGET_ADDRESS       (uint32_t)0x20000000
#define CONFIG_SECTOR             FLASH_SECTOR_2
#define FACTORY_SECTOR            FLASH_SECTOR_5
#define APPLICATION_1_SECTOR      FLASH_SECTOR_6
//#define APPLICATION_1_SECTOR      FLASH_SECTOR_17
#define APPLICATION_2_SECTOR      FLASH_SECTOR_7
#define RESERVE_SECTOR            FLASH_SECTOR_8
#endif


/* Notable Flash addresses */
#define USER_FLASH_END_ADDRESS        0x08100000

/* Define the user application size */
#define BOOTLOADER_FLASH_SIZE         ((uint32_t)0x00008000)
#define USER_FLASH_SIZE               ((uint32_t)0x00020000)


/* Exported macro ------------------------------------------------------------*/
/* ABSoulute value */
#define ABS_RETURN(x,y)               ((x) < (y)) ? ((y)-(x)) : ((x)-(y))

/* Exported functions ------------------------------------------------------- */
void FLASH_If_Init(void);
uint32_t FLASH_If_Erase(uint32_t StartSector);
uint32_t FLASH_If_GetWriteProtectionStatus(uint32_t sector, uint8_t *status);
uint32_t FLASH_If_GetOptionByte(FLASH_OBProgramInitTypeDef *OptionsBytesStruct);
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length);
uint32_t FLASH_If_WriteProtectionConfig(uint32_t sector);

#endif  /* __FLASH_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
