/**
  ******************************************************************************
  * @file    IAP_Main/Src/flash_if.c 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   This file provides all the memory related operation functions.
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

/** @addtogroup STM32L4xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "flash_if.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*  1M0 flash 1 * 1024 * 1024 */
#define FLASH_START_ADRESS    0x08000000
#define FLASH_SECTOR_COUNT     12

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  */
void FLASH_If_Init(void)
{
  /* Unlock the Program memory */
  HAL_FLASH_Unlock();

  /* Clear all FLASH flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_WRPERR);
  /* Unlock the Program memory */
  HAL_FLASH_Lock();
}

/**
  * @brief  This function does an erase of all user flash area
  * @param  start: start of user flash area
  * @retval FLASHIF_OK : user flash area successfully erased
  *         FLASHIF_ERASEKO : error occurred
  */
uint32_t FLASH_If_Erase(uint32_t sector)
{
  uint32_t SectorError = 0;
  FLASH_EraseInitTypeDef pEraseInit;
  HAL_StatusTypeDef status = HAL_OK;

  if (sector >= FLASH_SECTOR_COUNT)
    return FLASHIF_ERASEKO;

  /* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();

  pEraseInit.Banks = FLASH_BANK_1;
  pEraseInit.Sector = sector;
  pEraseInit.NbSectors = 1;
  pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  status = HAL_FLASHEx_Erase(&pEraseInit, &SectorError);
  
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  if (status != HAL_OK)
  {
    /* Error occurred while page erase */
    return FLASHIF_ERASEKO;
  }

  return FLASHIF_OK;
}

/* Public functions ---------------------------------------------------------*/
/**
  * @brief  This function writes a data buffer in flash (data are 32-bit aligned).
  * @note   After writing data buffer, the flash content is checked.
  * @param  destination: start address for target location
  * @param  p_source: pointer on buffer with data to write
  * @param  length: length of data buffer (unit is 32-bit word)
  * @retval uint32_t 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  */
uint32_t FLASH_If_Write(uint32_t destination, uint32_t *p_source, uint32_t length)
{
  uint32_t status = FLASHIF_OK;
  uint32_t i = 0;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* DataLength must be a multiple of 64 bit */
  #if 0
  for (i = 0; (i < length/2) && (destination <= (USER_FLASH_END_ADDRESS-8)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, destination, *((uint64_t *)(p_source+2*i))) == HAL_OK)      
    {
     /* Check the written value */
      if (*(uint64_t*)destination != *(uint64_t *)(p_source+2*i))
      {
        /* Flash content doesn't match SRAM content */
        status = FLASHIF_WRITINGCTRL_ERROR;
        break;
      }
      /* Increment FLASH destination address */
      destination += 8;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      status = FLASHIF_WRITING_ERROR;
      break;
    }
  }
  #else
  for (i = 0; i < length && (destination <= (USER_FLASH_END_ADDRESS-8)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, destination, (uint64_t)(*(p_source+i))) == HAL_OK)      
    {
     /* Check the written value */
      if (*(uint32_t*)destination != *(p_source+i))
      {
        /* Flash content doesn't match SRAM content */
        status = FLASHIF_WRITINGCTRL_ERROR;
        break;
      }
      /* Increment FLASH destination address */
      destination += 4;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      status = FLASHIF_WRITING_ERROR;
      break;
    }
  }
  #endif
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  return status;
}

/**
  * @brief  Returns the write protection status of application flash area.
  * @param  sector
  * @param  status: 1 means protection not active, 0 means protection active.
  * @retval If a sector in application area is write-protected returned value is a combinaison
            of the possible values : FLASHIF_PROTECTION_WRPENABLED, FLASHIF_PROTECTION_PCROPENABLED, ...
  *         If no sector is write-protected FLASHIF_PROTECTION_NONE is returned.
  */
uint32_t FLASH_If_GetWriteProtectionStatus(uint32_t sector, uint8_t *status)
{
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;
  HAL_StatusTypeDef retr;
  uint32_t wrp_sector;

  switch (sector) {
    case FLASH_SECTOR_0:
      wrp_sector = OB_WRP_SECTOR_0;
      break;
    case FLASH_SECTOR_1:
      wrp_sector = OB_WRP_SECTOR_1;
      break;
    case FLASH_SECTOR_2:
      wrp_sector = OB_WRP_SECTOR_2;
      break;
    case FLASH_SECTOR_3:
      wrp_sector = OB_WRP_SECTOR_3;
      break;
    case FLASH_SECTOR_4:
      wrp_sector = OB_WRP_SECTOR_4;
      break;
    case FLASH_SECTOR_5:
      wrp_sector = OB_WRP_SECTOR_5;
      break;
    case FLASH_SECTOR_6:
      wrp_sector = OB_WRP_SECTOR_6;
      break;
    case FLASH_SECTOR_7:
      wrp_sector = OB_WRP_SECTOR_7;
      break;
    default:
      return FLASHIF_PROTECTION_ERRROR;
  }

  if (sector >= FLASH_SECTOR_COUNT)
    return FLASHIF_ERASEKO;

  /* Unlock the Flash to enable the flash control register access *************/
  retr = HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  retr |= HAL_FLASH_OB_Unlock();

  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);
  // status equals 1 meaning protection not active
  *status = OptionsBytesStruct.WRPSector & wrp_sector;

  retr |= HAL_FLASH_OB_Lock();

  retr |= HAL_FLASH_Lock();

  return (retr == HAL_OK ? FLASHIF_OK: FLASHIF_PROTECTION_ERRROR);
}

uint32_t FLASH_If_GetOptionByte(FLASH_OBProgramInitTypeDef *OptionsBytesStruct)
{
  HAL_StatusTypeDef retr;

  /* Unlock the Flash to enable the flash control register access *************/
  retr = HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  retr |= HAL_FLASH_OB_Unlock();

  HAL_FLASHEx_OBGetConfig(OptionsBytesStruct);

  retr |= HAL_FLASH_OB_Lock();

  retr |= HAL_FLASH_Lock();

  return (retr == HAL_OK ? FLASHIF_OK: FLASHIF_PROTECTION_ERRROR);
}

/**
  * @brief  Configure the write protection status of user flash area.
  * @param  protectionstate : FLASHIF_WRP_DISABLE or FLASHIF_WRP_ENABLE the protection
  * @retval uint32_t FLASHIF_OK if change is applied.
  */
uint32_t FLASH_If_SetWriteProtection(uint32_t sector)
{
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;
  HAL_StatusTypeDef retr;
  uint32_t wrp_sector;

  switch (sector) {
    case FLASH_SECTOR_0:
      wrp_sector = OB_WRP_SECTOR_0;
      break;
    case FLASH_SECTOR_1:
      wrp_sector = OB_WRP_SECTOR_1;
      break;
    case FLASH_SECTOR_2:
      wrp_sector = OB_WRP_SECTOR_2;
      break;
    case FLASH_SECTOR_3:
      wrp_sector = OB_WRP_SECTOR_3;
      break;
    case FLASH_SECTOR_4:
      wrp_sector = OB_WRP_SECTOR_4;
      break;
    default:
      return FLASHIF_PROTECTION_ERRROR;
  }
  
  /* Unlock the Flash to enable the flash control register access *************/
  retr = HAL_FLASH_Unlock();

  /* Unlock the Options Bytes *************************************************/
  retr |= HAL_FLASH_OB_Unlock();
  
  OptionsBytesStruct.OptionType = OPTIONBYTE_WRP;
  OptionsBytesStruct.Banks = 0;
  OptionsBytesStruct.WRPSector = wrp_sector;
  OptionsBytesStruct.WRPState = OB_WRPSTATE_ENABLE;

  retr|= HAL_FLASHEx_OBProgram(&OptionsBytesStruct);

  retr |= HAL_FLASH_OB_Lock();

  retr |= HAL_FLASH_Lock();

  return (retr == HAL_OK ? FLASHIF_OK: FLASHIF_PROTECTION_ERRROR);
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
