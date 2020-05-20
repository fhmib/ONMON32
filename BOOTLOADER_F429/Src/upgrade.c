#include "upgrade.h"
#include "main.h"
#include "common.h"
#include "flash_if.h"
#include <stdio.h>
#include <string.h>

uint8_t upgrade_process()
{
  UpgradeFlashState f_state;
  uint8_t *pdata;
  uint16_t crc;
  uint8_t buf[256];

  memcpy(&f_state, (void*)CONFIG_ADDRESS, sizeof(UpgradeFlashState));
  if (f_state.state == 0xFF) {
    sprintf((char*)buf, "state is uninitialized, reset configuration data\n");
    Serial_PutString(buf);
    reset_config_data();
    return START_FROM_APPLICATION;
  }
  sprintf((char*)buf, "f_state: %u, %u, %#X, %u\n", f_state.state, f_state.upgrade, f_state.crc16, f_state.length);
  Serial_PutString(buf);
  
  if (f_state.upgrade == 1) {
    
    FLASH_If_Erase(APPLICATION_SECTOR);
    if (FLASH_If_Write(APPLICATION_ADDRESS, (uint32_t*)DOWNLOAD_ADDRESS, f_state.length / 4) != FLASHIF_OK) {
      Serial_PutString((uint8_t*)"Write to Application flash failed\n");
      f_state.state = UPGRADE_FAILURE;
      f_state.upgrade = 0;
      f_state.crc16 = 0;
      f_state.length = 0;
      FLASH_If_Erase(CONFIG_SECTOR);
      FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
      return START_FROM_FACTORY;
    }

    pdata = (uint8_t*)APPLICATION_ADDRESS;
    crc = Cal_CRC16(pdata, f_state.length);
    if (crc != f_state.crc16) {
      sprintf((char*)buf, "CRC verified failed. %#X != %#X\n", crc, f_state.crc16);
      Serial_PutString(buf);
      f_state.state = UPGRADE_FAILURE;
      f_state.upgrade = 0;
      f_state.crc16 = 0;
      f_state.length = 0;
      FLASH_If_Erase(CONFIG_SECTOR);
      FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
      return START_FROM_FACTORY;
    }

    f_state.state = UPGRADE_SUCCESS;
    f_state.upgrade = 0;
    f_state.crc16 = 0;
    f_state.length = 0;
    FLASH_If_Erase(CONFIG_SECTOR);
    FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
    sprintf((char*)buf, "Upgrade success, state change to %u\n", f_state.state);
    Serial_PutString(buf);
    return START_FROM_APPLICATION;

  } else if (f_state.upgrade == 0) {
    if (f_state.state == UPGRADE_SUCCESS || f_state.state == UPGRADE_RESET) {
      return START_FROM_APPLICATION;
    } else {
      return START_FROM_FACTORY;
    }
  } else {
    sprintf((char*)buf, "unexpected error, upgrade recognized\n");
    Serial_PutString(buf);
    reset_config_data();
    return START_FROM_APPLICATION;
  }
}
void reset_config_data(void)
{
  UpgradeFlashState f_state;

  f_state.state = UPGRADE_RESET;
  f_state.upgrade = 0;
  f_state.crc16 = 0;
  f_state.length = 0;
  FLASH_If_Erase(CONFIG_SECTOR);
  FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
  
  return;
}
