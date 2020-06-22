#include "upgrade.h"
#include "main.h"
#include "common.h"
#include "menu.h"
#include "flash_if.h"
#include <stdio.h>
#include <string.h>

#if defined(USE_SRAM_FOR_FW_IMG) || defined(USE_FLASH_FOR_FW_IMG)
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
#ifdef USE_SRAM_FOR_FW_IMG
    if (!IS_RESETFLAG_SET(SFT_RESET_BIT)) {
      sprintf((char*)buf, "Error occured before upgrade success\n");
      Serial_PutString(buf);
      f_state.state = UPGRADE_FAILURE;
      f_state.upgrade = 0;
      f_state.crc16 = 0;
      f_state.length = 0;
      FLASH_If_Erase(CONFIG_SECTOR);
      FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
      return START_FROM_FACTORY;
    } else {
      pdata = fw_buffer;
      crc = Cal_CRC16(pdata, f_state.length);
      if (crc != f_state.crc16) {
        sprintf((char*)buf, "CRC verified failed before copy. %#X != %#X\n", crc, f_state.crc16);
        Serial_PutString(buf);
        f_state.state = UPGRADE_FAILURE;
        f_state.upgrade = 0;
        f_state.crc16 = 0;
        f_state.length = 0;
        FLASH_If_Erase(CONFIG_SECTOR);
        FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
        return START_FROM_FACTORY;
      }
    }
#endif
    FLASH_If_Erase(APPLICATION_SECTOR);
#ifdef USE_SRAM_FOR_FW_IMG
    if (FLASH_If_Write(APPLICATION_ADDRESS, (uint32_t*)fw_buffer, f_state.length / 4) != FLASHIF_OK) {
#else
    if (FLASH_If_Write(APPLICATION_ADDRESS, (uint32_t*)DOWNLOAD_ADDRESS, f_state.length / 4) != FLASHIF_OK) {
#endif
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
#elif defined(RUN_WITH_SRAM)
void startup_process()
{
  UpgradeFlashState f_state;
  uint8_t buf[256];

  memcpy(&f_state, (void*)CONFIG_ADDRESS, sizeof(UpgradeFlashState));
  if (f_state.magic != 0xAA) {
    sprintf((char*)buf, "state is uninitialized, boot from factory\n");
    Serial_PutString(buf);
    update_config_data(FACTORY_ADDRESS, 0x10000);
    memcpy(&f_state, (void*)CONFIG_ADDRESS, sizeof(UpgradeFlashState));
  }

  sprintf((char*)buf, "f_state: %#X, %u, %#X, %u, %#x, %u\n", f_state.magic, f_state.run, f_state.crc16, f_state.length, f_state.factory_crc16, f_state.factory_length);
  Serial_PutString(buf);
  if (f_state.run == 0) {
    sprintf((char*)buf, "Boot from factory...\n");
    Serial_PutString(buf);
    if (boot_process(FACTORY_ADDRESS, f_state.factory_length, f_state.factory_crc16)) {
      sprintf((char*)buf, "Fatal error!!!\n");
      Serial_PutString(buf);
    }
  } else if (f_state.run == 1) {
    sprintf((char*)buf, "Boot from application 1...\n");
    Serial_PutString(buf);
    if (boot_process(APPLICATION_1_ADDRESS, f_state.length, f_state.crc16)) {
      sprintf((char*)buf, "Boot application failed\n");
      Serial_PutString(buf);
      if (boot_process(FACTORY_ADDRESS, f_state.factory_length, f_state.factory_crc16)) {
        sprintf((char*)buf, "Fatal error!!!\n");
        Serial_PutString(buf);
      }
    }
  } else if (f_state.run == 2) {
    sprintf((char*)buf, "Boot from application 2...\n");
    Serial_PutString(buf);
    if (boot_process(APPLICATION_2_ADDRESS, f_state.length, f_state.crc16)) {
      sprintf((char*)buf, "Boot application failed\n");
      Serial_PutString(buf);
      if (boot_process(FACTORY_ADDRESS, f_state.factory_length, f_state.factory_crc16)) {
        sprintf((char*)buf, "Fatal error!!!\n");
        Serial_PutString(buf);
      }
    }
  }
  
  return;
}

uint8_t boot_process(uint32_t addr, uint32_t length, uint16_t crc16)
{
  memcpy((void*)SRAM_TARGET_ADDRESS, (void*)addr, length);
  
  if (crc16 != Cal_CRC16((uint8_t*)addr, length)) {
    return 1;
  } else {
    JumpToAddr(SRAM_TARGET_ADDRESS);
  }

  return 0;
}

void update_config_data(uint32_t addr, uint32_t size)
{
  UpgradeFlashState f_state;
  
  if (addr == FACTORY_ADDRESS) {
    f_state.magic = 0xAA;
    f_state.run = 0;
    f_state.length = 0;
    f_state.crc16 = 0;
    f_state.factory_crc16 = Cal_CRC16((uint8_t*)addr, size);
    f_state.factory_length = size;
  } else {
    memcpy(&f_state, (void*)CONFIG_ADDRESS, sizeof(UpgradeFlashState));
    f_state.run = addr == APPLICATION_1_ADDRESS ? 1 : 2;
    f_state.length = size;
    f_state.crc16 = Cal_CRC16((uint8_t*)addr, size);
  }

  FLASH_If_Erase(CONFIG_SECTOR);
  FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&f_state, sizeof(UpgradeFlashState) / 4);
}
#endif
