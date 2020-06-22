#ifndef __UPGRADE_H
#define __UPGRADE_H

#include "main.h"

typedef enum {
  UPGRADE_UNUSABLE       = 0x00,
  UPGRADE_RESET          = 0x01,
  UPGRADE_FAILURE        = 0x02,
  UPGRADE_SUCCESS        = 0x03,
} UpgradeState;


#if 0
typedef struct {
  uint8_t state;
  uint8_t upgrade;
  uint16_t crc16;
  uint32_t length;
} UpgradeFlashState;
#else
typedef struct {
  uint8_t magic;
  uint8_t run;
  uint16_t crc16;
  uint32_t length;
  uint16_t factory_crc16;
  uint32_t factory_length;
} UpgradeFlashState;
#endif

typedef enum {
  START_ERROR,
  START_FROM_FACTORY,
  START_FROM_APPLICATION,
} UpgradeReturn;

#if defined(USE_SRAM_FOR_FW_IMG) || defined(USE_FLASH_FOR_FW_IMG)
uint8_t upgrade_process(void);
void reset_config_data(void);
#elif defined(RUN_WITH_SRAM)
void startup_process(void);
uint8_t boot_process(uint32_t, uint32_t, uint16_t);
void update_config_data(uint32_t, uint32_t);
#endif

#endif
