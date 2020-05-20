#ifndef __UPGRADE_H
#define __UPGRADE_H

#include "main.h"

typedef enum {
  UPGRADE_UNUSABLE       = 0x00,
  UPGRADE_RESET          = 0x01,
  UPGRADE_FAILURE        = 0x02,
  UPGRADE_SUCCESS        = 0x03,
} UpgradeState;

typedef struct {
  uint8_t state;
  uint8_t upgrade;
  uint16_t crc16;
  uint32_t length;
} UpgradeFlashState;

typedef enum {
  START_ERROR,
  START_FROM_FACTORY,
  START_FROM_APPLICATION,
} UpgradeReturn;

uint8_t upgrade_process(void);
void reset_config_data(void);

#endif
