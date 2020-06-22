#include "main.h"
#include "common.h"
#include "command.h"
#include "usart.h"
#include "flash_if.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "adc.h"
#include "iwdg.h"

UpgradeStruct up_state;
extern osSemaphoreId_t flashSemaphore;

char *FW_VERSION = "TESTSR05  "; // Total length must bigger than 8 bytes
char *SERIAL_NUMBER = "123456789  "; // Total length must bigger than 9 bytes
char *PART_NUMBER = "1231231231231231  "; // Total length must bigger than 16 bytes
char *MDATE = "20200202    "; // Total length must bigger than 8 bytes

CmdStruct command_list[] = {
  {CMD_QUERY_SN, Cmd_Get_Sn},
  {CMD_QUERY_VERSION, Cmd_Get_Version},
  {CMD_QUERY_MDATE, Cmd_Get_Mdate},
  {CMD_QUERY_PN, Cmd_Get_Pn},
  {CMD_QUERY_TEMP, Cmd_Get_Temperature},
  {CMD_UPGRADE_MODE, Cmd_Upgrade_Mode},
  {CMD_UPGRADE_DATA, Cmd_Upgrade_Data},
  {CMD_UPGRADE_RUN, Cmd_Upgrade_Run},
  {CMD_SOFTRESET, Cmd_Softreset},
  {CMD_FOR_TEST, Cmd_For_Test},
};

uint32_t Cmd_Process()
{
  int i;
  uint16_t cmd_id;

  cmd_id = (uart_msg.uart_buf[CMD_SEQ_COMMAND_HIGH] << 8) | uart_msg.uart_buf[CMD_SEQ_COMMAND_LOW];

  for (i = 0; i < sizeof(command_list) / sizeof(command_list[0]); ++i) {
    if (cmd_id == command_list[i].cmd_id) {
      return command_list[i].func();
    }
  }

  EPT("Unknow command id = %#X\n", cmd_id);
  FILL_RESP_MSG(RESPOND_UNSUPPORT, cmd_id, 0);
  return RESPOND_UNSUPPORT;
}

uint32_t Cmd_Get_Sn()
{
  sprintf((char*)resp_msg.buf, "123456789");
  FILL_RESP_MSG(RESPOND_SUCCESS, CMD_QUERY_SN, 9);
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);
  return RESPOND_SUCCESS;
}

uint32_t Cmd_Get_Version()
{
  memcpy(resp_msg.buf, FW_VERSION, 8);
  FILL_RESP_MSG(RESPOND_SUCCESS, CMD_QUERY_VERSION, 8);
  return RESPOND_SUCCESS;
}

uint32_t Cmd_Get_Mdate()
{
  memcpy(resp_msg.buf, MDATE, 8);
  pinout_enable = 1;
  FILL_RESP_MSG(RESPOND_SUCCESS, CMD_QUERY_MDATE, 8);
  return RESPOND_SUCCESS;
}

uint32_t Cmd_Get_Pn()
{
  memcpy(resp_msg.buf, PART_NUMBER, 16);
  pinout_enable = 0;
  FILL_RESP_MSG(RESPOND_SUCCESS, CMD_QUERY_PN, 16);
  return RESPOND_SUCCESS;
}

uint32_t Cmd_Get_Temperature(void)
{
  uint32_t vol;
  int16_t res;
  double voltage, temp;
  
  resp_msg.buf[0] = 0;
  resp_msg.buf[1] = 0;
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, CMD_WAIT_TEMP) != HAL_OK) {
    EPT("Poll for temperature sensor failed\n");
    HAL_ADC_Stop(&hadc1);
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_QUERY_TEMP, 2);
    return RESPOND_FAILURE;
  }
  vol = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  voltage = ((double)vol / (double)4096) * 3.3;
  temp = (voltage - 0.76) / 0.0025 + 25;
  res = (int16_t)(temp * 10);
  if (res > 1250 || res < -45) {
    EPT("res invalid\n");
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_QUERY_TEMP, 2);
    return RESPOND_FAILURE;
  }

  //EPT("temp = %.2lf, res = %d, res = %#X\n", temp, res, res);
  resp_msg.buf[0] = (res >> 8) & 0xFF;
  resp_msg.buf[1] = res & 0xFF;
  FILL_RESP_MSG(RESPOND_SUCCESS, CMD_QUERY_TEMP, 2);
  return RESPOND_SUCCESS;
}

uint32_t Cmd_Upgrade_Mode()
{
  uint8_t mode = uart_msg.uart_buf[CMD_SEQ_DATA];
  
  if (mode == 0x1) {
    if (run_state.mode == RUN_MODE_APPLICATION) {
      FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_MODE, 0);
      return RESPOND_FAILURE;
    } else {
      // TODO: Switch mode to application
      run_state.mode = RUN_MODE_APPLICATION;
      FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_MODE, 0);
      return RESPOND_SUCCESS;
    }
  } else if (mode == 0x2) {
#if defined(RUN_WITH_SRAM)
    if (run_state.mode == RUN_MODE_UPGRADE || upgrade_addr == RESERVE_ADDRESS) {
#else
    if (run_state.mode == RUN_MODE_UPGRADE) {
#endif
      FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_MODE, 0);
      return RESPOND_FAILURE;
    } else {
      // TODO: Switch mode to upgrading
      run_state.mode = RUN_MODE_UPGRADE;
      up_state.pre_state = UPGRADE_UNUSABLE;
      up_state.pre_seq = 0;
      FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_MODE, 0);
      return RESPOND_SUCCESS;
    }
  } else {
    // Invalid parameters
    FILL_RESP_MSG(RESPOND_INVALID_PARAM, CMD_UPGRADE_MODE, 0);
    return RESPOND_INVALID_PARAM;
  }
}

uint32_t Cmd_Upgrade_Data()
{
  uint16_t seq;
  uint8_t *p_fw_data = &uart_msg.uart_buf[CMD_SEQ_DATA + 2];
#if defined(USE_FLASH_FOR_FW_IMG)
  uint32_t status;
#endif
#if defined(RUN_WITH_SRAM)
  uint32_t to;
#endif

  if (run_state.mode != RUN_MODE_UPGRADE) {
    EPT("Cannot excute cmd beacuse of wrong mode\n");
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
    return RESPOND_FAILURE;
  }
  if (uart_msg.length != UART_UPGRADE_LENGTH) {
    EPT("Received data with wrong length = %u\n", uart_msg.length);
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
    return RESPOND_FAILURE;
  }

  seq = (uart_msg.uart_buf[CMD_SEQ_DATA] << 8) | uart_msg.uart_buf[CMD_SEQ_DATA + 1];
  if (seq == 0x1) {
    up_state.pre_seq = seq;
    // TODO: Parse file header
    if (strcmp((char*)&p_fw_data[FW_FILE_MODULE_NAME], "MON32")) {
      // failed
      up_state.pre_state = UPGRADE_FAILURE;
      FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
      return RESPOND_FAILURE;
    }

    if (1) {
      // success
      //osDelay(pdMS_TO_TICKS(10));
      up_state.pre_state = UPGRADE_SUCCESS;
      FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
      return RESPOND_SUCCESS;
    }

  } else if (seq == 0x2) {
    if ((seq == up_state.pre_seq + 1 && up_state.pre_state == UPGRADE_SUCCESS) || \
      (seq == up_state.pre_seq && up_state.pre_state == UPGRADE_FAILURE)) {
      up_state.pre_seq = seq;
      // TODO: Parse file header
      up_state.length = (p_fw_data[FW_FILE_FW_LENGTH + 0 - 0x80] << 24) | \
               (p_fw_data[FW_FILE_FW_LENGTH + 1 - 0x80] << 16) | \
               (p_fw_data[FW_FILE_FW_LENGTH + 2 - 0x80] << 8 ) | \
               (p_fw_data[FW_FILE_FW_LENGTH + 3 - 0x80] << 0 );
        
      up_state.crc16 = (p_fw_data[FW_FILE_CRC + 0 - 0x80] << 8) | \
              (p_fw_data[FW_FILE_CRC + 1 - 0x80] << 0);
      EPT("Fw size = %#X, crc = %#X\n", up_state.length, up_state.crc16);

#if defined(USE_FLASH_FOR_FW_IMG)
      // TODO: Upgrade initialization process
      FLASH_If_Init();

      wd_enable = 0;
      HAL_IWDG_Refresh(&hiwdg);
      status = FLASH_If_Erase(DOWNLOAD_SECTOR);
      HAL_IWDG_Refresh(&hiwdg);
      wd_enable = 1;

      if (status == FLASHIF_OK) {
        // success
        //osDelay(pdMS_TO_TICKS(10));
        up_state.pre_state = UPGRADE_SUCCESS;
        FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
        return RESPOND_SUCCESS;
      } else {
        // failure
        EPT("Erase flash failed\n");
        up_state.pre_state = UPGRADE_FAILURE;
        FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
        return RESPOND_FAILURE;
      }
#endif
#if defined(USE_SRAM_FOR_FW_IMG)
      if (up_state.length > FW_MAX_LENGTH) {
        EPT("Length invalid\n");
        up_state.pre_state = UPGRADE_FAILURE;
        FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
        return RESPOND_FAILURE;
      } else {
        up_state.pre_state = UPGRADE_SUCCESS;
        FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
        return RESPOND_SUCCESS;
      }
#endif
#if defined(RUN_WITH_SRAM)
      if (up_state.length > FW_MAX_LENGTH) {
        EPT("Length invalid\n");
        up_state.pre_state = UPGRADE_FAILURE;
        FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
        return RESPOND_FAILURE;
      }
      if (*(uint32_t*)upgrade_addr != 0xFFFFFFFFU && !flash_in_use) {
        // erase flash
        flash_in_use = 1;
        FLASH_If_Erase_IT(upgrade_sector);
      }
      up_state.pre_state = UPGRADE_SUCCESS;
      FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
      return RESPOND_SUCCESS;
#endif
    } else {
      EPT("Seq invalid\n");
      FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
      return RESPOND_FAILURE;
    }
  } else if (seq <= UPGRADE_MAX_SEQ && seq > 2) {
    if ((seq == up_state.pre_seq + 1 && up_state.pre_state == UPGRADE_SUCCESS) || \
      (seq == up_state.pre_seq && up_state.pre_state == UPGRADE_FAILURE)) {
      up_state.pre_seq = seq;
#if defined(USE_FLASH_FOR_FW_IMG)
      // Subsequent data
      if (FLASH_If_Write(DOWNLOAD_ADDRESS + (seq - 3) * PACKET_LENGTH, (uint32_t*)p_fw_data, PACKET_LENGTH / 4) == FLASHIF_OK) {
        up_state.pre_state = UPGRADE_SUCCESS;
        FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
        return RESPOND_SUCCESS;
      }
      else {
        /* An error occurred while writing to Flash memory */
        EPT("Write flash failed\n");
        up_state.pre_state = UPGRADE_FAILURE;
        FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
        return RESPOND_FAILURE;
      }
#endif
#if defined(USE_SRAM_FOR_FW_IMG)
      memcpy(fw_buffer + (seq - 3) * PACKET_LENGTH, p_fw_data, PACKET_LENGTH);
      up_state.pre_state = UPGRADE_SUCCESS;
      FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
      return RESPOND_SUCCESS;
#endif
#if defined(RUN_WITH_SRAM)
      if (seq == 3) {
        to = 0;
        while (flash_in_use && to++ < 15) {
          osDelay(pdMS_TO_TICKS(100));
        }
        if (to >= 15) {
          EPT("Waiting flash timeout\n");
          up_state.pre_state = UPGRADE_FAILURE;
          FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
          return RESPOND_FAILURE;
        }
      }
      if (FLASH_If_Write(upgrade_addr + (seq - 3) * PACKET_LENGTH, (uint32_t*)p_fw_data, PACKET_LENGTH / 4) == FLASHIF_OK) {
        up_state.pre_state = UPGRADE_SUCCESS;
        FILL_RESP_MSG(RESPOND_SUCCESS, CMD_UPGRADE_DATA, 0);
        return RESPOND_SUCCESS;
      } else {
        /* An error occurred while writing to Flash memory */
        EPT("Write flash failed\n");
        up_state.pre_state = UPGRADE_FAILURE;
        FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
        return RESPOND_FAILURE;
      }
#endif
    } else {
      EPT("Invalid seq = %u\n", seq);
      FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_DATA, 0);
      return RESPOND_FAILURE;
    }
  } else {
    // seq == 0
    EPT("seq = %u\n", seq);
    FILL_RESP_MSG(RESPOND_INVALID_PARAM, CMD_UPGRADE_DATA, 0);
    return RESPOND_INVALID_PARAM;
  }
}

uint32_t Cmd_Upgrade_Run()
{
  uint32_t status;

  if (run_state.mode != RUN_MODE_UPGRADE) {
    EPT("Cannot excute cmd beacuse of wrong mode\n");
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_RUN, 0);
    return RESPOND_FAILURE;
  }
  if (up_state.pre_seq < 3) {
    EPT("Need upgrade data\n");
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_RUN, 0);
    return RESPOND_FAILURE;
  }

#if defined(USE_FLASH_FOR_FW_IMG)
  uint8_t *pdata = (uint8_t*)DOWNLOAD_ADDRESS;
  uint16_t crc = Cal_CRC16(pdata, up_state.length);
#endif
#if defined(USE_SRAM_FOR_FW_IMG)
  uint8_t *pdata = fw_buffer;
  uint16_t crc = Cal_CRC16(pdata, up_state.length);
#endif
#if defined(RUN_WITH_SRAM)
  uint8_t *pdata = (uint8_t*)upgrade_addr;
  uint16_t crc = Cal_CRC16(pdata, up_state.length);
#endif
  if (crc != up_state.crc16) {
    EPT("CRC verified failed. %#X != %#X\n", crc, up_state.crc16);
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_RUN, 0);
    return RESPOND_FAILURE;
  }

  // TODO: save necessary info
#if defined(USE_FLASH_FOR_FW_IMG) || defined(USE_SRAM_FOR_FW_IMG)
  memcpy(&f_state, (void*)CONFIG_ADDRESS, sizeof(UpgradeFlashState));
  EPT("Old f_state: %u, %u, %#X, %u\n", f_state.state, f_state.upgrade, f_state.crc16, f_state.length);
  f_state.state = UPGRADE_UNUSABLE;
  f_state.crc16 = up_state.crc16;
  f_state.length = up_state.length;
  f_state.upgrade = 1;
#endif
#if defined(RUN_WITH_SRAM)
  EPT("f_state: %#X, %u, %#X, %u, %#x, %u\n", upgrade_status.magic, upgrade_status.run, upgrade_status.crc16, upgrade_status.length, upgrade_status.factory_crc16, upgrade_status.factory_length);
  upgrade_status.run = upgrade_addr == APPLICATION_1_ADDRESS ? 1 : 2;
  upgrade_status.crc16 = up_state.crc16;
  upgrade_status.length = up_state.length;
#endif
  wd_enable = 0;
  HAL_IWDG_Refresh(&hiwdg);
  status = FLASH_If_Erase(CONFIG_SECTOR);
  HAL_IWDG_Refresh(&hiwdg);
  wd_enable = 1;

  if (status != FLASHIF_OK) {
    // failure
    //osDelay(pdMS_TO_TICKS(10));
    EPT("Erase flash failed\n");
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_RUN, 0);
    return RESPOND_FAILURE;
  }

  if (FLASH_If_Write(CONFIG_ADDRESS, (uint32_t*)&upgrade_status, sizeof(UpgradeFlashState) / 4) != FLASHIF_OK) {
    EPT("Write flash failed\n");
    FILL_RESP_MSG(RESPOND_FAILURE, CMD_UPGRADE_RUN, 0);
    return RESPOND_FAILURE;
  }
  memcpy(&upgrade_status, (void*)CONFIG_ADDRESS, sizeof(UpgradeFlashState));
#if defined(USE_FLASH_FOR_FW_IMG) || defined(USE_SRAM_FOR_FW_IMG)
  EPT("New f_state: %u, %u, %#X, %u\n", f_state.state, f_state.upgrade, f_state.crc16, f_state.length);
#endif

  Uart_Respond(RESPOND_SUCCESS, CMD_UPGRADE_RUN, NULL, 0);

  // RESET
  __NVIC_SystemReset();

  return RESPOND_SUCCESS;
}

uint32_t Cmd_Softreset(void)
{
  // TODO: Save system configuration data

  Uart_Respond(RESPOND_SUCCESS, CMD_SOFTRESET, NULL, 0);

  __NVIC_SystemReset();

  return RESPOND_SUCCESS;
}


uint32_t Cmd_For_Test(void)
{
  uint8_t data = uart_msg.uart_buf[CMD_SEQ_DATA];

  if (data == 0)
    wd_enable = 0;

  FILL_RESP_MSG(RESPOND_SUCCESS, CMD_FOR_TEST, 0);

  return RESPOND_SUCCESS;
}


uint8_t Cal_Check(uint8_t *pdata, uint32_t len)
{
  uint32_t i;
  uint8_t chk = 0;

  for (i = 0; i < len; ++i) {
    chk ^= pdata[i];
  }
  
  return (chk + 1);
}

int Uart_Respond(uint8_t status, uint16_t cmd, uint8_t *pdata, uint32_t len)
{
  uint32_t cmd_len = 0;

  uart_msg.uart_buf[cmd_len++] = UART_START_BYTE;
  uart_msg.uart_buf[cmd_len++] = 1 + 2 + len + 1 + 1; // Length + command + data + status + chk
  uart_msg.uart_buf[cmd_len++] = (uint8_t)(cmd >> 8);
  uart_msg.uart_buf[cmd_len++] = (uint8_t)cmd;
  if (len) {
    memcpy(&uart_msg.uart_buf[cmd_len], pdata, len);
    cmd_len += len;
  }
  uart_msg.uart_buf[cmd_len++] = status;
  uart_msg.uart_buf[cmd_len++] = Cal_Check(&uart_msg.uart_buf[CMD_SEQ_LENGTH], 1 + 2 + len);

  if (HAL_UART_Transmit(&huart1, uart_msg.uart_buf, cmd_len, 0xFF) != HAL_OK) {
    EPT("Respond command failed\n");
      return -1;
  } else {
    if (status != 0)
      EPT("Respond command = %#X, status = %#X\n", cmd, status);
    return 0;
  }
}
