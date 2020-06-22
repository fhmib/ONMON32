#ifndef __command_H
#define __command_H

#include "main.h"

#define UART_MAX_LENGTH               256
#define PACKET_LENGTH                 128
#define UART_UPGRADE_LENGTH           (1 + 1 + 2 + 2 + 128 + 1) // start + length + command + pkt_idx + fw + chksum
#define UPGRADE_MAX_SEQ               (2 + 1024)

#define FILL_RESP_MSG(s, c, l)        do {\
                                        resp_msg.status = s;\
                                        resp_msg.cmd = c;\
                                        resp_msg.length = l;\
                                      } while(0)

// For process command 
typedef enum {
#if 1
  CMD_QUERY_SN            = 0x02,
  CMD_QUERY_VERSION       = 0x03,
  CMD_QUERY_MDATE         = 0x06,
  CMD_QUERY_PN            = 0x07,
  CMD_QUERY_TEMP          = 0x17,
  CMD_UPGRADE_MODE        = 0x80,
  CMD_UPGRADE_DATA        = 0x81,
  CMD_UPGRADE_RUN         = 0x83,
  CMD_SOFTRESET           = 0x84,
  
  // for test
  CMD_FOR_TEST            = 0xFF,
#else
  CMD_QUERY_SN = 0x31,
  CMD_QUERY_VERSION = 0x32,
  CMD_QUERY_MDATE = 0x33,
  CMD_QUERY_PN = 0x34,
  CMD_UPGRADE_MODE = 0x35,
  CMD_UPGRADE_DATA = 0x36,
  CMD_UPGRADE_RUN = 0x37,
#endif
} CmdId;

typedef enum {
  CMD_SEQ_START          = 0x00,
  CMD_SEQ_LENGTH         = 0x01,
  CMD_SEQ_COMMAND_HIGH   = 0x02,
  CMD_SEQ_COMMAND_LOW    = 0x03,
  CMD_SEQ_DATA           = 0x04,
} CmdSeq;

typedef uint32_t (*cmdFunc)(void);

typedef struct {
  uint16_t cmd_id;
  cmdFunc func;
}CmdStruct;

// For Upgrade
typedef enum {
  FW_FILE_MODULE_NAME    = 0x00,
  FW_FILE_FW_VERSION     = 0x20,
  FW_FILE_HW_VERSION     = 0x30,
  FW_FILE_SN             = 0x40,
  FW_FILE_PN             = 0x60,
  FW_FILE_Date           = 0x80,
  FW_FILE_FW_LENGTH      = 0xC0,
  FW_FILE_CRC            = 0xC4,
  FW_FILE_END            = 0xFF,
  FW_FILE_HEADER_LENGTH,
} FwFileHeader;

typedef struct {
  uint8_t pre_state;
  uint16_t pre_seq;
  uint16_t crc16;
  uint32_t length;
} UpgradeStruct;

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

// For Respond
typedef enum {
  RESPOND_SUCCESS        = 0x00,
  RESPOND_FAILURE        = 0x01,
  RESPOND_CHECKSUM_ERR   = 0x02,
  RESPOND_INVALID_PARAM  = 0x03,
  RESPOND_UNSUPPORT      = 0x04,
} RespondCode;

typedef struct {
  uint8_t status;
  uint16_t cmd;
  uint32_t length;
  uint8_t buf[UART_MAX_LENGTH];
} RespondStruct;

uint32_t Cmd_Process(void);
uint32_t Get_Cmd_Id(void);

uint32_t Cmd_Get_Sn(void);
uint32_t Cmd_Get_Version(void);
uint32_t Cmd_Get_Mdate(void);
uint32_t Cmd_Get_Pn(void);
uint32_t Cmd_Get_Temperature(void);
uint32_t Cmd_Upgrade_Mode(void);
uint32_t Cmd_Upgrade_Data(void);
uint32_t Cmd_Upgrade_Run(void);
uint32_t Cmd_Softreset(void);

uint32_t Cmd_For_Test(void);

uint8_t Cal_Check(uint8_t *pdata, uint32_t len);
int Uart_Respond(uint8_t status, uint16_t cmd, uint8_t *pdata, uint32_t len);

#endif
