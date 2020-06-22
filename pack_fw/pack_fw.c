#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>

//typedef unsigned char uint8_t;
//typedef unsigned short uint16_t;
//typedef unsigned int uint32_t;

#define VER_LEN 8

uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte);
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size);

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


int main(int argc, char *argv[])
{
	FILE *fp = NULL;
	FILE *fp_old = NULL;
	int ret, len = 0, i;
	struct stat file_stat;
	uint32_t file_length, length;
	uint8_t buf[256];
	uint16_t crc = 0;
	uint32_t crc32 = 0;
	char d_name[16];
	char m_name[32];

	if (argc < 2) {
		printf("Need file\n");
		return -1;
	}

	fp_old = fopen(argv[1], "rb");
	if (fp_old == NULL) {
		printf("open %s failed\n", argv[1]);
	}

	printf("Please input version:");
	scanf("%s", buf);
	sprintf(d_name, "fw_%s", buf);
	printf("Please input Module Name:");
	scanf("%s", m_name);

	fp = fopen(d_name, "wb");

	ret = stat(argv[1], &file_stat); 
	if (ret) {
		printf("stat()\n");
		return -1;
	}
	file_length = file_stat.st_size;


	while ((length = fread(buf, 1, 256, fp_old)) != 0) {
		for (i = 0; i < length; ++i) {
			crc32 = UpdateCRC16(crc32, buf[i]);
		}
	}
	crc32 = UpdateCRC16(crc32, 0);
	crc32 = UpdateCRC16(crc32, 0);

	crc = crc32 & 0xffffu;

	printf("fw size is %u, crc is %#X\n", file_length, crc);

	fseek(fp_old, 0, SEEK_SET);

	memset(buf, 0, sizeof(buf));
	sprintf(&buf[FW_FILE_MODULE_NAME], "%s", m_name);
	buf[FW_FILE_FW_LENGTH] = (uint8_t)(file_length >> 24);
	buf[FW_FILE_FW_LENGTH + 1] = (uint8_t)(file_length >> 16);
	buf[FW_FILE_FW_LENGTH + 2] = (uint8_t)(file_length >> 8);
	buf[FW_FILE_FW_LENGTH + 3] = (uint8_t)file_length;
	buf[FW_FILE_CRC] = (uint8_t)(crc >> 8);
	buf[FW_FILE_CRC + 1] = (uint8_t)crc;

	length = fwrite(buf, 1, 256, fp);
	printf("Write %u bytes to %s\n", length, d_name);

	while ((length = fread(buf, 1, 256, fp_old)) != 0) {
		printf("Write %u bytes to %s\n", length, argv[1]);
		fwrite(buf, 1, length, fp);
	}

	fclose(fp);
	fclose(fp_old);
	
	system("pause");
	return 0;
}

/**
 ** @brief  Update CRC16 for input byte
 ** @param  crc_in input value 
 ** @param  input byte
 ** @retval None
 **/
uint16_t UpdateCRC16(uint16_t crc_in, uint8_t byte)
{
	uint32_t crc = crc_in;
	uint32_t in = byte | 0x100;

	do
	{
		crc <<= 1;
		in <<= 1;
		if(in & 0x100)
			++crc;
		if(crc & 0x10000)
			crc ^= 0x1021;
	}

	while(!(in & 0x10000));

	return crc & 0xffffu;
}

/**
 ** @brief  Cal CRC16 for YModem Packet
 ** @param  data
 ** @param  length
 ** @retval None
 **/
#if 0
uint16_t Cal_CRC16(const uint8_t* p_data, uint32_t size)
{
	uint32_t crc = 0;
	const uint8_t* dataEnd = p_data+size;

	while(p_data < dataEnd)
		crc = UpdateCRC16(crc, *p_data++);

	crc = UpdateCRC16(crc, 0);
	crc = UpdateCRC16(crc, 0);

	return crc&0xffffu;
}
#endif
