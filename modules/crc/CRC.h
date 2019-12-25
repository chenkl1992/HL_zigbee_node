#ifndef __CRC_H__
#define __CRC_H__

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

//uint8_t CRC8_PLC_Calc(const void* buf, uint32_t len);
//uint8_t CRC8_PLC_Calc2(const void* buf, uint32_t len, uint8_t init_val);

uint8_t CRC8_CCITT_Calc(const void* buf, uint32_t len);
//uint8_t CRC8_CCITT_Calc2(const void* buf, uint32_t len, uint8_t init_val);

//uint16_t CRC16_IBM_Calc(const void* buf, uint32_t len);
//uint16_t CRC16_IBM_Calc2(const void* buf, uint32_t len, uint16_t init_val);
//uint16_t GetModbusCrc16(uint16_t crc_seed, const void *ptr, int len);

unsigned short crc16_modbus(unsigned char *arr_buff, int len);

uint32_t CRC32_IEEE_Calc(const void* buf, uint32_t len);
uint32_t CRC32_IEEE_Calc2(const void* buf, uint32_t len, uint32_t init_val);


#endif /* __CRC_H__ */
