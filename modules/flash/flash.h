#ifndef __FLASH_H
#define __FLASH_H

#include "stm32l0xx_hal.h"

#define EEPROM_BASE_ADDR    (0x08080000)
#define EEPROM_END_ADDR     (0x080807FF)

void EEPROM_Init(void);
void EEPROM_WriteOneByte(uint32_t add, uint8_t data);
uint8_t EEPROM_ReadOneByte(uint32_t add);
void EEPROM_WritePage(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte);
void EEPROM_WriteBytes(uint32_t add, uint8_t * p_wbuf, uint32_t nbyte);
void EEPROM_ReadBytes(uint32_t add, uint8_t * p_rbuf, uint32_t nbyte);

#endif
