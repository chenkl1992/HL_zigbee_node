#ifndef __RN8209_CFG_H
#define __RN8209_CFG_H

#include "RN8209.h"

RN8209_Init_Struct RN8209_Cfg[] = 
{
	{
		.chn = 1,
		.interface = RN8209_SP,
		.usart = LPUART1,
		.sp_tx_pin = {.port = GPIOB, .pin = GPIO_PIN_10},
		.sp_rx_pin = {.port = GPIOB, .pin = GPIO_PIN_11},
		.sp_rst_pin = {.port = GPIOB, .pin = GPIO_PIN_4},
		.baud = 2400,
		.chip = {.ch_b_en = 0, .gain_currA = 0, .gain_currB = 0, .gain_volt = 0},
	},
};

#endif