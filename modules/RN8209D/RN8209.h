#ifndef __RN8209_H
#define __RN8209_H

#include "stm32l0xx_hal.h"
#include  <stdbool.h>

#define CMD_W               0x80
#define CMD_R               0x00

#define CMD_REG             0xEA
#define SYSCON_REG          0x00
#define EMUCON_REG          0x01
#define HFCONST_REG         0x02
#define IARMSOS_REG         0x0E
#define IARMS_REG           0x22
#define IBRMS_REG           0x23
#define URMS_REG            0x24
#define PWRPA_REG           0x26
#define EMUSTAT_REG         0x2d

// 写入命令寄存器 0xEA 的值 
#define CMD_WRITE_ENABLE    0xE5 // 使能写操作
#define CMD_WRITE_DISABLE   0xDC // 关闭写操作
#define CMD_RESET           0xFA // 复位

typedef enum
{
	RN8209_SP,	//串口
	RN8209_SPI, //spi接口
}RN8209_Interface;

typedef enum
{
	RN8209_OK,
	RN8209_CHIP_ERR,//芯片故障
	RN8209_SP_ERR,	//串口故障
	RN8209_SPI_ERR,	//spi故障
	RN8209_WR_ERR,	//写入失败
	RN8209_RD_ERR,	//读取失败
	RN8209_REG_ERR,	//寄存器错误
}RN8209_ERR_Def;

typedef struct
{
	GPIO_TypeDef      *port;
	uint32_t           pin;
}RN8209_SP_Pin_t;

typedef struct
{
	uint8_t 	ch_b_en;				//ADC电流通道B开启或者关闭
	uint8_t 	gain_currA;				//电流通道B模拟增益
	uint8_t		gain_currB;				//电流通道A模拟增益		
	uint8_t 	gain_volt;				//电压通道模拟增益
}RN8209_Chip_Conf;

typedef struct 
{
	uint8_t 		chn;		//8209编号
	RN8209_Interface	interface;	//通信接口
	USART_TypeDef* 		usart;		//串口号
	RN8209_SP_Pin_t		sp_tx_pin;	//串口发送io
	RN8209_SP_Pin_t		sp_rx_pin;	//串口接收io
	RN8209_SP_Pin_t		sp_rst_pin;	//复位io
	uint16_t			baud;		//串口波特率
	SPI_TypeDef*		spi;		//spi号
	RN8209_Chip_Conf	chip;
}RN8209_Init_Struct;

typedef struct
{
	UART_HandleTypeDef	handle;	//uart handle
	bool 			is_ok;	//初始化结果
	uint16_t		sum_val;
}RN8209_Info_t;

typedef struct
{
	uint32_t 	u_adc_val;      //电压有效值
	uint32_t 	ia_adc_val; 	//电流A有效值
	uint32_t 	ib_adc_val;	//电流B有效值
	int32_t 	ap_adc_val;     //平均有功功率
}RN8209_Data_t;

/********************************************************以下为RN8209的校准****************************/
#define RN_VOL_N            (1)
#define RN_CUR_N            (1)
#define RN_POW_N            (1)

#define RN_VOL_CALIVALS_N   2
#define RN_CUR_CALIVALS_N   2
#define RN_POW_CALIVALS_N   2
// 校准结构
typedef struct
{
	uint32_t       RegVal;
	uint32_t       ActVal;
} RN_CaliVal_t;


#define RN_CALI_VAL_NB ((RN_VOL_N * RN_VOL_CALIVALS_N) + \
                        (RN_CUR_N * RN_CUR_CALIVALS_N) + \
                        (RN_POW_N * RN_POW_CALIVALS_N))

/* 校准值结构 */
typedef struct
{
	uint8_t		CaliValsN[3];
	RN_CaliVal_t  	RN_CaliVals[RN_CALI_VAL_NB];
}RN_CaliVals_t;

void RN_VolCali(uint8_t vol_index, uint8_t index , uint32_t act_val);
void RN_CurCali(uint8_t cur_index, uint8_t index , uint32_t act_val);
void RN_PowCali(uint8_t pow_index, uint8_t index , uint32_t act_val);
//void RN_CaliSave(void);

extern RN_CaliVals_t   RN_CaliVal;

RN8209_ERR_Def RN8209_Init(void);
RN8209_ERR_Def RN8209_Poll(void); //8209初始化

uint32_t RN_GetVol(void);
uint32_t RN_GetCur(void);
uint32_t RN_GetPow(void);
uint8_t RN_GetPF(void);
#endif