#include "RN8209_cfg.h"
#include "usart.h"

#define RN8209_N	        sizeof(RN8209_Cfg) / sizeof(RN8209_Init_Struct)
#define RN8209_RTY_CNT		5

#define TXRX_TIMEOUT		300u
#define RN8209_BUF_SIZE		128u
#define BLOCK_SIZE	        16u
#define BLOCK_N			RN8209_BUF_SIZE / BLOCK_SIZE
#define CACHE_LINE_LEN          16u

static RN8209_Info_t RN8209_Info[RN8209_N];
static RN8209_Data_t RN8209_Raw[RN8209_N];

static void  _get_real_val(RN8209_Data_t *RawData , int index);
static uint32_t Voltage[RN8209_N] = {0};
static uint32_t Current[RN8209_N] = {0};
static uint32_t  PowerP[RN8209_N] = {0};
//static uint8_t  PF[RN8209_N]  = {0};

RN_CaliVals_t   RN_CaliVal;

/**
*******************************************************************************
* @brief       初始化io口
* @param[in]   无
* @return      无
*******************************************************************************
*/
//static void gpio_init(RN8209_SP_Pin_t* p_pin, uint32_t mode)
//{
//	GPIO_InitTypeDef gpio;
//	HAL_GPIO_WritePin(p_pin->port, p_pin->pin, GPIO_PIN_SET);
//	gpio.Pin        = p_pin->pin;
//	gpio.Mode       = mode;
//	gpio.Speed      = GPIO_SPEED_FREQ_HIGH;
//	gpio.Pull       = GPIO_PULLUP;
//	HAL_GPIO_Init(p_pin->port, &gpio);
//}

static uint8_t rn8209_get_reg_len(uint8_t addr)
{
  switch (addr)
  {
  case 0x00 : return 2;
  case 0x01 : return 2;
  case 0x02 : return 2;
  case 0x0E : return 2;
  case 0x20 : return 2;
  case 0x22 : return 3;
  case 0x23 : return 3;
  case 0x24 : return 3;
  case 0x25 : return 2;
  case 0x26 : return 4;
  case 0x27 : return 4;
  case 0x28 : return 4;
  case 0x29 : return 3;
  case 0xEA : return 1;
  default :   return 0;
  }
}
/**
*******************************************************************************
* @brief       计算校验值
* @param[in]   无
* @return      无
*******************************************************************************
*/
static uint8_t Cal_Sum(uint8_t* data, uint8_t len)
{
  int i = 0;
  uint16_t sum = 0;
  for(i = 0; i < len; i++) sum += *(data + i);
  return (uint8_t)~sum;
}
/**
*******************************************************************************
* @brief       写数据
* @param[in]   无
* @return      无
*******************************************************************************
*/
static RN8209_ERR_Def RN8209_Write_SP(uint8_t addr, uint16_t* p_data, uint8_t chn)
{
  uint8_t wr_buf[5] = {0};
  uint8_t rx_buf[5] = {0};
  uint8_t wr_len = 0;
  uint16_t local_word;
  
  
  wr_len = rn8209_get_reg_len(addr); //获取寄存器长度
  
  wr_buf[0] = CMD_W | addr;
  
  if (wr_len == 2)
  {
    wr_buf[1] = (unsigned char)((*p_data & 0xff00) >> 8);
    wr_buf[2] = (unsigned char)(*p_data & 0x00ff);
  }
  else if (wr_len == 1)
  {
    wr_buf[1] = (unsigned char)(*p_data & 0x00ff);
  }
  else
  {
    return RN8209_REG_ERR;
  }
  
  HAL_UART_Receive(&RN8209_Info[chn].handle, rx_buf, 5, 100); //确保串口无数据
  
  wr_buf[wr_len + 1] = Cal_Sum(wr_buf, wr_len + 1);
  HAL_UART_Transmit(&RN8209_Info[chn].handle, wr_buf, 1 + wr_len + 1, TXRX_TIMEOUT);
  
  if ((addr == 0xEA) && (*p_data == 0xFA)) //复位命令无法校验
  {
    return RN8209_OK;
  }
  
  wr_buf[0] = 0x45;
  wr_buf[1] = 0x45 ^ 0xff;
  
  HAL_UART_Transmit(&RN8209_Info[chn].handle, wr_buf, 2, TXRX_TIMEOUT); //读回写入值进行校验
  
  if(HAL_UART_Receive(&RN8209_Info[chn].handle, rx_buf, 3, TXRX_TIMEOUT) != HAL_OK)
  {
    return RN8209_WR_ERR;
  }
  
  local_word = *(rx_buf + 1);
  if(wr_len == 2)
  {
    local_word += (*(uint16_t*)rx_buf) << 8;
  }
  if (*p_data == local_word)
  {
    return RN8209_OK;
  }
  else 
  {
    return RN8209_WR_ERR;
  }
}
/**
*******************************************************************************
* @brief       读数据
* @param[in]   无
* @return      无
*******************************************************************************
*/
static RN8209_ERR_Def RN8209_Read_SP(uint8_t addr, uint32_t* p_data, uint8_t chn)
{
  uint8_t wr_buf[6];
  uint8_t rx_len = 0;
  
  rx_len = rn8209_get_reg_len(addr); //获取寄存器长度
  if ((rx_len == 0) || (rx_len > 4)) //检查长度
  {
    return RN8209_REG_ERR;
  }
  
  wr_buf[0] = addr;
  HAL_UART_Transmit(&RN8209_Info[chn].handle, wr_buf, 1, TXRX_TIMEOUT);
  if(HAL_UART_Receive(&RN8209_Info[chn].handle, &wr_buf[1], rx_len + 1, TXRX_TIMEOUT) != HAL_OK)
  {
    return RN8209_RD_ERR;
  }
  if(Cal_Sum(wr_buf, rx_len + 1) != wr_buf[rx_len + 1]) //检查校验值
  {
    return RN8209_RD_ERR;
  }
  
  *p_data = 0;
  for(uint8_t i = 0; i < rx_len; i++)
  {
    *p_data += wr_buf[i + 1] << ((rx_len - 1 - i) * 8);
  }
  
  return RN8209_OK;
}
/**
*******************************************************************************
* @brief       8209 读配置校验值
* @param[in]   chn-通道号
* @return      无
*******************************************************************************
*/
static uint16_t rn8209_read_sum(uint8_t chn)
{
  uint8_t wr_buf[5];
  
  uint8_t rty = 0;
  
  wr_buf[0] = EMUSTAT_REG;
  wr_buf[1] = ~EMUSTAT_REG;
  
  while(rty < RN8209_RTY_CNT)
  {
    HAL_UART_Transmit(&RN8209_Info[chn].handle, wr_buf, 2, TXRX_TIMEOUT);
    if(HAL_UART_Receive(&RN8209_Info[chn].handle, &wr_buf[1], 4, TXRX_TIMEOUT) != HAL_OK)
    {
      rty++;
      continue;
    }
    uint16_t chk = (uint16_t)wr_buf[2] * 256 + wr_buf[3];
    return chk; //返回校验值
  }
  return 0xff;
}
/**
*******************************************************************************
* @brief       通信接口初始化
* @param[in]   无
* @return      无
*******************************************************************************
*/
static RN8209_ERR_Def RN8209_Port_Init(void)
{
  
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  __HAL_RCC_LPUART1_CLK_ENABLE(); 
  
  
  for(uint8_t i = 0; i < RN8209_N; i++) 
  {
    if(RN8209_Cfg[i].interface == RN8209_SP)
    {
      RN8209_Info[i].handle.Instance = RN8209_Cfg[i].usart;
      RN8209_Info[i].handle.Init.BaudRate = RN8209_Cfg[i].baud;
      RN8209_Info[i].handle.Init.Parity = UART_PARITY_EVEN;
      RN8209_Info[i].handle.Init.WordLength = UART_WORDLENGTH_9B;
      RN8209_Info[i].handle.Init.Mode = UART_MODE_TX_RX;	
      
      if(HAL_UART_Init(&RN8209_Info[i].handle) != HAL_OK)
      {
        return RN8209_SP_ERR;
      }
    }		
  }
  return RN8209_OK;
}
/**
*******************************************************************************
* @brief       硬件复位
* @param[in]   无
* @return      无
*******************************************************************************
*/
static void Hard_Reset(uint8_t chn)
{
  //	OS_ERR err;
  //	GPIO_InitTypeDef GPIO_InitStruct;
  //	
  //	GPIO_InitStruct.Pin = RN8209_Cfg[chn].sp_rst_pin.pin;
  //  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  //  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  //  	HAL_GPIO_Init(RN8209_Cfg[chn].sp_rst_pin.port, &GPIO_InitStruct);
  
  HAL_GPIO_WritePin(RN8209_Cfg[chn].sp_rst_pin.port, RN8209_Cfg[chn].sp_rst_pin.pin, GPIO_PIN_RESET);
  
  //	OS_Delay(30, &err);
  HAL_Delay(50);
  
  //	gpio_init(&RN8209_Cfg[chn].sp_tx_pin, GPIO_MODE_AF_PP);
  HAL_GPIO_WritePin(RN8209_Cfg[chn].sp_rst_pin.port, RN8209_Cfg[chn].sp_rst_pin.pin, GPIO_PIN_SET);
  
  //	OS_Delay(1000, &err);
  HAL_Delay(100);
}
/**
*******************************************************************************
* @brief       单路配置参数
* @param[in]   无
* @return      无
*******************************************************************************
*/
static RN8209_ERR_Def chip_init_chn(uint8_t chn)
{
  uint16_t local_word = 0;
  //OS_ERR err;
  
  Hard_Reset(chn);
  
  local_word = CMD_WRITE_ENABLE;
  if(RN8209_Write_SP(CMD_REG, &local_word, chn) != RN8209_OK) return RN8209_WR_ERR;
  
  //OS_Delay(100, &err);
  HAL_Delay(100);
  
  local_word = ((RN8209_Cfg[chn].chip.ch_b_en & 0x01) << 6) | ((RN8209_Cfg[chn].chip.gain_currB & 0x03) << 4) | ((RN8209_Cfg[chn].chip.gain_volt & 0x03) << 2)  | ((RN8209_Cfg[chn].chip.gain_currA & 0x03) << 0);
  if(RN8209_Write_SP(SYSCON_REG, &local_word, 0) != RN8209_OK) return RN8209_WR_ERR;
  
  local_word = 0xE000;
  if(RN8209_Write_SP(HFCONST_REG, &local_word, 0) != RN8209_OK) return RN8209_WR_ERR;
  
  local_word = (0 << 15) | (2 << 12) | (2 << 10) | (1 << 9) | (1 << 7) | (0 << 5) | (1 << 0);
  if(RN8209_Write_SP(EMUCON_REG, &local_word, 0) != RN8209_OK) return RN8209_WR_ERR;
  
  RN8209_Info[chn].sum_val = rn8209_read_sum(chn);
  
  return RN8209_OK;
}
/**
*******************************************************************************
* @brief       芯片参数配置
* @param[in]   无
* @return      无
*******************************************************************************
*/
static RN8209_ERR_Def RN8209_Chip_Init(uint8_t chn)
{
  for(uint8_t j = 0; j < RN8209_RTY_CNT; j++)
  {
    if(chip_init_chn(chn) == RN8209_OK)
    {
      RN8209_Info[chn].is_ok = true;
      break;
    }
  }
  return RN8209_OK;
}
/**
*******************************************************************************
* @brief       芯片轮询
* @param[in]   无
* @return      无
*******************************************************************************
*/
uint32_t rn_errtimes = 0;

RN8209_ERR_Def RN8209_Poll(void)
{
  for(uint8_t i = 0; i < RN8209_N; i++)
  {
    if(RN8209_Info[i].is_ok == true)
    {
      if(rn8209_read_sum(i) != RN8209_Info[i].sum_val)
      {
        RN8209_Info[i].is_ok = false;
        RN8209_Chip_Init(i);
        rn_errtimes++;
        continue;
      }
      RN8209_Read_SP(URMS_REG, &RN8209_Raw[i].u_adc_val, i);
      RN8209_Read_SP(IARMS_REG, &RN8209_Raw[i].ia_adc_val, i);
      if(RN8209_Cfg[i].chip.ch_b_en)
      {
        RN8209_Read_SP(IBRMS_REG, &RN8209_Raw[i].ib_adc_val, i);
      }
      RN8209_Read_SP(PWRPA_REG, (uint32_t*)&RN8209_Raw[i].ap_adc_val, i);
      
      //根据寄存器值计算实际值
      if(RN8209_Raw[i].ap_adc_val < 0)
        RN8209_Raw[i].ap_adc_val = ~RN8209_Raw[i].ap_adc_val + 1;
      _get_real_val(&RN8209_Raw[i],i);
    }
    else
    {
      RN8209_Chip_Init(i);
    }
  }
  
  return RN8209_OK;
}
/**
*******************************************************************************
* @brief       初始化
* @param[in]   无
* @return      无
*******************************************************************************
*/
RN8209_ERR_Def RN8209_Init(void)
{
  RN8209_ERR_Def err = RN8209_OK;
  RN8209_Port_Init(); //初始化通信接口
  for(uint8_t i = 0; i < RN8209_N; i++)
  {
    if(RN8209_Chip_Init(i) != RN8209_OK)	//初始化芯片参数
    {
      err = RN8209_CHIP_ERR;
    }
  }
  return err;
}

static void _get_real_val(RN8209_Data_t *RawData , int index)
{
  uint32_t base,off_act,off_reg,off_my;
  /*获取电压*/
  for(int i = 1 ; i < RN_VOL_CALIVALS_N ; i++)
  {
    if(RN_CaliVal.CaliValsN[0] < RN_VOL_CALIVALS_N)//电压没有校准完成
      break ;
    if(RN8209_Raw[index].u_adc_val < RN_CaliVal.RN_CaliVals[0].RegVal)
    {
      Voltage[index] = 0;
      break;
    }
    if(RN8209_Raw[index].u_adc_val > RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 1].RegVal)
    {
      base = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 1].ActVal;
      off_act = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 1].ActVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 2].ActVal;
      off_reg = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 1].RegVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 2].RegVal;
      off_my  = RN8209_Raw[index].u_adc_val - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N - 1].RegVal;
      Voltage[index] = base + ((uint64_t)off_act * off_my / off_reg);			
      break;
    }
    if(RN8209_Raw[index].u_adc_val <= RN_CaliVal.RN_CaliVals[i].RegVal)
    {
      base = RN_CaliVal.RN_CaliVals[i - 1].ActVal;
      off_act = RN_CaliVal.RN_CaliVals[i].ActVal - RN_CaliVal.RN_CaliVals[i-1].ActVal;
      off_reg = RN_CaliVal.RN_CaliVals[i].RegVal - RN_CaliVal.RN_CaliVals[i-1].RegVal;
      off_my  = RN8209_Raw[index].u_adc_val - RN_CaliVal.RN_CaliVals[i-1].RegVal;
      Voltage[index] =  base + ((uint64_t)off_act * off_my / off_reg);		
      break;
    }
    else
      continue;
  }
  /*获取电流*/
  for(int i = 1 ; i < RN_CUR_CALIVALS_N ; i++)
  {
    if(RN_CaliVal.CaliValsN[1] < RN_CUR_CALIVALS_N)//电流没有校准完成
      break ;
    if(RN8209_Raw[index].ia_adc_val < RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + 0].RegVal)
    {
      Current[index] = 0;
      break;
    }
    if(RN8209_Raw[index].ia_adc_val > RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 1].RegVal)
    {
      base = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 1].ActVal;
      off_act = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 1].ActVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 2].ActVal;
      off_reg = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 1].RegVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 2].RegVal;
      off_my  = RN8209_Raw[index].ia_adc_val - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N - 1].RegVal;
      Current[index] = base + ((uint64_t)off_act * off_my / off_reg);
      break;
    }
    if(RN8209_Raw[index].ia_adc_val <= RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i].RegVal)
    {
      base = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i - 1].ActVal;
      off_act = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i].ActVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i-1].ActVal;
      off_reg = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i].RegVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i-1].RegVal;
      off_my  = RN8209_Raw[index].ia_adc_val - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + i-1].RegVal;
      Current[index] = base + ((uint64_t)off_act * off_my / off_reg);		
      break;
    }
    else
      continue;
  }
  /*获取有功功率*/
  for(int i = 1 ; i < RN_POW_CALIVALS_N ; i++)
  {
    if(RN_CaliVal.CaliValsN[2] < RN_POW_CALIVALS_N)//功率没有校准完成
      break ;
    if(RN8209_Raw[index].ap_adc_val < RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + 0].RegVal)
    {
      PowerP[index] = 0;
      break;
    }
    if(RN8209_Raw[index].ap_adc_val > RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N  + RN_POW_CALIVALS_N - 1].RegVal)
    {
      base = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N  + RN_POW_CALIVALS_N - 1].ActVal;
      off_act = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + RN_POW_CALIVALS_N - 1].ActVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + RN_POW_CALIVALS_N - 2].ActVal;
      off_reg = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + RN_POW_CALIVALS_N - 1].RegVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + RN_POW_CALIVALS_N - 2].RegVal;
      off_my  = RN8209_Raw[index].ap_adc_val - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + RN_POW_CALIVALS_N - 1].RegVal;
      PowerP[index] = base + ((uint64_t)off_act * off_my / off_reg);
      break;
    }
    if(RN8209_Raw[index].ap_adc_val <= RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i].RegVal)
    {
      base = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i - 1].ActVal;
      off_act = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i].ActVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i-1].ActVal;
      off_reg = RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i].RegVal - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i-1].RegVal;
      off_my  = RN8209_Raw[index].ap_adc_val - RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + i-1].RegVal;
      PowerP[index] = base + ((uint64_t)off_act * off_my / off_reg);		
      break;
    }
    else
      continue;
  }
  //	uint32_t P = Voltage[index] * Current[index];
  //	if(P != 0)
  //		PF[index] = (uint8_t)(((float)(PowerP[index] * 100)/(float)P) * 100);
  //	else
  //		PF[index] = 0;
}

void RN_VolCali(uint8_t vol_index, uint8_t index , uint32_t act_val)
{
  if(vol_index > RN_VOL_CALIVALS_N - 1) //每组参数校准4次，从小到大校准
    return ;
  RN_CaliVal.CaliValsN[0] = vol_index + 1;
  RN_CaliVal.RN_CaliVals[vol_index].ActVal = act_val;
  RN_CaliVal.RN_CaliVals[vol_index].RegVal = RN8209_Raw[index].u_adc_val;
}
void RN_CurCali(uint8_t cur_index, uint8_t index , uint32_t act_val)
{
  if(cur_index > RN_CUR_CALIVALS_N - 1) //每组参数校准6次，从小到大校准
    return ;
  RN_CaliVal.CaliValsN[1] = cur_index + 1;
  RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + cur_index].ActVal = act_val;
  RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + cur_index].RegVal = RN8209_Raw[index].ia_adc_val;	
}

void RN_PowCali(uint8_t pow_index, uint8_t index , uint32_t act_val)
{
  if(pow_index > RN_POW_CALIVALS_N - 1) //每组参数校准4次，从小到大校准
    return ;
  RN_CaliVal.CaliValsN[2] = pow_index + 1;
  RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + pow_index].ActVal = act_val;
  RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N + RN_CUR_CALIVALS_N + pow_index].RegVal = RN8209_Raw[index].ap_adc_val;	
}

uint32_t RN_GetVol(void)
{
  uint32_t temp = 0;
  temp = Voltage[0];
  if (temp <= 100) return 0;
  return temp;
}

uint32_t RN_GetCur(void)
{
  uint32_t temp = 0;	
  temp = Current[0];	
  return temp;
}

uint32_t RN_GetPow(void)
{
  uint32_t temp = 0;
  temp = PowerP[0];	
  return temp;
}

//uint8_t RN_GetPF(void)
//{
//	uint8_t temp = 0;
//	CPU_SR_ALLOC();	
//	CPU_CRITICAL_ENTER();
//	temp = PF[0];
//	CPU_CRITICAL_EXIT();
//	return temp;
//}


