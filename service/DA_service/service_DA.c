/*******************************************************************************
* Filename : service_DA.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "stm32l0xx_hal.h"
#include "service_Data.h"
#include "service_DA.h"
#include "service_Mesh.h"
#include "string.h"
#include "event.h"
#include "tim.h"
#include "adc.h"
#include "timer.h"
#include "main.h"
#include "flash_table.h"
#include "packet.h"
#include "RN8209.h"
#include "log.h"
#include "usart.h"
/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/
#define DA_EVENT_NUM                                                   4
#define DA_DATA_NUM                                                    20
#define DA_REFRESH_PERIOD                                              1000

#define ADC_DMA_BUF_SIZE                                               3
#define ADC_STATE_I                                                    0
#define ADC_STATE_U                                                    1

#ifdef AC_BOARD  
#define I_OVERLOAD_VALUE                                               500
#else
#define I_OVERLOAD_VALUE                                               3500
#endif

/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/
static queue_t queue_DA;
static event_t event_DA;
static event_element_t element_DA[DA_EVENT_NUM];

static queue_t queue_DA_ack;
static event_t event_DA_ack;
static event_element_t element_DA_ack[DA_EVENT_NUM];

timer_t timer_DA;

static hb_info_t hb_info = {0};
static uint8_t adc_state = ADC_STATE_I;
static uint16_t adc_in_buf[ADC_DMA_BUF_SIZE] = {0};
static uint16_t adc_in_data = 0;

uint8_t get_rssi[6] = {0xDE, 0xDF, 0xEF, 0xDE, 0x00, 0x00};
/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/

static void DA_timer_callback(void);

/*******************************************************************************
*函数名: service_da_init
*说明: DA服务初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_DA_init(void)
{
    event_create(event_DA_id_get(), 
                 &queue_DA, 
                 element_DA, 
                 DA_EVENT_NUM);
    
    event_create(event_DA_ack_id_get(), 
                 &queue_DA_ack, 
                 element_DA_ack,
                 DA_EVENT_NUM);
     
    timer_create(&timer_DA, 
                 DA_REFRESH_PERIOD, 
                 TIMER_OPT_MODE_ONCE, 
                 DA_timer_callback);

    timer_start(&timer_DA);

#ifdef AC_BOARD
    memcpy(&RN_CaliVal, &(zigbee_load.RN_CaliVal), sizeof(RN_CaliVals_t));
    if(RN8209_Init() == RN8209_CHIP_ERR)
    {
      logerr("RN8209 err\r\n");
    }
#endif
    
}

/*******************************************************************************
*函数名: get_hb_info
*说明: 获取心跳信息
*参数: *hb_data 所读取地址
*返回: 无
*其他: 无
*******************************************************************************/
void get_hb_info(hb_info_t* hb_data)
{
  __disable_irq(); 
  *hb_data = hb_info;
  __enable_irq();  
}

/*******************************************************************************
*函数名: set_hb_info
*说明: 设置心跳信息
*参数: hb_data* 要设置的心跳信息
*返回: 无
*其他: 无
*******************************************************************************/
void set_hb_info(hb_info_t* hb_data)
{
  __disable_irq();
  memcpy(&hb_info, hb_data, sizeof(hb_info_t));
  __enable_irq();  
} 

/*******************************************************************************
*函数名: adc_start
*说明: ADC开始采集 采集结果保存在DMA缓存
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void adc_start(void)
{
    adc_in_data = 0;
    HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adc_in_data, 1);
}

/*******************************************************************************
*函数名: adc_stop
*说明: ADC停止采集
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void adc_stop(void)
{
    HAL_ADC_Stop_DMA(&hadc);
}

/*******************************************************************************
*函数名: ADC_I_Init
*说明: 配置电压采集ADC
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void ADC_I_Init(void)
{
  hadc.Instance->CHSELR |= (uint32_t)(ADC_CHANNEL_5 & ADC_CHANNEL_MASK);
  hadc.Instance->CHSELR &= ~((uint32_t)(ADC_CHANNEL_6 & ADC_CHANNEL_MASK));
}

/*******************************************************************************
*函数名: adc_ini
*说明: 配置ADC采集
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void adc_init(void)
{
  ADC_I_Init();
  adc_state = ADC_STATE_I;
  adc_stop();
}

/*******************************************************************************
*函数名: ADC_U_Init
*说明: 配置ADC采集 电压
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void ADC_U_Init(void)
{
  hadc.Instance->CHSELR |= (uint32_t)(ADC_CHANNEL_6 & ADC_CHANNEL_MASK);
  hadc.Instance->CHSELR &= ~((uint32_t)(ADC_CHANNEL_5 & ADC_CHANNEL_MASK));
}

/*******************************************************************************
*函数名: adc_init_u
*说明: 配置ADC采集 电压
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void adc_init_u(void)
{
  ADC_U_Init();
  adc_state = ADC_STATE_U;
  adc_stop();
}

/*******************************************************************************
*函数名: adc2ma
*说明: adc值转换为电流值
*参数: *data 数据域  dlen 数据长度
*返回: 电流值
*其他: 无
*******************************************************************************/
static int16_t adc2ma(uint8_t* data, uint32_t dlen)
{
  int16_t ret = -1;
  uint16_t i_adc = 0;
  if(dlen == 2)
  {
    memcpy(&i_adc, data, 2);
    //采样值转换为电压
    i_adc  = (i_adc*3300) >> 12;
    ret = i_adc*40/21;
  }
  return ret;
}

/*******************************************************************************
*函数名: adc2v
*说明: adc值转换为电压值
*参数: *data 数据域  dlen 数据长度
*返回: 电压值
*其他: 无
*******************************************************************************/
static int16_t adc2v(uint8_t* data, uint32_t dlen)
{
  int16_t ret = -1;
  uint16_t v_adc = 0;
  if(dlen == 2)
  {
    memcpy(&v_adc, data, 2);
    //采样值转换为电压
    v_adc  = (v_adc*3300) >> 12;
    ret = v_adc*21/1000;
  }
  return ret;
}

static uint8_t adc_ma_buf_num = 0;
/*******************************************************************************
*函数名: adc_ma_data_get
*说明: 处理采集的电流数据
*参数: *data 数据域  dlen 数据长度
*返回: 采样所得电流平均值
*其他: 无
*******************************************************************************/
int16_t adc_ma_data_get(uint8_t* data, uint32_t dlen)
{
  int16_t adc_data = 0;
  int16_t adc_value = -1;
  int32_t sum = 0;
  uint8_t i;
  
  adc_data = adc2ma(data, dlen);
  if(adc_data != -1)
  {
    adc_in_buf[adc_ma_buf_num] = adc_data;
    adc_ma_buf_num = (adc_ma_buf_num + 1)% ADC_DMA_BUF_SIZE;
    
    for(i = 0; i < ADC_DMA_BUF_SIZE; i++)
    {
      sum += adc_in_buf[i];
    }
    adc_value = sum / ADC_DMA_BUF_SIZE;
    if(adc_value > 80)
    {
      adc_value = adc_value - 80;
    }
  }
  return adc_value;
}

/*******************************************************************************
*函数名: I_value_process
*说明: 电流 数据判断
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void I_value_process(hb_info_t hb_data)
{
  //电流过大 关灯
#ifdef AC_BOARD  
  if(hb_data.i > I_OVERLOAD_VALUE)
  {
     set_led_off();
     loginf("I over load\r\n");
  }
#endif
  if(zigbee_load.da_event.flag == 1)
  {
    if(hb_data.i > zigbee_load.da_event.alarm_value)
    {
      if(hb_data.alarm == 0)
      {
        hb_data.alarm = 0x01;
        loginf("I alarm\r\n");
      }
    }
    else if(hb_data.i < zigbee_load.da_event.recover_value)
    {
      if(hb_data.alarm == 0x01)
      {
        hb_data.alarm = 0x00;
        loginf("I recover\r\n");
      }
    }
  }
  set_hb_info(&hb_data);
}

/*******************************************************************************
*函数名: DA_I_process
*说明: 处理电流采集的数据回复
*参数: *data 数据域  dlen 数据长度
*返回: 0:收到正确的回复   -1:超时或数据异常
*其他: 无
*******************************************************************************/
int32_t DA_I_process(uint8_t* data, uint32_t dlen)
{
  int32_t ret = -1;
  int16_t adc_value = 0;
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);
  
  adc_value = adc_ma_data_get(data, dlen);
  if(adc_value != -1)
  {
    hb_data.i = adc_value;
    I_value_process(hb_data);
    ret = 0;
  }
  return ret;
}

/*******************************************************************************
*函数名: DA_U_process
*说明: 处理电压采集的数据回复
*参数: * data 数据域  dlen 数据长度
*返回: 0:收到正确的回复   -1:超时或数据异常
*其他: 无
*******************************************************************************/
int32_t DA_U_process(uint8_t* data, uint32_t dlen)
{
  int32_t ret = -1;
  int16_t adc_data = 0;
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);
  
  adc_data = adc2v(data, dlen);
  if(adc_data != -1)
  {
    hb_data.u = adc_data;
    set_hb_info(&hb_data);
    ret = 0;
  }
  return ret;
}

#ifdef AC_BOARD

#else
/*******************************************************************************
*函数名: wait_DA_ack
*说明: 等待模组指令返回
*参数: 无
*返回: 0:收到正确的回复   -1:超时或数据异常
*其他: 无
*******************************************************************************/
static int32_t wait_DA_ack(void)
{
  int32_t ret = -1;
  event_element_t element; 
  ret = event_wait(&event_DA_ack, &element, 500);
  if(ret == 0)
  {
    switch(element.type)
    {
    case EVT_DA_I:
      DA_I_process(element.data, element.size);
      break;
    case EVT_DA_U:
      DA_U_process(element.data, element.size);
      break;
    }
  }
  return ret;
}    
#endif

/*******************************************************************************
*函数名: get_led_parm
*说明:  电压 电流 采集
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void get_led_parm(void)
{
#ifdef AC_BOARD  
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);
  RN8209_Poll();
  hb_data.i = (uint16_t)RN_GetCur();
  hb_data.u = (uint16_t)RN_GetVol();
  I_value_process(hb_data);
#else 
  int32_t ret = -1;
  uint8_t i = 0;
  adc_init();
  adc_start();
  //测试电流
  for(i=0; i<3; i++)
  {   
    //ADCI_DMA中断 同步等待
    ret = wait_DA_ack();
    if(ret == 0)
    {
      break;
    }
    else
    {
      logerr("DA i timeout\r\n");
    }
  }
  //测试电压
  adc_init_u();
  adc_start();  
  for(i=0; i<3; i++)
  {
    //ADCU_DMA中断 同步等待
    ret = wait_DA_ack();
    if(ret == 0)
    {
      break;
    }
    else
    {
      logerr("DA u timeout\r\n");
    }
  }
#endif
}

/*******************************************************************************
*函数名: cmd_get_rssi
*说明: 发送 信号强度采集命令
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void cmd_get_rssi(void)
{  
   //zigbee_data_send("123456", 6);  
   //zigbee_data_send(get_rssi, 6);
}

/*******************************************************************************
*函数名: get_zigbee_info
*说明: 信号强度采集
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void get_zigbee_info(void)
{
//  int32_t ret = -1;
//  for(uint8_t i=0; i<3; i++)
//  {
    //发送消息给zigbee服务 同步等待
    __disable_irq();
    cmd_get_rssi();
    __enable_irq();
//    ret = wait_data_ack(CMD_RSSI);
//    if(ret == 0)
//    {
//      break;
//    }
//    else
//    {
//      logerr("DA zigbee rssi timeout\r\n");
//    }
//  }
}

/*******************************************************************************
*函数名: get_time
*说明: 时间采集
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void get_time(void)
{
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);
  uint32_t mcur_time = 0;
  
  if(led_state.start == 1)
  {
    if(led_state.onoff == 0)
    {
      hb_data.hour = led_state.working_hour + add_time;
    }
    else if(led_state.onoff == 1)
    {
      mcur_time = Time_To_Unix();
      hb_data.hour = (mcur_time - led_state.unix_time)/3600;
    }
    set_hb_info(&hb_data);
  }
}

/*******************************************************************************
*函数名: data_collection
*说明: 数据采集
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void data_collection(void)
{
  get_led_parm();
  get_time();
  if(get_dev_state() == IN_NET)
  {   
    //set_intac();
    if(!is_interact_cmd())
    {
#warning test
      //get_zigbee_info();
    }
    //clear_intac();
  }
  timer_start(&timer_DA);
}

/*******************************************************************************
*函数名: service_DA
*说明: DA服务
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_DA(void)
{
    int32_t ret;
    event_element_t element;

    ret = event_wait(&event_DA, &element, 0);
    if(ret == 0)
    {
      switch(element.type)
      {   
        case EVT_DA_TIMEOUT:
          data_collection();
          break;
      }
    }
}

/*******************************************************************************
*函数名: DA_timer_callback
*说明: DA周期发送 电压电流采集事件回调
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void DA_timer_callback(void)
{
    event_element_t element;
  
    element.data = NULL;
    element.size = 0;
    element.type = EVT_DA_TIMEOUT;
    
    event_send(&event_DA, &element);
}

/*******************************************************************************
*函数名: event_DA_id_get
*说明: DA服务  id
*参数: 无
*返回: 
*其他: 无
*******************************************************************************/
event_t *event_DA_id_get(void)
{
    return &event_DA;
}

/*******************************************************************************
*函数名: event_DA_ack_id_get
*说明: DA服务 消息回复 id
*参数: 无
*返回: 
*其他: 无
*******************************************************************************/
event_t *event_DA_ack_id_get(void)
{
    return &event_DA_ack;
}

/*******************************************************************************
*函数名: DMA_Channel_IRQHandler
*说明: ADC DMA中断
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
    if(DMA1->ISR & DMA_ISR_TCIF1)
    {
      __disable_irq();
      DMA1->IFCR |= DMA_IFCR_CTCIF1;
      adc_stop();
      event_element_t element;     
      element.data = &adc_in_data;
      element.size = 2;
      if(adc_state == ADC_STATE_I)
      {
        element.type = EVT_DA_I;
        event_send(event_DA_ack_id_get(), &element);
      }
      else if(adc_state == ADC_STATE_U)
      {
        element.type = EVT_DA_U;
        event_send(event_DA_ack_id_get(), &element);
      }
      __enable_irq();
    }
}

