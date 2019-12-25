/*******************************************************************************
* Filename : packet.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "packet.h"
#include "stm32l0xx_hal.h"
#include "service_Mesh.h"
#include "service_Data.h"
#include "service_DA.h"
#include "log.h"
#include "timer.h"
#include "flash.h"
#include "flash_table.h"
#include "event.h"
#include "string.h"
#include "main.h"
#include "tim.h"
#include "rtc.h"
#include "CRC.h"

/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/
#define SET_COMMAND				0x01
#define GET_COMMAND				0x02
#define CTL_COMMAND				0x03
#define HB_COMMAND				0x04

/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/
static uint8_t zigbee_head[3] = {0xAB, 0xBC, 0xCD};
static uint8_t zigbee_tmp_head[3] = {0xDE, 0xDF, 0xEF};
static uint8_t zigbee_broadcast[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t sysrstFlag = 0;
static uint8_t rejoinFlag = 0;
//static uint8_t groupsetFlag = 0;
static uint8_t is_intacFlag = 0;
led_state_t led_state = {0};

#ifdef AC_BOARD
static uint8_t zeropassFlag = 0;
#endif
static uint32_t cur_time = 0;
uint32_t add_time = 0;

/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
typedef void (*cbFun)(packet_body_t*);

typedef struct
{
	cbFun 	set_callback;  		//设置和控制类回调
	cbFun 	get_callback;		//查询和心跳类回调
}FunItem_t;

/*******************************************************************************
*函数名: need_sys_rst
*说明: 系统重启标志位
*参数: 无
*返回: 重启标志位
*其他: 无
*******************************************************************************/ 
uint8_t need_sys_rst(void)
{
  return sysrstFlag;
}

/*******************************************************************************
*函数名: need_rejoin
*说明: 重新入网标志位
*参数: 无
*返回: 重新入网标志位
*其他: 无
*******************************************************************************/ 
uint8_t need_rejoin(void)
{
  return rejoinFlag;
}

/*******************************************************************************
*函数名: clear_rejoin
*说明: 清除重新入网标志位
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
void clear_rejoin(void)
{
  rejoinFlag = 0;
}

/*******************************************************************************
*函数名: set_rejoin
*说明: 置位重新入网标志位
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
void set_rejoin(void)
{
  rejoinFlag = 1;
}

///*******************************************************************************
//*函数名: is_groupset
//*说明: 设置组号
//*参数: 无
//*返回: 组号设置标志位 1 是  0否
//*其他: 无
//*******************************************************************************/ 
//uint8_t is_groupset(void)
//{
//  return groupsetFlag;
//}

/*******************************************************************************
*函数名: is_interact_cmd
*说明: 是否是同步等待模组返回的命令
*参数: 无
*返回: 同步等待标志位 1 是  0否
*其他: 无
*******************************************************************************/
uint8_t is_interact_cmd(void)
{
  return is_intacFlag;
}

///*******************************************************************************
//*函数名: clear_groupset
//*说明: 清除组号标志
//*参数: 无
//*返回: 无
//*其他: 无
//*******************************************************************************/ 
//void clear_groupset(void)
//{
//  groupsetFlag = 0;
//}

/*******************************************************************************
*函数名: clear_intac
*说明: 清除同步标志
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
void clear_intac(void)
{
  is_intacFlag = 0;
}

/*******************************************************************************
*函数名: set_intac
*说明: 置位同步标志
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
void set_intac(void)
{
  is_intacFlag = 1;
}

///*******************************************************************************
//*函数名: set_groupsetFlag
//*说明: 置位组播标志
//*参数: 无
//*返回: 无
//*其他: 无
//*******************************************************************************/ 
//void set_groupsetFlag(void)
//{
//  groupsetFlag = 1;
//}

#ifdef AC_BOARD
/*******************************************************************************
*函数名: outside_led_close
*说明: 关继电器
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void outside_led_close(void)
{
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
*函数名: outside_led_open
*说明: 开继电器
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void outside_led_open(void)
{
  zeropassFlag = 0;
  uint32_t tick;
  tick = HAL_GetTick();
  while((zeropassFlag == 0) && (HAL_GetTick() - tick < 500))
  {
    //等待过零点检测
  }
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_RESET);
}
#endif

/*******************************************************************************
*函数名: change_bright
*说明: 调光
*参数: pulse 占空比(0~100)
*返回: 无
*其他: 无
*******************************************************************************/
void change_bright(uint8_t pulse)
{
  hb_info_t hb_data = {0};
  if(pulse > 100)
  {
    return;
  }
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if(HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  get_hb_info(&hb_data);
  hb_data.pwm = pulse; 
  set_hb_info(&hb_data);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);
}

/*******************************************************************************
*函数名: set_led_on
*说明: 开灯
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
void set_led_on(void)
{
#ifdef AC_BOARD
  outside_led_open();
#else
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2);
#endif
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);
  hb_data.onoff = 1;
  set_hb_info(&hb_data);
  //若上一次是关灯，则将此次开灯的时间 记为最近开灯时间
  if(led_state.onoff == 0)
  {
    if(led_state.start == 0)
    {
      led_state.start = 1;
    }
    led_state.unix_time = Time_To_Unix();
  }
  led_state.onoff = 1;
}

/*******************************************************************************
*函数名: set_led_off
*说明: 关灯
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
void set_led_off(void)
{
    hb_info_t hb_data = {0};
    get_hb_info(&hb_data);
#ifdef AC_BOARD
    outside_led_close();
#else
    change_bright(0);
    //HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_2);
#endif
    hb_data.onoff = 0;
    set_hb_info(&hb_data);
    //若关灯  保存关灯前的总开灯时间
    led_state.onoff = 0;
    //确保第一次为开灯
    if(led_state.start == 1)
    {
      cur_time = Time_To_Unix();
      led_state.working_hour =  (cur_time - led_state.unix_time)/3600;
      add_time += led_state.working_hour;
    }
}

/*******************************************************************************
*函数名: set_ledonoff_cb
*说明: 开关灯
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void set_ledonoff_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  if(para->data[0] == 0)
  {
     set_led_off();
  }
  else
  {
     set_led_on();
  }
  para->data[ack_dlen++] = SET_SUCCESS;
  para->dlen = ack_dlen+1;
}

/*******************************************************************************
*函数名: set_bright_cb
*说明: 调光
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void set_bright_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  if(para->data[0] > 100)
  {
    para->data[ack_dlen++] = SET_ERROR;
  }
  else
  {
    change_bright(para->data[0]);
    para->data[ack_dlen++] = SET_SUCCESS;
  }
  para->dlen = ack_dlen+1;
}

/*******************************************************************************
*函数名: set_sysrstFlag
*说明: 设置系统重启标志位
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/ 
static void set_sysrstFlag(void)
{
  sysrstFlag = 1;
}

/*******************************************************************************
*函数名: set_sysrst_cb
*说明: 系统重启
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void set_sysrst_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  para->data[ack_dlen++] = SET_SUCCESS;
  para->dlen = ack_dlen+1;
  set_sysrstFlag();
}

/*******************************************************************************
*函数名: set_group_cb
*说明: 设置分组
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void set_group_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  uint8_t ret = SET_ERROR;
  uint8_t groupCount = 0;
  uint8_t opcode = 0;
  uint8_t updateFlag = 0;
  uint8_t overloadFlag = 0;
  uint8_t i,j;
  //判断格式
  if(para->dlen < 2+2*GROUP_NUM_MAX)
  {
    if(para->dlen %2 == 0)
    {
      opcode = para->data[0];
      if(opcode <= GROUP_UPDATE)
      {
        groupCount = (para->dlen - 2)/2;     
          if(zigbee_load.zigbee_group.list_len + groupCount > GROUP_NUM_MAX && opcode == GROUP_ADD)
          {
            //防止增加过多
            overloadFlag = 1;
          }
          if(opcode == GROUP_UPDATE)
          {
            opcode = GROUP_ADD;
            updateFlag = 1;
          }
          if(overloadFlag == 0)
          {
            if(opcode == GROUP_CLEAR)
            {
              memset(&groupList, 0, sizeof(groupList));
              groupList.groupCount = 1;
              groupList.group[0].opcode = GROUP_CLEAR;
            }
            else
            {
              groupList.groupCount = groupCount;
              for(i=0, j=0; i<groupCount; i++,j+=2)
              {
                groupList.group[i].opcode = opcode;
                memcpy(&(groupList.group[i].num), para->data+1+j, 2);
              }
            }
            set_intac();
            ret = zigbee_set_group_process(updateFlag);
            clear_intac();
          }
      }
    }
  }
  para->data[ack_dlen++] = ret;
  para->dlen = ack_dlen+1;  
}

/*******************************************************************************
*函数名: set_alarm_cb
*说明: 设置报警值
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void set_alarm_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  uint8_t errorFlag = 0;
  da_event_t da_event;
  
  if(para->dlen != (sizeof(da_event_t)+1))
  {
    para->data[ack_dlen++] = SET_ERROR;
    para->dlen = ack_dlen+1;
    return;
  }
  else
  {
    memcpy(&da_event, para->data, sizeof(da_event_t));
    //键值
    if(da_event.key != 1)
    {
      para->data[ack_dlen++] = SET_ERROR;
      para->dlen = ack_dlen+1;
      return;
    }
    //使能
    if(da_event.flag == 1)
    {
      //报警值需大于恢复值
      if(da_event.recover_value > da_event.alarm_value)
      {
        errorFlag = 1;
      }
    }
    //取消
    else if(da_event.flag == 0)
    {
      da_event.recover_value = 0;
      da_event.alarm_value = 0;
    }
    if(errorFlag)
    {
      para->data[ack_dlen++] = SET_ERROR;
      para->dlen = ack_dlen+1;
    }
    else
    {
      para->data[ack_dlen++] = SET_SUCCESS;
      para->dlen = ack_dlen+1;
      //掉电保存
      zigbee_load.da_event = da_event;
      EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
    }
  }
}

/*******************************************************************************
*函数名: set_rejoin_cb
*说明: 控制重新加入网络
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void set_rejoin_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  uint8_t ret = SET_SUCCESS;
  para->data[ack_dlen++] = ret;
  para->dlen = ack_dlen+1;
  set_rejoin();
}

/*******************************************************************************
*函数名: get_group_cb
*说明: 获取心跳
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/    
static void get_group_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  para->data[ack_dlen++] = SET_SUCCESS;
  for(uint8_t i=0; i<GROUP_NUM_MAX; i++)
  {
    if(zigbee_load.zigbee_group.group_inf[i].enflag == 1)
    {
      memcpy(para->data+ack_dlen, &(zigbee_load.zigbee_group.group_inf[i].num), 2);
      ack_dlen += 2;
    }
  }
  para->dlen = ack_dlen+1;
}

/*******************************************************************************
*函数名: hb_cb
*说明: 获取心跳
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void hb_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);  
  
  para->data[ack_dlen++] = SET_SUCCESS;
   
  memcpy(para->data + ack_dlen, &hb_data, sizeof(hb_info_t));
  ack_dlen = ack_dlen + sizeof(hb_info_t);
  para->dlen = ack_dlen+1;
}

/*******************************************************************************
*函数名: get_node_cb
*说明: 获取节点数据
*参数: para 数据内容
*返回: 无
*其他: 无
*******************************************************************************/        
static void get_node_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  para->data[ack_dlen++] = SET_SUCCESS;
  para->dlen = ack_dlen+1;
}

//参数回调表
const static FunItem_t cbFunTab[] = 
{
	{NULL				, NULL				},	//0x00
	{set_ledonoff_cb		, NULL			        },	//0x01
	{set_bright_cb			, NULL	                        },	//0x02
	{hb_cb			        , NULL			        },	//0x03
        {set_alarm_cb			, NULL			        },	//0x04
        {set_group_cb			, get_group_cb			},	//0x05
        {set_sysrst_cb			, NULL			        },	//0x06
        {set_rejoin_cb			, NULL				},	//0x07
        {NULL				, get_node_cb			},	//0x08
};

/*******************************************************************************
*函数名: parse_packet
*说明: 判断数据指令属于哪种类型
*参数: 无
*返回: 0 异常指令 1协议指令 2模组指令 3临时指令
*其他: 无
*******************************************************************************/
uint8_t parse_packet(packet_head_t* packet_head, packet_body_t* packet_body, uint8_t* data, uint32_t datalen)
{
  uint8_t ret = ERROR_CMD;
  
  if(memcmp(data, zigbee_head, 3) == 0)
  {
    ret = MODULE_CMD;
    return ret;
  }
  else if(memcmp(data, zigbee_tmp_head, 3) == 0)
  {
    ret = TEMP_CMD;
    return ret;
  }
  else
  {
    //判断包头包尾
    if(data[0] != 0x68 || data[datalen-1] != 0x16)
    {
      ret = ERROR_CMD;
      return ret;
    }
    //判断长度
    if(datalen > sizeof(packet_head_t)+sizeof(packet_body_t))
    {
      ret = ERROR_CMD;
      return ret;
    }
    memcpy(packet_head, data, sizeof(packet_head_t));
    memcpy(packet_body, data+sizeof(packet_head_t), datalen - sizeof(packet_head_t)); 
    //组播 判断组号
    if(packet_head->mainId & 0x40)
    {
      uint16_t packet_group = 0;
      uint8_t matchFlag = 0;
      memcpy(&packet_group, packet_head->dev_addr+6, 2);
      for(uint8_t i=0; i<5; i++)
      {
        if(zigbee_load.zigbee_group.group_inf[i].enflag == 1)
        {
          if(packet_group == zigbee_load.zigbee_group.group_inf[i].num)
          {
            matchFlag = 1;
          }
        }
      }
      
      if(memcmp(packet_head->dev_addr, zigbee_broadcast, 8) == 0)
      {
        matchFlag = 1;
      }
      
      if(matchFlag == 0)
      {
        ret = ERROR_CMD;
        return ret;
      }
    }
    //点播 判断mac地址
    else if(memcmp(packet_head->dev_addr, zigbee_mac, MAC_ADDR_LEN) != 0)
    {
      ret = ERROR_CMD;
      return ret;
    }
    //数据包长 异常
    if(packet_body->dlen + sizeof(packet_head_t) + 3 != datalen)
    {
      ret = ERROR_CMD;
      return ret;
    }
    if(data[datalen - 2] != CRC8_CCITT_Calc(data+1, datalen-3))
    {
      ret = ERROR_CMD;
      return ret;
    }
    memcpy(packet_body->data, data + sizeof(packet_head_t) + 2, packet_body->dlen);
    ret = PACKET_CMD;
  }
  return ret;
}

/*******************************************************************************
*函数名: set_handle
*说明: 设置类回调
*参数: param 命令中的数据域
*返回: 无
*其他: 无
*******************************************************************************/
void set_handle(packet_body_t* param)
{
	cbFun pFunction = NULL;
	pFunction = cbFunTab[param->subId].set_callback;
        if(pFunction != NULL)
        {
          pFunction(param);
        }
}

/*******************************************************************************
*函数名: get_handle
*说明: 查询类回调
*参数: param 命令中的数据域
*返回: 无
*其他: 无
*******************************************************************************/
void get_handle(packet_body_t* param)
{
	cbFun pFunction = NULL;
	pFunction = cbFunTab[param->subId].get_callback;
        if(pFunction != NULL)
        {
          pFunction(param);
        }
}

/*******************************************************************************
*函数名: data_handler
*说明: 根据主id 确认包是否要回复 以及选择哪种类型的回调
*参数: 无
*返回: 0需要回复 1不需要回复
*其他: 无
*******************************************************************************/
uint8_t data_handler(packet_head_t* packet_head, packet_body_t* packet_body)
{
    uint8_t cmd_state = 0;
    uint8_t cmd_type = 0;
    if(packet_head->mainId&0x01)
    {
      cmd_state = SET_COMMAND;
    }
    else if(packet_head->mainId&0x02)
    {
      cmd_state = GET_COMMAND;
    }
    else if(packet_head->mainId&0x04)
    {
      cmd_state = CTL_COMMAND;
    }
    else if(packet_head->mainId&0x08)
    {
      cmd_state = HB_COMMAND;
    }
    
    if(packet_head->mainId&0x40)
    {
      cmd_type = 1;
    }
    else
    {
      cmd_type = 0;
    }
    
    switch(cmd_state)
    {
    case SET_COMMAND:
      set_handle(packet_body);
      break;
    case GET_COMMAND:
      get_handle(packet_body);
      break;
    case CTL_COMMAND:
      set_handle(packet_body);
      break;
    case HB_COMMAND:
      get_handle(packet_body);
      break;
    default:
      break;
    }
    return cmd_type;
}

/*******************************************************************************
*函数名: pack_ack_packet
*说明: 组成回复包
*参数: 无
*返回: 包长
*其他: 无
*******************************************************************************/
uint32_t pack_ack_packet(uint8_t* pbuf, packet_head_t* packet_head, packet_body_t* packet_body)
{
  uint32_t packetlen = 0;
  uint8_t datalen = 0;
  uint8_t crc8 = 0;
  
  packetlen += sizeof(packet_head_t);
  
  memcpy(pbuf, packet_head, sizeof(packet_head_t));
  datalen = packet_body->dlen;
  pbuf[sizeof(packet_head_t)] = datalen;
  pbuf[sizeof(packet_head_t)+1] = packet_body->subId;
  memcpy(pbuf+sizeof(packet_head_t)+2, packet_body->data, datalen);  
  packetlen += datalen + 2;
  
  crc8 = CRC8_CCITT_Calc(pbuf+1, packetlen-2);
  pbuf[packetlen-1] = crc8;
  pbuf[packetlen] = 0x16;
  packetlen += 1;
  
  return packetlen;
}

/*******************************************************************************
*函数名: HandelKeyInterrupt
*说明: 中断处理
*参数: GPIO_Pin 中断脚
*返回: 无
*其他: 无
*******************************************************************************/ 
void HandelKeyInterrupt(uint16_t GPIO_Pin)
{
#ifdef AC_BOARD  
    if(GPIO_Pin == zero_cross_Pin)
    {
      zeropassFlag = 1;
    }
#endif
}

/*******************************************************************************
*函数名: HAL_GPIO_EXTI_Callback
*说明: 外部IO中断
*参数: GPIO_Pin 中断脚
*返回: 无
*其他: 无
*******************************************************************************/ 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HandelKeyInterrupt(GPIO_Pin);
    /* NOTE: This function Should not be modified, when the callback is needed,
    the HAL_GPIO_EXTI_Callback could be implemented in the user file
    */ 
}
