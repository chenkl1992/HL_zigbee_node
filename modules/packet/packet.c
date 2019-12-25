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
	cbFun 	set_callback;  		//���úͿ�����ص�
	cbFun 	get_callback;		//��ѯ��������ص�
}FunItem_t;

/*******************************************************************************
*������: need_sys_rst
*˵��: ϵͳ������־λ
*����: ��
*����: ������־λ
*����: ��
*******************************************************************************/ 
uint8_t need_sys_rst(void)
{
  return sysrstFlag;
}

/*******************************************************************************
*������: need_rejoin
*˵��: ����������־λ
*����: ��
*����: ����������־λ
*����: ��
*******************************************************************************/ 
uint8_t need_rejoin(void)
{
  return rejoinFlag;
}

/*******************************************************************************
*������: clear_rejoin
*˵��: �������������־λ
*����: ��
*����: ��
*����: ��
*******************************************************************************/ 
void clear_rejoin(void)
{
  rejoinFlag = 0;
}

/*******************************************************************************
*������: set_rejoin
*˵��: ��λ����������־λ
*����: ��
*����: ��
*����: ��
*******************************************************************************/ 
void set_rejoin(void)
{
  rejoinFlag = 1;
}

///*******************************************************************************
//*������: is_groupset
//*˵��: �������
//*����: ��
//*����: ������ñ�־λ 1 ��  0��
//*����: ��
//*******************************************************************************/ 
//uint8_t is_groupset(void)
//{
//  return groupsetFlag;
//}

/*******************************************************************************
*������: is_interact_cmd
*˵��: �Ƿ���ͬ���ȴ�ģ�鷵�ص�����
*����: ��
*����: ͬ���ȴ���־λ 1 ��  0��
*����: ��
*******************************************************************************/
uint8_t is_interact_cmd(void)
{
  return is_intacFlag;
}

///*******************************************************************************
//*������: clear_groupset
//*˵��: �����ű�־
//*����: ��
//*����: ��
//*����: ��
//*******************************************************************************/ 
//void clear_groupset(void)
//{
//  groupsetFlag = 0;
//}

/*******************************************************************************
*������: clear_intac
*˵��: ���ͬ����־
*����: ��
*����: ��
*����: ��
*******************************************************************************/ 
void clear_intac(void)
{
  is_intacFlag = 0;
}

/*******************************************************************************
*������: set_intac
*˵��: ��λͬ����־
*����: ��
*����: ��
*����: ��
*******************************************************************************/ 
void set_intac(void)
{
  is_intacFlag = 1;
}

///*******************************************************************************
//*������: set_groupsetFlag
//*˵��: ��λ�鲥��־
//*����: ��
//*����: ��
//*����: ��
//*******************************************************************************/ 
//void set_groupsetFlag(void)
//{
//  groupsetFlag = 1;
//}

#ifdef AC_BOARD
/*******************************************************************************
*������: outside_led_close
*˵��: �ؼ̵���
*����: ��
*����: ��
*����: ��
*******************************************************************************/
void outside_led_close(void)
{
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
*������: outside_led_open
*˵��: ���̵���
*����: ��
*����: ��
*����: ��
*******************************************************************************/
static void outside_led_open(void)
{
  zeropassFlag = 0;
  uint32_t tick;
  tick = HAL_GetTick();
  while((zeropassFlag == 0) && (HAL_GetTick() - tick < 500))
  {
    //�ȴ��������
  }
  HAL_GPIO_WritePin(RELAY_EN_GPIO_Port, RELAY_EN_Pin, GPIO_PIN_RESET);
}
#endif

/*******************************************************************************
*������: change_bright
*˵��: ����
*����: pulse ռ�ձ�(0~100)
*����: ��
*����: ��
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
*������: set_led_on
*˵��: ����
*����: ��
*����: ��
*����: ��
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
  //����һ���ǹصƣ��򽫴˴ο��Ƶ�ʱ�� ��Ϊ�������ʱ��
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
*������: set_led_off
*˵��: �ص�
*����: ��
*����: ��
*����: ��
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
    //���ص�  ����ص�ǰ���ܿ���ʱ��
    led_state.onoff = 0;
    //ȷ����һ��Ϊ����
    if(led_state.start == 1)
    {
      cur_time = Time_To_Unix();
      led_state.working_hour =  (cur_time - led_state.unix_time)/3600;
      add_time += led_state.working_hour;
    }
}

/*******************************************************************************
*������: set_ledonoff_cb
*˵��: ���ص�
*����: para ��������
*����: ��
*����: ��
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
*������: set_bright_cb
*˵��: ����
*����: para ��������
*����: ��
*����: ��
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
*������: set_sysrstFlag
*˵��: ����ϵͳ������־λ
*����: ��
*����: ��
*����: ��
*******************************************************************************/ 
static void set_sysrstFlag(void)
{
  sysrstFlag = 1;
}

/*******************************************************************************
*������: set_sysrst_cb
*˵��: ϵͳ����
*����: para ��������
*����: ��
*����: ��
*******************************************************************************/        
static void set_sysrst_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  para->data[ack_dlen++] = SET_SUCCESS;
  para->dlen = ack_dlen+1;
  set_sysrstFlag();
}

/*******************************************************************************
*������: set_group_cb
*˵��: ���÷���
*����: para ��������
*����: ��
*����: ��
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
  //�жϸ�ʽ
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
            //��ֹ���ӹ���
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
*������: set_alarm_cb
*˵��: ���ñ���ֵ
*����: para ��������
*����: ��
*����: ��
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
    //��ֵ
    if(da_event.key != 1)
    {
      para->data[ack_dlen++] = SET_ERROR;
      para->dlen = ack_dlen+1;
      return;
    }
    //ʹ��
    if(da_event.flag == 1)
    {
      //����ֵ����ڻָ�ֵ
      if(da_event.recover_value > da_event.alarm_value)
      {
        errorFlag = 1;
      }
    }
    //ȡ��
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
      //���籣��
      zigbee_load.da_event = da_event;
      EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
    }
  }
}

/*******************************************************************************
*������: set_rejoin_cb
*˵��: �������¼�������
*����: para ��������
*����: ��
*����: ��
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
*������: get_group_cb
*˵��: ��ȡ����
*����: para ��������
*����: ��
*����: ��
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
*������: hb_cb
*˵��: ��ȡ����
*����: para ��������
*����: ��
*����: ��
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
*������: get_node_cb
*˵��: ��ȡ�ڵ�����
*����: para ��������
*����: ��
*����: ��
*******************************************************************************/        
static void get_node_cb(packet_body_t* para)
{
  uint8_t ack_dlen = 0;
  para->data[ack_dlen++] = SET_SUCCESS;
  para->dlen = ack_dlen+1;
}

//�����ص���
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
*������: parse_packet
*˵��: �ж�����ָ��������������
*����: ��
*����: 0 �쳣ָ�� 1Э��ָ�� 2ģ��ָ�� 3��ʱָ��
*����: ��
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
    //�жϰ�ͷ��β
    if(data[0] != 0x68 || data[datalen-1] != 0x16)
    {
      ret = ERROR_CMD;
      return ret;
    }
    //�жϳ���
    if(datalen > sizeof(packet_head_t)+sizeof(packet_body_t))
    {
      ret = ERROR_CMD;
      return ret;
    }
    memcpy(packet_head, data, sizeof(packet_head_t));
    memcpy(packet_body, data+sizeof(packet_head_t), datalen - sizeof(packet_head_t)); 
    //�鲥 �ж����
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
    //�㲥 �ж�mac��ַ
    else if(memcmp(packet_head->dev_addr, zigbee_mac, MAC_ADDR_LEN) != 0)
    {
      ret = ERROR_CMD;
      return ret;
    }
    //���ݰ��� �쳣
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
*������: set_handle
*˵��: ������ص�
*����: param �����е�������
*����: ��
*����: ��
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
*������: get_handle
*˵��: ��ѯ��ص�
*����: param �����е�������
*����: ��
*����: ��
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
*������: data_handler
*˵��: ������id ȷ�ϰ��Ƿ�Ҫ�ظ� �Լ�ѡ���������͵Ļص�
*����: ��
*����: 0��Ҫ�ظ� 1����Ҫ�ظ�
*����: ��
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
*������: pack_ack_packet
*˵��: ��ɻظ���
*����: ��
*����: ����
*����: ��
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
*������: HandelKeyInterrupt
*˵��: �жϴ���
*����: GPIO_Pin �жϽ�
*����: ��
*����: ��
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
*������: HAL_GPIO_EXTI_Callback
*˵��: �ⲿIO�ж�
*����: GPIO_Pin �жϽ�
*����: ��
*����: ��
*******************************************************************************/ 
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    HandelKeyInterrupt(GPIO_Pin);
    /* NOTE: This function Should not be modified, when the callback is needed,
    the HAL_GPIO_EXTI_Callback could be implemented in the user file
    */ 
}
