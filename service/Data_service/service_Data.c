/*******************************************************************************
* Filename : service_Data.c
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
#include "event.h"
#include "usart.h"
#include "flash.h"
#include "flash_table.h"
#include "packet.h"
#include "CRC.h"
#include "string.h"
#include "log.h"
#include "main.h"
/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/
#define DATA_EVENT_NUM                                                   4
#define DATA_ACK_EVENT_NUM                                               4
#define ZIGBEE_DATA_LEN_MAX                                              300

/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/
static queue_t queue_Data;
static event_t event_Data;
static event_element_t element_Data[DATA_EVENT_NUM];

static queue_t queue_Data_ack;
static event_t event_Data_ack;
static event_element_t element_Data_ack[DATA_ACK_EVENT_NUM];

groupList_t groupList = {0};
static group_t group = {0};


uint8_t uart_busy_Flag = 0;
static uint8_t ack_buf[ZIGBEE_DATA_LEN_MAX] = {0};
static uint8_t zigbee_cmd[20] = {0xAB, 0xBC, 0xCD};
static uint8_t zigbee_set_group_ack[17] = {0xAB, 0xBC, 0xCD, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
static uint8_t zigbee_set_quitnet_ack[17] = {0xAB, 0xBC, 0xCD, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00};
static uint8_t zigbee_quit[18] = {0xAB, 0xBC, 0xCD, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0xAA};
/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
int32_t wait_data_ack(uint8_t cmdFlag);

/*******************************************************************************
*函数名: zigbee_uart_rx_en
*说明: zigbee串口 使能接收
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void zigbee_uart_rx_en(void)
{
  HAL_UART_Receive_DMA(&huart1, ack_buf, ZIGBEE_DATA_LEN_MAX);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}

/*******************************************************************************
*函数名: service_Data_init
*说明: Data服务初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_Data_init(void)
{
  event_create(&event_Data,
               &queue_Data, 
               element_Data, 
               DATA_EVENT_NUM);
  
  event_create(&event_Data_ack,
               &queue_Data_ack, 
               element_Data_ack, 
               DATA_ACK_EVENT_NUM);
  
  zigbee_uart_rx_en();
}

/*******************************************************************************
*函数名: zigbee_data_send
*说明: zigbee数据发送
*参数: * data 数据域  dlen 数据长度
*返回: 无
*其他: 无
*******************************************************************************/
void zigbee_data_send(uint8_t* data, uint32_t dlen)
{
  //if(uart_busy_Flag == 0)
  {
    //uart_busy_Flag = 1;
    __disable_irq();
    HAL_UART_Transmit_DMA(&huart1, data, dlen);
    __enable_irq();
    //uart_busy_Flag = 0;
  }
}

/*******************************************************************************
*函数名: quit_net
*说明: 退网
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void quit_net(void)
{
    HAL_Delay(2000);
    clear_rejoin();
    //后续退网处理
    event_element_t element;
    element.data = NULL;
    element.size = 0;
    element.type = EVT_MESH_QUIT_NET;
    event_send(event_Mesh_id_get(), &element);
}

/*******************************************************************************
*函数名: zigbee_data_process
*说明: zigbee数据解包
*参数: *data 数据域  dlen 数据长度
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_data_process(uint8_t* data, uint32_t dlen)
{
  //若是网关的指令，处理 并回复
  uint8_t ret = ERROR_CMD;
  packet_head_t packet_head;
  packet_body_t packet_body;
  hb_info_t hb_data = {0};
  event_element_t element;

  //判断包的类型
  ret = parse_packet(&packet_head, &packet_body, data, dlen);
  
  if(ret == ERROR_CMD)
  {
    logerr("packet err\r\n");
  }
  else if(ret == PACKET_CMD)
  {
    loginf("d:%s", data);
    ack_info_t ack_info;
    ack_info.flag  = data_handler(&packet_head, &packet_body);
    //需要立即回复 如点播等，则立即回复
    if(ack_info.flag == 0)
    {
      //mainId 最高位置位表示应答
      packet_head.mainId = packet_head.mainId | 0x80;
      //组成回复
      ack_info.len = pack_ack_packet(ack_buf, &packet_head, &packet_body);
      //协议数据发送
      element.data = ack_buf;
      element.size = ack_info.len;
      element.type = EVT_DATA_SEND;
      event_send(event_Data_id_get(), &element);
    }
    else
    {  
      if(need_rejoin())
      {
        quit_net();
      }
      if(need_sys_rst())
      {
        //复位
        HAL_Delay(200);
        HAL_NVIC_SystemReset();
      }
    }
    //和网关有数据交互 发消息 刷新退网定时器
    event_element_t element_mesh;
    element_mesh.data = NULL;
    element_mesh.size = 0;
    element_mesh.type = EVT_MESH_NET_DATA;
    event_send(event_Mesh_id_get(), &element_mesh);
  }
  else if(ret == TEMP_CMD)
  {
    if(dlen == 8)
    {
      //判断包头
      if(memcmp(get_rssi, data, 6) == 0)
      {
        get_hb_info(&hb_data);
        hb_data.rssi = data[6];
        set_hb_info(&hb_data);
      }
    }
  }
}

/*******************************************************************************
*函数名: zigbee_send_quitnet_cmd
*说明: zigbee发送退网命令
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_send_quitnet_cmd(void)
{
  memcpy(zigbee_quit+4, zigbee_mac, MAC_ADDR_LEN);
  zigbee_data_send(zigbee_quit, 18);
}

/*******************************************************************************
*函数名: zigbee_send_group_cmd
*说明: zigbee发送配置分组命令
*参数: tgroup 分组信息
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_send_group_cmd(group_t tgroup)
{
  uint32_t len = 0;
  zigbee_cmd[3] = 0x42;
  memcpy(zigbee_cmd+4, zigbee_mac, MAC_ADDR_LEN);
  zigbee_cmd[4+MAC_ADDR_LEN] = tgroup.opcode;
  zigbee_cmd[5+MAC_ADDR_LEN] = HI_UINT16(tgroup.num);
  zigbee_cmd[6+MAC_ADDR_LEN] = LO_UINT16(tgroup.num);
  zigbee_cmd[7+MAC_ADDR_LEN] = 0xAA;
  
  len = 3 + 5 + MAC_ADDR_LEN;
  zigbee_data_send(zigbee_cmd, len);
}

/*******************************************************************************
*函数名: zigbee_quitnet_process
*说明: zigbee退网流程
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
uint8_t zigbee_quitnet_process(void)
{
  //发送退网命令（配置PAN ID 为FFFF）
  int32_t ret = -1;
  uint8_t cmd_ret = SET_ERROR;
  for(uint8_t i=0; i<3; i++)
  {
    zigbee_send_quitnet_cmd();
    ret = wait_data_ack(CMD_NET);
    if(ret == 0)
    {
      cmd_ret = SET_SUCCESS;
      break;
    }
    else
    {
      logerr("qiut net cmd error\r\n");
    }
  }
  return cmd_ret;
}

/*******************************************************************************
*函数名: zigbee_set_group_process
*说明: zigbee分组流程
*参数: updateFlag 是否是更新命令
*返回: 设置成功或失败
*其他: 无
*******************************************************************************/
uint8_t zigbee_set_group_process(uint8_t updateFlag)
{
  //发送配置分组命令
  int32_t ret = -1;
  uint8_t cmd_ret = SET_ERROR;
  uint8_t group_ack_counter = 0;
  uint8_t update_finishFlag = 0;
  //如需更新先全清
  if(updateFlag == 1)
  {
    memset(&group, 0, sizeof(group_t));
    group.opcode = GROUP_CLEAR;
    for(uint8_t k=0; k<3; k++)
    {  
      zigbee_send_group_cmd(group);
      ret = wait_data_ack(CMD_GROUP);
      if(ret == 0)
      {
        update_finishFlag = 1;
        break;
      }
      else
      {
        logerr("group set update error\r\n");
      }
    }
  }
  if(updateFlag == 0 || update_finishFlag == 1)
  {
    for(uint8_t j=0; j<groupList.groupCount; j++)
    {
      for(uint8_t i=0; i<3; i++)
      {
        memcpy(&group, &(groupList.group[j]), sizeof(group_t));    
        zigbee_send_group_cmd(group);
        ret = wait_data_ack(CMD_GROUP);
        if(ret == 0)
        {
          group_ack_counter++;
          break;
        }
        else
        {
          logerr("group set cmd error\r\n");
        }
      }
    }
  }
  if(group_ack_counter == groupList.groupCount)
  {
    cmd_ret = SET_SUCCESS;
  }
  return cmd_ret;
}

/*******************************************************************************
*函数名: service_Data
*说明: Data服务
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_Data(void)
{
    int32_t ret;
    event_element_t element;

    ret = event_wait(&event_Data, &element, 0);
    if(ret == 0)
    {
        switch(element.type)
        {
        case EVT_DATA_SEND:
            //zigbee数据发送
            zigbee_data_send(element.data, element.size);
            if(need_sys_rst())
            {
              //复位
              HAL_Delay(200);
              HAL_NVIC_SystemReset();
            }
            if(need_rejoin())
            {
              quit_net();
            }
            break;
            
        case EVT_DATA_GET:
            //zigbee指令解包和处理
            zigbee_data_process(element.data, element.size);
            break;
        }
    }
}

/*******************************************************************************
*函数名: event_Data_id_get
*说明: Data服务id
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
event_t *event_Data_id_get(void)
{
    return &event_Data;
}

/*******************************************************************************
*函数名: event_Data_ack_id_get
*说明: Data ack 服务id
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static event_t *event_Data_ack_id_get(void)
{
    return &event_Data_ack;
}

/*******************************************************************************
*函数名: ita_data_process
*说明: zigbee 配置/查询 回复数据解包
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static int32_t ita_data_process(uint8_t cmdFlag, uint8_t* data, uint32_t dlen)
{
  uint8_t cmdret = ERROR_CMD;
  int32_t ret = -1;
//  hb_info_t hb_data = {0};
  packet_head_t packet_head;
  packet_body_t packet_body;
  //判断包的类型
  cmdret = parse_packet(&packet_head, &packet_body, data, dlen);
  if(cmdret == ERROR_CMD)
  {
    return ret;
  }
  else if(cmdret == MODULE_CMD)
  {
    if(cmdFlag == CMD_NET)
    {
      //配置退网的应答 AB BC CD 41 14 B4 57 FF FE 53 09 B1 FF FF 00 00 00
      if(dlen != 17)
      {
        return ret;
      }
      else
      {
        memcpy(zigbee_set_quitnet_ack+4, zigbee_mac, MAC_ADDR_LEN);
        if(memcmp(zigbee_set_quitnet_ack, data, 17) != 0)
        {
          return ret;
        }
        ret = 0;
      }
    }
    else if(cmdFlag == CMD_GROUP)
    {
      uint8_t opcode = 0;
      uint8_t group_existFlag = 0;
      int8_t group_num = -1;
      
      //配置分组的回复 AB BC CD 42 14 B4 57 FF FE 53 09 B1 XX 01 XX XX 00
      //清空 分组的回复 AB BC CD 42 14 B4 57 FF FE 53 09 B1 02 00 00    
      if(dlen != 17 && dlen != 15)
      {
        
        return ret;
      }
      else
      {
        if(group.opcode > GROUP_DELETE)
        {
          return ret;
        }
        memcpy(zigbee_set_group_ack+4, zigbee_mac, MAC_ADDR_LEN);
        if(memcmp(zigbee_set_group_ack, data, 12) != 0)
        {
          return ret;
        }
        //清除全部分组
        if(group.opcode == GROUP_CLEAR)
        {
          ret = 0;
          opcode = GROUP_CLEAR;
        }
        //增和删
        else
        {
          opcode = group.opcode;
          //配置分组错误
          //发送 AB BC CD 42 14 B4 57 FF FE 35 06 BA 01 00 01 AA
          //回复 AB BC CD 42 14 B4 57 FF FE 35 06 BA 01 01 00 06 07
          if(data[dlen-1] != 0x00)
          {
            return ret;
          }         
          ret = 0;
        }
        //保存分组信息
          if(opcode == GROUP_ADD)
          {
            if(zigbee_load.zigbee_group.list_len < GROUP_NUM_MAX)
            {
              for(uint8_t i=0; i<GROUP_NUM_MAX; i++)
              {
                //判断是否已在表内
                if(zigbee_load.zigbee_group.group_inf[i].enflag == 1)
                {
                  if(zigbee_load.zigbee_group.group_inf[i].num == group.num)
                  {
                    group_existFlag = 1;
                  }
                }
                //表中有空位
                else
                {
                  group_num = i;
                }
              }
              if(group_existFlag == 0 && group_num >= 0 )
              {
                zigbee_load.zigbee_group.group_inf[group_num].enflag = 1;
                zigbee_load.zigbee_group.group_inf[group_num].num = group.num;
                zigbee_load.zigbee_group.list_len++;
              }
            }
          }
          else if(opcode == GROUP_CLEAR)
          {
            //全部清除
            zigbee_load.zigbee_group.list_len = 0;
            for(uint8_t i=0; i<GROUP_NUM_MAX; i++)
            {
              zigbee_load.zigbee_group.group_inf[i].enflag = 0;
            }
          }
          else if(opcode == GROUP_DELETE)
          {
            //删除组号
            for(uint8_t i=0; i<GROUP_NUM_MAX; i++)
            {
              if(zigbee_load.zigbee_group.group_inf[i].enflag == 1)
              {
                if(zigbee_load.zigbee_group.group_inf[i].num == group.num)
                {
                  zigbee_load.zigbee_group.group_inf[i].enflag = 0;
                  zigbee_load.zigbee_group.list_len--;
                  break;
                }
              }
            }
          }
          EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
      }
    }
  }
//  else if(cmdret == TEMP_CMD)
//  {
//    if(dlen == 8)
//    {
//      //判断包头
//      if(memcmp(get_rssi, data, 6) == 0)
//      {
//        get_hb_info(&hb_data);
//        hb_data.rssi = data[6];
//        set_hb_info(&hb_data);
//        ret = 0;
//      }
//    }
//  }
  return ret; 
}

/*******************************************************************************
*函数名: wait_data_ack
*说明: 等待模组指令返回
*参数: cmdFlag 哪种类型的指令
*返回: 0:收到正确的回复   -1:超时或数据异常
*其他: 无
*******************************************************************************/
int32_t wait_data_ack(uint8_t cmdFlag)
{
  int32_t ret = -1;
  event_element_t element; 
  ret = event_wait(event_Data_ack_id_get(), &element, 800);
  if(ret == 0)
  {
    switch(element.type)
    {
    case EVT_DATA_ITA_RSP:
      ret = ita_data_process(cmdFlag, element.data, element.size);
      break;
    }
  }
  return ret;
}



/*******************************************************************************
*函数名: net_data_process
*说明: 数据处理
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void net_data_process(event_element_t element)
{
  if(is_interact_cmd())
  {
    element.type = EVT_DATA_ITA_RSP;
    event_send(event_Data_ack_id_get(), &element);
  }
  else
  {
    //uint8_t packet_head = 0x68;
    //if(memcmp(&packet_head, element.data, 1) == 0)
    //{
      element.type = EVT_DATA_GET;
      event_send(event_Data_id_get(), &element);
    //}
    //else if(memcmp(get_rssi, element.data, 6) == 0)
//    {
//      element.type = EVT_DATA_ITA_RSP;
//      event_send(event_Data_ack_id_get(), &element);
//    }
  }
}

/*******************************************************************************
*函数名: USART1_IRQHandler
*说明: 串口 空闲中断处理，区分透传指令，配置指令，状态查询指令
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void USART1_IRQHandler(void)
{
  uint16_t temp;
  uint16_t data_len = 0;
  event_element_t element;
  
  __disable_irq();
  if((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET))  
  {   
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);  
    HAL_UART_DMAStop(&huart1);
    temp = huart1.hdmarx->Instance->CNDTR;
    data_len = ZIGBEE_DATA_LEN_MAX - temp;
    if(data_len > ZIGBEE_DATA_LEN_MAX)
    {
      data_len = 0;
    }
  }
  if(data_len != 0)
  {
    element.data = ack_buf;
    element.size = data_len;
    if(get_dev_state() == IN_NET)
    {
      net_data_process(element);
    }
    else
    {
      //未组网完成前的命令
      element.type = EVT_MESH_CFG_RSP;
      event_send(event_Mesh_ack_id_get(), &element);
    }
  }
  HAL_UART_IRQHandler(&huart1);
  HAL_UART_Receive_DMA(&huart1, ack_buf, ZIGBEE_DATA_LEN_MAX);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  __enable_irq();
}


///*******************************************************************************
//*函数名: HAL_UART_TxCpltCallback
//*说明: 串口 发送完成中断
//*参数: huart 串口结构体
//*返回: 无
//*其他: 无
//*******************************************************************************/
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//  if(huart->Instance == huart1.Instance)
//  {
//    uart_busy_Flag = 0;
//  }
//}
