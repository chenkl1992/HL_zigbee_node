/*******************************************************************************
* Filename : service_Mesh.c
* Version  : V1.0
********************************************************************************
* Note(s)  : 
*******************************************************************************/


/*******************************************************************************
*                                 INCLUDE FILES
*******************************************************************************/
#include "stm32l0xx_hal.h"
#include "service_Mesh.h"
#include "service_Data.h"
#include "string.h"
#include "event.h"
#include "timer.h"
#include "crc.h"
#include "log.h"
#include "packet.h"
#include "flash.h"
#include "main.h"
/*******************************************************************************
*                                 LOCAL DEFINES
*******************************************************************************/
#define MESH_EVENT_NUM                                                   4
#define MESH_ACK_EVENT_NUM                                               4

#define QUIT_NET_REFRESH_PERIOD                                          3600000 //60分钟
#define IN_NET_REFRESH_PERIOD                                            2000  //2s

#define CFG_CMD_HEAD_LEN                                                 3      //AB BC CD
#define CFG_CMD_ID_LEN                                                   1      //命令ID

#define ZIGBEE_CFG_LEN                                                   80

//设备类型为路由
#define DEV_TYPE_ROUTE                                                   1

enum {
  ZIGBEE_STATE_INIT= 0,
  ZIGBEE_STATE_SET_CFG,
  ZIGBEE_STATE_SET_MESH,
  ZIGBEE_STATE_RESET,
  ZIGBEE_STATE_CFG_OK,
  ZIGBEE_STATE_INNET_OK,
  ZIGBEE_STATE_INNET_JUDGE,
};


/*******************************************************************************
*                             LOCAL GLOBAL VARIABLES
*******************************************************************************/
static queue_t queue_Mesh;
static event_t event_Mesh;
static event_element_t element_Mesh[MESH_EVENT_NUM];

static queue_t queue_Mesh_ack;
static event_t event_Mesh_ack;
static event_element_t element_Mesh_ack[MESH_ACK_EVENT_NUM];

static uint8_t zigbee_state = ZIGBEE_STATE_INIT;
static uint8_t dev_state = NOT_IN_NET;
//static uint8_t innet_judge_flag = RESET;

static timer_t timer_mesh_quit_net;
static timer_t timer_mesh_innet;

uint8_t zigbee_mac[MAC_ADDR_LEN] = {0};

static uint8_t zigbee_cfg_cmd[ZIGBEE_CFG_LEN] = {0xAB, 0xBC, 0xCD};
zigbee_cfgData_t zigbee_cfgData = {0};
static zigbee_tail_t zigbee_tail = {0};
static uint8_t zigbee_n_tail[4] = {0x00, 0x04, 0x02, 0x00};
static uint8_t zigbee_cfg_ack[7] = {0xAB, 0xBC, 0xCD, 0xD6, 0xFF, 0xFF, 0x00};
static uint8_t zigbee_set_mesh_ack[7] = {0xAB, 0xBC, 0xCD, 0x27, 0x02, 0x01, 0x00};


/*******************************************************************************
*                            LOCAL FUNCTION PROTOTYPES
*******************************************************************************/
static void quit_net_timer_callback(void);
static void in_net_timer_callback(void);

/*******************************************************************************
*函数名: zigbee_netled_close
*说明: 入网指示灯灭
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_netled_close(void)
{
  HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_SET);
}

/*******************************************************************************
*函数名: zigbee_netled_open
*说明: 入网指示灯亮
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_netled_open(void)
{
  HAL_GPIO_WritePin(STATE_GPIO_Port, STATE_Pin, GPIO_PIN_RESET);
}

/*******************************************************************************
*函数名: zigbeeInit_msg_send
*说明: 发消息 开始zigbee模组初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbeeInit_msg_send(void)
{
  event_element_t element;
  element.data = NULL;
  element.size = 0;
  element.type = EVT_MESH_INIT;
  event_send(event_Mesh_id_get(), &element);
}

/*******************************************************************************
*函数名: zigbeeInnet_timeout_msg_send
*说明: 发消息 开始自动入网
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbeeInnet_msg_send(void)
{
  event_element_t element;
  element.data = NULL;
  element.size = 0;
  element.type = EVT_MESH_IN_NEI_TIMEOUT;
  event_send(event_Mesh_id_get(), &element);
}

/*******************************************************************************
*函数名: zigbee_cfg_init
*说明: zigbee模块初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_cfg_init(void)
{
  zigbee_cfgData.DevType = DEV_TYPE_ROUTE;
  zigbee_cfgData.Chan = 0x19;
  zigbee_cfgData.PAN_ID[0] = 0xFF;
  zigbee_cfgData.PAN_ID[1] = 0xFF;
  //zigbee_cfgData.MyAddr[0] = 0xFF;
  //zigbee_cfgData.MyAddr[1] = 0xFF;
  memcpy(zigbee_cfgData.MyIEEE, zigbee_mac, MAC_ADDR_LEN);
  zigbee_cfgData.DstAddr[0] = 0x00;
  zigbee_cfgData.DstAddr[1] = 0x00;
  memset(zigbee_cfgData.DstAddr, 0, 8);
  zigbee_cfgData.Serial_Rate = 0x07;
  zigbee_cfgData.Serial_DataB = 0x08;
  zigbee_cfgData.Serial_StopB = 0x01;
  zigbee_cfgData.Serial_ParityB = 0x00;
}


/*******************************************************************************
*函数名: cmd_send
*说明: 向data服务发送 消息
*参数: state zigbee当前配置状态
*返回: 无
*其他: 无
*******************************************************************************/
static void cmd_send(uint8_t state)
{
  uint8_t error = 0;
  event_element_t element;
  switch(state)
  {
  case ZIGBEE_STATE_INNET_JUDGE:
    zigbee_cfg_cmd[3] = 0xD1;
    zigbee_cfg_cmd[4] = 0xAA;
    element.size = CFG_CMD_HEAD_LEN + 2;
    break;
    
  case ZIGBEE_STATE_INIT:
    //查询设备信息
    zigbee_cfg_cmd[3] = 0xD1;
    zigbee_cfg_cmd[4] = 0xAA;
    element.size = CFG_CMD_HEAD_LEN + 2;
    break;
    
  case ZIGBEE_STATE_SET_CFG:
    //将设备配置为路由 ...等
    zigbee_cfg_init();
    zigbee_cfg_cmd[3] = 0xD6;
    zigbee_cfg_cmd[4] = zigbee_cfgData.MyAddr[0];
    zigbee_cfg_cmd[5] = zigbee_cfgData.MyAddr[1];
    memcpy(zigbee_cfg_cmd+6, &zigbee_cfgData, sizeof(zigbee_cfgData_t));
    zigbee_cfg_cmd[6+sizeof(zigbee_cfgData_t)] = 0xAA;
    element.size = CFG_CMD_HEAD_LEN + 4 + sizeof(zigbee_cfgData_t);
    break;
    
  case ZIGBEE_STATE_SET_MESH:
    HAL_Delay(200);
    //设置为快速自组网
    zigbee_cfg_cmd[3] = 0x27;
    zigbee_cfg_cmd[4] = 0x01;
    zigbee_cfg_cmd[5] = 0x02;
    zigbee_cfg_cmd[6] = 0x01;
    zigbee_cfg_cmd[7] = 0xAA;
    element.size = CFG_CMD_HEAD_LEN + 5;
    break;
    
  case ZIGBEE_STATE_RESET:
    //复位模块
    zigbee_cfg_cmd[3] = 0x59;
    memcpy(zigbee_cfg_cmd+4, zigbee_mac, MAC_ADDR_LEN);
    zigbee_cfg_cmd[4+MAC_ADDR_LEN] = 0x00;
    zigbee_cfg_cmd[5+MAC_ADDR_LEN] = 0x04;
    zigbee_cfg_cmd[6+MAC_ADDR_LEN] = 0xAA;
    element.size = CFG_CMD_HEAD_LEN + 4 + MAC_ADDR_LEN;
    break;
    
  case ZIGBEE_STATE_CFG_OK:
    //判断入网标志位
    zigbee_cfg_cmd[3] = 0xE8;
    zigbee_cfg_cmd[4] = 0xAA;
    element.size = CFG_CMD_HEAD_LEN + 2;
    break;
    
  default:
    error = 1;
    logerr("cmd err\r\n");
    break;
  }
  if(!error)
  {
    element.data = zigbee_cfg_cmd;
    zigbee_data_send(element.data, element.size);
    if(zigbee_state == ZIGBEE_STATE_RESET)
    {
      zigbee_state = ZIGBEE_STATE_CFG_OK;
    }
  }
}

/*******************************************************************************
*函数名: is_innet_parm_correct
*说明: 入网参数判断
*参数: 无
*返回: 0 未入网 1入网
*其他: 无
*******************************************************************************/
static uint8_t is_innet_parm_correct(void)
{
  uint8_t in_net_flag = SET;
  if(zigbee_cfgData.MyAddr[0]== 0xFF && zigbee_cfgData.MyAddr[1]== 0xFF)
  {
    in_net_flag = RESET;
  }
  else if(zigbee_cfgData.MyAddr[0]== 0 && zigbee_cfgData.MyAddr[1]== 0)
  {
    in_net_flag = RESET;
  }
  else if(zigbee_cfgData.Chan<11 || zigbee_cfgData.Chan>26)
  {
    in_net_flag = RESET;
  }
  else if(zigbee_cfgData.PAN_ID[0]== 0xFF && zigbee_cfgData.PAN_ID[1]== 0xFF)
  {
    in_net_flag = RESET;
  }
  else if(zigbee_cfgData.PAN_ID[0]== 0 && zigbee_cfgData.PAN_ID[1]== 0)
  {
    in_net_flag = RESET;
  }
  if(in_net_flag == SET)
  {
    loginf("Net id:%02x%02x\r\n", zigbee_cfgData.MyAddr[0], zigbee_cfgData.MyAddr[1]);
    loginf("Pan id:%02x%02x\r\n", zigbee_cfgData.PAN_ID[0], zigbee_cfgData.PAN_ID[1]);
    loginf("Channel id:%d\r\n", zigbee_cfgData.Chan);
  }
  return in_net_flag;
}

/*******************************************************************************
*函数名: zigbee_cfg_data_process
*说明: zigbee配置指令处理
*参数: state: zigbee状态 * data 数据域  size 数据长度
*返回: 0 正确 -1错误
*其他: 无
*******************************************************************************/
static int32_t zigbee_cfg_data_process(uint8_t state, uint8_t* data, uint32_t size)
{  
  int32_t ret = -1;
  uint8_t datalen = 0;
  
  switch(state)
  {
  case ZIGBEE_STATE_CFG_OK:
    //入网判断的应答 AB BC CD E8 xx 00
    if(size != 6)
    {
      logerr("zigbee innet judge ack len err\r\n");
      return ret;
    }
    else
    {
      if(data[4] != 1)
      {
        logerr("not in net\r\n");
        return ret;
      }
      else
      {
        loginf("In net flag ok!\r\n");
        zigbee_state = ZIGBEE_STATE_INNET_JUDGE;
      }
    }
    break;
  case ZIGBEE_STATE_INIT:
    //读取初始化信息
    datalen = sizeof(zigbee_cfgData_t) + sizeof(zigbee_tail_t)+1 + CFG_CMD_HEAD_LEN + CFG_CMD_ID_LEN;
    if(size != datalen)
    {
      logerr("zigbee state len err\r\n");
      return ret;
    }
    else
    {
      //获取配置信息
      memcpy(&zigbee_cfgData, data+4, sizeof(zigbee_cfgData_t));
      //获取Mac地址
      memcpy(zigbee_mac, zigbee_cfgData.MyIEEE, MAC_ADDR_LEN);
      //获取包尾
      memcpy(&zigbee_tail, data+4+sizeof(zigbee_cfgData_t)+1, sizeof(zigbee_tail_t));
      if(memcmp(zigbee_n_tail, &zigbee_tail, sizeof(zigbee_tail_t)) != 0)
      {
        logerr("zigbee tail err\r\n");
        return ret;
      }
      //设备出厂时先退网一次
      if(zigbee_load.init_flag == RESET)
      {
        zigbee_state = ZIGBEE_STATE_SET_CFG;
        loginf("Device init\r\n");
      }
      else
      {
#warning add innet process        
        if(is_innet_parm_correct())
        {
          loginf("Already In net !\r\n");
          //zigbee_state = ZIGBEE_STATE_INNET_OK;
          zigbee_state = ZIGBEE_STATE_CFG_OK;
        }
        else
        {
          zigbee_state = ZIGBEE_STATE_SET_CFG;
        }
      }
    }
    break;
    
  case ZIGBEE_STATE_SET_CFG:
    //配置命令的 回复 AB BC CD D6 00 00 00
    if(size != 7)
    {
      logerr("zigbee cfg ack len err\r\n");
      return ret;
    }
    else
    {
      if(memcmp(zigbee_cfg_ack, data, 4) != 0)
      {
        if(data[6] != 0)
        {
          logerr("zigbee cfg ack err\r\n");
          return ret;
        }
      }
#warning 111
      if(zigbee_load.init_flag == RESET)
      {
        zigbee_load.init_flag = SET;
        EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
        loginf("Device init finish:%d\r\n", zigbee_load.init_flag);
      }
      zigbee_state = ZIGBEE_STATE_SET_MESH;
    }
    break;
    
  case ZIGBEE_STATE_SET_MESH:
    //配置快速自组网的应答 AB BC CD 27 02 01 00
    if(size != 7)
    {
      logerr("zigbee set mesh ack len err\r\n");
      return ret;
    }
    else
    {
      if(memcmp(zigbee_set_mesh_ack, data, 7) != 0)
      {
        logerr("zigbee set mesh ack err\r\n");
        return ret;
      }
      zigbee_state = ZIGBEE_STATE_RESET;
    }
    break;
    
  case ZIGBEE_STATE_INNET_JUDGE:
    //获取设备状态 与读取初始化信息时的回复 格式一致
    datalen = sizeof(zigbee_cfgData_t) + sizeof(zigbee_tail_t)+1 + CFG_CMD_HEAD_LEN + CFG_CMD_ID_LEN;
    if(size != datalen)
    {
      logerr("zigbee state2 len err\r\n");
      return ret;
    }
    else
    {
      //获取配置信息
      memcpy(&zigbee_cfgData, data+4, sizeof(zigbee_cfgData_t));
#warning add innet process      
      if(is_innet_parm_correct())
      {
        loginf("In net parm ok!\r\n");
        zigbee_state = ZIGBEE_STATE_INNET_OK;
        //zigbee_state = ZIGBEE_STATE_CFG_OK;
      }
    }
    break;
    
  default:
    logerr("rcv cfg err\r\n");
    return ret;
    break;
  }
  ret = 0;
  return ret;
}

/*******************************************************************************
*函数名: wait_zigbee_ack
*说明: 等待模组指令返回
*参数: state: zigbee状态
*返回: 0:收到正确的回复   -1:超时或数据异常
*其他: 无
*******************************************************************************/
static int32_t wait_zigbee_ack(uint8_t state)
{
  int32_t ret = -1;
  event_element_t element;
  ret = event_wait(event_Mesh_ack_id_get(), &element, 1000);
  if(ret == 0)
  {
    switch(element.type)
    {
    case EVT_MESH_CFG_RSP:
      ret = zigbee_cfg_data_process(state, element.data, element.size);
      break;
    }
  }
  return ret;
}

/*******************************************************************************
*函数名: zigbee_cfg
*说明: zigbee模组配置 等待回复
*参数: 
*返回: 
*其他: 无
*******************************************************************************/
static void zigbee_cfg(void)
{
  int32_t ret = 0xFF;
  for(int i=0; i<3; i++)
  {
    cmd_send(zigbee_state);
    if(zigbee_state == ZIGBEE_STATE_RESET)
    {
      ret = 0;
      break;
    }
    else
    {
      //其他配置命令需回复
      ret = wait_zigbee_ack(zigbee_state);
    }
    if(ret == 0)
    {
      break;
    }
    else
    {
      loginf("zigbee state:%d\r\n", zigbee_state);
      logerr("cfg timeout\r\n");
    }
  }
}

/*******************************************************************************
*函数名: is_zigbee_cfg_finish
*说明: zigbee设备是否已配置完成
*参数: 无
*返回: 1 完成 0未完成
*其他: 无
*******************************************************************************/
static uint8_t is_zigbee_cfg_finish(void)
{
  if(zigbee_state == ZIGBEE_STATE_CFG_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/*******************************************************************************
*函数名: is_zigbee_innet_ok
*说明: zigbee设备是否已入网完成
*参数: 无
*返回: 1 完成 0未完成
*其他: 无
*******************************************************************************/
static uint8_t is_zigbee_innet_ok(void)
{
  if(zigbee_state == ZIGBEE_STATE_INNET_OK)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/*******************************************************************************
*函数名: get_dev_state
*说明: 获取设备是否入网
*参数: 无
*返回: dev_state 1入网 0未入网
*其他: 无
*******************************************************************************/
uint8_t get_dev_state(void)
{
  return dev_state;
}

/*******************************************************************************
*函数名: set_dev_state_not_innet
*说明: 设置设备状态为未入网
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void set_dev_state_not_innet(void)
{
  dev_state = NOT_IN_NET;
}

/*******************************************************************************
*函数名: zigbee_innet
*说明: zigbee已入网
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_innet(void)
{
  //入网成功 
  loginf("zigbee innet success\r\n");
  dev_state = IN_NET;
  zigbee_netled_open();
  //开启超时退网定时器
  timer_start(&timer_mesh_quit_net);
}

/*******************************************************************************
*函数名: zigbee_Init
*说明: 开始zigbee模组初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_Init(void)
{
  while((!is_zigbee_cfg_finish()) && (!is_zigbee_innet_ok()))
  {
    zigbee_cfg();
  }
  if(is_zigbee_innet_ok())
  {
    zigbee_innet();
  }
  else if(is_zigbee_cfg_finish())
  {
    //配置完成 
    zigbeeInnet_msg_send();
  }
}

/*******************************************************************************
*函数名: zigbee_Innet_poll
*说明: 开始zigbee模组入网查询
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_Innet_poll(void)
{
  zigbee_cfg();
  if(!is_zigbee_innet_ok())
  {
    timer_start(&timer_mesh_innet);
    loginf("zigbee innet poll\r\n");
  }
  else
  {
    zigbee_innet();
  }
}

/*******************************************************************************
*函数名: zigbee_quitnet
*说明: 退网处理
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void zigbee_quitnet(void)
{
  uint8_t qiut_netFlag = SET_ERROR;
  
  set_intac();
  qiut_netFlag = zigbee_quitnet_process();
  clear_intac();
  if(qiut_netFlag == SET_SUCCESS)
  {
    //停止超时退网定时器
    timer_stop(&timer_mesh_quit_net);
    timer_stop(&timer_DA);
    //HAL_Delay(500);
    zigbee_netled_close();
    set_dev_state_not_innet();
    //cmd_send(ZIGBEE_STATE_RESET);
    //重新开始入网
    zigbee_state = ZIGBEE_STATE_RESET;
    //复位后重新入网
    timer_stop(&timer_mesh_innet);
    zigbeeInnet_msg_send();
  }
}

/*******************************************************************************
*函数名: service_Mesh_init
*说明: Mesh服务初始化
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_Mesh_init(void)
{
  event_create(event_Mesh_id_get(), 
               &queue_Mesh, 
               element_Mesh,
               MESH_EVENT_NUM);
  
  event_create(event_Mesh_ack_id_get(), 
               &queue_Mesh_ack, 
               element_Mesh_ack,
               MESH_ACK_EVENT_NUM);
  
  //超时退网
  timer_create(&timer_mesh_quit_net, 
               QUIT_NET_REFRESH_PERIOD, 
               TIMER_OPT_MODE_ONCE, 
               quit_net_timer_callback);
  //入网查询
  timer_create(&timer_mesh_innet,
               IN_NET_REFRESH_PERIOD, 
               TIMER_OPT_MODE_ONCE, 
               in_net_timer_callback);
  
  zigbeeInit_msg_send();
}

/*******************************************************************************
*函数名: service_Mesh
*说明: Mesh服务
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
void service_Mesh(void)
{
  int32_t ret;
  event_element_t element;
  
  ret = event_wait(&event_Mesh, &element, 0);
  if(ret == 0)
  {
    switch(element.type)
    {
    case EVT_MESH_INIT:
      //初始化配置
      memset(&zigbee_cfgData, 0, sizeof(zigbee_cfgData_t));
      HAL_Delay(500);
      zigbee_Init();
      break;
    case EVT_MESH_IN_NEI_TIMEOUT:
      //查询是否自动组网成功
      zigbee_Innet_poll();
      break;
    case EVT_MESH_NET_DATA:
      //有zigbee数据交互 -刷新退网定时器
      timer_start(&timer_mesh_quit_net);
      loginf("data\r\n");
      break;
    case EVT_MESH_QUIT_NET:
      zigbee_quitnet();
      break;
    }
  }
}

/*******************************************************************************
*函数名: quit_net_timer_callback
*说明: 退网定时回调
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void quit_net_timer_callback(void)
{
#warning disable auto quitnet 
  //    event_element_t element;
  //  
  //    element.data = NULL;
  //    element.size = 0;
  //    element.type = EVT_MESH_QUIT_NET;
  //    event_send(event_Mesh_id_get(), &element);
}

/*******************************************************************************
*函数名: in_net_timer_callback
*说明: 入网定时回调
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
static void in_net_timer_callback(void)
{
  event_element_t element;
  
  element.data = NULL;
  element.size = 0;
  element.type = EVT_MESH_IN_NEI_TIMEOUT;
  event_send(&event_Mesh, &element);
}

/*******************************************************************************
*函数名: event_Mesh_id_get
*说明: Mesh服务id
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
event_t *event_Mesh_id_get(void)
{
  return &event_Mesh;
}

/*******************************************************************************
*函数名: event_Mesh_ack_id_get
*说明: Mesh服务 消息回复 id
*参数: 无
*返回: 无
*其他: 无
*******************************************************************************/
event_t *event_Mesh_ack_id_get(void)
{
  return &event_Mesh_ack;
}
