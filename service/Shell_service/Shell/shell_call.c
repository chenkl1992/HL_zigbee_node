#include "stm32l0xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "printf-stdarg.h"
#include "shell.h"
#include "log.h"
#include "shell_call.h"
#include "string.h"
#include "service_shell.h"
#include "service_mesh.h"
#include "service_Data.h"
#include "service_DA.h"
#include "flash.h"
#include "flash_table.h"
#include "packet.h"
#include "RN8209.h"
#include "main.h"

#ifdef AC_BOARD
static uint8_t version[] = "2.0.2t\r\n";
#else
static uint8_t version[] = "1.0.2t\r\n";
#endif

static uint8_t zigbee_reset[15] = {0xAB, 0xBC, 0xCD, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xAA};
static uint8_t zigbee_reboot[15] = {0xAB, 0xBC, 0xCD, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xAA};

void shell_printf(char *format, ...)
{
    va_list args;
    
    va_start(args, format);
    
    printk_va(0, format, args);
    
    va_end(args);
}

//开关log等级
eExecStatus shell_log_process( int ac,
                                signed char *av[],
                                signed char **ppcStringReply)
{
    if(0 == strcmp((char *)av[0], "on"))
    {
        log_level_set((uint8_t)~LOG_LEVEL_OFF);
    }
    else if(0 == strcmp((char *)av[0], "off"))
    {
        log_level_set(LOG_LEVEL_ERR);
    }
    else
    {
       shell_printf("log cmd error !\r\n");
       return SHELL_EXECSTATUS_KO;
    }
    return SHELL_EXECSTATUS_OK;
}

//设置
eExecStatus shell_set_process( int ac,
                                    signed char *av[],
                                    signed char **ppcStringReply)
{
    eExecStatus exec_status = SHELL_EXECSTATUS_OK;
    if(0 == strcmp((char *)av[0], "group"))
    {
      uint8_t setCounter = 0;
      uint16_t groupnum = 0;
      uint8_t ret = SET_ERROR;
      
      if(0 == strcmp((char *)av[1], "A"))
      {
        groupList.group[0].opcode = GROUP_ADD;
        setCounter++;
      }
      else if(0 == strcmp((char *)av[1], "C"))
      {
        groupList.group[0].opcode = GROUP_CLEAR;
        setCounter++;
      }
      else if(0 == strcmp((char *)av[1], "D"))
      {
        groupList.group[0].opcode = GROUP_DELETE;
        setCounter++;
      }
      if((char *)av[2] != NULL)
      {
        groupnum = atoi((char *)av[2]);
        if(groupnum != 0)
        {
           setCounter++;
        }
      }
      if(setCounter == 2)
      {
        groupList.groupCount = 1;
        groupList.group[0].num = groupnum;
        set_intac();
        ret = zigbee_set_group_process(0);
        clear_intac();
        if(ret == SET_SUCCESS)
        {
          shell_printf("set group success !\r\n");
        }
        else
        {
          shell_printf("set group fail !\r\n");
        }
      }
      else
      {
        shell_printf("set group fail !\r\n");
      }
    }
    //电流报警与恢复值
    else if(0 == strcmp((char *)av[0], "alarm"))
    {
        uint16_t alarm_value = 0;
        uint16_t recover_value = 0;
        uint8_t flag = 0;
        uint8_t setCounter = 0;
        if(0 == strcmp((char *)av[1], "I"))
        {
          if((char *)av[2] != NULL)
          {
            alarm_value = atoi((char *)av[2]);
            setCounter++;
          }
          if((char *)av[3] != NULL)
          {
            recover_value = atoi((char *)av[3]);
            if(alarm_value > recover_value)
            {
              setCounter++;
            }
          }
          if(0 == strcmp((char *)av[4], "enable"))
          {
            flag = 1;
            setCounter++;
          }
          else if(0 == strcmp((char *)av[4], "disable"))
          {
            flag = 0;
            setCounter++;
          }
          if(setCounter == 3)
          {
            zigbee_load.da_event.alarm_value = alarm_value;
            zigbee_load.da_event.recover_value = recover_value;
            zigbee_load.da_event.flag = flag;
            EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
            shell_printf("set alarm success !\r\n");
          }
          else
          {
            shell_printf("set alarm error !\r\n");
          }
        }
        else
        {
          shell_printf("set alarm error !\r\n");
        }
    }
   //其他参数
#ifdef AC_BOARD
   //校准电流
   else if(0 == strcmp((char *)av[0], "caliI"))
   {
     uint16_t value = 0;
     uint8_t index = 0;
     uint8_t setCounter = 0;
     if((char *)av[1] != NULL)
     {
       index = atoi((char *)av[1]);
       if(index < RN_POW_CALIVALS_N)
       {
         setCounter++;
       }
     }
     if((char *)av[2] != NULL)
     {
       value = atoi((char *)av[2]);
       if(value <= 5000)
       {
         setCounter++;
       }
     }
     if(setCounter == 2)
     {
       RN_CurCali(index, 0, value);
       memcpy(&(zigbee_load.RN_CaliVal), &RN_CaliVal, sizeof(RN_CaliVals_t));
       EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
       shell_printf("set caliI success !\r\n");
     }
     else
     {
       shell_printf("set caliI error !\r\n");
     }
   }
   //校准电压
   else if(0 == strcmp((char *)av[0], "caliV"))
   {
     uint16_t value = 0;
     uint8_t index = 0;
     uint8_t setCounter = 0;
     if((char *)av[1] != NULL)
     {
       index = atoi((char *)av[1]);
       if(index < RN_POW_CALIVALS_N)
       {
         setCounter++;
       }
     }
     if((char *)av[2] != NULL)
     {
       value = atoi((char *)av[2]);
       if(value <= 300)
       {
         setCounter++;
       }
     }
     if(setCounter == 2)
     {
       RN_VolCali(index, 0, value);
       memcpy(&(zigbee_load.RN_CaliVal), &RN_CaliVal, sizeof(RN_CaliVals_t));
       EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
       shell_printf("set caliV success !\r\n");
     }
     else
     {
       shell_printf("set caliV error !\r\n");
     }
   }
   else if(0 == strcmp((char *)av[0], "cali"))
   {
     if(0 == strcmp((char *)av[1], "clear"))
     {
       memset(&RN_CaliVal, 0, sizeof(RN_CaliVals_t));
       memcpy(&(zigbee_load.RN_CaliVal), &RN_CaliVal, sizeof(RN_CaliVals_t));
       EEPROM_WriteBytes(0, (uint8_t*)&zigbee_load, sizeof(zigbee_load_t));
       shell_printf("set cali clear success !\r\n");
     }
     else
     {
       shell_printf("set cali clear error !\r\n");
     }
   }
#endif
    else
    {
      shell_printf("set error !\r\n");
    }
    return exec_status;
}

//显示设备Mac地址
void show_devinfo(void)
{
  shell_printf("devinfo: %02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x\r\n", zigbee_mac[0], zigbee_mac[1], zigbee_mac[2], zigbee_mac[3], zigbee_mac[4], zigbee_mac[5], zigbee_mac[6], zigbee_mac[7]);
}

//显示设备是否入网
void show_devstat(void)
{
  uint8_t device_state;
  device_state = get_dev_state();
  if(device_state == NOT_IN_NET)
  {
    shell_printf("devstat: not in_net\r\n");
  }
  else if(device_state == IN_NET)
  {
    shell_printf("devstat: in_net\r\n");
  }
}

//显示分组信息
void show_group(void)
{
  shell_printf("group: %d\r\n", zigbee_load.zigbee_group.list_len);
  for(uint8_t i=0; i<5; i++)
  {
    if(zigbee_load.zigbee_group.group_inf[i].enflag == 1)
    {
      shell_printf("num: %d\r\n", zigbee_load.zigbee_group.group_inf[i].num);
    }
  }
}

//显示报警信息
void show_alarm(void)
{
  shell_printf("alarm:I aValue:%d rValue:%d enable:%d\r\n", zigbee_load.da_event.alarm_value, zigbee_load.da_event.recover_value, zigbee_load.da_event.flag);
}

//显示心跳信息
void show_hb(void)
{
  hb_info_t hb_data = {0};
  get_hb_info(&hb_data);
  shell_printf("hb:"); 
  if(hb_data.onoff == 1)
  {
    shell_printf("led on "); 
  }
  else
  {
    shell_printf("led off "); 
  }
  shell_printf("pwm:%d U:%d I:%d hour:%d dbm:%d alarm:%d \r\n", hb_data.pwm, hb_data.u, hb_data.i, hb_data.hour, hb_data.rssi, hb_data.alarm);
  shell_printf("freq:%d  pan id:%02x%02x\r\n", zigbee_cfgData.Chan, zigbee_cfgData.PAN_ID[0], zigbee_cfgData.PAN_ID[1]);
  shell_printf("net id:%02x%02x\r\n", zigbee_cfgData.MyAddr[0], zigbee_cfgData.MyAddr[1]);
  shell_printf("Dev_version:%s", version);
}

#ifdef AC_BOARD
//显示校准信息
void show_cali(void)
{
  uint8_t i_caliNum = 0;
  uint8_t v_caliNum = 0;
  uint8_t i = 0;
  
  i_caliNum = RN_CaliVal.CaliValsN[1];
  v_caliNum = RN_CaliVal.CaliValsN[0];
  
  shell_printf("i_caliNum: %d\r\n", i_caliNum);
  shell_printf("v_caliNum: %d\r\n", v_caliNum);
  
  for(i=0; i<v_caliNum; i++)
  {
    shell_printf("Cali V -- Act:%d Reg:%d\r\n", RN_CaliVal.RN_CaliVals[i].ActVal, RN_CaliVal.RN_CaliVals[i].RegVal);
  }
  for(i=0; i<i_caliNum; i++)
  {
    shell_printf("Cali I -- Act:%d Reg:%d\r\n", RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N+i].ActVal, RN_CaliVal.RN_CaliVals[RN_VOL_CALIVALS_N+i].RegVal);
  }
}
#endif


//显示
eExecStatus shell_show_process( int ac,
                                signed char *av[],
                                signed char **ppcStringReply)
{
  if(0 == strcmp((char *)av[0], "all"))
  {
    show_devinfo();
    show_devstat();
    show_group();
    show_alarm();
    show_hb();
#ifdef AC_BOARD
    show_cali();
#endif    
    
  }
  else if(0 == strcmp((char *)av[0], "devinfo"))
  {
    show_devinfo();
  }
  else if(0 == strcmp((char *)av[0], "devstat"))
  {
    show_devstat();
  }
  else if(0 == strcmp((char *)av[0], "group"))
  {
    show_group();
  }
  else if(0 == strcmp((char *)av[0], "alarm"))
  {
    show_alarm();
  }
  else if(0 == strcmp((char *)av[0], "hb"))
  {
    show_hb();
  }
#ifdef AC_BOARD
  else if(0 == strcmp((char *)av[0], "cali"))
  {
    show_cali();
  }
#endif
  else
  {
    shell_printf("show error !\r\n");
  }
  return SHELL_EXECSTATUS_OK;
}

//控制
eExecStatus shell_ctrl_process( int ac, 
                          signed char *av[],
                          signed char **ppcStringReply)
{
  if(0 == strcmp((char *)av[0], "on"))
  {
    set_led_on();
    shell_printf("ctrl on success \r\n");
  }
  else if(0 == strcmp((char *)av[0], "off"))
  {
    set_led_off();
    shell_printf("ctrl off success \r\n");
  }
  else if(0 == strcmp((char *)av[0], "pwm"))
  {
    if((char *)av[1] != NULL)
    {
      uint8_t bright;
      bright = atoi((char *)av[1]);
      if(bright <= 100)
      {
        change_bright(bright);
        shell_printf("ctrl pwm success\r\n");
      }
      else
      {
        shell_printf("ctrl pwm err\r\n");
      }
    }
  }
  else if(0 == strcmp((char *)av[0], "reset"))
  {
    shell_printf("ctrl reset success\r\n");
    HAL_Delay(500);
    HAL_NVIC_SystemReset();
  }
  else if(0 == strcmp((char *)av[0], "reboot"))
  {
    set_dev_state_not_innet();
    memcpy(zigbee_reboot+4, zigbee_mac, MAC_ADDR_LEN);
    memcpy(zigbee_reset+4, zigbee_mac, MAC_ADDR_LEN);
    zigbee_data_send(zigbee_reboot, 15);
    HAL_Delay(500);
    zigbee_data_send(zigbee_reset, 15);
    HAL_Delay(2000);
    shell_printf("ctrl reboot success\r\n");
  }
  else
  {
    shell_printf("ctrl error !\r\n");
  }
  return SHELL_EXECSTATUS_OK;
}
