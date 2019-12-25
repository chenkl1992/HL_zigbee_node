#ifndef __FLASH_TABLE_H
#define __FLASH_TABLE_H

#include "RN8209.h"

#define ZIGBEE_GROUP_NUM                5

__packed typedef struct
{
    uint8_t   enflag;
    uint16_t  num;
}group_inf_t;

__packed typedef struct
{
    uint8_t   list_len;
    group_inf_t  group_inf[ZIGBEE_GROUP_NUM];
}zigbee_group_t;

__packed typedef struct
{
    uint8_t   key;
    uint16_t  alarm_value;
    uint16_t  recover_value;
    uint8_t   flag;
}da_event_t;

__packed typedef struct
{
  da_event_t da_event;
  zigbee_group_t zigbee_group;
  RN_CaliVals_t   RN_CaliVal;
  uint8_t init_flag;
}zigbee_load_t;

#endif