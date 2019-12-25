#ifndef __SERVICE_MESH_H
#define __SERVICE_MESH_H

#include "event.h"
#include "timer.h"

#define MAC_ADDR_LEN                                    8
//设备状态
#define NOT_IN_NET                                      0
#define IN_NET                                          1

__packed typedef struct
{
    uint8_t   DevName[16];
    uint8_t   DevPwd[16];
    uint8_t   DevType;
    uint8_t   Chan;
    uint8_t   PAN_ID[2];
    uint8_t   MyAddr[2];                //本地网络地址-由组网时分配
    uint8_t   MyIEEE[8];                //设备Mac地址
    uint8_t   DstAddr[2];
    uint8_t   DstIEEE[8];
    uint8_t   RecvMode;
    uint8_t   PowerLevel;
    uint8_t   DstGroup[2];
    uint8_t   Serial_Rate;
    uint8_t   Serial_DataB;
    uint8_t   Serial_StopB;
    uint8_t   Serial_ParityB;
    uint8_t   SendMode;
    uint8_t   Serial_Interval;
    uint8_t   Reserved[2];
}zigbee_cfgData_t;

__packed typedef struct
{
    //uint8_t   DevState;
    uint8_t   AgmType[2];
    uint8_t   FirmVer[2];
}zigbee_tail_t;


extern uint8_t zigbee_mac[MAC_ADDR_LEN];
extern zigbee_cfgData_t zigbee_cfgData;

void service_Mesh_init(void);
void service_Mesh(void);
uint8_t get_dev_state(void);
void set_dev_state_not_innet(void);

event_t *event_Mesh_id_get(void);
event_t *event_Mesh_ack_id_get(void);

#endif