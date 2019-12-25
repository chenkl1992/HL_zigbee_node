#ifndef __SERVICE_DATA_H
#define __SERVICE_DATA_H

#include "event.h"

#define CMD_GROUP                                                        1
#define CMD_RSSI                                                         2
#define CMD_NET                                                          3

#define GROUP_READ                                                       0
#define GROUP_ADD                                                        1
#define GROUP_CLEAR                                                      2
#define GROUP_DELETE                                                     3
#define GROUP_UPDATE                                                     4
#define GROUP_NUM_MAX                                                    5

//设置协议里面的组号
__packed typedef struct
{
    uint8_t opcode;
    uint16_t num;
}group_t;

__packed typedef struct
{
    uint8_t groupCount;
    group_t group[GROUP_NUM_MAX];
}groupList_t;

extern groupList_t groupList;
extern uint8_t uart_busy_Flag;

void service_Data_init(void);
void service_Data(void);
uint8_t zigbee_quitnet_process(void);
uint8_t zigbee_set_group_process(uint8_t updateFlag);

int32_t wait_data_ack(uint8_t cmdFlag);
event_t *event_Data_id_get(void);
void zigbee_data_send(uint8_t* data, uint32_t dlen);

#endif