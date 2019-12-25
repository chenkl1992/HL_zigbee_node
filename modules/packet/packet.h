#ifndef __PACKET_H
#define __PACKET_H

#include "stm32l0xx_hal.h"
#include "service_DA.h"

#define MAX_PARAM_LENGTH  255

//0 异常指令 1协议指令 2模组指令 3临时指令
#define ERROR_CMD               0
#define PACKET_CMD              1
#define MODULE_CMD              2
#define TEMP_CMD                3

#define SET_SUCCESS             0
#define SET_ERROR               1

//子id
#define SUBID_ONOFFLED          1
#define SUBID_PWM               2
#define SUBID_NODEINFO          3
#define SUBID_ALARM_VALUE       4
#define SUBID_SET_GROUP         5
#define SUBID_SYS_RESET         6
#define SUBID_REJOIN            7
#define SUBID_DEV_STATE         8

__packed typedef struct
{
    uint8_t dlen;
    uint8_t subId;
    uint8_t data[MAX_PARAM_LENGTH];
}packet_body_t;

__packed typedef struct
{
    uint8_t head_flg;
    uint8_t dev_addr[8];
    uint8_t seq;
    uint8_t mainId;
}packet_head_t;

__packed typedef struct
{
    uint8_t flag;
    uint32_t len;
}ack_info_t;

#ifdef AC_BOARD
void outside_led_close(void);
#endif
extern led_state_t led_state;
extern uint32_t add_time;

uint8_t parse_packet(packet_head_t* packet_head, packet_body_t* packet_body, uint8_t* data, uint32_t datalen);
uint32_t pack_ack_packet(uint8_t* pbuf, packet_head_t* packet_head, packet_body_t* packet_body);
uint8_t data_handler(packet_head_t* packet_head, packet_body_t* packet_body);
uint8_t need_sys_rst(void);
uint8_t need_rejoin(void);
void clear_rejoin(void);
uint8_t is_groupset(void);
uint8_t is_interact_cmd(void);
void set_intac(void);
void clear_groupset(void);
void clear_intac(void);
void change_bright(uint8_t pulse);
void set_led_off(void);
void set_led_on(void);

#endif
