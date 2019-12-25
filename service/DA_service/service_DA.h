#ifndef __SERVICE_DA_H
#define __SERVICE_DA_H

#include "event.h"
#include "stm32l0xx_hal.h"
#include "timer.h"

void service_DA_init(void);
void service_DA(void);

event_t *event_DA_id_get(void);
event_t *event_DA_ack_id_get(void);

__packed typedef struct
{
    uint8_t onoff;
    uint8_t pwm;
    uint16_t u;
    uint16_t i;
    uint16_t hour;
    int8_t rssi;
    uint8_t alarm;
}hb_info_t;

__packed typedef struct
{
    uint8_t start;
    uint8_t onoff;          //开关灯状态
    uint32_t unix_time;     //最近一次开灯的时间戳
    uint16_t working_hour;
}led_state_t;

extern uint8_t get_rssi[6];
extern timer_t timer_DA;

void get_hb_info(hb_info_t* hb_data);
void set_hb_info(hb_info_t* hb_data);

#endif