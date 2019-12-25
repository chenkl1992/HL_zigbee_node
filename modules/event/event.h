#ifndef __EVENT_H
#define __EVENT_H

#include "queue.h"

#define EVENT_QUEUE_NUM                                         4

typedef enum {
    EVT_DA_TIMEOUT = 0,
    EVT_DA_I,
    EVT_DA_U,
    EVT_MESH_NET_DATA,
    //EVT_MESH_ZIGBEE_RSP,
    EVT_MESH_INIT,
    EVT_MESH_IN_NEI_TIMEOUT,
    EVT_MESH_CFG_RSP,
    EVT_MESH_QUIT_NET,
    EVT_DATA_SEND,
    EVT_DATA_GET,
    EVT_DATA_ITA_RSP,
    EVT_DATA_QUIT_NET,
    EVT_DATA_SET_GROUP,
    EVT_SHELL_DATA,
}event_type_t;

typedef struct {
    void *data;
    uint32_t size;
    uint32_t type;
}event_element_t;

typedef struct {
    queue_t *queue;
    event_element_t *elements;
}event_t;

void event_create(event_t *evt,
                  queue_t *p_q,
                  event_element_t *p_e,
                  uint32_t e_num);

int32_t event_send(event_t *evt, event_element_t *p_e);
int32_t event_wait(event_t *evt, event_element_t *p_e, uint32_t timeout);

#endif