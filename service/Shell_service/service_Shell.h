#ifndef __SERVICE_SHELL_H
#define __SERVICE_SHELL_H

#include "event.h"

void service_Shell_init(void);
void service_Shell(void);

event_t *event_Shell_id_get(void);

#endif